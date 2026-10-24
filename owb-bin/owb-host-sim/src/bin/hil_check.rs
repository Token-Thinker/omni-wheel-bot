//! Hardware-in-the-loop assertion harness.
//!
//! Sends newline-delimited JSON over USB serial to a real board and asserts
//! invariants on the replies. This is the bring-up regression suite: it answers
//! "is this board still behaving?" in about a minute, without hand-writing
//! throwaway serial scripts.
//!
//! Distinct from [`crate::hil_bridge`] (in the host-sim binary), which mirrors
//! operator commands one-way to hardware. This tool is bidirectional: it reads
//! replies back and checks them.
//!
//! # Usage
//!
//! ```text
//! cargo run -p owb-host-sim --bin hil-check                  # static checks only
//! cargo run -p owb-host-sim --bin hil-check -- --motion      # + drive the wheels
//! cargo run -p owb-host-sim --bin hil-check -- --port /dev/ttyACM1 --motion
//! cargo run -p owb-host-sim --bin hil-check -- --motion --characterize [--csv sweep.csv]
//! cargo run -p owb-host-sim --bin hil-check -- --motion --soak [--soak-duty 0.8] [--csv soak.csv]
//! ```
//!
//! **Motion scenarios are opt-in.** They spin the wheels, so they require
//! `--motion` and a bot that is elevated with wheels free. Without the flag the
//! harness runs boot and sensor checks only, which are safe on a desk.
//!
//! # Measurement modes
//!
//! `--characterize` and `--soak` replace the scenario suite for that run (they
//! also need `--motion`, and are skipped without it). They only ever drive W1
//! and W2; W3 is dead on the v1-alpha bench unit and is never commanded.
//!
//! - `--characterize` (~80 s): open-loop `probe_speed` duty sweep per wheel,
//!   0.10..=1.00 in 0.05 steps plus -0.3/-0.6/-1.0 for symmetry, wheels
//!   interleaved so heating builds evenly. Prints the speed table, a loud
//!   warning for any non-monotonic duty (speed >10% below a lower duty's),
//!   breakaway duty, each wheel's monotonic region and a linear fit over it,
//!   the W1/W2 ratio at duty 0.6, a recommended duty ceiling, and a
//!   paste-ready snippet for `max_wheel_rad_s`, `default_wheel_trim` and
//!   `min_forward_speed_norm`. Nothing is derived from points above the
//!   monotonic region: on the v1-alpha unit duty 1.0 collapses to ~60%.
//! - `--soak` (~85 s): diagnoses the two-wheel speed droop. Five 10 s phases at
//!   `--soak-duty` (default 0.8) with 5 s rests: W1 alone, W2 alone, both, both
//!   with `pwm_stagger`, both again as a control. Prints the mean speed over an
//!   early window (after a 1.5 s spin-up skip) and an equal-length late window,
//!   percent decay, a per-second series and an interpretation hint that only
//!   makes claims above the 5% noise threshold. The stagger phase is skipped on
//!   firmware without `drive_loop`.
//! - `--csv <path>` writes the raw data. When both modes run, the mode name is
//!   inserted before the extension (`sweep.characterize.csv`, `sweep.soak.csv`).
//!
//! Both modes hold motion with a keepalive inside the firmware's 300 ms motion
//! deadman. Ctrl-C stops the wheels, disables the motors and resets
//! `pwm_stagger`; press it twice to force-quit.
//!
//! Exit status is 0 only if every scenario that ran passed, so this is usable
//! as a CI gate on a bench runner.

use std::{
    io::{Read, Write},
    time::{Duration, Instant},
};

/// Reply wait budget.
///
/// Must clear the firmware's *own* 1 s reply timeout plus contention on
/// `COMMAND_QUERY_LOCK`, which every read command takes. `read_power` is the
/// slow one -- it can hold that lock for close to a second -- so a 2 s budget
/// sat right on the boundary and flaked. 4 s is comfortably clear while still
/// failing fast on a genuinely unresponsive board.
const REPLY_TIMEOUT: Duration = Duration::from_millis(1_500);

/// The control loop zeroes the wheels if no motion command arrives, so motion
/// scenarios must re-assert the command faster than this.
const MOTION_REFRESH: Duration = Duration::from_millis(400);

/// How many times to re-ask when a command is swallowed.
const QUERY_ATTEMPTS: u32 = 4;

/// Quiet period enforced before every command write. See [`Bot::write_line`]
/// for why acknowledgement-based pacing alone loses commands. 400 ms clears the
/// firmware's worst-case `COMMAND_QUERY_LOCK` hold (read_power approaches a
/// second); 150 ms was measurably flaky.
const COMMAND_GAP: Duration = Duration::from_millis(250);

type Check = Result<(), String>;

/// Echo every line received from the board. Set by `--verbose`; invaluable when
/// a scenario fails and you need to see what the firmware actually said.
static VERBOSE: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

fn verbose() -> bool { VERBOSE.load(std::sync::atomic::Ordering::Relaxed) }

fn now_s() -> f64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs_f64() % 1000.0)
        .unwrap_or(0.0)
}

/// Set by the Ctrl-C handler; polled by the measurement modes' drive and rest
/// loops so they unwind through the normal stop/disable path.
static INTERRUPTED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

fn interrupted() -> bool { INTERRUPTED.load(std::sync::atomic::Ordering::Relaxed) }

/// Route Ctrl-C to [`INTERRUPTED`] instead of the default kill.
///
/// The default SIGINT terminates without running `Drop`, so `Bot::drop`'s
/// `make_safe` never runs. The firmware deadman would still zero the wheels
/// within 300 ms, but it does not disable the motors or put back a
/// `pwm_stagger` the soak switched on. A second Ctrl-C force-quits in case the
/// safe path itself wedges on a dead link.
///
/// Only installed for the measurement modes: the scenario suite has blocking
/// waits (the 12 s boot capture) that never poll the flag, so catching SIGINT
/// there would make Ctrl-C look dead. tokio is already a dependency of this
/// package; a one-thread runtime on a side thread is the cheapest portable
/// signal hook available without adding a crate.
fn install_ctrl_c() {
    std::thread::spawn(|| {
        let Ok(rt) = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
        else {
            return;
        };
        rt.block_on(async {
            while tokio::signal::ctrl_c().await.is_ok() {
                if INTERRUPTED.swap(true, std::sync::atomic::Ordering::Relaxed) {
                    eprintln!("\nhil-check: forced quit; motors may still be enabled");
                    std::process::exit(130);
                }
                eprintln!("\nhil-check: interrupted; stopping wheels (Ctrl-C again to force quit)");
            }
        });
    });
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let allow_motion = args.iter().any(|a| a == "--motion");
    if args.iter().any(|a| a == "--verbose" || a == "-v") {
        VERBOSE.store(true, std::sync::atomic::Ordering::Relaxed);
    }
    let port_path = arg_value(&args, "--port")
        .or_else(|| std::env::var("OWB_HIL_SERIAL").ok())
        .unwrap_or_else(|| "/dev/ttyACM0".to_string());
    let characterize = args.iter().any(|a| a == "--characterize");
    let soak = args.iter().any(|a| a == "--soak");
    let soak_duty = match arg_value(&args, "--soak-duty") {
        None => 0.8,
        Some(raw) => match raw.parse::<f64>() {
            Ok(v) if (0.1..=1.0).contains(&v) => v,
            _ => {
                eprintln!("FATAL: --soak-duty must be a number in 0.1..=1.0, got {raw:?}");
                std::process::exit(2);
            }
        },
    };
    let csv = arg_value(&args, "--csv");

    println!("hil-check: port={port_path} motion={allow_motion}");
    if !allow_motion {
        println!("  (motion scenarios skipped; pass --motion with wheels free to enable)");
    }

    let mut bot = match Bot::open(&port_path) {
        Ok(bot) => bot,
        Err(err) => {
            eprintln!("FATAL: cannot open {port_path}: {err}");
            std::process::exit(2);
        }
    };

    let mut tally = Tally::default();

    if characterize || soak {
        // The measurement modes are long-running and hold the wheels driving,
        // so this is the one place Ctrl-C has to stop them cleanly.
        if allow_motion {
            install_ctrl_c();
        }
        let both = characterize && soak;
        if both {
            println!("  (running both modes back to back; separate runs give a cooler soak)");
        }
        let csv_for = |mode: &str| csv.as_deref().map(|base| mode_csv_path(base, mode, both));
        if characterize {
            let path = csv_for("characterize");
            run_scenario(
                &mut bot,
                &mut tally,
                allow_motion,
                "characterize",
                true,
                |b| mode_characterize(b, path.as_deref()),
            );
        }
        if soak {
            let path = csv_for("soak");
            run_scenario(&mut bot, &mut tally, allow_motion, "soak", true, |b| {
                mode_soak(b, soak_duty, path.as_deref())
            });
        }
    }
    else {
        // (name, needs_motion, body)
        #[allow(clippy::type_complexity)]
        let scenarios: Vec<(&str, bool, fn(&mut Bot) -> Check)> = vec![
            ("boot_is_clean", false, scenario_boot),
            ("sensors_all_respond", false, scenario_sensors),
            ("encoders_idle_at_rest", false, scenario_encoders_idle),
            ("forward_reverse_signs", true, scenario_forward_reverse),
            ("encoder_deltas_sum_to_totals", true, scenario_delta_sum),
            ("estop_latches", true, scenario_estop),
        ];
        for (name, needs_motion, body) in scenarios {
            run_scenario(&mut bot, &mut tally, allow_motion, name, needs_motion, body);
        }
    }

    let Tally {
        passed,
        failed,
        skipped,
    } = tally;
    println!("\n{passed} passed, {failed} failed, {skipped} skipped");
    if failed > 0 {
        std::process::exit(1);
    }
}

#[derive(Default)]
struct Tally {
    passed: usize,
    failed: usize,
    skipped: usize,
}

fn run_scenario(
    bot: &mut Bot,
    tally: &mut Tally,
    allow_motion: bool,
    name: &str,
    needs_motion: bool,
    body: impl FnOnce(&mut Bot) -> Check,
) {
    if needs_motion && !allow_motion {
        println!("  SKIP {name} (needs --motion)");
        tally.skipped += 1;
        return;
    }
    match body(bot) {
        Ok(()) => {
            println!("  PASS {name}");
            tally.passed += 1;
        }
        Err(err) => {
            println!("  FAIL {name}: {err}");
            tally.failed += 1;
        }
    }
    // Never leave the wheels driving between scenarios, even after a
    // failure that returned early.
    bot.make_safe();
    if interrupted() {
        println!("\ninterrupted: wheels stopped, motors disabled");
        std::process::exit(130);
    }
}

fn arg_value(
    args: &[String],
    key: &str,
) -> Option<String> {
    let idx = args.iter().position(|a| a == key)?;
    args.get(idx + 1).cloned()
}

// ---------------------------------------------------------------------------
// scenarios
// ---------------------------------------------------------------------------

/// Reset the board and require a clean boot: no panic, no watchdog reset, no
/// pin conflicts, and the drive geometry the profile is supposed to report.
fn scenario_boot(bot: &mut Bot) -> Check {
    let log = bot.reset_and_capture(Duration::from_secs(12))?;

    for bad in ["panic", "STORE_PROHIBITED", "Guru Meditation", "rst:0x"] {
        if log.iter().any(|l| l.contains(bad)) {
            return Err(format!("boot log contains {bad:?}"));
        }
    }
    require_line(&log, "no GPIO conflicts detected")?;
    require_line(&log, "encoder capture: pcnt active")?;

    // Geometry is the number most likely to silently regress, since the radii
    // cancel out of the normalised path and a wrong value is invisible until
    // something reports m/s or deg/s.
    let drive = log
        .iter()
        .find(|l| l.contains("board probe: drive"))
        .ok_or("no drive-parameter line in boot log")?;
    for expect in ["wheel_r=0.019m", "robot_r=0.195m", "max_wheel=50.00rad/s"] {
        if !drive.contains(expect) {
            return Err(format!("drive params missing {expect:?}; got: {drive}"));
        }
    }
    Ok(())
}

/// Every I2C sensor must answer with a plausible value. Guards the shared-bus
/// handoff: a regression there shows up as one sensor silently returning
/// defaults rather than as a build failure.
fn scenario_sensors(bot: &mut Bot) -> Check {
    // The motor `disable` command also disables the IMU, which then reports
    // all zeros rather than erroring -- so the sensor scenario has to enable
    // first or it measures nothing. Zero the drive targets *before* enabling so
    // asserting the gate cannot act on a stale target from an earlier scenario.
    bot.send_json(r#"{"ct":"i","ic":"t","direction_deg":0,"speed_norm":0.0}"#)?;
    bot.send_json(r#"{"ct":"i","ic":"enable"}"#)?;

    let imu = bot.query("read_imu", "imu")?;
    let az = f64_at(&imu, &["data", "accel", "z"]).ok_or("imu reply has no accel.z")?;
    // At rest on a level-ish surface gravity dominates one axis. A dead driver
    // returns 0.0 for everything, which this catches.
    if az.abs() < 0.5 {
        return Err(format!(
            "accel.z={az:.3}g implies no gravity vector (driver dead?)"
        ));
    }

    let env = bot.query("read_env", "env")?;
    let temp = f64_at(&env, &["data", "temp_c"]).ok_or("env reply has no temp_c")?;
    // -40 C exactly is the HDC2010's all-zeros decode, i.e. it was never
    // started. That was a real bug; keep it pinned.
    if temp <= -39.0 {
        return Err(format!("temp_c={temp:.1} looks like an unstarted HDC2010"));
    }
    if !(0.0..60.0).contains(&temp) {
        return Err(format!("temp_c={temp:.1} out of plausible indoor range"));
    }

    let power = bot.query("read_power", "power")?;
    let mv = f64_at(&power, &["data", "battery_mv"]).ok_or("power reply has no battery_mv")?;
    if !(2_500.0..4_500.0).contains(&mv) {
        return Err(format!("battery_mv={mv} out of range for a 1S pack"));
    }
    Ok(())
}

/// With the motors idle, the query path must report zero movement and stable
/// totals. Cheap, safe, and catches a counter that free-runs.
fn scenario_encoders_idle(bot: &mut Bot) -> Check {
    bot.query("read_encoders", "encoders")?; // rebase the query baseline
    std::thread::sleep(Duration::from_millis(300));
    let a = bot.encoders()?;
    if a.delta != [0, 0, 0] {
        return Err(format!("idle delta should be zero, got {:?}", a.delta));
    }
    let b = bot.encoders()?;
    if a.counts != b.counts {
        return Err(format!(
            "idle totals drifted {:?} -> {:?} with motors off",
            a.counts, b.counts
        ));
    }
    Ok(())
}

/// Forward must move the two active wheels in opposite senses, and reverse must
/// invert both. This is the invariant that a "cleanup" of the
/// `convert_to_body_frame` tuple order would break: reading the strafe
/// component instead of forward makes a 0-degree command drive nothing.
fn scenario_forward_reverse(bot: &mut Bot) -> Check {
    let fwd = bot.drive_and_measure(0.0, 0.85, 5, true)?;
    let rev = bot.drive_and_measure(180.0, 0.85, 5, true)?;

    // Separate "did not move" from "moved the wrong way". This bench unit has a
    // known two-motor droop and real stiction, so a wheel reading ~0 is a
    // mechanical result, not evidence about the frame convention -- reporting it
    // as the latter would send someone to debug the kinematics for no reason.
    check_direction("forward", fwd, 1, -1)?;
    check_direction("reverse", rev, -1, 1)
}

/// Assert the sign of each wheel that actually turned.
///
/// `want_w1`/`want_w2` are expected signs. Wheels below the movement floor are
/// reported as inconclusive rather than failed, and it is only an error if
/// *neither* wheel produced usable motion.
fn check_direction(
    label: &str,
    delta: [i32; 3],
    want_w1: i32,
    want_w2: i32,
) -> Check {
    const FLOOR: i32 = 60;
    let mut conclusive = 0;

    for (index, want) in [want_w1, want_w2].into_iter().enumerate() {
        let got = delta[index];
        if got.abs() < FLOOR {
            continue;
        }
        conclusive += 1;
        if got.signum() != want {
            return Err(format!(
                "{label}: W{} moved {got} but expected sign {want} -- real direction inversion \
                 (check the frame convention and wheel_phase_invert)",
                index + 1
            ));
        }
    }

    if conclusive == 0 {
        return Err(format!(
            "{label}: no wheel exceeded {FLOOR} counts (got {delta:?}); drivetrain did not move, \
             so this says nothing about direction. Mechanical (stiction/droop), not software."
        ));
    }
    Ok(())
}

/// Per-query deltas must partition the totals exactly. If the control loop and
/// the query path ever share a delta baseline again, whichever polls first
/// steals the other's counts and this sum comes up short.
fn scenario_delta_sum(bot: &mut Bot) -> Check {
    bot.send_json(r#"{"ct":"i","ic":"enable"}"#)?;
    let start = bot.encoders()?; // rebase
    let mut summed = [0i64; 3];

    for _ in 0..6 {
        bot.send_json(r#"{"ct":"i","ic":"t","direction_deg":0,"speed_norm":0.75}"#)?;
        std::thread::sleep(MOTION_REFRESH);
        let s = bot.encoders()?;
        for (acc, d) in summed.iter_mut().zip(s.delta) {
            *acc += d as i64;
        }
    }
    bot.make_safe();
    let end = bot.encoders()?;

    for (wheel, (&sum, (&first, &last))) in summed
        .iter()
        .zip(start.counts.iter().zip(end.counts.iter()))
        .enumerate()
    {
        let expected = last as i64 - first as i64;
        // The trailing read after make_safe() can add a few counts of coast,
        // so allow a small tolerance rather than demanding bit-equality.
        if (sum - expected).abs() > 40 {
            return Err(format!(
                "wheel {}: deltas summed to {} but totals moved {} (delta steal?)",
                wheel + 1,
                sum,
                expected
            ));
        }
    }
    if summed.iter().all(|v| v.abs() < 50) {
        return Err("no wheel moved; cannot conclude anything about delta accounting".into());
    }
    Ok(())
}

/// E-stop must latch: after tripping it, a drive command must not move the
/// wheels until the bot is explicitly re-enabled.
///
/// Note this deliberately uses the normal `t` drive command. `probe_speed`
/// bypasses the enable gate by design, so testing with it reports a false
/// failure -- that mistake has been made before.
fn scenario_estop(bot: &mut Bot) -> Check {
    bot.send_json(r#"{"ct":"p","pc":"estop"}"#)?;
    std::thread::sleep(Duration::from_millis(300));
    bot.encoders()?; // rebase after the trip

    let moved = bot.drive_and_measure(0.0, 0.8, 3, false)?;
    if moved.iter().any(|d| d.abs() > 30) {
        return Err(format!("wheels moved {moved:?} while e-stopped"));
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// measurement modes
// ---------------------------------------------------------------------------

/// Encoder counts per wheel revolution: Pololu 5161, 12 CPR x 50:1. Mirrors
/// `POLOLU_5161_COUNTS_PER_WHEEL_REV` in the esp profile.
const COUNTS_PER_REV: f64 = 600.0;

/// The wheels the modes are allowed to drive. W3 is dead by design on the
/// v1-alpha bench unit and is never commanded.
const ACTIVE_WHEELS: [usize; 2] = [0, 1];

/// Encoder polarity relative to the commanded sense. The firmware corrects
/// W2's motor phase but not its encoder, so W2 counts down for a positive
/// `speed_norm`. Folding this in reports every wheel in its *drive* sense: a
/// positive duty reads as positive speed, and a negative reading at positive
/// duty is then a genuine inversion rather than the known wiring.
const ENCODER_SIGN: [f64; 3] = [1.0, -1.0, 1.0];

/// The firmware's motion deadman (`MOTION_WATCHDOG_MS` in owb-core's
/// command_bus.rs): no active motion command for this long and the bus zeroes
/// the wheels.
const DEADMAN: Duration = Duration::from_millis(300);

/// Keepalive cadence inside a held drive window. Chosen so that one dropped
/// command (detected after [`KICK_ACK_WAIT`], then re-sent) still lands inside
/// [`DEADMAN`]: 150 + 90 + a poll or two is about 270 ms.
const KEEPALIVE_PERIOD: Duration = Duration::from_millis(150);

/// How long a keepalive may go unacknowledged before it is re-sent.
const KICK_ACK_WAIT: Duration = Duration::from_millis(90);

/// Reply budget for an in-window encoder sample. Deliberately short and
/// single-shot: a lost sample just widens the gap between its neighbours,
/// whereas the [`Bot::query`] retry ladder would starve the keepalive.
const SAMPLE_REPLY_WAIT: Duration = Duration::from_millis(250);

/// Do not start a sample this close to the next keepalive, so its reply
/// normally lands before the keepalive is due.
const QUERY_SLOT: Duration = Duration::from_millis(50);

/// Minimum spacing between writes inside a held window. Far below
/// [`COMMAND_GAP`], which cannot be used here: 250 ms per write puts two
/// writes (keepalive + sample) past the deadman. That gap exists for slow
/// lock-holding reads like `read_power`; a held window only ever sends the
/// motion command (no lock, non-blocking) and `read_encoders` (a PCNT read, no
/// I2C), and anything dropped is re-sent on the next tick. If a held window
/// reports keepalive gaps on hardware, raise this first.
const TICK_GAP: Duration = Duration::from_millis(30);

/// Below this a wheel is treated as not turning (~0.5 rad/s), so decay and
/// ratio maths are not run on noise.
const MOVING_FLOOR_CPS: f64 = 50.0;

const CHAR_HOLD: Duration = Duration::from_millis(1_000);
/// Spin-up discarded from the start of every characterisation step.
const CHAR_SPINUP_S: f64 = 0.3;
const CHAR_REST: Duration = Duration::from_millis(400);
const CHAR_SAMPLE_EVERY: Duration = Duration::from_millis(150);
/// Breakaway is the lowest duty whose speed stays above this fraction of the
/// wheel's maximum all the way up the sweep.
const BREAKAWAY_FRACTION: f64 = 0.05;
/// A duty whose speed falls more than this fraction below the best speed at
/// any lower duty is non-monotonic. On the v1-alpha unit duty 1.0 (sometimes
/// 0.99) runs at ~60% of the 0.95 speed, and the band above ~0.85 is bistable;
/// anything derived from those points is wrong.
const NONMONOTONIC_DROP: f64 = 0.10;
/// A single sweep step gaining more than this multiple of the region's
/// median step is a jump into a different operating mode (the bistable band),
/// not the linear response, so the usable region stops before it. 3x, not 2x:
/// a real sweep's steps are noisy enough that an ordinary step reaches ~2.1x
/// the median (W1 0.55 -> 0.60 on 2026-10-23), while the genuine mode jump
/// measured at ~4x.
const BISTABLE_JUMP_FACTOR: f64 = 3.0;
/// Duty at which the two wheels' fits are compared for the trim. Mid-range,
/// where the operator actually drives and both fits are well supported;
/// full duty is exactly where this hardware misbehaves.
const TRIM_REF_DUTY: f64 = 0.6;
/// Headroom added to the measured breakaway for `min_forward_speed_norm`.
/// Breakaway was measured from rest on a warm, unloaded wheel; on the floor it
/// is higher, so the floor must sit above it, not on it.
const MIN_FORWARD_MARGIN: f64 = 0.05;

const SOAK_PHASE: Duration = Duration::from_secs(10);
const SOAK_REST: Duration = Duration::from_secs(5);
const SOAK_SAMPLE_EVERY: Duration = Duration::from_millis(250);
/// Skipped at the start of every soak phase before the "first" window. Not
/// just the 120 ms stiction kick: on hardware the per-second series is still
/// ramping through the first second (1460 -> 3970 -> 4064 c/s), and a 0.3 s
/// skip made a flat run read as -72% "decay".
const SOAK_SKIP_S: f64 = 1.5;
/// Length of both the "first" and "last" windows. Equal, so the two rates
/// carry the same noise; 2 s is ~8 samples each.
const SOAK_WINDOW_S: f64 = 2.0;
/// Decay (and decay differences) smaller than this many percentage points are
/// within the noise of a host-timestamped rate estimate and are not called.
const SIGNIFICANT_PP: f64 = 5.0;

const T_STOP: &str = r#"{"ct":"i","ic":"t","direction_deg":0,"speed_norm":0.0}"#;

fn probe_cmd(
    wheel: usize,
    duty: f64,
) -> String {
    format!(r#"{{"ct":"i","ic":"probe_speed","wheel":{wheel},"speed_norm":{duty:.2}}}"#)
}

fn forward_cmd(duty: f64) -> String {
    format!(r#"{{"ct":"i","ic":"t","direction_deg":0.0,"speed_norm":{duty:.2}}}"#)
}

/// `drive_loop` with only the fields that are being changed; both are
/// optional on the firmware side.
fn drive_loop_json(
    closed_loop: Option<bool>,
    pwm_stagger: Option<bool>,
) -> String {
    let mut fields = String::new();
    if let Some(v) = closed_loop {
        fields.push_str(&format!(r#","closed_loop":{v}"#));
    }
    if let Some(v) = pwm_stagger {
        fields.push_str(&format!(r#","pwm_stagger":{v}"#));
    }
    format!(r#"{{"ct":"i","ic":"drive_loop"{fields}}}"#)
}

/// Positive sweep 0.10..=1.00 in 0.05 steps, then the symmetry points.
/// Integer steps so the duties print and compare exactly.
fn characterize_duties() -> Vec<f64> {
    let mut duties: Vec<f64> = (2..=20).map(|k| f64::from(k) * 0.05).collect();
    duties.extend([-0.3, -0.6, -1.0]);
    duties
}

struct SweepRow {
    duty: f64,
    /// Drive-sense counts/s per [`ACTIVE_WHEELS`] slot; `None` if too few
    /// samples made it back to fit a rate.
    speed: [Option<f64>; 2],
}

/// Open-loop duty sweep per wheel. See the module docs.
fn mode_characterize(
    bot: &mut Bot,
    csv: Option<&str>,
) -> Check {
    let duties = characterize_duties();
    println!(
        "    sweeping {} duty points x {} wheels, interleaved (~{} s)",
        duties.len(),
        ACTIVE_WHEELS.len(),
        duties.len() * ACTIVE_WHEELS.len() * 9 / 5
    );
    println!("    speeds are in drive sense (W2's encoder sign folded in)");
    println!(
        "    {:>6} {:>9} {:>9} {:>9} {:>9}",
        "duty", "W1 c/s", "W2 c/s", "W1 rad/s", "W2 rad/s"
    );

    let mut rows = Vec::with_capacity(duties.len());
    for &duty in &duties {
        let mut speed = [None; 2];
        let mut notes = Vec::new();
        // Interleave: each wheel rests while the other runs, so heating builds
        // evenly across the sweep instead of W2's whole sweep starting hot.
        for (slot, &wheel) in ACTIVE_WHEELS.iter().enumerate() {
            let hold = bot.hold_and_sample(
                &probe_cmd(wheel, duty),
                &probe_cmd(wheel, 0.0),
                CHAR_HOLD,
                CHAR_SAMPLE_EVERY,
            )?;
            speed[slot] = rate_in_window(&hold.samples, wheel, CHAR_SPINUP_S, f64::INFINITY)
                .map(|r| r * ENCODER_SIGN[wheel]);
            if hold.deadman_risk() {
                notes.push(format!(
                    "W{} keepalive gap {} ms",
                    wheel + 1,
                    hold.worst_keepalive_gap.as_millis()
                ));
            }
            bot.rest(CHAR_REST)?;
        }
        let note = if notes.is_empty() {
            String::new()
        }
        else {
            format!("  ! {} (deadman may have cut this step)", notes.join(", "))
        };
        println!(
            "    {duty:>6.2} {} {} {} {}{note}",
            fmt_opt(speed[0], 9, 0),
            fmt_opt(speed[1], 9, 0),
            fmt_opt(speed[0].map(counts_to_rad_s), 9, 2),
            fmt_opt(speed[1].map(counts_to_rad_s), 9, 2),
        );
        rows.push(SweepRow { duty, speed });
    }

    // -- analysis ---------------------------------------------------------
    println!("    analysis:");
    let mut fits = Vec::new();
    for (slot, &wheel) in ACTIVE_WHEELS.iter().enumerate() {
        let positive: Vec<(f64, f64)> = rows
            .iter()
            .filter(|r| r.duty > 0.0)
            .filter_map(|r| r.speed[slot].map(|v| (r.duty, v)))
            .collect();
        let name = format!("W{}", wheel + 1);
        // A wheel that reads backwards at positive duty means ENCODER_SIGN no
        // longer matches the wiring. Say so, then derive on magnitudes: the
        // profile numbers are about how fast, not which way.
        let net: f64 = positive.iter().map(|p| p.1).sum();
        if net < -MOVING_FLOOR_CPS {
            println!(
                "      ! {name} reads negative at positive duty: its encoder sign differs from \
                 ENCODER_SIGN (or the motor phase is inverted); deriving from magnitudes"
            );
        }
        let magnitudes: Vec<(f64, f64)> = positive.iter().map(|&(d, v)| (d, v.abs())).collect();
        fits.push(analyse_wheel(&magnitudes));
    }

    // Loud, and before anything derived: the old analysis trusted duty 1.0 on
    // a unit where it collapses, and suggested max_wheel_rad_s 29.7 and a 0.639
    // trim -- both nonsense.
    let drops: Vec<String> = fits
        .iter()
        .zip(ACTIVE_WHEELS)
        .flat_map(|(fit, wheel)| {
            fit.drops.iter().map(move |d| {
                format!(
                    "W{} duty {:.2}: {:.0} c/s, {:.0}% below {:.0} c/s at {:.2}",
                    wheel + 1,
                    d.duty,
                    d.speed,
                    (1.0 - d.speed / d.best_speed) * 100.0,
                    d.best_speed,
                    d.best_duty
                )
            })
        })
        .collect();
    if !drops.is_empty() {
        println!(
            "    !!! NON-MONOTONIC SPEED: more speed below these duties than at them \
             (collapse/bistable band):"
        );
        for line in &drops {
            println!("    !!!   {line}");
        }
        println!("    !!! everything derived below uses only each wheel's monotonic region");
    }

    for (fit, wheel) in fits.iter().zip(ACTIVE_WHEELS) {
        let name = format!("W{}", wheel + 1);
        match fit.breakaway {
            Some(b) => println!("      {name}: breakaway duty {b:.2}"),
            None => {
                println!(
                    "      {name}: never sustained >{:.0}% of max speed -- no breakaway (wheel not turning?)",
                    BREAKAWAY_FRACTION * 100.0
                );
                continue;
            }
        }
        if let (Some(top), Some(v)) = (fit.ceiling, fit.ceiling_speed) {
            let why = match fit.ceiling_reason {
                CeilingReason::SweepTop => "top of the sweep".to_string(),
                CeilingReason::BeforeDrop(d) => format!("last duty before the drop at {d:.2}"),
                CeilingReason::BeforeJump(d) => {
                    format!("last duty before a bistable jump at {d:.2}")
                }
            };
            println!(
                "          monotonic region {:.2}..={top:.2} ({why}): {v:.0} c/s = {:.2} rad/s at the top",
                fit.breakaway.unwrap_or(0.0),
                counts_to_rad_s(v)
            );
        }
        if let Some((m, c)) = fit.line {
            println!(
                "          fit over that region: {m:.0} c/s per unit duty {c:+.0} c/s  \
                 (= {:.2} rad/s per unit duty {:+.2} rad/s)",
                counts_to_rad_s(m),
                counts_to_rad_s(c)
            );
        }
        if let Some(v) = fit.full {
            println!(
                "          duty 1.00 (informational): {v:.0} c/s = {:.2} rad/s",
                counts_to_rad_s(v)
            );
        }
    }
    // Reverse/forward magnitude at each symmetry point. A clear asymmetry is a
    // driver or brush issue, and means one trim cannot serve both directions.
    // Points in the non-monotonic band compare two different modes, so read
    // the -1.00 figure with the warning above in mind.
    for (slot, &wheel) in ACTIVE_WHEELS.iter().enumerate() {
        let symmetry: Vec<String> = rows
            .iter()
            .filter(|r| r.duty < 0.0)
            .filter_map(|neg| {
                let rev = neg.speed[slot]?;
                let fwd = rows
                    .iter()
                    .find(|r| (r.duty + neg.duty).abs() < 1e-6)?
                    .speed[slot]?;
                (fwd.abs() >= MOVING_FLOOR_CPS)
                    .then(|| format!("{:.2} {:.0}%", neg.duty, rev.abs() / fwd.abs() * 100.0))
            })
            .collect();
        if !symmetry.is_empty() {
            println!(
                "      W{} |reverse|/forward: {}",
                wheel + 1,
                symmetry.join("  ")
            );
        }
    }

    // -- profile suggestions ----------------------------------------------
    let mut trim = [1.0f64; 3];
    let mut trim_ok = false;
    let mut max_rad_s = None;
    let mut ceiling = None;
    if let ([Some(b1), Some(b2)], [Some(top1), Some(top2)], [Some(line1), Some(line2)]) = (
        [fits[0].breakaway, fits[1].breakaway],
        [fits[0].ceiling, fits[1].ceiling],
        [fits[0].line, fits[1].line],
    ) {
        // Both wheels must be inside their usable region, so the pair's
        // ceiling is the lower of the two.
        let common_top = top1.min(top2);
        let common_bottom = b1.max(b2);
        ceiling = Some(common_top);
        // The slower wheel at the shared ceiling caps what the trimmed pair
        // can do, and the PID normalises feedback by this value, so it has to
        // be a speed the drive reliably reaches.
        let at_top = |slot: usize| {
            rows.iter()
                .find(|r| (r.duty - common_top).abs() < 1e-6)
                .and_then(|r| r.speed[slot])
                .map(f64::abs)
        };
        if let (Some(v1), Some(v2)) = (at_top(0), at_top(1)) {
            max_rad_s = Some(counts_to_rad_s(v1.min(v2)));
        }
        if common_bottom <= common_top {
            let reference = TRIM_REF_DUTY.clamp(common_bottom, common_top);
            let speed_at = |(m, c): (f64, f64)| m * reference + c;
            let (v1, v2) = (speed_at(line1), speed_at(line2));
            let (fast, slow) = if v1 >= v2 { (0, 1) } else { (1, 0) };
            let lines = [line1, line2];
            if let Some(t) = suggest_trim(lines[fast], lines[slow], reference) {
                trim[fast] = t;
                trim_ok = true;
                println!(
                    "      W1/W2 at duty {reference:.2} (from the fits): {v1:.0}/{v2:.0} c/s = {:.3} \
                     -> W{} is faster; trim it to {t:.3}",
                    v1 / v2,
                    fast + 1
                );
            }
        }
        else {
            println!(
                "      monotonic regions do not overlap ({common_bottom:.2} > {common_top:.2}); no trim"
            );
        }
    }
    let min_forward = suggest_min_forward(&[fits[0].breakaway, fits[1].breakaway]);

    println!("    // measured by `hil-check --characterize` (probe_speed, wheels unloaded)");
    match (max_rad_s, ceiling) {
        (Some(v), Some(top)) => println!(
            "    // DriveTuning -- slower wheel at the duty ceiling {top:.2}\n    max_wheel_rad_s: {v:.1},"
        ),
        _ => println!(
            "    // max_wheel_rad_s: not derived (needs both wheels' monotonic region)"
        ),
    }
    if let Some(top) = ceiling {
        println!(
            "    // recommended duty ceiling: {top:.2} -- no profile field yet; above it this \
             unit's speed is not reliably monotonic, so cap speed_norm there"
        );
    }
    println!("    // BoardProfile -- absolute values, not relative to the current trim");
    if trim_ok {
        println!(
            "    default_wheel_trim: [{:.3}, {:.3}, {:.3}],",
            trim[0], trim[1], trim[2]
        );
    }
    else {
        println!(
            "    // default_wheel_trim: not derived (needs both wheels' fits over a shared region)"
        );
    }
    match min_forward {
        Some(v) => {
            println!("    min_forward_speed_norm: {v:.2}, // breakaway + {MIN_FORWARD_MARGIN:.2}")
        }
        None => println!("    // min_forward_speed_norm: not derived (no breakaway found)"),
    }

    if let Some(path) = csv {
        let mut out = String::from("duty,w1_cps,w2_cps,w1_rad_s,w2_rad_s\n");
        for r in &rows {
            let cell =
                |v: Option<f64>, prec: usize| v.map_or(String::new(), |v| format!("{v:.prec$}"));
            out.push_str(&format!(
                "{:.2},{},{},{},{}\n",
                r.duty,
                cell(r.speed[0], 1),
                cell(r.speed[1], 1),
                cell(r.speed[0].map(counts_to_rad_s), 3),
                cell(r.speed[1].map(counts_to_rad_s), 3),
            ));
        }
        write_csv(path, &out);
    }

    if fits.iter().all(|f| f.breakaway.is_none()) {
        return Err(
            "no wheel ever turned during the sweep; mechanical, or the board ignored probe_speed"
                .into(),
        );
    }
    Ok(())
}

/// Derived figures for one wheel's positive sweep.
struct WheelFit {
    breakaway: Option<f64>,
    /// Points slower than some lower duty by more than [`NONMONOTONIC_DROP`].
    drops: Vec<SpeedDrop>,
    /// Top of the usable (monotonic, linear) region.
    ceiling: Option<f64>,
    ceiling_reason: CeilingReason,
    /// Measured speed at [`WheelFit::ceiling`].
    ceiling_speed: Option<f64>,
    /// `(slope, intercept)` of counts/s against duty over breakaway..=ceiling.
    line: Option<(f64, f64)>,
    /// Measured speed at duty 1.0. Informational only: on this unit it is
    /// exactly the point that cannot be trusted.
    full: Option<f64>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum CeilingReason {
    SweepTop,
    /// Stopped below the first non-monotonic duty.
    BeforeDrop(f64),
    /// Stopped below a step that jumped into the bistable band.
    BeforeJump(f64),
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct SpeedDrop {
    duty: f64,
    speed: f64,
    /// The fastest lower duty, and its speed.
    best_duty: f64,
    best_speed: f64,
}

/// `points` are `(duty, counts/s magnitude)`, ascending by duty.
fn analyse_wheel(points: &[(f64, f64)]) -> WheelFit {
    let breakaway = breakaway_duty(points, BREAKAWAY_FRACTION);
    let drops = speed_drops(points, NONMONOTONIC_DROP);
    let mut fit = WheelFit {
        breakaway,
        drops,
        ceiling: None,
        ceiling_reason: CeilingReason::SweepTop,
        ceiling_speed: None,
        line: None,
        full: points
            .iter()
            .find(|&&(d, _)| (d - 1.0).abs() < 1e-6)
            .map(|&(_, v)| v),
    };
    let Some(b) = breakaway
    else {
        return fit;
    };
    // A blip below breakaway (coast, a stray count) is not the drive
    // collapsing; only drops inside the driven range matter.
    fit.drops.retain(|d| d.duty > b);

    // Monotonic region: breakaway up to the last point before the first drop.
    let first_drop = fit.drops.iter().map(|d| d.duty).find(|&d| d > b);
    let mut region: Vec<(f64, f64)> = points
        .iter()
        .copied()
        .filter(|&(d, _)| d >= b - 1e-9 && first_drop.is_none_or(|top| d < top - 1e-9))
        .collect();
    if let Some(top) = first_drop {
        fit.ceiling_reason = CeilingReason::BeforeDrop(top);
    }
    // Then cut before any bistable jump, so the fit describes the linear
    // response rather than straddling two operating modes.
    if let Some(jump) = bistable_jump(&region, BISTABLE_JUMP_FACTOR) {
        region.retain(|&(d, _)| d < jump - 1e-9);
        fit.ceiling_reason = CeilingReason::BeforeJump(jump);
    }
    if let Some(&(top, v)) = region.last() {
        fit.ceiling = Some(top);
        fit.ceiling_speed = Some(v);
    }
    fit.line = linear_fit(&region);
    fit
}

/// Every point slower than the best speed at any lower duty by more than
/// `fraction`. `points` ascending by duty.
fn speed_drops(
    points: &[(f64, f64)],
    fraction: f64,
) -> Vec<SpeedDrop> {
    let mut out = Vec::new();
    let mut best: Option<(f64, f64)> = None;
    for &(duty, speed) in points {
        if let Some((best_duty, best_speed)) = best {
            if best_speed >= MOVING_FLOOR_CPS && speed < best_speed * (1.0 - fraction) {
                out.push(SpeedDrop {
                    duty,
                    speed,
                    best_duty,
                    best_speed,
                });
            }
        }
        if best.is_none_or(|(_, v)| speed > v) {
            best = Some((duty, speed));
        }
    }
    out
}

/// Duty at which the sweep first jumps by more than `factor` x the region's
/// median step gain, if any. `region` starts at breakaway, ascending.
///
/// The first step out of breakaway is excluded: leaving stiction is its own
/// nonlinearity and routinely gains more than later steps. Needs at least
/// three remaining steps for the median to mean anything.
fn bistable_jump(
    region: &[(f64, f64)],
    factor: f64,
) -> Option<f64> {
    let steps: Vec<(f64, f64)> = region
        .windows(2)
        .skip(1)
        .map(|w| (w[1].0, w[1].1 - w[0].1))
        .collect();
    if steps.len() < 3 {
        return None;
    }
    let mut gains: Vec<f64> = steps.iter().map(|s| s.1).collect();
    gains.sort_by(f64::total_cmp);
    let median = gains[gains.len() / 2];
    if median <= 0.0 {
        return None;
    }
    steps
        .iter()
        .find(|&&(_, gain)| gain > factor * median)
        .map(|&(duty, _)| duty)
}

#[derive(Clone, Copy)]
enum SoakDrive {
    Alone(usize),
    Both { stagger: bool },
}

impl SoakDrive {
    /// Which [`ACTIVE_WHEELS`] slots this phase drives.
    fn slots(self) -> [bool; 2] {
        match self {
            Self::Alone(wheel) => [wheel == 0, wheel == 1],
            Self::Both { .. } => [true, true],
        }
    }
}

struct PhaseStats {
    tag: &'static str,
    label: &'static str,
    ran: bool,
    first: [Option<f64>; 2],
    decay: [Option<f64>; 2],
}

/// Two-wheel droop soak. See the module docs.
fn mode_soak(
    bot: &mut Bot,
    duty: f64,
    csv: Option<&str>,
) -> Check {
    let phases = [
        ("a", "W1 alone", SoakDrive::Alone(0)),
        ("b", "W2 alone", SoakDrive::Alone(1)),
        ("c", "both", SoakDrive::Both { stagger: false }),
        ("d", "both + pwm_stagger", SoakDrive::Both { stagger: true }),
        (
            "e",
            "both again (control)",
            SoakDrive::Both { stagger: false },
        ),
    ];
    let phase_s = SOAK_PHASE.as_secs_f64();
    println!(
        "    duty {duty:.2}: {} phases x {phase_s:.0} s, {} s rest between (~{} s)",
        phases.len(),
        SOAK_REST.as_secs(),
        phases.len() as u64 * SOAK_PHASE.as_secs()
            + (phases.len() as u64 - 1) * SOAK_REST.as_secs()
            + 15
    );
    println!(
        "    a/b drive raw duty (probe_speed); c-e drive `t` forward, whose per-wheel duty goes\n    \
         through kinematics, trim and the stiction floor -- compare decay %, not absolute speed"
    );

    // Pin a known baseline: closed_loop would hide exactly the droop being
    // measured, and a stagger left on by a crashed run would contaminate the
    // stagger-off controls.
    let drive_loop = bot.send_optional(&drive_loop_json(Some(false), Some(false)));
    match drive_loop {
        Ack::Accepted => println!("    drive_loop: closed_loop=false pwm_stagger=false"),
        Ack::Unsupported => println!(
            "    drive_loop: unsupported by this firmware (open loop only); stagger phase will be skipped"
        ),
        Ack::Silent => println!(
            "    drive_loop: never acknowledged; loop/stagger state unknown, stagger phase will be skipped"
        ),
    }

    let mut csv_rows = String::from("phase,t_s,w1_counts,w2_counts,w1_cps,w2_cps\n");
    let mut stats: Vec<PhaseStats> = Vec::with_capacity(phases.len());

    for (index, &(tag, label, drive)) in phases.iter().enumerate() {
        if index > 0 {
            bot.rest(SOAK_REST)?;
        }
        let skipped = |why: &str| {
            println!("    [{tag}] {label}: SKIPPED ({why})");
            PhaseStats {
                tag,
                label,
                ran: false,
                first: [None; 2],
                decay: [None; 2],
            }
        };

        let hold = match drive {
            SoakDrive::Alone(wheel) => {
                println!("    [{tag}] {label} (probe_speed {duty:.2})");
                bot.hold_and_sample(
                    &probe_cmd(wheel, duty),
                    &probe_cmd(wheel, 0.0),
                    SOAK_PHASE,
                    SOAK_SAMPLE_EVERY,
                )?
            }
            SoakDrive::Both { stagger } => {
                println!("    [{tag}] {label} (t forward {duty:.2})");
                if stagger {
                    if drive_loop != Ack::Accepted {
                        stats.push(skipped("firmware did not accept drive_loop"));
                        continue;
                    }
                    // Armed *before* sending: if only the ack is lost the
                    // firmware may still have applied it, and make_safe must
                    // then put it back.
                    bot.stagger_armed = true;
                    if bot.send_optional(&drive_loop_json(None, Some(true))) != Ack::Accepted {
                        bot.reset_stagger();
                        stats.push(skipped("pwm_stagger=true not acknowledged"));
                        continue;
                    }
                }
                // probe_speed drops the enable gate, so `t` needs it back.
                let held = bot.send_json(r#"{"ct":"i","ic":"enable"}"#).and_then(|()| {
                    bot.hold_and_sample(&forward_cmd(duty), T_STOP, SOAK_PHASE, SOAK_SAMPLE_EVERY)
                });
                // Unconditionally, so a failed phase cannot leave stagger on
                // under the stagger-off control that follows it.
                if stagger {
                    bot.reset_stagger();
                }
                let _ = bot.send_json(r#"{"ct":"i","ic":"disable"}"#);
                held?
            }
        };

        // Coalesced is not lost: the firmware already had a motion frame
        // queued, which is what feeds the deadman. Only "unanswered" is.
        println!(
            "      link: {} keepalives: {} acked, {} coalesced by firmware, {} unanswered; \
             {}/{} samples, worst keepalive gap {} ms{}",
            hold.kicks_sent,
            hold.kicks_acked,
            hold.kicks_coalesced,
            hold.kicks_sent
                .saturating_sub(hold.kicks_acked + hold.kicks_coalesced),
            hold.samples.len(),
            hold.queries_sent,
            hold.worst_keepalive_gap.as_millis(),
            if hold.deadman_risk() {
                "  ! deadman may have cut the drive"
            }
            else {
                ""
            }
        );

        let mut first = [None; 2];
        let mut decay = [None; 2];
        for (slot, &wheel) in ACTIVE_WHEELS.iter().enumerate() {
            if !drive.slots()[slot] {
                continue;
            }
            let sign = ENCODER_SIGN[wheel];
            let rate = |from: f64, to: f64| {
                rate_in_window(&hold.samples, wheel, from, to).map(|r| r * sign)
            };
            first[slot] = rate(SOAK_SKIP_S, SOAK_SKIP_S + SOAK_WINDOW_S);
            // Open-ended at the top so a sample landing at the very end of
            // the hold still counts; the window is still SOAK_WINDOW_S long.
            let last = rate(phase_s - SOAK_WINDOW_S, phase_s + 1.0);
            decay[slot] = match (first[slot], last) {
                (Some(f), Some(l)) if f.abs() >= MOVING_FLOOR_CPS => decay_percent(f, l),
                _ => None,
            };
            let series: Vec<String> = (0..phase_s as usize)
                .map(|k| fmt_opt(rate(k as f64, k as f64 + 1.0), 5, 0))
                .collect();
            println!(
                "      W{}: early {} c/s  late {} c/s  decay {}%",
                wheel + 1,
                fmt_opt(first[slot], 5, 0),
                fmt_opt(last, 5, 0),
                fmt_opt(decay[slot], 5, 1),
            );
            println!("          per second: {}", series.join(" "));
        }

        let mut prev: Option<&Sample> = None;
        for s in &hold.samples {
            let cps = |slot: usize| {
                prev.and_then(|p| {
                    let wheel = ACTIVE_WHEELS[slot];
                    let dt = s.t_s - p.t_s;
                    (dt > 0.0).then(|| {
                        f64::from(s.counts[wheel].wrapping_sub(p.counts[wheel])) / dt
                            * ENCODER_SIGN[wheel]
                    })
                })
                .map_or(String::new(), |v| format!("{v:.1}"))
            };
            csv_rows.push_str(&format!(
                "{tag},{:.3},{},{},{},{}\n",
                s.t_s,
                s.counts[ACTIVE_WHEELS[0]],
                s.counts[ACTIVE_WHEELS[1]],
                cps(0),
                cps(1)
            ));
            prev = Some(s);
        }

        stats.push(PhaseStats {
            tag,
            label,
            ran: true,
            first,
            decay,
        });
    }

    // -- summary ------------------------------------------------------------
    println!(
        "    summary (decay = mean speed {:.1}-{:.1} s vs the last {:.1} s; positive = slowed down):",
        SOAK_SKIP_S,
        SOAK_SKIP_S + SOAK_WINDOW_S,
        SOAK_WINDOW_S
    );
    println!("      {:<24} {:>9} {:>9}", "phase", "W1 decay", "W2 decay");
    for p in &stats {
        let cell = |slot: usize| match (p.ran, p.decay[slot]) {
            (false, _) => format!("{:>9}", "skipped"),
            (true, Some(d)) => format!("{d:>8.1}%"),
            (true, None) => format!("{:>9}", "-"),
        };
        println!(
            "      {:<24} {} {}",
            format!("{} {}", p.tag, p.label),
            cell(0),
            cell(1)
        );
    }

    let by_tag = |tag: &str| stats.iter().find(|p| p.tag == tag && p.ran);
    let single = [
        by_tag("a").and_then(|p| p.decay[0]),
        by_tag("b").and_then(|p| p.decay[1]),
    ]
    .into_iter()
    .flatten()
    .reduce(f64::max);
    let both_mean = |tag: &str| by_tag(tag).and_then(|p| mean(&p.decay));
    let start_drop = match (
        by_tag("c").and_then(|p| mean(&p.first)),
        by_tag("e").and_then(|p| mean(&p.first)),
    ) {
        (Some(c), Some(e)) if c.abs() >= MOVING_FLOOR_CPS => decay_percent(c, e),
        _ => None,
    };
    let every_decay: Vec<f64> = stats
        .iter()
        .flat_map(|p| p.decay.iter().flatten().copied())
        .collect();
    println!("    interpretation:");
    for line in interpret_soak(
        single,
        both_mean("c"),
        both_mean("d"),
        both_mean("e"),
        start_drop,
        &every_decay,
    ) {
        println!("      - {line}");
    }

    if let Some(path) = csv {
        write_csv(path, &csv_rows);
    }

    if stats.iter().all(|p| p.decay.iter().all(Option::is_none)) {
        return Err("no phase produced a measurable speed; the drivetrain did not move".into());
    }
    Ok(())
}

/// Turn the soak's decay figures into one-line hints. Inputs are percent
/// decay: worst single-wheel, and the wheel mean of each two-wheel phase;
/// `repeat_start_drop` is how much slower phase (e) *started* than (c);
/// `every_decay` is every per-wheel decay that was measured.
///
/// Only makes a claim when the number behind it clears [`SIGNIFICANT_PP`].
/// Inside that band the honest answer is "no droop", and it says so once
/// instead of narrating noise (a spin-up-contaminated run once produced
/// "still warming up" and "per-motor heating" for a perfectly flat drive).
fn interpret_soak(
    single: Option<f64>,
    both: Option<f64>,
    stagger: Option<f64>,
    repeat: Option<f64>,
    repeat_start_drop: Option<f64>,
    every_decay: &[f64],
) -> Vec<String> {
    if every_decay.is_empty() {
        return vec!["no phase produced a decay figure; nothing to interpret".into()];
    }
    let worst = every_decay.iter().fold(0.0f64, |acc, d| acc.max(d.abs()));
    let start_flat = repeat_start_drop.is_none_or(|d| d.abs() < SIGNIFICANT_PP);
    let mut out = Vec::new();
    if worst < SIGNIFICANT_PP && start_flat {
        out.push(format!(
            "no droop: all phases within {worst:.1}% (noise threshold {SIGNIFICANT_PP:.0}%)"
        ));
        if stagger.is_none() {
            out.push("stagger phase not measured".into());
        }
        return out;
    }

    match (single, both) {
        (Some(s), Some(b)) if b - s >= SIGNIFICANT_PP => out.push(format!(
            "both-wheel decay {b:.1}% >> single-wheel {s:.1}%: points at supply sag or a \
             thermal limit under combined load (battery/regulator/driver), not a per-motor fault"
        )),
        (Some(s), Some(b)) if s >= SIGNIFICANT_PP && b >= SIGNIFICANT_PP => out.push(format!(
            "single-wheel runs decay about as much as both ({s:.1}% vs {b:.1}%): per-motor \
             heating, not a combined-load effect"
        )),
        (Some(s), Some(b)) if s >= SIGNIFICANT_PP => out.push(format!(
            "single-wheel decay {s:.1}% but both-wheel only {b:.1}%: not a combined-load \
             limit; inconsistent, rerun before reading anything into it"
        )),
        _ => {}
    }

    // Stagger only has something to fix if the stagger-off phases drooped.
    let controls = [both, repeat].into_iter().flatten().reduce(f64::min);
    match (stagger, controls) {
        (None, _) => out.push("stagger phase not measured".into()),
        (Some(st), Some(ctl)) if ctl >= SIGNIFICANT_PP && st <= ctl - SIGNIFICANT_PP => {
            out.push(format!(
                "stagger phase decayed {st:.1}% vs {ctl:.1}% for the best stagger-off phase: \
                 PWM interleaving helps (simultaneous switch-on current is part of the problem)"
            ))
        }
        (Some(st), Some(ctl)) if st >= ctl + SIGNIFICANT_PP => out.push(format!(
            "stagger phase decayed more ({st:.1}% vs {ctl:.1}% stagger-off): interleaving \
             does not help here"
        )),
        (Some(st), Some(ctl)) if ctl >= SIGNIFICANT_PP => out.push(format!(
            "stagger phase {st:.1}% vs {ctl:.1}% stagger-off: no clear benefit from PWM \
             interleaving"
        )),
        _ => {}
    }

    match repeat_start_drop {
        Some(drop) if drop >= SIGNIFICANT_PP => out.push(format!(
            "control (e) started {drop:.1}% slower than (c): the {}-s rests did not fully \
             recover it, so the loss accumulates across runs (thermal or supply recovery)",
            SOAK_REST.as_secs()
        )),
        Some(drop) if drop <= -SIGNIFICANT_PP => out.push(format!(
            "control (e) started {:.1}% faster than (c)",
            drop.abs()
        )),
        _ => {}
    }

    if out.is_empty() {
        out.push(format!(
            "no claim: the phase-to-phase differences are inside the {SIGNIFICANT_PP:.0}% noise \
             threshold (worst single decay {worst:.1}%)"
        ));
    }
    out
}

// ---------------------------------------------------------------------------
// serial plumbing
// ---------------------------------------------------------------------------

struct EncoderSample {
    counts: [i32; 3],
    delta: [i32; 3],
}

/// Absolute encoder counts, timestamped (seconds since the start of the held
/// window) on the host when the reply arrived.
#[derive(Clone, Copy, Debug)]
struct Sample {
    t_s: f64,
    counts: [i32; 3],
}

/// Result of one [`Bot::hold_and_sample`] window.
struct Hold {
    samples: Vec<Sample>,
    /// Longest stretch between acknowledged keepalives (host-side, so +-20 ms).
    worst_keepalive_gap: Duration,
    kicks_sent: usize,
    kicks_acked: usize,
    /// "[USB] motion bus full; dropped (coalesced)": a motion frame was
    /// already queued, so this one was redundant rather than lost.
    kicks_coalesced: usize,
    queries_sent: usize,
}

impl Hold {
    /// Past the deadman the firmware zeroed the wheels mid-window, and the
    /// samples around the gap understate the speed.
    fn deadman_risk(&self) -> bool { self.worst_keepalive_gap > DEADMAN }
}

/// Outcome of [`Bot::send_optional`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Ack {
    Accepted,
    /// The firmware answered "unrecognized command" (consistently).
    Unsupported,
    /// Nothing definite came back.
    Silent,
}

struct Bot {
    port: Box<dyn serialport::SerialPort>,
    /// Kept so the handle can be reopened after a board reset re-enumerates
    /// the USB device.
    path: String,
    /// Leftover bytes from a read that split a line.
    pending: String,
    /// A mode switched `pwm_stagger` on and it has not been confirmed off
    /// again. [`Bot::make_safe`] resets it only when set, so firmware without
    /// `drive_loop` does not pay for an unsupported command on every scenario.
    stagger_armed: bool,
}

impl Bot {
    fn open(path: &str) -> Result<Self, String> {
        Ok(Self {
            port: open_port(path)?,
            path: path.to_string(),
            pending: String::new(),
            stagger_armed: false,
        })
    }
}

/// Open the serial handle with the settings the harness relies on.
///
/// Standalone rather than a `Bot` method because the reopen-after-reset path
/// needs to replace `Bot::port` in place, and `Bot` implements `Drop` so it
/// cannot be destructured.
fn open_port(path: &str) -> Result<Box<dyn serialport::SerialPort>, String> {
    let mut port = serialport::new(path, 115_200)
        .timeout(Duration::from_millis(20))
        .open()
        .map_err(|e| e.to_string())?;
    // Assert DTR: on the ESP32-S3 USB Serial/JTAG peripheral this signals that
    // a terminal is attached. pyserial does this on open, which is part of why
    // equivalent Python probes behaved differently.
    let _ = port.write_data_terminal_ready(true);
    let _ = port.write_request_to_send(false);
    Ok(port)
}

impl Bot {
    /// Pulse the reset lines and collect the boot log.
    fn reset_and_capture(
        &mut self,
        window: Duration,
    ) -> Result<Vec<String>, String> {
        let _ = self.port.write_data_terminal_ready(false);
        let _ = self.port.write_request_to_send(true);
        std::thread::sleep(Duration::from_millis(100));
        let _ = self.port.write_request_to_send(false);
        // Re-assert DTR: the pulse above deasserted it, and leaving it low
        // stalls the device's TX flush (see `open`).
        let _ = self.port.write_data_terminal_ready(true);
        let _ = self.port.clear(serialport::ClearBuffer::Input);
        self.pending.clear();

        // The reset also resets the USB Serial/JTAG peripheral, so the device
        // re-enumerates and the handle opened before the pulse goes stale:
        // reads from it starve for seconds at a time and replies trickle in
        // late. Reopen so the rest of the run talks to the live device. (An
        // equivalent Python probe never hit this because it opens a fresh
        // handle per invocation.)
        std::thread::sleep(Duration::from_millis(600));
        if let Ok(fresh) = open_port(&self.path) {
            self.port = fresh;
            self.pending.clear();
        }

        let mut out = Vec::new();
        let deadline = Instant::now() + window;
        while Instant::now() < deadline {
            out.extend(self.read_lines());
        }
        if out.is_empty() {
            return Err("no output after reset (wrong port, or board not running?)".into());
        }
        Ok(out)
    }

    fn send_json(
        &mut self,
        json: &str,
    ) -> Result<(), String> {
        // Fire-and-forget commands: wait for the firmware's acknowledgement so
        // the next write cannot overrun it, and re-send if it never arrives.
        //
        // Retrying matters as much here as for queries. A dropped `enable` is
        // silent and poisons everything downstream: the board stays disabled,
        // every later drive command is ignored, and the encoders read zero --
        // which looks exactly like a seized drivetrain. That misdiagnosis is
        // precisely what this retry prevents.
        //
        // Reply-bearing queries must NOT come through here: `settle` returns on
        // the first `[USB_REPLY]` it sees, which would swallow the reply the
        // caller is about to wait for.
        for _ in 0..QUERY_ATTEMPTS {
            self.write_line(json)?;
            if self.settle() {
                return Ok(());
            }
            if verbose() {
                println!("      !! no acknowledgement for {json}; re-sending");
            }
        }
        Err(format!(
            "command never acknowledged after {QUERY_ATTEMPTS} attempts: {json}"
        ))
    }

    /// Write one JSON line and nothing else. No settling, so the caller keeps
    /// ownership of whatever the firmware says next.
    ///
    /// Always leaves [`COMMAND_GAP`] of quiet before writing. The firmware's
    /// "[USB] command accepted" means *queued to the command bus*, not
    /// *executed*, so acknowledgement alone is not enough spacing: a command
    /// written immediately after it lands while the executor is still working
    /// and the 64-byte RX FIFO is unattended, and its bytes are dropped
    /// outright -- no parse, no reply, no error. Observed directly: a
    /// `read_imu` sent 0 ms after `enable` was acknowledged simply vanished.
    fn write_line(
        &mut self,
        json: &str,
    ) -> Result<(), String> {
        std::thread::sleep(COMMAND_GAP);
        self.write_raw(json)
    }

    /// [`Bot::write_line`] without the pacing gap, for the held-drive loop,
    /// which does its own (much tighter) pacing.
    fn write_raw(
        &mut self,
        json: &str,
    ) -> Result<(), String> {
        if verbose() {
            println!("      tx[{:.3}]| {json}", now_s());
        }
        self.port
            .write_all(format!("{json}\n").as_bytes())
            .map_err(|e| format!("serial write failed: {e}"))?;
        self.port.flush().map_err(|e| e.to_string())
    }

    /// Wait until the firmware acknowledges the command just sent.
    ///
    /// This is the crux of talking to this device: `usb_command_task` is a
    /// single task that, for any reply-bearing command, takes
    /// `COMMAND_QUERY_LOCK` and blocks for up to a second waiting on the reply
    /// bus. While it is blocked it is *not* draining the USB Serial/JTAG RX
    /// FIFO, which is only 64 bytes. Pipelining commands therefore overflows
    /// that FIFO and the firmware loses whole lines -- observed as
    /// "[USB] unrecognized command" and as replies that never arrive.
    ///
    /// So: send one command, wait for its acknowledgement, then send the next.
    /// Reply-bearing commands answer with `[USB_REPLY]`; the rest print
    /// "[USB] command accepted".
    fn settle(&mut self) -> bool {
        let deadline = Instant::now() + Duration::from_millis(1_500);
        while Instant::now() < deadline {
            for line in self.read_lines() {
                if line.contains("[USB_REPLY]") || line.contains("command accepted") {
                    return true;
                }
            }
        }
        false
    }

    /// Issue a read command and return its `[USB_REPLY]` payload, retrying if
    /// the command is swallowed.
    ///
    /// The firmware drops serial input whenever `usb_command_task` is busy, so
    /// the channel is genuinely lossy and host-side pacing alone cannot make it
    /// reliable -- measured at roughly a 50% per-run failure rate even with a
    /// 400 ms inter-command gap. A lost command produces no reply and no error,
    /// so the only robust answer is to ask again.
    ///
    /// Note `read_encoders` is not idempotent: it advances the query baseline.
    /// A retry after a *lost command* is safe (nothing was consumed), but a
    /// retry after a *lost reply* forfeits one interval's delta. That is rare,
    /// and it surfaces as a loud failure in the delta-sum scenario rather than
    /// a silently wrong number.
    fn query(
        &mut self,
        command: &str,
        reply_type: &str,
    ) -> Result<serde_json::Value, String> {
        let mut last = String::new();
        for attempt in 1..=QUERY_ATTEMPTS {
            match self.query_once(command, reply_type) {
                Ok(v) => return Ok(v),
                Err(e) => {
                    if verbose() {
                        println!("      !! attempt {attempt}/{QUERY_ATTEMPTS} failed: {e}");
                    }
                    last = e;
                }
            }
        }
        Err(format!("after {QUERY_ATTEMPTS} attempts: {last}"))
    }

    fn query_once(
        &mut self,
        command: &str,
        reply_type: &str,
    ) -> Result<serde_json::Value, String> {
        // Drop anything buffered so an earlier reply cannot be mistaken for
        // this one.
        let _ = self.read_lines();
        self.write_line(&format!(r#"{{"ct":"i","ic":"{command}"}}"#))?;

        let t0 = Instant::now();
        let deadline = t0 + REPLY_TIMEOUT;
        let mut seen = 0usize;
        while Instant::now() < deadline {
            for line in self.read_lines() {
                seen += 1;
                if !line.contains("[USB_REPLY]") {
                    continue;
                }
                let start = match line.find('{') {
                    Some(i) => i,
                    None => continue,
                };
                let value: serde_json::Value = match serde_json::from_str(&line[start..]) {
                    Ok(v) => v,
                    Err(_) => continue,
                };
                if value.get("type").and_then(|t| t.as_str()) == Some(reply_type) {
                    if value.get("status").and_then(|s| s.as_str()) != Some("ok") {
                        return Err(format!("{command} replied non-ok: {value}"));
                    }
                    return Ok(value);
                }
            }
        }
        Err(format!(
            "timed out after {:.2}s ({seen} lines seen) waiting for a {reply_type:?} reply to {command}",
            t0.elapsed().as_secs_f32()
        ))
    }

    fn encoders(&mut self) -> Result<EncoderSample, String> {
        let v = self.query("read_encoders", "encoders")?;
        let counts = i32_triple(&v, "counts").ok_or("encoder reply missing counts")?;
        let delta = i32_triple(&v, "delta_counts").ok_or("encoder reply missing delta_counts")?;
        Ok(EncoderSample { counts, delta })
    }

    /// Enable, hold a translate command for `cycles` refreshes, and return the
    /// encoder delta accumulated over the burst.
    fn drive_and_measure(
        &mut self,
        direction_deg: f32,
        speed_norm: f32,
        cycles: usize,
        enable: bool,
    ) -> Result<[i32; 3], String> {
        // `enable` is a parameter because the e-stop scenario must *not* send
        // it: `enable` legitimately clears the latch, so enabling here would
        // make a working e-stop look broken.
        if enable {
            self.send_json(r#"{"ct":"i","ic":"enable"}"#)?;
        }

        let cmd = format!(
            r#"{{"ct":"i","ic":"t","direction_deg":{direction_deg},"speed_norm":{speed_norm}}}"#
        );

        // Spin-up cycle, measured but discarded. These motors have real
        // stiction: the first interval after a standstill frequently reads near
        // zero on one wheel, which is a property of the drivetrain rather than
        // a fault, and it made a 3-cycle burst flaky.
        self.send_json(&cmd)?;
        std::thread::sleep(MOTION_REFRESH);
        self.encoders()?; // also rebases the query baseline

        let mut total = [0i32; 3];
        for _ in 0..cycles {
            self.send_json(&cmd)?;
            std::thread::sleep(MOTION_REFRESH);
            let s = self.encoders()?;
            for (acc, d) in total.iter_mut().zip(s.delta) {
                *acc += d;
            }
        }
        self.make_safe();
        Ok(total)
    }

    /// Zero the command and drop the enable gate. Best-effort: used on the
    /// failure path too, so errors are intentionally swallowed.
    fn make_safe(&mut self) {
        let _ = self.send_json(r#"{"ct":"i","ic":"t","direction_deg":0,"speed_norm":0.0}"#);
        let _ = self.send_json(r#"{"ct":"i","ic":"disable"}"#);
        // After the wheels are stopped, not before: stagger is a refinement,
        // stopping is the safety property.
        self.reset_stagger();
        std::thread::sleep(Duration::from_millis(150));
        let _ = self.read_lines();
    }

    /// Put `pwm_stagger` back to off if a mode switched it on. Clears the flag
    /// on any definite answer; on silence it stays armed so the next
    /// `make_safe` (or `Drop`) tries again.
    fn reset_stagger(&mut self) {
        if !self.stagger_armed {
            return;
        }
        match self.send_optional(&drive_loop_json(None, Some(false))) {
            Ack::Accepted | Ack::Unsupported => self.stagger_armed = false,
            Ack::Silent => {
                println!("      !! pwm_stagger=false never acknowledged; stagger may still be on")
            }
        }
    }

    /// Send a command the flashed firmware may not know yet, and report which.
    ///
    /// Unlike [`Bot::send_json`] this distinguishes "unrecognized" from
    /// silence, so a missing `drive_loop` costs two round trips rather than
    /// the full retry ladder followed by a hard failure.
    fn send_optional(
        &mut self,
        json: &str,
    ) -> Ack {
        let mut unrecognized = 0;
        for _ in 0..QUERY_ATTEMPTS {
            if self.write_line(json).is_err() {
                return Ack::Silent;
            }
            let deadline = Instant::now() + Duration::from_millis(1_500);
            'wait: while Instant::now() < deadline {
                for line in self.read_lines() {
                    if line.contains("command accepted") {
                        return Ack::Accepted;
                    }
                    if line.contains("unrecognized command") {
                        unrecognized += 1;
                        break 'wait;
                    }
                }
            }
            // One "unrecognized" can be a line mangled by a dropped byte (the
            // channel is lossy), so only conclude the command is missing once
            // the firmware has said so twice.
            if unrecognized >= 2 {
                return Ack::Unsupported;
            }
        }
        Ack::Silent
    }

    /// Hold a motion command for `window`, re-asserting it inside the
    /// firmware deadman, and sample absolute encoder counts about every
    /// `sample_every`. Always sends `stop` afterwards, including when the loop
    /// fails part-way: the deadman is the backstop, not the plan.
    ///
    /// The existing [`Bot::drive_and_measure`] cannot be reused for this. Its
    /// cycle is `send_json` + [`MOTION_REFRESH`] + a query, which with
    /// [`COMMAND_GAP`] on every write spaces motion commands roughly a second
    /// apart -- so the 300 ms deadman zeroes the wheels for most of each
    /// cycle. That is tolerable for its sign/sum checks, fatal for a speed
    /// measurement.
    fn hold_and_sample(
        &mut self,
        cmd: &str,
        stop: &str,
        window: Duration,
        sample_every: Duration,
    ) -> Result<Hold, String> {
        // Stale "command accepted" lines would otherwise count as keepalive
        // acknowledgements.
        let _ = self.read_lines();
        let held = self.hold_inner(cmd, window, sample_every);
        // Stop immediately, unpaced, then again through the acknowledged
        // path. `send_json` alone waits out COMMAND_GAP first, which lands the
        // stop just past the deadman, so every window ended in a watchdog trip
        // (harmless, but a firmware warning per step).
        let _ = self.write_raw(stop);
        let stopped = self.send_json(stop);
        let hold = held?;
        stopped?;
        Ok(hold)
    }

    fn hold_inner(
        &mut self,
        cmd: &str,
        window: Duration,
        sample_every: Duration,
    ) -> Result<Hold, String> {
        // Event loop rather than send/wait/send: a keepalive must be able to
        // go out while a sample is still in flight. That is one line (<64
        // bytes) queued behind the pending query, which the RX FIFO holds;
        // what it must never do is stack two unacknowledged keepalives.
        let t0 = Instant::now();
        let end = t0 + window;
        let mut hold = Hold {
            samples: Vec::new(),
            worst_keepalive_gap: Duration::ZERO,
            kicks_sent: 0,
            kicks_acked: 0,
            kicks_coalesced: 0,
            queries_sent: 0,
        };
        let mut next_kick = t0;
        let mut next_query = t0;
        let mut kick_pending: Option<Instant> = None;
        let mut query_pending: Option<Instant> = None;
        let mut last_write: Option<Instant> = None;
        let mut last_ack: Option<Instant> = None;

        while Instant::now() < end {
            if interrupted() {
                return Err("interrupted".into());
            }
            let now = Instant::now();
            let quiet = last_write.is_none_or(|w| now - w >= TICK_GAP);
            if quiet && kick_pending.is_none() && now >= next_kick {
                self.write_raw(cmd)?;
                hold.kicks_sent += 1;
                kick_pending = Some(now);
                last_write = Some(now);
                next_kick = now + KEEPALIVE_PERIOD;
            }
            else if quiet
                && kick_pending.is_none()
                && query_pending.is_none()
                && now >= next_query
                && next_kick.saturating_duration_since(now) >= QUERY_SLOT
            {
                self.write_raw(r#"{"ct":"i","ic":"read_encoders"}"#)?;
                hold.queries_sent += 1;
                query_pending = Some(now);
                last_write = Some(now);
                next_query = now + sample_every;
            }

            // Doubles as the loop's pacing: blocks up to the port's 20 ms
            // read timeout when the board is quiet. Samples are timestamped on
            // arrival, so that poll interval (and the firmware's own 15 ms RX
            // poll) is the timing jitter; rates are fitted over several
            // samples rather than diffed pairwise to average it out.
            for line in self.read_lines() {
                let at = Instant::now();
                if line.contains("command accepted") {
                    if kick_pending.take().is_some() {
                        if let Some(prev) = last_ack {
                            hold.worst_keepalive_gap = hold.worst_keepalive_gap.max(at - prev);
                        }
                        last_ack = Some(at);
                        hold.kicks_acked += 1;
                    }
                }
                else if line.contains("motion bus full") {
                    // Dropped by the firmware's coalescer: re-send promptly.
                    hold.kicks_coalesced += 1;
                    kick_pending = None;
                    next_kick = at;
                }
                else if line.contains("[USB_REPLY]") {
                    if let Some(counts) = parse_encoder_reply(&line) {
                        hold.samples.push(Sample {
                            t_s: (at - t0).as_secs_f64(),
                            counts,
                        });
                    }
                    query_pending = None;
                }
            }

            let now = Instant::now();
            // A keepalive queued behind an in-flight query is not lost, just
            // late; re-sending it then would stack two lines in the FIFO.
            if query_pending.is_none() && kick_pending.is_some_and(|k| now - k > KICK_ACK_WAIT) {
                kick_pending = None;
                next_kick = now;
            }
            if query_pending.is_some_and(|q| now - q > SAMPLE_REPLY_WAIT) {
                query_pending = None;
            }
        }

        hold.worst_keepalive_gap = match last_ack {
            Some(prev) => hold
                .worst_keepalive_gap
                .max(end.saturating_duration_since(prev)),
            // Never acknowledged: as far as we know it never drove at all.
            None => window,
        };
        Ok(hold)
    }

    /// Idle for `duration` with the wheels stopped, draining the port so
    /// stale lines do not leak into the next held window, and honouring
    /// Ctrl-C.
    fn rest(
        &mut self,
        duration: Duration,
    ) -> Check {
        let end = Instant::now() + duration;
        while Instant::now() < end {
            if interrupted() {
                return Err("interrupted".into());
            }
            let _ = self.read_lines(); // ~20 ms per call while idle
        }
        Ok(())
    }

    /// Non-blocking-ish drain of whole lines, with ANSI colour codes stripped
    /// (the firmware logger emits them around every level-tagged line).
    fn read_lines(&mut self) -> Vec<String> {
        // Drain everything currently available, not just one chunk. A single
        // read per call let the port stay backlogged: the caller's deadline
        // could expire with the reply already sitting in the OS buffer.
        let mut buf = [0u8; 4096];
        loop {
            match self.port.read(&mut buf) {
                Ok(n) if n > 0 => {
                    self.pending.push_str(&String::from_utf8_lossy(&buf[..n]));
                    // A short read means the device has nothing more queued.
                    if n < buf.len() {
                        break;
                    }
                }
                _ => break,
            }
        }
        let mut out = Vec::new();
        while let Some(idx) = self.pending.find('\n') {
            let line: String = self.pending.drain(..=idx).collect();
            let line = strip_ansi(line.trim_end());
            if verbose() && !line.is_empty() {
                println!("      rx[{:.3}]| {line}", now_s());
            }
            out.push(line);
        }
        out
    }
}

impl Drop for Bot {
    /// The wheels must not be left driving because the harness panicked or the
    /// operator hit Ctrl-C between scenarios.
    fn drop(&mut self) { self.make_safe(); }
}

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------

fn require_line(
    log: &[String],
    needle: &str,
) -> Check {
    if log.iter().any(|l| l.contains(needle)) {
        Ok(())
    }
    else {
        Err(format!("boot log missing {needle:?}"))
    }
}

fn strip_ansi(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    let mut chars = s.chars();
    while let Some(c) = chars.next() {
        if c != '\x1b' {
            out.push(c);
            continue;
        }
        // Skip CSI ... final-byte. Terminators for SGR are in @..~.
        for skip in chars.by_ref() {
            if skip.is_ascii_alphabetic() {
                break;
            }
        }
    }
    out
}

fn f64_at(
    value: &serde_json::Value,
    path: &[&str],
) -> Option<f64> {
    let mut cur = value;
    for key in path {
        cur = cur.get(key)?;
    }
    cur.as_f64()
}

fn i32_triple(
    value: &serde_json::Value,
    key: &str,
) -> Option<[i32; 3]> {
    let arr = value.get("data")?.get(key)?.as_array()?;
    if arr.len() != 3 {
        return None;
    }
    let mut out = [0i32; 3];
    for (slot, v) in out.iter_mut().zip(arr) {
        *slot = v.as_i64()? as i32;
    }
    Some(out)
}

/// Parse an encoder `[USB_REPLY]` line into absolute counts. `None` for any
/// other reply (including the firmware's "timeout waiting for reply").
fn parse_encoder_reply(line: &str) -> Option<[i32; 3]> {
    let value: serde_json::Value = serde_json::from_str(&line[line.find('{')?..]).ok()?;
    if value.get("type")?.as_str()? != "encoders" || value.get("status")?.as_str()? != "ok" {
        return None;
    }
    i32_triple(&value, "counts")
}

fn counts_to_rad_s(counts_per_s: f64) -> f64 {
    counts_per_s / COUNTS_PER_REV * std::f64::consts::TAU
}

/// Least-squares `(slope, intercept)`. `None` with fewer than two points or
/// no spread in x.
fn linear_fit(points: &[(f64, f64)]) -> Option<(f64, f64)> {
    if points.len() < 2 {
        return None;
    }
    let n = points.len() as f64;
    let mean_x = points.iter().map(|p| p.0).sum::<f64>() / n;
    let mean_y = points.iter().map(|p| p.1).sum::<f64>() / n;
    let sxx: f64 = points.iter().map(|p| (p.0 - mean_x).powi(2)).sum();
    if sxx <= f64::EPSILON {
        return None;
    }
    let sxy: f64 = points.iter().map(|p| (p.0 - mean_x) * (p.1 - mean_y)).sum();
    let slope = sxy / sxx;
    Some((slope, mean_y - slope * mean_x))
}

/// Raw counts/s for `wheel` over samples with `from_s <= t < to_s`, as the
/// slope of a fit rather than an end-to-end diff, so one late-timestamped
/// sample cannot swing the result. Counts are taken relative to the first
/// sample with wrapping arithmetic, so an i32 rollover mid-window is harmless.
fn rate_in_window(
    samples: &[Sample],
    wheel: usize,
    from_s: f64,
    to_s: f64,
) -> Option<f64> {
    let inside: Vec<&Sample> = samples
        .iter()
        .filter(|s| s.t_s >= from_s && s.t_s < to_s)
        .collect();
    let base = inside.first()?.counts[wheel];
    let points: Vec<(f64, f64)> = inside
        .iter()
        .map(|s| (s.t_s, f64::from(s.counts[wheel].wrapping_sub(base))))
        .collect();
    linear_fit(&points).map(|(slope, _)| slope)
}

/// Lowest duty from which the wheel keeps turning all the way to the top of
/// the sweep, i.e. |speed| stays above `fraction` of the maximum. `points` are
/// `(duty, speed)` ascending by duty.
///
/// Walks down from the top and stops at the first point that is not moving,
/// so a lone blip below breakaway (coast from the previous step, a stray
/// count) is not mistaken for sustained motion.
fn breakaway_duty(
    points: &[(f64, f64)],
    fraction: f64,
) -> Option<f64> {
    let max = points.iter().map(|p| p.1.abs()).fold(0.0, f64::max);
    if max < MOVING_FLOOR_CPS {
        return None;
    }
    let threshold = max * fraction;
    let mut breakaway = None;
    for &(duty, speed) in points.iter().rev() {
        if speed.abs() <= threshold {
            break;
        }
        breakaway = Some(duty);
    }
    breakaway
}

/// Percent speed lost from `first` to `last` (positive = slowed down), on
/// magnitudes so it reads the same for either rotation sense. `None` if the
/// wheel was not moving to begin with.
fn decay_percent(
    first: f64,
    last: f64,
) -> Option<f64> {
    if first.abs() < f64::EPSILON {
        return None;
    }
    Some((first.abs() - last.abs()) / first.abs() * 100.0)
}

/// Trim for the faster wheel so that, at a commanded `reference` duty, it
/// runs at the slower wheel's speed. The trim multiplies the command, so solve
/// `m_f * (reference * trim) + c_f = m_s * reference + c_s` on the two fits,
/// which respects the intercepts (speed is not proportional to duty). Exact at
/// `reference`, close either side of it. Clamped to [0.5, 1.0]: anything
/// outside is a measurement problem, not a trim. `None` without a usable
/// faster-wheel slope.
fn suggest_trim(
    fast_line: (f64, f64),
    slow_line: (f64, f64),
    reference: f64,
) -> Option<f64> {
    let (m_fast, c_fast) = fast_line;
    if m_fast <= f64::EPSILON || reference <= f64::EPSILON {
        return None;
    }
    let slow_speed = slow_line.0 * reference + slow_line.1;
    let duty = (slow_speed - c_fast) / m_fast;
    Some((duty / reference).clamp(0.5, 1.0))
}

/// Worst breakaway plus [`MIN_FORWARD_MARGIN`], rounded up to 0.01.
fn suggest_min_forward(breakaways: &[Option<f64>]) -> Option<f64> {
    let worst = breakaways.iter().flatten().copied().reduce(f64::max)?;
    // The epsilon keeps float noise (0.45 + 0.05 = 0.5000000000000001) from
    // rounding up a whole step.
    Some((((worst + MIN_FORWARD_MARGIN) * 100.0 - 1e-6).ceil() / 100.0).min(1.0))
}

fn mean(values: &[Option<f64>]) -> Option<f64> {
    let present: Vec<f64> = values.iter().flatten().copied().collect();
    (!present.is_empty()).then(|| present.iter().sum::<f64>() / present.len() as f64)
}

fn fmt_opt(
    value: Option<f64>,
    width: usize,
    precision: usize,
) -> String {
    match value {
        Some(v) => format!("{v:>width$.precision$}"),
        None => format!("{:>width$}", "-"),
    }
}

/// `--csv` path for a mode: as given for a single mode, with the mode name
/// inserted before `.csv` when both modes share one flag.
fn mode_csv_path(
    base: &str,
    mode: &str,
    both: bool,
) -> String {
    if !both {
        return base.to_string();
    }
    match base.strip_suffix(".csv") {
        Some(stem) => format!("{stem}.{mode}.csv"),
        None => format!("{base}.{mode}.csv"),
    }
}

/// Best-effort: the data has already been printed, so a bad path should not
/// turn a good measurement into a failed one.
fn write_csv(
    path: &str,
    contents: &str,
) {
    match std::fs::write(path, contents) {
        Ok(()) => println!("    wrote {path}"),
        Err(e) => println!("    !! could not write {path}: {e}"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn strip_ansi_removes_colour_codes() {
        assert_eq!(
            strip_ansi("\x1b[32mINFO - board wiring: ok\x1b[0m"),
            "INFO - board wiring: ok"
        );
        assert_eq!(strip_ansi("plain"), "plain");
    }

    #[test]
    fn i32_triple_reads_encoder_shape() {
        let v = serde_json::json!({
            "type": "encoders",
            "status": "ok",
            "data": { "counts": [1, -2, 0], "delta_counts": [4, 5, 6] }
        });
        assert_eq!(i32_triple(&v, "counts"), Some([1, -2, 0]));
        assert_eq!(i32_triple(&v, "delta_counts"), Some([4, 5, 6]));
        assert_eq!(i32_triple(&v, "missing"), None);
    }

    #[test]
    fn f64_at_walks_nested_paths() {
        let v = serde_json::json!({ "data": { "accel": { "z": 0.99 } } });
        assert_eq!(f64_at(&v, &["data", "accel", "z"]), Some(0.99));
        assert_eq!(f64_at(&v, &["data", "nope"]), None);
    }

    #[test]
    fn arg_value_reads_flag_pairs() {
        let args = vec!["--port".to_string(), "/dev/ttyACM1".to_string()];
        assert_eq!(arg_value(&args, "--port"), Some("/dev/ttyACM1".to_string()));
        assert_eq!(arg_value(&args, "--baud"), None);
    }

    fn close(
        a: f64,
        b: f64,
    ) -> bool {
        (a - b).abs() < 1e-6
    }

    #[test]
    fn linear_fit_recovers_an_exact_line() {
        let points: Vec<(f64, f64)> = (0..5)
            .map(|i| (f64::from(i), 3.0 * f64::from(i) - 2.0))
            .collect();
        let (m, c) = linear_fit(&points).unwrap();
        assert!(close(m, 3.0) && close(c, -2.0), "got {m} {c}");
    }

    #[test]
    fn linear_fit_rejects_degenerate_input() {
        assert_eq!(linear_fit(&[]), None);
        assert_eq!(linear_fit(&[(1.0, 2.0)]), None);
        // No spread in x: the slope is undefined, not infinite.
        assert_eq!(linear_fit(&[(0.5, 1.0), (0.5, 9.0)]), None);
    }

    fn ramp(
        cps: [f64; 2],
        times: &[f64],
    ) -> Vec<Sample> {
        times
            .iter()
            .map(|&t| Sample {
                t_s: t,
                counts: [(cps[0] * t) as i32, (cps[1] * t) as i32, 0],
            })
            .collect()
    }

    #[test]
    fn rate_in_window_fits_only_the_window() {
        // W2 counts down, as on the bench unit. A spin-up sample at 0.1 s that
        // is far off the line must be excluded by the window.
        let mut samples = ramp([3_000.0, -3_000.0], &[0.3, 0.45, 0.6, 0.75, 0.9]);
        samples.insert(
            0,
            Sample {
                t_s: 0.1,
                counts: [0, 0, 0],
            },
        );
        let w1 = rate_in_window(&samples, 0, 0.3, f64::INFINITY).unwrap();
        let w2 = rate_in_window(&samples, 1, 0.3, f64::INFINITY).unwrap();
        assert!((w1 - 3_000.0).abs() < 5.0, "{w1}");
        assert!((w2 + 3_000.0).abs() < 5.0, "{w2}");
        // Folding in the encoder sign reports both in drive sense.
        assert!(w2 * ENCODER_SIGN[1] > 0.0);
        assert_eq!(rate_in_window(&samples, 0, 2.0, 3.0), None);
    }

    #[test]
    fn rate_in_window_survives_counter_rollover() {
        let samples: Vec<Sample> = [0.0, 0.25, 0.5]
            .iter()
            .enumerate()
            .map(|(i, &t)| Sample {
                t_s: t,
                counts: [(i32::MAX - 500).wrapping_add(i as i32 * 750), 0, 0],
            })
            .collect();
        let rate = rate_in_window(&samples, 0, 0.0, 1.0).unwrap();
        assert!((rate - 3_000.0).abs() < 1e-6, "{rate}");
    }

    #[test]
    fn breakaway_is_the_start_of_sustained_motion() {
        // A blip at 0.15 (coast from the previous step) below a dead band
        // must not count: breakaway is where motion becomes sustained.
        let points = [
            (0.10, 0.0),
            (0.15, 400.0),
            (0.20, 0.0),
            (0.25, 20.0),
            (0.30, 900.0),
            (0.35, 1_500.0),
            (1.00, 4_800.0),
        ];
        assert_eq!(breakaway_duty(&points, 0.05), Some(0.30));
    }

    #[test]
    fn breakaway_absent_for_a_dead_wheel() {
        assert_eq!(breakaway_duty(&[(0.5, 0.0), (1.0, 3.0)], 0.05), None);
        assert_eq!(breakaway_duty(&[], 0.05), None);
    }

    #[test]
    fn analyse_wheel_fits_above_breakaway_only() {
        // Linear 5000*d - 500 from 0.3 up; stalled below. Monotonic, so the
        // region runs to the top of the sweep.
        let mut points = vec![(0.1, 0.0), (0.2, 0.0)];
        points.extend(
            [0.3, 0.5, 0.7, 1.0]
                .iter()
                .map(|&d| (d, 5_000.0 * d - 500.0)),
        );
        let fit = analyse_wheel(&points);
        assert_eq!(fit.breakaway, Some(0.3));
        let (m, c) = fit.line.unwrap();
        assert!(close(m, 5_000.0) && close(c, -500.0), "got {m} {c}");
        assert_eq!(fit.full, Some(4_500.0));
        assert_eq!(fit.ceiling, Some(1.0));
        assert_eq!(fit.ceiling_reason, CeilingReason::SweepTop);
        assert!(fit.drops.is_empty());
    }

    type Sweep = Vec<(f64, f64)>;

    /// The real v1-alpha sweep from 0.80 up (reported from hardware), with a
    /// plausible linear-ish region below it. W1 collapses at 1.00; W2 also
    /// jumps into the bistable band at 0.85 and plateaus.
    fn v1_alpha_sweep() -> (Sweep, Sweep) {
        let duties = [
            0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95,
            1.00,
        ];
        let w1 = [
            0.0, 0.0, 900.0, 1_300.0, 1_750.0, 2_200.0, 2_620.0, 2_950.0, 3_300.0, 3_500.0,
            3_743.0, 4_196.0, 4_765.0, 5_082.0, 3_059.0,
        ];
        let w2 = [
            0.0, 0.0, 800.0, 1_250.0, 1_700.0, 2_100.0, 2_500.0, 2_850.0, 3_150.0, 3_400.0,
            3_658.0, 4_878.0, 4_816.0, 4_804.0, 2_838.0,
        ];
        let zip = |v: &[f64]| duties.iter().copied().zip(v.iter().copied()).collect();
        (zip(&w1), zip(&w2))
    }

    #[test]
    fn speed_drops_flag_the_collapse_at_full_duty() {
        let (w1, w2) = v1_alpha_sweep();
        let drops = speed_drops(&w1, NONMONOTONIC_DROP);
        assert_eq!(drops.len(), 1, "{drops:?}");
        assert!(close(drops[0].duty, 1.0) && close(drops[0].best_duty, 0.95));
        // W2's 0.90/0.95 sag is ~1%, well inside the 10% band; only 1.00 drops.
        let drops = speed_drops(&w2, NONMONOTONIC_DROP);
        assert_eq!(drops.len(), 1, "{drops:?}");
        assert!(close(drops[0].duty, 1.0) && close(drops[0].best_duty, 0.85));
    }

    #[test]
    fn bistable_jump_finds_the_step_into_the_fast_mode() {
        let (w1, w2) = v1_alpha_sweep();
        let region = |pts: &[(f64, f64)]| -> Vec<(f64, f64)> {
            pts.iter()
                .copied()
                .filter(|&(d, _)| (0.40..=0.95).contains(&d))
                .collect()
        };
        assert_eq!(
            bistable_jump(&region(&w2), BISTABLE_JUMP_FACTOR),
            Some(0.85)
        );
        assert_eq!(bistable_jump(&region(&w1), BISTABLE_JUMP_FACTOR), None);
        // Too few steps to call anything.
        assert_eq!(
            bistable_jump(&[(0.4, 0.0), (0.5, 1.0), (0.6, 9.0)], 2.0),
            None
        );
    }

    /// Real W1 sweep from the bench unit (2026-10-23). Noisy but monotonic up
    /// to 0.95; a 2x jump factor wrongly cut it at 0.55.
    #[test]
    fn bistable_jump_ignores_ordinary_step_noise() {
        let duties = [0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95];
        let w1 = [
            1_254.0, 1_960.0, 1_879.0, 2_122.0, 2_795.0, 3_226.0, 3_547.0, 3_686.0, 3_833.0,
            4_251.0, 4_906.0, 5_238.0,
        ];
        let region: Vec<(f64, f64)> = duties.iter().copied().zip(w1.iter().copied()).collect();
        assert_eq!(bistable_jump(&region, BISTABLE_JUMP_FACTOR), None);
    }

    #[test]
    fn analyse_wheel_stops_below_drops_and_jumps() {
        let (w1, w2) = v1_alpha_sweep();
        let f1 = analyse_wheel(&w1);
        assert_eq!(f1.breakaway, Some(0.40));
        assert_eq!(f1.ceiling, Some(0.95));
        assert_eq!(f1.ceiling_reason, CeilingReason::BeforeDrop(1.0));
        assert_eq!(f1.ceiling_speed, Some(5_082.0));

        let f2 = analyse_wheel(&w2);
        assert_eq!(f2.breakaway, Some(0.40));
        assert_eq!(f2.ceiling, Some(0.80));
        assert_eq!(f2.ceiling_reason, CeilingReason::BeforeJump(0.85));
        assert_eq!(f2.ceiling_speed, Some(3_658.0));
        // The fit never sees the collapsed 1.00 point: a positive slope near
        // the region's mean gain, not dragged down to the 29.7 rad/s nonsense.
        let (m, _) = f2.line.unwrap();
        assert!((6_000.0..8_000.0).contains(&m), "{m}");

        // The pair's ceiling is the lower one (0.80); max_wheel_rad_s is the
        // slower wheel there: 3658 c/s = 38.3 rad/s, not the 1.00 point's 29.7.
        let pair_top = f1.ceiling.unwrap().min(f2.ceiling.unwrap());
        let at = |pts: &[(f64, f64)]| pts.iter().find(|p| close(p.0, pair_top)).unwrap().1;
        let max = counts_to_rad_s(at(&w1).min(at(&w2)));
        assert!((max - 38.3).abs() < 0.05, "{max}");

        // Trim at the mid reference duty stays near 1, unlike the 0.639 the
        // full-duty ratio produced.
        let (l1, l2) = (f1.line.unwrap(), f2.line.unwrap());
        let (fast, slow) = if l1.0 * TRIM_REF_DUTY + l1.1 >= l2.0 * TRIM_REF_DUTY + l2.1 {
            (l1, l2)
        }
        else {
            (l2, l1)
        };
        let trim = suggest_trim(fast, slow, TRIM_REF_DUTY).unwrap();
        assert!((0.9..=1.0).contains(&trim), "{trim}");
    }

    #[test]
    fn speed_drops_ignore_a_stalled_start() {
        // Zeros before breakaway are not a drop from anything.
        assert!(speed_drops(&[(0.1, 0.0), (0.2, 0.0), (0.3, 500.0)], 0.1).is_empty());
        // A blip below the moving floor is not a best speed to fall from.
        assert!(speed_drops(&[(0.1, 30.0), (0.2, 0.0)], 0.1).is_empty());
    }

    #[test]
    fn decay_percent_is_loss_on_magnitude() {
        assert!(close(decay_percent(100.0, 90.0).unwrap(), 10.0));
        assert!(close(decay_percent(-100.0, -80.0).unwrap(), 20.0));
        // Speeding up is negative decay, not an error.
        assert!(close(decay_percent(100.0, 110.0).unwrap(), -10.0));
        assert_eq!(decay_percent(0.0, 50.0), None);
    }

    #[test]
    fn suggest_trim_matches_the_slower_wheel_at_the_reference() {
        // Fast 5000*d - 200, slow 4500*d - 200. At reference 0.6 the slow
        // wheel does 2500; the fast one needs duty 0.54, i.e. trim 0.9.
        let trim = suggest_trim((5_000.0, -200.0), (4_500.0, -200.0), 0.6).unwrap();
        assert!(close(trim, 0.9), "{trim}");
        // Intercepts matter: same slopes, fast offset by +300 c/s.
        let trim = suggest_trim((5_000.0, 100.0), (5_000.0, -200.0), 0.6).unwrap();
        assert!(close(trim, 0.9), "{trim}");
        // Absurd inputs clamp rather than produce a runaway trim.
        assert!(close(
            suggest_trim((5_000.0, 0.0), (500.0, 0.0), 0.6).unwrap(),
            0.5
        ));
        assert!(close(
            suggest_trim((5_000.0, 0.0), (6_000.0, 0.0), 0.6).unwrap(),
            1.0
        ));
        // No usable slope: no trim.
        assert_eq!(suggest_trim((0.0, 100.0), (5_000.0, 0.0), 0.6), None);
    }

    #[test]
    fn suggest_min_forward_takes_worst_breakaway_plus_margin() {
        assert_eq!(suggest_min_forward(&[Some(0.45), Some(0.30)]), Some(0.50));
        assert_eq!(suggest_min_forward(&[None, Some(0.32)]), Some(0.37));
        assert_eq!(suggest_min_forward(&[Some(0.98)]), Some(1.0));
        assert_eq!(suggest_min_forward(&[None, None]), None);
    }

    #[test]
    fn counts_to_rad_s_uses_600_counts_per_rev() {
        assert!(close(counts_to_rad_s(600.0), std::f64::consts::TAU));
        // The profile's measured anchor: 4769 c/s is ~49.9 rad/s.
        assert!((counts_to_rad_s(4_769.0) - 49.94).abs() < 0.01);
    }

    #[test]
    fn characterize_duties_cover_the_sweep() {
        let duties = characterize_duties();
        assert_eq!(duties.len(), 22);
        assert!(close(duties[0], 0.10));
        assert!(close(duties[18], 1.00));
        assert_eq!(&duties[19..], &[-0.3, -0.6, -1.0]);
    }

    #[test]
    fn command_builders_emit_firmware_json() {
        assert_eq!(
            probe_cmd(1, 0.15000000000000002),
            r#"{"ct":"i","ic":"probe_speed","wheel":1,"speed_norm":0.15}"#
        );
        assert_eq!(
            drive_loop_json(None, Some(true)),
            r#"{"ct":"i","ic":"drive_loop","pwm_stagger":true}"#
        );
        assert_eq!(
            drive_loop_json(Some(false), Some(false)),
            r#"{"ct":"i","ic":"drive_loop","closed_loop":false,"pwm_stagger":false}"#
        );
        for json in [
            probe_cmd(0, -1.0),
            forward_cmd(0.8),
            drive_loop_json(Some(true), None),
        ] {
            assert!(
                serde_json::from_str::<serde_json::Value>(&json).is_ok(),
                "{json}"
            );
            // One line must fit the firmware's 64-byte RX FIFO, since a
            // keepalive can queue behind an in-flight sample.
            assert!(json.len() < 64, "{json} is {} bytes", json.len());
        }
    }

    #[test]
    fn parse_encoder_reply_reads_counts_only_from_encoder_replies() {
        let line = r#"[USB_REPLY] {"data":{"counts":[10,-20,0],"delta_counts":[1,2,3]},"status":"ok","type":"encoders"}"#;
        assert_eq!(parse_encoder_reply(line), Some([10, -20, 0]));
        assert_eq!(
            parse_encoder_reply("[USB_REPLY] timeout waiting for reply"),
            None
        );
        assert_eq!(
            parse_encoder_reply(
                r#"[USB_REPLY] {"data":{"temp_c":21.0},"status":"ok","type":"env"}"#
            ),
            None
        );
    }

    #[test]
    fn interpret_soak_calls_combined_load_and_stagger() {
        let all = [1.0, 0.5, 15.0, 15.0, 4.0, 4.0, 14.0, 14.0];
        let lines = interpret_soak(
            Some(1.0),
            Some(15.0),
            Some(4.0),
            Some(14.0),
            Some(8.0),
            &all,
        );
        assert!(lines[0].contains("supply sag"), "{lines:?}");
        assert!(lines[1].contains("interleaving helps"), "{lines:?}");
        assert!(lines[2].contains("did not fully"), "{lines:?}");
        assert_eq!(lines.len(), 3);

        let all = [9.0, 8.0, 10.0, 10.0, 10.0, 10.0];
        let heat = interpret_soak(Some(9.0), Some(10.0), None, Some(10.0), None, &all);
        assert!(heat[0].contains("per-motor heating"), "{heat:?}");
        assert!(heat[1].contains("not measured"), "{heat:?}");
        assert_eq!(heat.len(), 2);
    }

    #[test]
    fn interpret_soak_says_no_droop_inside_the_noise() {
        // The real hardware result once spin-up is skipped: flat everywhere.
        // No "warming up" or "heating" claims may appear.
        let all = [0.4, -1.2, 2.1, 1.8, -0.6, 0.9, 1.5, 2.4];
        let lines = interpret_soak(Some(0.4), Some(2.0), Some(0.2), Some(2.0), Some(-1.0), &all);
        assert_eq!(lines.len(), 1, "{lines:?}");
        assert!(
            lines[0].starts_with("no droop: all phases within 2.4%"),
            "{lines:?}"
        );
        assert!(!lines
            .iter()
            .any(|l| l.contains("warming") || l.contains("heating")));

        // A start difference alone is reported as a fact, not a mechanism.
        let all = [0.4, 1.0, 2.0, 2.0];
        let lines = interpret_soak(Some(0.4), Some(2.0), Some(1.0), Some(2.0), Some(-8.0), &all);
        assert_eq!(
            lines,
            vec!["control (e) started 8.0% faster than (c)".to_string()]
        );

        assert!(interpret_soak(None, None, None, None, None, &[])[0].contains("nothing"));
    }

    #[test]
    fn mean_ignores_missing_values() {
        assert_eq!(mean(&[Some(2.0), None, Some(4.0)]), Some(3.0));
        assert_eq!(mean(&[None, None]), None);
    }

    #[test]
    fn mode_csv_path_splits_only_when_both_modes_run() {
        assert_eq!(mode_csv_path("run.csv", "soak", false), "run.csv");
        assert_eq!(mode_csv_path("run.csv", "soak", true), "run.soak.csv");
        assert_eq!(
            mode_csv_path("run", "characterize", true),
            "run.characterize.csv"
        );
    }

    #[test]
    fn hold_flags_gaps_past_the_deadman() {
        let hold = |ms| Hold {
            samples: Vec::new(),
            worst_keepalive_gap: Duration::from_millis(ms),
            kicks_sent: 0,
            kicks_acked: 0,
            kicks_coalesced: 0,
            queries_sent: 0,
        };
        assert!(!hold(180).deadman_risk());
        assert!(hold(320).deadman_risk());
    }
}
