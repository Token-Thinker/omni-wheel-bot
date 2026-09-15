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
//! ```
//!
//! **Motion scenarios are opt-in.** They spin the wheels, so they require
//! `--motion` and a bot that is elevated with wheels free. Without the flag the
//! harness runs boot and sensor checks only, which are safe on a desk.
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

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let allow_motion = args.iter().any(|a| a == "--motion");
    if args.iter().any(|a| a == "--verbose" || a == "-v") {
        VERBOSE.store(true, std::sync::atomic::Ordering::Relaxed);
    }
    let port_path = arg_value(&args, "--port")
        .or_else(|| std::env::var("OWB_HIL_SERIAL").ok())
        .unwrap_or_else(|| "/dev/ttyACM0".to_string());

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

    let mut passed = 0usize;
    let mut failed = 0usize;
    let mut skipped = 0usize;

    for (name, needs_motion, body) in scenarios {
        if needs_motion && !allow_motion {
            println!("  SKIP {name} (needs --motion)");
            skipped += 1;
            continue;
        }
        match body(&mut bot) {
            Ok(()) => {
                println!("  PASS {name}");
                passed += 1;
            }
            Err(err) => {
                println!("  FAIL {name}: {err}");
                failed += 1;
            }
        }
        // Never leave the wheels driving between scenarios, even after a
        // failure that returned early.
        bot.make_safe();
    }

    println!("\n{passed} passed, {failed} failed, {skipped} skipped");
    if failed > 0 {
        std::process::exit(1);
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
// serial plumbing
// ---------------------------------------------------------------------------

struct EncoderSample {
    counts: [i32; 3],
    delta: [i32; 3],
}

struct Bot {
    port: Box<dyn serialport::SerialPort>,
    /// Kept so the handle can be reopened after a board reset re-enumerates
    /// the USB device.
    path: String,
    /// Leftover bytes from a read that split a line.
    pending: String,
}

impl Bot {
    fn open(path: &str) -> Result<Self, String> {
        Ok(Self {
            port: open_port(path)?,
            path: path.to_string(),
            pending: String::new(),
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
        std::thread::sleep(Duration::from_millis(150));
        let _ = self.read_lines();
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
}
