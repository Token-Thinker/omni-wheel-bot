//! Minimal wheel-tuning rig, ported from `scripts/tune.py`.
//!
//! Bypasses the phone UI entirely. A thin host-side wrapper around the
//! firmware's `probe_speed`, `t`/`o`/`y` motion and `wheel_trim` WebSocket
//! commands, so the operator can dial in trim numbers one wheel at a time and
//! then validate a drive at a chosen trim. Works the same against a real bot
//! and against `owb-host-sim`.
//!
//! # Usage
//!
//! ```text
//! cargo run -p owb-host-sim --bin owb-tune -- --host 192.168.4.1 probe 0 0.5
//! cargo run -p owb-host-sim --bin owb-tune -- --host 192.168.4.1 --key s3cr3t drive 0.6 0.85 1.0
//! cargo run -p owb-host-sim --bin owb-tune -- --host 127.0.0.1 --port 3000 drive 0.6 1.0 1.0
//! cargo run -p owb-host-sim --bin owb-tune -- --url ws://127.0.0.1:3000/ws stop
//! ```
//!
//! Run with no arguments (or `--help`) for the full subcommand list.
//!
//! # Differences from `tune.py`
//!
//! - **No default target.** `tune.py` silently drove `192.168.1.236`; a
//!   missing `--host`/`--url` is now an error (docs/backlog.md, "Promote
//!   `tune.py` to first-class").
//! - **Motion keepalive.** The firmware's command bus zeroes the wheels when no
//!   motion command arrives for 300 ms (`MOTION_WATCHDOG_MS` in
//!   `owb-core/src/node/command_bus.rs`). `tune.py` sent each motion command
//!   once and slept, so every timed run actually stopped after ~300 ms. Timed
//!   runs here re-send the motion command every [`KEEPALIVE_PERIOD`]. The
//!   stiction kick only re-arms on a sign change, so re-sending does not
//!   re-kick.
//! - **Always stops.** Ctrl-C (or any error) during a motion subcommand sends
//!   the zero command plus `disable` before exiting; press Ctrl-C twice to
//!   force-quit.
//! - **Deterministic `drive` arguments.** See [`parse_command`].

use std::{
    net::{TcpStream, ToSocketAddrs},
    sync::atomic::{AtomicBool, Ordering},
    time::{Duration, Instant},
};

use tungstenite::{client::IntoClientRequest, http::HeaderValue, Message, WebSocket};

/// Firmware motion deadman (`MOTION_WATCHDOG_MS`): wheels are zeroed this long
/// after the last motion command.
const DEADMAN: Duration = Duration::from_millis(300);

/// Re-send cadence for the motion command during a timed run. Half the
/// deadman, so one lost or late frame still does not trip it.
const KEEPALIVE_PERIOD: Duration = Duration::from_millis(150);

/// Pause after every one-shot command, as `tune.py`'s `_send` did. Keeps the
/// setup burst (trim, heading hold, kick, enable) from landing in one TCP read.
const SEND_GAP: Duration = Duration::from_millis(50);

/// Upper bound on one blocking read while pausing, so Ctrl-C is noticed fast.
const POLL_SLICE: Duration = Duration::from_millis(20);

/// TCP connect + WebSocket handshake budget (`tune.py`'s `open_timeout=6`).
const OPEN_TIMEOUT: Duration = Duration::from_secs(6);

/// Budget for the best-effort reconnect used to deliver a stop after the
/// original link died.
const FALLBACK_TIMEOUT: Duration = Duration::from_secs(2);

const DEFAULT_PORT: u16 = 80;

/// Trim defaults used by `back`, `turn` and `backward` (from `tune.py`).
const DEFAULT_W1: f64 = 0.93;
const DEFAULT_W2: f64 = 1.0;
const DEFAULT_W3: f64 = 1.0;

const USAGE: &str = "\
owb-tune: minimal wheel-tuning rig (Rust port of scripts/tune.py)

Global options (anywhere on the command line):
    --host HOST     Bot hostname or IP (required unless --url is given)
    --port PORT     WebSocket port (default: 80; host-sim uses 3000 or OWB_BIND_ADDR)
    --url  URL      Full ws:// URL instead of --host/--port (e.g. ws://192.168.4.1/ws)
    --key  KEY      Control secret for C1 auth (sent as the ?key=... query param)
    -v, --verbose   Print every frame sent and every non-telemetry reply
    -h, --help      Show this text

Subcommands:
    probe <wheel 0|1|2> <speed -1..1> [duration_s=2.5]
        Spin a single wheel at the given signed speed. Use this to identify
        which physical wheel each index maps to and to feel out per-wheel response.

    drive <speed> <w1> <w2> [w3=1.0] [duration_s=4]
        Drive forward (direction 0) with the given trim ratios. Bakes in the
        stiction kick (120 ms, +0.25); heading hold off. With exactly four
        numbers the fourth is a duration if it is > 1, else w3.

    back <speed> [w1=0.93] [w2=1.0] [duration_s=4]
        Drive backward (direction 180) at the given speed and trim.

    turn <yaw_deg_s> [w1=0.93] [w2=1.0] [duration_s=2]
        Yaw in place (positive = CCW/left), sent as an `o` intent with speed 0.

    backward <speed> [duration_s=4] [pivot_s=1.5] [w1=0.93] [w2=1.0]
        Fake-backward macro: pivot 180 (CW), drive forward, pivot 180 (CCW).

    trim <w1> <w2> [w3=1.0]
        Push a wheel_trim command only; no motion.

    stop
        Zero translate and all wheel probes, then disable. Panic stop.

Timed runs re-send their motion command every 150 ms to stay inside the
firmware's 300 ms motion deadman. Ctrl-C stops the wheels and disables the
motors; press it twice to force-quit.

Examples:
    owb-tune --host 192.168.4.1 probe 0 0.5
    owb-tune --host 192.168.4.1 drive 0.6 0.85 1.0
    owb-tune --host 192.168.4.1 drive 0.6 0.75 1.0 4
    owb-tune --host 192.168.4.1 --key s3cr3t stop
    owb-tune --host 127.0.0.1 --port 3000 drive 0.6 1.0 1.0
";

// ---------------------------------------------------------------------------
// Ctrl-C
// ---------------------------------------------------------------------------

/// Set by the Ctrl-C handler; polled by every interruptible pause so a run
/// unwinds through the stop/disable path instead of dying mid-drive.
static INTERRUPTED: AtomicBool = AtomicBool::new(false);

fn interrupted() -> bool { INTERRUPTED.load(Ordering::Relaxed) }

/// Route Ctrl-C to [`INTERRUPTED`] instead of the default kill (which skips
/// `Drop`, so the stop would never be sent). Same approach as `hil-check`:
/// tokio is already a dependency, and a one-thread runtime on a side thread is
/// the cheapest portable signal hook. A second Ctrl-C force-quits.
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
                if INTERRUPTED.swap(true, Ordering::Relaxed) {
                    eprintln!("\nowb-tune: forced quit; motors may still be enabled");
                    std::process::exit(130);
                }
                eprintln!("\nowb-tune: interrupted; stopping wheels (Ctrl-C again to force quit)");
            }
        });
    });
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let opts = match parse_args(&args) {
        Ok(Parsed::Run(opts)) => opts,
        Ok(Parsed::Help) => {
            print!("{USAGE}");
            return;
        }
        Err(ArgError::Usage) => {
            print!("{USAGE}");
            std::process::exit(1);
        }
        Err(ArgError::Bad(msg)) => {
            eprintln!("bad args: {msg}");
            eprint!("{USAGE}");
            std::process::exit(2);
        }
    };

    let url = match base_url(&opts.target) {
        Ok(base) => build_url(&base, &session_id(), opts.key.as_deref()),
        Err(msg) => {
            eprintln!("bad args: {msg}");
            std::process::exit(2);
        }
    };

    install_ctrl_c();

    match run(&opts.cmd, &url, opts.verbose) {
        Ok(()) => {}
        Err(Fail::Interrupted) => {
            eprintln!("owb-tune: interrupted");
            std::process::exit(130);
        }
        Err(Fail::Error(msg)) => {
            eprintln!("owb-tune: {msg}");
            std::process::exit(1);
        }
    }
}

#[derive(Debug)]
enum Fail {
    Interrupted,
    Error(String),
}

impl From<String> for Fail {
    fn from(msg: String) -> Self { Fail::Error(msg) }
}

type Step = Result<(), Fail>;

fn run(
    cmd: &Command,
    url: &str,
    verbose: bool,
) -> Step {
    if verbose {
        println!("  connecting to {}", redact_key(url));
    }
    let mut link = Link::connect(url, OPEN_TIMEOUT, verbose)
        .map_err(|e| format!("cannot connect to {}: {e}", redact_key(url)))?;
    if interrupted() {
        return Err(Fail::Interrupted);
    }
    // Arm before the first frame: from here on, leaving the function any way
    // other than a clean finish (error, Ctrl-C, panic) sends the stop in Drop.
    link.armed = cmd.zero_commands();

    match *cmd {
        Command::Probe {
            wheel,
            speed,
            duration,
        } => {
            link.send(&enable())?;
            link.pause(Duration::from_millis(150))?;
            println!("  spinning wheel {wheel} @ speed_norm={speed:?} for {duration:?}s");
            link.hold(&probe_speed(wheel, speed), secs(duration))?;
            link.send(&probe_speed(wheel, 0.0))?;
            link.pause(Duration::from_millis(200))?;
            link.send(&disable())?;
            println!("  done");
        }
        Command::Drive {
            speed,
            trim,
            duration,
        } => {
            drive_setup(&mut link, trim)?;
            println!(
                "  DRIVING forward @ speed={speed:?} trim={} for {duration:?}s",
                show_trim(trim)
            );
            link.hold(&translate(0.0, speed), secs(duration))?;
            link.send(&translate(0.0, 0.0))?;
            link.pause(Duration::from_millis(300))?;
            link.send(&disable())?;
            println!("  done");
        }
        Command::Back {
            speed,
            trim,
            duration,
        } => {
            drive_setup(&mut link, trim)?;
            println!(
                "  DRIVING backward @ speed={speed:?} trim={} for {duration:?}s",
                show_trim(trim)
            );
            link.hold(&translate(180.0, speed), secs(duration))?;
            link.send(&translate(180.0, 0.0))?;
            link.pause(Duration::from_millis(300))?;
            link.send(&disable())?;
            println!("  done");
        }
        Command::Turn {
            yaw_rate,
            trim,
            duration,
        } => {
            // Sent as an `o` intent with speed 0 so it routes through
            // `apply_motion`, the same path forward translation uses; yaw-only
            // `y` intents were observed not to move v1-alpha (cause TBD).
            drive_setup(&mut link, trim)?;
            println!(
                "  YAWING (omni) @ {yaw_rate:?} deg/s trim={} for {duration:?}s",
                show_trim(trim)
            );
            link.hold(&omni_yaw(yaw_rate), secs(duration))?;
            link.send(&omni_yaw(0.0))?;
            link.pause(Duration::from_millis(300))?;
            link.send(&disable())?;
            println!("  done");
        }
        Command::Backward {
            speed,
            duration,
            pivot,
            trim,
        } => {
            // Pivot 180 one way, drive forward, pivot 180 back: net rotation
            // cancels and the bot ends translated "backward". `pivot` is the
            // per-chassis time a 180 takes; start at 1.5 s and adjust.
            drive_setup(&mut link, trim)?;

            println!("  [1/3] PIVOT 180° (W1-fwd, CW) for {pivot:?}s");
            link.hold(&yaw(-60.0), secs(pivot))?;
            link.send(&yaw(0.0))?;
            link.pause(Duration::from_millis(300))?;

            println!(
                "  [2/3] DRIVING forward @ speed={speed:?} for {duration:?}s (= fake backward)"
            );
            link.hold(&translate(0.0, speed), secs(duration))?;
            link.send(&translate(0.0, 0.0))?;
            link.pause(Duration::from_millis(300))?;

            println!("  [3/3] PIVOT 180° (W2-fwd, CCW) for {pivot:?}s");
            link.hold(&yaw(60.0), secs(pivot))?;
            link.send(&yaw(0.0))?;
            link.pause(Duration::from_millis(300))?;

            link.send(&disable())?;
            println!("  done — heading should match start, position translated 'backward'");
        }
        Command::Trim { trim } => {
            link.send(&wheel_trim(trim))?;
            println!("  trim set to {}", show_trim(trim));
        }
        Command::Stop => {
            // A panic stop must not itself be abortable by Ctrl-C. Rejections
            // are expected here (the sim refuses `t` while disabled), so they
            // are not reported.
            link.quiet_rejects = true;
            for msg in stop_commands() {
                link.write(&msg)?;
                link.pause_until(Instant::now() + SEND_GAP, false)?;
            }
            println!("  stopped + disabled");
        }
    }
    link.finish();
    Ok(())
}

/// Shared preamble for the driving subcommands: trim, heading hold off,
/// stiction kick, enable, then a short settle.
fn drive_setup(
    link: &mut Link,
    trim: [f64; 3],
) -> Step {
    link.send(&wheel_trim(trim))?;
    link.send(&heading_hold_off())?;
    link.send(&stiction_kick())?;
    link.send(&enable())?;
    link.pause(Duration::from_millis(150))
}

fn secs(s: f64) -> Duration { Duration::from_secs_f64(s) }

/// Python-style `(0.85, 1.0, 1.0)`, matching tune.py's output.
fn show_trim(trim: [f64; 3]) -> String { format!("({:?}, {:?}, {:?})", trim[0], trim[1], trim[2]) }

// ---------------------------------------------------------------------------
// argument parsing
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, PartialEq)]
enum Target {
    Host { host: String, port: u16 },
    Url(String),
}

#[derive(Debug, Clone, PartialEq)]
enum Command {
    Probe {
        wheel: u8,
        speed: f64,
        duration: f64,
    },
    Drive {
        speed: f64,
        trim: [f64; 3],
        duration: f64,
    },
    Back {
        speed: f64,
        trim: [f64; 3],
        duration: f64,
    },
    Turn {
        yaw_rate: f64,
        trim: [f64; 3],
        duration: f64,
    },
    Backward {
        speed: f64,
        duration: f64,
        pivot: f64,
        trim: [f64; 3],
    },
    Trim {
        trim: [f64; 3],
    },
    Stop,
}

impl Command {
    /// Commands that zero this subcommand's motion, sent (followed by
    /// [`stop_commands`]) if the run is cut short. `None` for subcommands that
    /// never enable the motors.
    fn zero_commands(&self) -> Option<Vec<String>> {
        match *self {
            Command::Probe { wheel, .. } => Some(vec![probe_speed(wheel, 0.0)]),
            Command::Drive { .. } => Some(vec![translate(0.0, 0.0)]),
            Command::Back { .. } => Some(vec![translate(180.0, 0.0)]),
            Command::Turn { .. } => Some(vec![omni_yaw(0.0)]),
            Command::Backward { .. } => Some(vec![yaw(0.0), translate(0.0, 0.0)]),
            Command::Trim { .. } | Command::Stop => None,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
struct Options {
    target: Target,
    key: Option<String>,
    verbose: bool,
    cmd: Command,
}

#[derive(Debug, PartialEq)]
enum Parsed {
    Run(Options),
    Help,
}

#[derive(Debug, PartialEq)]
enum ArgError {
    /// No or unknown subcommand: print usage, exit 1 (as `tune.py` did).
    Usage,
    /// Malformed arguments: print the reason and usage to stderr, exit 2.
    Bad(String),
}

impl From<String> for ArgError {
    fn from(msg: String) -> Self { ArgError::Bad(msg) }
}

fn bad(msg: impl Into<String>) -> ArgError { ArgError::Bad(msg.into()) }

fn parse_args(args: &[String]) -> Result<Parsed, ArgError> {
    let mut host = None;
    let mut port = None;
    let mut url = None;
    let mut key = None;
    let mut verbose = false;
    let mut rest: Vec<&str> = Vec::new();

    let mut i = 0;
    while i < args.len() {
        let arg = args[i].as_str();
        i += 1;
        match arg {
            "-h" | "--help" => return Ok(Parsed::Help),
            "-v" | "--verbose" => {
                verbose = true;
                continue;
            }
            _ => {}
        }
        // `--flag value` and `--flag=value`.
        let (flag, inline) = match arg.split_once('=') {
            Some((f, v)) if f.starts_with("--") => (f, Some(v.to_string())),
            _ => (arg, None),
        };
        let slot = match flag {
            "--host" => &mut host,
            "--port" => &mut port,
            "--url" => &mut url,
            "--key" => &mut key,
            _ if flag.starts_with("--") => return Err(bad(format!("unknown option {flag}"))),
            _ => {
                // Positional; negative numbers like `-0.5` land here too.
                rest.push(arg);
                continue;
            }
        };
        let value = match inline {
            Some(v) => v,
            None => {
                let v = args.get(i).ok_or_else(|| format!("{flag} needs a value"))?;
                i += 1;
                v.clone()
            }
        };
        *slot = Some(value);
    }

    let Some((&name, cmd_args)) = rest.split_first()
    else {
        return Err(ArgError::Usage);
    };
    let cmd = parse_command(name, cmd_args)?;

    let target =
        match (host, url) {
            (Some(_), Some(_)) => return Err(bad("give either --host/--port or --url, not both")),
            (None, Some(url)) => {
                if port.is_some() {
                    return Err(bad(
                        "--port cannot be combined with --url (put the port in the URL)",
                    ));
                }
                Target::Url(url)
            }
            (Some(host), None) => {
                if host.is_empty() {
                    return Err(bad("--host must not be empty"));
                }
                let port = match port {
                    None => DEFAULT_PORT,
                    Some(raw) => raw
                        .parse::<u16>()
                        .ok()
                        .filter(|p| *p != 0)
                        .ok_or_else(|| format!("--port must be 1..=65535, got {raw:?}"))?,
                };
                Target::Host { host, port }
            }
            // Deliberately no default: tune.py silently drove a hardcoded LAN IP.
            (None, None) => return Err(bad(
                "no bot address: pass --host <ip> (e.g. --host 192.168.4.1) or --url ws://<ip>/ws",
            )),
        };

    Ok(Parsed::Run(Options {
        target,
        key,
        verbose,
        cmd,
    }))
}

/// Parse one subcommand's positionals. Defaults match `tune.py`.
///
/// `drive` differs on purpose: `tune.py` guessed with `w3 = args[4]` *and*
/// `duration = args[-1] if > 1`, so its own documented example
/// `drive 0.6 0.75 1.0 4` ran with w3 = 4.0. Here, with exactly four numbers
/// the fourth is the duration if it is > 1 and w3 otherwise; with five they are
/// w3 then duration.
fn parse_command(
    name: &str,
    a: &[&str],
) -> Result<Command, ArgError> {
    let cmd = match name {
        "probe" => {
            max_args(a, 3)?;
            let raw = req(a, 0, "wheel")?;
            let wheel = raw
                .parse::<u8>()
                .ok()
                .filter(|w| *w <= 2)
                .ok_or_else(|| format!("wheel must be 0, 1 or 2, got {raw:?}"))?;
            Command::Probe {
                wheel,
                speed: speed_arg(a, 1)?,
                duration: duration_arg(a, 2, 2.5)?,
            }
        }
        "drive" => {
            max_args(a, 5)?;
            let speed = speed_arg(a, 0)?;
            let w1 = num(a, 1, "w1")?;
            let w2 = num(a, 2, "w2")?;
            let (w3, duration) = match a.len() {
                3 => (DEFAULT_W3, 4.0),
                4 => {
                    let x = num(a, 3, "w3/duration")?;
                    if x > 1.0 {
                        (DEFAULT_W3, x)
                    }
                    else {
                        (x, 4.0)
                    }
                }
                _ => (num(a, 3, "w3")?, duration_arg(a, 4, 4.0)?),
            };
            Command::Drive {
                speed,
                trim: [w1, w2, w3],
                duration,
            }
        }
        "back" => {
            max_args(a, 4)?;
            Command::Back {
                speed: speed_arg(a, 0)?,
                trim: [
                    opt_num(a, 1, "w1", DEFAULT_W1)?,
                    opt_num(a, 2, "w2", DEFAULT_W2)?,
                    DEFAULT_W3,
                ],
                duration: duration_arg(a, 3, 4.0)?,
            }
        }
        "turn" => {
            max_args(a, 4)?;
            Command::Turn {
                yaw_rate: num(a, 0, "yaw_deg_s")?,
                trim: [
                    opt_num(a, 1, "w1", DEFAULT_W1)?,
                    opt_num(a, 2, "w2", DEFAULT_W2)?,
                    DEFAULT_W3,
                ],
                duration: duration_arg(a, 3, 2.0)?,
            }
        }
        "backward" => {
            max_args(a, 5)?;
            Command::Backward {
                speed: speed_arg(a, 0)?,
                duration: duration_arg(a, 1, 4.0)?,
                pivot: duration_arg(a, 2, 1.5)?,
                trim: [
                    opt_num(a, 3, "w1", DEFAULT_W1)?,
                    opt_num(a, 4, "w2", DEFAULT_W2)?,
                    DEFAULT_W3,
                ],
            }
        }
        "trim" => {
            max_args(a, 3)?;
            Command::Trim {
                trim: [
                    num(a, 0, "w1")?,
                    num(a, 1, "w2")?,
                    opt_num(a, 2, "w3", DEFAULT_W3)?,
                ],
            }
        }
        "stop" => {
            max_args(a, 0)?;
            Command::Stop
        }
        _ => return Err(ArgError::Usage),
    };
    Ok(cmd)
}

fn max_args(
    a: &[&str],
    max: usize,
) -> Result<(), String> {
    match a.get(max) {
        Some(extra) => Err(format!("unexpected extra argument {extra:?}")),
        None => Ok(()),
    }
}

fn req<'a>(
    a: &[&'a str],
    idx: usize,
    name: &str,
) -> Result<&'a str, String> {
    a.get(idx)
        .copied()
        .ok_or_else(|| format!("missing <{name}>"))
}

fn parse_num(
    raw: &str,
    name: &str,
) -> Result<f64, String> {
    // Rust accepts "nan"/"inf"; neither is a usable command (or valid JSON).
    raw.parse::<f64>()
        .ok()
        .filter(|v| v.is_finite())
        .ok_or_else(|| format!("<{name}> must be a number, got {raw:?}"))
}

fn num(
    a: &[&str],
    idx: usize,
    name: &str,
) -> Result<f64, String> {
    parse_num(req(a, idx, name)?, name)
}

fn opt_num(
    a: &[&str],
    idx: usize,
    name: &str,
    default: f64,
) -> Result<f64, String> {
    a.get(idx).map_or(Ok(default), |raw| parse_num(raw, name))
}

fn speed_arg(
    a: &[&str],
    idx: usize,
) -> Result<f64, String> {
    let v = num(a, idx, "speed")?;
    if !(-1.0..=1.0).contains(&v) {
        return Err(format!("<speed> must be in -1..=1, got {v}"));
    }
    Ok(v)
}

fn duration_arg(
    a: &[&str],
    idx: usize,
    default: f64,
) -> Result<f64, String> {
    let v = opt_num(a, idx, "duration_s", default)?;
    if v <= 0.0 {
        return Err(format!("durations must be > 0 s, got {v}"));
    }
    Ok(v)
}

// ---------------------------------------------------------------------------
// URL building
// ---------------------------------------------------------------------------

fn base_url(target: &Target) -> Result<String, String> {
    match target {
        Target::Host { host, port } => {
            // Bare IPv6 literals need brackets in a URL authority.
            let host = if host.contains(':') && !host.starts_with('[') {
                format!("[{host}]")
            }
            else {
                host.clone()
            };
            Ok(if *port == DEFAULT_PORT {
                format!("ws://{host}/ws")
            }
            else {
                format!("ws://{host}:{port}/ws")
            })
        }
        Target::Url(url) => {
            let Some(rest) = url.strip_prefix("ws://")
            else {
                return Err(format!(
                    "--url must start with ws:// (the bot serves plain WebSocket), got {url:?}"
                ));
            };
            if rest.is_empty() || rest.starts_with('/') {
                return Err(format!("--url has no host: {url:?}"));
            }
            // `ws://host` or `ws://host:port` → the bot's control socket.
            Ok(if rest.contains('/') || rest.contains('?') {
                url.clone()
            }
            else {
                format!("{url}/ws")
            })
        }
    }
}

/// Append the `session` and (when given) `key` query params, as `tune.py` did.
fn build_url(
    base: &str,
    session: &str,
    key: Option<&str>,
) -> String {
    let sep = if base.contains('?') { '&' } else { '?' };
    let mut url = format!("{base}{sep}session={session}");
    if let Some(key) = key.filter(|k| !k.is_empty()) {
        url.push_str("&key=");
        url.push_str(&percent_encode(key));
    }
    url
}

/// Percent-encode everything but RFC 3986 unreserved characters. The firmware
/// (picoserve) decodes `%XX`, so keys with `&`, `=`, `#` or spaces survive;
/// plain alphanumeric keys are sent byte-for-byte as before.
fn percent_encode(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for b in s.bytes() {
        if b.is_ascii_alphanumeric() || matches!(b, b'-' | b'.' | b'_' | b'~') {
            out.push(char::from(b));
        }
        else {
            out.push_str(&format!("%{b:02X}"));
        }
    }
    out
}

/// Never print the control secret.
fn redact_key(url: &str) -> String {
    let Some(start) = url
        .find("key=")
        .filter(|&i| i > 0 && matches!(url.as_bytes()[i - 1], b'?' | b'&'))
    else {
        return url.to_string();
    };
    let value_start = start + "key=".len();
    let value_end = url[value_start..]
        .find('&')
        .map_or(url.len(), |i| value_start + i);
    format!("{}***{}", &url[..value_start], &url[value_end..])
}

/// `tune-<8 hex>`, unique per run, without pulling in a rand crate.
fn session_id() -> String {
    use std::hash::{BuildHasher, Hasher};
    let mut h = std::collections::hash_map::RandomState::new().build_hasher();
    if let Ok(t) = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH) {
        h.write_u128(t.as_nanos());
    }
    h.write_u32(std::process::id());
    format!("tune-{:08x}", h.finish() as u32)
}

// ---------------------------------------------------------------------------
// wire payloads
//
// Built by hand so key order matches tune.py exactly (serde_json::json! sorts
// keys). Floats use `{:?}`, which like Python's repr always keeps a decimal
// point ("0.0", "0.6"); finite values are guaranteed by the parser.
// ---------------------------------------------------------------------------

fn enable() -> String { r#"{"ct":"i","ic":"enable"}"#.to_string() }

fn disable() -> String { r#"{"ct":"i","ic":"disable"}"#.to_string() }

fn probe_speed(
    wheel: u8,
    speed: f64,
) -> String {
    format!(r#"{{"ct":"i","ic":"probe_speed","wheel":{wheel},"speed_norm":{speed:?}}}"#)
}

fn translate(
    direction_deg: f64,
    speed: f64,
) -> String {
    format!(r#"{{"ct":"i","ic":"t","direction_deg":{direction_deg:?},"speed_norm":{speed:?}}}"#)
}

fn omni_yaw(yaw_rate: f64) -> String {
    format!(
        r#"{{"ct":"i","ic":"o","direction_deg":0.0,"speed_norm":0.0,"yaw_rate_deg_s":{yaw_rate:?},"orientation_deg":null}}"#
    )
}

fn yaw(yaw_rate: f64) -> String {
    format!(r#"{{"ct":"i","ic":"y","yaw_rate_deg_s":{yaw_rate:?},"orientation_deg":null}}"#)
}

fn wheel_trim(trim: [f64; 3]) -> String {
    let [w1, w2, w3] = trim;
    format!(r#"{{"ct":"i","ic":"wheel_trim","w1":{w1:?},"w2":{w2:?},"w3":{w3:?}}}"#)
}

fn heading_hold_off() -> String {
    r#"{"ct":"i","ic":"heading_hold","enable":false,"kp":0.0}"#.to_string()
}

fn stiction_kick() -> String {
    r#"{"ct":"i","ic":"stiction_kick","ms":120,"boost":0.25}"#.to_string()
}

/// The `stop` subcommand, also the tail of every abort. `tune.py` zeroed
/// probes on wheels 0 and 1 only; wheel 2 is zeroed too so a `probe 2` is
/// covered (a zero command to a dead wheel is harmless).
fn stop_commands() -> Vec<String> {
    vec![
        translate(0.0, 0.0),
        probe_speed(0, 0.0),
        probe_speed(1, 0.0),
        probe_speed(2, 0.0),
        disable(),
    ]
}

// ---------------------------------------------------------------------------
// keepalive scheduling
// ---------------------------------------------------------------------------

/// Offsets from the start of a timed run at which to (re-)send the motion
/// command: `0, period, 2*period, ...` strictly before `duration`. The gap
/// between sends, and from the last send to the end of the run, never exceeds
/// `period`.
fn keepalive_offsets(
    duration: Duration,
    period: Duration,
) -> Vec<Duration> {
    let mut offsets = Vec::new();
    let mut t = Duration::ZERO;
    while t < duration {
        offsets.push(t);
        t += period;
    }
    offsets
}

// ---------------------------------------------------------------------------
// WebSocket link
// ---------------------------------------------------------------------------

struct Link {
    ws: WebSocket<TcpStream>,
    /// Kept for the best-effort stop reconnect.
    url: String,
    verbose: bool,
    /// While `Some`, dropping the link sends these zero commands followed by
    /// [`stop_commands`]. Cleared by [`Link::finish`].
    armed: Option<Vec<String>>,
    /// Suppress the "bot rejected a command" warning (used by `stop`).
    quiet_rejects: bool,
}

impl Link {
    fn connect(
        url: &str,
        timeout: Duration,
        verbose: bool,
    ) -> Result<Self, String> {
        let mut request = url
            .into_client_request()
            .map_err(|e| format!("bad URL: {e}"))?;
        // Both the firmware and the host-sim speak the "messages" subprotocol.
        request.headers_mut().insert(
            "Sec-WebSocket-Protocol",
            HeaderValue::from_static("messages"),
        );

        let uri = request.uri();
        let host = uri
            .host()
            .ok_or("URL has no host")?
            .trim_start_matches('[')
            .trim_end_matches(']')
            .to_string();
        let port = uri.port_u16().unwrap_or(DEFAULT_PORT);
        let addrs = (host.as_str(), port)
            .to_socket_addrs()
            .map_err(|e| format!("cannot resolve {host}: {e}"))?;

        let mut last_err = format!("no address for {host}");
        let mut stream = None;
        for addr in addrs {
            match TcpStream::connect_timeout(&addr, timeout) {
                Ok(s) => {
                    stream = Some(s);
                    break;
                }
                Err(e) => last_err = format!("{addr}: {e}"),
            }
        }
        let stream = stream.ok_or(last_err)?;
        let io = |e: std::io::Error| e.to_string();
        stream.set_read_timeout(Some(timeout)).map_err(io)?;
        stream.set_write_timeout(Some(timeout)).map_err(io)?;
        stream.set_nodelay(true).map_err(io)?;

        let (ws, _response) = tungstenite::client(request, stream).map_err(|e| match e {
            tungstenite::HandshakeError::Failure(e) => format!("handshake failed: {e}"),
            tungstenite::HandshakeError::Interrupted(_) => "handshake timed out".to_string(),
        })?;
        Ok(Self {
            ws,
            url: url.to_string(),
            verbose,
            armed: None,
            quiet_rejects: false,
        })
    }

    /// Write one frame, no pacing.
    fn write(
        &mut self,
        msg: &str,
    ) -> Result<(), String> {
        if self.verbose {
            println!("  -> {msg}");
        }
        self.ws
            .send(Message::Text(msg.to_string()))
            .map_err(|e| format!("send failed: {e}"))
    }

    /// Write one frame, then pause [`SEND_GAP`] (tune.py's `_send`).
    fn send(
        &mut self,
        msg: &str,
    ) -> Step {
        self.write(msg)?;
        self.pause(SEND_GAP)
    }

    fn pause(
        &mut self,
        d: Duration,
    ) -> Step {
        self.pause_until(Instant::now() + d, true)
    }

    /// Re-send `msg` on the [`keepalive_offsets`] schedule for `duration`,
    /// keeping the firmware's motion deadman fed.
    fn hold(
        &mut self,
        msg: &str,
        duration: Duration,
    ) -> Step {
        let start = Instant::now();
        for offset in keepalive_offsets(duration, KEEPALIVE_PERIOD) {
            self.pause_until(start + offset, true)?;
            self.write(msg)?;
        }
        self.pause_until(start + duration, true)
    }

    /// Wait until `deadline`, servicing incoming frames (pongs, close, error
    /// acks) in [`POLL_SLICE`] reads.
    fn pause_until(
        &mut self,
        deadline: Instant,
        interruptible: bool,
    ) -> Step {
        loop {
            if interruptible && interrupted() {
                return Err(Fail::Interrupted);
            }
            let now = Instant::now();
            if now >= deadline {
                return Ok(());
            }
            let slice = (deadline - now).clamp(Duration::from_millis(1), POLL_SLICE);
            self.ws
                .get_ref()
                .set_read_timeout(Some(slice))
                .map_err(|e| e.to_string())?;
            match self.ws.read() {
                Ok(msg) => self.on_message(msg)?,
                Err(e) if read_idle(&e) => {}
                Err(tungstenite::Error::ConnectionClosed | tungstenite::Error::AlreadyClosed) => {
                    return Err(Fail::Error("connection closed by the bot".to_string()));
                }
                Err(e) => return Err(format!("receive failed: {e}").into()),
            }
        }
    }

    fn on_message(
        &mut self,
        msg: Message,
    ) -> Step {
        match msg {
            Message::Text(text) => {
                let value: Option<serde_json::Value> = serde_json::from_str(&text).ok();
                let kind = value
                    .as_ref()
                    .and_then(|v| v.get("type"))
                    .and_then(|t| t.as_str());
                if self.verbose && kind != Some("telemetry") {
                    println!("  <- {text}");
                }
                match kind {
                    Some("ack")
                        if value
                            .as_ref()
                            .and_then(|v| v.get("status"))
                            .and_then(|s| s.as_str())
                            == Some("error")
                            && !self.quiet_rejects =>
                    {
                        let why = value
                            .as_ref()
                            .and_then(|v| v.get("message"))
                            .and_then(|m| m.as_str())
                            .unwrap_or("?");
                        eprintln!("  !! bot rejected a command: {why}");
                    }
                    Some("unauthorized") => {
                        eprintln!("  !! bot rejected the control key (pass the right --key)");
                    }
                    _ => {}
                }
                Ok(())
            }
            Message::Close(frame) => {
                let why = frame.map_or_else(
                    || "no reason given".to_string(),
                    |f| format!("{} {}", u16::from(f.code), f.reason),
                );
                Err(format!("bot closed the connection ({why})").into())
            }
            _ => Ok(()),
        }
    }

    /// Clean exit: disarm and close politely.
    fn finish(mut self) {
        self.armed = None;
        let _ = self.ws.close(None);
        // Let the close handshake complete; bounded, and errors are moot here.
        let deadline = Instant::now() + Duration::from_millis(200);
        while Instant::now() < deadline {
            let _ = self.ws.get_ref().set_read_timeout(Some(POLL_SLICE));
            match self.ws.read() {
                Ok(_) => {}
                Err(e) if read_idle(&e) => {}
                Err(_) => break,
            }
        }
    }

    /// Best-effort stop for an aborted run. Never aborts on Ctrl-C.
    fn safe_stop(
        &mut self,
        zero: Vec<String>,
    ) {
        eprintln!("  stopping wheels and disabling motors");
        let mut msgs = zero;
        let tail: Vec<String> = stop_commands()
            .into_iter()
            .filter(|m| !msgs.contains(m))
            .collect();
        msgs.extend(tail);
        let mut delivered = true;
        for msg in &msgs {
            if self.write(msg).is_err() {
                delivered = false;
                break;
            }
            let _ = self.pause_until(Instant::now() + SEND_GAP, false);
        }
        if delivered {
            return;
        }
        // The link is dead. The firmware deadman zeroes the wheels within
        // 300 ms regardless, but only an explicit disable de-energises the
        // drivers, so try once more on a fresh socket.
        match Link::connect(&self.url, FALLBACK_TIMEOUT, self.verbose) {
            Ok(mut fresh) => {
                for msg in &msgs {
                    if fresh.write(msg).is_err() {
                        break;
                    }
                    let _ = fresh.pause_until(Instant::now() + SEND_GAP, false);
                }
                fresh.finish();
            }
            Err(e) => eprintln!(
                "  !! could not deliver stop ({e}); the {} ms deadman zeroes the wheels, but motors stay enabled",
                DEADMAN.as_millis()
            ),
        }
    }
}

/// A read that returned no frame but left the link healthy: the poll slice
/// elapsed, or the Ctrl-C signal interrupted the syscall (EINTR), in which case
/// the caller's next loop turn sees [`INTERRUPTED`].
fn read_idle(e: &tungstenite::Error) -> bool {
    matches!(e, tungstenite::Error::Io(io) if matches!(
        io.kind(),
        std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut | std::io::ErrorKind::Interrupted
    ))
}

impl Drop for Link {
    fn drop(&mut self) {
        if let Some(zero) = self.armed.take() {
            self.safe_stop(zero);
        }
    }
}

// ---------------------------------------------------------------------------
// tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn argv(s: &str) -> Vec<String> { s.split_whitespace().map(str::to_string).collect() }

    fn parse(s: &str) -> Result<Parsed, ArgError> { parse_args(&argv(s)) }

    fn run_opts(s: &str) -> Options {
        match parse(s) {
            Ok(Parsed::Run(o)) => o,
            other => panic!("expected Run for {s:?}, got {other:?}"),
        }
    }

    fn cmd(s: &str) -> Command { run_opts(&format!("--host h {s}")).cmd }

    fn is_bad(r: Result<Parsed, ArgError>) -> bool { matches!(r, Err(ArgError::Bad(_))) }

    #[test]
    fn global_options_parse_anywhere_and_both_forms() {
        let o = run_opts("--host 10.0.0.5 --port 3000 --key s3cr3t -v stop");
        assert_eq!(
            o.target,
            Target::Host {
                host: "10.0.0.5".into(),
                port: 3000
            }
        );
        assert_eq!(o.key.as_deref(), Some("s3cr3t"));
        assert!(o.verbose);
        assert_eq!(o.cmd, Command::Stop);

        let o = run_opts("stop --host=bot.local --key=k");
        assert_eq!(
            o.target,
            Target::Host {
                host: "bot.local".into(),
                port: 80
            }
        );
        assert_eq!(o.key.as_deref(), Some("k"));

        let o = run_opts("--url ws://127.0.0.1:3311/ws stop");
        assert_eq!(o.target, Target::Url("ws://127.0.0.1:3311/ws".into()));
    }

    #[test]
    fn no_hardcoded_default_host() {
        assert!(is_bad(parse("stop")));
        assert!(is_bad(parse("--port 3000 stop")));
    }

    #[test]
    fn usage_and_help_paths() {
        assert_eq!(parse(""), Err(ArgError::Usage));
        assert_eq!(parse("--host h"), Err(ArgError::Usage));
        assert_eq!(parse("--host h dance"), Err(ArgError::Usage));
        assert_eq!(parse("--help"), Ok(Parsed::Help));
        assert_eq!(parse("--host h stop -h"), Ok(Parsed::Help));
    }

    #[test]
    fn bad_option_combinations() {
        assert!(is_bad(parse("--host h --url ws://x/ws stop")));
        assert!(is_bad(parse("--url ws://x/ws --port 1 stop")));
        assert!(is_bad(parse("--host h --port 0 stop")));
        assert!(is_bad(parse("--host h --port nope stop")));
        assert!(is_bad(parse("--host h --frobnicate stop")));
        assert!(is_bad(parse("--host")));
    }

    #[test]
    fn probe_args_and_defaults() {
        assert_eq!(
            cmd("probe 0 0.5"),
            Command::Probe {
                wheel: 0,
                speed: 0.5,
                duration: 2.5
            }
        );
        // Negative speeds are positionals, not flags.
        assert_eq!(
            cmd("probe 2 -0.6 1"),
            Command::Probe {
                wheel: 2,
                speed: -0.6,
                duration: 1.0
            }
        );
        assert!(is_bad(parse("--host h probe 3 0.5")));
        assert!(is_bad(parse("--host h probe 0 1.5")));
        assert!(is_bad(parse("--host h probe 0 nan")));
        assert!(is_bad(parse("--host h probe 0 0.5 0")));
        assert!(is_bad(parse("--host h probe 0")));
        assert!(is_bad(parse("--host h probe 0 0.5 1 9")));
    }

    #[test]
    fn drive_fourth_number_is_duration_only_above_one() {
        let drive = |speed, trim, duration| Command::Drive {
            speed,
            trim,
            duration,
        };
        assert_eq!(cmd("drive 0.6 0.85 1.0"), drive(0.6, [0.85, 1.0, 1.0], 4.0));
        // tune.py's own example; it used to set w3 = 4.0 as well.
        assert_eq!(
            cmd("drive 0.6 0.75 1.0 4"),
            drive(0.6, [0.75, 1.0, 1.0], 4.0)
        );
        assert_eq!(
            cmd("drive 0.6 0.75 1.0 0.9"),
            drive(0.6, [0.75, 1.0, 0.9], 4.0)
        );
        assert_eq!(
            cmd("drive 0.6 0.75 1.0 1.0"),
            drive(0.6, [0.75, 1.0, 1.0], 4.0)
        );
        assert_eq!(
            cmd("drive 0.6 0.75 1.0 0.9 0.5"),
            drive(0.6, [0.75, 1.0, 0.9], 0.5)
        );
        assert!(is_bad(parse("--host h drive 0.6 0.75")));
    }

    #[test]
    fn back_turn_backward_trim_defaults() {
        assert_eq!(
            cmd("back 0.6"),
            Command::Back {
                speed: 0.6,
                trim: [0.93, 1.0, 1.0],
                duration: 4.0
            }
        );
        assert_eq!(
            cmd("back 0.6 0.9 0.8 3"),
            Command::Back {
                speed: 0.6,
                trim: [0.9, 0.8, 1.0],
                duration: 3.0
            }
        );
        assert_eq!(
            cmd("turn -45"),
            Command::Turn {
                yaw_rate: -45.0,
                trim: [0.93, 1.0, 1.0],
                duration: 2.0
            }
        );
        assert_eq!(
            cmd("backward 0.5"),
            Command::Backward {
                speed: 0.5,
                duration: 4.0,
                pivot: 1.5,
                trim: [0.93, 1.0, 1.0]
            }
        );
        assert_eq!(
            cmd("backward 0.5 3 1.2 0.9 0.95"),
            Command::Backward {
                speed: 0.5,
                duration: 3.0,
                pivot: 1.2,
                trim: [0.9, 0.95, 1.0]
            }
        );
        assert_eq!(
            cmd("trim 0.9 1.0"),
            Command::Trim {
                trim: [0.9, 1.0, 1.0]
            }
        );
        assert_eq!(
            cmd("trim 0.9 1.0 0.8"),
            Command::Trim {
                trim: [0.9, 1.0, 0.8]
            }
        );
        assert!(is_bad(parse("--host h stop now")));
    }

    #[test]
    fn url_building_matches_tune_py() {
        let host = |h: &str, port| {
            base_url(&Target::Host {
                host: h.into(),
                port,
            })
            .unwrap()
        };
        assert_eq!(host("192.168.4.1", 80), "ws://192.168.4.1/ws");
        assert_eq!(host("127.0.0.1", 3311), "ws://127.0.0.1:3311/ws");
        assert_eq!(host("::1", 3000), "ws://[::1]:3000/ws");

        assert_eq!(
            build_url("ws://b/ws", "tune-0a1b2c3d", None),
            "ws://b/ws?session=tune-0a1b2c3d"
        );
        assert_eq!(
            build_url("ws://b/ws", "tune-0a1b2c3d", Some("s3cr3t")),
            "ws://b/ws?session=tune-0a1b2c3d&key=s3cr3t"
        );
        // Empty key is omitted, as `if key:` did.
        assert_eq!(build_url("ws://b/ws", "s", Some("")), "ws://b/ws?session=s");
        assert_eq!(
            build_url("ws://b/ws?x=1", "s", Some("a b&c=d")),
            "ws://b/ws?x=1&session=s&key=a%20b%26c%3Dd"
        );
    }

    #[test]
    fn explicit_urls() {
        let url = |u: &str| base_url(&Target::Url(u.into()));
        assert_eq!(url("ws://192.168.4.1/ws").unwrap(), "ws://192.168.4.1/ws");
        assert_eq!(
            url("ws://127.0.0.1:3311").unwrap(),
            "ws://127.0.0.1:3311/ws"
        );
        assert!(url("wss://bot/ws").is_err());
        assert!(url("http://bot/ws").is_err());
        assert!(url("ws://").is_err());
    }

    #[test]
    fn key_is_redacted_for_display() {
        assert_eq!(
            redact_key("ws://b/ws?session=s&key=s3cr3t"),
            "ws://b/ws?session=s&key=***"
        );
        assert_eq!(redact_key("ws://b/ws?key=k&x=1"), "ws://b/ws?key=***&x=1");
        assert_eq!(
            redact_key("ws://b/ws?session=monkey=1"),
            "ws://b/ws?session=monkey=1"
        );
    }

    #[test]
    fn session_ids_look_like_tune_py() {
        let id = session_id();
        assert!(id.starts_with("tune-"));
        assert_eq!(id.len(), "tune-".len() + 8);
        assert!(id["tune-".len()..].chars().all(|c| c.is_ascii_hexdigit()));
    }

    #[test]
    fn payloads_match_tune_py_exactly() {
        assert_eq!(enable(), r#"{"ct":"i","ic":"enable"}"#);
        assert_eq!(disable(), r#"{"ct":"i","ic":"disable"}"#);
        assert_eq!(
            probe_speed(0, 0.6),
            r#"{"ct":"i","ic":"probe_speed","wheel":0,"speed_norm":0.6}"#
        );
        assert_eq!(
            probe_speed(1, 0.0),
            r#"{"ct":"i","ic":"probe_speed","wheel":1,"speed_norm":0.0}"#
        );
        assert_eq!(
            translate(180.0, 0.6),
            r#"{"ct":"i","ic":"t","direction_deg":180.0,"speed_norm":0.6}"#
        );
        assert_eq!(
            omni_yaw(-30.0),
            r#"{"ct":"i","ic":"o","direction_deg":0.0,"speed_norm":0.0,"yaw_rate_deg_s":-30.0,"orientation_deg":null}"#
        );
        assert_eq!(
            yaw(60.0),
            r#"{"ct":"i","ic":"y","yaw_rate_deg_s":60.0,"orientation_deg":null}"#
        );
        assert_eq!(
            wheel_trim([0.85, 1.0, 1.0]),
            r#"{"ct":"i","ic":"wheel_trim","w1":0.85,"w2":1.0,"w3":1.0}"#
        );
        assert_eq!(
            heading_hold_off(),
            r#"{"ct":"i","ic":"heading_hold","enable":false,"kp":0.0}"#
        );
        assert_eq!(
            stiction_kick(),
            r#"{"ct":"i","ic":"stiction_kick","ms":120,"boost":0.25}"#
        );
    }

    #[test]
    fn payloads_are_valid_json_and_parse_as_firmware_commands() {
        let mut all = vec![
            enable(),
            translate(0.0, -1.0),
            omni_yaw(1e-7),
            yaw(-60.0),
            wheel_trim([0.93, 1.0, 1.0]),
            heading_hold_off(),
            stiction_kick(),
        ];
        all.extend(stop_commands());
        for msg in all {
            let v: serde_json::Value = serde_json::from_str(&msg).expect(&msg);
            assert_eq!(v["ct"], "i", "{msg}");
            assert!(
                owb_core::net::protocol::parse_command_from_str(&msg).is_ok(),
                "firmware parser rejected {msg}"
            );
        }
    }

    #[test]
    fn stop_zeroes_every_wheel_then_disables() {
        let stop = stop_commands();
        assert_eq!(stop.first(), Some(&translate(0.0, 0.0)));
        for wheel in 0..=2 {
            assert!(stop.contains(&probe_speed(wheel, 0.0)));
        }
        assert_eq!(stop.last(), Some(&disable()));
    }

    #[test]
    fn only_motion_subcommands_arm_the_abort_stop() {
        assert_eq!(
            cmd("probe 2 0.5").zero_commands(),
            Some(vec![probe_speed(2, 0.0)])
        );
        assert_eq!(
            cmd("back 0.5").zero_commands(),
            Some(vec![translate(180.0, 0.0)])
        );
        assert!(cmd("turn 30").zero_commands().is_some());
        assert!(cmd("backward 0.5").zero_commands().is_some());
        assert!(cmd("drive 0.5 1 1").zero_commands().is_some());
        assert_eq!(cmd("trim 1 1").zero_commands(), None);
        assert_eq!(cmd("stop").zero_commands(), None);
    }

    #[test]
    fn keepalive_stays_inside_the_deadman() {
        for ms in [1_u64, 149, 150, 151, 1_000, 2_500, 4_000] {
            let duration = Duration::from_millis(ms);
            let offsets = keepalive_offsets(duration, KEEPALIVE_PERIOD);
            assert_eq!(offsets.first(), Some(&Duration::ZERO), "{ms} ms");
            for pair in offsets.windows(2) {
                assert_eq!(pair[1] - pair[0], KEEPALIVE_PERIOD);
            }
            let tail = duration - *offsets.last().unwrap();
            assert!(tail <= KEEPALIVE_PERIOD, "{ms} ms: tail {tail:?}");
            assert!(offsets.iter().all(|o| *o < duration));
        }
        assert!(KEEPALIVE_PERIOD * 2 <= DEADMAN);
        // 2.5 s probe: sends at 0, 150, ..., 2400 ms.
        assert_eq!(
            keepalive_offsets(Duration::from_millis(2_500), KEEPALIVE_PERIOD).len(),
            17
        );
        assert!(keepalive_offsets(Duration::ZERO, KEEPALIVE_PERIOD).is_empty());
    }

    #[test]
    fn percent_encoding_leaves_unreserved_alone() {
        assert_eq!(percent_encode("s3cr3t-._~"), "s3cr3t-._~");
        assert_eq!(percent_encode("a/b?#"), "a%2Fb%3F%23");
    }
}
