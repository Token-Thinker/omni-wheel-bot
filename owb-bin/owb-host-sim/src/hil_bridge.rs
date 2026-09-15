//! Hardware-in-the-loop bridge: mirror operator commands to a real robot.
//!
//! When `OWB_HIL_SERIAL` names a serial device (e.g. `/dev/ttyACM0`), every
//! `SystemCommand` that flows through the host-sim is re-encoded as a
//! newline-delimited JSON frame and written to that port. The ESP32-S3
//! firmware's `usb_command_task` reads exactly this framing (UTF-8 JSON per
//! line) and feeds it through the *same* `parse_command_from_str` used by the
//! WebSocket path, so the bytes we emit here are byte-identical to a WS frame.
//!
//! Result: driving the on-screen sim from the operator UI simultaneously drives
//! the physical robot over USB — no network, no firmware changes.
//!
//! ## Two-lane outbox (safety)
//!
//! Serialization happens on the async caller; the blocking serial write lives
//! on a dedicated OS thread so a slow/unplugged device can never stall the
//! request path. Frames are split into two lanes, mirroring the firmware's own
//! bus semantics:
//!
//! * **Motion** (`T`/`Y`/`O`) is a continuous stream where only the newest
//!   value matters, so we keep a single coalescing slot — a stale joystick
//!   frame is overwritten, never queued. This eliminates any motion backlog.
//! * **Priority** (Estop, Enable/Disable, trim, reads, LED, …) is a short FIFO
//!   that the writer always drains *before* motion, so an Estop can never sit
//!   behind stale motion frames and is never coalesced away.
//!
//! This keeps Estop latency at one in-flight serial write regardless of how
//! fast the joystick is streaming.

use std::{
    collections::VecDeque,
    sync::{Arc, Condvar, Mutex},
    thread,
    time::Duration,
};

use owb_core::ctrl::{sysctrl::I2CCommand, SystemCommand};
use tracing::{error, info, warn};

/// Cap on the priority FIFO. Discrete safety/config commands are infrequent, so
/// this is generous; it exists only to bound memory if the device goes dead.
/// On overflow we drop the *oldest* priority frame, preserving the newest
/// (which is the one most likely to be a fresh Estop).
const PRIORITY_CAP: usize = 64;

/// Outbound frames, split into a coalescing motion slot and a priority FIFO.
struct Outbox {
    priority: VecDeque<String>,
    motion: Option<String>,
    closed: bool,
    /// Set by the writer thread when the serial link dies unrecoverably.
    ///
    /// The writer has no reconnect path, so once it exits, no frame will ever
    /// reach the robot again. Without this flag `forward` kept cheerfully
    /// enqueuing into a queue nobody drained — the operator saw a "live" HIL
    /// bridge while every command, including Estop, was silently discarded.
    failed: bool,
}

impl Outbox {
    fn is_idle(&self) -> bool { self.priority.is_empty() && self.motion.is_none() }
}

/// A handle that forwards `SystemCommand`s to a serial-attached robot.
pub struct HilBridge {
    shared: Arc<(Mutex<Outbox>, Condvar)>,
    port_label: String,
    writer: Option<thread::JoinHandle<()>>,
}

/// Motion frames are coalescable (only the latest matters); everything else is
/// a discrete command that must be delivered promptly and in order.
fn is_motion(cmd: &SystemCommand) -> bool {
    matches!(
        cmd,
        SystemCommand::I(I2CCommand::T { .. })
            | SystemCommand::I(I2CCommand::Y { .. })
            | SystemCommand::I(I2CCommand::O { .. })
    )
}

impl HilBridge {
    /// Open `port` and spawn the writer thread.
    pub fn open(
        port: &str,
        baud: u32,
    ) -> Result<Self, serialport::Error> {
        // USB-Serial-JTAG is a USB-CDC endpoint that ignores the baud rate, but
        // `serialport` requires one; 115200 is conventional.
        let mut handle = serialport::new(port, baud)
            .timeout(Duration::from_millis(200))
            .open()?;

        let shared = Arc::new((
            Mutex::new(Outbox {
                priority: VecDeque::new(),
                motion: None,
                closed: false,
                failed: false,
            }),
            Condvar::new(),
        ));

        let port_label = port.to_string();
        let thread_label = port_label.clone();
        let writer_shared = shared.clone();

        let writer = thread::Builder::new()
            .name("owb-hil-serial".into())
            .spawn(move || {
                let (lock, cvar) = &*writer_shared;
                loop {
                    // Wait for work; drain priority before motion.
                    let frame = {
                        let mut out = lock.lock().unwrap();
                        while out.is_idle() && !out.closed {
                            out = cvar.wait(out).unwrap();
                        }
                        if out.is_idle() && out.closed {
                            break;
                        }
                        out.priority
                            .pop_front()
                            .or_else(|| out.motion.take())
                            .expect("outbox non-idle but yielded no frame")
                    };

                    // Serial write happens with the lock released.
                    if let Err(e) = handle
                        .write_all(frame.as_bytes())
                        .and_then(|_| handle.write_all(b"\n"))
                    {
                        // Unrecoverable: there is no reconnect path. Mark the
                        // outbox failed so `forward` stops pretending the robot
                        // is still reachable, and drop the undeliverable
                        // backlog rather than holding it forever.
                        error!(
                            "HIL serial write to {thread_label} failed: {e}; bridge is DEAD \
                             — the physical robot will receive NO further commands \
                             (including Estop). Restart the host-sim to reconnect."
                        );
                        let mut out = lock.lock().unwrap();
                        out.failed = true;
                        out.priority.clear();
                        out.motion = None;
                        break;
                    }
                    let _ = handle.flush();
                }
                info!("HIL serial bridge to {thread_label} closed");
            })
            .map_err(|e| {
                serialport::Error::new(
                    serialport::ErrorKind::Unknown,
                    format!("failed to spawn HIL serial thread: {e}"),
                )
            })?;

        info!(
            "HIL bridge live: mirroring commands to {port_label} @ {baud} baud (Estop prioritized)"
        );
        Ok(Self {
            shared,
            port_label,
            writer: Some(writer),
        })
    }

    /// False once the serial writer has died. A dead bridge silently discards
    /// every command, so callers that surface HIL status to the operator must
    /// consult this rather than assuming `Some(bridge)` means "connected".
    pub fn is_healthy(&self) -> bool {
        let (lock, _) = &*self.shared;
        lock.lock().map(|out| !out.failed).unwrap_or(false)
    }

    /// Construct a bridge from `OWB_HIL_SERIAL` (and optional `OWB_HIL_BAUD`).
    /// Returns `None` when the env var is unset (sim-only mode) or when the
    /// port/thread fails to start — in the failure case a warning is logged and
    /// the server continues without HIL rather than refusing to boot.
    pub fn from_env() -> Option<Self> {
        let port = std::env::var("OWB_HIL_SERIAL").ok()?;
        let port = port.trim();
        if port.is_empty() {
            return None;
        }
        let baud = std::env::var("OWB_HIL_BAUD")
            .ok()
            .and_then(|v| v.trim().parse::<u32>().ok())
            .unwrap_or(115_200);
        match Self::open(port, baud) {
            Ok(bridge) => Some(bridge),
            Err(e) => {
                warn!("OWB_HIL_SERIAL={port} but the bridge could not start: {e}; continuing sim-only");
                None
            }
        }
    }

    /// Re-encode `cmd` as a JSON frame and enqueue it. Non-blocking and
    /// infallible from the caller's view. Motion frames coalesce into the latest
    /// value; priority frames queue ahead of motion so Estop is never delayed by
    /// the motion stream.
    pub fn forward(
        &self,
        cmd: &SystemCommand,
    ) {
        let frame = match serde_json::to_string(cmd) {
            Ok(f) => f,
            Err(e) => {
                warn!("HIL bridge could not serialize command: {e}");
                return;
            }
        };
        let (lock, cvar) = &*self.shared;
        let mut out = lock.lock().unwrap();
        if out.failed {
            // Never queue into a dead bridge. Discrete commands are logged at
            // error level every time — they are infrequent, and the operator
            // must not believe an Estop reached the robot when it did not.
            if !is_motion(cmd) {
                error!(
                    "HIL bridge to {} is DEAD; discarded {:?} — robot NOT reachable",
                    self.port_label, cmd
                );
            }
            return;
        }
        if is_motion(cmd) {
            // Overwrite any un-sent motion frame — only the newest matters.
            out.motion = Some(frame);
        }
        else {
            if out.priority.len() >= PRIORITY_CAP {
                // Device stalled: drop the oldest to keep the freshest safety
                // command (e.g. the most recent Estop).
                out.priority.pop_front();
                tracing::debug!(
                    "HIL priority queue full for {}; dropped oldest",
                    self.port_label
                );
            }
            out.priority.push_back(frame);
        }
        cvar.notify_one();
    }
}

impl Drop for HilBridge {
    fn drop(&mut self) {
        let (lock, cvar) = &*self.shared;
        if let Ok(mut out) = lock.lock() {
            out.closed = true;
        }
        cvar.notify_all();
        // Join so queued frames — which may include a final Disable/Estop —
        // actually reach the wire before the process exits. The writer only
        // exits once the outbox is drained, and each write is bounded by the
        // 200 ms port timeout, so this cannot hang indefinitely.
        if let Some(writer) = self.writer.take() {
            let _ = writer.join();
        }
    }
}
