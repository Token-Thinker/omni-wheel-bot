//! Command bus node.
//!
//! Owns queue draining, high-rate motion coalescing, command metrics, and the
//! no-board sink executor. The `runtime` module re-exports these APIs for
//! compatibility while this becomes the canonical node boundary.

#[cfg(feature = "net")]
use core::sync::atomic::{AtomicU32, Ordering};

#[cfg(feature = "net")]
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, mutex::Mutex};
#[cfg(feature = "net")]
use embassy_time::{with_timeout, Duration, Instant, Timer};

#[cfg(feature = "net")]
use crate::ctrl::{sysctrl::I2CCommand, CommandExecutor, CommandReply, SystemCommand};

#[cfg(feature = "net")]
// Alpha uses one HTTP/WS worker, so short bursts can wait behind sensor or socket work.
// Keep stale protection, but give live joystick frames enough time to reach the executor.
const MOTION_COMMAND_TTL_MS: u64 = 500;
#[cfg(feature = "net")]
/// Failsafe deadman window: if no *active* motion command executes within this
/// window, the bus forces the wheels to zero. This is independent of any
/// transport (WebSocket, BLE, ESP-NOW), so a silent or hung control link
/// cannot leave a physically actuated robot driving on its last command.
const MOTION_WATCHDOG_MS: u64 = 300;
#[cfg(feature = "net")]
/// Upper bound on how long the idle receive blocks before re-checking the
/// watchdog. Kept well under `MOTION_WATCHDOG_MS` so the failsafe latency is
/// dominated by the window, not the poll cadence.
const MOTION_WATCHDOG_POLL_MS: u64 = 50;
#[cfg(feature = "net")]
const COMMAND_METRICS_LOG_INTERVAL_MS: u64 = 5_000;
#[cfg(feature = "net")]
const COMMAND_IDLE_POLL_MS: u64 = 10;

#[cfg(feature = "net")]
static COMMAND_METRIC_EXECUTED_TOTAL: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "net")]
static COMMAND_METRIC_COALESCED_TOTAL: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "net")]
static COMMAND_METRIC_DROPPED_STALE_TOTAL: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "net")]
static COMMAND_METRIC_DROPPED_QUEUE_FULL_TOTAL: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "net")]
static COMMAND_METRIC_MAX_QUEUE_DEPTH: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "net")]
static COMMAND_METRIC_WATCHDOG_STOPS_TOTAL: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "net")]
static COMMAND_METRIC_LAST_LOOP_DT_MS: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "net")]
/// Counts `CommandExecutor::execute_command` calls that returned `Err`.
/// Non-zero during bring-up means PWM/I2C write failures are occurring.
static COMMAND_METRIC_EXECUTOR_ERRORS_TOTAL: AtomicU32 = AtomicU32::new(0);

#[cfg(feature = "net")]
pub static COMMAND_BUS: Channel<CriticalSectionRawMutex, CommandEnvelope, 16> = Channel::new();
#[cfg(feature = "net")]
pub static COMMAND_REPLY_BUS: Channel<CriticalSectionRawMutex, CommandReply, 4> = Channel::new();
#[cfg(feature = "net")]
pub static COMMAND_QUERY_LOCK: Mutex<CriticalSectionRawMutex, ()> = Mutex::new(());

#[cfg(feature = "net")]
#[derive(Debug)]
pub struct CommandEnvelope {
    pub cmd: SystemCommand,
    pub expects_reply: bool,
    /// Command receive timestamp (milliseconds since boot).
    pub received_ms: u64,
}

#[cfg(feature = "net")]
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize))]
pub struct CommandBusMetricsSnapshot {
    pub executed_total: u64,
    pub coalesced_total: u64,
    pub dropped_stale_total: u64,
    pub dropped_queue_full_total: u64,
    pub max_queue_depth: u64,
    pub last_loop_dt_ms: u64,
    pub watchdog_stops_total: u64,
    /// Number of `execute_command` calls that returned `Err` (I-05).
    /// Non-zero during bring-up indicates PWM / I2C write failures.
    pub executor_errors_total: u64,
}

#[cfg(feature = "net")]
pub fn command_bus_metrics_snapshot() -> CommandBusMetricsSnapshot {
    CommandBusMetricsSnapshot {
        executed_total: COMMAND_METRIC_EXECUTED_TOTAL.load(Ordering::Relaxed) as u64,
        coalesced_total: COMMAND_METRIC_COALESCED_TOTAL.load(Ordering::Relaxed) as u64,
        dropped_stale_total: COMMAND_METRIC_DROPPED_STALE_TOTAL.load(Ordering::Relaxed) as u64,
        dropped_queue_full_total: COMMAND_METRIC_DROPPED_QUEUE_FULL_TOTAL.load(Ordering::Relaxed)
            as u64,
        max_queue_depth: COMMAND_METRIC_MAX_QUEUE_DEPTH.load(Ordering::Relaxed) as u64,
        last_loop_dt_ms: COMMAND_METRIC_LAST_LOOP_DT_MS.load(Ordering::Relaxed) as u64,
        watchdog_stops_total: COMMAND_METRIC_WATCHDOG_STOPS_TOTAL.load(Ordering::Relaxed) as u64,
        executor_errors_total: COMMAND_METRIC_EXECUTOR_ERRORS_TOTAL.load(Ordering::Relaxed) as u64,
    }
}

#[cfg(feature = "net")]
pub fn note_command_queue_full_drop() {
    COMMAND_METRIC_DROPPED_QUEUE_FULL_TOTAL.fetch_add(1, Ordering::Relaxed);
}

#[cfg(feature = "net")]
/// Map the result of `CommandExecutor::execute_command` to a `CommandReply`.
///
/// On `Err`, the fault is surfaced by:
/// 1. Incrementing `COMMAND_METRIC_EXECUTOR_ERRORS_TOTAL` (visible in metrics).
/// 2. Emitting a `tracing::warn!` log line (visible on USB-serial and RTT).
///
/// The reply value on error is `CommandReply::Err`, mapped on the wire to
/// `{"type":"ack","status":"error","message":"executor_error"}` by
/// `gateway.rs`. This is an additive, backward-compatible variant (old clients
/// ignore an unknown reply type); no existing serde tag was changed.
fn map_executor_result<E>(result: Result<CommandReply, E>) -> CommandReply {
    match result {
        Ok(reply) => reply,
        Err(_) => {
            COMMAND_METRIC_EXECUTOR_ERRORS_TOTAL.fetch_add(1, Ordering::Relaxed);
            tracing::warn!(
                target: "owb::cmdbus",
                "executor error: command failed; surfacing as CommandReply::Err \
                 and incrementing error metric (I-05)"
            );
            CommandReply::Err
        }
    }
}

#[cfg(feature = "net")]
fn is_motion_stream_command(cmd: &SystemCommand) -> bool {
    matches!(
        cmd,
        SystemCommand::I(I2CCommand::T { .. } | I2CCommand::Y { .. } | I2CCommand::O { .. })
    )
}

#[cfg(feature = "net")]
/// True for probe commands (`Probe`, `ProbeSpeed`, `PidProbeSpeed`) that drive
/// real wheels but are intentionally excluded from the motion-stream coalescing
/// path (they must never be coalesced or subject to the stale-TTL drop).
fn is_probe_command(cmd: &SystemCommand) -> bool {
    matches!(
        cmd,
        SystemCommand::I(
            I2CCommand::Probe { .. }
                | I2CCommand::ProbeSpeed { .. }
                | I2CCommand::PidProbeSpeed { .. }
        )
    )
}

#[cfg(feature = "net")]
/// True when a motion-stream command requests *nonzero* motion. A zero-speed
/// command is itself a stop, so it disarms the watchdog rather than arming it.
///
/// Also covers probe commands (`Probe`, `ProbeSpeed`, `PidProbeSpeed`): a
/// `Probe` always drives a wheel (bang-bang, no zero state); `ProbeSpeed` /
/// `PidProbeSpeed` arm the watchdog when `speed_norm != 0`.
fn is_active_motion_command(cmd: &SystemCommand) -> bool {
    match cmd {
        SystemCommand::I(I2CCommand::T { speed_norm, .. }) => *speed_norm != 0.0,
        SystemCommand::I(I2CCommand::Y { yaw_rate_deg_s, .. }) => *yaw_rate_deg_s != 0.0,
        SystemCommand::I(I2CCommand::O {
            speed_norm,
            yaw_rate_deg_s,
            ..
        }) => *speed_norm != 0.0 || *yaw_rate_deg_s != 0.0,
        // Probe always drives a wheel (bang-bang direction pulse — no zero state).
        SystemCommand::I(I2CCommand::Probe { .. }) => true,
        // Speed-controlled probes: arm only when a nonzero target is commanded.
        SystemCommand::I(I2CCommand::ProbeSpeed { speed_norm, .. }) => *speed_norm != 0.0,
        SystemCommand::I(I2CCommand::PidProbeSpeed { speed_norm, .. }) => *speed_norm != 0.0,
        _ => false,
    }
}

#[cfg(feature = "net")]
/// True for any command that should arm the deadman watchdog.
///
/// This is a strict superset of `is_motion_stream_command`: it includes both
/// the high-rate T/Y/O motion stream *and* the one-shot probe commands.
///
/// Critically, this is **separate** from the coalescing gate
/// (`is_motion_stream_command`): probe commands bypass the coalescer (each
/// probe must execute independently and is not subject to the stale-TTL drop)
/// but still arm the watchdog so that a connection loss during a probe session
/// stops the wheel within `MOTION_WATCHDOG_MS`.
fn arms_watchdog(cmd: &SystemCommand) -> bool {
    is_motion_stream_command(cmd) || is_probe_command(cmd)
}

#[cfg(feature = "net")]
/// Force wheels to zero if motion is armed and no fresh active command has
/// executed within `MOTION_WATCHDOG_MS`. Disarms after tripping so the stop is
/// issued once per silent interval.
fn trip_motion_watchdog_if_idle<E>(
    executor: &mut E,
    now_ms: u64,
    last_motion_ms: u64,
    motion_active: &mut bool,
) where
    E: CommandExecutor,
{
    if !*motion_active {
        return;
    }
    let age_ms = now_ms.saturating_sub(last_motion_ms);
    if age_ms > MOTION_WATCHDOG_MS {
        let _ = executor.stop_motion();
        *motion_active = false;
        COMMAND_METRIC_WATCHDOG_STOPS_TOTAL.fetch_add(1, Ordering::Relaxed);
        tracing::warn!(
            target: "owb::cmdbus",
            age_ms,
            "motion watchdog tripped: no command within failsafe window, wheels zeroed"
        );
    }
}

#[cfg(feature = "net")]
fn same_motion_channel(
    left: &SystemCommand,
    right: &SystemCommand,
) -> bool {
    matches!(
        (left, right),
        (
            SystemCommand::I(I2CCommand::T { .. }),
            SystemCommand::I(I2CCommand::T { .. })
        ) | (
            SystemCommand::I(I2CCommand::Y { .. }),
            SystemCommand::I(I2CCommand::Y { .. })
        ) | (
            SystemCommand::I(I2CCommand::O { .. }),
            SystemCommand::I(I2CCommand::O { .. })
        )
    )
}

#[cfg(feature = "net")]
fn note_queue_depth(depth: usize) {
    let depth = depth as u32;
    let mut current = COMMAND_METRIC_MAX_QUEUE_DEPTH.load(Ordering::Relaxed);
    while depth > current {
        match COMMAND_METRIC_MAX_QUEUE_DEPTH.compare_exchange_weak(
            current,
            depth,
            Ordering::Relaxed,
            Ordering::Relaxed,
        ) {
            Ok(_) => break,
            Err(next) => current = next,
        }
    }
}

#[cfg(feature = "net")]
fn maybe_log_command_metrics(last_log_ms: &mut u64) {
    let now_ms = Instant::now().as_millis();
    if now_ms.saturating_sub(*last_log_ms) < COMMAND_METRICS_LOG_INTERVAL_MS {
        return;
    }
    *last_log_ms = now_ms;
    let metrics = command_bus_metrics_snapshot();
    tracing::info!(
        target: "owb::cmdbus",
        executed_total = metrics.executed_total,
        coalesced_total = metrics.coalesced_total,
        dropped_stale_total = metrics.dropped_stale_total,
        dropped_queue_full_total = metrics.dropped_queue_full_total,
        max_queue_depth = metrics.max_queue_depth,
        last_loop_dt_ms = metrics.last_loop_dt_ms,
        watchdog_stops_total = metrics.watchdog_stops_total,
        "command bus metrics"
    );
}

#[cfg(feature = "net")]
/// Consume commands from the shared bus and hand them to a board runtime.
pub async fn run_command_bus<E>(executor: &mut E) -> !
where
    E: CommandExecutor,
{
    let receiver = COMMAND_BUS.receiver();
    let mut carry: Option<CommandEnvelope> = None;
    let mut last_exec_ms = Instant::now().as_millis();
    let mut last_log_ms = last_exec_ms;
    // LED-animation timer. This drives `background_tick` (LED effect + PID
    // refresh) and is deliberately NOT reset when a command executes: at
    // joystick rates (≥33 fps) a per-command reset would perpetually push the
    // deadline past `now` and the animation tick would never fire, freezing the
    // strip. Tracking it as an independent wall-clock deadline guarantees the
    // effect advances on its own cadence regardless of incoming command rate.
    let mut next_background_tick_ms = last_exec_ms;
    // Deadman watchdog state: armed once an active motion command executes.
    let mut last_motion_ms = last_exec_ms;
    let mut motion_active = false;

    loop {
        let mut envelope = if let Some(pending) = carry.take() {
            pending
        }
        else if let Ok(pending) = receiver.try_receive() {
            pending
        }
        else if let Some(interval_ms) = executor.background_poll_interval_ms() {
            let now_ms = Instant::now().as_millis();
            if now_ms >= next_background_tick_ms {
                let _ = executor.background_tick();
                // Advance from the prior deadline (not `now`) so the cadence
                // does not drift and a backlog of missed ticks self-corrects.
                next_background_tick_ms = next_background_tick_ms
                    .saturating_add(interval_ms.max(1))
                    .max(now_ms.saturating_sub(interval_ms.max(1)));
                trip_motion_watchdog_if_idle(executor, now_ms, last_motion_ms, &mut motion_active);
                maybe_log_command_metrics(&mut last_log_ms);
                continue;
            }

            // Cap the sleep at both poll intervals and floor it at 1 ms. This is
            // deliberately a chain of caps, not a `clamp`: converting to
            // `clamp(1, MOTION_WATCHDOG_POLL_MS)` would drop the independent
            // COMMAND_IDLE_POLL_MS cap and panic if any poll constant were ever 0.
            // This is P0 watchdog-timing code — keep the semantics explicit.
            #[allow(clippy::manual_clamp)]
            let wait_ms = next_background_tick_ms
                .saturating_sub(now_ms)
                .min(COMMAND_IDLE_POLL_MS)
                .min(MOTION_WATCHDOG_POLL_MS)
                .max(1);
            Timer::after(Duration::from_millis(wait_ms)).await;
            trip_motion_watchdog_if_idle(
                executor,
                Instant::now().as_millis(),
                last_motion_ms,
                &mut motion_active,
            );
            maybe_log_command_metrics(&mut last_log_ms);
            continue;
        }
        else {
            // No background poll work: bound the idle wait so the motor
            // watchdog still fires if the operator goes silent without closing
            // the control link (Wi-Fi stall, backgrounded app, dropped BLE).
            match with_timeout(
                Duration::from_millis(MOTION_WATCHDOG_POLL_MS),
                receiver.receive(),
            )
            .await
            {
                Ok(pending) => pending,
                Err(_) => {
                    trip_motion_watchdog_if_idle(
                        executor,
                        Instant::now().as_millis(),
                        last_motion_ms,
                        &mut motion_active,
                    );
                    maybe_log_command_metrics(&mut last_log_ms);
                    continue;
                }
            }
        };

        note_queue_depth(receiver.len());

        if !envelope.expects_reply && is_motion_stream_command(&envelope.cmd) {
            while let Ok(next) = receiver.try_receive() {
                note_queue_depth(receiver.len());
                if !next.expects_reply && same_motion_channel(&envelope.cmd, &next.cmd) {
                    COMMAND_METRIC_COALESCED_TOTAL.fetch_add(1, Ordering::Relaxed);
                    envelope = next;
                }
                else {
                    carry = Some(next);
                    break;
                }
            }

            let age_ms = Instant::now()
                .as_millis()
                .saturating_sub(envelope.received_ms);
            if age_ms > MOTION_COMMAND_TTL_MS {
                COMMAND_METRIC_DROPPED_STALE_TOTAL.fetch_add(1, Ordering::Relaxed);
                maybe_log_command_metrics(&mut last_log_ms);
                continue;
            }
        }

        // Arm/disarm the deadman watchdog around the command we are about to
        // run: a nonzero motion command (re)arms it; a zero motion command is
        // an explicit stop that disarms it.  `arms_watchdog` covers both the
        // high-rate T/Y/O stream and one-shot probe commands (I-03 safety fix).
        if arms_watchdog(&envelope.cmd) {
            if is_active_motion_command(&envelope.cmd) {
                last_motion_ms = Instant::now().as_millis();
                motion_active = true;
            }
            else {
                motion_active = false;
            }
        }

        // Route through map_executor_result: executor Err increments the error
        // metric and emits a tracing::warn log line rather than being silently
        // masked as CommandReply::Ok (I-05 fix).
        let reply = map_executor_result(executor.execute_command(envelope.cmd));
        COMMAND_METRIC_EXECUTED_TOTAL.fetch_add(1, Ordering::Relaxed);
        let now_ms = Instant::now().as_millis();
        COMMAND_METRIC_LAST_LOOP_DT_MS.store(
            now_ms.saturating_sub(last_exec_ms) as u32,
            Ordering::Relaxed,
        );
        last_exec_ms = now_ms;
        // Evaluate the deadman watchdog on the execute path too, not just the
        // idle branches. A sustained stream of non-motion commands (LED effects,
        // sensor reads) keeps the queue non-empty, so the loop never reaches an
        // idle branch and the failsafe would never be checked — leaving the
        // wheels driving on the last motion command indefinitely. This is the
        // same starvation class as the LED-tick bug fixed inline just below.
        //
        // Safe to run unconditionally: the arm/disarm block above already set
        // `last_motion_ms = now_ms` for an active motion command, so that case
        // has `age_ms == 0` and cannot trip.
        trip_motion_watchdog_if_idle(executor, now_ms, last_motion_ms, &mut motion_active);
        // LED-animation tick: drive it from the independent wall-clock deadline
        // rather than resetting the deadline on every command. A sustained
        // joystick stream keeps the loop draining commands without ever
        // reaching the idle branch, so we fire the background tick inline here
        // whenever its own deadline has elapsed. Crucially we DO NOT push the
        // deadline out past `now` on command execution — that was the
        // starvation bug that froze the strip at high command rates.
        if let Some(interval_ms) = executor.background_poll_interval_ms() {
            if now_ms >= next_background_tick_ms {
                let _ = executor.background_tick();
                next_background_tick_ms = next_background_tick_ms
                    .saturating_add(interval_ms.max(1))
                    .max(now_ms.saturating_sub(interval_ms.max(1)));
            }
        }
        maybe_log_command_metrics(&mut last_log_ms);
        if envelope.expects_reply {
            // try_send, never send().await: a cancelled query (client disconnect
            // between send and receive — routine on WiFi) leaves an orphan reply
            // queued. With a blocking send, four such orphans fill this
            // capacity-4 channel and the bus stalls forever — motion commands
            // stop executing *and* the deadman watchdog above stops running, so
            // the wheels keep driving. Dropping a reply only fails one query,
            // which the requester already handles via timeout.
            if COMMAND_REPLY_BUS.try_send(reply).is_err() {
                tracing::warn!(
                    target: "owb::cmdbus",
                    "reply bus full; dropped reply (stale queries pending?)"
                );
            }
        }
    }
}

#[cfg(feature = "net")]
struct DropExecutor {
    log_drops: bool,
}

#[cfg(feature = "net")]
impl CommandExecutor for DropExecutor {
    type Error = ();

    fn execute_command(
        &mut self,
        cmd: SystemCommand,
    ) -> Result<CommandReply, Self::Error> {
        if self.log_drops {
            match cmd {
                SystemCommand::I(_) => {
                    tracing::debug!("dropped motion command (no board runtime)");
                }
                SystemCommand::L(_) => {
                    tracing::debug!("dropped LED command (no board runtime)");
                }
                SystemCommand::P(_) => {
                    tracing::debug!("dropped power command (no board runtime)");
                }
                SystemCommand::A(_) => {
                    tracing::debug!("dropped autonomy command (no board runtime)");
                }
            }
        }

        Ok(CommandReply::Ok)
    }
}

#[cfg(feature = "net")]
#[embassy_executor::task]
async fn command_sink_task(log_drops: bool) -> ! {
    if log_drops {
        tracing::info!("command sink enabled (board runtime not attached)");
    }
    let mut executor = DropExecutor { log_drops };
    run_command_bus(&mut executor).await
}

#[cfg(feature = "net")]
/// Drain the shared command bus when a binary has not attached a real board
/// executor yet. This keeps websocket producers non-blocking while the board
/// runtime is still under construction.
///
/// # Panics
///
/// Panics if the task pool is exhausted (i.e. `spawn_command_sink` has already
/// been called and the sink task is already running). The command bus requires
/// exactly one drain loop; a duplicate spawn is a programming error.
pub fn spawn_command_sink(
    spawner: embassy_executor::Spawner,
    log_drops: bool,
) {
    // Surface spawn failures rather than silently dropping them (I-05).
    // A failed spawn here means the shared COMMAND_BUS will never be drained
    // and all WebSocket / BLE command producers will stall.
    match command_sink_task(log_drops) {
        Ok(token) => spawner.spawn(token),
        Err(_) => {
            panic!(
                "command_sink_task: failed to spawn — task pool exhausted. \
                 Did spawn_command_sink get called twice?"
            );
        }
    }
}

#[cfg(all(test, feature = "net"))]
mod watchdog_tests {
    //! Host-side coverage for the transport-independent deadman failsafe.
    //!
    //! The async bus loop reads "now" from the monotonic clock; the watchdog
    //! *decision* is factored out as a pure function so the safety-critical
    //! arm/disarm/trip behaviour can be asserted deterministically without an
    //! executor, a time driver, or hardware.
    use super::*;
    use crate::ctrl::{sysctrl::I2CCommand, CommandExecutor, CommandReply, SystemCommand};

    /// Executor that records how many times the bus asked it to stop motion
    /// (`stop_motion` issues a zero-speed translate by default).
    #[derive(Default)]
    struct RecordingExecutor {
        stop_calls: u32,
    }

    impl CommandExecutor for RecordingExecutor {
        type Error = ();

        fn execute_command(
            &mut self,
            cmd: SystemCommand,
        ) -> Result<CommandReply, Self::Error> {
            if matches!(
                cmd,
                SystemCommand::I(I2CCommand::T { speed_norm, .. }) if speed_norm == 0.0
            ) {
                self.stop_calls += 1;
            }
            Ok(CommandReply::Ok)
        }
    }

    #[test]
    fn watchdog_does_not_fire_when_disarmed() {
        let mut ex = RecordingExecutor::default();
        let mut active = false;
        // Even with a huge idle age, a disarmed watchdog must not touch motors.
        trip_motion_watchdog_if_idle(&mut ex, 10_000, 0, &mut active);
        assert_eq!(ex.stop_calls, 0);
    }

    #[test]
    fn watchdog_holds_within_window() {
        let mut ex = RecordingExecutor::default();
        let mut active = true;
        // age = 100ms, under MOTION_WATCHDOG_MS (300ms): keep driving.
        trip_motion_watchdog_if_idle(&mut ex, 1_100, 1_000, &mut active);
        assert_eq!(ex.stop_calls, 0);
        assert!(active, "watchdog must stay armed inside the window");
    }

    #[test]
    fn watchdog_trips_once_after_window_then_disarms() {
        let mut ex = RecordingExecutor::default();
        let mut active = true;
        // age = 400ms, past the 300ms window: zero the wheels and disarm.
        trip_motion_watchdog_if_idle(&mut ex, 1_400, 1_000, &mut active);
        assert_eq!(ex.stop_calls, 1);
        assert!(!active);
        // Still idle later, but already disarmed: must not re-issue the stop.
        trip_motion_watchdog_if_idle(&mut ex, 5_000, 1_000, &mut active);
        assert_eq!(ex.stop_calls, 1);
    }

    // ── I-02: connection-loss stop ─────────────────────────────────────────
    // Regression coverage: a translate command arms the watchdog; after
    // MOTION_WATCHDOG_MS of silence (WebSocket drop / network loss), the
    // watchdog fires and zeroes the wheels.  The mechanism already works for
    // T/Y/O commands; this test records that behaviour as an explicit
    // regression guard and documents the connection-loss stop contract.
    #[test]
    fn connection_loss_stops_motion_after_watchdog_window() {
        let motion_cmd = SystemCommand::I(I2CCommand::T {
            direction_deg: 45.0,
            speed_norm: 0.4,
        });

        // A nonzero T command is both a motion-stream command and an active
        // motion command, so the bus loop will arm the watchdog for it.
        assert!(is_motion_stream_command(&motion_cmd));
        assert!(is_active_motion_command(&motion_cmd));

        let mut ex = RecordingExecutor::default();
        let mut motion_active = false;
        let t0_ms: u64 = 1_000;

        // Simulate the bus arming the watchdog after executing the command.
        if is_active_motion_command(&motion_cmd) {
            motion_active = true;
        }

        // Inside the window: stop must NOT be issued.
        trip_motion_watchdog_if_idle(
            &mut ex,
            t0_ms + MOTION_WATCHDOG_MS - 1,
            t0_ms,
            &mut motion_active,
        );
        assert_eq!(
            ex.stop_calls, 0,
            "watchdog must not fire inside the 300 ms window"
        );
        assert!(motion_active, "watchdog must stay armed inside the window");

        // Past the window (connection lost — no commands arrived): stop MUST fire.
        trip_motion_watchdog_if_idle(
            &mut ex,
            t0_ms + MOTION_WATCHDOG_MS + 1,
            t0_ms,
            &mut motion_active,
        );
        assert_eq!(
            ex.stop_calls, 1,
            "watchdog must zero wheels after connection-loss silence"
        );
        assert!(!motion_active, "watchdog must disarm after firing");
    }

    #[test]
    fn active_motion_arms_only_on_nonzero_intent() {
        let t_move = SystemCommand::I(I2CCommand::T {
            direction_deg: 90.0,
            speed_norm: 0.5,
        });
        let t_stop = SystemCommand::I(I2CCommand::T {
            direction_deg: 0.0,
            speed_norm: 0.0,
        });
        let y_spin = SystemCommand::I(I2CCommand::Y {
            yaw_rate_deg_s: 12.0,
            orientation_deg: None,
        });
        let o_idle = SystemCommand::I(I2CCommand::O {
            direction_deg: 0.0,
            speed_norm: 0.0,
            yaw_rate_deg_s: 0.0,
            orientation_deg: None,
        });
        assert!(is_active_motion_command(&t_move));
        assert!(!is_active_motion_command(&t_stop));
        assert!(is_active_motion_command(&y_spin));
        assert!(!is_active_motion_command(&o_idle));
        assert!(!is_active_motion_command(&SystemCommand::I(
            I2CCommand::Enable
        )));
    }

    // ── I-03: probe commands arm the deadman watchdog ──────────────────────
    // Failing tests (TDD step): written before the fix.  Before the fix,
    // `is_active_motion_command` returns false for ProbeSpeed/PidProbeSpeed
    // (falls through to `_ => false`) and the watchdog arm gate
    // (`is_motion_stream_command`) also returns false for probe commands,
    // leaving the watchdog permanently disarmed during probe-only sessions.
    // Both assertions below fail until the fix is applied.

    #[test]
    fn probe_watchdog_probe_speed_is_active_motion() {
        // ProbeSpeed with nonzero speed drives a real wheel — it must be
        // treated as active motion so the deadman watchdog can arm.
        let cmd = SystemCommand::I(I2CCommand::ProbeSpeed {
            wheel: 0,
            speed_norm: 0.5,
        });
        assert!(
            is_active_motion_command(&cmd),
            "ProbeSpeed at nonzero speed must be active motion so the watchdog arm path fires"
        );
    }

    #[test]
    fn probe_watchdog_pid_probe_speed_is_active_motion() {
        let cmd = SystemCommand::I(I2CCommand::PidProbeSpeed {
            wheel: 1,
            speed_norm: 0.5,
        });
        assert!(
            is_active_motion_command(&cmd),
            "PidProbeSpeed at nonzero speed must be active motion so the watchdog arm path fires"
        );
    }

    // Additional tests verifying the full arm path and the critical disarm trap.

    /// Probe commands must arm the watchdog (`arms_watchdog` returns true) so the
    /// connection-loss failsafe covers probe-only sessions.  This is the key safety
    /// assertion: before the I-03 fix `arms_watchdog` would return false for probes,
    /// leaving the watchdog permanently disarmed.
    #[test]
    fn probe_watchdog_arms_watchdog_returns_true_for_probe_commands() {
        let probe = SystemCommand::I(I2CCommand::Probe {
            wheel: 0,
            forward: true,
        });
        let probe_speed = SystemCommand::I(I2CCommand::ProbeSpeed {
            wheel: 1,
            speed_norm: 0.5,
        });
        let pid_probe = SystemCommand::I(I2CCommand::PidProbeSpeed {
            wheel: 2,
            speed_norm: 0.5,
        });

        assert!(arms_watchdog(&probe), "Probe must arm the watchdog");
        assert!(
            arms_watchdog(&probe_speed),
            "ProbeSpeed must arm the watchdog"
        );
        assert!(
            arms_watchdog(&pid_probe),
            "PidProbeSpeed must arm the watchdog"
        );

        // Regression: motion-stream commands must still arm the watchdog.
        let t_cmd = SystemCommand::I(I2CCommand::T {
            direction_deg: 0.0,
            speed_norm: 0.5,
        });
        assert!(
            arms_watchdog(&t_cmd),
            "T must still arm the watchdog (regression)"
        );

        // Non-motion commands must NOT arm the watchdog.
        assert!(
            !arms_watchdog(&SystemCommand::I(I2CCommand::Enable)),
            "Enable must not arm the watchdog"
        );
    }

    /// Simulate the exact bus-loop arm/disarm logic for a ProbeSpeed command to
    /// verify motion_active becomes true (not false — the disarm trap).
    ///
    /// Before the fix the outer gate (`is_motion_stream_command`) returned false
    /// for ProbeSpeed, so the arm block was never entered and the watchdog was
    /// never armed.  After the fix the gate is `arms_watchdog`, which returns
    /// true for ProbeSpeed, and `is_active_motion_command` returns true for
    /// nonzero speed — so `motion_active` becomes true.
    #[test]
    fn probe_watchdog_bus_loop_arms_on_probe_speed() {
        let cmd = SystemCommand::I(I2CCommand::ProbeSpeed {
            wheel: 0,
            speed_norm: 0.5,
        });
        let mut motion_active = false;

        // Mirror the bus-loop arm/disarm logic.
        if arms_watchdog(&cmd) {
            motion_active = is_active_motion_command(&cmd);
        }

        assert!(
            motion_active,
            "ProbeSpeed must set motion_active=true in the bus loop — the disarm trap is closed"
        );
    }

    #[test]
    fn probe_watchdog_fires_after_probe_speed_silence() {
        // End-to-end: probe arms watchdog → silence → stop_motion fires.
        let cmd = SystemCommand::I(I2CCommand::ProbeSpeed {
            wheel: 0,
            speed_norm: 0.5,
        });
        let mut ex = RecordingExecutor::default();
        let mut motion_active = false;
        let t0_ms: u64 = 1_000;

        // Arm the watchdog (simulates the bus loop after the fix).
        if arms_watchdog(&cmd) && is_active_motion_command(&cmd) {
            motion_active = true;
        }

        // After MOTION_WATCHDOG_MS of silence, stop_motion must fire.
        trip_motion_watchdog_if_idle(
            &mut ex,
            t0_ms + MOTION_WATCHDOG_MS + 1,
            t0_ms,
            &mut motion_active,
        );
        assert_eq!(
            ex.stop_calls, 1,
            "watchdog must stop motors after ProbeSpeed silence (connection-loss during probe)"
        );
    }

    #[test]
    fn probe_watchdog_fires_after_pid_probe_speed_silence() {
        let cmd = SystemCommand::I(I2CCommand::PidProbeSpeed {
            wheel: 2,
            speed_norm: 0.5,
        });
        let mut ex = RecordingExecutor::default();
        let mut motion_active = false;
        let t0_ms: u64 = 2_000;

        if arms_watchdog(&cmd) && is_active_motion_command(&cmd) {
            motion_active = true;
        }

        trip_motion_watchdog_if_idle(
            &mut ex,
            t0_ms + MOTION_WATCHDOG_MS + 1,
            t0_ms,
            &mut motion_active,
        );
        assert_eq!(
            ex.stop_calls, 1,
            "watchdog must stop motors after PidProbeSpeed silence"
        );
    }

    /// ProbeSpeed at zero speed must NOT arm the watchdog (no active motion).
    #[test]
    fn probe_watchdog_zero_speed_probe_does_not_arm() {
        let cmd = SystemCommand::I(I2CCommand::ProbeSpeed {
            wheel: 0,
            speed_norm: 0.0,
        });
        assert!(
            !is_active_motion_command(&cmd),
            "ProbeSpeed at zero speed must not be active motion"
        );
    }

    /// Non-motion commands (LED, Enable) must never arm the watchdog.
    #[test]
    fn probe_watchdog_non_motion_commands_do_not_arm() {
        assert!(!arms_watchdog(&SystemCommand::I(I2CCommand::Enable)));
        assert!(!arms_watchdog(&SystemCommand::I(I2CCommand::Disable)));
        assert!(!arms_watchdog(&SystemCommand::I(I2CCommand::ReadIMU)));
    }

    // ── I-05: executor errors surfaced, not masked as Ok ──────────────────
    // Before the fix: `executor.execute_command(cmd).unwrap_or(CommandReply::Ok)`
    // silently converted any `Err` to `Ok`, hiding PWM/I2C write faults during
    // bring-up.  The fix routes all results through `map_executor_result` which
    // increments `COMMAND_METRIC_EXECUTOR_ERRORS_TOTAL` and emits a tracing::warn
    // log line on `Err`.
    //
    // TDD verification: the test below was written and run against the buggy
    // `unwrap_or` path first (failing: counter stayed at `before`), then the
    // fix was applied and the test passed.

    /// A mock executor that always returns `Err` — simulates a PWM/I2C fault.
    struct FailingExecutor;
    impl CommandExecutor for FailingExecutor {
        type Error = &'static str;

        fn execute_command(
            &mut self,
            _cmd: SystemCommand,
        ) -> Result<CommandReply, Self::Error> {
            Err("motor write failed: I2C NACK")
        }
    }

    #[test]
    fn executor_error_increments_error_metric() {
        let before = COMMAND_METRIC_EXECUTOR_ERRORS_TOTAL.load(Ordering::Relaxed);

        let result = FailingExecutor.execute_command(SystemCommand::I(I2CCommand::Enable));
        // map_executor_result must surface Err as CommandReply::Err AND
        // increment the error metric — neither the old unwrap_or(Ok) path did.
        let reply = map_executor_result(result);

        let after = COMMAND_METRIC_EXECUTOR_ERRORS_TOTAL.load(Ordering::Relaxed);
        assert_eq!(
            after,
            before + 1,
            "executor Err must increment COMMAND_METRIC_EXECUTOR_ERRORS_TOTAL, not be silently \
             masked as CommandReply::Ok"
        );
        assert!(
            matches!(reply, CommandReply::Err),
            "executor Err must surface as CommandReply::Err, not CommandReply::Ok"
        );
    }
}
