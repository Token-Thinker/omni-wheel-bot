//! Top-down 2D simulation engine for the `/sim` view.
//!
//! Integrates a simplified differential-drive model of the v1-alpha two-wheel
//! mode. Even though the physical robot is a three-wheel omni, the closed-loop
//! straightness work (WheelTrim + HeadingHold) was baked around W1/W2 as the
//! drive pair with W3 idle. This engine reproduces that pair so we can replay
//! the closed-loop behaviour in software.
//!
//! All state is owned by `SimEngine` behind a `Mutex`. The engine is driven by
//! a tokio task at 50 Hz (see `main.rs`). Telemetry is sampled at 10 Hz by the
//! WS task that publishes `/sim/state` messages.
//!
//! Configurable knobs:
//! - `slip_ratio` per wheel (0.0 = no slip, 0.5 = 50% commanded velocity lost)
//! - `stiction_threshold_norm` (commanded magnitude below which the wheel does
//!   not move). Default 0.55 matches the v1-alpha profile.
//! - `wheel_trim` multipliers (mirrors the firmware `WheelTrim` command).
//! - `heading_hold` (mirrors the firmware `HeadingHold` straightness loop).

use std::{collections::VecDeque, time::Instant};

use serde::Serialize;

/// Distance between the two drive wheels (track width), metres.
const TRACK_WIDTH_M: f32 = 0.39;
/// Top linear speed at `speed_norm = 1.0`, metres/second.
const MAX_LINEAR_SPEED_M_S: f32 = 0.9;
/// Top yaw rate at `yaw_norm = 1.0`, degrees/second.
const MAX_YAW_RATE_DEG_S: f32 = 180.0;
/// Encoder counts per metre travelled by the wheel rim. Arbitrary but
/// consistent with what the firmware reports per tick.
const ENCODER_COUNTS_PER_M: f32 = 1024.0 / (2.0 * std::f32::consts::PI * 0.148);
/// Wheel actuator response (first-order lag). Higher = snappier.
const WHEEL_RESPONSE_PER_S: f32 = 6.0;
/// Default stiction threshold baked into the v1-alpha profile.
const DEFAULT_STICTION: f32 = 0.55;
/// Default per-wheel slip ratios.
const DEFAULT_SLIP: [f32; 2] = [0.0, 0.0];
/// History buffer capacity (10 Hz * 60 s).
const HISTORY_CAPACITY: usize = 600;
/// Deadband below which a commanded |speed_norm| is treated as "no input"
/// for the purpose of detecting an idle→forward transition. Matches the
/// default `motor_deadband_norm` in `DEFAULT_DRIVE_TUNING`.
const STICTION_KICK_DEADBAND_NORM: f32 = 0.15;

/// A single recorded pose used to draw the trail in the viewer.
#[derive(Clone, Copy, Debug, Serialize)]
pub struct PoseSample {
    pub t: f32,
    pub x: f32,
    pub y: f32,
    pub yaw: f32,
}

/// Snapshot of the engine state pushed over `/sim/state`.
#[derive(Clone, Debug, Serialize)]
pub struct SimState {
    pub x_m: f32,
    pub y_m: f32,
    pub yaw_rad: f32,
    pub v_left: f32,
    pub v_right: f32,
    pub encoder_delta_total: [i32; 2],
    pub encoder_total: [i32; 2],
    pub trim: [f32; 3],
    pub heading_hold: HeadingHoldState,
    pub slip: [f32; 2],
    pub stiction: f32,
    pub stiction_kick_ms: u16,
    pub stiction_kick_boost: f32,
    pub stiction_kick_active: bool,
    pub enabled: bool,
    pub paused: bool,
    pub tick: u64,
    pub history: Vec<PoseSample>,
    pub replay: Option<ReplayStatus>,
}

#[derive(Clone, Copy, Debug, Serialize)]
pub struct HeadingHoldState {
    pub enable: bool,
    pub kp: f32,
}

#[derive(Clone, Debug, Serialize)]
pub struct ReplayStatus {
    pub path: String,
    pub frame: usize,
    pub total: usize,
}

/// Commanded inputs the firmware mirror dispatches to us.
#[derive(Clone, Copy, Debug, Default)]
pub struct DriveInputs {
    /// Translate vector heading (degrees, body frame).
    pub direction_deg: f32,
    /// Forward speed magnitude (-1.0..=1.0).
    pub speed_norm: f32,
    /// Yaw rate command (deg/s).
    pub yaw_rate_deg_s: f32,
}

/// A replay step pre-baked from a HIL log.
#[derive(Clone, Copy, Debug)]
struct ReplayFrame {
    dt_s: f32,
    enc_delta: [i32; 2],
}

pub struct SimEngine {
    // Pose (world frame, metres / radians)
    x_m: f32,
    y_m: f32,
    yaw_rad: f32,

    // Last commanded translate/yaw (body frame inputs).
    inputs: DriveInputs,
    // Filtered wheel command (after stiction + slip), in m/s linear at the wheel.
    v_left: f32,
    v_right: f32,
    // Targeted wheel velocities (before lag/stiction/slip), m/s.
    target_left: f32,
    target_right: f32,

    enabled: bool,
    paused: bool,

    // Per-wheel slip ratios for [W1=left, W2=right].
    slip: [f32; 2],
    stiction_norm: f32,

    // Stiction-kick configuration mirrored from the firmware. `kick_ms = 0`
    // disables the kick. `kick_active_remaining_s` counts down from
    // `kick_ms / 1000` while a kick window is active; the boost is applied to
    // the commanded forward magnitude during that window.
    kick_ms: u16,
    kick_boost_norm: f32,
    kick_active_remaining_s: f32,
    last_speed_norm_sign: f32,

    // Mirrors the firmware multipliers (W1, W2, W3).
    trim: [f32; 3],
    heading_hold: HeadingHoldState,
    heading_hold_target_yaw_rad: f32,

    // Encoder bookkeeping.
    enc_total: [i32; 2],
    enc_delta_accum: [i32; 2],

    tick: u64,
    started: Instant,
    history: VecDeque<PoseSample>,

    // Replay buffer (consumed one frame per tick when present).
    replay: Option<Vec<ReplayFrame>>,
    replay_path: Option<String>,
    replay_cursor: usize,
}

impl Default for SimEngine {
    fn default() -> Self { Self::new() }
}

impl SimEngine {
    pub fn new() -> Self {
        Self {
            x_m: 0.0,
            y_m: 0.0,
            yaw_rad: 0.0,
            inputs: DriveInputs::default(),
            v_left: 0.0,
            v_right: 0.0,
            target_left: 0.0,
            target_right: 0.0,
            enabled: true,
            paused: false,
            slip: DEFAULT_SLIP,
            stiction_norm: DEFAULT_STICTION,
            kick_ms: 0,
            kick_boost_norm: 0.0,
            kick_active_remaining_s: 0.0,
            last_speed_norm_sign: 0.0,
            trim: [1.0, 1.0, 1.0],
            heading_hold: HeadingHoldState {
                enable: false,
                kp: 0.0,
            },
            heading_hold_target_yaw_rad: 0.0,
            enc_total: [0; 2],
            enc_delta_accum: [0; 2],
            tick: 0,
            started: Instant::now(),
            history: VecDeque::with_capacity(HISTORY_CAPACITY),
            replay: None,
            replay_path: None,
            replay_cursor: 0,
        }
    }

    /// Reset pose, encoders, and history but keep tuning knobs.
    pub fn reset_pose(&mut self) {
        self.x_m = 0.0;
        self.y_m = 0.0;
        self.yaw_rad = 0.0;
        self.v_left = 0.0;
        self.v_right = 0.0;
        self.target_left = 0.0;
        self.target_right = 0.0;
        self.inputs = DriveInputs::default();
        self.enc_total = [0; 2];
        self.enc_delta_accum = [0; 2];
        self.history.clear();
        self.heading_hold_target_yaw_rad = 0.0;
        self.kick_active_remaining_s = 0.0;
        self.last_speed_norm_sign = 0.0;
        self.replay = None;
        self.replay_path = None;
        self.replay_cursor = 0;
        self.tick = 0;
        self.started = Instant::now();
    }

    pub fn set_paused(
        &mut self,
        paused: bool,
    ) -> bool {
        self.paused = paused;
        self.paused
    }

    pub fn toggle_paused(&mut self) -> bool {
        self.paused = !self.paused;
        self.paused
    }

    pub fn set_enabled(
        &mut self,
        enabled: bool,
    ) {
        self.enabled = enabled;
        if !enabled {
            self.inputs = DriveInputs::default();
            self.target_left = 0.0;
            self.target_right = 0.0;
        }
    }

    pub fn enabled(&self) -> bool { self.enabled }

    pub fn set_translate(
        &mut self,
        direction_deg: f32,
        speed_norm: f32,
    ) {
        let clamped = speed_norm.clamp(-1.0, 1.0);
        self.maybe_arm_kick(clamped);
        self.inputs.direction_deg = direction_deg;
        self.inputs.speed_norm = clamped;
        self.recompute_targets();
        if !self.heading_hold.enable {
            self.heading_hold_target_yaw_rad = self.yaw_rad;
        }
    }

    /// Arm or refresh the stiction-kick window when the commanded
    /// `speed_norm` rises above the deadband from rest, or when its sign
    /// flips. Mirrors the firmware behaviour in `RealBoardRuntime`.
    fn maybe_arm_kick(
        &mut self,
        new_speed_norm: f32,
    ) {
        if self.kick_ms == 0 || self.kick_boost_norm <= 0.0 {
            self.last_speed_norm_sign = if new_speed_norm == 0.0 {
                0.0
            }
            else if new_speed_norm > 0.0 {
                1.0
            }
            else {
                -1.0
            };
            self.kick_active_remaining_s = 0.0;
            return;
        }
        let new_abs = new_speed_norm.abs();
        let new_sign = if new_speed_norm == 0.0 {
            0.0
        }
        else if new_speed_norm > 0.0 {
            1.0
        }
        else {
            -1.0
        };
        let prev_sign = self.last_speed_norm_sign;
        let crossed_up = prev_sign == 0.0 && new_abs > STICTION_KICK_DEADBAND_NORM;
        let direction_flipped = new_abs > STICTION_KICK_DEADBAND_NORM
            && prev_sign != 0.0
            && new_sign != 0.0
            && new_sign != prev_sign;
        if crossed_up || direction_flipped {
            self.kick_active_remaining_s = f32::from(self.kick_ms) * 1e-3;
        }
        self.last_speed_norm_sign = if new_abs <= STICTION_KICK_DEADBAND_NORM {
            0.0
        }
        else {
            new_sign
        };
    }

    /// Mirror the firmware `StictionKick` runtime command. Currently invoked
    /// from tests; the WS/HTTP wiring in `sim_routes.rs` will plumb it through
    /// when the UI agent's refactor lands.
    #[allow(dead_code)]
    pub fn set_stiction_kick(
        &mut self,
        ms: u16,
        boost: f32,
    ) {
        self.kick_ms = ms.min(1000);
        self.kick_boost_norm = boost.clamp(0.0, 0.5);
        if self.kick_ms == 0 || self.kick_boost_norm == 0.0 {
            self.kick_active_remaining_s = 0.0;
        }
        self.recompute_targets();
    }

    pub fn set_yaw(
        &mut self,
        yaw_rate_deg_s: f32,
    ) {
        self.inputs.yaw_rate_deg_s = yaw_rate_deg_s;
        self.recompute_targets();
    }

    pub fn set_wheel_trim(
        &mut self,
        w1: f32,
        w2: f32,
        w3: f32,
    ) {
        self.trim = [w1.clamp(0.5, 1.5), w2.clamp(0.5, 1.5), w3.clamp(0.5, 1.5)];
        self.recompute_targets();
    }

    pub fn set_heading_hold(
        &mut self,
        enable: bool,
        kp: f32,
    ) {
        self.heading_hold = HeadingHoldState {
            enable,
            kp: kp.clamp(0.0, 5.0),
        };
        if enable {
            self.heading_hold_target_yaw_rad = self.yaw_rad;
        }
    }

    pub fn set_slip(
        &mut self,
        w1: f32,
        w2: f32,
    ) {
        self.slip = [w1.clamp(0.0, 0.95), w2.clamp(0.0, 0.95)];
    }

    pub fn set_stiction(
        &mut self,
        threshold_norm: f32,
    ) {
        self.stiction_norm = threshold_norm.clamp(0.0, 0.95);
    }

    /// Convert direction/speed/yaw into per-wheel linear m/s targets.
    fn recompute_targets(&mut self) {
        if !self.enabled {
            self.target_left = 0.0;
            self.target_right = 0.0;
            return;
        }
        // For the diff-drive sim, forward component is `cos(direction)` so 0°
        // means "straight forward". This matches the host-sim convention where
        // 0° is forward.
        let dir = self.inputs.direction_deg.to_radians();
        let effective_speed_norm = if self.kick_active_remaining_s > 0.0
            && self.kick_boost_norm > 0.0
            && self.inputs.speed_norm.abs() > 0.0
        {
            let sign = if self.inputs.speed_norm >= 0.0 {
                1.0
            }
            else {
                -1.0
            };
            (self.inputs.speed_norm + sign * self.kick_boost_norm).clamp(-1.0, 1.0)
        }
        else {
            self.inputs.speed_norm
        };
        let forward_norm = dir.cos() * effective_speed_norm;
        let lateral_norm = dir.sin() * effective_speed_norm;
        // Lateral motion isn't reachable with two parallel wheels — translate
        // it into a softened forward bias so the operator still gets feedback
        // when nudging sideways.
        let forward = (forward_norm + 0.15 * lateral_norm) * MAX_LINEAR_SPEED_M_S;
        let yaw_norm = (self.inputs.yaw_rate_deg_s / MAX_YAW_RATE_DEG_S).clamp(-1.0, 1.0);
        let yaw_contrib = yaw_norm * (TRACK_WIDTH_M / 2.0) * MAX_YAW_RATE_DEG_S.to_radians();

        let left = (forward - yaw_contrib) * self.trim[0];
        let right = (forward + yaw_contrib) * self.trim[1];
        self.target_left = left;
        self.target_right = right;
    }

    /// Advance the engine by `dt_s` seconds.
    pub fn tick(
        &mut self,
        dt_s: f32,
    ) {
        if self.paused {
            return;
        }

        // Replay overrides drive integration with recorded encoder deltas.
        if let Some(frames) = self.replay.as_ref() {
            if self.replay_cursor >= frames.len() {
                // End of replay — keep the buffer around for the status panel.
                self.maybe_record_history();
                self.tick = self.tick.saturating_add(1);
                return;
            }
            let frame = frames[self.replay_cursor];
            self.replay_cursor += 1;
            self.apply_replay_frame(frame);
            self.maybe_record_history();
            self.tick = self.tick.saturating_add(1);
            return;
        }

        // Drain the stiction-kick window before computing targets so the
        // boost expires exactly on schedule.
        let kick_was_active = self.kick_active_remaining_s > 0.0;
        if kick_was_active {
            self.kick_active_remaining_s = (self.kick_active_remaining_s - dt_s).max(0.0);
        }
        if kick_was_active {
            // Re-apply targets every tick while the kick is in flight (and
            // once on the tick it expires) so the boost is reflected even
            // when set_translate isn't called again, and is removed promptly.
            self.recompute_targets();
        }

        // First-order lag toward target velocity.
        let alpha = (WHEEL_RESPONSE_PER_S * dt_s).clamp(0.0, 1.0);
        self.v_left += (self.target_left - self.v_left) * alpha;
        self.v_right += (self.target_right - self.v_right) * alpha;

        // Heading-hold correction (firmware mirrors this in two-wheel mode).
        if self.heading_hold.enable && self.heading_hold.kp > 0.0 {
            let err = wrap_angle(self.heading_hold_target_yaw_rad - self.yaw_rad);
            let correction = self.heading_hold.kp * err * MAX_LINEAR_SPEED_M_S * 0.5;
            self.v_left -= correction;
            self.v_right += correction;
        }

        // Apply stiction + slip when converting commanded → actual.
        let actual_left = self.apply_friction(self.v_left, self.slip[0]);
        let actual_right = self.apply_friction(self.v_right, self.slip[1]);

        // Diff-drive kinematics in world frame.
        let v = 0.5 * (actual_left + actual_right);
        let omega = (actual_right - actual_left) / TRACK_WIDTH_M;
        self.yaw_rad = wrap_angle(self.yaw_rad + omega * dt_s);
        self.x_m += v * self.yaw_rad.cos() * dt_s;
        self.y_m += v * self.yaw_rad.sin() * dt_s;

        // Encoder bookkeeping (m * counts/m → counts).
        let d_left = actual_left * dt_s;
        let d_right = actual_right * dt_s;
        let delta_left = (d_left * ENCODER_COUNTS_PER_M) as i32;
        let delta_right = (d_right * ENCODER_COUNTS_PER_M) as i32;
        self.enc_total[0] = self.enc_total[0].saturating_add(delta_left);
        self.enc_total[1] = self.enc_total[1].saturating_add(delta_right);
        self.enc_delta_accum[0] = self.enc_delta_accum[0].saturating_add(delta_left);
        self.enc_delta_accum[1] = self.enc_delta_accum[1].saturating_add(delta_right);

        self.maybe_record_history();
        self.tick = self.tick.saturating_add(1);
    }

    fn apply_friction(
        &self,
        commanded_m_s: f32,
        slip_ratio: f32,
    ) -> f32 {
        let normalized = commanded_m_s / MAX_LINEAR_SPEED_M_S;
        if normalized.abs() < self.stiction_norm * 0.5 {
            // Below half-stiction we're firmly stuck.
            return 0.0;
        }
        // Soft cubic blend between 0 and the full commanded velocity for the
        // stiction-affected region so the operator can creep out of a stop.
        let lossy = if normalized.abs() < self.stiction_norm {
            normalized * (normalized.abs() / self.stiction_norm).powi(2)
        }
        else {
            normalized
        };
        lossy * MAX_LINEAR_SPEED_M_S * (1.0 - slip_ratio)
    }

    fn maybe_record_history(&mut self) {
        // Sample at ~10 Hz (every 5 ticks at 50 Hz).
        if self.tick.is_multiple_of(5) {
            let t = self.started.elapsed().as_secs_f32();
            if self.history.len() == HISTORY_CAPACITY {
                self.history.pop_front();
            }
            self.history.push_back(PoseSample {
                t,
                x: self.x_m,
                y: self.y_m,
                yaw: self.yaw_rad,
            });
        }
    }

    /// Drain accumulated encoder delta since the last snapshot.
    fn take_encoder_delta(&mut self) -> [i32; 2] {
        let out = self.enc_delta_accum;
        self.enc_delta_accum = [0; 2];
        out
    }

    pub fn snapshot(&mut self) -> SimState {
        let encoder_delta_total = self.take_encoder_delta();
        SimState {
            x_m: self.x_m,
            y_m: self.y_m,
            yaw_rad: self.yaw_rad,
            v_left: self.v_left,
            v_right: self.v_right,
            encoder_delta_total,
            encoder_total: self.enc_total,
            trim: self.trim,
            heading_hold: self.heading_hold,
            slip: self.slip,
            stiction: self.stiction_norm,
            stiction_kick_ms: self.kick_ms,
            stiction_kick_boost: self.kick_boost_norm,
            stiction_kick_active: self.kick_active_remaining_s > 0.0,
            enabled: self.enabled,
            paused: self.paused,
            tick: self.tick,
            history: self.history.iter().copied().collect(),
            replay: self.replay_path.as_ref().map(|path| ReplayStatus {
                path: path.clone(),
                frame: self.replay_cursor,
                total: self.replay.as_ref().map(|v| v.len()).unwrap_or(0),
            }),
        }
    }

    pub fn load_replay(
        &mut self,
        path: &str,
    ) -> Result<usize, String> {
        let text = std::fs::read_to_string(path).map_err(|e| format!("read {path}: {e}"))?;
        let value: serde_json::Value =
            serde_json::from_str(&text).map_err(|e| format!("parse {path}: {e}"))?;
        let frames = decode_replay_value(&value)?;
        if frames.is_empty() {
            return Err("no replay frames found in file".into());
        }
        let count = frames.len();
        self.reset_pose();
        self.replay = Some(frames);
        self.replay_path = Some(path.to_string());
        self.replay_cursor = 0;
        Ok(count)
    }

    fn apply_replay_frame(
        &mut self,
        frame: ReplayFrame,
    ) {
        // Treat the recorded deltas as ground truth wheel travel: each tick we
        // re-derive pose from them so the viewer animates the real run.
        let counts_to_m = 1.0 / ENCODER_COUNTS_PER_M;
        let dl = frame.enc_delta[0] as f32 * counts_to_m;
        let dr = frame.enc_delta[1] as f32 * counts_to_m;
        let v = 0.5 * (dl + dr) / frame.dt_s.max(1e-3);
        let omega = (dr - dl) / TRACK_WIDTH_M / frame.dt_s.max(1e-3);
        self.v_left = dl / frame.dt_s.max(1e-3);
        self.v_right = dr / frame.dt_s.max(1e-3);
        self.yaw_rad = wrap_angle(self.yaw_rad + omega * frame.dt_s);
        self.x_m += v * self.yaw_rad.cos() * frame.dt_s;
        self.y_m += v * self.yaw_rad.sin() * frame.dt_s;
        self.enc_total[0] = self.enc_total[0].saturating_add(frame.enc_delta[0]);
        self.enc_total[1] = self.enc_total[1].saturating_add(frame.enc_delta[1]);
        self.enc_delta_accum[0] = self.enc_delta_accum[0].saturating_add(frame.enc_delta[0]);
        self.enc_delta_accum[1] = self.enc_delta_accum[1].saturating_add(frame.enc_delta[1]);
    }
}

fn wrap_angle(a: f32) -> f32 {
    let two_pi = std::f32::consts::TAU;
    let mut x = a % two_pi;
    if x > std::f32::consts::PI {
        x -= two_pi;
    }
    else if x < -std::f32::consts::PI {
        x += two_pi;
    }
    x
}

/// Decode either a `camera_loop/samples.json` or a `trim_sweep/sweep.json`
/// payload into a flat list of replay frames. Both share an `samples` or
/// `steps` array containing `dt_s` and `encoder_delta`/`enc_delta` fields.
fn decode_replay_value(value: &serde_json::Value) -> Result<Vec<ReplayFrame>, String> {
    let array = value
        .get("samples")
        .or_else(|| value.get("steps"))
        .or_else(|| value.get("frames"))
        .and_then(|v| v.as_array())
        .ok_or_else(|| "expected `samples`, `steps`, or `frames` array".to_string())?;

    let mut frames = Vec::with_capacity(array.len());
    for (idx, item) in array.iter().enumerate() {
        let dt_s = item
            .get("dt_s")
            .or_else(|| item.get("dt"))
            .and_then(|v| v.as_f64())
            .unwrap_or(0.02) as f32;
        let deltas = item
            .get("encoder_delta")
            .or_else(|| item.get("enc_delta"))
            .or_else(|| item.get("delta_counts"))
            .ok_or_else(|| format!("frame {idx}: missing encoder_delta"))?;
        let deltas = deltas
            .as_array()
            .ok_or_else(|| format!("frame {idx}: encoder_delta not array"))?;
        let left = deltas
            .first()
            .and_then(|v| v.as_i64())
            .ok_or_else(|| format!("frame {idx}: missing left delta"))? as i32;
        let right = deltas
            .get(1)
            .and_then(|v| v.as_i64())
            .ok_or_else(|| format!("frame {idx}: missing right delta"))? as i32;
        frames.push(ReplayFrame {
            dt_s: dt_s.max(1e-3),
            enc_delta: [left, right],
        });
    }
    Ok(frames)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn translate_moves_robot_forward() {
        let mut e = SimEngine::new();
        e.set_translate(0.0, 1.0);
        for _ in 0..100 {
            e.tick(0.02);
        }
        assert!(e.x_m > 0.5, "expected forward motion, got x={}", e.x_m);
    }

    #[test]
    fn stiction_floor_prevents_creep() {
        let mut e = SimEngine::new();
        e.set_stiction(0.6);
        e.set_translate(0.0, 0.05);
        for _ in 0..50 {
            e.tick(0.02);
        }
        assert!(e.x_m.abs() < 1e-3, "stiction failed; x={}", e.x_m);
    }

    #[test]
    fn stiction_kick_breaks_idle() {
        let mut e = SimEngine::new();
        e.set_slip(0.0, 0.0);
        e.set_stiction(0.5);
        e.set_stiction_kick(100, 0.25);
        e.set_translate(0.0, 0.45);
        // 200 ms at 50 Hz = 10 ticks. The 100 ms kick covers the first 5
        // ticks and drives the chassis past the stiction-affected band.
        for _ in 0..10 {
            e.tick(0.02);
        }
        assert!(
            e.x_m > 0.0,
            "expected stiction kick to break the chassis free; x={}",
            e.x_m
        );
        let kick_x = e.x_m;
        // After the kick window closes the boost must be removed: keep
        // ticking and verify the velocity settles to the (lossy) commanded
        // forward speed rather than the boosted value.
        for _ in 0..50 {
            e.tick(0.02);
        }
        let snap = e.snapshot();
        assert!(!snap.stiction_kick_active, "kick should be expired");
        // The post-kick velocity must not exceed the boosted equivalent — it
        // tracks the commanded speed_norm * MAX_LINEAR_SPEED_M_S minus
        // friction losses, never the kick-augmented value.
        let boosted_v = (0.45 + 0.25) * MAX_LINEAR_SPEED_M_S;
        let settled_v = 0.5 * (snap.v_left + snap.v_right);
        assert!(
            settled_v < boosted_v - 0.01,
            "chassis did not settle below kick velocity (settled={}, boosted={})",
            settled_v,
            boosted_v
        );
        assert!(
            e.x_m > kick_x,
            "chassis kept moving after kick (x={}, kick_x={})",
            e.x_m,
            kick_x
        );
    }

    #[test]
    fn pause_freezes_pose() {
        let mut e = SimEngine::new();
        e.set_translate(0.0, 1.0);
        for _ in 0..10 {
            e.tick(0.02);
        }
        let snap_x = e.x_m;
        e.set_paused(true);
        for _ in 0..50 {
            e.tick(0.02);
        }
        assert!((e.x_m - snap_x).abs() < 1e-6);
    }
}
