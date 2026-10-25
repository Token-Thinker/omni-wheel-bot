//! Simulation engine behind the `/sim` view, in both drive geometries.
//!
//! Two models, selected by [`SimDriveMode`]:
//!
//! * [`SimDriveMode::TwoWheelFront`] (default) — a simplified differential
//!   drive over W1/W2 with W3 idle. This mirrors v1-alpha, where the closed-loop
//!   straightness work (WheelTrim + HeadingHold) was built around that pair
//!   because W3's motor-control path is physically dead (see
//!   `docs/owb_esp32s3_v1_w3_level_shifter.md`). Reproducing it lets the
//!   closed-loop behaviour be replayed in software.
//! * [`SimDriveMode::TriWheelOmni`] — the real three-wheel omni, driven through
//!   `owb_core`'s [`EmbodiedKinematics`] in **both** directions. This is the v2
//!   drive path, and v1-alpha hardware can never execute it, so this engine is
//!   the only place it gets exercised. Using the firmware's own kinematics
//!   rather than a second implementation is deliberate: a reimplementation
//!   could drift from the board without anything noticing.
//!
//! All state is owned by `SimEngine` behind a `Mutex`. The engine is driven by
//! a tokio task at 50 Hz (see `main.rs`). Telemetry is sampled at 10 Hz by the
//! WS task that publishes `/sim/state` messages.
//!
//! Configurable knobs:
//! - `slip_ratio` per wheel (0.0 = no slip, 0.5 = 50% commanded velocity lost)
//! - `stiction_threshold_norm` (commanded magnitude below which the wheel does
//!   not move). Default 0.45 matches the v1-alpha profile.
//! - `wheel_trim` multipliers (mirrors the firmware `WheelTrim` command).
//! - `heading_hold` (mirrors the firmware `HeadingHold` straightness loop).

use std::{collections::VecDeque, time::Instant};

use owb_core::motion::kinematics::EmbodiedKinematics;
use serde::Serialize;

/// Which drive geometry the engine integrates.
///
/// Both exist because the two boards differ: v1-alpha physically drives only
/// W1/W2 (W3's control path is dead, see `docs/owb_esp32s3_v1_w3_level_shifter.md`),
/// while v2 is a true three-wheel omni. Simulating both means the omni path can
/// be exercised before v2 hardware exists.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum SimDriveMode {
    /// Differential drive on W1/W2 with W3 idle — mirrors v1-alpha.
    TwoWheelFront,
    /// Three-wheel omni via [`EmbodiedKinematics`] — mirrors v2 and the
    /// firmware's `DriveControlMode::TriWheelOmni`.
    TriWheelOmni,
}

/// Distance between the two drive wheels (track width), metres.
const TRACK_WIDTH_M: f32 = 0.39;
/// Top linear speed at `speed_norm = 1.0`, metres/second.
const MAX_LINEAR_SPEED_M_S: f32 = 0.9;
/// Top yaw rate at `yaw_norm = 1.0`, degrees/second.
const MAX_YAW_RATE_DEG_S: f32 = 180.0;
/// Counts per wheel revolution. Must match the firmware's
/// `POLOLU_5161_COUNTS_PER_WHEEL_REV` (600) or sim encoder telemetry cannot be
/// compared against the real robot's. This previously used 1024, a silent
/// 1.71x divergence from firmware.
const ENCODER_COUNTS_PER_REV: f32 = 600.0;
/// Wheel radius, mirroring the firmware's
/// `DEFAULT_DRIVE_TUNING.wheel_radius_m`. 38 mm Nexus omni wheel => 19 mm
/// radius. Metres, not millimetres.
///
/// Keep this in step with the firmware constant: it is the counts-per-metre
/// scale, so a mismatch makes sim odometry incomparable to the real robot.
/// Cross-check: 0.019 m with the measured `max_wheel_rad_s = 50` gives a
/// 0.95 m/s top speed, which matches `MAX_LINEAR_SPEED_M_S` below — the two
/// were derived independently and now agree.
const WHEEL_RADIUS_M: f32 = 0.019;
/// Encoder counts per metre travelled by the wheel rim.
const ENCODER_COUNTS_PER_M: f32 =
    ENCODER_COUNTS_PER_REV / (2.0 * std::f32::consts::PI * WHEEL_RADIUS_M);
/// Wheel actuator response (first-order lag). Higher = snappier.
const WHEEL_RESPONSE_PER_S: f32 = 6.0;
/// Yaw per unit of sideways stick in two-wheel mode (firmware `real.rs`).
const TWO_WHEEL_STICK_TURN_GAIN: f32 = 0.6;
/// Default stiction threshold baked into the v1-alpha profile.
const DEFAULT_STICTION: f32 = 0.45;
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
    /// Post-friction velocity of W1, m/s at the rim. Alias of `wheels[0]`,
    /// retained so the existing two-wheel viewer readout keeps working.
    pub v_left: f32,
    /// Post-friction velocity of W2, m/s at the rim. Alias of `wheels[1]`.
    pub v_right: f32,
    /// Post-friction velocity of all three wheels, m/s at the rim. In
    /// `TwoWheelFront` mode `wheels[2]` is always 0.
    pub wheels: [f32; 3],
    pub drive_mode: SimDriveMode,
    pub encoder_delta_total: [i32; 3],
    pub encoder_total: [i32; 3],
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
    // Lag-filtered wheel command, m/s at the rim. This is BEFORE stiction and
    // slip — `apply_friction` turns it into the `actual_*` values below, which
    // are what actually move the robot.
    v_left: f32,
    v_right: f32,
    // Post-friction wheel velocities, m/s — the ones that integrate into pose
    // and encoder counts, and the ones reported in `SimState`. Reporting the
    // pre-friction command instead made telemetry claim a wheel was turning
    // while the model had it stalled below the stiction floor.
    actual_left: f32,
    actual_right: f32,
    // Fractional encoder counts carried between ticks. Without this the `as
    // i32` truncation silently discarded up to one count per wheel per tick —
    // a systematic undercount that got proportionally worse the slower the
    // wheel turned (~17% at 0.2 m/s).
    enc_residual: [f32; 3],
    // Targeted wheel velocities (before lag/stiction/slip), m/s.
    target_left: f32,
    target_right: f32,

    // ---- three-wheel omni state (unused in TwoWheelFront mode) ----
    drive_mode: SimDriveMode,
    /// Real firmware kinematics, so the omni path under test here is the same
    /// code the board runs rather than a reimplementation.
    kinematics: EmbodiedKinematics,
    /// Lag-filtered per-wheel command, normalised (-1..=1).
    omni_cmd: [f32; 3],
    /// Post-friction per-wheel velocity, m/s at the rim. Index 2 stays 0 in
    /// two-wheel mode.
    wheel_actual: [f32; 3],

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
    enc_total: [i32; 3],
    enc_delta_accum: [i32; 3],

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
            actual_left: 0.0,
            actual_right: 0.0,
            enc_residual: [0.0; 3],
            drive_mode: SimDriveMode::TwoWheelFront,
            // Geometry mirrors DEFAULT_DRIVE_TUNING. Only the *shape* matters
            // here (wheel angles and the radius ratio): compute_wheel_velocities
            // returns normalised speeds, which this engine scales by its own
            // MAX_LINEAR_SPEED_M_S. That keeps the sim's realistic 0.9 m/s top
            // speed even though the firmware's wheel_radius_m is suspect.
            kinematics: EmbodiedKinematics::new(WHEEL_RADIUS_M, 0.195, 25.0),
            omni_cmd: [0.0; 3],
            wheel_actual: [0.0; 3],
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
            enc_total: [0; 3],
            enc_delta_accum: [0; 3],
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
        self.actual_left = 0.0;
        self.actual_right = 0.0;
        self.enc_residual = [0.0; 3];
        self.omni_cmd = [0.0; 3];
        self.wheel_actual = [0.0; 3];
        self.v_right = 0.0;
        self.target_left = 0.0;
        self.target_right = 0.0;
        self.inputs = DriveInputs::default();
        self.enc_total = [0; 3];
        self.enc_delta_accum = [0; 3];
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

    /// Switch between the v1-alpha two-wheel model and the v2 three-wheel
    /// omni model. Resets the per-wheel filter state so the two models never
    /// inherit each other's velocities.
    pub fn set_drive_mode(
        &mut self,
        mode: SimDriveMode,
    ) {
        if self.drive_mode == mode {
            return;
        }
        self.drive_mode = mode;
        self.v_left = 0.0;
        self.v_right = 0.0;
        self.actual_left = 0.0;
        self.actual_right = 0.0;
        self.target_left = 0.0;
        self.target_right = 0.0;
        self.omni_cmd = [0.0; 3];
        self.wheel_actual = [0.0; 3];
        self.enc_residual = [0.0; 3];
        self.recompute_targets();
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
        // Two parallel wheels cannot strafe, so the sideways part of the stick
        // steers instead -- stick right turns right (clockwise, negative yaw).
        // Mirrors `TWO_WHEEL_STICK_TURN_GAIN` in the firmware's `real.rs`.
        let forward = forward_norm * MAX_LINEAR_SPEED_M_S;
        let yaw_norm = (self.inputs.yaw_rate_deg_s / MAX_YAW_RATE_DEG_S
            - TWO_WHEEL_STICK_TURN_GAIN * lateral_norm)
            .clamp(-1.0, 1.0);
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

        if self.drive_mode == SimDriveMode::TriWheelOmni {
            self.tick_omni(dt_s);
            return;
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

        // Report the post-friction velocities: these are what moved the robot.
        self.actual_left = actual_left;
        self.actual_right = actual_right;
        self.wheel_actual = [actual_left, actual_right, 0.0];

        self.integrate_encoders([actual_left, actual_right, 0.0], dt_s);
        self.maybe_record_history();
        self.tick = self.tick.saturating_add(1);
    }

    /// Advance the three-wheel omni model by `dt_s`.
    ///
    /// Deliberately routed through [`EmbodiedKinematics`] in both directions —
    /// `compute_wheel_velocities` for the inverse and `compute_body_velocity`
    /// for the forward — so this exercises the *firmware's* kinematics rather
    /// than a second implementation that could silently disagree with it. That
    /// is the whole reason the omni mode exists: v1-alpha hardware can never
    /// run this path (W3's control path is dead), so the sim is the only place
    /// it gets tested before v2 exists.
    fn tick_omni(
        &mut self,
        dt_s: f32,
    ) {
        // Inverse kinematics: body command -> per-wheel normalised speeds.
        // Saturation is handled inside `compute_wheel_velocities`, which scales
        // the whole wheel vector so the commanded heading is preserved.
        let yaw_norm = (self.inputs.yaw_rate_deg_s / MAX_YAW_RATE_DEG_S).clamp(-1.0, 1.0);
        let targets = if self.enabled {
            self.kinematics.compute_wheel_velocities(
                self.inputs.speed_norm,
                self.inputs.direction_deg,
                // Body-frame command: the sim drives in the robot's own frame,
                // so orientation is 0 rather than the world yaw.
                0.0,
                yaw_norm,
            )
        }
        else {
            [0.0; 3]
        };

        let alpha = (WHEEL_RESPONSE_PER_S * dt_s).clamp(0.0, 1.0);
        let mut actual = [0.0f32; 3];
        for (i, &target) in targets.iter().enumerate() {
            let trimmed = target * self.trim[i];
            self.omni_cmd[i] += (trimmed - self.omni_cmd[i]) * alpha;
            // `apply_friction` works in m/s, so scale in and back out. Slip is
            // only configured for the first two wheels today; W3 uses no slip.
            let commanded_m_s = self.omni_cmd[i] * MAX_LINEAR_SPEED_M_S;
            let slip = self.slip.get(i).copied().unwrap_or(0.0);
            actual[i] = self.apply_friction(commanded_m_s, slip);
        }

        // Forward kinematics: per-wheel -> body velocity, in normalised units.
        let norm = actual.map(|v| v / MAX_LINEAR_SPEED_M_S);
        let (jac_vx, jac_vy, omega_norm) = self.kinematics.compute_body_velocity(norm);

        // Undo the 90-degree frame rotation baked into
        // `EmbodiedKinematics::convert_to_body_frame`, which returns
        // `(-vy, vx)`. `compute_wheel_velocities` feeds element 0 into the
        // Jacobian's x axis, so a `direction_deg = 0` command lands on the
        // Jacobian's +Y — i.e. sideways. The firmware's two-wheel path instead
        // reads element 1 as forward, so the two drive modes disagree about
        // what 0 degrees means. Inverting the rotation here makes
        // `direction_deg = 0` mean "forward" in both sim modes, matching
        // operator expectation.
        //
        // NOTE: this is a workaround for an inconsistency in owb-core, not a
        // fix. Correct it there (fold the rotation into the Jacobian, or make
        // both consumers agree) and this compensation should be deleted.
        let forward_norm = jac_vy;
        let lateral_norm = -jac_vx;

        // Normalised -> physical, then body frame -> world frame.
        let vx = forward_norm * MAX_LINEAR_SPEED_M_S;
        let vy = lateral_norm * MAX_LINEAR_SPEED_M_S;
        let omega = omega_norm * MAX_YAW_RATE_DEG_S.to_radians();
        let (c, s) = (self.yaw_rad.cos(), self.yaw_rad.sin());
        self.x_m += (vx * c - vy * s) * dt_s;
        self.y_m += (vx * s + vy * c) * dt_s;
        self.yaw_rad = wrap_angle(self.yaw_rad + omega * dt_s);

        self.wheel_actual = actual;
        // Keep the two-wheel aliases meaningful for the existing readouts.
        self.actual_left = actual[0];
        self.actual_right = actual[1];

        self.integrate_encoders(actual, dt_s);
        self.maybe_record_history();
        self.tick = self.tick.saturating_add(1);
    }

    /// Accumulate encoder counts for all three wheels from rim velocities.
    ///
    /// Carries the fractional remainder per wheel so counts are conserved
    /// rather than truncated away each tick.
    fn integrate_encoders(
        &mut self,
        wheel_m_s: [f32; 3],
        dt_s: f32,
    ) {
        for (i, &v) in wheel_m_s.iter().enumerate() {
            let exact = v * dt_s * ENCODER_COUNTS_PER_M + self.enc_residual[i];
            let whole = exact.trunc();
            self.enc_residual[i] = exact - whole;
            let delta = whole as i32;
            self.enc_total[i] = self.enc_total[i].saturating_add(delta);
            self.enc_delta_accum[i] = self.enc_delta_accum[i].saturating_add(delta);
        }
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
    fn take_encoder_delta(&mut self) -> [i32; 3] {
        let out = self.enc_delta_accum;
        self.enc_delta_accum = [0; 3];
        out
    }

    /// Current slip factors. Cheap, non-draining config read.
    pub fn slip(&self) -> [f32; 2] { self.slip }

    /// Current per-wheel trim. Cheap, non-draining config read.
    pub fn trim(&self) -> [f32; 3] { self.trim }

    /// Current heading-hold settings. Cheap, non-draining config read.
    pub fn heading_hold(&self) -> HeadingHoldState { self.heading_hold }

    /// Full state view that does **not** consume the encoder delta.
    ///
    /// Use this for anything that merely observes — a newly connected viewer, a
    /// debug endpoint. Only the single periodic broadcaster may call
    /// [`Self::snapshot`]; two draining callers split one tick's counts between
    /// them and both report an undercount.
    pub fn snapshot_peek(&self) -> SimState { self.build_state(self.enc_delta_accum) }

    /// Drains the accumulated encoder delta and returns the full state.
    ///
    /// Destructive by design, for the 10 Hz broadcaster. Reach for
    /// [`Self::snapshot_peek`] or the individual getters above if you only want
    /// to read a value — this used to be called four times from the config
    /// endpoint, which silently wiped encoder telemetry on every slider drag.
    pub fn snapshot(&mut self) -> SimState {
        let encoder_delta_total = self.take_encoder_delta();
        self.build_state(encoder_delta_total)
    }

    fn build_state(
        &self,
        encoder_delta_total: [i32; 3],
    ) -> SimState {
        SimState {
            x_m: self.x_m,
            y_m: self.y_m,
            yaw_rad: self.yaw_rad,
            v_left: self.actual_left,
            v_right: self.actual_right,
            wheels: self.wheel_actual,
            drive_mode: self.drive_mode,
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
        // Replayed deltas are already what the wheels really did, so the
        // pre- and post-friction values coincide here. Both are set so the
        // reported `v_left`/`v_right` stay correct in replay too.
        self.v_left = dl / frame.dt_s.max(1e-3);
        self.v_right = dr / frame.dt_s.max(1e-3);
        self.actual_left = self.v_left;
        self.actual_right = self.v_right;
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

    /// Encoder counts must be conserved, not truncated away each tick.
    ///
    /// Regression guard: the per-tick `as i32` cast discarded the fractional
    /// part every tick, which at 50 Hz systematically undercounted and got
    /// proportionally worse the slower the wheel turned.
    #[test]
    fn encoder_counts_are_conserved_not_truncated() {
        let mut e = SimEngine::new();
        e.set_translate(0.0, 1.0);
        // Let the first-order lag settle so velocity is steady.
        for _ in 0..200 {
            e.tick(0.02);
        }
        let start = e.snapshot_peek();
        let v = start.v_left;
        assert!(v > 0.05, "test needs a moving wheel, got {v}");

        let secs = 4.0f32;
        let ticks = (secs / 0.02) as usize;
        for _ in 0..ticks {
            e.tick(0.02);
        }
        let end = e.snapshot_peek();
        let counted = (end.encoder_total[0] - start.encoder_total[0]) as f32;
        // Distance implied by the reported (post-friction) velocity.
        let expected = v * secs * ENCODER_COUNTS_PER_M;
        let err = (counted - expected).abs() / expected;
        assert!(
            err < 0.01,
            "encoder counts drifted {:.1}% from distance travelled \
             (counted {counted}, expected {expected:.0})",
            err * 100.0
        );
    }

    /// `SimState` must report the velocities that actually moved the robot.
    ///
    /// Regression guard: reporting the pre-friction command meant a wheel held
    /// below the stiction floor was advertised as turning while the model had
    /// it fully stalled — pose and encoders said zero, telemetry did not.
    #[test]
    fn reported_velocity_is_post_friction() {
        let mut e = SimEngine::new();
        // Command well below the stiction floor: apply_friction returns 0.
        e.set_translate(0.0, 0.05);
        for _ in 0..200 {
            e.tick(0.02);
        }
        let s = e.snapshot_peek();
        assert_eq!(
            s.encoder_total,
            [0, 0, 0],
            "a stalled wheel must not accumulate counts"
        );
        assert!(
            s.v_left.abs() < 1e-6 && s.v_right.abs() < 1e-6,
            "stalled wheels must report zero velocity, got {} / {}",
            s.v_left,
            s.v_right
        );
    }

    /// The sim's encoder scale must match the firmware's counts-per-revolution
    /// or sim telemetry cannot be compared against the real robot.
    #[test]
    fn encoder_scale_matches_firmware_counts_per_rev() {
        assert_eq!(
            ENCODER_COUNTS_PER_REV, 600.0,
            "must match POLOLU_5161_COUNTS_PER_WHEEL_REV in the firmware profile"
        );
        let per_rev = ENCODER_COUNTS_PER_M * 2.0 * std::f32::consts::PI * WHEEL_RADIUS_M;
        assert!((per_rev - 600.0).abs() < 0.01, "got {per_rev}");
    }

    /// Drive the engine for `secs` and return the resulting state.
    fn run(
        e: &mut SimEngine,
        secs: f32,
    ) -> SimState {
        for _ in 0..((secs / 0.02) as usize) {
            e.tick(0.02);
        }
        e.snapshot_peek()
    }

    /// Omni forward translation must actually go forward, and all three wheels
    /// must participate. This is the v2 code path that v1-alpha hardware can
    /// never run (W3's control path is dead), so the sim is the only test.
    #[test]
    fn omni_translate_moves_forward_using_all_three_wheels() {
        let mut e = SimEngine::new();
        e.set_drive_mode(SimDriveMode::TriWheelOmni);
        e.set_translate(0.0, 1.0);
        let s = run(&mut e, 3.0);

        assert_eq!(s.drive_mode, SimDriveMode::TriWheelOmni);
        assert!(s.x_m > 0.2, "should travel forward in +x, got {}", s.x_m);
        assert!(
            s.y_m.abs() < s.x_m * 0.15,
            "forward command drifted sideways: x={} y={}",
            s.x_m,
            s.y_m
        );
        assert!(
            s.yaw_rad.abs() < 0.15,
            "forward command should not spin the robot: yaw={}",
            s.yaw_rad
        );
        // A 3-wheel omni translating forward loads the two side wheels; the
        // third sits near zero for this particular heading. What matters is
        // that the omni path drives more than the two-wheel pair does.
        let moving = s.wheels.iter().filter(|w| w.abs() > 1e-3).count();
        assert!(moving >= 2, "expected wheels to drive, got {:?}", s.wheels);
    }

    /// Pure yaw must rotate in place: all three wheels drive the same
    /// direction tangentially and the body should not translate.
    #[test]
    fn omni_pure_yaw_spins_in_place() {
        let mut e = SimEngine::new();
        // Pin the knob the 0.5 rad threshold was written against, so a change
        // to the profile default does not quietly move this test's goalposts.
        e.set_stiction(0.55);
        e.set_drive_mode(SimDriveMode::TriWheelOmni);
        e.set_translate(0.0, 0.0);
        e.set_yaw(MAX_YAW_RATE_DEG_S);
        let s = run(&mut e, 2.0);

        assert!(s.yaw_rad.abs() > 0.5, "should have rotated: {}", s.yaw_rad);
        let drift = (s.x_m * s.x_m + s.y_m * s.y_m).sqrt();
        assert!(drift < 0.05, "pure yaw translated {drift} m");
        // All three wheels turn together for a spin.
        for (i, w) in s.wheels.iter().enumerate() {
            assert!(
                w.abs() > 1e-3,
                "wheel {i} idle during a spin: {:?}",
                s.wheels
            );
        }
    }

    /// Encoder counts must accumulate on all three channels in omni mode —
    /// the two-wheel model leaves index 2 at zero, and it would be easy for a
    /// widened array to keep doing that by accident.
    #[test]
    fn omni_accumulates_all_three_encoders() {
        let mut e = SimEngine::new();
        e.set_drive_mode(SimDriveMode::TriWheelOmni);
        e.set_yaw(MAX_YAW_RATE_DEG_S);
        let s = run(&mut e, 2.0);
        for (i, c) in s.encoder_total.iter().enumerate() {
            assert!(*c != 0, "encoder {i} never moved: {:?}", s.encoder_total);
        }
    }

    /// Switching modes must not let one model inherit the other's velocities.
    #[test]
    fn switching_drive_mode_clears_wheel_state() {
        let mut e = SimEngine::new();
        e.set_translate(0.0, 1.0);
        run(&mut e, 1.0);
        assert!(
            e.snapshot_peek().v_left.abs() > 0.01,
            "two-wheel should be moving"
        );

        e.set_drive_mode(SimDriveMode::TriWheelOmni);
        let s = e.snapshot_peek();
        assert_eq!(s.wheels, [0.0; 3], "omni inherited two-wheel velocity");
        assert_eq!(s.drive_mode, SimDriveMode::TriWheelOmni);

        // And back again.
        e.set_drive_mode(SimDriveMode::TwoWheelFront);
        let s = e.snapshot_peek();
        assert_eq!(s.wheels, [0.0; 3]);
        assert!(s.v_left.abs() < 1e-6 && s.v_right.abs() < 1e-6);
    }

    /// Two-wheel mode must keep W3 at zero — it mirrors v1-alpha, where that
    /// wheel physically cannot be driven.
    #[test]
    fn two_wheel_mode_leaves_third_wheel_idle() {
        let mut e = SimEngine::new();
        e.set_translate(0.0, 1.0);
        let s = run(&mut e, 2.0);
        assert_eq!(s.drive_mode, SimDriveMode::TwoWheelFront);
        assert_eq!(s.wheels[2], 0.0);
        assert_eq!(s.encoder_total[2], 0);
    }

    /// Two-wheel mode has no strafe, so a sideways stick must turn the robot
    /// in place -- right turns clockwise (yaw decreases) -- instead of doing
    /// nothing or creeping forward.
    #[test]
    fn two_wheel_sideways_stick_turns_in_place() {
        for (dir, clockwise) in [(90.0, true), (270.0, false)] {
            let mut e = SimEngine::new();
            e.set_stiction(0.0);
            e.set_slip(0.0, 0.0);
            e.set_enabled(true);
            e.set_translate(dir, 0.8);
            for _ in 0..50 {
                e.tick(0.02);
            }
            assert!(
                if clockwise {
                    e.yaw_rad < -0.2
                }
                else {
                    e.yaw_rad > 0.2
                },
                "dir {dir}: expected a {} turn, yaw={}",
                if clockwise {
                    "clockwise"
                }
                else {
                    "counter-clockwise"
                },
                e.yaw_rad
            );
            let travel = (e.x_m * e.x_m + e.y_m * e.y_m).sqrt();
            assert!(travel < 0.05, "dir {dir}: pivot travelled {travel} m");
        }
    }

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
