#[cfg(feature = "real-executors")]
use esp_hal::gpio::{AnyPin, Level, Output, OutputConfig};
#[cfg(feature = "real-executors")]
use log::{debug, info, warn};
#[cfg(feature = "real-executors")]
use owb_core::{
    ctrl::sysctrl::EncoderData,
    ek,
    motion::omni::{DriveTuning, HalfBridgeOutput, TriWheelDrivePlan},
    motion::pid::TriWheelPidController,
    motion::DiffDrivePlan,
    msg::DriveCommand,
    platform::BoardRuntimeKind,
};

#[cfg(feature = "real-executors")]
use crate::profile::{BoardProfile, DriveControlMode, MotorBridgeMode, MotorPins};

#[cfg(feature = "real-executors")]
const ALPHA_TWO_WHEEL_FULL_YAW_DEG_S: f32 = 720.0;

#[cfg(feature = "real-executors")]
use super::{
    AddressableLedDriver,
    BoardCapabilities,
    GpioBridge,
    GpioStatusLed,
    MotorBridge,
    OnboardHdc2010Sensor,
    OnboardLis2dh12Imu,
    OnboardMax17048Gauge,
    OnboardTca9555Indicators,
    PcntEncoderCapture,
    PhasePwmBridge,
    PwmBridge,
    RgbColor,
};

#[cfg(feature = "real-executors")]
pub struct RealBoardRuntime {
    pub(super) runtime: BoardRuntimeKind,
    pub(super) drive: DriveTuning,
    drive_mode: DriveControlMode,
    active_wheels: [bool; 3],
    motor_pins: Option<MotorPins>,
    kinematics: ek,
    pid: TriWheelPidController,
    pub(super) enabled: bool,
    last_direction_deg: f32,
    last_speed_norm: f32,
    last_yaw_rate_deg_s: f32,
    pub(super) last_orientation_deg: f32,
    last_encoder_delta_counts: [i32; 3],
    last_drive_targets: [f32; 3],
    last_pid_output: [f32; 3],
    wheel_trim: [f32; 3],
    wheel_phase_invert: [bool; 3],
    min_forward_speed_norm: f32,
    /// Pulse duration in ms applied on idle→forward transitions to break
    /// static friction. `0` disables the kick.
    stiction_kick_ms: u16,
    /// Additive boost on top of commanded `|speed_norm|` during the kick.
    stiction_kick_boost_norm: f32,
    /// Wall-clock deadline (ms) for the active kick; `None` when idle.
    stiction_kick_active_until_ms: Option<u32>,
    /// Sign of the last commanded forward component. Used to re-arm the kick
    /// on a direction reversal as well as on the rising edge above deadband.
    last_speed_norm_sign: f32,
    heading_hold_enabled: bool,
    heading_hold_kp: f32,
    heading_hold_correction: f32,
    heading_hold_baseline_raw: [i16; 3],
    pid_probe_targets: Option<[f32; 3]>,
    sleep: Option<Output<'static>>,
    status_led: Option<GpioStatusLed>,
    addressable_led: Option<AddressableLedDriver>,
    pub(super) led_effect: super::leds::LedEffect,
    pub(super) led_phase: u8,
    /// Last-known Wi-Fi/client indicator state (TCA9555 index 1). Tracked so the
    /// LED-effect "quiet" restore can return the indicator to its real state
    /// instead of hardcoding it off and dropping a live client connection.
    wifi_connected: bool,
    indicator_leds: Option<OnboardTca9555Indicators>,
    pub(super) imu: Option<OnboardLis2dh12Imu>,
    pub(super) env: Option<OnboardHdc2010Sensor>,
    pub(super) power: Option<OnboardMax17048Gauge>,
    pub(super) encoders: Option<PcntEncoderCapture>,
    motors: Option<[MotorBridge; 3]>,
}

#[cfg(feature = "real-executors")]
impl RealBoardRuntime {
    fn log_drive_plan(
        &self,
        reason: &str,
        plan: TriWheelDrivePlan,
    ) {
        let level = if matches!(
            reason,
            "translation_command" | "yaw_command" | "motion_command"
        ) {
            log::Level::Debug
        }
        else {
            log::Level::Info
        };

        if let Some(motor_pins) = self.motor_pins {
            let wheel0 = GpioBridge::levels_for(motor_pins.mode, plan.wheels[0]);
            let wheel1 = GpioBridge::levels_for(motor_pins.mode, plan.wheels[1]);
            let wheel2 = GpioBridge::levels_for(motor_pins.mode, plan.wheels[2]);
            log::log!(
                level,
                "real runtime {:?}: {} GPIO{}={} GPIO{}={} GPIO{}={} GPIO{}={} GPIO{}={} GPIO{}={} GPIO{}={}",
                self.runtime,
                reason,
                motor_pins.sleep,
                plan.enabled,
                motor_pins.bridges[0].phase,
                wheel0.0,
                motor_pins.bridges[0].drive,
                wheel0.1,
                motor_pins.bridges[1].phase,
                wheel1.0,
                motor_pins.bridges[1].drive,
                wheel1.1,
                motor_pins.bridges[2].phase,
                wheel2.0,
                motor_pins.bridges[2].drive,
                wheel2.1
            );
        }
        else {
            log::log!(
                level,
                "real runtime {:?}: {} sleep={} bridges=[({},{}) ({},{}) ({},{})]",
                self.runtime,
                reason,
                plan.enabled,
                plan.wheels[0].phase_high,
                plan.wheels[0].drive_high,
                plan.wheels[1].phase_high,
                plan.wheels[1].drive_high,
                plan.wheels[2].phase_high,
                plan.wheels[2].drive_high
            );
        }
    }

    pub fn from_profile(
        profile: &BoardProfile,
        capabilities: BoardCapabilities,
    ) -> Self {
        let drive = profile.drive;
        let sleep = profile.motor.map(|pins| {
            let mut pin = Output::new(
                unsafe { AnyPin::steal(pins.sleep) },
                Level::Low,
                OutputConfig::default(),
            );
            pin.set_low();
            pin
        });

        let motors = profile.motor.map(|pins| {
            if matches!(
                pins.mode,
                MotorBridgeMode::PhaseEnable | MotorBridgeMode::Drv8845PhaseEnable
            ) {
                if let Some(bridges) = PhasePwmBridge::new_three(pins, profile.active_wheels) {
                    return bridges;
                }
                warn!("board pwm: falling back to GPIO drive outputs");
            }

            let wheel0 = if !profile.active_wheels[0] {
                MotorBridge::Disabled
            }
            else if profile.runtime == BoardRuntimeKind::DevKitReference
                && pins.mode == MotorBridgeMode::In1In2
            {
                PwmBridge::new_wheel0(pins.mode, pins.bridges[0].phase, pins.bridges[0].drive)
                    .map(MotorBridge::Pwm)
                    .unwrap_or_else(|| {
                        MotorBridge::Gpio(GpioBridge::new(
                            pins.mode,
                            pins.bridges[0].phase,
                            pins.bridges[0].drive,
                        ))
                    })
            }
            else {
                MotorBridge::Gpio(GpioBridge::new(
                    pins.mode,
                    pins.bridges[0].phase,
                    pins.bridges[0].drive,
                ))
            };

            let wheel1 = if profile.active_wheels[1] {
                MotorBridge::Gpio(GpioBridge::new(
                    pins.mode,
                    pins.bridges[1].phase,
                    pins.bridges[1].drive,
                ))
            }
            else {
                MotorBridge::Disabled
            };

            let wheel2 = if profile.active_wheels[2] {
                MotorBridge::Gpio(GpioBridge::new(
                    pins.mode,
                    pins.bridges[2].phase,
                    pins.bridges[2].drive,
                ))
            }
            else {
                MotorBridge::Disabled
            };

            [wheel0, wheel1, wheel2]
        });

        // The real executor is currently the only owner of RMT in firmware, so
        // it can safely claim the peripheral when a board exposes an
        // addressable LED chain.
        let addressable_led = if capabilities.has_addressable_led {
            profile
                .addressable_led
                .and_then(|led| AddressableLedDriver::new(led.pin, led.pixels, led.inverted))
        }
        else {
            None
        };

        let mut runtime = Self {
            runtime: profile.runtime,
            drive,
            drive_mode: profile.drive_mode,
            active_wheels: profile.active_wheels,
            motor_pins: profile.motor,
            kinematics: ek::new(
                drive.wheel_radius_m,
                drive.robot_radius_m,
                drive.max_wheel_rad_s,
            ),
            pid: TriWheelPidController::new(
                drive.counts_per_wheel_revolution as f32,
                drive.max_wheel_rad_s,
                drive.pid.kp,
                drive.pid.ki,
                drive.pid.kd,
                0.5,
                1.0,
            ),
            enabled: false,
            last_direction_deg: 0.0,
            last_speed_norm: 0.0,
            last_yaw_rate_deg_s: 0.0,
            last_orientation_deg: 0.0,
            last_encoder_delta_counts: [0; 3],
            last_drive_targets: [0.0; 3],
            last_pid_output: [0.0; 3],
            wheel_trim: profile.default_wheel_trim,
            wheel_phase_invert: profile.wheel_phase_invert,
            min_forward_speed_norm: profile.min_forward_speed_norm,
            stiction_kick_ms: profile.stiction_kick_ms,
            stiction_kick_boost_norm: profile.stiction_kick_boost_norm,
            stiction_kick_active_until_ms: None,
            last_speed_norm_sign: 0.0,
            heading_hold_enabled: false,
            heading_hold_kp: 0.0,
            heading_hold_correction: 0.0,
            heading_hold_baseline_raw: [0; 3],
            pid_probe_targets: None,
            sleep,
            status_led: profile
                .status_led
                .map(|led| GpioStatusLed::new(led.pin, led.active_high)),
            addressable_led,
            led_effect: super::leds::LedEffect::None,
            led_phase: 0,
            wifi_connected: false,
            indicator_leds: OnboardTca9555Indicators::from_profile(profile, capabilities),
            imu: OnboardLis2dh12Imu::from_profile(profile, capabilities),
            env: OnboardHdc2010Sensor::from_profile(profile, capabilities),
            power: OnboardMax17048Gauge::from_profile(profile, capabilities),
            encoders: PcntEncoderCapture::from_profile(profile),
            motors,
        };
        runtime.apply_drive_plan(TriWheelDrivePlan::stopped(false));
        runtime
    }

    fn wheel_active(
        &self,
        index: usize,
    ) -> bool {
        self.active_wheels.get(index).copied().unwrap_or(false)
    }

    fn mask_targets(
        &self,
        mut targets: [f32; 3],
    ) -> [f32; 3] {
        for (index, target) in targets.iter_mut().enumerate() {
            if !self.wheel_active(index) {
                *target = 0.0;
            }
        }
        targets
    }

    fn mask_plan(
        &self,
        mut plan: TriWheelDrivePlan,
    ) -> TriWheelDrivePlan {
        for (index, wheel) in plan.wheels.iter_mut().enumerate() {
            if !self.wheel_active(index) {
                *wheel = HalfBridgeOutput::stop();
            }
        }
        plan
    }

    fn two_wheel_front_targets(&mut self) -> [f32; 3] {
        let yaw_norm = (self.last_yaw_rate_deg_s / ALPHA_TWO_WHEEL_FULL_YAW_DEG_S).clamp(-1.0, 1.0);
        let (_, forward_raw) = ek::convert_to_body_frame(
            self.last_speed_norm,
            self.last_direction_deg,
            self.last_orientation_deg,
        );
        // Stiction floor: when the operator commands any forward motion above the
        // deadband but below the static-friction threshold, snap the magnitude up
        // to `min_forward_speed_norm`. Preserves sign. Baked in per profile.
        let forward_after_floor = {
            let floor = self.min_forward_speed_norm;
            if floor > 0.0
                && forward_raw.abs() > self.drive.motor_deadband_norm
                && forward_raw.abs() < floor
            {
                if forward_raw >= 0.0 {
                    floor
                }
                else {
                    -floor
                }
            }
            else {
                forward_raw
            }
        };

        // Stiction kick: if armed and still within its window, add an
        // additive PWM boost in the direction of travel to break the wheels
        // free of static friction. Clears the deadline once expired.
        let forward_norm = if let Some(deadline) = self.stiction_kick_active_until_ms {
            let now = Self::now_ms();
            // `now >= deadline` accounting for u32 wrap: treat the kick as
            // active only when the signed difference is non-positive.
            let expired = (now.wrapping_sub(deadline) as i32) >= 0;
            if expired {
                self.stiction_kick_active_until_ms = None;
                forward_after_floor
            }
            else if forward_after_floor != 0.0 && self.stiction_kick_boost_norm > 0.0 {
                let boost = if forward_after_floor >= 0.0 {
                    self.stiction_kick_boost_norm
                }
                else {
                    -self.stiction_kick_boost_norm
                };
                (forward_after_floor + boost).clamp(-1.0, 1.0)
            }
            else {
                forward_after_floor
            }
        }
        else {
            forward_after_floor
        };

        // Pure-yaw stiction floor: snap |yaw_norm| up to the stiction floor
        // when forward is in the deadband. Standard diff-drive below then
        // splits this into [+mag, -mag] for a true two-wheel in-place pivot
        // around the geometric center. Without the floor, the per-wheel
        // diff-drive output for a low-rate yaw command would land below
        // static friction and the bot wouldn't move. Above the floor, yaw
        // is rate-proportional as the operator expects.
        let yaw_norm =
            if forward_norm.abs() <= self.drive.motor_deadband_norm && yaw_norm.abs() > 0.0 {
                let pivot_floor =
                    (self.min_forward_speed_norm + self.stiction_kick_boost_norm).clamp(0.0, 1.0);
                if yaw_norm.abs() < pivot_floor {
                    if yaw_norm > 0.0 {
                        pivot_floor
                    }
                    else {
                        -pivot_floor
                    }
                }
                else {
                    yaw_norm
                }
            }
            else {
                yaw_norm
            };

        // Forward/diagonal stick positions keep standard differential mixing for predictable arcs.
        let plan = DiffDrivePlan::from_drive_command(
            true,
            DriveCommand {
                linear_norm: forward_norm,
                angular_norm: yaw_norm,
            },
            self.drive.motor_deadband_norm,
        );

        [plan.wheels.left_norm, plan.wheels.right_norm, 0.0]
    }

    pub(super) fn set_enabled(
        &mut self,
        enabled: bool,
    ) {
        self.enabled = enabled;
        if self.runtime == BoardRuntimeKind::OwbS3V1 {
            self.last_speed_norm = 0.0;
            self.last_yaw_rate_deg_s = 0.0;
        }
        if !enabled {
            self.pid.reset();
            self.last_drive_targets = [0.0; 3];
            self.last_pid_output = [0.0; 3];
            self.pid_probe_targets = None;
            self.stiction_kick_active_until_ms = None;
            self.last_speed_norm_sign = 0.0;
        }
        let plan = if enabled {
            self.plan_from_state()
        }
        else {
            TriWheelDrivePlan::stopped(false)
        };
        self.log_drive_plan("enable_command", plan);
        self.apply_drive_plan(plan);
        info!("real runtime {:?}: enable={enabled}", self.runtime);
    }

    pub(super) fn set_led_color(
        &mut self,
        color: RgbColor,
    ) {
        if let Some(led) = self.status_led.as_mut() {
            led.set_on(color.r > 0 || color.g > 0 || color.b > 0);
        }
        if let Some(led) = self.addressable_led.as_mut() {
            led.set_rgb(color);
        }
        info!(
            "real runtime {:?}: led=({}, {}, {})",
            self.runtime, color.r, color.g, color.b
        );
    }

    /// Write a frame to the addressable strip only, without touching the GPIO
    /// status LED or logging. Used by the animated effect loop so a breath dip
    /// to black doesn't flicker the status LED or spam the log each tick.
    pub(super) fn render_addressable(
        &mut self,
        color: RgbColor,
    ) {
        if let Some(led) = self.addressable_led.as_mut() {
            led.set_rgb(color);
        }
    }

    /// Select the active LED effect, resetting its phase and drawing the first
    /// frame immediately. `LedEffect::None` clears any running animation.
    pub(super) fn set_led_effect(
        &mut self,
        effect: super::leds::LedEffect,
    ) {
        use super::leds::LedEffect;
        let was_active = !matches!(self.led_effect, LedEffect::None);
        let now_active = !matches!(effect, LedEffect::None);
        self.led_effect = effect;
        self.led_phase = 0;
        // Aesthetics: while the addressable strip runs an effect, all the other
        // LEDs in the system are silenced so they don't distract from the show.
        // Toggle only on the transition to avoid per-tick I2C writes.
        if now_active && !was_active {
            self.set_other_leds_quiet(true);
        }
        else if !now_active && was_active {
            self.set_other_leds_quiet(false);
        }
        if let Some(color) = effect.color_at(0) {
            self.render_addressable(color);
        }
    }

    /// Silence (or restore) every non-addressable LED for effect aesthetics:
    /// the GPIO status LED and the TCA9555 power/wifi/user indicators. Restore
    /// returns power to its boot default (on) and the Wi-Fi/client indicator to
    /// its real connection state, so running an LED effect while a client is
    /// connected does not falsely drop the connection indicator.
    fn set_other_leds_quiet(
        &mut self,
        quiet: bool,
    ) {
        let wifi_connected = self.wifi_connected;
        if quiet {
            if let Some(led) = self.status_led.as_mut() {
                led.set_on(false);
            }
        }
        if let Some(indicators) = self.indicator_leds.as_mut() {
            indicators.set(0, !quiet);
            // Index 1 is the Wi-Fi/client indicator: while quiet, force it off;
            // on restore, return it to the saved connection state rather than
            // hardcoding off.
            indicators.set(1, if quiet { false } else { wifi_connected });
            indicators.set(2, false);
        }
    }

    pub(super) fn set_indicator_led(
        &mut self,
        index: u8,
        on: bool,
    ) {
        // Track the Wi-Fi/client indicator (index 1) so an LED-effect restore
        // can return it to the real connection state.
        if index == 1 {
            self.wifi_connected = on;
        }
        if let Some(indicators) = self.indicator_leds.as_mut() {
            indicators.set(index, on);
        }
        else {
            warn!(
                "real runtime {:?}: indicator_led index={} on={} ignored; TCA9555 unavailable",
                self.runtime, index, on
            );
        }
    }

    pub(super) fn apply_motion(
        &mut self,
        direction_deg: f32,
        speed_norm: f32,
        yaw_rate_deg_s: f32,
        orientation_deg: f32,
    ) {
        let clamped = speed_norm.clamp(-1.0, 1.0);
        self.maybe_arm_stiction_kick(clamped);
        self.last_direction_deg = direction_deg;
        self.last_speed_norm = clamped;
        self.last_yaw_rate_deg_s = yaw_rate_deg_s;
        self.last_orientation_deg = orientation_deg;
        self.pid_probe_targets = None;
        let plan = self.plan_from_state();
        self.log_drive_plan("motion_command", plan);
        self.apply_drive_plan(plan);
        debug!(
            "real runtime {:?}: dir={:.1} speed={:.2} yaw={:.1} orient={:.1}",
            self.runtime,
            self.last_direction_deg,
            self.last_speed_norm,
            self.last_yaw_rate_deg_s,
            self.last_orientation_deg
        );
    }

    pub(super) fn apply_translation(
        &mut self,
        direction_deg: f32,
        speed_norm: f32,
    ) {
        let clamped = speed_norm.clamp(-1.0, 1.0);
        self.maybe_arm_stiction_kick(clamped);
        self.last_direction_deg = direction_deg;
        self.last_speed_norm = clamped;
        self.pid_probe_targets = None;
        let plan = self.plan_from_state();
        self.log_drive_plan("translation_command", plan);
        self.apply_drive_plan(plan);
        debug!(
            "real runtime {:?}: dir={:.1} speed={:.2} yaw={:.1} orient={:.1}",
            self.runtime,
            self.last_direction_deg,
            self.last_speed_norm,
            self.last_yaw_rate_deg_s,
            self.last_orientation_deg
        );
    }

    pub(super) fn apply_yaw(
        &mut self,
        yaw_rate_deg_s: f32,
        orientation_deg: Option<f32>,
    ) {
        self.last_yaw_rate_deg_s = yaw_rate_deg_s;
        if let Some(orientation_deg) = orientation_deg {
            self.last_orientation_deg = orientation_deg;
        }
        self.pid_probe_targets = None;
        let plan = self.plan_from_state();
        self.log_drive_plan("yaw_command", plan);
        self.apply_drive_plan(plan);
        debug!(
            "real runtime {:?}: dir={:.1} speed={:.2} yaw={:.1} orient={:.1}",
            self.runtime,
            self.last_direction_deg,
            self.last_speed_norm,
            self.last_yaw_rate_deg_s,
            self.last_orientation_deg
        );
    }

    pub(super) fn apply_probe(
        &mut self,
        wheel: u8,
        forward: bool,
    ) {
        self.enabled = false;
        self.pid.reset();
        self.last_speed_norm = 0.0;
        self.last_yaw_rate_deg_s = 0.0;
        self.last_encoder_delta_counts = [0; 3];
        self.last_drive_targets = [0.0; 3];
        self.last_pid_output = [0.0; 3];
        self.pid_probe_targets = None;

        let mut plan = TriWheelDrivePlan::stopped(true);
        let index = usize::from(wheel.min(2));
        if !self.wheel_active(index) {
            warn!(
                "real runtime {:?}: probe wheel={} ignored; inactive in profile",
                self.runtime, index
            );
            self.apply_drive_plan(TriWheelDrivePlan::stopped(false));
            return;
        }
        plan.wheels[index] = HalfBridgeOutput {
            phase_high: forward,
            drive_high: true,
        };
        self.log_drive_plan("probe_command", plan);
        self.apply_drive_plan(plan);
        info!(
            "real runtime {:?}: probe wheel={} forward={}",
            self.runtime, index, forward
        );
    }

    pub(super) fn apply_probe_speed(
        &mut self,
        wheel: u8,
        speed_norm: f32,
    ) {
        self.enabled = false;
        self.pid.reset();
        self.last_speed_norm = 0.0;
        self.last_yaw_rate_deg_s = 0.0;
        self.last_encoder_delta_counts = [0; 3];
        self.last_drive_targets = [0.0; 3];
        self.last_pid_output = [0.0; 3];
        self.pid_probe_targets = None;

        let index = usize::from(wheel.min(2));
        if !self.wheel_active(index) {
            warn!(
                "real runtime {:?}: probe_speed wheel={} ignored; inactive in profile",
                self.runtime, index
            );
            self.apply_drive_plan(TriWheelDrivePlan::stopped(false));
            return;
        }
        let raw = speed_norm.clamp(-1.0, 1.0);
        // Same per-wheel phase invert the TwoWheelFront drive path uses.
        let phase_sign: f32 = if self.wheel_phase_invert[index] {
            -1.0
        }
        else {
            1.0
        };
        let speed = raw * phase_sign;
        let active = speed.abs() > 0.0;
        let plan = TriWheelDrivePlan::stopped(active);
        self.log_drive_plan("probe_speed_command", plan);
        self.apply_drive_plan(plan);
        if let Some(motors) = self.motors.as_mut() {
            if let Some(bridge) = motors.get_mut(index) {
                bridge.apply_signed_norm(speed);
            }
        }
        info!(
            "real runtime {:?}: probe_speed wheel={} speed_norm={:.2}",
            self.runtime, index, speed
        );
    }

    pub(super) fn apply_pid_probe_speed(
        &mut self,
        wheel: u8,
        speed_norm: f32,
    ) {
        let index = usize::from(wheel.min(2));
        if !self.wheel_active(index) {
            warn!(
                "real runtime {:?}: pid_probe_speed wheel={} ignored; inactive in profile",
                self.runtime, index
            );
            self.apply_drive_plan(TriWheelDrivePlan::stopped(false));
            return;
        }
        let speed = speed_norm.clamp(-1.0, 1.0);
        let mut targets = [0.0; 3];
        targets[index] = speed;

        self.enabled = speed.abs() > 0.0;
        self.pid.reset();
        self.last_speed_norm = 0.0;
        self.last_yaw_rate_deg_s = 0.0;
        self.last_encoder_delta_counts = [0; 3];
        self.last_drive_targets = [0.0; 3];
        self.last_pid_output = [0.0; 3];
        self.pid_probe_targets = if self.enabled { Some(targets) } else { None };

        let plan = self.plan_from_state();
        self.log_drive_plan("pid_probe_speed_command", plan);
        self.apply_drive_plan(plan);
        info!(
            "real runtime {:?}: pid_probe_speed wheel={} speed_norm={:.2}",
            self.runtime, index, speed
        );
    }

    pub(super) fn apply_raw_bridge(
        &mut self,
        wheel: u8,
        phase_gpio_high: bool,
        drive_gpio_high: bool,
    ) {
        self.enabled = false;
        self.pid.reset();
        self.last_speed_norm = 0.0;
        self.last_yaw_rate_deg_s = 0.0;
        self.last_encoder_delta_counts = [0; 3];
        self.last_drive_targets = [0.0; 3];
        self.last_pid_output = [0.0; 3];
        self.pid_probe_targets = None;

        let index = usize::from(wheel.min(2));
        if !self.wheel_active(index) {
            warn!(
                "real runtime {:?}: raw_bridge wheel={} ignored; inactive in profile",
                self.runtime, index
            );
            self.apply_drive_plan(TriWheelDrivePlan::stopped(false));
            return;
        }
        self.apply_drive_plan(TriWheelDrivePlan::stopped(true));
        if let Some(motors) = self.motors.as_mut() {
            if let Some(bridge) = motors.get_mut(index) {
                bridge.apply_raw_levels(phase_gpio_high, drive_gpio_high);
            }
        }
        if let Some(sleep) = self.sleep.as_mut() {
            sleep.set_high();
        }

        if let Some(motor_pins) = self.motor_pins {
            let pins = motor_pins.bridges[index];
            info!(
                "real runtime {:?}: raw_bridge wheel={} GPIO{}={} GPIO{}={}",
                self.runtime, index, pins.phase, phase_gpio_high, pins.drive, drive_gpio_high
            );
        }
        else {
            info!(
                "real runtime {:?}: raw_bridge wheel={} phase={} drive={}",
                self.runtime, index, phase_gpio_high, drive_gpio_high
            );
        }
    }

    fn pid_plan_from_targets(
        &mut self,
        targets: [f32; 3],
    ) -> TriWheelDrivePlan {
        let targets = self.mask_targets(targets);
        let delta_counts = self.read_encoder_delta_counts();
        self.last_encoder_delta_counts = delta_counts;
        let pid_output = self.pid.update_from_counts(
            targets,
            delta_counts,
            f32::from(self.drive.control_dt_ms) * 0.001,
        );
        self.last_pid_output = self.mask_targets(pid_output);
        let terms = self.pid.last_terms();

        debug!(
            "real runtime {:?}: pid counts=[{},{},{}] target=[{:.2},{:.2},{:.2}] measured=[{:.2},{:.2},{:.2}] out=[{:.2},{:.2},{:.2}]",
            self.runtime,
            self.last_encoder_delta_counts[0],
            self.last_encoder_delta_counts[1],
            self.last_encoder_delta_counts[2],
            terms[0].target_norm,
            terms[1].target_norm,
            terms[2].target_norm,
            terms[0].measured_norm,
            terms[1].measured_norm,
            terms[2].measured_norm,
            self.last_pid_output[0],
            self.last_pid_output[1],
            self.last_pid_output[2]
        );

        self.mask_plan(TriWheelDrivePlan {
            enabled: true,
            wheels: self.last_pid_output.map(|speed| {
                if speed.abs() <= self.drive.motor_deadband_norm {
                    HalfBridgeOutput {
                        phase_high: false,
                        drive_high: false,
                    }
                }
                else {
                    HalfBridgeOutput {
                        phase_high: speed.is_sign_positive(),
                        drive_high: true,
                    }
                }
            }),
        })
    }

    pub(super) fn plan_from_state(&mut self) -> TriWheelDrivePlan {
        if !self.enabled {
            self.last_encoder_delta_counts = [0; 3];
            self.last_drive_targets = [0.0; 3];
            self.last_pid_output = [0.0; 3];
            return TriWheelDrivePlan::stopped(false);
        }

        if let Some(targets) = self.pid_probe_targets {
            return self.pid_plan_from_targets(targets);
        }

        let targets = match self.drive_mode {
            DriveControlMode::TriWheelOmni => {
                let yaw_norm = self
                    .kinematics
                    .normalize_yaw_rate_deg(self.last_yaw_rate_deg_s);
                self.kinematics.compute_wheel_velocities(
                    self.last_speed_norm,
                    self.last_direction_deg,
                    self.last_orientation_deg,
                    yaw_norm,
                )
            }
            DriveControlMode::TwoWheelFront => self.two_wheel_front_targets(),
        };
        self.last_drive_targets = self.mask_targets(targets);
        if self.drive_mode == DriveControlMode::TwoWheelFront {
            // Heading-hold reads encoders non-destructively so /ws read_encoders still works.
            self.apply_heading_hold_correction();
            return self.mask_plan(TriWheelDrivePlan {
                enabled: true,
                wheels: self.last_drive_targets.map(|speed| {
                    owb_core::motion::omni::half_bridge_from_speed(
                        speed,
                        self.drive.motor_deadband_norm,
                    )
                }),
            });
        }
        self.pid_plan_from_targets(targets)
    }

    fn read_encoder_delta_counts(&mut self) -> [i32; 3] {
        self.encoders
            .as_mut()
            .map(PcntEncoderCapture::read_deltas)
            .unwrap_or([0; 3])
    }

    /// Wall-clock millisecond timestamp used by the stiction-kick deadline.
    /// Truncating to `u32` is fine: the kick window is ≤1 s, and the wrap
    /// horizon is ~49 days.
    fn now_ms() -> u32 { owb_core::ext::time::Instant::now().as_millis() as u32 }

    /// Arm or refresh the stiction-kick window if the new commanded forward
    /// component crosses the deadband from below, or its sign flips relative
    /// to the previous step. Caller passes the unfiltered `forward_norm`
    /// (commanded), not the post-floor value.
    ///
    /// Arming condition (kept here for parity with sim_engine.rs):
    ///   `kick_ms > 0 && new_abs > deadband && (prev_sign == 0.0 || new_sign != prev_sign)`
    fn maybe_arm_stiction_kick(
        &mut self,
        new_forward_norm: f32,
    ) {
        if self.stiction_kick_ms == 0 {
            self.last_speed_norm_sign = if new_forward_norm == 0.0 {
                0.0
            }
            else if new_forward_norm > 0.0 {
                1.0
            }
            else {
                -1.0
            };
            self.stiction_kick_active_until_ms = None;
            return;
        }
        let deadband = self.drive.motor_deadband_norm;
        let new_abs = new_forward_norm.abs();
        let new_sign = if new_forward_norm == 0.0 {
            0.0
        }
        else if new_forward_norm > 0.0 {
            1.0
        }
        else {
            -1.0
        };
        let prev_sign = self.last_speed_norm_sign;
        let crossed_up = prev_sign == 0.0 && new_abs > deadband;
        let direction_flipped =
            new_abs > deadband && prev_sign != 0.0 && new_sign != 0.0 && new_sign != prev_sign;
        if crossed_up || direction_flipped {
            let deadline = Self::now_ms().wrapping_add(u32::from(self.stiction_kick_ms));
            self.stiction_kick_active_until_ms = Some(deadline);
            info!(
                "real runtime {:?}: stiction_kick armed until_ms={} (boost={:.3})",
                self.runtime, deadline, self.stiction_kick_boost_norm
            );
        }
        if new_abs <= deadband {
            self.last_speed_norm_sign = 0.0;
        }
        else {
            self.last_speed_norm_sign = new_sign;
        }
    }

    pub(super) fn set_stiction_kick(
        &mut self,
        ms: u16,
        boost: f32,
    ) {
        self.stiction_kick_ms = ms;
        self.stiction_kick_boost_norm = boost;
        if ms == 0 || boost == 0.0 {
            self.stiction_kick_active_until_ms = None;
        }
        info!(
            "real runtime {:?}: stiction_kick ms={} boost={:.3}",
            self.runtime, ms, boost
        );
    }

    pub(super) fn set_wheel_trim(
        &mut self,
        w1: f32,
        w2: f32,
        w3: f32,
    ) {
        // Clamp each axis to [0.0, 2.0] so a bad command can't immediately runaway-double the PWM.
        let clamp = |v: f32| v.clamp(0.0, 2.0);
        self.wheel_trim = [clamp(w1), clamp(w2), clamp(w3)];
        info!(
            "real runtime {:?}: wheel_trim set w1={:.3} w2={:.3} w3={:.3}",
            self.runtime, self.wheel_trim[0], self.wheel_trim[1], self.wheel_trim[2]
        );
    }

    pub(super) fn set_heading_hold(
        &mut self,
        enable: bool,
        kp: f32,
    ) {
        self.heading_hold_enabled = enable;
        self.heading_hold_kp = kp.clamp(0.0, 0.5);
        self.heading_hold_correction = 0.0;
        // Reset baseline so the first correction tick after enabling sees a zero delta.
        self.heading_hold_baseline_raw = self
            .encoders
            .as_ref()
            .map(super::PcntEncoderCapture::raw_counts)
            .unwrap_or([0; 3]);
        info!(
            "real runtime {:?}: heading_hold enabled={} kp={:.4}",
            self.runtime, self.heading_hold_enabled, self.heading_hold_kp
        );
    }

    /// Closed-loop straightness correction. Called once per control tick.
    /// Drives (|W1_delta| - |W2_delta|) toward zero by biasing the per-wheel
    /// drive targets. P-only for v1-alpha; no integral, no heading drift.
    /// Uses non-destructive `raw_counts` so the script-facing `read_encoders`
    /// command still sees the cumulative delta.
    fn apply_heading_hold_correction(&mut self) {
        if !self.heading_hold_enabled
            || self.drive_mode != DriveControlMode::TwoWheelFront
            || !self.enabled
        {
            self.heading_hold_correction = 0.0;
            return;
        }
        let forward_active = self.last_speed_norm.abs() > self.drive.motor_deadband_norm
            && self.last_yaw_rate_deg_s.abs() < 1.0;
        if !forward_active {
            self.heading_hold_correction = 0.0;
            return;
        }
        let current_raw = match self.encoders.as_ref() {
            Some(enc) => enc.raw_counts(),
            None => return,
        };
        // Wrapping subtract → signed delta since last tick. Does NOT consume the encoder state.
        let mut delta = [0i32; 3];
        for i in 0..3 {
            delta[i] = current_raw[i].wrapping_sub(self.heading_hold_baseline_raw[i]) as i32;
        }
        self.heading_hold_baseline_raw = current_raw;
        let imbalance = (delta[0].abs() - delta[1].abs()) as f32;
        let cprev = self.drive.counts_per_wheel_revolution as f32;
        let norm_imbalance = imbalance / cprev.max(1.0);
        let new_corr = self.heading_hold_kp * norm_imbalance;
        self.heading_hold_correction = 0.6 * self.heading_hold_correction + 0.4 * new_corr;
        let bias = self.heading_hold_correction.clamp(-0.5, 0.5);
        self.last_drive_targets[0] = (self.last_drive_targets[0] - bias).clamp(-1.0, 1.0);
        self.last_drive_targets[1] = (self.last_drive_targets[1] + bias).clamp(-1.0, 1.0);
    }

    pub(super) fn read_encoder_snapshot(&mut self) -> EncoderData {
        self.encoders
            .as_mut()
            .map(PcntEncoderCapture::snapshot_and_update)
            .unwrap_or_default()
    }

    pub(super) fn apply_drive_plan(
        &mut self,
        plan: TriWheelDrivePlan,
    ) {
        let plan = self.mask_plan(plan);
        if self.runtime == BoardRuntimeKind::OwbS3V1 {
            if plan.enabled {
                if let Some(motors) = self.motors.as_mut() {
                    for (index, bridge) in motors.iter_mut().enumerate() {
                        let output = plan.wheels[index];
                        // Alpha two-wheel mode is open-loop until encoder PID signs and gains are calibrated.
                        // Direct signed PWM keeps W1/W2 proportional to the joystick target.
                        if self.drive_mode == DriveControlMode::TwoWheelFront && self.enabled {
                            // Apply per-wheel sign flip BEFORE trim scaling so the
                            // PWM bridge sees a signed speed whose sign matches
                            // the desired physical direction of rotation.
                            let phase_sign: f32 = if self.wheel_phase_invert[index] {
                                -1.0
                            }
                            else {
                                1.0
                            };
                            bridge.apply_signed_norm(
                                self.last_drive_targets[index]
                                    * self.wheel_trim[index]
                                    * phase_sign,
                            );
                        }
                        else {
                            bridge.apply(output);
                        }
                    }
                }
                if let Some(sleep) = self.sleep.as_mut() {
                    sleep.set_high();
                }
            }
            else {
                if let Some(sleep) = self.sleep.as_mut() {
                    sleep.set_low();
                }
                if let Some(motors) = self.motors.as_mut() {
                    for (bridge, output) in motors.iter_mut().zip(plan.wheels.iter().copied()) {
                        bridge.apply(output);
                    }
                }
            }
            return;
        }

        if let Some(sleep) = self.sleep.as_mut() {
            if plan.enabled {
                sleep.set_high();
            }
            else {
                sleep.set_low();
            }
        }

        if let Some(motors) = self.motors.as_mut() {
            for (bridge, output) in motors.iter_mut().zip(plan.wheels.iter().copied()) {
                bridge.apply(output);
            }
        }
    }
}
