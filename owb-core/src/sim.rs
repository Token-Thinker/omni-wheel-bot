//! Host-side simulation helpers for exercising the `SystemController`.
//!
//! This module is available when the `sim` feature is enabled (requires
//! `std`). It provides lightweight mock implementations of the `MotorPwm`,
//! `Imu`, and `Lights` traits plus telemetry helpers so host binaries can
//! emulate MCU behaviour without any hardware.

use std::{
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};

use serde::Serialize;
use tracing::warn;

use crate::{
    ctrl::{
        sysctrl::{AutonomyCommand, EnvData, I2CCommand, PowerCommand, PowerData},
        CommandExecutor,
        CommandReply,
        Imu,
        ImuData,
        Lights,
        MotorPwm,
        SystemCommand,
        SystemController,
    },
    motion::{kinematics::EmbodiedKinematics, pid::PidController},
};

const DEFAULT_MAX_WHEEL_RAD_S: f32 = 7.0;
const SIM_PID_RESPONSE_PER_S: f32 = 5.0;

/// Bundle returned by [`Builder::build`] containing a controller and shared
/// simulation context.
pub struct Bundle {
    pub controller: Controller,
    pub context: Context,
}

/// Builder for constructing a simulated controller with optional overrides.
#[derive(Default)]
pub struct Builder {
    wheel_radius: f32,
    robot_radius: f32,
    max_wheel_rad_s: f32,
}

impl Builder {
    /// Creates a builder with the default Omni-Wheel geometry.
    pub fn new() -> Self {
        Self {
            wheel_radius: 0.148,
            robot_radius: 0.195,
            max_wheel_rad_s: DEFAULT_MAX_WHEEL_RAD_S,
        }
    }

    /// Overrides the wheel radius used for kinematics.
    pub fn wheel_radius(
        mut self,
        radius: f32,
    ) -> Self {
        self.wheel_radius = radius;
        self
    }

    /// Overrides the robot radius used for kinematics.
    pub fn robot_radius(
        mut self,
        radius: f32,
    ) -> Self {
        self.robot_radius = radius;
        self
    }

    /// Overrides the maximum wheel angular velocity used for normalisation.
    pub fn max_wheel_rad_s(
        mut self,
        value: f32,
    ) -> Self {
        self.max_wheel_rad_s = value;
        self
    }

    /// Builds the simulated controller bundle.
    pub fn build(self) -> Bundle {
        let shared = Shared::new(self.wheel_radius, self.robot_radius, self.max_wheel_rad_s);
        let motor = Motor::new(shared.clone());
        let imu = ImuMock::new(shared.clone());
        let head = LightsMock::new(shared.clone());
        let controller = SystemController::new(
            motor,
            imu,
            head,
            self.wheel_radius,
            self.robot_radius,
            self.max_wheel_rad_s,
        );

        Bundle {
            controller,
            context: Context { shared },
        }
    }
}

/// Convenience constructor using default geometry.
pub fn build_default() -> Bundle { Builder::new().build() }

/// Controller type alias with simulated peripherals.
pub type Controller = SystemController<Motor, ImuMock, LightsMock>;

/// Simulation context shared between the controller and telemetry loop.
#[derive(Clone)]
pub struct Context {
    shared: Shared,
}

impl Context {
    /// Advances simulated sensors and battery state by `dt`.
    pub fn advance(
        &self,
        dt: Duration,
    ) {
        self.shared.advance(dt);
    }

    /// Captures the current telemetry snapshot.
    pub fn snapshot(&self) -> TelemetryFrame { self.shared.snapshot() }

    /// Returns whether the simulated motors are currently enabled.
    pub fn motors_enabled(&self) -> bool { self.shared.motor_enabled() }
}

#[derive(Clone)]
struct Shared {
    inner: Arc<Inner>,
}

struct Inner {
    started: Instant,
    motor: Mutex<MotorState>,
    imu: Mutex<ImuState>,
    lights: Mutex<LightsState>,
    battery: Mutex<BatteryState>,
    motion: Mutex<MotionState>,
    pid: Mutex<PidState>,
    kinematics: EmbodiedKinematics,
    wheel_radius: f32,
    robot_radius: f32,
}

impl Shared {
    fn new(
        wheel_radius: f32,
        robot_radius: f32,
        max_wheel_rad_s: f32,
    ) -> Self {
        Self {
            inner: Arc::new(Inner {
                started: Instant::now(),
                motor: Mutex::new(MotorState::default()),
                imu: Mutex::new(ImuState::default()),
                lights: Mutex::new(LightsState::default()),
                battery: Mutex::new(BatteryState { voltage: 12.4 }),
                motion: Mutex::new(MotionState::default()),
                pid: Mutex::new(PidState::default()),
                kinematics: EmbodiedKinematics::new(wheel_radius, robot_radius, max_wheel_rad_s),
                wheel_radius,
                robot_radius,
            }),
        }
    }

    fn advance(
        &self,
        dt: Duration,
    ) {
        let dt_s = dt.as_secs_f32().max(1e-3);
        let (enabled, commanded_wheels) = {
            let motor = self.inner.motor.lock().unwrap();
            (motor.enabled, motor.speeds)
        };
        let measured_wheel_norm = {
            let mut pid = self.inner.pid.lock().unwrap();
            for (idx, &commanded) in commanded_wheels.iter().enumerate() {
                let target = if enabled { commanded } else { 0.0 };
                let measured = pid.measured_norm[idx];
                let output = {
                    let controller = &mut pid.controllers[idx];
                    if !enabled && target.abs() < 1e-6 {
                        controller.reset();
                    }
                    controller.update(target, measured, dt_s)
                };
                let response = (SIM_PID_RESPONSE_PER_S * dt_s).clamp(0.0, 1.0);
                let next_measured = measured + (output - measured) * response;
                pid.target_norm[idx] = target;
                pid.measured_norm[idx] = next_measured.clamp(-1.0, 1.0);
                pid.error_norm[idx] = (target - pid.measured_norm[idx]).clamp(-1.0, 1.0);
                pid.output_norm[idx] = output;
            }
            pid.measured_norm
        };
        let measured_wheels = self
            .inner
            .kinematics
            .wheel_norm_to_rad_s(measured_wheel_norm);
        let avg_speed = measured_wheels.iter().map(|speed| speed.abs()).sum::<f32>()
            / measured_wheels.len() as f32;
        let yaw_rate_deg_s = {
            let motion = self.inner.motion.lock().unwrap();
            motion.yaw_rate_deg_s
        };

        {
            let mut imu = self.inner.imu.lock().unwrap();
            if imu.enabled {
                let delta_yaw = yaw_rate_deg_s * dt.as_secs_f32();
                imu.yaw = (imu.yaw + delta_yaw).rem_euclid(360.0);
                imu.pitch = 0.0;
                imu.roll = 0.0;
                let yaw_rate = if dt.as_secs_f32() > 0.0 {
                    yaw_rate_deg_s
                }
                else {
                    0.0
                };
                imu.data.gyro = (0.0, 0.0, yaw_rate);
                imu.data.accel = (
                    measured_wheels[0] * 0.6,
                    measured_wheels[1] * 0.6,
                    9.81 + measured_wheels[2] * 0.3,
                );
                imu.data.temp = 28.0 + avg_speed * 3.5;
            }
            else {
                imu.data.gyro = (0.0, 0.0, 0.0);
            }
        }

        {
            let mut battery = self.inner.battery.lock().unwrap();
            if enabled {
                let drain = (avg_speed * dt.as_secs_f32() * 0.06).min(0.08);
                battery.voltage = (battery.voltage - drain).max(11.4);
            }
            else {
                battery.voltage = (battery.voltage + 0.015).min(12.4);
            }
        }
    }

    fn snapshot(&self) -> TelemetryFrame {
        let timestamp_ms = self.inner.started.elapsed().as_millis() as u64;
        let motor = self.inner.motor.lock().unwrap();
        let imu = self.inner.imu.lock().unwrap();
        let led = self.inner.lights.lock().unwrap();
        let battery = self.inner.battery.lock().unwrap();
        let motion = self.inner.motion.lock().unwrap();
        let pid = self.inner.pid.lock().unwrap();
        let measured_wheel_rad_s = self.inner.kinematics.wheel_norm_to_rad_s(pid.measured_norm);

        TelemetryFrame {
            timestamp_ms,
            motors: TelemetryMotors {
                enabled: motor.enabled,
                wheel_speed_rad_s: measured_wheel_rad_s,
                wheel_speed_norm: pid.measured_norm,
                wheel_radius_m: self.inner.wheel_radius,
                robot_radius_m: self.inner.robot_radius,
            },
            motion: TelemetryMotion {
                direction_deg: motion.direction_deg,
                speed_mps: motion.speed_mps,
                speed_norm: Some(motion.speed_norm),
                yaw_rate_deg_s: motion.yaw_rate_deg_s,
            },
            imu: TelemetryImu {
                enabled: imu.enabled,
                yaw_deg: imu.yaw,
                pitch_deg: imu.pitch,
                roll_deg: imu.roll,
                gyro_dps: [imu.data.gyro.0, imu.data.gyro.1, imu.data.gyro.2],
                accel_mps2: [imu.data.accel.0, imu.data.accel.1, imu.data.accel.2],
                temp_c: imu.data.temp,
            },
            led: TelemetryLed {
                brightness: led.brightness,
            },
            battery_v: battery.voltage,
            pid: TelemetryPid {
                target_norm: pid.target_norm,
                measured_norm: pid.measured_norm,
                error_norm: pid.error_norm,
                output_norm: pid.output_norm,
            },
        }
    }

    fn motor_enabled(&self) -> bool { self.inner.motor.lock().unwrap().enabled }

    fn set_motion(
        &self,
        direction_deg: f32,
        speed_norm: f32,
        yaw_rate_deg_s: f32,
    ) {
        let clamped = speed_norm.clamp(-1.0, 1.0);
        let max_linear_speed = self.inner.kinematics.max_linear_speed();
        let speed_mps = clamped * max_linear_speed;
        let orientation_deg = {
            let imu = self.inner.imu.lock().unwrap();
            imu.yaw
        };
        let yaw_rate_norm = self.inner.kinematics.normalize_yaw_rate_deg(yaw_rate_deg_s);
        let wheel_speed_norm = if clamped.abs() < 1e-6 && yaw_rate_deg_s.abs() < 1e-6 {
            [0.0; 3]
        }
        else {
            self.inner.kinematics.compute_wheel_velocities(
                clamped,
                direction_deg,
                orientation_deg,
                yaw_rate_norm,
            )
        };
        let wheel_speed_rad_s = self.inner.kinematics.wheel_norm_to_rad_s(wheel_speed_norm);
        let mut motion = self.inner.motion.lock().unwrap();
        motion.direction_deg = direction_deg;
        motion.speed_norm = clamped;
        motion.speed_mps = speed_mps;
        motion.yaw_rate_deg_s = yaw_rate_deg_s;
        motion.wheel_speed_rad_s = wheel_speed_rad_s;
        motion.wheel_speed_norm = wheel_speed_norm;
    }

    fn motion(&self) -> MotionState { *self.inner.motion.lock().unwrap() }
}

struct MotorState {
    enabled: bool,
    speeds: [f32; 3],
}

impl Default for MotorState {
    fn default() -> Self {
        Self {
            enabled: true,
            speeds: [0.0; 3],
        }
    }
}

#[derive(Clone, Copy)]
struct MotionState {
    direction_deg: f32,
    speed_norm: f32,
    speed_mps: f32,
    yaw_rate_deg_s: f32,
    wheel_speed_rad_s: [f32; 3],
    wheel_speed_norm: [f32; 3],
}

impl Default for MotionState {
    fn default() -> Self {
        Self {
            direction_deg: 0.0,
            speed_norm: 0.0,
            speed_mps: 0.0,
            yaw_rate_deg_s: 0.0,
            wheel_speed_rad_s: [0.0; 3],
            wheel_speed_norm: [0.0; 3],
        }
    }
}

#[derive(Clone, Copy)]
struct ImuState {
    enabled: bool,
    yaw: f32,
    pitch: f32,
    roll: f32,
    data: ImuData,
}

impl Default for ImuState {
    fn default() -> Self {
        Self {
            enabled: false,
            yaw: 0.0,
            pitch: 0.0,
            roll: 0.0,
            data: ImuData::default(),
        }
    }
}

#[derive(Default)]
struct LightsState {
    brightness: u8,
}

#[derive(Default)]
struct BatteryState {
    voltage: f32,
}

#[derive(Clone, Copy)]
struct PidState {
    controllers: [PidController; 3],
    target_norm: [f32; 3],
    measured_norm: [f32; 3],
    error_norm: [f32; 3],
    output_norm: [f32; 3],
}

impl Default for PidState {
    fn default() -> Self {
        Self {
            controllers: [PidController::new(0.9, 0.25, 0.04, 0.75, 1.0); 3],
            target_norm: [0.0; 3],
            measured_norm: [0.0; 3],
            error_norm: [0.0; 3],
            output_norm: [0.0; 3],
        }
    }
}

/// Error type reported by simulated peripherals.
#[derive(Debug)]
pub enum SimDeviceError {
    MotorDisabled,
    InvalidWheel(usize),
    ImuDisabled,
}

impl std::fmt::Display for SimDeviceError {
    fn fmt(
        &self,
        f: &mut std::fmt::Formatter<'_>,
    ) -> std::fmt::Result {
        match self {
            SimDeviceError::MotorDisabled => write!(f, "motor outputs disabled"),
            SimDeviceError::InvalidWheel(idx) => write!(f, "invalid wheel index {idx}"),
            SimDeviceError::ImuDisabled => write!(f, "imu disabled"),
        }
    }
}

impl std::error::Error for SimDeviceError {}

/// Simulated motor PWM peripheral.
pub struct Motor {
    shared: Shared,
}

impl Motor {
    fn new(shared: Shared) -> Self { Self { shared } }

    fn set_enabled(
        &self,
        enabled: bool,
    ) {
        let mut state = self.shared.inner.motor.lock().unwrap();
        state.enabled = enabled;
    }

    fn record_motion(
        &self,
        direction_deg: f32,
        speed_norm: f32,
        yaw_rate_deg_s: f32,
    ) {
        self.shared
            .set_motion(direction_deg, speed_norm, yaw_rate_deg_s);
    }

    fn current_motion(&self) -> MotionState { self.shared.motion() }

    fn is_enabled(&self) -> bool { self.shared.inner.motor.lock().unwrap().enabled }
}

impl MotorPwm for Motor {
    type Error = SimDeviceError;

    fn enable(&mut self) -> Result<(), Self::Error> {
        let mut state = self.shared.inner.motor.lock().unwrap();
        state.enabled = true;
        Ok(())
    }

    fn disable(&mut self) -> Result<(), Self::Error> {
        let mut state = self.shared.inner.motor.lock().unwrap();
        state.enabled = false;
        Ok(())
    }

    fn set_wheel(
        &mut self,
        index: usize,
        speed: f32,
    ) -> Result<(), Self::Error> {
        let mut state = self.shared.inner.motor.lock().unwrap();
        if index >= state.speeds.len() {
            return Err(SimDeviceError::InvalidWheel(index));
        }
        if !state.enabled {
            return Err(SimDeviceError::MotorDisabled);
        }
        state.speeds[index] = speed.clamp(-1.0, 1.0);
        Ok(())
    }
}

/// Simulated IMU peripheral.
pub struct ImuMock {
    shared: Shared,
}

impl ImuMock {
    fn new(shared: Shared) -> Self { Self { shared } }

    fn is_enabled(&self) -> bool { self.shared.inner.imu.lock().unwrap().enabled }
}

impl Imu for ImuMock {
    type Error = SimDeviceError;

    fn enable(&mut self) -> Result<(), Self::Error> {
        let mut state = self.shared.inner.imu.lock().unwrap();
        state.enabled = true;
        state.yaw = 0.0;
        state.pitch = 0.0;
        state.roll = 0.0;
        state.data = ImuData::default();
        Ok(())
    }

    fn disable(&mut self) -> Result<(), Self::Error> {
        let mut state = self.shared.inner.imu.lock().unwrap();
        state.enabled = false;
        Ok(())
    }

    fn read(&mut self) -> Result<ImuData, Self::Error> {
        let state = self.shared.inner.imu.lock().unwrap();
        if !state.enabled {
            return Err(SimDeviceError::ImuDisabled);
        }
        Ok(state.data)
    }
}

/// Simulated LED PWM peripheral.
pub struct LightsMock {
    shared: Shared,
}

impl LightsMock {
    fn new(shared: Shared) -> Self { Self { shared } }
}

impl Lights for LightsMock {
    type Error = SimDeviceError;

    fn set_brightness(
        &mut self,
        value: u8,
    ) -> Result<(), Self::Error> {
        let mut state = self.shared.inner.lights.lock().unwrap();
        state.brightness = value;
        Ok(())
    }
}

/// Telemetry frame emitted by the simulator.
#[derive(Clone, Debug, Serialize)]
pub struct TelemetryFrame {
    pub timestamp_ms: u64,
    pub motors: TelemetryMotors,
    pub motion: TelemetryMotion,
    pub imu: TelemetryImu,
    pub led: TelemetryLed,
    pub battery_v: f32,
    pub pid: TelemetryPid,
}

#[derive(Clone, Copy, Debug, Serialize)]
pub struct TelemetryMotors {
    pub enabled: bool,
    pub wheel_speed_rad_s: [f32; 3],
    pub wheel_speed_norm: [f32; 3],
    pub wheel_radius_m: f32,
    pub robot_radius_m: f32,
}

#[derive(Clone, Copy, Debug, Default, Serialize)]
pub struct TelemetryMotion {
    pub direction_deg: f32,
    pub speed_mps: f32,
    pub speed_norm: Option<f32>,
    pub yaw_rate_deg_s: f32,
}

#[derive(Clone, Copy, Debug, Serialize)]
pub struct TelemetryImu {
    pub enabled: bool,
    pub yaw_deg: f32,
    pub pitch_deg: f32,
    pub roll_deg: f32,
    pub gyro_dps: [f32; 3],
    pub accel_mps2: [f32; 3],
    pub temp_c: f32,
}

#[derive(Clone, Copy, Debug, Serialize)]
pub struct TelemetryLed {
    pub brightness: u8,
}

#[derive(Clone, Copy, Debug, Serialize)]
pub struct TelemetryPid {
    pub target_norm: [f32; 3],
    pub measured_norm: [f32; 3],
    pub error_norm: [f32; 3],
    pub output_norm: [f32; 3],
}

/// Executes a `SystemCommand` against the controller and maps the result into
/// a simple acknowledgement.
pub fn apply_command(
    controller: &mut Controller,
    cmd: SystemCommand,
) -> Result<CommandResult, CommandError> {
    match cmd {
        SystemCommand::I(I2CCommand::ReadEnv) => {
            let temp_c = controller.imu.read().map(|data| data.temp).unwrap_or(24.0);
            Ok(CommandResult::EnvData(EnvData {
                temp_c,
                humidity_percent: 45.0,
            }))
        }
        SystemCommand::I(I2CCommand::ReadPower) => Ok(CommandResult::PowerData(PowerData {
            battery_mv: 7400,
            soc_percent: 62.5,
        })),
        SystemCommand::I(I2CCommand::ReadEncoders) => Ok(CommandResult::EncoderData(
            crate::ctrl::sysctrl::EncoderData {
                counts: [0; 3],
                delta_counts: [0; 3],
            },
        )),
        SystemCommand::I(I2CCommand::Snap) => Ok(CommandResult::Ok),
        SystemCommand::I(ic) => {
            let (command_to_send, new_state) = match ic {
                I2CCommand::Enable => (I2CCommand::Enable, Some(true)),
                I2CCommand::Disable => (I2CCommand::Disable, Some(false)),
                other => (other, None),
            };

            let previous_motion = controller.pwm.current_motion();

            controller
                .execute_command(SystemCommand::I(command_to_send))
                .map(|reply| {
                    if let Some(enabled) = new_state {
                        controller.pwm.set_enabled(enabled);
                        if !enabled {
                            controller
                                .pwm
                                .record_motion(previous_motion.direction_deg, 0.0, 0.0);
                        }
                    }
                    match command_to_send {
                        I2CCommand::T {
                            direction_deg,
                            speed_norm,
                        } => controller.pwm.record_motion(direction_deg, speed_norm, 0.0),
                        I2CCommand::O {
                            direction_deg,
                            speed_norm,
                            yaw_rate_deg_s,
                            ..
                        } => {
                            controller
                                .pwm
                                .record_motion(direction_deg, speed_norm, yaw_rate_deg_s)
                        }
                        I2CCommand::Y { yaw_rate_deg_s, .. } => {
                            controller.pwm.record_motion(
                                previous_motion.direction_deg,
                                0.0,
                                yaw_rate_deg_s,
                            );
                        }
                        I2CCommand::Probe { .. } => {
                            controller
                                .pwm
                                .record_motion(previous_motion.direction_deg, 0.0, 0.0);
                        }
                        I2CCommand::ProbeSpeed { .. } => {
                            controller
                                .pwm
                                .record_motion(previous_motion.direction_deg, 0.0, 0.0);
                        }
                        I2CCommand::PidProbeSpeed { .. } => {
                            controller
                                .pwm
                                .record_motion(previous_motion.direction_deg, 0.0, 0.0);
                        }
                        I2CCommand::RawBridge { .. } => {
                            controller
                                .pwm
                                .record_motion(previous_motion.direction_deg, 0.0, 0.0);
                        }
                        I2CCommand::IndicatorLed { .. } => {}
                        I2CCommand::Disable => {
                            controller
                                .pwm
                                .record_motion(previous_motion.direction_deg, 0.0, 0.0);
                        }
                        _ => {}
                    }
                    match reply {
                        CommandReply::ImuData(data) => CommandResult::ImuData(data),
                        CommandReply::EnvData(data) => CommandResult::EnvData(data),
                        CommandReply::PowerData(data) => CommandResult::PowerData(data),
                        CommandReply::EncoderData(data) => CommandResult::EncoderData(data),
                        CommandReply::Ok | CommandReply::Err => CommandResult::Ok,
                    }
                })
                .map_err(|_| CommandError::Controller)
        }
        SystemCommand::L(lc) => controller
            .execute_command(SystemCommand::L(lc))
            .map(|_| CommandResult::Ok)
            .map_err(|_| CommandError::Controller),
        SystemCommand::P(pc) => match pc {
            PowerCommand::MotorsToggle => {
                let enable = !controller.pwm.is_enabled();
                let cmd = if enable {
                    I2CCommand::Enable
                }
                else {
                    I2CCommand::Disable
                };
                apply_command(controller, SystemCommand::I(cmd))
            }
            PowerCommand::DozeToggle => {
                if controller.imu.is_enabled() {
                    controller
                        .imu
                        .disable()
                        .map_err(|_| CommandError::Controller)?;
                }
                else {
                    controller
                        .imu
                        .enable()
                        .map_err(|_| CommandError::Controller)?;
                }
                Ok(CommandResult::Ok)
            }
            PowerCommand::CameraToggle => Ok(CommandResult::Ok),
            PowerCommand::Estop => {
                controller
                    .pwm
                    .disable()
                    .map_err(|_| CommandError::Controller)?;
                controller.pwm.record_motion(0.0, 0.0, 0.0);
                Ok(CommandResult::Ok)
            }
        },
        SystemCommand::A(_ac) => match _ac {
            AutonomyCommand::AutoMode | AutonomyCommand::Start | AutonomyCommand::Stop => {
                Ok(CommandResult::Ok)
            }
        },
    }
}

/// Outcome of applying a command.
pub enum CommandResult {
    Ok,
    ImuData(ImuData),
    EnvData(EnvData),
    PowerData(PowerData),
    EncoderData(crate::ctrl::sysctrl::EncoderData),
}

/// Command failure reason.
#[derive(Debug)]
pub enum CommandError {
    Controller,
    Invalid(String),
}

impl CommandError {
    pub fn message(&self) -> &str {
        match self {
            CommandError::Controller => "controller error",
            CommandError::Invalid(msg) => msg.as_str(),
        }
    }
}

/// Helper to log device errors with context.
pub fn log_device_error(
    err: &SimDeviceError,
    context: &str,
) {
    warn!("{context}: {err}");
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pid_telemetry_tracks_translate_command() {
        let bundle = build_default();
        let context = bundle.context;
        let mut controller = bundle.controller;

        let result = apply_command(
            &mut controller,
            SystemCommand::I(I2CCommand::T {
                direction_deg: 90.0,
                speed_norm: 0.6,
            }),
        );
        assert!(matches!(result, Ok(CommandResult::Ok)));

        for _ in 0..5 {
            context.advance(Duration::from_millis(100));
        }

        let frame = context.snapshot();
        assert!(frame.pid.target_norm.iter().any(|value| value.abs() > 0.01));
        assert!(frame
            .pid
            .measured_norm
            .iter()
            .any(|value| value.abs() > 0.01));
        assert!(frame.pid.output_norm.iter().any(|value| value.abs() > 0.01));
    }

    #[test]
    fn estop_zeroes_pid_targets() {
        let bundle = build_default();
        let context = bundle.context;
        let mut controller = bundle.controller;

        let _ = apply_command(
            &mut controller,
            SystemCommand::I(I2CCommand::T {
                direction_deg: 0.0,
                speed_norm: 0.8,
            }),
        );
        context.advance(Duration::from_millis(100));

        let result = apply_command(&mut controller, SystemCommand::P(PowerCommand::Estop));
        assert!(matches!(result, Ok(CommandResult::Ok)));

        for _ in 0..3 {
            context.advance(Duration::from_millis(100));
        }

        let frame = context.snapshot();
        assert!(frame.pid.target_norm.iter().all(|value| value.abs() < 1e-6));
        assert!(frame
            .pid
            .measured_norm
            .iter()
            .all(|value| value.abs() < 0.4));
    }
}
