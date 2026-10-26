use crate::ek;
#[cfg(feature = "net")]
pub use crate::node::command_bus::{
    CommandEnvelope,
    COMMAND_BUS,
    COMMAND_QUERY_LOCK,
    COMMAND_REPLY_BUS,
};
pub use crate::{
    ctrl::traits::{CommandExecutor, ControllerError, Imu, Lights, MotorPwm},
    msg::legacy::{
        AutonomyCommand,
        CommandReply,
        EncoderData,
        EnvData,
        I2CCommand,
        ImuData,
        LEDCommand,
        PowerCommand,
        PowerData,
        SystemCommand,
    },
};

pub struct SystemController<P, M, H> {
    pub pwm: P,
    pub imu: M,
    pub lights: H,
    pub embodied: ek,
    pub motor_channels: [(usize, usize); 3],
}

impl<P, M, H> SystemController<P, M, H>
where
    P: MotorPwm,
    M: Imu,
    H: Lights,
{
    fn normalize_degrees(angle: f32) -> f32 {
        if !angle.is_finite() {
            return 0.0;
        }
        let mut wrapped = angle % 360.0;
        if wrapped < 0.0 {
            wrapped += 360.0;
        }
        wrapped
    }

    pub fn new(
        pwm: P,
        imu: M,
        lights: H,
        wheel_radius: f32,
        robot_radius: f32,
        max_wheel_rad_s: f32,
    ) -> Self {
        Self {
            pwm,
            imu,
            lights,
            embodied: ek::new(wheel_radius, robot_radius, max_wheel_rad_s),
            motor_channels: [(0, 1), (2, 3), (4, 5)],
        }
    }

    pub fn handle_i2c(
        &mut self,
        cmd: I2CCommand,
    ) -> Result<Option<ImuData>, ControllerError<P::Error, M::Error>>
    where
        P::Error: core::fmt::Debug,
        M::Error: core::fmt::Debug,
    {
        match cmd {
            I2CCommand::T {
                direction_deg,
                speed_norm,
            } => {
                let s = speed_norm.clamp(-1.0, 1.0);
                tracing::trace!(
                    target: "owb::sysctrl",
                    direction_deg,
                    requested = speed_norm,
                    applied = s,
                    "translate command"
                );
                self.apply_wheels(self.embodied.compute_wheel_velocities(
                    s,
                    direction_deg,
                    0.0,
                    0.0,
                ))
                .map_err(ControllerError::Pwm)?;
                Ok(None)
            }
            I2CCommand::Y {
                yaw_rate_deg_s,
                orientation_deg,
            } => {
                let orientation = Self::normalize_degrees(orientation_deg.unwrap_or(0.0));
                let yaw_rate_norm = self.embodied.normalize_yaw_rate_deg(yaw_rate_deg_s);
                tracing::trace!(
                    target: "owb::sysctrl",
                    requested_yaw_deg_s = yaw_rate_deg_s,
                    normalised = yaw_rate_norm,
                    orientation_deg = orientation,
                    "yaw command"
                );
                self.apply_wheels(self.embodied.compute_wheel_velocities(
                    0.0,
                    0.0,
                    orientation,
                    yaw_rate_norm,
                ))
                .map_err(ControllerError::Pwm)?;
                Ok(None)
            }
            I2CCommand::O {
                direction_deg,
                speed_norm,
                yaw_rate_deg_s,
                orientation_deg,
            } => {
                let orientation = Self::normalize_degrees(orientation_deg.unwrap_or(0.0));
                let s = speed_norm.clamp(-1.0, 1.0);
                let yaw_rate_norm = self.embodied.normalize_yaw_rate_deg(yaw_rate_deg_s);
                tracing::trace!(
                    target: "owb::sysctrl",
                    direction_deg,
                    requested_speed = speed_norm,
                    applied_speed = s,
                    requested_yaw_deg_s = yaw_rate_deg_s,
                    normalised_yaw = yaw_rate_norm,
                    orientation_deg = orientation,
                    "omni command"
                );
                self.apply_wheels(self.embodied.compute_wheel_velocities(
                    s,
                    direction_deg,
                    orientation,
                    yaw_rate_norm,
                ))
                .map_err(ControllerError::Pwm)?;
                Ok(None)
            }
            I2CCommand::ReadIMU => self.imu.read().map(Some).map_err(|e| {
                tracing::warn!(?e, "imu read error");
                ControllerError::Imu(e)
            }),
            I2CCommand::Probe { wheel, forward } => {
                tracing::trace!(target: "owb::sysctrl", wheel, forward, "probe command");
                self.pwm.enable().map_err(|e| {
                    tracing::warn!(?e, "pwm enable");
                    ControllerError::Pwm(e)
                })?;
                for idx in 0..3 {
                    let value = if idx == usize::from(wheel.min(2)) {
                        if forward {
                            1.0
                        }
                        else {
                            -1.0
                        }
                    }
                    else {
                        0.0
                    };
                    self.pwm.set_wheel(idx, value).map_err(|e| {
                        tracing::warn!(?e, "pwm set_wheel");
                        ControllerError::Pwm(e)
                    })?;
                }
                Ok(None)
            }
            I2CCommand::ProbeSpeed { wheel, speed_norm } => {
                let index = usize::from(wheel.min(2));
                let speed = speed_norm.clamp(-1.0, 1.0);
                tracing::trace!(
                    target: "owb::sysctrl",
                    wheel = index,
                    requested = speed_norm,
                    applied = speed,
                    "probe speed command"
                );
                self.pwm.enable().map_err(|e| {
                    tracing::warn!(?e, "pwm enable");
                    ControllerError::Pwm(e)
                })?;
                for idx in 0..3 {
                    let value = if idx == index { speed } else { 0.0 };
                    self.pwm.set_wheel(idx, value).map_err(|e| {
                        tracing::warn!(?e, "pwm set_wheel");
                        ControllerError::Pwm(e)
                    })?;
                }
                Ok(None)
            }
            I2CCommand::PidProbeSpeed { wheel, speed_norm } => {
                let index = usize::from(wheel.min(2));
                let speed = speed_norm.clamp(-1.0, 1.0);
                tracing::trace!(
                    target: "owb::sysctrl",
                    wheel = index,
                    requested = speed_norm,
                    applied = speed,
                    "pid probe speed command"
                );
                self.pwm.enable().map_err(|e| {
                    tracing::warn!(?e, "pwm enable");
                    ControllerError::Pwm(e)
                })?;
                for idx in 0..3 {
                    let value = if idx == index { speed } else { 0.0 };
                    self.pwm.set_wheel(idx, value).map_err(|e| {
                        tracing::warn!(?e, "pwm set_wheel");
                        ControllerError::Pwm(e)
                    })?;
                }
                Ok(None)
            }
            I2CCommand::RawBridge { .. }
            | I2CCommand::IndicatorLed { .. }
            | I2CCommand::WheelTrim { .. }
            | I2CCommand::HeadingHold { .. }
            | I2CCommand::StictionKick { .. }
            | I2CCommand::DriveLoop { .. } => Ok(None),
            I2CCommand::ReadEnv
            | I2CCommand::ReadPower
            | I2CCommand::ReadEncoders
            | I2CCommand::Snap => Ok(None),
            I2CCommand::Enable => {
                tracing::trace!(target: "owb::sysctrl", "enable command");
                self.pwm.enable().map_err(|e| {
                    tracing::warn!(?e, "pwm enable");
                    ControllerError::Pwm(e)
                })?;
                self.imu.enable().map_err(|e| {
                    tracing::warn!(?e, "imu enable");
                    ControllerError::Imu(e)
                })?;
                Ok(None)
            }
            I2CCommand::Disable => {
                tracing::trace!(target: "owb::sysctrl", "disable command");
                let _ = self.apply_wheels([0.0, 0.0, 0.0]);
                self.pwm.disable().map_err(|e| {
                    tracing::warn!(?e, "pwm disable");
                    ControllerError::Pwm(e)
                })?;
                self.imu.disable().map_err(|e| {
                    tracing::warn!(?e, "imu disable");
                    ControllerError::Imu(e)
                })?;
                Ok(None)
            }
        }
    }

    fn apply_wheels(
        &mut self,
        wheel_speeds: [f32; 3],
    ) -> Result<(), P::Error>
    where
        P::Error: core::fmt::Debug,
    {
        let mut applied = [0.0; 3];
        for (i, &speed) in wheel_speeds.iter().enumerate() {
            // Final safety gate before PWM: a non-finite speed must become a
            // hard zero. `NaN.clamp(..)` returns NaN, so the clamp alone is not
            // sufficient to keep a bad value off an actuated wheel.
            let value = if speed.is_finite() {
                speed.clamp(-1.0, 1.0)
            }
            else {
                0.0
            };
            applied[i] = value;
            self.pwm.set_wheel(i, value).map_err(|e| {
                tracing::warn!(?e, "pwm set_wheel");
                e
            })?;
        }
        tracing::trace!(target: "owb::sysctrl", wheels = ?applied, "apply wheels");
        Ok(())
    }
}

impl<P, M, H> CommandExecutor for SystemController<P, M, H>
where
    P: MotorPwm,
    M: Imu,
    H: Lights,
    P::Error: core::fmt::Debug,
    M::Error: core::fmt::Debug,
    H::Error: core::fmt::Debug,
{
    type Error = ControllerError<P::Error, M::Error>;

    fn execute_command(
        &mut self,
        cmd: SystemCommand,
    ) -> Result<CommandReply, Self::Error> {
        match cmd {
            SystemCommand::I(ic) => self.handle_i2c(ic).map(|maybe| match maybe {
                Some(data) => CommandReply::ImuData(data),
                None => CommandReply::Ok,
            }),
            SystemCommand::L(lc) => {
                let _ = match lc {
                    LEDCommand::On => self.lights.set_brightness(255),
                    LEDCommand::Off => self.lights.set_brightness(0),
                    LEDCommand::SC { r, g, b } => {
                        let v = r.max(g).max(b);
                        self.lights.set_brightness(v)
                    }
                    LEDCommand::Brightness { v } => self.lights.set_brightness(v),
                    // This high-level controller only tracks brightness; the
                    // animated effects are rendered by the board runtime.
                    LEDCommand::Breath { r, g, b } => self.lights.set_brightness(r.max(g).max(b)),
                    LEDCommand::Wheel => self.lights.set_brightness(255),
                };
                Ok(CommandReply::Ok)
            }
            SystemCommand::P(_) | SystemCommand::A(_) => Ok(CommandReply::Ok),
        }
    }
}

#[cfg(feature = "net")]
impl<P, M, H> SystemController<P, M, H>
where
    P: MotorPwm,
    M: Imu,
    H: Lights,
    P::Error: core::fmt::Debug,
    M::Error: core::fmt::Debug,
    H::Error: core::fmt::Debug,
{
    /// Runs an async control loop receiving commands from the global bus.
    pub async fn run(&mut self) -> ! { crate::node::command_bus::run_command_bus(self).await }
}
