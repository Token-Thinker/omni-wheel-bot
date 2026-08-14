//! Generic controller traits shared by board runtimes and transport nodes.

use crate::msg::{CommandReply, I2CCommand, ImuData, SystemCommand};

// Generic device traits to keep core MCU-agnostic.
pub trait MotorPwm {
    type Error;

    fn enable(&mut self) -> Result<(), Self::Error>;

    fn disable(&mut self) -> Result<(), Self::Error> { Ok(()) }

    fn set_wheel(
        &mut self,
        index: usize,
        speed: f32,
    ) -> Result<(), Self::Error>; // [-1.0, 1.0]
}

pub trait Imu {
    type Error;

    fn enable(&mut self) -> Result<(), Self::Error> { Ok(()) }

    fn disable(&mut self) -> Result<(), Self::Error> { Ok(()) }

    fn read(&mut self) -> Result<ImuData, Self::Error>;
}

/// Simple lights control over a single PWM channel (brightness 0..255).
pub trait Lights {
    type Error;

    fn set_brightness(
        &mut self,
        value: u8,
    ) -> Result<(), Self::Error>;
}

#[derive(Debug)]
pub enum ControllerError<PwmErr, ImuErr> {
    Pwm(PwmErr),
    Imu(ImuErr),
}

/// Board runtimes implement this trait so transport layers can target one
/// command interface instead of talking to concrete hardware directly.
pub trait CommandExecutor {
    type Error;

    fn execute_command(
        &mut self,
        cmd: SystemCommand,
    ) -> Result<CommandReply, Self::Error>;

    fn background_poll_interval_ms(&self) -> Option<u64> { None }

    fn background_tick(&mut self) -> Result<(), Self::Error> { Ok(()) }

    /// Stop active motion by commanding zero wheel velocity, without otherwise
    /// changing enable/sleep state.
    ///
    /// The command bus calls this from its motor watchdog when no motion
    /// command has arrived within the failsafe window. It is intentionally
    /// transport-independent: any control source (WebSocket, BLE, ESP-NOW)
    /// that feeds the shared bus inherits the same deadman behaviour. The
    /// default issues a zero-speed translate, which every executor already
    /// maps to "all wheels stopped".
    fn stop_motion(&mut self) -> Result<CommandReply, Self::Error> {
        self.execute_command(SystemCommand::I(I2CCommand::T {
            direction_deg: 0.0,
            speed_norm: 0.0,
        }))
    }
}
