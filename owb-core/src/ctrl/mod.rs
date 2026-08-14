//! Controllers and command definitions for Omni-Wheel Bot hardware.
//!
//! This module defines generic traits for actuators/sensors and the
//! `SystemController` that interprets high-level commands.

#[cfg(feature = "leds")]
pub mod leds;
pub mod sysctrl;
pub mod traits;

#[cfg(feature = "leds")]
pub use leds::LedModule;
pub use sysctrl::{CommandReply, ImuData, SystemCommand, SystemController};
pub use traits::{CommandExecutor, ControllerError, Imu, Lights, MotorPwm};
