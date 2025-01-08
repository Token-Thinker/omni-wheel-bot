//! Module Exports
//!
//! This file exports key modules used in the robotics control system.
//!
//! - `i2c_devices`: Handles I2C-connected devices such as motor controllers and
//!   IMUs.
//! - `wheel_kinematics`: Manages wheel kinematics calculations for omni-wheel
//!   robots.

/// Module for managing I2C-connected devices.
pub(crate) mod i2c;

pub(crate) mod leds;
/// Module for handling wheel kinematics calculations.
pub mod kinematics;

// Re-export for easier access
pub use i2c::*;
pub use leds::*;
use serde::{Deserialize, Serialize};
pub use kinematics::*;

#[derive(Debug, Serialize, Deserialize)]
#[serde(tag = "ct", rename_all = "snake_case")] // ct = command type
pub enum SystemCommand
{
    I(I2CCommand),

    L(LEDCommand),
}
