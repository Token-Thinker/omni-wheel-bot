//! Module Exports
//!
//! This file exports key modules used in the robotics control system.
//!
//! - `i2c_devices`: Handles I2C-connected devices such as motor controllers and IMUs.
//! - `wheel_kinematics`: Manages wheel kinematics calculations for omni-wheel robots.

/// Module for managing I2C-connected devices.
pub(crate) mod i2c_devices;

/// Module for handling wheel kinematics calculations.
pub mod wheel_kinematics;

// Re-exports for easier access.
pub use i2c_devices::*;
pub use wheel_kinematics::*;
