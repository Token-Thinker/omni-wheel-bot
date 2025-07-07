//! Module Exports
//!
//! This file exports key modules used in the robotics control system.
//!
//! - `i2c_devices`: Handles I2C-connected devices such as motor controllers and
//!   IMUs.
//! - `wheel_kinematics`: Manages wheel kinematics calculations for omni-wheel
//!   robots.

/// Module for managing I2C-connected devices.
pub mod i2c;
pub mod leds;

use core::cell::RefCell;
use serde::{Deserialize, Serialize};

pub use i2c::I2C_CHANNEL;
pub use leds::LED_CHANNEL;

#[derive(Debug, Serialize, Deserialize)]
#[serde(tag = "ct", rename_all = "snake_case")] // ct = command type
pub enum SystemCommand {
    I(i2c::I2CCommand),
    L(leds::LEDCommand),
}

pub struct SystemController<I2C: 'static> {
    pub sensors: Option<i2c::I2CDevices<'static, I2C>>,
    pub robot_dimensions: (f32, f32), // (wheel_radius, robot_radius)
}
impl<I2C> SystemController<I2C>
where
    I2C: embedded_hal::i2c::I2c + 'static,
{
    pub fn new(
        i2c_bus: &'static RefCell<I2C>,
        wheel_radius: Option<f32>,
        robot_radius: Option<f32>,
    ) -> Self {
        let wr = wheel_radius.unwrap_or(0.148f32);
        let rr = robot_radius.unwrap_or(0.195f32);

        let mut i2c_dev = i2c::I2CDevices::new(i2c_bus, wr, rr);

        let sensors = match i2c_dev.init_devices() {
            Ok(()) => {
                let _ = i2c_dev.configure_pwm();
                i2c_dev.init_imu_data();
                Some(i2c_dev)
            }
            Err(e) => {
                tracing::warn!("I2C init failed, scanning instead: {:?}", e);
                i2c_dev.scan_bus();
                None
            }
        };

        SystemController {
            sensors,
            robot_dimensions: (wr, rr),
        }
    }

    pub async fn i2c_ch(&mut self) -> ! {
        loop {
            let i2c_channel = i2c::I2C_CHANNEL.receiver().receive().await;
            tracing::info!("Received I2C Command: {:?}", i2c_channel);
            if let Some(devs) = self.sensors.as_mut() {
                match devs.execute_command(i2c_channel) {
                    Ok(Some((accel, gyro, temp))) => {
                        tracing::info!(?accel, ?gyro, ?temp, "IMU Data Read");
                    }
                    Ok(None) => tracing::info!("I2C command executed successfully"),
                    Err(_) => tracing::error!("I2C command failed"),
                }
            } else {
                tracing::warn!(
                    "I2C command received but devices not initialized: {:?}",
                    i2c_channel
                );
            }
        }
    }
}
