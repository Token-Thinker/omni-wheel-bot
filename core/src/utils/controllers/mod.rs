//! Module Exports
//!
//! This file exports key modules used in the robotics control system.
//!
//! - `i2c_devices`: Handles I2C-connected devices such as motor controllers and
//!   IMUs.
//! - `wheel_kinematics`: Manages wheel kinematics calculations for omni-wheel
//!   robots.

mod driver;
/// Module for managing I2C-connected devices.
pub(crate) mod i2c;
/// Module for handling wheel kinematics calculations.
pub(crate) mod kinematics;
pub(crate) mod leds;

use core::cell::RefCell;

use esp_hal::Blocking;
use esp_hal::rmt::Channel;
// Re-export for easier access
pub use i2c::*;
pub(crate) use kinematics::*;
pub use leds::*;
use serde::{Deserialize, Serialize};
use crate::utils::smart_leds::SmartLedsAdapter;

//I fucking hate types and stupid adapters and shit that i have to get rid of later -.-
const BUF_SIZE: usize = 2 * 24 + 1;
type Fuckleds = SmartLedsAdapter<Channel<Blocking, 0>, { BUF_SIZE }>;

#[derive(Debug, Serialize, Deserialize)]
#[serde(tag = "ct", rename_all = "snake_case")] // ct = command type
pub enum SystemCommand {
    I(I2CCommand),
    L(LEDCommand),
}

pub struct SystemController<I2C: 'static> {
    pub sensors: Option<I2CDevices<'static, I2C>>,
    pub robot_dimensions: (f32, f32), // (wheel_radius, robot_radius)
}
impl<I2C> SystemController<I2C>
where
    I2C: embedded_hal::i2c::I2c + 'static,
{
    //noinspection ALL
    //noinspection ALL
    //noinspection ALL
    //noinspection ALL
    //noinspection ALL
    //noinspection ALL
    pub fn new(
        i2c_bus: &'static RefCell<I2C>,
        wheel_radius: Option<f32>,
        robot_radius: Option<f32>,
    ) -> Self {
        let wr = wheel_radius.unwrap_or(0.148f32);
        let rr = robot_radius.unwrap_or(0.195f32);

        let mut i2c_dev = I2CDevices::new(i2c_bus, wr, rr);

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

    //noinspection ALL
    //noinspection ALL
    //noinspection ALL
    //noinspection ALL
    pub async fn i2c_ch(&mut self) -> ! {
        loop {
            let i2c_channel = I2C_CHANNEL.receiver().receive().await;
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

    //noinspection ALL
    //noinspection ALL
    // todo: implement LED control (generic)
    /* pub async fn led_ch(self, adapter: Fuckleds ) -> !
    {
        let mut led_ctrl = LedModule::new(adapter);

        loop {
            let cmd = LED_CHANNEL.receiver().receive().await;
            tracing::info!("LED Command: {:?}", &cmd);
            if let Err(_) = led_ctrl.ex_command(cmd) {
                tracing::error!("LED Command failed");
            }
        }
    } */
}
