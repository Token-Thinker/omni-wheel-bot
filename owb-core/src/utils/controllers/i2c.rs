//! I2C device management for the Omni-Wheel Bot.
//!
//! This module provides abstractions for initializing and controlling motor PWM drivers
//! and the IMU sensor over a shared I2C bus. Commands are received via `I2C_CHANNEL`.

use crate::utils;
use core::cell::RefCell;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embedded_hal::i2c::I2c;
use embedded_hal_bus::i2c::RefCellDevice;
use icm42670::{
    accelerometer::{Accelerometer, Error as AccelerometerError},
    Address as ImuAddress, Error as ImuError, Icm42670, PowerMode,
};
use pwm_pca9685::{Address as PwmAddress, Channel, Error as PwmError, Pca9685};
use serde::{Deserialize, Serialize};

/// Channel used to receive I2C commands (`I2CCommand` messages).
pub static I2C_CHANNEL: embassy_sync::channel::Channel<CriticalSectionRawMutex, I2CCommand, 16> =
    embassy_sync::channel::Channel::new();

/// Errors that can occur when interacting with I2C-based devices.
#[derive(Debug)]
pub enum DeviceError<E: core::fmt::Debug> {
    PwmError(PwmError<E>),
    ImuError(ImuError<E>),
    AccelError(AccelerometerError<ImuError<E>>),
    ImuNotInitialized,
    PwmNotInitialized,
}

/// I2C command variants for motion control and device management.
///
/// Serialized as JSON with tag `"ic"`.
#[derive(Debug, Serialize, Deserialize, Clone, Copy)]
#[serde(tag = "ic", rename_all = "snake_case")]
pub enum I2CCommand {
    // Motion Control Variants
    /// Omnidirectional translation (no rotation).
    T { d: f32, s: f32 },
    /// Pure rotation in place (yaw).
    Y { s: f32, o: Option<f32> },
    /// Combined translational and rotational command.
    O {
        d: f32,
        s: f32,
        rs: f32,
        o: Option<f32>,
    },

    // Device Management Variants
    /// Read IMU sensor data (accelerometer, gyro, temperature).
    ReadIMU,
    /// Enable I2C-connected devices.
    Enable,
    /// Disable I2C-connected devices.
    Disable,
}

/// High-level driver for PWM motor controller and IMU over a shared I2C bus.
pub struct I2CDevices<'a, I2C: 'static> {
    #[allow(dead_code)]
    i2c: &'a RefCell<I2C>,
    pub pwm: Option<Pca9685<RefCellDevice<'a, I2C>>>,
    imu: Option<Icm42670<RefCellDevice<'a, I2C>>>,
    motor_channels: [(Channel, Channel); 3],
    embodied: utils::ek,
}

impl<'a, I2C, E> I2CDevices<'a, I2C>
where
    I2C: I2c<Error = E> + 'static,
    E: core::fmt::Debug,
{
    /// Create a new I2CDevices manager with specified wheel and robot dimensions.
    pub fn new(
        i2c_bus: &'a RefCell<I2C>,
        wheel_radius: f32,
        robot_radius: f32,
    ) -> Self {
        I2CDevices {
            i2c: i2c_bus,
            pwm: None,
            imu: None,
            motor_channels: [
                (Channel::C6, Channel::C7),
                (Channel::C2, Channel::C3),
                (Channel::C4, Channel::C5),
            ],
            embodied: utils::ek::new(wheel_radius, robot_radius),
        }
    }
    /// Initialize the IMU and PWM motor controller on the I2C bus.
    ///
    /// On success, both `self.imu` and `self.pwm` are set. Returns error if initialization fails.
    pub fn init_devices(&mut self) -> Result<(), DeviceError<E>> {
        let imu = Icm42670::new(RefCellDevice::new(&self.i2c), ImuAddress::Primary)
            .map_err(DeviceError::ImuError)?;
        let pwm = Pca9685::new(RefCellDevice::new(&self.i2c), PwmAddress::from(0x55))
            .map_err(DeviceError::PwmError)?;

        self.imu = Some(imu);
        self.pwm = Some(pwm);
        Ok(())
    }
    /// Scan the I2C bus for devices and log any found addresses.
    pub fn scan_bus(&self) {
        let mut bus = self.i2c.borrow_mut();
        for addr in 0x03..0x78 {
            if bus.write(addr, &[]).is_ok() {
                tracing::warn!("I2C device found at 0x{:02X}", addr);
            }
        }
    }
    /// Configure and enable the PWM motor driver (prescale to 60Hz).
    pub fn configure_pwm(&mut self) -> Result<(), DeviceError<E>> {
        if let Some(pca) = &mut self.pwm {
            pca.enable().map_err(DeviceError::PwmError)?;
            tracing::info!("PWM enabled");
            pca.set_prescale(100).map_err(DeviceError::PwmError)?;
            tracing::info!("PWM prescale set to 60Hz");
        } else {
            tracing::error!("PWM not initialized");
        }

        Ok(())
    }
    /// Perform an initial IMU data read and log accelerometer, gyro, and temperature.
    pub fn init_imu_data(&mut self) {
        match self.read_imu() {
            Ok((accel, gyro, temp)) => {
                tracing::info!("Initial IMU read successful:");
                tracing::info!("Accelerometer: {:?}", accel);
                tracing::info!("Gyroscope: {:?}", gyro);
                tracing::info!("Temperature: {:?}", temp);
            }
            Err(e) => {
                tracing::error!("Failed to read IMU data: {:?}", e);
            }
        }
    }

    /// Execute a high-level `I2CCommand`, performing motion or sensor operations.
    ///
    /// Returns sensor data for `ReadIMU` or `None` for other commands.
    pub fn execute_command(
        &mut self,
        command: I2CCommand,
    ) -> Result<Option<((f32, f32, f32), (f32, f32, f32), f32)>, DeviceError<E>> {
        match command {
            I2CCommand::T { d, s } => {
                self.set_motor_velocities_strafe(d, s)?;
                Ok(None)
            }
            I2CCommand::Y { s, o } => {
                self.set_motor_velocities_rotate(s, o)?;
                Ok(None)
            }
            I2CCommand::O { d, s, rs, o } => {
                let orientation = o.unwrap_or(0.0);
                let new_orientation = (orientation + rs) % 360.0;
                let wheel_speeds =
                    self.embodied
                        .compute_wheel_velocities(s, d, new_orientation, rs);
                self.apply_wheel_speeds(&wheel_speeds)?;
                Ok(None)
            }
            I2CCommand::ReadIMU => Ok(Some(self.read_imu()?)),
            I2CCommand::Enable => {
                self.enable()?;
                Ok(None)
            }
            I2CCommand::Disable => {
                self.disable()?;
                Ok(None)
            }
        }
    }

    /// Computes and applies motor speeds for strafing.
    fn set_motor_velocities_strafe(
        &mut self,
        direction: f32,
        speed: f32,
    ) -> Result<(), DeviceError<E>> {
        let wheel_speeds = self
            .embodied
            .compute_wheel_velocities(speed, direction, 0.0, 0.0);
        self.apply_wheel_speeds(&wheel_speeds)
    }

    /// Computes and applies motor speeds for rotation.
    fn set_motor_velocities_rotate(
        &mut self,
        speed: f32,
        orientation: Option<f32>,
    ) -> Result<(), DeviceError<E>> {
        let new_orientation = (orientation.unwrap_or(0.0) + speed) % 360.0;
        let wheel_speeds = self
            .embodied
            .compute_wheel_velocities(0.0, 0.0, new_orientation, speed);
        self.apply_wheel_speeds(&wheel_speeds)
    }

    /// Applies calculated motor speeds using the PWM driver.
    pub fn apply_wheel_speeds(
        &mut self,
        wheel_speeds: &[f32],
    ) -> Result<(), DeviceError<E>> {
        const MAX_DUTY: u16 = 4095;

        for (i, &(phase_channel, enable_channel)) in self.motor_channels.iter().enumerate() {
            let speed = wheel_speeds[i].abs().min(1.0);
            let direction = wheel_speeds[i] >= 0.0;

            if let Some(pca) = &mut self.pwm {
                pca.set_channel_on_off(phase_channel, 0, if direction { 0 } else { MAX_DUTY })
                    .map_err(DeviceError::PwmError)?;
                pca.set_channel_on_off(enable_channel, 0, (speed * MAX_DUTY as f32) as u16)
                    .map_err(DeviceError::PwmError)?;
            } else {
                tracing::error!("PWM not initialized");
            }
        }
        Ok(())
    }
    #[allow(dead_code)]
    fn apply_wheels_bulk(
        &mut self,
        _wheels: &[f32],
    ) -> Result<(), DeviceError<E>> {
        todo!("Need to implement function for bulk all on and off for simulations changes")
    }

    /// Read accelerometer, gyroscope, and temperature data from the IMU.
    ///
    /// # Returns
    ///
    /// `Ok(((ax, ay, az), (gx, gy, gz), temp))` on success.
    pub fn read_imu(&mut self) -> Result<((f32, f32, f32), (f32, f32, f32), f32), DeviceError<E>> {
        let imu = self.imu.as_mut().ok_or(DeviceError::ImuNotInitialized)?;
        let accel = imu.accel_norm().map_err(DeviceError::AccelError)?;
        let gyro = imu.gyro_norm().map_err(DeviceError::ImuError)?;
        let temp = imu.temperature().map_err(DeviceError::ImuError)?;

        Ok(((accel.x, accel.y, accel.z), (gyro.x, gyro.y, gyro.z), temp))
    }

    /// Enable the PWM motor controller and power up the IMU sensor.
    pub fn enable(&mut self) -> Result<(), DeviceError<E>> {
        if let Some(pca) = self.pwm.as_mut() {
            pca.enable().map_err(DeviceError::PwmError)?;
        }

        if let Some(imu) = self.imu.as_mut() {
            imu.set_power_mode(PowerMode::SixAxisLowNoise)
                .map_err(DeviceError::ImuError)?;
        }

        Ok(())
    }

    /// Disable motor PWM and put the IMU into sleep mode.
    pub fn disable(&mut self) -> Result<(), DeviceError<E>> {
        if let Some(pca) = self.pwm.as_mut() {
            pca.disable().map_err(DeviceError::PwmError)?;
        }

        if let Some(imu) = self.imu.as_mut() {
            imu.set_power_mode(PowerMode::Sleep)
                .map_err(DeviceError::ImuError)?;
        }

        Ok(())
    }
}
