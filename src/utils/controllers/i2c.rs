//! I2C Devices Module
//! This module manages I2C-connected devices, including motor control and IMU
//! integration.

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

use crate::utils::controllers::WheelKinematics;

// Global communication channel for I2C commands.
pub static I2C_CHANNEL: embassy_sync::channel::Channel<CriticalSectionRawMutex, I2CCommand, 16> =
    embassy_sync::channel::Channel::new();

/// Represents possible device-related errors.
#[derive(Debug)]
pub enum DeviceError<E: core::fmt::Debug> {
    PwmError(PwmError<E>),
    ImuError(ImuError<E>),
    AccelError(AccelerometerError<ImuError<E>>),
}

/// Unified command structure for I2C operations.
#[derive(Debug, Serialize, Deserialize)]
#[serde(tag = "ic", rename_all = "snake_case")] // ic = 12c command
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

/// Manages I2C devices including PWM motor controller and IMU.
pub struct I2CDevices<'a, I2C>
{
    #[allow(dead_code)]
    i2c: &'a RefCell<I2C>,
    pwm: Pca9685<RefCellDevice<'a, I2C>>,
    imu: Icm42670<RefCellDevice<'a, I2C>>,
    motor_channels: [(Channel, Channel); 3],
    kinematics: WheelKinematics,
}

impl<'a, I2C, E> I2CDevices<'a, I2C>
where
    I2C: I2c<Error = E> + 'a,
    E: core::fmt::Debug,
{
    /// Initializes I2C devices, setting up PWM and IMU.
    pub fn new(
        i2c: &'a RefCell<I2C>,
        wheel_radius: f32,
        robot_radius: f32,
    ) -> Result<Self, DeviceError<E>> {
        let imu = Icm42670::new(RefCellDevice::new(i2c), ImuAddress::Primary)
            .map_err(DeviceError::ImuError)?;
        let mut pwm = Pca9685::new(RefCellDevice::new(i2c), PwmAddress::default())
            .map_err(DeviceError::PwmError)?;

        pwm.enable().map_err(DeviceError::PwmError)?;
        pwm.set_prescale(3).map_err(DeviceError::PwmError)?;
        pwm.set_all_on_off(&[0; 16], &[0x0FFF; 16])
            .map_err(DeviceError::PwmError)?;

        Ok(Self {
            i2c,
            pwm,
            imu,
            motor_channels: [
                (Channel::C6, Channel::C7),
                (Channel::C2, Channel::C3),
                (Channel::C4, Channel::C5),
            ],
            kinematics: WheelKinematics::new(wheel_radius, robot_radius),
        })
    }

    /// Processes and executes I2C commands.
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
                    self.kinematics
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
            .kinematics
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
        let wheel_speeds =
            self.kinematics
                .compute_wheel_velocities(0.0, 0.0, new_orientation, speed);
        self.apply_wheel_speeds(&wheel_speeds)
    }

    /// Applies calculated motor speeds using the PWM driver.
    fn apply_wheel_speeds(
        &mut self,
        wheel_speeds: &[f32],
    ) -> Result<(), DeviceError<E>> {
        const MAX_DUTY: u16 = 4095;

        for (i, &(phase_channel, enable_channel)) in self.motor_channels.iter().enumerate() {
            let speed = wheel_speeds[i].abs().min(1.0);
            let direction = wheel_speeds[i] >= 0.0;

            self.pwm
                .set_channel_on_off(phase_channel, 0, if direction { 0 } else { MAX_DUTY })
                .map_err(DeviceError::PwmError)?;
            self.pwm
                .set_channel_on_off(enable_channel, 0, (speed * MAX_DUTY as f32) as u16)
                .map_err(DeviceError::PwmError)?;
        }
        Ok(())
    }

    /// Reads IMU sensor data.
    pub fn read_imu(&mut self) -> Result<((f32, f32, f32), (f32, f32, f32), f32), DeviceError<E>> {
        let accel = self.imu.accel_norm().map_err(DeviceError::AccelError)?;
        let gyro = self.imu.gyro_norm().map_err(DeviceError::ImuError)?;
        let temp = self.imu.temperature().map_err(DeviceError::ImuError)?;
        Ok(((accel.x, accel.y, accel.z), (gyro.x, gyro.y, gyro.z), temp))
    }

    /// Enables both PWM and IMU devices.
    pub fn enable(&mut self) -> Result<(), DeviceError<E>> {
        self.pwm.enable().map_err(DeviceError::PwmError)?;
        self.imu
            .set_power_mode(PowerMode::SixAxisLowNoise)
            .map_err(DeviceError::ImuError)
    }

    /// Disables both PWM and IMU devices.
    pub fn disable(&mut self) -> Result<(), DeviceError<E>> {
        self.pwm.disable().map_err(DeviceError::PwmError)?;
        self.imu
            .set_power_mode(PowerMode::Sleep)
            .map_err(DeviceError::ImuError)?;
        Ok(())
    }
}
