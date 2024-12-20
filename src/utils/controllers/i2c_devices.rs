//! I2C Devices Module
//! This module handles I2C-connected devices, including motor control and IMU integration.
use crate::utils::controllers::WheelKinematics;

use core::cell::RefCell;
use esp_hal::Blocking;
use esp_hal::i2c::master::I2c as espI2c;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embedded_hal::i2c::I2c;
use embedded_hal_bus::i2c::RefCellDevice;
use icm42670::accelerometer::{Accelerometer, Error as AccelerometerError};
use icm42670::{Address as imu_address, Error as ImuError, Icm42670, PowerMode};
use pwm_pca9685::{Address as pwm_address, Channel, Error as PwmError, Pca9685};
use serde::{Deserialize, Serialize};

use micromath::F32Ext;

pub static CHANNEL: embassy_sync::channel::Channel<CriticalSectionRawMutex, I2CCommand, 64> = embassy_sync::channel::Channel::new();

#[derive(Debug)]
pub enum DeviceError<E: core::fmt::Debug> {
    PwmError(PwmError<E>),
    ImuError(ImuError<E>),
    AccelError(AccelerometerError<ImuError<E>>),
}

/// A **single** enum that covers both motion and device commands.
#[derive(Debug, Serialize, Deserialize)]
pub enum I2CCommand {
    // ------------------------------------------------------------------------
    // Motion control variants
    // ------------------------------------------------------------------------
    /// Omnidirectional Translation (no rotation).
    T { d: f32, s: f32 },

    /// Pure rotation in place (yaw).
    Y { s: f32, o: Option<f32> },

    /// Combined translational + rotational command (ideal for dual joysticks).
    O {
        d: f32,
        s: f32,
        rs: f32,
        o: Option<f32>,
    },

    // ------------------------------------------------------------------------
    // Device management variants
    // ------------------------------------------------------------------------
    /// Read IMU data (accelerometer, gyro, temp).
    ReadIMU,

    /// Enable PWM & IMU hardware.
    Enable,

    /// Disable PWM & IMU hardware.
    Disable,
}

pub struct I2CDevices<'a, I2C> {
    pwm: Pca9685<RefCellDevice<'a, I2C>>,
    imu: Icm42670<RefCellDevice<'a, I2C>>,
    motor_channels: [(Channel, Channel); 3],
    kinematics: WheelKinematics,
}

impl<I2C, E> I2CDevices<'_, I2C>
where
    I2C: I2c<Error = E>,
    E: core::fmt::Debug,
{
    /// Creates a new `I2CDevices` instance, initializing IMU and PCA9685 PWM.
    pub fn new(
        i2c: RefCell<espI2c<Blocking>>,
        wheel_radius: f32,
        robot_radius: f32,
    ) -> Result<Self, DeviceError<E>>
    where
        I2C: 'static,
    {
        let imu = Icm42670::new(RefCellDevice::new(&i2c), imu_address::Primary)
            .map_err(DeviceError::ImuError)?;

        let mut pwm = Pca9685::new(RefCellDevice::new(&i2c), pwm_address::default())
            .map_err(DeviceError::PwmError)?;

        pwm.enable().map_err(DeviceError::PwmError)?;
        pwm.set_prescale(3).map_err(DeviceError::PwmError)?;

        let mut on = [0u16; 16];
        let mut off = [0u16; 16];

        // Set channels 0-7 to fully on
        for i in 0..8 {
            on[i] = 0;
            off[i] = 0x0FFF; // 4096 sets the full-on bit
        }
        pwm.set_all_on_off(&on, &off)
            .map_err(DeviceError::PwmError)?;

        let motor_channels = [
            (Channel::C6, Channel::C7), // Motor 0
            (Channel::C2, Channel::C3), // Motor 1
            (Channel::C4, Channel::C5), // Motor 2
        ];

        let kinematics = WheelKinematics::new(wheel_radius, robot_radius);

        Ok(Self {
            pwm,
            imu,
            motor_channels,
            kinematics,
        })
    }

    /// **Single** dispatcher method for all motion & device commands.
    /// - Returns `Ok(Some(...))` if the command is `ReadIMU` (with the IMU data).
    /// - Otherwise returns `Ok(None)`.
    pub fn execute_command(
        &mut self,
        command: I2CCommand,
    ) -> Result<Option<((f32, f32, f32), (f32, f32, f32), f32)>, DeviceError<E>> {
        match command {
            // ----------------------------------------------------------------
            // Motion commands
            // ----------------------------------------------------------------
            I2CCommand::T { d, s } => {
                self.set_motor_velocities_strafe(d, s)?;
                Ok(None)
            }
            I2CCommand::Y {s, o} => {
                self.set_motor_velocities_rotate(s, o)?;
                Ok(None)
            }
            I2CCommand::O {
                d,
                s,
                rs,
                o,
            } => {
                let orientation = o.unwrap_or(0.0);
                let new_orientation = (orientation + rs) % 360.0;

                // Compute wheel velocities for simultaneous strafe + rotate
                let wheel_speeds =
                    self.kinematics
                        .compute_wheel_velocities(s, d, new_orientation, rs);
                self.apply_wheel_speeds(&wheel_speeds)?;
                Ok(None)
            }

            // ----------------------------------------------------------------
            // Device management commands
            // ----------------------------------------------------------------
            I2CCommand::ReadIMU => {
                let data = self.read_imu()?;
                Ok(Some(data)) // Return the IMU sensor data
            }
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

    // ------------------------------------------------------------------------
    // Internal helpers for strafe vs rotate
    // ------------------------------------------------------------------------

    /// Strafe without rotation
    fn set_motor_velocities_strafe(
        &mut self,
        direction: f32,
        speed: f32,
    ) -> Result<(), DeviceError<E>> {
        let wz = 0.0;
        let orientation = 0.0;
        let wheel_speeds =
            self.kinematics
                .compute_wheel_velocities(speed, direction, orientation, wz);

        self.apply_wheel_speeds(&wheel_speeds)?;
        Ok(())
    }

    /// Pure rotation in place
    fn set_motor_velocities_rotate(
        &mut self,
        speed: f32,
        orientation: Option<f32>,
    ) -> Result<(), DeviceError<E>> {
        let wz = speed;
        let orientation = orientation.unwrap_or(0.0);
        let new_orientation = (orientation + wz) % 360.0;

        // No translational speed
        let wheel_speeds =
            self.kinematics
                .compute_wheel_velocities(0.0, 0.0, new_orientation, wz);

        self.apply_wheel_speeds(&wheel_speeds)?;
        Ok(())
    }

    /// Writes the computed wheel speeds to PCA9685 channels
    fn apply_wheel_speeds(&mut self, wheel_speeds: &[f32]) -> Result<(), DeviceError<E>> {
        const MAX_DUTY: u16 = 4095;

        for (i, &(phase_channel, enable_channel)) in self.motor_channels.iter().enumerate() {
            let speed = wheel_speeds[i];
            let duty_cycle = speed.abs().min(1.0) * (MAX_DUTY as f32);
            let direction = speed >= 0.0;

            // Direction Control
            self.pwm
                .set_channel_on_off(phase_channel, 0, if direction { 0 } else { MAX_DUTY })
                .map_err(DeviceError::PwmError)?;

            // Speed Control
            self.pwm
                .set_channel_on_off(enable_channel, 0, duty_cycle as u16)
                .map_err(DeviceError::PwmError)?;
        }

        Ok(())
    }

    // ------------------------------------------------------------------------
    // IMU & PWM/IMU Power Control
    // ------------------------------------------------------------------------

    pub fn read_imu(&mut self) -> Result<((f32, f32, f32), (f32, f32, f32), f32), DeviceError<E>> {
        let accel = self.imu.accel_norm().map_err(DeviceError::AccelError)?;
        let gyro = self.imu.gyro_norm().map_err(DeviceError::ImuError)?;
        let temp = self.imu.temperature().map_err(DeviceError::ImuError)?;
        Ok(((accel.x, accel.y, accel.z), (gyro.x, gyro.y, gyro.z), temp))
    }

    pub fn enable(&mut self) -> Result<(), DeviceError<E>> {
        self.pwm.enable().map_err(DeviceError::PwmError)?;
        self.pwm.set_prescale(3).map_err(DeviceError::PwmError)?;

        let power_mode = PowerMode::SixAxisLowNoise;
        self.imu
            .set_power_mode(power_mode)
            .map_err(DeviceError::ImuError)?;
        Ok(())
    }

    pub fn disable(&mut self) -> Result<(), DeviceError<E>> {
        self.pwm.disable().map_err(DeviceError::PwmError)?;
        self.imu
            .set_power_mode(PowerMode::Sleep)
            .map_err(DeviceError::ImuError)?;
        Ok(())
    }
}
