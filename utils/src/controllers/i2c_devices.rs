//! I2C Devices Module
//! This module handles I2C-connected devices, including motor control and IMU integration.

use crate::controllers::WheelKinematics;
use core::cell::RefCell;
use embedded_hal::i2c::I2c;
use embedded_hal_bus::i2c::RefCellDevice;
use icm42670::accelerometer::{Accelerometer, Error as AccelerometerError};
use icm42670::{Address as imu_address, Error as ImuError, Icm42670, PowerMode};
use pwm_pca9685::{Address as pwm_address, Channel, Error as PwmError, Pca9685};

#[derive(Debug)]
pub enum DeviceError<E: core::fmt::Debug> {
    PwmError(PwmError<E>),
    ImuError(ImuError<E>),
    AccelError(AccelerometerError<ImuError<E>>),
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
    pub fn new(
        i2c: &'static RefCell<I2C>,
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

    pub fn set_motor_velocities(
        &mut self,
        direction: f32,
        speed: f32,
        rotation_enabled: bool,
        orientation: Option<f32>,
    ) -> Result<(), DeviceError<E>> {
        let wz = if rotation_enabled { speed } else { 0.0 };
        let orientation = orientation.unwrap_or(0.0);

        let new_orientation = (orientation + wz) % 360.0;

        let wheel_speeds =
            self.kinematics
                .compute_wheel_velocities(speed, direction, new_orientation, wz);

        for (i, &(ph, en)) in self.motor_channels.iter().enumerate() {
            let speed = wheel_speeds[i];
            let duty_cycle = speed.abs().min(1.0) * 4095.0;
            let direction = speed >= 0.0;

            const MAX_DUTY: u16 = 4095;

            // **Direction Control:**
            self.pwm
                .set_channel_on_off(ph, 0, if direction { 0 } else { MAX_DUTY })
                .map_err(DeviceError::PwmError)?;

            // **Speed Control:**
            self.pwm
                .set_channel_on_off(en, 0, duty_cycle as u16)
                .map_err(DeviceError::PwmError)?;
        }
        Ok(())
    }

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
