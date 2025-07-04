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

use crate::utils::controllers::{driver::WheelDriver, WheelKinematics};

// Global communication channel for I2C commands.
pub static I2C_CHANNEL: embassy_sync::channel::Channel<CriticalSectionRawMutex, I2CCommand, 16> =
    embassy_sync::channel::Channel::new();

/// Represents possible device-related errors.
#[derive(Debug)]
pub enum DeviceError<E: core::fmt::Debug> {
    PwmError(PwmError<E>),
    ImuError(ImuError<E>),
    AccelError(AccelerometerError<ImuError<E>>),
    ImuNotInitialized,
    PwmNotInitialized,
}

/// Unified command structure for I2C operations.
#[derive(Debug, Serialize, Deserialize, Clone, Copy)]
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
pub struct I2CDevices<'a, I2C: 'static> {
    #[allow(dead_code)]
    i2c: &'a RefCell<I2C>,
    pwm: Option<Pca9685<RefCellDevice<'a, I2C>>>,
    imu: Option<Icm42670<RefCellDevice<'a, I2C>>>,
    motor_channels: [(Channel, Channel); 3],
    kinematics: WheelKinematics,
}

impl<'a, I2C, E> I2CDevices<'a, I2C>
where
    I2C: I2c<Error = E> + 'static,
    E: core::fmt::Debug,
{
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
            kinematics: WheelKinematics::new(wheel_radius, robot_radius),
        }
    }
    pub fn init_devices(&mut self) -> Result<(), DeviceError<E>> {
        let imu = Icm42670::new(RefCellDevice::new(&self.i2c), ImuAddress::Primary)
            .map_err(DeviceError::ImuError)?;
        let pwm = Pca9685::new(RefCellDevice::new(&self.i2c), PwmAddress::from(0x55))
            .map_err(DeviceError::PwmError)?;

        self.imu = Some(imu);
        self.pwm = Some(pwm);
        Ok(())
    }
    pub fn scan_bus(&self) {
        let mut bus = self.i2c.borrow_mut();
        for addr in 0x03..0x78 {
            if bus.write(addr, &[]).is_ok() {
                tracing::warn!("I2C device found at 0x{:02X}", addr);
            }
        }
    }
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
    pub(crate) fn apply_wheel_speeds(
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

    /// Reads IMU sensor data.
    pub fn read_imu(&mut self) -> Result<((f32, f32, f32), (f32, f32, f32), f32), DeviceError<E>> {
        let imu = self.imu.as_mut().ok_or(DeviceError::ImuNotInitialized)?;
        let accel = imu.accel_norm().map_err(DeviceError::AccelError)?;
        let gyro = imu.gyro_norm().map_err(DeviceError::ImuError)?;
        let temp = imu.temperature().map_err(DeviceError::ImuError)?;

        Ok(((accel.x, accel.y, accel.z), (gyro.x, gyro.y, gyro.z), temp))
    }

    /// Enables both PWM and IMU devices.
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

    /// Disables both PWM and IMU devices.
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

impl<'a, I2C, E> WheelDriver for I2CDevices<'_, I2C>
where
    I2C: embedded_hal::i2c::I2c<Error = E> + 'a,
    E: core::fmt::Debug,
{
    type Error = DeviceError<E>;

    fn read_wheel_speeds(&mut self) -> Result<[f32; 3], Self::Error> {
        // here you might actually read encoders or fuse IMU data.
        // For now, stub it or use your read_imu & a little math:
        let ((_ax, _ay, _az), (gx, gy, gz), _) = self.read_imu()?;
        // convert gyro about z into wheel speeds or whatever…
        Ok([gx, gy, gz])
    }

    fn set_wheel_speeds(
        &mut self,
        speeds: [f32; 3],
    ) -> Result<(), Self::Error> {
        self.apply_wheel_speeds(&speeds)
    }
}
