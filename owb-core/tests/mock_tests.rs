use core::cell::RefCell;

use embedded_hal_bus::i2c::RefCellDevice;
use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTrans};
use owb_core::utils::controllers::i2c::I2CDevices;
use pwm_pca9685::{Address as PwmAddress, Pca9685};
use owb_core::utils::math::kinematics::EmbodiedKinematics;
use owb_core::utils::connection::server::{WebSocket, ServerTimer};


/// Default I2C address for the PWM motor controller.
pub const PWM_ADDRESS: u8 = 0x55;
/// Default I2C address for the IMU sensor.
pub const IMU_ADDRESS: u8 = 0x68;

/// Create a write transaction for the given I2C address and data payload.
pub fn write(
    addr: u8,
    data: Vec<u8>,
) -> I2cTrans {
    I2cTrans::write(addr, data)
}
/// Create a write_read transaction for the given I2C address/payloads.
pub fn write_read(
    addr: u8,
    write: Vec<u8>,
    read: Vec<u8>,
) -> I2cTrans {
    I2cTrans::write_read(addr, write, read)
}
/// Create a read transaction for the given I2C address and expected data.
pub fn read(
    addr: u8,
    data: Vec<u8>,
) -> I2cTrans {
    I2cTrans::read(addr, data)
}
#[test]
fn test_init_devices() {
    // Define only the initialization-related transactions (IMU)
    let expectations = [
        write_read(IMU_ADDRESS, vec![0x75], vec![0x67]),
        write_read(IMU_ADDRESS, vec![0x21], vec![0x00]),
        write(IMU_ADDRESS, vec![0x21, 0x00]),
        write_read(IMU_ADDRESS, vec![0x20], vec![0x00]),
        write(IMU_ADDRESS, vec![0x20, 0x00]),
        write_read(IMU_ADDRESS, vec![0x1F], vec![0x0F]),
        write(IMU_ADDRESS, vec![0x1F, 0x0F]),
    ];

    let mock = I2cMock::new(&expectations);
    let i2c_bus = RefCell::new(mock);
    let mut devs = I2CDevices::new(&i2c_bus, 0.148, 0.195);
    devs.init_devices().unwrap();
    i2c_bus.borrow_mut().done();
}

#[test]
fn test_configure_pwm() {
    // Expected transactions for enabling PWM and setting prescale (includes sleep handling)
    let expectations = [
        write(PWM_ADDRESS, vec![0x00, 0x01]),
        write(PWM_ADDRESS, vec![0x00, 0x11]),
        write(PWM_ADDRESS, vec![0xFE, 100]),
        write(PWM_ADDRESS, vec![0x00, 0x01]),
    ];

    let mock = I2cMock::new(&expectations);
    let i2c_bus = RefCell::new(mock);
    let mut devs = I2CDevices::new(&i2c_bus, 0.148, 0.195);
    let pwm = Pca9685::new(RefCellDevice::new(&i2c_bus), PwmAddress::from(PWM_ADDRESS)).unwrap();
    devs.pwm = Some(pwm);
    devs.configure_pwm().unwrap();
    i2c_bus.borrow_mut().done();
}

#[test]
fn test_apply_wheel_speeds_zero() {
    // Zero speeds should issue one auto-increment and six channel writes
    let expectations = [
        write(PWM_ADDRESS, vec![0x00, 0x31]),
        write(PWM_ADDRESS, vec![0x1E, 0x00, 0x00, 0x00, 0x00]),
        write(PWM_ADDRESS, vec![0x22, 0x00, 0x00, 0x00, 0x00]),
        write(PWM_ADDRESS, vec![0x0E, 0x00, 0x00, 0x00, 0x00]),
        write(PWM_ADDRESS, vec![0x12, 0x00, 0x00, 0x00, 0x00]),
        write(PWM_ADDRESS, vec![0x16, 0x00, 0x00, 0x00, 0x00]),
        write(PWM_ADDRESS, vec![0x1A, 0x00, 0x00, 0x00, 0x00]),
    ];

    let mock = I2cMock::new(&expectations);
    let i2c_bus = RefCell::new(mock);
    let mut devs = I2CDevices::new(&i2c_bus, 0.148, 0.195);
    let pwm = Pca9685::new(RefCellDevice::new(&i2c_bus), PwmAddress::from(PWM_ADDRESS)).unwrap();
    devs.pwm = Some(pwm);
    devs.apply_wheel_speeds(&[0.0, 0.0, 0.0]).unwrap();
    i2c_bus.borrow_mut().done();
}

/// Smoke test for wheel kinematics via the controller helper.
#[test]
fn wheel_velocities_nonzero() {
    let kin = owb_core::utils::math::kinematics::EmbodiedKinematics::new(0.148, 0.195);
    let wheels = kin.compute_wheel_velocities(1.0, 0.0, 0.0, 0.0);
    assert!(wheels.iter().any(|&v| v != 0.0));
}

/// Example: Embodied kinematics instantiation and wheel velocity computation.
#[test]
fn example_embodied_kinematics() {
    let ek = EmbodiedKinematics::new(0.148, 0.195);
    let wheels = ek.compute_wheel_velocities(1.0, 0.0, 0.0, 0.0);
    assert!(wheels.iter().any(|&v| v != 0.0));
}

/// Example: instantiating WebSocket server types.
#[test]
fn example_websocket_types_exist() {
    let _ws: WebSocket = WebSocket;
    let _timer: ServerTimer = ServerTimer;
}