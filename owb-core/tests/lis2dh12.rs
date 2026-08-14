use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
use owb_core::drivers::lis2dh12::{
    Lis2dh12,
    DEFAULT_ADDR_LOW,
    REG_CTRL1,
    REG_CTRL4,
    REG_OUT_X_L,
    REG_WHO_AM_I,
    WHO_AM_I_RESPONSE,
};

#[test]
fn lis2dh12_reads_who_am_i() {
    let expectations = [I2cTransaction::write_read(
        DEFAULT_ADDR_LOW,
        vec![REG_WHO_AM_I],
        vec![WHO_AM_I_RESPONSE],
    )];
    let i2c = I2cMock::new(&expectations);
    let mut imu = Lis2dh12::new(i2c, DEFAULT_ADDR_LOW);

    let who = imu.who_am_i().expect("who am i");

    assert_eq!(who, WHO_AM_I_RESPONSE);
    imu.release().done();
}

#[test]
fn lis2dh12_enables_default_mode() {
    let expectations = [I2cTransaction::write(
        DEFAULT_ADDR_LOW,
        vec![REG_CTRL1, 0x57],
    )];
    let i2c = I2cMock::new(&expectations);
    let mut imu = Lis2dh12::new(i2c, DEFAULT_ADDR_LOW);

    imu.enable_default().expect("enable");

    imu.release().done();
}

#[test]
fn lis2dh12_configures_runtime_mode() {
    let expectations = [
        I2cTransaction::write(DEFAULT_ADDR_LOW, vec![REG_CTRL1, 0x57]),
        I2cTransaction::write(DEFAULT_ADDR_LOW, vec![REG_CTRL4, 0x88]),
    ];
    let i2c = I2cMock::new(&expectations);
    let mut imu = Lis2dh12::new(i2c, DEFAULT_ADDR_LOW);

    imu.configure_runtime().expect("configure runtime");

    imu.release().done();
}

#[test]
fn lis2dh12_reads_accel_sample() {
    let expectations = [I2cTransaction::write_read(
        DEFAULT_ADDR_LOW,
        vec![REG_OUT_X_L | 0x80],
        vec![0x40, 0x06, 0x20, 0xF9, 0x00, 0x10],
    )];
    let i2c = I2cMock::new(&expectations);
    let mut imu = Lis2dh12::new(i2c, DEFAULT_ADDR_LOW);

    let (x, y, z) = imu.read_accel_g().expect("read accel");

    assert!((x - 0.100).abs() < 0.001);
    assert!((y + 0.110).abs() < 0.001);
    assert!((z - 0.256).abs() < 0.001);
    imu.release().done();
}
