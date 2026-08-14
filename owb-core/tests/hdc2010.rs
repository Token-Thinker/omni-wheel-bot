use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
use owb_core::drivers::hdc2010::{Hdc2010, DEFAULT_ADDR_LOW, REG_HUMIDITY_L, REG_TEMP_L};

#[test]
fn hdc2010_reads_temperature() {
    let expectations = [I2cTransaction::write_read(
        DEFAULT_ADDR_LOW,
        vec![REG_TEMP_L],
        vec![0x00, 0x80],
    )];
    let i2c = I2cMock::new(&expectations);
    let mut sensor = Hdc2010::new(i2c, DEFAULT_ADDR_LOW);

    let temp = sensor.read_temperature_c().expect("temp");

    assert!((temp - 42.5).abs() < 0.1);
    sensor.release().done();
}

#[test]
fn hdc2010_reads_humidity() {
    let expectations = [I2cTransaction::write_read(
        DEFAULT_ADDR_LOW,
        vec![REG_HUMIDITY_L],
        vec![0x00, 0x80],
    )];
    let i2c = I2cMock::new(&expectations);
    let mut sensor = Hdc2010::new(i2c, DEFAULT_ADDR_LOW);

    let humidity = sensor.read_humidity_percent().expect("humidity");

    assert!((humidity - 50.0).abs() < 0.1);
    sensor.release().done();
}
