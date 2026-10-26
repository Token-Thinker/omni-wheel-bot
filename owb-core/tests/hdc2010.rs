use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
use owb_core::drivers::hdc2010::{
    Hdc2010,
    AMM_1HZ,
    DEFAULT_ADDR_LOW,
    MEAS_CONF_14BIT_BOTH_TRIGGER,
    REG_HUMIDITY_L,
    REG_MEASUREMENT_CONF,
    REG_RESET_DRDY_INT_CONF,
    REG_TEMP_L,
};

/// Regression guard: without `init` the device never converts, so both data
/// registers stay at 0x0000 and decode to exactly -40.0 C / 0.0 % — the values
/// the board probe reported on real hardware.
#[test]
fn hdc2010_init_enables_auto_measurement() {
    let expectations = [
        I2cTransaction::write(DEFAULT_ADDR_LOW, vec![REG_RESET_DRDY_INT_CONF, AMM_1HZ]),
        I2cTransaction::write(
            DEFAULT_ADDR_LOW,
            vec![REG_MEASUREMENT_CONF, MEAS_CONF_14BIT_BOTH_TRIGGER],
        ),
    ];
    let i2c = I2cMock::new(&expectations);
    let mut sensor = Hdc2010::new(i2c, DEFAULT_ADDR_LOW);

    sensor.init().expect("init");

    sensor.release().done();
}

#[test]
fn hdc2010_uninitialised_registers_decode_to_sentinel() {
    let expectations = [
        I2cTransaction::write_read(DEFAULT_ADDR_LOW, vec![REG_TEMP_L], vec![0x00, 0x00]),
        I2cTransaction::write_read(DEFAULT_ADDR_LOW, vec![REG_HUMIDITY_L], vec![0x00, 0x00]),
    ];
    let i2c = I2cMock::new(&expectations);
    let mut sensor = Hdc2010::new(i2c, DEFAULT_ADDR_LOW);

    // Documents why -40.0/0.0 means "never initialised" rather than "cold".
    assert!((sensor.read_temperature_c().expect("temp") - (-40.0)).abs() < 0.01);
    assert!(sensor.read_humidity_percent().expect("humidity").abs() < 0.01);
    sensor.release().done();
}

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
