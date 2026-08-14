use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
use owb_core::drivers::max17048::{Max17048, DEFAULT_ADDR, REG_SOC, REG_VCELL, REG_VERSION};

#[test]
fn max17048_reads_voltage_mv() {
    let expectations = [I2cTransaction::write_read(
        DEFAULT_ADDR,
        vec![REG_VCELL],
        vec![0x80, 0x00],
    )];
    let i2c = I2cMock::new(&expectations);
    let mut gauge = Max17048::new(i2c, DEFAULT_ADDR);

    let mv = gauge.read_cell_voltage_mv().expect("vcell");

    assert_eq!(mv, 2560);
    gauge.release().done();
}

#[test]
fn max17048_reads_soc_and_version() {
    let expectations = [
        I2cTransaction::write_read(DEFAULT_ADDR, vec![REG_SOC], vec![0x4B, 0x80]),
        I2cTransaction::write_read(DEFAULT_ADDR, vec![REG_VERSION], vec![0x00, 0x10]),
    ];
    let i2c = I2cMock::new(&expectations);
    let mut gauge = Max17048::new(i2c, DEFAULT_ADDR);

    let soc = gauge.read_soc_percent().expect("soc");
    let version = gauge.read_version().expect("version");

    assert!((soc - 75.5).abs() < 0.01);
    assert_eq!(version, 0x0010);
    gauge.release().done();
}
