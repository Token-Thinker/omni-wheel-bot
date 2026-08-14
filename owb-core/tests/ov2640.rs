use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
use owb_core::drivers::ov2640::{
    Bank,
    Ov2640,
    RegisterValue,
    BANK_SEL,
    COM7,
    COM7_SRST,
    DEFAULT_ADDR,
    REG_PID,
    REG_VER,
};

#[test]
fn reset_selects_sensor_bank_and_issues_soft_reset() {
    let expectations = [
        I2cTransaction::write(DEFAULT_ADDR, vec![BANK_SEL, Bank::Sensor as u8]),
        I2cTransaction::write(DEFAULT_ADDR, vec![COM7, COM7_SRST]),
    ];
    let i2c = I2cMock::new(&expectations);
    let mut camera = Ov2640::new(i2c, DEFAULT_ADDR);

    camera.reset().expect("reset");

    camera.release().done();
}

#[test]
fn read_id_reads_pid_and_version_from_sensor_bank() {
    let expectations = [
        I2cTransaction::write(DEFAULT_ADDR, vec![BANK_SEL, Bank::Sensor as u8]),
        I2cTransaction::write_read(DEFAULT_ADDR, vec![REG_PID], vec![0x26]),
        I2cTransaction::write_read(DEFAULT_ADDR, vec![REG_VER], vec![0x42]),
    ];
    let i2c = I2cMock::new(&expectations);
    let mut camera = Ov2640::new(i2c, DEFAULT_ADDR);

    let (pid, ver) = camera.read_id().expect("id");

    assert_eq!(pid, 0x26);
    assert_eq!(ver, 0x42);
    camera.release().done();
}

#[test]
fn write_sequence_applies_register_pairs_in_order() {
    let expectations = [
        I2cTransaction::write(DEFAULT_ADDR, vec![BANK_SEL, Bank::Dsp as u8]),
        I2cTransaction::write(DEFAULT_ADDR, vec![0xE0, 0x14]),
        I2cTransaction::write(DEFAULT_ADDR, vec![0xDA, 0x10]),
    ];
    let i2c = I2cMock::new(&expectations);
    let mut camera = Ov2640::new(i2c, DEFAULT_ADDR);
    let sequence = [
        RegisterValue {
            reg: BANK_SEL,
            value: Bank::Dsp as u8,
        },
        RegisterValue {
            reg: 0xE0,
            value: 0x14,
        },
        RegisterValue {
            reg: 0xDA,
            value: 0x10,
        },
    ];

    camera.write_sequence(&sequence).expect("write sequence");

    camera.release().done();
}
