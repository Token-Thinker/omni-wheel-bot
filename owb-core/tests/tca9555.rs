use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
use owb_core::drivers::tca9555::{Tca9555, DEFAULT_ADDR, PORT0};

#[test]
fn configure_outputs_sets_port_modes() {
    let expectations = [
        I2cTransaction::write(DEFAULT_ADDR, vec![0x06, 0xF0]),
        I2cTransaction::write(DEFAULT_ADDR, vec![0x07, 0xCC]),
    ];
    let i2c = I2cMock::new(&expectations);
    let mut io = Tca9555::new(i2c, DEFAULT_ADDR);
    io.configure_outputs(0x0F, 0x33).expect("configure outputs");
    io.release().done();
}

#[test]
fn set_output_bits_updates_port_state() {
    let expectations = [
        I2cTransaction::write(DEFAULT_ADDR, vec![0x02, 0x00]),
        I2cTransaction::write(DEFAULT_ADDR, vec![0x03, 0x00]),
        I2cTransaction::write(DEFAULT_ADDR, vec![0x02, 0x05]),
        I2cTransaction::write(DEFAULT_ADDR, vec![0x02, 0x01]),
    ];
    let i2c = I2cMock::new(&expectations);
    let mut io = Tca9555::new(i2c, DEFAULT_ADDR);
    io.set_output_ports(0x00, 0x00).expect("clear outputs");
    io.set_output_bits(PORT0, 0x05, true).expect("set bits");
    io.set_output_bits(PORT0, 0x04, false).expect("clear bit");
    io.release().done();
}

#[test]
fn read_inputs_reads_both_ports() {
    let expectations = [
        I2cTransaction::write_read(DEFAULT_ADDR, vec![0x00], vec![0xA5]),
        I2cTransaction::write_read(DEFAULT_ADDR, vec![0x01], vec![0x5A]),
    ];
    let i2c = I2cMock::new(&expectations);
    let mut io = Tca9555::new(i2c, DEFAULT_ADDR);

    let inputs = io.read_inputs().expect("read inputs");

    assert_eq!(inputs, [0xA5, 0x5A]);
    io.release().done();
}
