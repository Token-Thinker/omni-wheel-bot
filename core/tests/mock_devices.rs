// pub use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as
// I2cTrans};
//
// pub const PWM_ADDRESS: u8 = 0x40;
// pub const IMU_ADDRESS: u8 = 0x68;
//
// pub static I2C_CELL: static_cell::StaticCell<core::cell::RefCell<I2cMock>> =
// static_cell::StaticCell::new();
//
// pub fn write(
// addr: u8,
// data: Vec<u8>,
// ) -> I2cTrans {
// I2cTrans::write(addr, data)
// }
//
// pub fn write_read(
// addr: u8,
// write_data: Vec<u8>,
// read_data: Vec<u8>,
// ) -> I2cTrans {
// I2cTrans::write_read(addr, write_data, read_data)
// }
