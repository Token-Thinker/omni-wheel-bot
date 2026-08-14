//! Minimal LIS2DH12 register driver used for board presence checks and basic
//! bring-up.

use crate::ext::hal;

pub const DEFAULT_ADDR_LOW: u8 = 0x18;
pub const DEFAULT_ADDR_HIGH: u8 = 0x19;
pub const REG_WHO_AM_I: u8 = 0x0F;
pub const WHO_AM_I_RESPONSE: u8 = 0x33;
pub const REG_CTRL1: u8 = 0x20;
pub const REG_CTRL4: u8 = 0x23;
pub const REG_OUT_X_L: u8 = 0x28;
const AUTO_INCREMENT: u8 = 0x80;

#[derive(Debug)]
pub enum Error<E> {
    Bus(E),
}

pub struct Lis2dh12<I2C> {
    bus: I2C,
    address: u8,
}

impl<I2C, E> Lis2dh12<I2C>
where
    I2C: hal::i2c::I2c<Error = E>,
{
    pub const fn new(
        bus: I2C,
        address: u8,
    ) -> Self {
        Self { bus, address }
    }

    pub fn who_am_i(&mut self) -> Result<u8, Error<E>> { self.read_reg(REG_WHO_AM_I) }

    pub fn enable_default(&mut self) -> Result<(), Error<E>> { self.write_reg(REG_CTRL1, 0x57) }

    pub fn configure_runtime(&mut self) -> Result<(), Error<E>> {
        // 100 Hz, all axes enabled, high-resolution, BDU on, +/-2 g range.
        self.write_reg(REG_CTRL1, 0x57)?;
        self.write_reg(REG_CTRL4, 0x88)
    }

    pub fn read_accel_g(&mut self) -> Result<(f32, f32, f32), Error<E>> {
        let mut bytes = [0u8; 6];
        self.bus
            .write_read(self.address, &[REG_OUT_X_L | AUTO_INCREMENT], &mut bytes)
            .map_err(Error::Bus)?;

        let x = decode_axis(bytes[0], bytes[1]);
        let y = decode_axis(bytes[2], bytes[3]);
        let z = decode_axis(bytes[4], bytes[5]);

        // In high-resolution +/-2g mode the LIS2DH12 reports 1 mg/LSB.
        Ok((x as f32 * 0.001, y as f32 * 0.001, z as f32 * 0.001))
    }

    pub fn release(self) -> I2C { self.bus }

    fn write_reg(
        &mut self,
        reg: u8,
        value: u8,
    ) -> Result<(), Error<E>> {
        self.bus
            .write(self.address, &[reg, value])
            .map_err(Error::Bus)
    }

    fn read_reg(
        &mut self,
        reg: u8,
    ) -> Result<u8, Error<E>> {
        let mut value = [0u8; 1];
        self.bus
            .write_read(self.address, &[reg], &mut value)
            .map_err(Error::Bus)?;
        Ok(value[0])
    }
}

fn decode_axis(
    low: u8,
    high: u8,
) -> i16 {
    i16::from_le_bytes([low, high]) >> 4
}
