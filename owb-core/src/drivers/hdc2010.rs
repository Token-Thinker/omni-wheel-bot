//! Minimal HDC2010 driver for board bring-up telemetry.

use crate::ext::hal;

pub const DEFAULT_ADDR_LOW: u8 = 0x40;
pub const DEFAULT_ADDR_HIGH: u8 = 0x41;
pub const REG_TEMP_L: u8 = 0x00;
pub const REG_HUMIDITY_L: u8 = 0x02;

#[derive(Debug)]
pub enum Error<E> {
    Bus(E),
}

pub struct Hdc2010<I2C> {
    bus: I2C,
    address: u8,
}

impl<I2C, E> Hdc2010<I2C>
where
    I2C: hal::i2c::I2c<Error = E>,
{
    pub const fn new(
        bus: I2C,
        address: u8,
    ) -> Self {
        Self { bus, address }
    }

    pub fn read_temperature_c(&mut self) -> Result<f32, Error<E>> {
        let raw = self.read_u16(REG_TEMP_L)?;
        Ok((raw as f32 / 65536.0) * 165.0 - 40.0)
    }

    pub fn read_humidity_percent(&mut self) -> Result<f32, Error<E>> {
        let raw = self.read_u16(REG_HUMIDITY_L)?;
        Ok((raw as f32 / 65536.0) * 100.0)
    }

    pub fn release(self) -> I2C { self.bus }

    fn read_u16(
        &mut self,
        reg: u8,
    ) -> Result<u16, Error<E>> {
        let mut bytes = [0u8; 2];
        self.bus
            .write_read(self.address, &[reg], &mut bytes)
            .map_err(Error::Bus)?;
        Ok(u16::from_le_bytes(bytes))
    }
}
