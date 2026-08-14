//! Minimal MAX17048 fuel-gauge driver for voltage and state-of-charge reads.

use crate::ext::hal;

pub const DEFAULT_ADDR: u8 = 0x36;
pub const REG_VCELL: u8 = 0x02;
pub const REG_SOC: u8 = 0x04;
pub const REG_VERSION: u8 = 0x08;

#[derive(Debug)]
pub enum Error<E> {
    Bus(E),
}

pub struct Max17048<I2C> {
    bus: I2C,
    address: u8,
}

impl<I2C, E> Max17048<I2C>
where
    I2C: hal::i2c::I2c<Error = E>,
{
    pub const fn new(
        bus: I2C,
        address: u8,
    ) -> Self {
        Self { bus, address }
    }

    pub fn read_cell_voltage_mv(&mut self) -> Result<u16, Error<E>> {
        let raw = self.read_u16(REG_VCELL)?;
        Ok(((raw as u32) * 5 / 64) as u16)
    }

    pub fn read_soc_percent(&mut self) -> Result<f32, Error<E>> {
        let raw = self.read_u16(REG_SOC)?;
        Ok(raw as f32 / 256.0)
    }

    pub fn read_version(&mut self) -> Result<u16, Error<E>> { self.read_u16(REG_VERSION) }

    pub fn release(self) -> I2C { self.bus }

    fn read_u16(
        &mut self,
        reg: u8,
    ) -> Result<u16, Error<E>> {
        let mut bytes = [0u8; 2];
        self.bus
            .write_read(self.address, &[reg], &mut bytes)
            .map_err(Error::Bus)?;
        Ok(u16::from_be_bytes(bytes))
    }
}
