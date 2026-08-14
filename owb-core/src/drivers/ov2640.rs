//! Minimal OV2640 SCCB/I2C register driver.
//!
//! This keeps the Rust-side interface small and testable. The temporary
//! archived OV2640 register headers can be translated into Rust tables later
//! without changing the low-level bus API here.

use crate::ext::hal;

pub const DEFAULT_ADDR: u8 = 0x30;
pub const BANK_SEL: u8 = 0xFF;
pub const REG_PID: u8 = 0x0A;
pub const REG_VER: u8 = 0x0B;
pub const COM7: u8 = 0x12;
pub const COM7_SRST: u8 = 0x80;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Bank {
    Dsp    = 0x00,
    Sensor = 0x01,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RegisterValue {
    pub reg: u8,
    pub value: u8,
}

#[derive(Debug)]
pub enum Error<E> {
    Bus(E),
}

pub struct Ov2640<I2C> {
    bus: I2C,
    address: u8,
}

impl<I2C, E> Ov2640<I2C>
where
    I2C: hal::i2c::I2c<Error = E>,
{
    pub const fn new(
        bus: I2C,
        address: u8,
    ) -> Self {
        Self { bus, address }
    }

    pub fn select_bank(
        &mut self,
        bank: Bank,
    ) -> Result<(), Error<E>> {
        self.write_reg(BANK_SEL, bank as u8)
    }

    pub fn reset(&mut self) -> Result<(), Error<E>> {
        self.select_bank(Bank::Sensor)?;
        self.write_reg(COM7, COM7_SRST)
    }

    pub fn read_id(&mut self) -> Result<(u8, u8), Error<E>> {
        self.select_bank(Bank::Sensor)?;
        let pid = self.read_reg(REG_PID)?;
        let ver = self.read_reg(REG_VER)?;
        Ok((pid, ver))
    }

    pub fn write_sequence(
        &mut self,
        sequence: &[RegisterValue],
    ) -> Result<(), Error<E>> {
        for pair in sequence {
            self.write_reg(pair.reg, pair.value)?;
        }
        Ok(())
    }

    pub fn release(self) -> I2C { self.bus }

    pub fn write_reg(
        &mut self,
        reg: u8,
        value: u8,
    ) -> Result<(), Error<E>> {
        self.bus
            .write(self.address, &[reg, value])
            .map_err(Error::Bus)
    }

    pub fn read_reg(
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
