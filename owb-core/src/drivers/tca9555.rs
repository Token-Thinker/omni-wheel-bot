//! Minimal TCA9555 I/O expander driver for output control over I2C.

use crate::ext::hal;

pub const DEFAULT_ADDR: u8 = 0x20;
pub const PORT0: u8 = 0;
pub const PORT1: u8 = 1;

const REG_INPUT_PORT0: u8 = 0x00;
const REG_INPUT_PORT1: u8 = 0x01;
const REG_OUTPUT_PORT0: u8 = 0x02;
const REG_OUTPUT_PORT1: u8 = 0x03;
const REG_CONFIG_PORT0: u8 = 0x06;
const REG_CONFIG_PORT1: u8 = 0x07;

#[derive(Debug)]
pub enum Error<E> {
    Bus(E),
    InvalidPort(u8),
}

pub struct Tca9555<I2C> {
    bus: I2C,
    address: u8,
    output_state: [u8; 2],
}

impl<I2C, E> Tca9555<I2C>
where
    I2C: hal::i2c::I2c<Error = E>,
{
    pub const fn new(
        bus: I2C,
        address: u8,
    ) -> Self {
        Self {
            bus,
            address,
            output_state: [0; 2],
        }
    }

    pub fn configure_outputs(
        &mut self,
        port0_mask: u8,
        port1_mask: u8,
    ) -> Result<(), Error<E>> {
        self.write_reg(REG_CONFIG_PORT0, !port0_mask)?;
        self.write_reg(REG_CONFIG_PORT1, !port1_mask)
    }

    pub fn set_output_ports(
        &mut self,
        port0: u8,
        port1: u8,
    ) -> Result<(), Error<E>> {
        self.output_state = [port0, port1];
        self.write_reg(REG_OUTPUT_PORT0, port0)?;
        self.write_reg(REG_OUTPUT_PORT1, port1)
    }

    pub fn read_inputs(&mut self) -> Result<[u8; 2], Error<E>> {
        Ok([
            self.read_reg(REG_INPUT_PORT0)?,
            self.read_reg(REG_INPUT_PORT1)?,
        ])
    }

    pub fn read_outputs(&mut self) -> Result<[u8; 2], Error<E>> {
        Ok([
            self.read_reg(REG_OUTPUT_PORT0)?,
            self.read_reg(REG_OUTPUT_PORT1)?,
        ])
    }

    pub fn set_output_bits(
        &mut self,
        port: u8,
        mask: u8,
        value: bool,
    ) -> Result<(), Error<E>> {
        let state = self
            .output_state
            .get_mut(port as usize)
            .ok_or(Error::InvalidPort(port))?;

        if value {
            *state |= mask;
        }
        else {
            *state &= !mask;
        }
        let next_state = *state;

        let reg = match port {
            PORT0 => REG_OUTPUT_PORT0,
            PORT1 => REG_OUTPUT_PORT1,
            _ => unreachable!(),
        };

        self.write_reg(reg, next_state)
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
