//! Minimal HDC2010 driver for board bring-up telemetry.

use crate::ext::hal;

pub const DEFAULT_ADDR_LOW: u8 = 0x40;
pub const DEFAULT_ADDR_HIGH: u8 = 0x41;
pub const REG_TEMP_L: u8 = 0x00;
pub const REG_HUMIDITY_L: u8 = 0x02;
/// Reset / DRDY-interrupt config. Bits 6:4 select auto-measurement mode (AMM).
pub const REG_RESET_DRDY_INT_CONF: u8 = 0x0E;
/// Measurement config: resolution, which signals to measure, and MEAS_TRIG.
pub const REG_MEASUREMENT_CONF: u8 = 0x0F;
/// AMM = 0b101 in bits 6:4 → one automatic conversion per second.
pub const AMM_1HZ: u8 = 0b0101_0000;
/// 14-bit temperature + humidity, MEAS_TRIG set to start converting.
pub const MEAS_CONF_14BIT_BOTH_TRIGGER: u8 = 0b0000_0001;

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

    /// Start automatic 1 Hz conversions.
    ///
    /// Required before any read: the HDC2010 powers up idle and performs no
    /// conversion on its own, so `REG_TEMP_L` / `REG_HUMIDITY_L` stay at
    /// `0x0000` and decode to exactly -40.0 C and 0.0 % — which is what the
    /// board probe reported on hardware until this was added. Auto-measurement
    /// mode is used rather than a one-shot trigger so that reads need no delay
    /// dependency and this stays usable from the blocking sensor path.
    pub fn init(&mut self) -> Result<(), Error<E>> {
        self.write_reg(REG_RESET_DRDY_INT_CONF, AMM_1HZ)?;
        self.write_reg(REG_MEASUREMENT_CONF, MEAS_CONF_14BIT_BOTH_TRIGGER)
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

    fn write_reg(
        &mut self,
        reg: u8,
        value: u8,
    ) -> Result<(), Error<E>> {
        self.bus
            .write(self.address, &[reg, value])
            .map_err(Error::Bus)
    }

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
