#[cfg(feature = "real-executors")]
use esp_hal::{
    gpio::AnyPin,
    i2c::master::{Config as I2cConfig, I2c},
    Blocking,
};
#[cfg(feature = "real-executors")]
use log::{info, warn};
#[cfg(feature = "real-executors")]
use owb_core::{
    ctrl::{
        sysctrl::{EnvData, PowerData},
        ImuData,
    },
    drivers::{
        hdc2010::{Hdc2010, DEFAULT_ADDR_HIGH as HDC_ADDR_HIGH, DEFAULT_ADDR_LOW as HDC_ADDR_LOW},
        lis2dh12::{
            Lis2dh12,
            DEFAULT_ADDR_HIGH as LIS_ADDR_HIGH,
            DEFAULT_ADDR_LOW as LIS_ADDR_LOW,
            WHO_AM_I_RESPONSE,
        },
        max17048::{Max17048, DEFAULT_ADDR as MAX17048_ADDR},
        tca9555::Tca9555,
    },
    platform::BoardRuntimeKind,
};

#[cfg(feature = "real-executors")]
use super::BoardCapabilities;
#[cfg(feature = "real-executors")]
use crate::profile::{BoardProfile, I2cPins};

#[cfg(feature = "real-executors")]
/// TCA9555 port1 bits driven as indicator LEDs (power, wifi, user).
///
/// `write_state` claims the whole expander on every update: it configures port0
/// entirely as outputs and drives it to `0`, and masks port1 to these bits. That
/// is fine while the LEDs are the only thing on the chip, but it will fight any
/// future co-tenant. `owb_core::drivers::tof::TCA9555_INDICATOR_PORT1_MASK`
/// mirrors this value and must be kept in step -- the planned ToF LPn lines
/// currently overlap it (see that module's tests).
const INDICATOR_LED_MASK: u8 = 0b0000_0111;
#[cfg(feature = "real-executors")]
const INDICATOR_POWER_BIT: u8 = 0b0000_0001;

#[cfg(feature = "real-executors")]
pub(super) struct OnboardLis2dh12Imu {
    pins: I2cPins,
    address: u8,
    enabled: bool,
}

#[cfg(feature = "real-executors")]
pub(super) struct OnboardTca9555Indicators {
    pins: I2cPins,
    address: u8,
    port1_state: u8,
}

#[cfg(feature = "real-executors")]
pub(super) struct OnboardHdc2010Sensor {
    pins: I2cPins,
    address: u8,
}

#[cfg(feature = "real-executors")]
pub(super) struct OnboardMax17048Gauge {
    pins: I2cPins,
    address: u8,
}

#[cfg(feature = "real-executors")]
fn open_shared_i2c(pins: I2cPins) -> Option<I2c<'static, Blocking>> {
    I2c::new(
        unsafe { esp_hal::peripherals::I2C0::steal() },
        I2cConfig::default(),
    )
    .ok()
    .map(|bus| {
        bus.with_sda(unsafe { AnyPin::steal(pins.sda) })
            .with_scl(unsafe { AnyPin::steal(pins.scl) })
    })
}

#[cfg(feature = "real-executors")]
impl OnboardTca9555Indicators {
    pub(super) fn from_profile(
        profile: &BoardProfile,
        capabilities: BoardCapabilities,
    ) -> Option<Self> {
        match profile.runtime {
            BoardRuntimeKind::OwbS3V1 | BoardRuntimeKind::OwbS3V2 => {}
            _ => return None,
        }

        let address = capabilities.tca9555_addr?;
        let mut indicators = Self {
            pins: profile.i2c_bus,
            address,
            port1_state: INDICATOR_POWER_BIT,
        };

        if indicators.write_state("init") {
            info!("board indicators: TCA9555 @0x{address:02X} ready; power=on wifi=off user=off");
            Some(indicators)
        }
        else {
            None
        }
    }

    pub(super) fn set(
        &mut self,
        index: u8,
        on: bool,
    ) {
        let bit = match index {
            0 => 0b0000_0001,
            1 => 0b0000_0010,
            2 => 0b0000_0100,
            _ => {
                warn!("board indicators: invalid index {index}");
                return;
            }
        };

        if on {
            self.port1_state |= bit;
        }
        else {
            self.port1_state &= !bit;
        }

        if self.write_state("set") {
            info!(
                "board indicators: {}={}",
                Self::label(index),
                if on { "on" } else { "off" }
            );
        }
    }

    fn write_state(
        &mut self,
        action: &str,
    ) -> bool {
        let Some(bus) = open_shared_i2c(self.pins)
        else {
            warn!("board indicators: shared I2C open failed during {action}");
            return false;
        };

        let mut io = Tca9555::new(bus, self.address);
        if io.configure_outputs(0, INDICATOR_LED_MASK).is_err() {
            warn!(
                "board indicators: configure TCA9555 @0x{:02X} failed during {action}",
                self.address
            );
            return false;
        }

        if io
            .set_output_ports(0, self.port1_state & INDICATOR_LED_MASK)
            .is_err()
        {
            warn!(
                "board indicators: write TCA9555 @0x{:02X} failed during {action}",
                self.address
            );
            return false;
        }

        true
    }

    const fn label(index: u8) -> &'static str {
        match index {
            0 => "power",
            1 => "wifi",
            2 => "user",
            _ => "unknown",
        }
    }
}

#[cfg(feature = "real-executors")]
impl OnboardLis2dh12Imu {
    pub(super) fn from_profile(
        profile: &BoardProfile,
        capabilities: BoardCapabilities,
    ) -> Option<Self> {
        match profile.runtime {
            BoardRuntimeKind::OwbS3V1 | BoardRuntimeKind::OwbS3V2 => {}
            _ => return None,
        }
        if !capabilities.has_lis2dh12 {
            return None;
        }

        let address = if {
            let bus = open_shared_i2c(profile.i2c_bus)?;
            let mut sensor = Lis2dh12::new(bus, LIS_ADDR_LOW);
            matches!(sensor.who_am_i(), Ok(WHO_AM_I_RESPONSE))
        } {
            LIS_ADDR_LOW
        }
        else if {
            let bus = open_shared_i2c(profile.i2c_bus)?;
            let mut sensor = Lis2dh12::new(bus, LIS_ADDR_HIGH);
            matches!(sensor.who_am_i(), Ok(WHO_AM_I_RESPONSE))
        } {
            LIS_ADDR_HIGH
        }
        else {
            warn!("board imu: LIS2DH12 not found on shared bus");
            return None;
        };

        let bus = open_shared_i2c(profile.i2c_bus)?;
        let mut sensor = Lis2dh12::new(bus, address);
        if sensor.configure_runtime().is_err() {
            warn!("board imu: LIS2DH12 runtime configuration failed");
            return None;
        }

        Some(Self {
            pins: profile.i2c_bus,
            address,
            enabled: true,
        })
    }

    pub(super) fn set_enabled(
        &mut self,
        enabled: bool,
    ) {
        self.enabled = enabled;
    }

    pub(super) fn is_enabled(&self) -> bool { self.enabled }

    pub(super) fn read(&mut self) -> ImuData {
        if !self.enabled {
            return ImuData::default();
        }

        let Some(bus) = open_shared_i2c(self.pins)
        else {
            warn!("board imu: shared I2C open failed");
            return ImuData::default();
        };

        let mut sensor = Lis2dh12::new(bus, self.address);
        match sensor.read_accel_g() {
            Ok((x, y, z)) => ImuData {
                accel: (x, y, z),
                // The LIS2DH12 is an accelerometer with no rate sensor, so
                // these are placeholders to satisfy the shared `ImuData` shape
                // — NOT a measurement of "not rotating". Yaw rate is simply
                // unobservable on this board; heading comes from wheel
                // odometry. Roll/pitch are derived from `accel` by the gateway
                // via `owb_core::drivers::lis2dh12::tilt_from_accel_g`.
                gyro: (0.0, 0.0, 0.0),
                // Likewise unpopulated: the part's internal temperature sensor
                // is relative and uncalibrated, and the HDC2010 already
                // provides a real ambient reading via `read_env`.
                temp: 0.0,
            },
            Err(_) => {
                warn!("board imu: LIS2DH12 read failed");
                ImuData::default()
            }
        }
    }
}

#[cfg(feature = "real-executors")]
impl OnboardHdc2010Sensor {
    pub(super) fn from_profile(
        profile: &BoardProfile,
        capabilities: BoardCapabilities,
    ) -> Option<Self> {
        if !capabilities.has_hdc2010 {
            return None;
        }

        for address in [HDC_ADDR_LOW, HDC_ADDR_HIGH] {
            let bus = open_shared_i2c(profile.i2c_bus)?;
            let mut sensor = Hdc2010::new(bus, address);
            // Put the device into 1 Hz auto-measurement once, here at setup.
            // `read` below reconstructs the driver on every call, so it has no
            // place to keep state — but AMM lives in the device, so a single
            // init keeps every later read returning a fresh conversion instead
            // of the power-on 0x0000 (which decodes to -40.0 C / 0.0 %).
            if sensor.init().is_err() {
                continue;
            }
            if sensor.read_temperature_c().is_ok() && sensor.read_humidity_percent().is_ok() {
                return Some(Self {
                    pins: profile.i2c_bus,
                    address,
                });
            }
        }

        None
    }

    pub(super) fn read(&self) -> EnvData {
        let Some(bus) = open_shared_i2c(self.pins)
        else {
            warn!("board env: shared I2C open failed");
            return EnvData::default();
        };

        let mut sensor = Hdc2010::new(bus, self.address);
        match (sensor.read_temperature_c(), sensor.read_humidity_percent()) {
            (Ok(temp_c), Ok(humidity_percent)) => EnvData {
                temp_c,
                humidity_percent,
            },
            _ => {
                warn!("board env: HDC2010 read failed");
                EnvData::default()
            }
        }
    }
}

#[cfg(feature = "real-executors")]
impl OnboardMax17048Gauge {
    pub(super) fn from_profile(
        profile: &BoardProfile,
        capabilities: BoardCapabilities,
    ) -> Option<Self> {
        if !capabilities.has_max17048 {
            return None;
        }

        let bus = open_shared_i2c(profile.i2c_bus)?;
        let mut gauge = Max17048::new(bus, MAX17048_ADDR);
        if gauge.read_version().is_ok() {
            Some(Self {
                pins: profile.i2c_bus,
                address: MAX17048_ADDR,
            })
        }
        else {
            None
        }
    }

    pub(super) fn read(&self) -> PowerData {
        let Some(bus) = open_shared_i2c(self.pins)
        else {
            warn!("board power: shared I2C open failed");
            return PowerData::default();
        };

        let mut gauge = Max17048::new(bus, self.address);
        match (gauge.read_cell_voltage_mv(), gauge.read_soc_percent()) {
            (Ok(battery_mv), Ok(soc_percent)) => PowerData {
                battery_mv,
                soc_percent,
            },
            _ => {
                warn!("board power: MAX17048 read failed");
                PowerData::default()
            }
        }
    }
}
