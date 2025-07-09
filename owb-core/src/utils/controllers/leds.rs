//! LED control module for the Omni-Wheel Bot.
//!
//! Manages an addressable LED strip via `SmartLedsWrite` and dispatches commands
//! received over `LED_CHANNEL`.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use serde::{Deserialize, Serialize};
use smart_leds_trait::{SmartLedsWrite, RGB8};

/// Channel used to receive LED commands (`LEDCommand` messages).
pub static LED_CHANNEL: embassy_sync::channel::Channel<CriticalSectionRawMutex, LEDCommand, 16> =
    embassy_sync::channel::Channel::new();

/// Number of LEDs in the attached chain.
const LED_COUNT: usize = 2;

/// LED command variants for switching on/off or setting a color.
///
/// Serialized as JSON with tag `"lc"`.
#[derive(Debug, Serialize, Deserialize, Clone, Copy)]
#[serde(tag = "lc", rename_all = "snake_case")]
pub enum LEDCommand {
    /// Turn the LEDs on (last color or white).
    On,
    /// Turn all LEDs off (set to black).
    Off,
    /// Set the LED strip to the given RGB color.
    SC { r: u8, g: u8, b: u8 },
}

/// High-level LED controller that drives a strip of addressable LEDs.
///
/// Maintains the on/off state and last selected color.
pub struct LedModule<Driver> {
    driver: Driver,
    is_on: bool,
    last_color: Option<RGB8>,
}

impl<Driver, E> LedModule<Driver>
where
    Driver: SmartLedsWrite<Color = RGB8, Error = E>,
{
    /// Create a new `LedModule` over the given LED driver.
    ///
    /// The strip is initially off with no last color.
    pub fn new(driver: Driver) -> Self {
        Self {
            driver,
            is_on: false,
            last_color: None,
        }
    }

    /// Execute an incoming `LEDCommand`, updating internal state and LED strip.
    ///
    /// - `On`: enable LEDs with the last color or white.
    /// - `Off`: disable LEDs (all black).
    /// - `SC {r,g,b}`: set a new color, applied immediately if strip is on.
    pub fn ex_command(
        &mut self,
        cmd: LEDCommand,
    ) -> Result<(), E> {
        match cmd {
            LEDCommand::On => {
                self.is_on = true;
                let color = self.last_color.unwrap_or(RGB8 {
                    r: 255,
                    g: 255,
                    b: 255,
                });
                self.set_all(color)?;
            }
            LEDCommand::Off => {
                self.is_on = false;
                self.set_all(RGB8 { r: 0, g: 0, b: 0 })?;
            }
            LEDCommand::SC { r, g, b } => {
                let new_color = RGB8 { r, g, b };
                self.last_color = Some(new_color);
                if self.is_on {
                    self.set_all(new_color)?;
                }
            }
        }
        Ok(())
    }

    /// Set all LEDs in the strip to the specified color.
    fn set_all(
        &mut self,
        color: RGB8,
    ) -> Result<(), E> {
        let data = core::iter::repeat(color).take(LED_COUNT);
        self.driver.write(data)
    }
}
