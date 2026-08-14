//! LED control module for the Omni-Wheel Bot.
//!
//! Manages an addressable LED strip via `SmartLedsWrite` and dispatches
//! commands received over `LED_CHANNEL`.

use smart_leds_trait::{SmartLedsWrite, RGB8};

use super::sysctrl::LEDCommand;

/// Number of LEDs in the attached chain.
const LED_COUNT: usize = 2;

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
    /// - `Brightness { v }`: scale the current color by `v/255`, or set a
    ///   grey at that brightness if no color has been set yet.
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
            LEDCommand::Brightness { v } => {
                let base = self.last_color.unwrap_or(RGB8 {
                    r: 255,
                    g: 255,
                    b: 255,
                });
                let scaled = RGB8 {
                    r: ((base.r as u16 * v as u16) / 255) as u8,
                    g: ((base.g as u16 * v as u16) / 255) as u8,
                    b: ((base.b as u16 * v as u16) / 255) as u8,
                };
                if self.is_on {
                    self.set_all(scaled)?;
                }
            }
        }
        Ok(())
    }

    /// Set all LEDs in the strip to the specified color.
    #[allow(clippy::manual_repeat_n)]
    fn set_all(
        &mut self,
        color: RGB8,
    ) -> Result<(), E> {
        let data = core::iter::repeat(color).take(LED_COUNT);
        self.driver.write(data)
    }
}
