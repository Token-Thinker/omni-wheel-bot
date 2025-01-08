use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use serde::{Deserialize, Serialize};
use smart_leds_trait::{SmartLedsWrite, RGB8};

// Global communication channel for LED commands.
pub static LED_CHANNEL: embassy_sync::channel::Channel<CriticalSectionRawMutex, LEDCommand, 16> =
    embassy_sync::channel::Channel::new();

/// Number of LEDs in your chain. Here, just 2.
const LED_COUNT: usize = 2;

/// Tagged enum for LED commands.
/// The top-level JSON key is `"led_cmd"`.
#[derive(Debug, Serialize, Deserialize)]
#[serde(tag = "lc", rename_all = "snake_case")] // lc = led command
pub enum LEDCommand
{
    /// Turn the LEDs on.
    /// If `last_color` is None, default to white.
    On,
    /// Turn all LEDs off (set to black).
    Off,
    /// Set color explicitly.
    /// Example JSON: `{ "lc": "set_color", "r": 255, "g": 128, "b": 0 }`
    SC
    {
        r: u8, g: u8, b: u8
    },
}

/// A “LED Module” that manages a chain of `LED_COUNT` addressable LEDs.
///
/// It is generic over any driver that implements `SmartLedsWrite<Color =
/// RGB8>`. You can pass in, for example, `esp_hal_smartled::SmartLedsAdapter`.
pub struct LedModule<Driver>
{
    driver: Driver,
    is_on: bool,
    last_color: Option<RGB8>,
}

impl<Driver, E> LedModule<Driver>
where
    Driver: SmartLedsWrite<Color = RGB8, Error = E>,
{
    /// Create a new module. By default, `is_on = false`, `last_color = None`.
    pub fn new(driver: Driver) -> Self
    {
        Self {
            driver,
            is_on: false,
            last_color: None,
        }
    }

    /// Handle an incoming command:
    /// - `On` => set `is_on = true`; use `last_color` or default to white.
    /// - `Off` => set `is_on = false`; turn strip black.
    /// - `SetColor` => store in `last_color`; if `is_on`, apply immediately.
    pub fn ex_command(
        &mut self,
        cmd: LEDCommand,
    ) -> Result<(), E>
    {
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

    /// Helper: set all LEDs in the chain to the same color.
    fn set_all(
        &mut self,
        color: RGB8,
    ) -> Result<(), E>
    {
        // Optionally apply brightness or gamma here. Example:
        // use smart_leds::brightness;
        // let data = brightness(core::iter::repeat(color).take(LED_COUNT), 128);

        let data = core::iter::repeat(color).take(LED_COUNT);
        self.driver.write(data)
    }
}
