#[cfg(feature = "real-executors")]
use esp_hal::{
    gpio::{AnyPin, Level, Output, OutputConfig},
    rmt::{Channel, PulseCode, Rmt, Tx, TxChannelConfig, TxChannelCreator},
    time::Rate,
    Blocking,
};
#[cfg(feature = "real-executors")]
use log::warn;

#[cfg(feature = "real-executors")]
const ADDRESSABLE_MAX_PIXELS: usize = 4;
#[cfg(feature = "real-executors")]
const ADDRESSABLE_BITS_PER_PIXEL: usize = 24;
#[cfg(feature = "real-executors")]
const ADDRESSABLE_FRAME_LEN: usize = ADDRESSABLE_MAX_PIXELS * ADDRESSABLE_BITS_PER_PIXEL + 1;

#[cfg(feature = "real-executors")]
pub(super) struct GpioStatusLed {
    pin: Output<'static>,
    active_high: bool,
}

#[cfg(feature = "real-executors")]
impl GpioStatusLed {
    pub(super) fn new(
        pin: u8,
        active_high: bool,
    ) -> Self {
        let mut pin = Output::new(
            unsafe { AnyPin::steal(pin) },
            if active_high { Level::Low } else { Level::High },
            OutputConfig::default(),
        );
        if active_high {
            pin.set_low();
        }
        else {
            pin.set_high();
        }
        Self { pin, active_high }
    }

    pub(super) fn set_on(
        &mut self,
        on: bool,
    ) {
        let asserted = if self.active_high { on } else { !on };
        if asserted {
            self.pin.set_high();
        }
        else {
            self.pin.set_low();
        }
    }
}

#[cfg(feature = "real-executors")]
#[derive(Clone, Copy)]
pub(super) struct RgbColor {
    pub(super) r: u8,
    pub(super) g: u8,
    pub(super) b: u8,
}

#[cfg(feature = "real-executors")]
pub(super) struct AddressableLedDriver {
    channel: Option<Channel<'static, Blocking, Tx>>,
    pixels: u8,
    inverted: bool,
}

#[cfg(feature = "real-executors")]
impl AddressableLedDriver {
    pub(super) fn new(
        pin: u8,
        pixels: u8,
        inverted: bool,
    ) -> Option<Self> {
        let pixels = pixels.clamp(1, ADDRESSABLE_MAX_PIXELS as u8);
        let idle_level = if inverted { Level::High } else { Level::Low };

        let rmt = match Rmt::new(
            unsafe { esp_hal::peripherals::RMT::steal() },
            Rate::from_mhz(80),
        ) {
            Ok(rmt) => rmt,
            Err(_) => {
                warn!("addressable led: RMT init failed");
                return None;
            }
        };

        let tx = TxChannelConfig::default()
            .with_clk_divider(4)
            .with_idle_output(true)
            .with_idle_output_level(idle_level)
            .with_carrier_modulation(false);

        // esp-hal 1.1: configure_tx takes only the config by reference and the
        // output pin is bound afterwards via Channel::with_pin.
        let channel = match rmt.channel0.configure_tx(&tx) {
            Ok(channel) => channel.with_pin(unsafe { AnyPin::steal(pin) }),
            Err(_) => {
                warn!("addressable led: configure_tx failed on GPIO{pin}");
                return None;
            }
        };

        Some(Self {
            channel: Some(channel),
            pixels,
            inverted,
        })
    }

    pub(super) fn set_rgb(
        &mut self,
        color: RgbColor,
    ) {
        let Some(channel) = self.channel.take()
        else {
            warn!("addressable led: channel unavailable");
            return;
        };

        let (data, used) = encode_addressable_frame(color, self.pixels, self.inverted);
        self.channel = {
            let tx = channel.transmit(&data[..used]);
            match tx {
                Ok(transaction) => match transaction.wait() {
                    Ok(channel) => Some(channel),
                    Err((_, channel)) => {
                        warn!("addressable led: transmit wait failed");
                        Some(channel)
                    }
                },
                Err(_) => {
                    warn!("addressable led: transmit start failed");
                    None
                }
            }
        };
    }
}

#[cfg(feature = "real-executors")]
fn encode_addressable_frame(
    color: RgbColor,
    pixels: u8,
    inverted: bool,
) -> ([PulseCode; ADDRESSABLE_FRAME_LEN], usize) {
    let mut data = [PulseCode::default(); ADDRESSABLE_FRAME_LEN];
    let mut idx = 0usize;
    let active = if inverted { Level::Low } else { Level::High };
    let idle = if inverted { Level::High } else { Level::Low };
    let pixels = usize::from(pixels.clamp(1, ADDRESSABLE_MAX_PIXELS as u8));

    for _ in 0..pixels {
        for byte in [color.g, color.r, color.b] {
            for bit in (0..8).rev() {
                let one = ((byte >> bit) & 1) != 0;
                let (high_ticks, low_ticks) = if one { (16, 9) } else { (8, 17) };
                data[idx] = PulseCode::new(active, high_ticks, idle, low_ticks);
                idx += 1;
            }
        }
    }

    data[idx] = PulseCode::end_marker();
    (data, idx + 1)
}

/// Animated effect state for the addressable LED strip. Advanced one `phase`
/// step per board background tick; integer-only so it needs no float/libm dep.
#[cfg(feature = "real-executors")]
#[derive(Clone, Copy)]
pub(super) enum LedEffect {
    None,
    Breath { r: u8, g: u8, b: u8 },
    Wheel,
}

#[cfg(feature = "real-executors")]
impl LedEffect {
    /// Color to display for an 8-bit `phase` (0..=255). Returns `None` for a
    /// static effect that needs no per-tick redraw.
    pub(super) fn color_at(
        self,
        phase: u8,
    ) -> Option<RgbColor> {
        match self {
            LedEffect::None => None,
            LedEffect::Breath { r, g, b } => {
                // Triangle wave 0->~255->0 across the phase, scaled per channel.
                let level = u16::from(if phase < 128 { phase } else { 255 - phase }) * 2;
                let scale = |c: u8| ((u16::from(c) * level) / 255) as u8;
                Some(RgbColor {
                    r: scale(r),
                    g: scale(g),
                    b: scale(b),
                })
            }
            LedEffect::Wheel => Some(color_wheel(phase)),
        }
    }
}

/// Map 0..=255 to a full-saturation RGB color wheel (integer only).
#[cfg(feature = "real-executors")]
pub(super) fn color_wheel(pos: u8) -> RgbColor {
    let p = 255 - pos;
    if p < 85 {
        RgbColor {
            r: 255 - p * 3,
            g: 0,
            b: p * 3,
        }
    }
    else if p < 170 {
        let p = p - 85;
        RgbColor {
            r: 0,
            g: p * 3,
            b: 255 - p * 3,
        }
    }
    else {
        let p = p - 170;
        RgbColor {
            r: p * 3,
            g: 255 - p * 3,
            b: 0,
        }
    }
}

#[cfg(all(test, feature = "real-executors"))]
mod tests {
    use esp_hal::rmt::PulseCode;

    use super::{encode_addressable_frame, RgbColor, ADDRESSABLE_BITS_PER_PIXEL};

    #[test]
    fn addressable_frame_has_end_marker_after_requested_pixels() {
        let (_data, used) = encode_addressable_frame(RgbColor { r: 1, g: 2, b: 3 }, 2, false);
        assert_eq!(used, 2 * ADDRESSABLE_BITS_PER_PIXEL + 1);
    }

    #[test]
    fn addressable_frame_terminates_with_end_marker() {
        let (data, used) = encode_addressable_frame(RgbColor { r: 255, g: 0, b: 0 }, 1, false);
        assert_eq!(data[used - 1].0, PulseCode::end_marker().0);
    }
}
