//! Shared planning helpers for multi-ToF board layouts.
//!
//! The current custom boards route three ToF sockets behind per-port LPn and
//! INT lines, so the MVP needs a consistent way to talk about those ports even
//! before the final sensor driver is attached.

/// Logical ToF port on the custom boards.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TofPort {
    FrontLeft,
    FrontRight,
    Rear,
}

/// One ToF channel's control-line mapping.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TofChannel {
    pub port: TofPort,
    pub lpn_bit: u8,
    pub int_bit: u8,
    pub target_addr: u8,
}

/// Default three-port ToF layout used by the current OWB custom boards.
pub const DEFAULT_TOF_CHANNELS: [TofChannel; 3] = [
    TofChannel {
        port: TofPort::FrontLeft,
        lpn_bit: 7,
        int_bit: 6,
        target_addr: 0x30,
    },
    TofChannel {
        port: TofPort::FrontRight,
        lpn_bit: 8,
        int_bit: 5,
        target_addr: 0x31,
    },
    TofChannel {
        port: TofPort::Rear,
        lpn_bit: 9,
        int_bit: 4,
        target_addr: 0x32,
    },
];

/// Two-byte TCA9555 output image for the current LPn bit assignments.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TofLpnMask {
    pub port0: u8,
    pub port1: u8,
}

impl TofLpnMask {
    pub const fn all_disabled() -> Self { Self { port0: 0, port1: 0 } }

    pub fn with_enabled(
        mut self,
        bit: u8,
    ) -> Self {
        if bit < 8 {
            self.port0 |= 1 << bit;
        }
        else {
            self.port1 |= 1 << (bit - 8);
        }
        self
    }
}

/// TCA9555 **port1** bits already claimed by the onboard indicator LEDs
/// (power, wifi, user).
///
/// Mirrors `INDICATOR_LED_MASK` in `owb-esp`'s `board::sensors`. It is
/// duplicated rather than shared because that lives in a binary crate, so if
/// one changes the other must be updated by hand.
///
/// This matters because the indicator writer is not cooperative: it drives
/// port0 to `0` unconditionally and masks port1 down to these bits on every
/// update. Anything else sharing the expander gets clobbered on the next LED
/// change, so ToF LPn lines must avoid these bits.
pub const TCA9555_INDICATOR_PORT1_MASK: u8 = 0b0000_0111;

/// Whether a channel's LPn bit collides with an indicator LED bit.
///
/// Bits 0-7 address port0 (free today); bits 8-15 address port1, where the
/// indicator LEDs live.
pub fn lpn_bit_conflicts_with_indicators(lpn_bit: u8) -> bool {
    lpn_bit >= 8 && (1u8 << (lpn_bit - 8)) & TCA9555_INDICATOR_PORT1_MASK != 0
}

/// Build the LPn enable sequence used to bring each ToF sensor up one at a time
/// and assign it a unique I2C address.
pub fn bringup_sequence(channels: &[TofChannel]) -> [TofLpnMask; 3] {
    let mut masks = [TofLpnMask::all_disabled(); 3];
    let mut current = TofLpnMask::all_disabled();
    let mut i = 0usize;

    while i < channels.len() && i < masks.len() {
        current = current.with_enabled(channels[i].lpn_bit);
        masks[i] = current;
        i += 1;
    }

    masks
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_tof_addresses_are_unique() {
        assert_ne!(
            DEFAULT_TOF_CHANNELS[0].target_addr,
            DEFAULT_TOF_CHANNELS[1].target_addr
        );
        assert_ne!(
            DEFAULT_TOF_CHANNELS[1].target_addr,
            DEFAULT_TOF_CHANNELS[2].target_addr
        );
        assert_ne!(
            DEFAULT_TOF_CHANNELS[0].target_addr,
            DEFAULT_TOF_CHANNELS[2].target_addr
        );
    }

    #[test]
    fn bringup_sequence_enables_channels_one_at_a_time() {
        let steps = bringup_sequence(&DEFAULT_TOF_CHANNELS);
        assert_eq!(
            steps[0],
            TofLpnMask {
                port0: 0x80,
                port1: 0x00
            }
        );
        assert_eq!(
            steps[1],
            TofLpnMask {
                port0: 0x80,
                port1: 0x01
            }
        );
        assert_eq!(
            steps[2],
            TofLpnMask {
                port0: 0x80,
                port1: 0x03
            }
        );
    }

    /// Characterisation test: the **current** default ToF map collides with the
    /// indicator LEDs, and that is a known-unresolved hardware planning bug, not
    /// an accident of this test.
    ///
    /// `lpn_bit: 8` and `lpn_bit: 9` land on TCA9555 port1 bits 0 and 1, which
    /// are the power and wifi indicator bits. Because the indicator writer
    /// rewrites port0 and masks port1 on every LED update, wiring ToF to these
    /// bits would mean an LED change powers a range sensor down -- and a ToF
    /// bring-up blinks the status LEDs.
    ///
    /// Nothing reads these channels yet (this module is planning-only, no I2C),
    /// so nothing is broken in the field. The v2 pin map has to be redone from
    /// scratch anyway; when it is, move these LPn lines to free bits (port0
    /// 0-6, or port1 3-7) and flip this test to assert the *absence* of
    /// conflicts.
    #[test]
    fn default_channels_currently_conflict_with_indicator_leds() {
        let mut conflicting = [None; 3];
        let mut n = 0;
        for channel in DEFAULT_TOF_CHANNELS.iter() {
            if lpn_bit_conflicts_with_indicators(channel.lpn_bit) {
                conflicting[n] = Some((channel.port, channel.lpn_bit));
                n += 1;
            }
        }

        // If this expectation changes, re-check the pin map against
        // INDICATOR_LED_MASK before touching the assertion.
        assert_eq!(n, 2, "expected exactly two colliding channels");
        assert_eq!(conflicting[0], Some((TofPort::FrontRight, 8)));
        assert_eq!(conflicting[1], Some((TofPort::Rear, 9)));

        // The bring-up sequence drives precisely the bits the LED mask owns,
        // which is the concrete form of the collision.
        let steps = bringup_sequence(&DEFAULT_TOF_CHANNELS);
        assert_ne!(
            steps[2].port1 & TCA9555_INDICATOR_PORT1_MASK,
            0,
            "fully-enabled ToF LPn image must currently overlap the LED mask"
        );
    }

    /// The front-left channel is on port0, which the indicators do not claim as
    /// LED outputs -- so the conflict is specific to the port1 assignments.
    #[test]
    fn port0_lpn_bits_do_not_conflict() {
        assert!(!lpn_bit_conflicts_with_indicators(0));
        assert!(!lpn_bit_conflicts_with_indicators(7));
    }

    /// Port1 bits above the LED mask are free for reassignment.
    #[test]
    fn port1_bits_above_the_led_mask_are_free() {
        assert!(lpn_bit_conflicts_with_indicators(8));
        assert!(lpn_bit_conflicts_with_indicators(9));
        assert!(lpn_bit_conflicts_with_indicators(10));
        assert!(!lpn_bit_conflicts_with_indicators(11));
        assert!(!lpn_bit_conflicts_with_indicators(15));
    }
}
