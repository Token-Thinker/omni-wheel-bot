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
}
