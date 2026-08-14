#[cfg(feature = "real-executors")]
use esp_hal::{
    gpio::{AnyPin, Input, InputConfig, Pull},
    pcnt::{channel, unit::Unit, Pcnt},
};
#[cfg(feature = "real-executors")]
use log::{info, warn};
#[cfg(feature = "real-executors")]
use owb_core::ctrl::sysctrl::EncoderData;

#[cfg(feature = "real-executors")]
use crate::profile::BoardProfile;

#[cfg(feature = "real-executors")]
pub(super) struct PcntEncoderCapture {
    _pcnt: Pcnt<'static>,
    _pins: [Input<'static>; 6],
    totals: [i32; 3],
    last_raw_counts: [i16; 3],
}

#[cfg(feature = "real-executors")]
impl PcntEncoderCapture {
    pub(super) fn from_profile(profile: &BoardProfile) -> Option<Self> {
        let channels = profile.encoder?;
        let pin_a0 = Self::new_encoder_input(channels[0].a);
        let pin_b0 = Self::new_encoder_input(channels[0].b);
        let pin_a1 = Self::new_encoder_input(channels[1].a);
        let pin_b1 = Self::new_encoder_input(channels[1].b);
        let pin_a2 = Self::new_encoder_input(channels[2].a);
        let pin_b2 = Self::new_encoder_input(channels[2].b);

        let pcnt = Pcnt::new(unsafe { esp_hal::peripherals::PCNT::steal() });
        if !Self::configure_unit(&pcnt.unit0, &pin_a0, &pin_b0)
            || !Self::configure_unit(&pcnt.unit1, &pin_a1, &pin_b1)
            || !Self::configure_unit(&pcnt.unit2, &pin_a2, &pin_b2)
        {
            warn!("encoder capture: PCNT setup failed");
            return None;
        }

        let last_raw_counts = [pcnt.unit0.value(), pcnt.unit1.value(), pcnt.unit2.value()];
        info!(
            "encoder capture: pcnt active on [({},{}) ({},{}) ({},{})]",
            channels[0].a,
            channels[0].b,
            channels[1].a,
            channels[1].b,
            channels[2].a,
            channels[2].b
        );

        Some(Self {
            _pcnt: pcnt,
            _pins: [pin_a0, pin_b0, pin_a1, pin_b1, pin_a2, pin_b2],
            totals: [0; 3],
            last_raw_counts,
        })
    }

    pub(super) fn read_deltas(&mut self) -> [i32; 3] {
        let raw_counts = self.raw_counts();
        let mut delta_counts = [0; 3];

        for index in 0..delta_counts.len() {
            let delta = raw_counts[index].wrapping_sub(self.last_raw_counts[index]) as i32;
            delta_counts[index] = delta;
            self.totals[index] += delta;
            self.last_raw_counts[index] = raw_counts[index];
        }

        delta_counts
    }

    pub(super) fn snapshot_and_update(&mut self) -> EncoderData {
        let delta_counts = self.read_deltas();

        EncoderData {
            counts: self.totals,
            delta_counts,
        }
    }

    pub(super) fn raw_levels(&self) -> [bool; 6] {
        [
            self._pins[0].is_high(),
            self._pins[1].is_high(),
            self._pins[2].is_high(),
            self._pins[3].is_high(),
            self._pins[4].is_high(),
            self._pins[5].is_high(),
        ]
    }

    fn configure_unit<const NUM: usize>(
        unit: &Unit<'static, NUM>,
        pin_a: &Input<'static>,
        pin_b: &Input<'static>,
    ) -> bool {
        let ch0 = &unit.channel0;
        let ch1 = &unit.channel1;
        let signal_a = pin_a.peripheral_input();
        let signal_b = pin_b.peripheral_input();

        if unit.set_filter(Some(800)).is_err() {
            return false;
        }
        if unit.set_low_limit(None).is_err() || unit.set_high_limit(None).is_err() {
            return false;
        }

        ch0.set_ctrl_signal(signal_a.clone());
        ch0.set_edge_signal(signal_b.clone());
        ch0.set_ctrl_mode(channel::CtrlMode::Reverse, channel::CtrlMode::Keep);
        ch0.set_input_mode(channel::EdgeMode::Increment, channel::EdgeMode::Decrement);

        ch1.set_ctrl_signal(signal_b);
        ch1.set_edge_signal(signal_a);
        ch1.set_ctrl_mode(channel::CtrlMode::Reverse, channel::CtrlMode::Keep);
        ch1.set_input_mode(channel::EdgeMode::Decrement, channel::EdgeMode::Increment);

        unit.clear();
        unit.resume();
        true
    }

    fn new_encoder_input(pin: u8) -> Input<'static> {
        Input::new(
            unsafe { AnyPin::steal(pin) },
            InputConfig::default().with_pull(Pull::Up),
        )
    }

    pub(super) fn raw_counts(&self) -> [i16; 3] {
        [
            self._pcnt.unit0.value(),
            self._pcnt.unit1.value(),
            self._pcnt.unit2.value(),
        ]
    }
}
