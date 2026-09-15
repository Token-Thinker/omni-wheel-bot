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
    /// Per-consumer `totals` baselines. The control loop and the script-facing
    /// `read_encoders` query both want "counts since *I* last looked", so they
    /// cannot share one baseline: whoever polled first would zero the delta for
    /// the other. The control loop would then see a stalled wheel and wind up
    /// its integrator, and `read_encoders` would report 0 while the bot drives.
    control_baseline: [i32; 3],
    query_baseline: [i32; 3],
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
            control_baseline: [0; 3],
            query_baseline: [0; 3],
        })
    }

    /// Fold the hardware counters into `totals` and rebase against the raw
    /// values. Safe to call any number of times per control tick: a second
    /// call simply adds a zero delta, so `totals` stays an honest absolute
    /// count no matter which consumer polls first.
    fn poll(&mut self) {
        let raw_counts = self.raw_counts();
        for ((total, last), raw) in self
            .totals
            .iter_mut()
            .zip(self.last_raw_counts.iter_mut())
            .zip(raw_counts)
        {
            // PCNT units are 16-bit and free-running (no limits set), so
            // `wrapping_sub` is what makes roll-over transparent.
            *total += raw.wrapping_sub(*last) as i32;
            *last = raw;
        }
    }

    /// Counts accumulated since the *control loop* last asked.
    pub(super) fn read_deltas(&mut self) -> [i32; 3] {
        self.poll();
        Self::drain(&self.totals, &mut self.control_baseline)
    }

    /// Absolute counts plus the delta since the *query path* last asked.
    pub(super) fn snapshot_and_update(&mut self) -> EncoderData {
        self.poll();
        let delta_counts = Self::drain(&self.totals, &mut self.query_baseline);

        EncoderData {
            counts: self.totals,
            delta_counts,
        }
    }

    /// Difference `totals` against one consumer's baseline and advance it.
    fn drain(
        totals: &[i32; 3],
        baseline: &mut [i32; 3],
    ) -> [i32; 3] {
        let mut delta_counts = [0i32; 3];
        for ((delta, total), base) in delta_counts
            .iter_mut()
            .zip(totals.iter())
            .zip(baseline.iter_mut())
        {
            *delta = total.wrapping_sub(*base);
            *base = *total;
        }
        delta_counts
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
