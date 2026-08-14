//! Shared quadrature encoder helpers for pre-HIL and host validation.

/// Incremental quadrature decoder state.
#[derive(Clone, Copy, Debug, Default)]
pub struct QuadratureDecoder {
    last_state: u8,
    count: i32,
}

impl QuadratureDecoder {
    pub const fn new() -> Self {
        Self {
            last_state: 0,
            count: 0,
        }
    }

    pub fn count(&self) -> i32 { self.count }

    pub fn update(
        &mut self,
        a: bool,
        b: bool,
    ) -> i8 {
        let state = ((a as u8) << 1) | (b as u8);
        let delta = match (self.last_state, state) {
            (0b00, 0b01) | (0b01, 0b11) | (0b11, 0b10) | (0b10, 0b00) => 1,
            (0b00, 0b10) | (0b10, 0b11) | (0b11, 0b01) | (0b01, 0b00) => -1,
            _ => 0,
        };
        self.last_state = state;
        // The absolute count is modular: it accumulates ±1 per edge and is
        // expected to wrap over very long runs. Make that explicit so a
        // release build with `overflow-checks = false` cannot UB-wrap, and a
        // debug build does not panic on the eventual overflow.
        self.count = self.count.wrapping_add(delta as i32);
        delta
    }
}

/// Convert encoder count deltas into wheel rev/s.
#[derive(Clone, Copy, Debug)]
pub struct VelocityEstimator {
    counts_per_revolution: f32,
}

impl VelocityEstimator {
    pub const fn new(counts_per_revolution: f32) -> Self {
        Self {
            counts_per_revolution,
        }
    }

    pub fn rev_per_sec(
        &self,
        delta_counts: i32,
        dt_s: f32,
    ) -> f32 {
        if self.counts_per_revolution <= 0.0 || !dt_s.is_finite() || dt_s <= 0.0 {
            return 0.0;
        }

        delta_counts as f32 / self.counts_per_revolution / dt_s
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quadrature_forward_sequence_counts_up() {
        let mut dec = QuadratureDecoder::new();
        let seq = [
            (false, false),
            (false, true),
            (true, true),
            (true, false),
            (false, false),
        ];
        let mut total = 0i32;
        for (a, b) in seq {
            total += dec.update(a, b) as i32;
        }
        assert!(total > 0);
        assert_eq!(dec.count(), total);
    }

    #[test]
    fn quadrature_reverse_sequence_counts_down() {
        let mut dec = QuadratureDecoder::new();
        let seq = [
            (false, false),
            (true, false),
            (true, true),
            (false, true),
            (false, false),
        ];
        let mut total = 0i32;
        for (a, b) in seq {
            total += dec.update(a, b) as i32;
        }
        assert!(total < 0);
        assert_eq!(dec.count(), total);
    }

    #[test]
    fn velocity_estimator_converts_counts() {
        let est = VelocityEstimator::new(12.0 * 50.0);
        let rps = est.rev_per_sec(60, 0.1);
        assert!((rps - 1.0).abs() < 1e-6);
    }
}
