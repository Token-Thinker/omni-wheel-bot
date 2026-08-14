//! Numeric utilities + the 20 Hz joystick throttle.
//!
//! Port of `throttle.ts`. The throttle enforces the firmware's preferred
//! send cadence:
//!   - At most one emit every `interval_ms` (default 50ms = 20 Hz).
//!   - Dedup when (direction_deg, speed_norm) is within 1° / 0.01.
//!   - Trailing-edge flush: the most-recent value queued while throttled is
//!     emitted after the cooldown so the firmware does not miss a release.
//!   - Keepalive: when the stick is held steady a forced emit is sent every
//!     200ms so the firmware's deadman watchdog (300ms) does not trip.

use std::{cell::RefCell, rc::Rc};

pub fn clamp01(v: f64) -> f64 {
    if !v.is_finite() {
        return 0.0;
    }
    v.clamp(0.0, 1.0)
}

pub fn wrap360(v: f64) -> f64 {
    if !v.is_finite() {
        return 0.0;
    }
    let mut r = v % 360.0;
    if r < 0.0 {
        r += 360.0;
    }
    r
}

pub fn round_to(
    v: f64,
    places: u32,
) -> f64 {
    let factor = 10_f64.powi(places as i32);
    (v * factor).round() / factor
}

const INTERVAL_MS: f64 = 50.0;
const DIR_EPSILON_DEG: f64 = 1.0;
const SPEED_EPSILON: f64 = 0.01;
const KEEPALIVE_MS: f64 = 200.0;

struct ThrottleInner {
    last_sent_dir: Option<f64>,
    last_sent_speed: Option<f64>,
    last_send_at: f64,
    last_actual_emit_at: f64,
    pending: Option<(f64, f64)>,
}

pub struct Throttle {
    emit: Rc<dyn Fn(f64, f64)>,
    inner: Rc<RefCell<ThrottleInner>>,
    trailing_handle: Option<gloo_timers::callback::Timeout>,
}

impl Throttle {
    pub fn new(emit: impl Fn(f64, f64) + 'static) -> Self {
        Self {
            emit: Rc::new(emit),
            inner: Rc::new(RefCell::new(ThrottleInner {
                last_sent_dir: None,
                last_sent_speed: None,
                last_send_at: 0.0,
                last_actual_emit_at: 0.0,
                pending: None,
            })),
            trailing_handle: None,
        }
    }

    /// Push a fresh sample. Emits immediately or coalesces into a trailing flush.
    pub fn push(
        &mut self,
        dir: f64,
        norm: f64,
    ) {
        let now = performance_now();
        let elapsed = now - self.inner.borrow().last_send_at;
        if elapsed >= INTERVAL_MS {
            self.inner.borrow_mut().last_send_at = now;
            self.inner.borrow_mut().pending = None;
            self.trailing_handle = None;
            self.try_emit(dir, norm, now);
            return;
        }
        self.inner.borrow_mut().pending = Some((dir, norm));
        if self.trailing_handle.is_none() {
            let delay_ms = (INTERVAL_MS - elapsed).max(0.0) as u32;
            let inner = self.inner.clone();
            let emit = self.emit.clone();
            let handle = gloo_timers::callback::Timeout::new(delay_ms, move || {
                let pending = inner.borrow_mut().pending.take();
                if let Some((d, n)) = pending {
                    let t = performance_now();
                    inner.borrow_mut().last_send_at = t;
                    try_emit_inner(&inner, &emit, d, n, t);
                }
            });
            self.trailing_handle = Some(handle);
        }
    }

    /// Called on stick release: force-emit a zero sample, bypassing dedup.
    pub fn flush_stop(&mut self) {
        self.trailing_handle = None;
        let now = performance_now();
        {
            let mut s = self.inner.borrow_mut();
            s.pending = None;
            s.last_send_at = now;
            s.last_actual_emit_at = now;
            s.last_sent_dir = None;
            s.last_sent_speed = None;
        }
        (self.emit)(0.0, 0.0);
    }

    fn try_emit(
        &mut self,
        dir: f64,
        norm: f64,
        now: f64,
    ) {
        try_emit_inner(&self.inner, &self.emit, dir, norm, now);
    }
}

fn try_emit_inner(
    inner: &Rc<RefCell<ThrottleInner>>,
    emit: &Rc<dyn Fn(f64, f64)>,
    dir: f64,
    norm: f64,
    now: f64,
) {
    let wrapped = wrap360(dir);
    let keepalive_forced = {
        let s = inner.borrow();
        (now - s.last_actual_emit_at) >= KEEPALIVE_MS
    };
    if !keepalive_forced {
        let s = inner.borrow();
        if let (Some(last_dir), Some(last_speed)) = (s.last_sent_dir, s.last_sent_speed) {
            if (wrapped - last_dir).abs() < DIR_EPSILON_DEG
                && (norm - last_speed).abs() < SPEED_EPSILON
            {
                return;
            }
        }
    }
    {
        let mut s = inner.borrow_mut();
        s.last_sent_dir = Some(wrapped);
        s.last_sent_speed = Some(norm);
        s.last_actual_emit_at = now;
    }
    emit(wrapped, norm);
}

fn performance_now() -> f64 {
    web_sys::window()
        .and_then(|w| w.performance())
        .map(|p| p.now())
        .unwrap_or(0.0)
}
