//! Floating-spawn joystick component.
//!
//! Port of `joystick.ts`. The joystick spawns centered at the touch point on
//! pointerdown and disappears on release. Response curve is applied before
//! the throttle so dedup compares wire values.

use std::{cell::RefCell, rc::Rc};

use leptos::prelude::*;
use web_sys::PointerEvent;

use crate::{
    throttle::{clamp01, wrap360},
    transport::{ConnState, I2CCommand, WsClient},
};

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ResponseCurve {
    Linear,
    Quadratic,
    Cubic,
}

impl ResponseCurve {
    pub fn apply(
        self,
        n: f64,
    ) -> f64 {
        let n = clamp01(n);
        clamp01(match self {
            ResponseCurve::Linear => n,
            ResponseCurve::Quadratic => n * n,
            ResponseCurve::Cubic => n * n * n,
        })
    }

    pub fn as_str(self) -> &'static str {
        match self {
            ResponseCurve::Linear => "linear",
            ResponseCurve::Quadratic => "quadratic",
            ResponseCurve::Cubic => "cubic",
        }
    }

    pub fn from_str(s: &str) -> Self {
        match s {
            "linear" => ResponseCurve::Linear,
            "quadratic" => ResponseCurve::Quadratic,
            _ => ResponseCurve::Cubic,
        }
    }
}

/// Shared mutable state for the joystick pointer tracking.
struct JoystickState {
    pointer_id: Option<i32>,
    center_x: f64,
    center_y: f64,
    radius: f64,
}

impl JoystickState {
    fn new() -> Self {
        JoystickState {
            pointer_id: None,
            center_x: 0.0,
            center_y: 0.0,
            radius: 1.0,
        }
    }
}

#[component]
pub fn JoystickArea(
    ws: Rc<WsClient>,
    /// Reserved for future use: disable joystick input when disconnected.
    #[allow(unused)]
    conn_state: RwSignal<ConnState>,
    curve: RwSignal<ResponseCurve>,
) -> impl IntoView {
    let dir_text = RwSignal::new(String::from("0\u{00b0}"));
    let speed_text = RwSignal::new(String::from("0.00"));
    let stick_left = RwSignal::new(String::from("0px"));
    let stick_top = RwSignal::new(String::from("0px"));
    let knob_transform = RwSignal::new(String::from("translate(0px, 0px)"));

    let area_ref = NodeRef::<leptos::html::Main>::new();
    let joystick_ref = NodeRef::<leptos::html::Div>::new();

    let js_state = Rc::new(RefCell::new(JoystickState::new()));

    // Throttle: last emit time and state tracked in shared cell
    let last_send_at = Rc::new(RefCell::new(0.0_f64));
    let last_actual_emit_at = Rc::new(RefCell::new(0.0_f64));
    let last_sent_dir: Rc<RefCell<Option<f64>>> = Rc::new(RefCell::new(None));
    let last_sent_speed: Rc<RefCell<Option<f64>>> = Rc::new(RefCell::new(None));
    let trailing_timer: Rc<RefCell<Option<gloo_timers::callback::Timeout>>> =
        Rc::new(RefCell::new(None));

    // Helper: emit a drive command
    let ws_send = {
        let ws = ws.clone();
        move |dir_deg: f64, speed: f64| {
            let cmd = I2CCommand::translate(
                crate::throttle::round_to(wrap360(dir_deg), 0) as f32,
                crate::throttle::round_to(clamp01(speed.abs()), 2) as f32,
            );
            ws.send(&cmd);
        }
    };

    // on_pointer_down
    let on_pointer_down = {
        let js_state = js_state.clone();
        let area_ref = area_ref.clone();
        let joystick_ref = joystick_ref.clone();
        let stick_left = stick_left;
        let stick_top = stick_top;
        let knob_transform = knob_transform;
        let dir_text = dir_text;
        let speed_text = speed_text;
        let last_send_at = last_send_at.clone();
        let last_actual_emit_at = last_actual_emit_at.clone();
        let last_sent_dir = last_sent_dir.clone();
        let last_sent_speed = last_sent_speed.clone();
        let trailing_timer = trailing_timer.clone();
        let ws_send = ws_send.clone();
        let curve = curve;

        move |ev: PointerEvent| {
            let mut state = js_state.borrow_mut();
            if state.pointer_id.is_some() {
                return;
            }
            state.pointer_id = Some(ev.pointer_id());
            state.center_x = ev.client_x() as f64;
            state.center_y = ev.client_y() as f64;

            // Position the joystick centered on the touch point
            if let Some(area_el) = area_ref.get() {
                let area_rect = area_el.get_bounding_client_rect();
                let left = ev.client_x() as f64 - area_rect.left();
                let top = ev.client_y() as f64 - area_rect.top();
                stick_left.set(format!("{left}px"));
                stick_top.set(format!("{top}px"));
            }

            // Cache radius from the joystick element dimensions
            if let Some(stick_el) = joystick_ref.get() {
                let rect = stick_el.get_bounding_client_rect();
                state.radius = (rect.width().min(rect.height()) / 2.0).max(1.0);
            }

            // Capture pointer on the area element
            if let Some(area_el) = area_ref.get() {
                let _ = area_el.set_pointer_capture(ev.pointer_id());
            }

            // Mark body as active
            if let Some(body) = web_sys::window()
                .and_then(|w| w.document())
                .and_then(|d| d.body())
            {
                let _ = body.set_attribute("data-stick", "active");
                let _ = body.set_attribute("data-stick-touched", "1");
            }

            // Reset readouts
            knob_transform.set(String::from("translate(0px, 0px)"));
            dir_text.set(String::from("0\u{00b0}"));
            speed_text.set(String::from("0.00"));

            // Emit initial position (center = stop)
            drop(state);
            let now = performance_now();
            *last_send_at.borrow_mut() = now;
            *last_actual_emit_at.borrow_mut() = now;
            *last_sent_dir.borrow_mut() = Some(0.0);
            *last_sent_speed.borrow_mut() = Some(0.0);
            *trailing_timer.borrow_mut() = None;
            ws_send(0.0, 0.0);
            let _ = curve; // suppress unused warning
        }
    };

    // on_pointer_move
    let on_pointer_move = {
        let js_state = js_state.clone();
        let knob_transform = knob_transform;
        let dir_text = dir_text;
        let speed_text = speed_text;
        let last_send_at = last_send_at.clone();
        let last_actual_emit_at = last_actual_emit_at.clone();
        let last_sent_dir = last_sent_dir.clone();
        let last_sent_speed = last_sent_speed.clone();
        let trailing_timer = trailing_timer.clone();
        let ws_send = ws_send.clone();
        let curve = curve;

        move |ev: PointerEvent| {
            let state = js_state.borrow();
            if state.pointer_id != Some(ev.pointer_id()) {
                return;
            }
            let dx = ev.client_x() as f64 - state.center_x;
            let dy = ev.client_y() as f64 - state.center_y;
            let dist = (dx * dx + dy * dy).sqrt();
            let norm = clamp01(dist / state.radius.max(1.0));
            let dir = if dist > 0.0001 {
                wrap360((dx.atan2(-dy) * 180.0) / std::f64::consts::PI)
            }
            else {
                0.0
            };

            // Update knob position (clamped to ring boundary)
            let limit = state.radius;
            let (px, py) = if dist > limit {
                ((dx / dist) * limit, (dy / dist) * limit)
            }
            else {
                (dx, dy)
            };
            knob_transform.set(format!("translate({px}px, {py}px)"));

            let applied = curve.get().apply(norm);
            dir_text.set(format!("{}\u{00b0}", dir.round() as i32));
            speed_text.set(format!("{applied:.2}"));

            drop(state);

            // Throttle logic (20 Hz, dedup, keepalive)
            let now = performance_now();
            let elapsed = now - *last_send_at.borrow();
            const INTERVAL_MS: f64 = 50.0;
            const DIR_EPS: f64 = 1.0;
            const SPEED_EPS: f64 = 0.01;
            const KEEPALIVE_MS: f64 = 200.0;

            if elapsed >= INTERVAL_MS {
                *last_send_at.borrow_mut() = now;
                *trailing_timer.borrow_mut() = None;

                // Dedup + keepalive check
                let wrapped = wrap360(dir);
                let since_emit = now - *last_actual_emit_at.borrow();
                let keepalive = since_emit >= KEEPALIVE_MS;
                let changed = {
                    let ld = *last_sent_dir.borrow();
                    let ls = *last_sent_speed.borrow();
                    match (ld, ls) {
                        (Some(d), Some(s)) => {
                            (wrapped - d).abs() >= DIR_EPS || (applied - s).abs() >= SPEED_EPS
                        }
                        _ => true,
                    }
                };
                if changed || keepalive {
                    *last_sent_dir.borrow_mut() = Some(wrapped);
                    *last_sent_speed.borrow_mut() = Some(applied);
                    *last_actual_emit_at.borrow_mut() = now;
                    ws_send(dir, applied);
                }
            }
            else {
                // Queue trailing flush
                let dir_c = dir;
                let applied_c = applied;
                let last_send_at2 = last_send_at.clone();
                let last_actual_emit_at2 = last_actual_emit_at.clone();
                let last_sent_dir2 = last_sent_dir.clone();
                let last_sent_speed2 = last_sent_speed.clone();
                let ws_send2 = ws_send.clone();
                if trailing_timer.borrow().is_none() {
                    let delay = ((INTERVAL_MS - elapsed).max(0.0)) as u32;
                    let timer = gloo_timers::callback::Timeout::new(delay, move || {
                        let t = performance_now();
                        *last_send_at2.borrow_mut() = t;
                        let wrapped = wrap360(dir_c);
                        let since_emit = t - *last_actual_emit_at2.borrow();
                        let keepalive = since_emit >= KEEPALIVE_MS;
                        let changed = {
                            let ld = *last_sent_dir2.borrow();
                            let ls = *last_sent_speed2.borrow();
                            match (ld, ls) {
                                (Some(d), Some(s)) => {
                                    (wrapped - d).abs() >= DIR_EPS
                                        || (applied_c - s).abs() >= SPEED_EPS
                                }
                                _ => true,
                            }
                        };
                        if changed || keepalive {
                            *last_sent_dir2.borrow_mut() = Some(wrapped);
                            *last_sent_speed2.borrow_mut() = Some(applied_c);
                            *last_actual_emit_at2.borrow_mut() = t;
                            ws_send2(dir_c, applied_c);
                        }
                    });
                    *trailing_timer.borrow_mut() = Some(timer);
                }
            }
        }
    };

    // on_pointer_up
    let on_pointer_up = {
        let js_state = js_state.clone();
        let knob_transform = knob_transform;
        let dir_text = dir_text;
        let speed_text = speed_text;
        let last_send_at = last_send_at.clone();
        let last_actual_emit_at = last_actual_emit_at.clone();
        let last_sent_dir = last_sent_dir.clone();
        let last_sent_speed = last_sent_speed.clone();
        let trailing_timer = trailing_timer.clone();
        let ws_send = ws_send.clone();

        move |ev: PointerEvent| {
            let mut state = js_state.borrow_mut();
            if state.pointer_id != Some(ev.pointer_id()) {
                return;
            }
            state.pointer_id = None;
            drop(state);

            // Remove active marker from body
            if let Some(body) = web_sys::window()
                .and_then(|w| w.document())
                .and_then(|d| d.body())
            {
                let _ = body.remove_attribute("data-stick");
            }

            // Reset visual
            knob_transform.set(String::from("translate(0px, 0px)"));
            dir_text.set(String::from("0\u{00b0}"));
            speed_text.set(String::from("0.00"));

            // Flush stop
            *trailing_timer.borrow_mut() = None;
            let now = performance_now();
            *last_send_at.borrow_mut() = now;
            *last_actual_emit_at.borrow_mut() = now;
            *last_sent_dir.borrow_mut() = None;
            *last_sent_speed.borrow_mut() = None;
            ws_send(0.0, 0.0);
        }
    };

    let on_pointer_up_cancel = on_pointer_up.clone();

    // Keepalive interval: fires every 50ms independent of pointer movement.
    // Ensures the firmware deadman watchdog (300ms) stays fed when the finger
    // is held perfectly still at max deflection and pointermove stops firing.
    {
        let js_state_ka = js_state.clone();
        let last_actual_emit_at_ka = last_actual_emit_at.clone();
        let last_sent_dir_ka = last_sent_dir.clone();
        let last_sent_speed_ka = last_sent_speed.clone();
        let ws_send_ka = ws_send.clone();
        gloo_timers::callback::Interval::new(50, move || {
            const KEEPALIVE_MS: f64 = 200.0;
            if js_state_ka.borrow().pointer_id.is_none() {
                return;
            }
            let now = performance_now();
            if now - *last_actual_emit_at_ka.borrow() < KEEPALIVE_MS {
                return;
            }
            let dir = last_sent_dir_ka.borrow().unwrap_or(0.0);
            let speed = last_sent_speed_ka.borrow().unwrap_or(0.0);
            *last_actual_emit_at_ka.borrow_mut() = now;
            ws_send_ka(dir, speed);
        })
        .forget();
    }

    view! {
        <main
            id="joystick-area"
            class="joystick-area"
            role="main"
            node_ref=area_ref
            on:pointerdown=on_pointer_down
            on:pointermove=on_pointer_move
            on:pointerup=on_pointer_up
            on:pointercancel=on_pointer_up_cancel
        >
            <div
                id="joystick"
                class="joystick"
                node_ref=joystick_ref
                aria-label="virtual joystick"
                role="application"
                style=move || format!(
                    "left: {}; top: {};",
                    stick_left.get(),
                    stick_top.get()
                )
            >
                <div class="joystick-readout" aria-live="polite">
                    <span id="js-dir-value">{move || dir_text.get()}</span>
                    <span class="readout-sep">"·"</span>
                    <span id="js-speed-value">{move || speed_text.get()}</span>
                </div>
                <div class="joystick-ring" aria-hidden="true"></div>
                <div
                    id="joystick-knob"
                    class="joystick-knob"
                    aria-hidden="true"
                    style=move || format!("transform: {};", knob_transform.get())
                ></div>
            </div>
            <p class="joystick-hint" aria-hidden="true">"tap anywhere to drive"</p>
        </main>
    }
}

fn performance_now() -> f64 {
    web_sys::window()
        .and_then(|w| w.performance())
        .map(|p| p.now())
        .unwrap_or(0.0)
}
