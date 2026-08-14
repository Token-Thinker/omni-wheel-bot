//! Settings slide-in drawer.
//!
//! Sections (in order):
//!   1. Connection — bot address + control key
//!   2. Response curve
//!   3. Wheel trim
//!   4. Heading hold
//!   5. Theme toggle
//!   6. Controls reference

use std::rc::Rc;

use leptos::prelude::*;

use crate::{
    components::joystick::ResponseCurve,
    storage,
    transport::{I2CCommand, WsClient},
};

#[derive(serde::Serialize, serde::Deserialize, Clone)]
struct TrimSnapshot {
    w1: f32,
    w2: f32,
    w3: f32,
}

#[derive(serde::Serialize, serde::Deserialize, Clone)]
struct HhSnapshot {
    enable: bool,
    kp: f32,
}

#[component]
pub fn SettingsDrawer(
    open: RwSignal<bool>,
    ws: Rc<WsClient>,
    curve: RwSignal<ResponseCurve>,
    theme: RwSignal<String>,
    /// Callback fired when the user submits a new host + key
    on_reconnect: Box<dyn Fn(String, String)>,
) -> impl IntoView {
    // --- Connection section ---
    let host_init = storage::get(storage::KEY_HOST)
        .filter(|h| !h.is_empty() && h != "ws://owb.local/ws")
        .unwrap_or_default();
    let key_init = storage::get(storage::KEY_CONTROL_KEY).unwrap_or_default();

    let host_val = RwSignal::new(host_init);
    let key_val = RwSignal::new(key_init);

    // --- Trim sliders ---
    let trim_snap: Option<TrimSnapshot> = storage::get_json(storage::KEY_TRIM);
    let w1_init = trim_snap.as_ref().map(|t| t.w1).unwrap_or(1.0_f32);
    let w2_init = trim_snap.as_ref().map(|t| t.w2).unwrap_or(1.0_f32);
    let w3_init = trim_snap.as_ref().map(|t| t.w3).unwrap_or(1.0_f32);

    let trim_w1 = RwSignal::new(w1_init);
    let trim_w2 = RwSignal::new(w2_init);
    let trim_w3 = RwSignal::new(w3_init);

    // --- Heading hold ---
    let hh_snap: Option<HhSnapshot> = storage::get_json(storage::KEY_HH);
    let hh_enable_init = hh_snap.as_ref().map(|h| h.enable).unwrap_or(false);
    let hh_kp_init = hh_snap.as_ref().map(|h| h.kp).unwrap_or(0.5_f32);

    let hh_enable = RwSignal::new(hh_enable_init);
    let hh_kp = RwSignal::new(hh_kp_init);

    // --- Apply trim ---
    let ws_trim = ws.clone();
    let on_apply_trim = move |_| {
        let w1 = trim_w1.get();
        let w2 = trim_w2.get();
        let w3 = trim_w3.get();
        storage::set_json(storage::KEY_TRIM, &TrimSnapshot { w1, w2, w3 });
        ws_trim.send(&I2CCommand::wheel_trim(w1, w2, w3));
    };

    // --- Apply heading hold ---
    let ws_hh = ws.clone();
    let on_apply_hh = move |_| {
        let enable = hh_enable.get();
        let kp = hh_kp.get();
        storage::set_json(storage::KEY_HH, &HhSnapshot { enable, kp });
        ws_hh.send(&I2CCommand::heading_hold(enable, kp));
    };

    // --- Theme toggle ---
    let theme_toggle = {
        let theme = theme;
        move |_| {
            let next = if theme.get() == "light" {
                "dark"
            }
            else {
                "light"
            };
            theme.set(next.to_string());
            storage::set(storage::KEY_THEME, next);
            if let Some(body) = web_sys::window()
                .and_then(|w| w.document())
                .and_then(|d| d.body())
            {
                let _ = body.set_attribute("data-theme", next);
            }
        }
    };

    // --- Connect button ---
    let on_connect_click = {
        let host_val = host_val;
        let key_val = key_val;
        let on_reconnect = on_reconnect;
        move |_| {
            let host = host_val.get();
            let key = key_val.get();
            storage::set(storage::KEY_HOST, &host);
            storage::set(storage::KEY_CONTROL_KEY, &key);
            on_reconnect(host, key);
        }
    };

    // --- Close button ---
    let on_close = move |_| open.set(false);

    let drawer_data_open = move || if open.get() { "1" } else { "0" };

    view! {
        <aside
            id="settings-drawer"
            class="drawer"
            data-open=drawer_data_open
            aria-hidden=move || if open.get() { "false" } else { "true" }
            tabindex="-1"
        >
            <header class="drawer-header">
                <h2 class="drawer-title">"Settings"</h2>
                <button
                    id="settings-close"
                    class="icon-btn"
                    type="button"
                    aria-label="close settings"
                    title="Close"
                    on:click=on_close
                >
                    <svg
                        viewBox="0 0 24 24"
                        fill="none"
                        stroke="currentColor"
                        stroke-width="1.8"
                        stroke-linecap="round"
                        aria-hidden="true"
                    >
                        <line x1="6" y1="6" x2="18" y2="18"></line>
                        <line x1="18" y1="6" x2="6" y2="18"></line>
                    </svg>
                </button>
            </header>

            // 1. Connection
            <section class="drawer-section">
                <h3 class="drawer-h">"Connection"</h3>
                <div class="slider-row" style="display:flex; flex-direction:column; gap:0.4rem;">
                    <input
                        type="text"
                        class="select"
                        placeholder="ws://owb.local/ws"
                        prop:value=move || host_val.get()
                        on:input=move |ev| {
                            host_val.set(event_target_value(&ev));
                        }
                        aria-label="Bot WebSocket address"
                    />
                    <input
                        type="password"
                        class="select"
                        placeholder="Control key"
                        prop:value=move || key_val.get()
                        on:input=move |ev| {
                            key_val.set(event_target_value(&ev));
                        }
                        aria-label="Control key"
                    />
                    <button
                        class="btn"
                        type="button"
                        on:click=on_connect_click
                    >
                        "Connect"
                    </button>
                </div>
            </section>

            // 2. Response curve
            <section class="drawer-section">
                <h3 class="drawer-h">"Response curve"</h3>
                <select
                    id="curve-select"
                    class="select"
                    aria-label="response curve"
                    on:change=move |ev| {
                        let v = event_target_value(&ev);
                        let c = ResponseCurve::from_str(&v);
                        curve.set(c);
                        storage::set(storage::KEY_CURVE, c.as_str());
                    }
                >
                    <option
                        value="linear"
                        selected=move || curve.get() == ResponseCurve::Linear
                    >"Linear"</option>
                    <option
                        value="quadratic"
                        selected=move || curve.get() == ResponseCurve::Quadratic
                    >"Quadratic"</option>
                    <option
                        value="cubic"
                        selected=move || curve.get() == ResponseCurve::Cubic
                    >"Cubic"</option>
                </select>
            </section>

            // 3. Wheel trim
            <section class="drawer-section">
                <h3 class="drawer-h">"Wheel trim"</h3>
                <label class="slider-row">
                    <span class="lbl">"W1"</span>
                    <input
                        id="trim-w1"
                        class="slider"
                        type="range"
                        min="0.5"
                        max="1.5"
                        step="0.01"
                        prop:value=move || trim_w1.get().to_string()
                        on:input=move |ev| {
                            if let Ok(v) = event_target_value(&ev).parse::<f32>() {
                                trim_w1.set(v);
                            }
                        }
                    />
                    <span id="trim-w1-val" class="val">
                        {move || format!("{:.2}", trim_w1.get())}
                    </span>
                </label>
                <label class="slider-row">
                    <span class="lbl">"W2"</span>
                    <input
                        id="trim-w2"
                        class="slider"
                        type="range"
                        min="0.5"
                        max="1.5"
                        step="0.01"
                        prop:value=move || trim_w2.get().to_string()
                        on:input=move |ev| {
                            if let Ok(v) = event_target_value(&ev).parse::<f32>() {
                                trim_w2.set(v);
                            }
                        }
                    />
                    <span id="trim-w2-val" class="val">
                        {move || format!("{:.2}", trim_w2.get())}
                    </span>
                </label>
                <label class="slider-row card-disabled">
                    <span class="lbl">"W3"</span>
                    <input
                        id="trim-w3"
                        class="slider"
                        type="range"
                        min="0.5"
                        max="1.5"
                        step="0.01"
                        prop:value=move || trim_w3.get().to_string()
                        disabled=true
                    />
                    <span id="trim-w3-val" class="val">
                        {move || format!("{:.2}", trim_w3.get())}
                    </span>
                </label>
                <button
                    id="trim-apply"
                    class="btn"
                    type="button"
                    on:click=on_apply_trim
                >"Apply trim"</button>
            </section>

            // 4. Heading hold
            <section class="drawer-section">
                <h3 class="drawer-h">"Heading hold"</h3>
                <label class="slider-row">
                    <span class="lbl">"enable"</span>
                    <input
                        id="hh-enable"
                        class="toggle"
                        type="checkbox"
                        prop:checked=move || hh_enable.get()
                        on:change=move |ev| {
                            use wasm_bindgen::JsCast;
                            if let Some(input) = ev.target().and_then(|t| t.dyn_into::<web_sys::HtmlInputElement>().ok()) {
                                hh_enable.set(input.checked());
                            }
                        }
                    />
                </label>
                <label class="slider-row">
                    <span class="lbl">"Kp"</span>
                    <input
                        id="hh-kp"
                        class="slider"
                        type="range"
                        min="0"
                        max="2"
                        step="0.05"
                        prop:value=move || hh_kp.get().to_string()
                        on:input=move |ev| {
                            if let Ok(v) = event_target_value(&ev).parse::<f32>() {
                                hh_kp.set(v);
                            }
                        }
                    />
                    <span id="hh-kp-val" class="val">
                        {move || format!("{:.2}", hh_kp.get())}
                    </span>
                </label>
                <button
                    id="hh-apply"
                    class="btn"
                    type="button"
                    on:click=on_apply_hh
                >"Apply heading hold"</button>
            </section>

            // 5. Theme
            <section class="drawer-section">
                <h3 class="drawer-h">"Theme"</h3>
                <button
                    id="theme-toggle"
                    class="btn"
                    type="button"
                    on:click=theme_toggle
                >"Toggle light/dark"</button>
            </section>

            // 6. Controls reference
            <section class="drawer-section drawer-tooltip" aria-label="controls reference">
                <h3 class="drawer-h">"Controls"</h3>
                <ul class="tip-list">
                    <li>
                        <strong>"Tap and drag"</strong>
                        " anywhere — joystick spawns at your finger, deflection in any direction sends a drive command."
                    </li>
                    <li>
                        <strong>"Release"</strong>
                        " — joystick disappears, robot stops."
                    </li>
                    <li>
                        <strong>"Speed"</strong>
                        " grows from the spawn point outward. The outer ring marks 100%; the inner mark is just visual."
                    </li>
                    <li>
                        <strong>"Long-press a toggle"</strong>
                        " in this drawer to switch its state on or off."
                    </li>
                    <li>
                        <strong>"Green dot"</strong>
                        " in the top-left = WebSocket connected. Red = closed/error. Grey = connecting."
                    </li>
                    <li>
                        "You must reach the connected state before the controller will move the robot."
                    </li>
                </ul>
            </section>
        </aside>
    }
}
