//! Root App component.
//!
//! Manages global signals, resolves the WS URL and control key, mounts the
//! WsClient, and composes top-level layout.

use std::rc::Rc;

use leptos::prelude::*;

use crate::{
    components::{
        joystick::{JoystickArea, ResponseCurve},
        settings_drawer::SettingsDrawer,
        top_strip::TopStrip,
    },
    storage,
    transport::{ConnState, I2CCommand, PowerCmd, WsClient},
};

/// Resolve the WS URL from URL fragment / query / localStorage / fallback.
fn resolve_ws_url(control_key: &str) -> String {
    let win = match web_sys::window() {
        Some(w) => w,
        None => return String::from("ws://owb.local/ws"),
    };
    let loc = win.location();

    // Resolve host: #host= > ?host= > localStorage > default
    let hash = loc.hash().unwrap_or_default();
    let hash_params = web_sys::UrlSearchParams::new_with_str(&hash.trim_start_matches('#')).ok();
    let query = loc.search().unwrap_or_default();
    let query_params = web_sys::UrlSearchParams::new_with_str(&query.trim_start_matches('?')).ok();

    // A host supplied by the *link* is attacker-controllable; one coming from
    // localStorage was entered deliberately via the settings drawer. Track
    // which, because the control key must never be handed to the former.
    let host_from_link = hash_params
        .as_ref()
        .and_then(|p| p.get("host"))
        .or_else(|| query_params.as_ref().and_then(|p| p.get("host")));
    let host_came_from_link = host_from_link.is_some();

    let host = host_from_link
        // Reject the old "owb.local" default if it was saved to storage by a
        // previous load — it was never a real mDNS name on this bot.
        .or_else(|| {
            storage::get(storage::KEY_HOST).filter(|h| !h.is_empty() && h != "ws://owb.local/ws")
        })
        .unwrap_or_else(|| {
            // Derive from the page's own origin so the PWA works out-of-the-box
            // when served directly from the bot at http://<bot-ip>/
            let proto = loc.protocol().unwrap_or_else(|_| String::from("http:"));
            let ws_proto = if proto.starts_with("https") {
                "wss:"
            }
            else {
                "ws:"
            };
            let page_host = loc.host().unwrap_or_else(|_| String::from("owb.local"));
            format!("{}//{}/ws", ws_proto, page_host)
        });

    // Mint a random session id
    let session_id = format!(
        "ui-{:08x}",
        (js_sys::Math::random() * f64::from(u32::MAX)) as u32
    );

    // Build URL with session + key params
    let url = web_sys::Url::new(&host)
        .unwrap_or_else(|_| web_sys::Url::new("ws://owb.local/ws").unwrap());

    // A link-supplied host is only trusted when it points back at the page's
    // own origin. Otherwise `https://bot/?host=ws://attacker/ws` would make the
    // PWA hand the saved control key to an arbitrary server — and, once
    // persisted, keep doing so on every subsequent load.
    let page_host = loc.host().unwrap_or_default();
    let trusted_host = !host_came_from_link || url.host() == page_host;

    if trusted_host {
        storage::set(storage::KEY_HOST, &host);
    }

    url.search_params().set("session", &session_id);
    if !control_key.is_empty() && trusted_host {
        url.search_params().set("key", control_key);
    }
    url.href()
}

/// Resolve control key: #key= > ?key= > localStorage.
fn resolve_control_key() -> String {
    let win = match web_sys::window() {
        Some(w) => w,
        None => return String::new(),
    };
    let loc = win.location();
    let hash = loc.hash().unwrap_or_default();
    let hash_params = web_sys::UrlSearchParams::new_with_str(hash.trim_start_matches('#')).ok();
    let query = loc.search().unwrap_or_default();
    let query_params = web_sys::UrlSearchParams::new_with_str(query.trim_start_matches('?')).ok();

    let from_hash = hash_params.as_ref().and_then(|p| p.get("key"));
    let from_query = query_params.as_ref().and_then(|p| p.get("key"));
    let supplied = from_hash.or(from_query);

    if let Some(key) = supplied {
        storage::set(storage::KEY_CONTROL_KEY, &key);
        // Strip from address bar
        if let (Ok(path), Ok(search)) = (loc.pathname(), loc.search()) {
            let _ = win.history().ok().and_then(|h| {
                h.replace_state_with_url(&wasm_bindgen::JsValue::NULL, "", Some(&(path + &search)))
                    .ok()
            });
        }
        return key;
    }
    storage::get(storage::KEY_CONTROL_KEY).unwrap_or_default()
}

fn load_theme() -> String {
    storage::get(storage::KEY_THEME).unwrap_or_else(|| String::from("dark"))
}

fn load_curve() -> ResponseCurve {
    storage::get(storage::KEY_CURVE)
        .as_deref()
        .map(ResponseCurve::from_str)
        .unwrap_or(ResponseCurve::Cubic)
}

#[component]
pub fn App() -> impl IntoView {
    let conn_state = RwSignal::new(ConnState::Connecting);
    let battery_label = RwSignal::new(String::from("--%"));
    let drawer_open = RwSignal::new(false);
    let camera_on = RwSignal::new(false);
    let theme = RwSignal::new(load_theme());
    let curve = RwSignal::new(load_curve());

    // Apply initial theme to body
    if let Some(body) = web_sys::window()
        .and_then(|w| w.document())
        .and_then(|d| d.body())
    {
        let _ = body.set_attribute("data-theme", &theme.get_untracked());
        let _ = body.set_attribute("data-ws-state", conn_state.get_untracked().as_str());
        let _ = body.set_attribute("data-ui-variant", "v1-alpha");
    }

    // Sync body data-theme when theme signal changes
    Effect::new(move |_| {
        let t = theme.get();
        if let Some(body) = web_sys::window()
            .and_then(|w| w.document())
            .and_then(|d| d.body())
        {
            let _ = body.set_attribute("data-theme", &t);
        }
    });

    // Sync body data-ws-state when conn_state changes
    Effect::new(move |_| {
        let s = conn_state.get().as_str();
        if let Some(body) = web_sys::window()
            .and_then(|w| w.document())
            .and_then(|d| d.body())
        {
            let _ = body.set_attribute("data-ws-state", s);
        }
    });

    // Resolve WS connection params
    let control_key = resolve_control_key();
    let ws_url = resolve_ws_url(&control_key);

    // Battery poll: fires every 5s on open connection
    let battery_label_for_ws = battery_label;
    let conn_state_for_open = conn_state;

    let ws = Rc::new(WsClient::new(
        ws_url.clone(),
        conn_state,
        Box::new(move |text: String| {
            // Parse battery telemetry
            if let Ok(val) = serde_json::from_str::<serde_json::Value>(&text) {
                if val.get("type").and_then(|v| v.as_str()) == Some("power") {
                    if let Some(soc) = val
                        .get("data")
                        .and_then(|d| d.get("soc_percent"))
                        .and_then(|v| v.as_f64())
                    {
                        battery_label_for_ws.set(format!("{}%", soc.round() as i32));
                    }
                }
            }
        }),
    ));

    // Send enable + first battery poll on open
    {
        let ws_for_open = ws.clone();
        // Auto-enable fires on the FIRST open only. Re-enabling on every
        // reconnect meant a network blip brought the motors back live with no
        // operator action — and the robot may have moved or be in someone's
        // hands by then. After a drop the operator must explicitly re-enable.
        let enabled_once = std::cell::Cell::new(false);
        Effect::new(move |prev_state: Option<ConnState>| {
            let state = conn_state_for_open.get();
            if state == ConnState::Open && prev_state != Some(ConnState::Open) {
                if !enabled_once.get() {
                    enabled_once.set(true);
                    ws_for_open.send(&I2CCommand::enable());
                }
                ws_for_open.send(&I2CCommand::read_power());
            }
            state
        });
    }

    // Auto-open settings drawer on auth-error so operator can fix the key
    {
        let drawer_open_for_auth = drawer_open;
        Effect::new(move |_| {
            if conn_state.get() == ConnState::AuthError {
                drawer_open_for_auth.set(true);
            }
        });
    }

    // Battery poll every 5s
    {
        let ws_for_poll = ws.clone();
        let conn_state_for_poll = conn_state;
        let _timer = gloo_timers::callback::Interval::new(5_000, move || {
            if conn_state_for_poll.get_untracked() == ConnState::Open {
                ws_for_poll.send(&I2CCommand::read_power());
            }
        });
        // Leak the interval — it lives for the app lifetime.
        _timer.forget();
    }

    ws.connect();

    let ws_for_joystick = ws.clone();
    let ws_for_drawer = ws.clone();
    let ws_for_camera = ws.clone();
    let ws_for_estop = ws.clone();

    let on_reconnect = {
        let ws = ws.clone();
        Box::new(move |_host: String, _key: String| {
            // Reconnect is handled by WsClient's exponential backoff on close;
            // close existing connection to trigger it immediately.
            ws.close();
        })
    };

    let backdrop_data_open = move || if drawer_open.get() { "1" } else { "0" };
    let backdrop_on_click = move |_| drawer_open.set(false);
    // iOS Safari does not fire `click` on plain <div>s. Using a <button> with
    // role="presentation" ensures tap events fire on all browsers.
    let backdrop_on_touch = move |_| drawer_open.set(false);

    view! {
        <TopStrip
            ws=ws_for_estop
            conn_state=conn_state
            battery_label=battery_label
            drawer_open=drawer_open
        />
        <JoystickArea
            ws=ws_for_joystick
            conn_state=conn_state
            curve=curve
        />
        <SettingsDrawer
            open=drawer_open
            ws=ws_for_drawer
            curve=curve
            theme=theme
            on_reconnect=on_reconnect
        />
        <button
            id="camera-fab"
            aria-label="Toggle camera feed"
            title="Toggle camera feed"
            disabled=move || conn_state.get() != ConnState::Open
            data-active=move || if camera_on.get() { "1" } else { "0" }
            on:click=move |_| {
                ws_for_camera.send(&PowerCmd::camera_toggle());
                camera_on.update(|v| *v = !*v);
            }
            style="\
                position:fixed;bottom:1.5rem;right:1.25rem;\
                width:3rem;height:3rem;border-radius:50%;\
                background:var(--panel,#161d26);border:1px solid var(--border,#24303c);\
                color:var(--muted,#8fa0b0);font-size:1.25rem;\
                display:flex;align-items:center;justify-content:center;\
                cursor:pointer;z-index:50;transition:color .15s,background .15s,border-color .15s;\
            "
        >
            "\u{1F4F7}"
        </button>
        <button
            id="drawer-backdrop"
            class="drawer-backdrop"
            data-open=backdrop_data_open
            aria-hidden="true"
            tabindex="-1"
            on:click=backdrop_on_click
            on:touchstart=backdrop_on_touch
        ></button>
    }
}
