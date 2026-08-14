//! WebSocket client with exponential-backoff reconnect.
//!
//! Port of `ws.ts`. Key behaviors:
//!   - Subprotocol `"messages"` (firmware refuses connections without it)
//!   - Exponential backoff: base 1000ms, max 15000ms, doubling
//!   - `ConnState` enum drives the connection indicator dot
//!   - `send()` serializes to JSON and places the frame on the socket

use leptos::prelude::*;
use owb_msg::legacy::{I2CCommand as WireI2CCommand, PowerCommand, SystemCommand};
use serde::Serialize;
use wasm_bindgen::{prelude::*, JsCast};
use web_sys::{CloseEvent, ErrorEvent, MessageEvent, WebSocket};

const RECONNECT_BASE_MS: u32 = 1000;
const RECONNECT_MAX_MS: u32 = 15000;

/// Connection state; mirrors the CSS `data-state` values in style.css.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ConnState {
    Connecting,
    Open,
    Closed,
    Error,
    AuthError,
}

impl ConnState {
    pub fn as_str(self) -> &'static str {
        match self {
            ConnState::Connecting => "connecting",
            ConnState::Open => "open",
            ConnState::Closed => "closed",
            ConnState::Error => "error",
            ConnState::AuthError => "auth-error",
        }
    }
}

/// Wire command — canonical `SystemCommand` from `owb-msg`.
pub type OwbCommand = SystemCommand;

/// Convenience constructors for I2C/drive commands.
///
/// Each method returns the canonical `OwbCommand` (`SystemCommand`) so
/// callers can pass the result directly to `WsClient::send()`.
pub struct I2CCommand;

impl I2CCommand {
    pub fn translate(
        direction_deg: f32,
        speed_norm: f32,
    ) -> OwbCommand {
        SystemCommand::I(WireI2CCommand::T {
            direction_deg,
            speed_norm,
        })
    }

    pub fn enable() -> OwbCommand { SystemCommand::I(WireI2CCommand::Enable) }

    pub fn disable() -> OwbCommand { SystemCommand::I(WireI2CCommand::Disable) }

    pub fn read_power() -> OwbCommand { SystemCommand::I(WireI2CCommand::ReadPower) }

    pub fn wheel_trim(
        w1: f32,
        w2: f32,
        w3: f32,
    ) -> OwbCommand {
        SystemCommand::I(WireI2CCommand::WheelTrim { w1, w2, w3 })
    }

    pub fn heading_hold(
        enable: bool,
        kp: f32,
    ) -> OwbCommand {
        SystemCommand::I(WireI2CCommand::HeadingHold { enable, kp })
    }
}

/// Convenience constructors for power/control commands.
pub struct PowerCmd;

impl PowerCmd {
    pub fn camera_toggle() -> OwbCommand { SystemCommand::P(PowerCommand::CameraToggle) }

    pub fn motors_toggle() -> OwbCommand { SystemCommand::P(PowerCommand::MotorsToggle) }

    pub fn estop() -> OwbCommand { SystemCommand::P(PowerCommand::Estop) }
}

struct Inner {
    ws: Option<WebSocket>,
    reconnect_delay: u32,
    _reconnect_timer: Option<gloo_timers::callback::Timeout>,
}

/// WebSocket client. Constructed once in `App` and shared via signals.
pub struct WsClient {
    url: String,
    conn_state: RwSignal<ConnState>,
    on_message: std::rc::Rc<dyn Fn(String)>,
    inner: std::rc::Rc<std::cell::RefCell<Inner>>,
}

impl WsClient {
    pub fn new(
        url: String,
        conn_state: RwSignal<ConnState>,
        on_message: Box<dyn Fn(String)>,
    ) -> Self {
        WsClient {
            url,
            conn_state,
            on_message: std::rc::Rc::from(on_message),
            inner: std::rc::Rc::new(std::cell::RefCell::new(Inner {
                ws: None,
                reconnect_delay: RECONNECT_BASE_MS,
                _reconnect_timer: None,
            })),
        }
    }

    pub fn connect(&self) {
        let ws_opt = self.inner.borrow().ws.as_ref().map(|ws| ws.ready_state());
        if ws_opt == Some(WebSocket::OPEN) || ws_opt == Some(WebSocket::CONNECTING) {
            return;
        }
        self.conn_state.set(ConnState::Connecting);

        let ws = match WebSocket::new_with_str(&self.url, "messages") {
            Ok(ws) => ws,
            Err(_) => {
                self.schedule_reconnect();
                return;
            }
        };

        let conn_state = self.conn_state;
        let inner_clone = self.inner.clone();
        let url_clone = self.url.clone();

        // onopen
        {
            let conn_state = conn_state;
            let inner = inner_clone.clone();
            let on_open_cb = Closure::<dyn FnMut()>::new(move || {
                conn_state.set(ConnState::Open);
                inner.borrow_mut().reconnect_delay = RECONNECT_BASE_MS;
            });
            ws.set_onopen(Some(on_open_cb.as_ref().unchecked_ref()));
            on_open_cb.forget();
        }

        // onclose
        {
            let conn_state = conn_state;
            let inner = inner_clone.clone();
            let url = url_clone.clone();
            let on_close_cb = Closure::<dyn FnMut(CloseEvent)>::new(move |ev: CloseEvent| {
                if ev.code() == 1008 {
                    conn_state.set(ConnState::AuthError);
                    return;
                }
                conn_state.set(ConnState::Closed);
                // Schedule reconnect
                let delay = {
                    let mut guard = inner.borrow_mut();
                    let d = guard.reconnect_delay;
                    guard.reconnect_delay = (d * 2).min(RECONNECT_MAX_MS);
                    d
                };
                let inner2 = inner.clone();
                let url2 = url.clone();
                let conn_state2 = conn_state;
                let timer = gloo_timers::callback::Timeout::new(delay, move || {
                    inner2.borrow_mut()._reconnect_timer = None;
                    // Trigger a new connect by constructing a temporary client.
                    // Since we can't call self.connect() from here, we
                    // replicate the minimal connect logic.
                    reconnect_ws(&url2, conn_state2, &inner2);
                });
                inner.borrow_mut()._reconnect_timer = Some(timer);
            });
            ws.set_onclose(Some(on_close_cb.as_ref().unchecked_ref()));
            on_close_cb.forget();
        }

        // onerror
        {
            let conn_state = conn_state;
            let on_error_cb = Closure::<dyn FnMut(ErrorEvent)>::new(move |_: ErrorEvent| {
                conn_state.set(ConnState::Error);
            });
            ws.set_onerror(Some(on_error_cb.as_ref().unchecked_ref()));
            on_error_cb.forget();
        }

        // onmessage
        {
            let on_msg = self.on_message.clone();
            let on_message_cb = Closure::<dyn FnMut(MessageEvent)>::new(move |ev: MessageEvent| {
                if let Some(txt) = ev.data().as_string() {
                    on_msg(txt);
                }
            });
            ws.set_onmessage(Some(on_message_cb.as_ref().unchecked_ref()));
            on_message_cb.forget();
        }

        self.inner.borrow_mut().ws = Some(ws);
    }

    pub fn is_open(&self) -> bool {
        self.inner
            .borrow()
            .ws
            .as_ref()
            .map(|ws| ws.ready_state() == WebSocket::OPEN)
            .unwrap_or(false)
    }

    /// Serialize `cmd` to JSON and send it. Returns `true` if the frame was placed on the socket.
    pub fn send<T: Serialize>(
        &self,
        cmd: &T,
    ) -> bool {
        if !self.is_open() {
            return false;
        }
        match serde_json::to_string(cmd) {
            Ok(json) => {
                if let Some(ws) = self.inner.borrow().ws.as_ref() {
                    ws.send_with_str(&json).is_ok()
                }
                else {
                    false
                }
            }
            Err(_) => false,
        }
    }

    pub fn close(&self) {
        if let Some(ws) = self.inner.borrow().ws.as_ref() {
            let _ = ws.close();
        }
    }

    fn schedule_reconnect(&self) {
        let delay = {
            let mut guard = self.inner.borrow_mut();
            let d = guard.reconnect_delay;
            guard.reconnect_delay = (d * 2).min(RECONNECT_MAX_MS);
            d
        };
        let inner = self.inner.clone();
        let url = self.url.clone();
        let conn_state = self.conn_state;
        let timer = gloo_timers::callback::Timeout::new(delay, move || {
            inner.borrow_mut()._reconnect_timer = None;
            reconnect_ws(&url, conn_state, &inner);
        });
        self.inner.borrow_mut()._reconnect_timer = Some(timer);
    }
}

/// Minimal reconnect helper used from timeout closures that can't borrow WsClient.
fn reconnect_ws(
    url: &str,
    conn_state: RwSignal<ConnState>,
    inner: &std::rc::Rc<std::cell::RefCell<Inner>>,
) {
    conn_state.set(ConnState::Connecting);
    let ws = match WebSocket::new_with_str(url, "messages") {
        Ok(ws) => ws,
        Err(_) => {
            let delay = {
                let mut guard = inner.borrow_mut();
                let d = guard.reconnect_delay;
                guard.reconnect_delay = (d * 2).min(RECONNECT_MAX_MS);
                d
            };
            let inner2 = inner.clone();
            let url2 = url.to_string();
            let timer = gloo_timers::callback::Timeout::new(delay, move || {
                reconnect_ws(&url2, conn_state, &inner2);
            });
            inner.borrow_mut()._reconnect_timer = Some(timer);
            return;
        }
    };

    let inner_clone = inner.clone();
    let url_clone = url.to_string();

    {
        let conn_state = conn_state;
        let inner = inner_clone.clone();
        let on_open_cb = Closure::<dyn FnMut()>::new(move || {
            conn_state.set(ConnState::Open);
            inner.borrow_mut().reconnect_delay = RECONNECT_BASE_MS;
        });
        ws.set_onopen(Some(on_open_cb.as_ref().unchecked_ref()));
        on_open_cb.forget();
    }

    {
        let conn_state = conn_state;
        let inner = inner_clone.clone();
        let url = url_clone.clone();
        let on_close_cb = Closure::<dyn FnMut(CloseEvent)>::new(move |ev: CloseEvent| {
            if ev.code() == 1008 {
                conn_state.set(ConnState::AuthError);
                return;
            }
            conn_state.set(ConnState::Closed);
            let delay = {
                let mut guard = inner.borrow_mut();
                let d = guard.reconnect_delay;
                guard.reconnect_delay = (d * 2).min(RECONNECT_MAX_MS);
                d
            };
            let inner2 = inner.clone();
            let url2 = url.clone();
            let timer = gloo_timers::callback::Timeout::new(delay, move || {
                inner2.borrow_mut()._reconnect_timer = None;
                reconnect_ws(&url2, conn_state, &inner2);
            });
            inner.borrow_mut()._reconnect_timer = Some(timer);
        });
        ws.set_onclose(Some(on_close_cb.as_ref().unchecked_ref()));
        on_close_cb.forget();
    }

    {
        let conn_state = conn_state;
        let on_error_cb = Closure::<dyn FnMut(ErrorEvent)>::new(move |_: ErrorEvent| {
            conn_state.set(ConnState::Error);
        });
        ws.set_onerror(Some(on_error_cb.as_ref().unchecked_ref()));
        on_error_cb.forget();
    }

    inner.borrow_mut().ws = Some(ws);
}
