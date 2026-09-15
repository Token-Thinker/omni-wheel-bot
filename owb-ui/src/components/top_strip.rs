//! Top ambient strip: connection indicator + battery readout (left), E-stop and
//! gear button (right).

use std::rc::Rc;

use leptos::prelude::*;

use crate::transport::{ConnState, PowerCmd, WsClient};

#[component]
pub fn TopStrip(
    ws: Rc<WsClient>,
    conn_state: RwSignal<ConnState>,
    battery_label: RwSignal<String>,
    drawer_open: RwSignal<bool>,
) -> impl IntoView {
    // E-stop latches motors off in firmware (`executor.rs` PowerCommand::Estop
    // clears `enabled` and zeroes motion), verified on hardware: a normal
    // translate command after an E-stop produces zero encoder movement until an
    // explicit re-enable. Deliberately always enabled — an operator must be
    // able to hit it even when the connection indicator looks unhealthy.
    let on_estop = {
        let ws = ws.clone();
        move |_| {
            ws.send(&PowerCmd::estop());
        }
    };
    let conn_state_str = move || conn_state.get().as_str();
    let conn_label = move || {
        if conn_state.get() == ConnState::AuthError {
            "Auth required"
        }
        else {
            ""
        }
    };

    view! {
        <header id="top-strip" class="strip" role="banner">
            <div class="strip-cluster strip-left">
                <span
                    class="indicator"
                    id="conn-indicator"
                    data-state=conn_state_str
                    aria-label="connection"
                >
                    <span class="indicator-dot"></span>
                </span>
                <span class="indicator-label" id="conn-label" aria-live="assertive">
                    {conn_label}
                </span>
                <span class="ambient" id="battery-readout" aria-label="battery">
                    <span class="ambient-value" id="battery-value">
                        {move || battery_label.get()}
                    </span>
                </span>
            </div>
            <div class="strip-cluster strip-right">
                <button
                    id="estop-btn"
                    class="estop-btn"
                    type="button"
                    aria-label="emergency stop"
                    title="Emergency stop — cuts motors until re-enabled"
                    on:click=on_estop
                >
                    "STOP"
                </button>
                <button
                    id="settings-toggle"
                    class="icon-btn"
                    type="button"
                    aria-label="open settings"
                    aria-expanded=move || if drawer_open.get() { "true" } else { "false" }
                    title="Settings"
                    on:click=move |_| drawer_open.set(true)
                >
                    <svg
                        class="gear-glyph"
                        viewBox="0 0 24 24"
                        fill="none"
                        stroke="currentColor"
                        stroke-width="1.6"
                        stroke-linecap="round"
                        stroke-linejoin="round"
                        aria-hidden="true"
                    >
                        <circle cx="12" cy="12" r="3"></circle>
                        <path d="M19.4 15a1.65 1.65 0 0 0 .33 1.82l.06.06a2 2 0 1 1-2.83 2.83l-.06-.06a1.65 1.65 0 0 0-1.82-.33 1.65 1.65 0 0 0-1 1.51V21a2 2 0 0 1-4 0v-.09a1.65 1.65 0 0 0-1-1.51 1.65 1.65 0 0 0-1.82.33l-.06.06a2 2 0 1 1-2.83-2.83l.06-.06a1.65 1.65 0 0 0 .33-1.82 1.65 1.65 0 0 0-1.51-1H3a2 2 0 0 1 0-4h.09a1.65 1.65 0 0 0 1.51-1 1.65 1.65 0 0 0-.33-1.82l-.06-.06a2 2 0 1 1 2.83-2.83l.06.06a1.65 1.65 0 0 0 1.82.33h0a1.65 1.65 0 0 0 1-1.51V3a2 2 0 0 1 4 0v.09a1.65 1.65 0 0 0 1 1.51h0a1.65 1.65 0 0 0 1.82-.33l.06-.06a2 2 0 1 1 2.83 2.83l-.06.06a1.65 1.65 0 0 0-.33 1.82v0a1.65 1.65 0 0 0 1.51 1H21a2 2 0 0 1 0 4h-.09a1.65 1.65 0 0 0-1.51 1z"></path>
                    </svg>
                </button>
            </div>
        </header>
    }
}
