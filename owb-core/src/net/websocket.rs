//! WebSocket frame handling and connection upgrade extraction.

extern crate alloc;

use alloc::{string::String, vec};
use core::sync::atomic::{AtomicBool, Ordering};

use embassy_time::{Duration, Timer};
use picoserve::{
    extract::FromRequest,
    futures::Either,
    io::embedded_io_async as embedded_aio,
    request::{RequestBody, RequestParts},
    response::ws::{
        Message,
        ReadMessageError,
        SocketRx,
        SocketTx,
        WebSocketCallback,
        WebSocketUpgrade,
    },
    url_encoded::deserialize_form,
};
use serde::Deserialize;

use crate::{
    ctrl::{sysctrl::I2CCommand, SystemCommand},
    net::{
        gateway::{dispatch_binary_command, dispatch_text_command},
        protocol::{parse_command_from_slice, parse_command_from_str, CommandParseError},
        session::{now_millis, session_is_expired, touch_session, SessionManager},
    },
    node::command_bus::{command_bus_metrics_snapshot, CommandEnvelope, COMMAND_BUS},
};

const SESSION_WATCHDOG_INTERVAL: Duration = Duration::from_secs(5);

/// Developer-telemetry tier. When enabled at runtime (via a
/// `{"type":"dev_telemetry","on":true}` control frame), every session-watchdog
/// tick also pushes a command-bus metrics frame to the client. Off by default;
/// purely additive to the operator control surface.
static DEV_TELEMETRY: AtomicBool = AtomicBool::new(false);

pub struct WebSocket {
    session_id: String,
    lease_id: u64,
    accepted: bool,
    authorized: bool,
}

impl WebSocket {
    pub fn new(
        session_id: String,
        lease_id: u64,
        accepted: bool,
        authorized: bool,
    ) -> Self {
        Self {
            session_id,
            lease_id,
            accepted,
            authorized,
        }
    }
}

fn try_queue_board_command(cmd: SystemCommand) {
    // Indicator updates are helpful feedback, but they must not block socket handling.
    let _ = COMMAND_BUS.try_send(CommandEnvelope {
        cmd,
        expects_reply: false,
        received_ms: now_millis(),
    });
}

async fn send_board_command(cmd: SystemCommand) {
    // Safety commands use the reliable channel send so a full queue cannot drop them.
    COMMAND_BUS
        .send(CommandEnvelope {
            cmd,
            expects_reply: false,
            received_ms: now_millis(),
        })
        .await;
}

fn queue_indicator(
    index: u8,
    on: bool,
) {
    try_queue_board_command(SystemCommand::I(I2CCommand::IndicatorLed { index, on }));
}

fn queue_client_connected() {
    // Index 1 is the board Wi-Fi/client indicator in the TCA9555 map.
    queue_indicator(1, true);
}

async fn queue_low_power_disconnect() {
    // Closing the controlling socket should leave motors and IMU in the simple low-power state.
    send_board_command(SystemCommand::I(I2CCommand::Disable)).await;
    queue_indicator(1, false);
    queue_indicator(2, false);
}

async fn touch_or_close(
    session_id: &str,
    lease_id: u64,
) -> bool {
    // A stale socket must stop before it can dispatch another motor command.
    touch_session(session_id, lease_id).await
}

/// Handles incoming WebSocket connections.
impl WebSocketCallback for WebSocket {
    async fn run<Reader, Writer>(
        self,
        mut rx: SocketRx<Reader>,
        mut tx: SocketTx<Writer>,
    ) -> Result<(), Writer::Error>
    where
        Reader: embedded_aio::Read,
        Writer: embedded_aio::Write<Error = Reader::Error>,
    {
        let session_id = self.session_id;
        let lease_id = self.lease_id;
        let mut buffer = vec![0; 512];

        if !self.authorized {
            // Reaching this socket is not authorization: a valid control secret
            // is required before any command is read.
            tx.send_text(r#"{"type":"unauthorized","status":"denied"}"#)
                .await?;
            return tx.close(Some((1008, "Unauthorized"))).await;
        }

        if !self.accepted {
            tx.send_text(r#"{"type":"lockout","status":"busy"}"#)
                .await?;
            return tx.close(Some((1008, "Controller already connected"))).await;
        }

        queue_client_connected();
        tx.send_text("Connected").await?;

        let close_reason = loop {
            match rx
                .next_message(
                    buffer.as_mut_slice(),
                    Timer::after(SESSION_WATCHDOG_INTERVAL),
                )
                .await
            {
                Ok(Either::Second(_)) => {
                    if session_is_expired(&session_id, lease_id).await {
                        tracing::info!(session_id = %session_id, "websocket session expired");
                        break Some((1001, "Session expired"));
                    }

                    if DEV_TELEMETRY.load(Ordering::Relaxed) {
                        tx.send_text(&dev_telemetry_frame()).await?;
                    }

                    continue;
                }
                Err(err) => {
                    // Dev-telemetry is a per-session tier; clear it so the next
                    // client does not inherit this session's telemetry-on state.
                    DEV_TELEMETRY.store(false, Ordering::Relaxed);
                    let removed = SessionManager::remove_session(&session_id, lease_id).await;
                    if removed {
                        queue_low_power_disconnect().await;
                    }
                    return Err(err);
                }
                Ok(Either::First(Err(error))) => {
                    tracing::error!(?error, "websocket error");
                    let code = match error {
                        ReadMessageError::TextIsNotUtf8 => 1007,
                        ReadMessageError::ReservedOpcode(_) => 1003,
                        ReadMessageError::ReadFrameError(_)
                        | ReadMessageError::UnexpectedMessageStart
                        | ReadMessageError::MessageStartsWithContinuation => 1002,
                    };
                    break Some((code, "Websocket Error"));
                }
                Ok(Either::First(Ok(Message::Pong(_)))) => {
                    if !touch_or_close(&session_id, lease_id).await {
                        break Some((1001, "Session replaced"));
                    }
                    continue;
                }
                Ok(Either::First(Ok(Message::Ping(data)))) => {
                    if !touch_or_close(&session_id, lease_id).await {
                        break Some((1001, "Session replaced"));
                    }
                    tx.send_pong(data).await?
                }
                Ok(Either::First(Ok(Message::Close(reason)))) => {
                    tracing::info!(?reason, "websocket closed");
                    break None;
                }
                Ok(Either::First(Ok(Message::Text(data)))) => {
                    if !touch_or_close(&session_id, lease_id).await {
                        break Some((1001, "Session replaced"));
                    }
                    if let Some(on) = dev_telemetry_toggle(data) {
                        DEV_TELEMETRY.store(on, Ordering::Relaxed);
                        tx.send_text(if on {
                            "{\"type\":\"dev_telemetry\",\"status\":\"on\"}"
                        }
                        else {
                            "{\"type\":\"dev_telemetry\",\"status\":\"off\"}"
                        })
                        .await?;
                    }
                    else if is_heartbeat_text(data) {
                        tx.send_text("{\"type\":\"pong\"}").await?;
                    }
                    else {
                        match parse_command_from_str(data) {
                            Ok(cmd) => {
                                if let Some(payload) =
                                    dispatch_text_command(cmd, now_millis()).await
                                {
                                    tx.send_text(&payload).await?;
                                }
                            }
                            Err(CommandParseError::UnsupportedMessage) => {
                                tracing::debug!(
                                    payload_len = data.len(),
                                    "unsupported websocket message"
                                );
                                tx.send_text("unsupported_message").await?;
                            }
                            Err(CommandParseError::InvalidFormat) => {
                                tracing::warn!(
                                    payload_len = data.len(),
                                    "malformed websocket payload"
                                );
                                tx.send_text("invalid_format").await?;
                            }
                        }
                    }
                }
                Ok(Either::First(Ok(Message::Binary(data)))) => {
                    if !touch_or_close(&session_id, lease_id).await {
                        break Some((1001, "Session replaced"));
                    }
                    if is_heartbeat_binary(data) {
                        tx.send_binary(b"{\"type\":\"pong\"}").await?;
                    }
                    else {
                        match parse_command_from_slice(data) {
                            Ok(cmd) => {
                                if let Some(payload) =
                                    dispatch_binary_command(cmd, now_millis()).await
                                {
                                    tx.send_binary(&payload).await?
                                }
                            }
                            Err(CommandParseError::UnsupportedMessage) => {
                                tracing::debug!(
                                    payload_len = data.len(),
                                    "unsupported websocket message"
                                );
                                tx.send_binary(b"unsupported_message").await?
                            }
                            Err(CommandParseError::InvalidFormat) => {
                                tracing::warn!(
                                    payload_len = data.len(),
                                    "malformed websocket payload"
                                );
                                tx.send_binary(b"invalid_format").await?
                            }
                        }
                    }
                }
            };
        };

        // Dev-telemetry is a per-session tier; clear it so the next client does
        // not inherit this session's telemetry-on state.
        DEV_TELEMETRY.store(false, Ordering::Relaxed);
        let result = tx.close(close_reason).await;
        let removed = SessionManager::remove_session(&session_id, lease_id).await;
        if removed {
            queue_low_power_disconnect().await;
        }
        result
    }
}

#[derive(Debug, Deserialize)]
pub struct QueryParams {
    pub session: String,
    /// Pre-shared control secret. Optional on the wire so the parse stays
    /// backward compatible; authorization is enforced in the route handler via
    /// [`crate::net::auth`].
    #[serde(default)]
    pub key: String,
}

pub struct WsConnectionParams {
    pub upgrade: WebSocketUpgrade,
    pub query: QueryParams,
}

#[derive(Debug, Deserialize)]
struct HeartbeatMessage<'a> {
    #[serde(borrow, rename = "type")]
    kind: &'a str,
}

fn is_heartbeat_text(payload: &str) -> bool {
    serde_json::from_str::<HeartbeatMessage<'_>>(payload)
        .map(|msg| msg.kind.eq_ignore_ascii_case("ping"))
        .unwrap_or(false)
}

#[derive(Debug, Deserialize)]
struct DevTelemetryToggle<'a> {
    #[serde(borrow, rename = "type")]
    kind: &'a str,
    #[serde(default)]
    on: bool,
}

/// Parse a `{"type":"dev_telemetry","on":<bool>}` control frame, returning the
/// requested tier state. `None` for any other message.
fn dev_telemetry_toggle(payload: &str) -> Option<bool> {
    serde_json::from_str::<DevTelemetryToggle<'_>>(payload)
        .ok()
        .filter(|msg| msg.kind.eq_ignore_ascii_case("dev_telemetry"))
        .map(|msg| msg.on)
}

/// Serialize the current command-bus metrics as a `dev_telemetry` frame.
///
/// The metrics payload is serialized straight from
/// [`CommandBusMetricsSnapshot`] via serde so the wire shape cannot drift from
/// the struct definition. The snapshot lands under `data`, wrapped in the
/// standard `{"type":...,"data":...}` frame envelope.
fn dev_telemetry_frame() -> String {
    let m = command_bus_metrics_snapshot();
    let data = serde_json::to_string(&m).unwrap_or_else(|_| String::from("{}"));
    alloc::format!("{{\"type\":\"dev_telemetry\",\"data\":{}}}", data)
}

fn is_heartbeat_binary(payload: &[u8]) -> bool {
    serde_json::from_slice::<HeartbeatMessage<'_>>(payload)
        .map(|msg| msg.kind.eq_ignore_ascii_case("ping"))
        .unwrap_or(false)
}

impl<'r, S> FromRequest<'r, S> for WsConnectionParams
where
    S: Sync,
{
    type Rejection = &'static str;

    #[allow(clippy::clone_on_copy)]
    async fn from_request<R: picoserve::io::Read>(
        state: &'r S,
        parts: RequestParts<'r>,
        body: RequestBody<'r, R>,
    ) -> Result<Self, Self::Rejection> {
        let query_str = parts.query().ok_or_else(|| {
            tracing::warn!("websocket request missing query parameters");
            "Missing query parameters"
        })?;
        let query = deserialize_form::<QueryParams>(query_str).map_err(|err| {
            tracing::warn!(?err, "invalid websocket query parameters");
            "Invalid query parameters"
        })?;
        let upgrade = WebSocketUpgrade::from_request(state, parts, body)
            .await
            .map_err(|err| {
                tracing::warn!(?err, "failed to extract websocket upgrade");
                "Failed to extract WebSocketUpgrade"
            })?;

        if query.session.is_empty() {
            return Err("Session ID is required");
        }

        Ok(WsConnectionParams { upgrade, query })
    }
}
