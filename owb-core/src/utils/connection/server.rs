//! WebSocket Server Module
//!
//! This module defines the WebSocket server implementation using the
//! `picoserve` framework. It manages incoming WebSocket connections, processes
//! I2C commands, and communicates with the embedded control system through a
//! channel interface.

extern crate alloc;

use alloc::{string::String, vec::Vec};

use embassy_net::Stack;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::Duration;
use embedded_io_async::Read;
use hashbrown::HashMap;
use lazy_static::lazy_static;
use picoserve::{
    extract::FromRequest,
    io::embedded_io_async as embedded_aio,
    request::{RequestBody, RequestParts},
    response::{
        ws::{Message, ReadMessageError, SocketRx, SocketTx, WebSocketCallback, WebSocketUpgrade},
        StatusCode,
    },
    url_encoded::deserialize_form,
    Router,
};
use serde::Deserialize;

use crate::utils::{
    controllers::{SystemCommand, I2C_CHANNEL, LED_CHANNEL},
    frontend::{CSS, HTML, JAVA},
};

pub struct ServerTimer;
pub struct WebSocket;
#[derive(Clone, Debug)]
pub struct SessionState {
    pub last_seen: u64,
}
pub struct SessionManager;

lazy_static! {
    pub static ref SESSION_STORE: Mutex<CriticalSectionRawMutex, HashMap<String, SessionState>> =
        Mutex::new(HashMap::new());
}

/// Manages timeouts for the WebSocket server.
#[allow(unused_qualifications)]
impl picoserve::Timer for ServerTimer {
    type Duration = embassy_time::Duration;
    type TimeoutError = embassy_time::TimeoutError;

    //noinspection ALL
    /// Runs a future with a timeout.
    async fn run_with_timeout<F: core::future::Future>(
        &mut self,
        duration: Self::Duration,
        future: F,
    ) -> Result<F::Output, Self::TimeoutError> {
        embassy_time::with_timeout(duration, future).await
    }
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
        let mut buffer = [0; 1024];

        tx.send_text("Connected").await?;

        let close_reason = loop {
            match rx.next_message(&mut buffer).await {
                Ok(Message::Pong(_)) => continue,
                Ok(Message::Ping(data)) => tx.send_pong(data).await?,
                Ok(Message::Close(reason)) => {
                    tracing::info!(?reason, "websocket closed");
                    break None;
                }
                Ok(Message::Text(data)) => match serde_json::from_str::<SystemCommand>(data) {
                    Ok(SystemCommand::I(i2c_cmd)) => {
                        I2C_CHANNEL.send(i2c_cmd).await;
                        tx.send_text("I2C command received and forwarded").await?;
                    }
                    Ok(SystemCommand::L(led_cmd)) => {
                        LED_CHANNEL.send(led_cmd).await;
                        tx.send_text("LED command received and forwarded").await?;
                    }
                    Err(error) => {
                        tracing::error!(?error, "error deserializing SystemCommand");
                        tx.send_text("Invalid command format").await?
                    }
                },
                Ok(Message::Binary(data)) => match serde_json::from_slice::<SystemCommand>(data) {
                    Ok(SystemCommand::I(i2c_cmd)) => {
                        I2C_CHANNEL.send(i2c_cmd).await;
                        tx.send_binary(b"I2C command received and forwarded")
                            .await?
                    }
                    Ok(SystemCommand::L(led_cmd)) => {
                        LED_CHANNEL.send(led_cmd).await;
                        tx.send_binary(b"LED command received and forwarded")
                            .await?
                    }
                    Err(error) => {
                        tracing::error!(?error, "error deserializing incoming message");
                        tx.send_binary(b"Invalid command format").await?
                    }
                },
                Err(error) => {
                    tracing::error!(?error, "websocket error");
                    let code = match error {
                        ReadMessageError::TextIsNotUtf8 => 1007,
                        ReadMessageError::ReservedOpcode(_) => 1003,
                        ReadMessageError::ReadFrameError(_)
                        | ReadMessageError::UnexpectedMessageStart
                        | ReadMessageError::MessageStartsWithContinuation => 1002,
                        ReadMessageError::Io(err) => return Err(err),
                    };
                    break Some((code, "Websocket Error"));
                }
            };
        };

        tx.close(close_reason).await
    }
}

#[allow(dead_code)]
impl SessionManager {
    /// Creates a new session with the given session ID and timestamp.
    pub async fn create_session(
        session_id: String,
        timestamp: u64,
    ) {
        SESSION_STORE.lock().await.insert(
            session_id,
            SessionState {
                last_seen: timestamp,
            },
        );
    }

    /// Retrieves a copy of the session state for the given session ID.
    /// Returns None if the session does not exist.
    pub async fn get_session(session_id: &str) -> Option<SessionState> {
        SESSION_STORE.lock().await.get(session_id).cloned()
    }

    //noinspection ALL
    //noinspection ALL
    /// Updates the last seen timestamp of the session identified by session_id.
    /// Returns true if the session was found and updated.
    pub async fn update_session(
        session_id: &str,
        timestamp: u64,
    ) -> bool {
        if let Some(session) = SESSION_STORE.lock().await.get_mut(session_id) {
            session.last_seen = timestamp;
            true
        } else {
            false
        }
    }

    /// Removes the session identified by session_id.
    /// Returns true if a session was removed.
    pub async fn remove_session(session_id: &str) -> bool {
        SESSION_STORE.lock().await.remove(session_id).is_some()
    }

    //noinspection ALL
    //noinspection ALL
    /// Purges sessions that have not been updated since the provided threshold.
    /// For example, pass in a timestamp and any session with last_seen less
    /// than that value will be removed.
    pub async fn purge_stale_sessions(threshold: u64) {
        // Retain sessions that have a last_seen timestamp >= threshold.
        SESSION_STORE
            .lock()
            .await
            .retain(|_id, session| session.last_seen >= threshold);
    }

    /// Returns a list of active session IDs.
    pub async fn list_sessions() -> Vec<String> {
        SESSION_STORE.lock().await.keys().cloned().collect()
    }
}

//noinspection ALL
//noinspection ALL
//noinspection ALL
//noinspection ALL
//noinspection ALL
/// Creates WS Server
pub async fn run(
    id: usize,
    port: u16,
    stack: Stack<'static>,
    config: Option<&'static picoserve::Config<Duration>>,
) -> ! {
    let default_config = picoserve::Config::new(picoserve::Timeouts {
        start_read_request: Some(Duration::from_secs(5)),
        persistent_start_read_request: None,
        read_request: Some(Duration::from_secs(1)),
        write: Some(Duration::from_secs(5)),
    });

    let config = config.unwrap_or(&default_config);

    let router = Router::new()
        // Serve the HTML file at "/"
        .route(
            "/",
            picoserve::routing::get(|| async {
                // Serve HTML content
                picoserve::response::Response::new(
                    StatusCode::OK,
                    HTML, // Static HTML content
                )
                .with_headers([
                    ("Content-Type", "text/html; charset=utf-8"),
                    ("Content-Encoding", "gzip"),
                ])
            }),
        )
        // Serve the CSS file at "/style.css"
        .route(
            "/style.css",
            picoserve::routing::get(|| async {
                // Serve CSS content
                picoserve::response::Response::new(
                    StatusCode::OK,
                    CSS, // Static CSS content
                )
                .with_headers([
                    ("Content-Type", "text/css; charset=utf-8"),
                    ("Content-Encoding", "gzip"),
                ])
            }),
        )
        // Serve the JS file at "/script.js"
        .route(
            "/script.js",
            picoserve::routing::get(|| async {
                // Serve JS content
                picoserve::response::Response::new(
                    StatusCode::OK,
                    JAVA, // Static JS content
                )
                .with_headers([
                    ("Content-Type", "application/javascript; charset=utf-8"),
                    ("Content-Encoding", "gzip"),
                ])
            }),
        )
        // WebSocket communication on "/ws"
        .route(
            "/ws",
            picoserve::routing::get(|params: WsConnectionParams| async move {
                let session_id = params.query.session;
                tracing::info!("New WebSocket connection with session id: {}", session_id);
                let now = embassy_time::Instant::now().as_secs();
                SessionManager::create_session(session_id.clone(), now).await;
                params
                    .upgrade
                    .on_upgrade(WebSocket)
                    .with_protocol("messages")
            }),
        );

    // Print out the IP and port before starting the server.
    if let Some(ip_cfg) = stack.config_v4() {
        tracing::info!("Starting server at {}:{}", ip_cfg.address, port);
    } else {
        tracing::warn!(
            "Starting WebSocket server on port {port}, but no IPv4 address is assigned yet!"
        );
    }

    let (mut rx_buffer, mut tx_buffer, mut http_buffer) = ([0; 1024], [0; 1024], [0; 4096]);

    picoserve::listen_and_serve_with_state(
        id,
        &router,
        config,
        stack,
        port,
        &mut rx_buffer,
        &mut tx_buffer,
        &mut http_buffer,
        &(),
    )
    .await
}

#[derive(Debug, Deserialize)]
pub struct QueryParams {
    session: String,
}

pub struct WsConnectionParams {
    pub upgrade: WebSocketUpgrade,
    pub query: QueryParams,
}

impl<'r, S> FromRequest<'r, S> for WsConnectionParams {
    type Rejection = &'static str; // Or a custom error type

    async fn from_request<R: Read>(
        state: &'r S,
        parts: RequestParts<'r>,
        body: RequestBody<'r, R>,
    ) -> Result<Self, Self::Rejection> {
        // First extract the WebSocketUpgrade as usual.
        let upgrade = WebSocketUpgrade::from_request(state, parts.clone(), body)
            .await
            .map_err(|_| "Failed to extract WebSocketUpgrade")?;

        // Then extract the query string for QueryParams.
        let query_str = parts.query().ok_or("Missing query parameters")?;
        let query =
            deserialize_form::<QueryParams>(query_str).map_err(|_| "Invalid query parameters")?;

        if query.session.is_empty() {
            return Err("Session ID is required");
        }

        Ok(WsConnectionParams { upgrade, query })
    }
}
