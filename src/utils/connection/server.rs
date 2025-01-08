//! WebSocket Server Module
//!
//! This module defines the WebSocket server implementation using the
//! `picoserve` framework. It manages incoming WebSocket connections, processes
//! I2C commands, and communicates with the embedded control system through a
//! channel interface.
//!
//! # Components
//! - `run`: Starts the WebSocket server.
//! - `ServerTimer`: Manages async timeouts.
//! - `WebSocket`: Defines WebSocket behavior for handling client messages.

use embassy_net::{driver::Driver as NetworkDriver, Stack};
use embassy_time::Duration;
use picoserve::{
    io::embedded_io_async as embedded_aio,
    response::ws::{
        Message,
        ReadMessageError,
        SocketRx,
        SocketTx,
        WebSocketCallback,
        WebSocketUpgrade,
    },
    Router,
};
use picoserve::response::StatusCode;
use crate::utils::controllers::{SystemCommand, I2C_CHANNEL, LED_CHANNEL};

/// Starts the WebSocket server with the provided configuration.
///
/// # Parameters
/// - `id`: Unique server identifier.
/// - `port`: Port to listen on.
/// - `stack`: Network stack for communication.
/// - `config`: Optional server configuration.
///
/// # Returns
/// This function runs indefinitely.
pub async fn run<Driver: NetworkDriver>(
    id: usize,
    port: u16,
    stack: &'static Stack<Driver>,
    config: Option<&'static picoserve::Config<Duration>>,
) -> !
{
    let default_config = picoserve::Config::new(picoserve::Timeouts {
        start_read_request: Some(Duration::from_secs(5)),
        read_request: Some(Duration::from_secs(1)),
        write: Some(Duration::from_secs(5)),
    });

    let config = config.unwrap_or(&default_config);

    let router = Router::new()
        .route("/",
               picoserve::routing::get(|| async {
                   picoserve::response::Response::new(StatusCode::OK, "Hello World").
                       with_header("Content-Type", "text/plain")
               })
        )

        .route(
        "/ws",
        picoserve::routing::get(|upgrade: WebSocketUpgrade| {
            if let Some(protocols) = upgrade.protocols() {
                for protocol in protocols {
                    tracing::info!("Client offered protocol: {}", protocol);
                }
            }
            upgrade
                .on_upgrade(WebSocket)
                .with_protocol("messages")
                }),
            );


    // Print out the IP and port before starting the server.
    if let Some(ip_cfg) = stack.config_v4() {
        tracing::info!(
            "Starting WebSocket server at ws://{}:{}/ws",
            ip_cfg.address,
            port
        );
    } else {
        tracing::warn!(
            "Starting WebSocket server on port {port}, but no IPv4 address is assigned yet!"
        );
    }

    let (mut rx_buffer, mut tx_buffer, mut http_buffer) = ([0; 1024], [0; 1024], [0; 256]);

    picoserve::listen_and_serve(
        id,
        &router,
        config,
        stack,
        port,
        &mut rx_buffer,
        &mut tx_buffer,
        &mut http_buffer,
    )
    .await
}

/// Manages timeouts for the WebSocket server.
pub struct ServerTimer;

#[allow(unused_qualifications)]
impl picoserve::Timer for ServerTimer
{
    type Duration = embassy_time::Duration;
    type TimeoutError = embassy_time::TimeoutError;

    /// Runs a future with a timeout.
    async fn run_with_timeout<F: core::future::Future>(
        &mut self,
        duration: Self::Duration,
        future: F,
    ) -> Result<F::Output, Self::TimeoutError>
    {
        embassy_time::with_timeout(duration, future).await
    }
}

/// Handles incoming WebSocket connections.
pub struct WebSocket;

impl WebSocketCallback for WebSocket
{
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
                        tx.send_binary(b"I2C command received and forwarded").await?
                    }
                    Ok(SystemCommand::L(led_cmd)) => {
                        LED_CHANNEL.send(led_cmd).await;
                        tx.send_binary(b"LED command received and forwarded").await?
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
