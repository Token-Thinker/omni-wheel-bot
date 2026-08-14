//! Transport-agnostic command ingest seam.
//!
//! Every control transport — the WebSocket server today, the BLE GATT service
//! next, ESP-NOW later — funnels raw command bytes through this one entry
//! point. That keeps a single code path for: protocol parsing, the shared
//! `COMMAND_BUS`, the high-rate motion coalescer, and (critically for an
//! actuated robot) the deadman watchdog and the speed/NaN clamps applied
//! downstream in the executor. No transport should parse-and-drive on its own.
//!
//! # Authentication
//!
//! This function performs *no* application-layer authentication. It is intended
//! for transports whose link layer already authenticated the peer:
//! - **BLE**: an LE Secure Connections bonded link (encrypted, authenticated
//!   writes only — see the BLE GATT service design in `docs/backlog.md`).
//! - **WebSocket / IP**: the upgrade is gated by
//!   [`crate::net::auth`] before any frame reaches the socket loop, so
//!   bytes arriving here are already from an authorized controller.
//!
//! A transport that cannot authenticate at the link layer MUST gate on
//! [`crate::net::auth::verify_control_secret`] before calling this.

extern crate alloc;

use alloc::vec::Vec;

use crate::net::{
    gateway::dispatch_binary_command,
    protocol::{parse_command_from_slice, CommandParseError},
};

/// Parse a single raw command payload and enqueue it on the shared command
/// bus, returning an optional reply payload to send back to the peer.
///
/// The receive timestamp is stamped here from the monotonic clock so the
/// motion TTL / coalescer / watchdog all see a consistent time base regardless
/// of transport.
pub async fn ingest_command_bytes(bytes: &[u8]) -> Result<Option<Vec<u8>>, CommandParseError> {
    let received_ms = embassy_time::Instant::now().as_millis();
    let cmd = parse_command_from_slice(bytes)?;
    Ok(dispatch_binary_command(cmd, received_ms).await)
}
