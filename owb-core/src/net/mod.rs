//! Networking and transport layer.
//!
//! Merges the former `connection` and `transport` modules into one home for
//! everything that moves bytes on or off the robot: the HTTP server and route
//! composition, the WebSocket frame handler, the SoftAP DHCP server, the
//! embassy-net stack bring-up, control-channel auth, and the wire protocol /
//! command-ingest helpers shared by every transport.

/// Transport-independent control-channel authentication shared by every
/// control transport (WebSocket today, BLE GATT next).
#[cfg(feature = "net")]
pub mod auth;

#[cfg(feature = "net")]
pub mod gateway;
#[cfg(feature = "net")]
pub mod router;
/// HTTP server, routes, and message handling.
#[cfg(feature = "net")]
pub mod server;
#[cfg(feature = "net")]
pub(crate) mod session;
#[cfg(feature = "net")]
pub(crate) mod websocket;

/// HTTP transport re-exports for the embedded server.
#[cfg(feature = "net")]
pub mod http;

/// Transport-agnostic command ingest shared by every control transport
/// (WebSocket, BLE GATT, ESP-NOW).
#[cfg(feature = "net")]
pub mod ingest;

/// Minimal DHCP server for SoftAP mode using embassy-net UDP sockets.
#[cfg(feature = "net")]
pub mod dhcp;

/// embassy-net stack construction and SoftAP defaults.
// The `net::net` path repetition is intentional: this submodule owns the
// embassy-net stack wiring specifically, distinct from the sibling transport
// modules (websocket, server, dhcp). Renaming would churn call sites for no gain.
#[allow(clippy::module_inception)]
#[cfg(feature = "net")]
pub mod net;

/// Shared protocol (request types and mappers) used by server and host-sim.
#[cfg(feature = "serde")]
pub mod protocol;
