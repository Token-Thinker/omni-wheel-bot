//! Module Exports
//!
//! This file exports the key modules used in the WebSocket server
//! implementation.
//!
//! # Modules
//! - `server`: Manages the WebSocket server, routes, and message handling.

/// Module for managing the WebSocket server, including routes and connection
/// handling.
pub(crate) mod server;

// Re-export the server module for easier access.
pub use server::run as app_server;
