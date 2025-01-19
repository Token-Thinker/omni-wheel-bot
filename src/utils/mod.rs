//! Core Modules
//!
//! This file defines core modules essential for the system's functionality.
//!
//! # Modules
//! - `connection`: Manages network connections and communication protocols.
//! - `controllers`: Handles hardware control logic, including motor and sensor
//!   management.

/// Module for managing network connections and communication protocols.
pub(crate) mod connection;

/// Module for handling hardware controllers, such as motors and sensors.
pub(crate) mod controllers;

pub(crate) mod frontend;
pub(crate) mod packages;
