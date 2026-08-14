//! OWB shared wire-protocol types.
//!
//! This crate provides the canonical message contracts used across firmware,
//! the host simulator, and the operator UI. It is `no_std` with an optional
//! `serde` feature for JSON serialization.

#![no_std]
extern crate alloc;

pub mod command;
pub mod dev;
pub mod legacy;
pub mod telemetry;
