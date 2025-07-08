# owb-core

Core drivers and utilities for the Omni-Wheel Bot on no-std embedded platforms.

[![crates.io](https://img.shields.io/crates/v/owb-core.svg)](https://crates.io/crates/owb-core)
[![docs.rs](https://docs.rs/owb-core/badge.svg)](https://docs.rs/owb-core)
[![license](https://img.shields.io/badge/license-MIT%20%2F%20Apache--2.0-blue.svg)](LICENSE-MIT)

## Overview

The **owb-core** crate provides core abstractions, drivers, and helper utilities
for building omnidirectional robot firmware using the embedded-hal and Embassy ecosystems.

It is designed for no-std targets and supports async/await via Embassy.

## Quick Start

Add **owb-core** to your Cargo.toml:

```toml
[dependencies]
owb-core = "0.1"
```

In your application:

```rust
#![no_std]
#![no_main]

use owb_core::utils::math::Vector2;

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    // Initialize peripherals and scheduler...
    let v = Vector2::new(1.0, 0.0);
    // Use utilities and controllers to drive the wheels
}
```

## Features

## Documentation

Full API documentation is available on [docs.rs](https://docs.rs/owb-core).

### WebSocket server

```no_run
use picoserve::Router;
use embassy_executor::Spawner;
use owb_core::utils::connection::server::{WebSocket, ServerTimer};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Build the WebSocket route at `/ws`
    let mut router = Router::new();
    router.ws("/ws", WebSocket, ServerTimer);

    // Listen for incoming WebSocket connections indefinitely
    router.listen().await.unwrap();
}
```

## License

This project is dual-licensed under MIT OR Apache-2.0. See the
[LICENSE-MIT] and [LICENSE-APACHE] files for details.