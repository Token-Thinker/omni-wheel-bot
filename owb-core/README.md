# owb-core

Core drivers and utilities for the Omni-Wheel Bot on no-std embedded platforms.

[![crates.io](https://img.shields.io/crates/v/owb-core.svg)](https://crates.io/crates/owb-core)
[![docs.rs](https://docs.rs/owb-core/badge.svg)](https://docs.rs/owb-core)
[![license](https://img.shields.io/badge/license-MIT%20%2F%20Apache--2.0-blue.svg)](LICENSE-MIT)

## Overview

The **owb-core** crate provides core abstractions, drivers, and helper utilities
for building robot firmware using the embedded-hal and Embassy ecosystems.

It is designed for no-std targets and supports async/await via Embassy.

## Block Architecture

```text
[ msg ]
  Drive, Stop, Enable, telemetry, diagnostics, dev/HIL commands
          |
          v
[ transport ]
  HTTP/WebSocket/protocol adapters
          |
          v
[ node ]
  command router + drive controller + telemetry publisher
          |
          v
[ motion ]
  kinematics + diff drive + PID + encoder math
          |
          v
[ board ]
  reusable board profiles, capabilities, and embedded-hal style traits
          |
          v
[ drivers / sim / target app ]
  reusable device drivers, host validation, and concrete HAL setup
```

Canonical paths are `motion::pid`, `motion::encoder`, and `motion::omni`. The
`controllers`, `connection`, and `protocol` re-exports remain for legacy callers.

## Quick Start

Add **owb-core** to your Cargo.toml:

```toml
[dependencies]
owb-core = "0.1.9"
```

In your application:

```rust
#![no_std]
#![no_main]

use owb_core::math::Vector2;

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    // Initialize peripherals and scheduler...
    let v = Vector2::new(1.0, 0.0);
    // Use the `motion`, `node`, and `board` namespaces to drive the wheels.
}
```

## Features

- `net`: Embassy + picoserve integration for the shared HTTP/WebSocket app shell, operator/setup routes, camera stream helpers, command-drain runtime glue, and DHCP helpers. Automatically enables `owb-ui`.
- `owb-ui`: Pull the embedded operator UI bundle from the [`owb-ui`](https://crates.io/crates/owb-ui) workspace crate. Auto-enabled by `net`; exposed standalone so headless consumers can swap the bundle.
- `sim`: Host-side simulation support (`std` + `serde`).
- `leds`: Optional LED traits (`smart-leds-trait`).
- `serde` (default): Serialization support for core command types.

Onboard device drivers under `drivers/` (`lis2dh12`, `hdc2010`, `max17048`,
`tca9555`, `ov2640`) are always available; they do not require a feature flag.

ROS-shaped module taxonomy:
- `msg`: canonical normal commands, telemetry, diagnostics, and dev/HIL commands.
- `motion`: kinematics, differential drive, PID, encoder math, and omni-wheel planning (`motion::pid`, `motion::encoder`, `motion::omni`).
- `node`: command bus, command routing, drive control, and telemetry publishing boundaries (`node::command_bus`).
- `board`: reusable board profiles, capabilities, and portable drive-board traits.
- `transport`: `protocol`/HTTP/WebSocket adapter namespace.
- `diagnostics`: health/status payloads.
- `tof`: three-port ToF bring-up/address-planning helpers for the custom boards.

### Shared motion planning

The core crate also owns the pre-HIL motion-to-output planning used by the
GPIO-backed executors. Board runtimes can convert shared omni-wheel motion
commands into simple direction/drive intents without reimplementing the
kinematics:

```rust
use owb_core::{
    motion::omni::drive_plan_from_motion,
    math::kinematics::EmbodiedKinematics,
};

let kin = EmbodiedKinematics::new(0.148, 0.195, 7.0);
let plan = drive_plan_from_motion(&kin, true, 0.6, 90.0, 0.0, 0.1, 0.08);
assert!(plan.enabled);
```

### Runtime calibration commands (v0.1.9)

v0.1.9 added three runtime commands to the WebSocket / command-bus surface so
operators can tune straight-line tracking without re-flashing:

- `I2CCommand::WheelTrim { w1, w2, w3 }` — per-wheel open-loop trim, clamped to
  `[0.0, 2.0]`. Routed to the board runtime's `set_wheel_trim` hook.
- `I2CCommand::HeadingHold { enable, kp }` — closed-loop encoder-differential
  straightness corrector, gated to forward-only motion so it does not fight
  operator-commanded turns.
- `I2CCommand::StictionKick { enable, boost, ms }` — one-shot startup PWM boost
  to break the static-friction floor on low-`speed_norm` motion.

### Migration from 0.1.0

v0.1.9 removed several compatibility shims (`control`, `encoder`, `actuation`),
the `runtime::*` command-bus re-exports, and the legacy `drivers` feature
(PCA9685 + ICM-42670 + MPU6050). The legacy ESP32 (Xtensa, non-S3) firmware
path was also removed; only ESP32-S3 is supported.

A complete old-path-to-new-path map lives in
[`owb-core/CHANGELOG.md`](CHANGELOG.md) under "Breaking Changes".

## Documentation

Full API documentation is available on [docs.rs](https://docs.rs/owb-core).

### Runtime command surface

Once the embassy-net stack and command bus are wired up, operators tune the
v1-alpha straight-line tracking loop entirely over the WebSocket — no
re-flashing required:

```jsonc
// per-wheel open-loop trim (clamped to [0.0, 2.0])
{ "ct": "i", "ic": "wheel_trim", "w1": 1.0, "w2": 0.96, "w3": 1.0 }

// closed-loop encoder-differential straightness corrector (forward-only)
{ "ct": "i", "ic": "heading_hold", "enable": true, "kp": 0.4 }

// one-shot startup PWM boost to break static-friction floor
{ "ct": "i", "ic": "stiction_kick", "enable": true, "boost": 0.2, "ms": 100 }
```

Migrating from 0.1.0? See [`CHANGELOG.md`](CHANGELOG.md) for the rename table.

## License

This project is dual-licensed under MIT OR Apache-2.0. See the
[LICENSE-MIT] and [LICENSE-APACHE] files for details.
