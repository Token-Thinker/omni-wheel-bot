# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

> The canonical, per-crate changelog is [`owb-core/CHANGELOG.md`](../owb-core/CHANGELOG.md).
> This workspace-level file mirrors the headline entries for the rendered mdBook.

## [Unreleased]

_No changes yet._

## [0.1.9] - 2026-05-23

The first published cut after the v1-alpha straight-line tracking research
session. See [`docs/reports/v1_alpha_straight_line_tracking.md`](reports/v1_alpha_straight_line_tracking.md)
for the engineering report this release ships against and
[`docs/reports/v1_alpha_improvements.md`](reports/v1_alpha_improvements.md)
for the deferred follow-ups.

### Added
- `I2CCommand::WheelTrim { w1, w2, w3 }` runtime command for per-wheel open-loop
  static trim. Routed through the command bus to a new `set_wheel_trim` hook on
  the board runtime.
- `I2CCommand::HeadingHold { enable, kp }` closed-loop encoder-differential
  straightness corrector, gated to forward-only motion.
- ROS-shaped module taxonomy: `motion/`, `node/`, `transport/`, `msg/`,
  `board/`, and `diagnostics/`. See
  [`docs/architecture/core_runtime_split.md`](architecture/core_runtime_split.md).
- `BoardProfile.default_wheel_trim` and `BoardProfile.min_forward_speed_norm`
  (stiction-floor speed-norm clamp). `owb-esp32s3-v1-alpha` ships with
  `min_forward_speed_norm = 0.55`.
- ESP-IDF App Descriptor support via `esp-bootloader-esp-idf` 0.4 on the
  ESP32-S3 target binary.

### Changed
- Bumped `embassy-net` from 0.8 to 0.9 to align with `picoserve` 0.18 and
  resolve the type conflict that blocked the ESP32-S3 firmware build.
- Command-bus APIs are now reached via `owb_core::node::command_bus` rather
  than through the `runtime` module re-exports (breaking change).

### Removed
- Compatibility shim modules `control`, `encoder`, and `actuation` at the
  crate root. Use the canonical `motion::pid`, `motion::encoder`, and
  `motion::omni` paths.
- Re-exports of `run_command_bus`, `spawn_command_sink`,
  `command_bus_metrics_snapshot`, `note_command_queue_full_drop`, and
  `CommandBusMetricsSnapshot` from `owb_core::runtime`. Use
  `owb_core::node::command_bus::*` instead.
- The `drivers` feature flag on `owb-core` (PCA9685 + ICM-42670 + MPU6050
  adapters). Those drivers were only used by the legacy ESP32 hardware path.
- Legacy ESP32 (Xtensa, non-S3) firmware path in `owb-bin/owb-esp`. Only
  `esp32s3` is supported.
- The `OWB_ESP32_V2_NO_PSRAM` build profile / env shim.

### Fixed
- Infinite flash-retry loop in `scripts/flash.sh` when invoked
  non-interactively: added a TTY guard and capped retries via
  `MAX_FLASH_RETRIES`.
- Absolute developer path leak in the top-level `README.md`.

## [0.1.0] - 2025-07-07
### Added
- Initial release of `owb-core`, including:
  - Core Omni-Wheel Bot kinematics and math utilities
  - I2C device manager for PWM (PCA9685) and IMU sensor (ICM-42670) — removed in v0.1.9
  - WebSocket server integration example and utilities
  - Mock-based unit tests for controllers and kinematics
