# Changelog

All notable changes to `owb-core` will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

_No changes yet._

## [0.1.9] - 2026-05-23

The first published cut after the v1-alpha straight-line tracking research
session. Adds the runtime calibration surface (`WheelTrim`, `HeadingHold`),
reshapes the crate around a ROS-inspired module taxonomy, and drops the legacy
ESP32 (PCA9685 + ICM-42670 + MPU6050) hardware path so the supported targets
are now ESP32-S3 only. See `docs/reports/v1_alpha_straight_line_tracking.md`
for the engineering report this release ships against, and
`docs/reports/v1_alpha_improvements.md` for the deferred follow-ups.

### Added

- `I2CCommand::WheelTrim { w1, w2, w3 }` runtime command for per-wheel
  open-loop static trim. Values clamp to `[0.0, 2.0]`. Routed through the
  command-bus to a new `set_wheel_trim` hook on the board runtime.
- `I2CCommand::HeadingHold { enable, kp }` runtime command for the
  closed-loop encoder-differential straightness corrector. Gated to
  forward-only motion so it does not fight operator-commanded turns.
- ROS-shaped module taxonomy: `motion/`, `node/`, `transport/`, `msg/`,
  `board/`, and `diagnostics/`. See
  `docs/architecture/core_runtime_split.md` for the rationale and the
  before/after layout.
- `#![recursion_limit = "256"]` at the crate root so the embassy executor's
  task-pool type generics compile under the `net` feature.
- ESP-IDF App Descriptor support via `esp-bootloader-esp-idf` 0.4 on the
  ESP32-S3 target binary (consumed by `espflash`/`picoserve` 0.18's bring-up
  expectations). This lives in the `owb-bin/owb-esp` binary but ships as
  part of the v0.1.9 release alongside this crate.
- `BoardProfile.default_wheel_trim` and `BoardProfile.min_forward_speed_norm`
  (stiction-floor speed-norm clamp) on the `owb-esp` board profile struct.
  `owb-esp32s3-v1-alpha` ships with `min_forward_speed_norm = 0.55`,
  matching the measured static-friction threshold in the alpha report.

### Changed

- Bumped `embassy-net` from 0.8 to 0.9 to align with `picoserve` 0.18 and
  resolve the type conflict that blocked the ESP32-S3 firmware build.
- Command-bus APIs are now reached via `owb_core::node::command_bus` rather
  than through the `runtime` module re-exports (see Breaking Changes
  below).
- `embassy-net` consumers downstream must move to 0.9-compatible imports
  (e.g. `Ipv4Cidr`, `StaticConfigV4`).

### Removed

- Compatibility shim modules `control`, `encoder`, and `actuation` at the
  crate root. They were thin re-exports of `motion::pid`, `motion::encoder`,
  and `motion::omni` respectively. Use the canonical `motion::*` paths.
- Re-exports of `run_command_bus`, `spawn_command_sink`,
  `command_bus_metrics_snapshot`, `note_command_queue_full_drop`, and
  `CommandBusMetricsSnapshot` from `owb_core::runtime`. Use
  `owb_core::node::command_bus::*` instead.
- The `drivers` feature flag on `owb-core` (PCA9685 + ICM-42670 + MPU6050
  adapters). Those drivers were only used by the legacy ESP32 hardware path
  and are no longer wired in. The driver source files remain available
  for reference, but they are not exposed via a published Cargo feature.
- Legacy ESP32 (Xtensa, non-S3) firmware path in `owb-bin/owb-esp`:
  the `esp32`, `esp32c2`, `esp32c3`, `esp32c6`, `esp32h2`, and `esp32s2`
  Cargo features are gone. The published firmware is ESP32-S3 only.
- The `OWB_ESP32_V2_NO_PSRAM` build profile / env shim, since legacy ESP32
  is no longer supported.

### Fixed

- Infinite flash-retry loop in `scripts/flash.sh` when invoked
  non-interactively (e.g. under `tee` or the HIL loop): added a TTY guard
  and capped retries via `MAX_FLASH_RETRIES`.
- Absolute developer path leak in the top-level `README.md`.
- Minor `clippy` warnings surfaced by the refactor; one remaining
  non-exhaustive match on `LEDCommand::Brightness` is tracked in the
  improvements companion as item D1.

### Breaking Changes

Downstream users of the removed shims, the dropped `drivers` feature, and
the removed `runtime` re-exports must update imports. Migration table:

| Old path / feature                                    | New path / replacement                                          |
| ----------------------------------------------------- | --------------------------------------------------------------- |
| `owb_core::control`                                   | `owb_core::motion::pid`                                         |
| `owb_core::encoder`                                   | `owb_core::motion::encoder`                                     |
| `owb_core::actuation`                                 | `owb_core::motion::omni`                                        |
| `owb_core::runtime::run_command_bus`                  | `owb_core::node::command_bus::run_command_bus`                  |
| `owb_core::runtime::spawn_command_sink`               | `owb_core::node::command_bus::spawn_command_sink`               |
| `owb_core::runtime::command_bus_metrics_snapshot`     | `owb_core::node::command_bus::command_bus_metrics_snapshot`     |
| `owb_core::runtime::note_command_queue_full_drop`     | `owb_core::node::command_bus::note_command_queue_full_drop`     |
| `owb_core::runtime::CommandBusMetricsSnapshot`        | `owb_core::node::command_bus::CommandBusMetricsSnapshot`        |
| `owb-core` Cargo feature `drivers`                    | _removed_ — depend on `pwm-pca9685` / `icm42670` directly       |
| `owb-bin/owb-esp` feature `esp32` / `esp32c*`/`s2`    | _removed_ — only `esp32s3` is supported                         |
| PCA9685 controller factory                            | _removed_ — use the ESP32-S3 DRV8845 path or vendor a copy      |

## [0.1.0]

Initial release on crates.io.
