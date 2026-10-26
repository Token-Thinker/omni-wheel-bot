# OWB Firmware Binaries

This directory hosts the firmware entrypoints for supported MCUs. The first
target is ESP32‑S3 (`owb-esp`). The longer‑term plan is to add additional
boards while keeping application logic in the shared `owb-core` crate.

## Block Architecture

```text
[ Binary Crate ]
  target entrypoint
        |
        v
[ Board Profile ]
  pins + features + default tuning
        |
        v
[ HAL / Network Wiring ]
  clocks + GPIO + I2C + Wi-Fi / transport
        |
        v
[ Shared owb-core Runtime ]
  web app shell + command loop helpers + motion planning (`motion::omni`)
        |
        v
[ Board-Specific Runtime ]
  actual actuator / sensor implementation
```

## Current Targets

- `owb-esp`: ESP32-S3 firmware using `esp-hal`, `esp-radio`/`esp-rtos`, and Embassy.
  - Provides a SoftAP fallback with a minimal DHCP server.
  - Reuses the shared `owb-core` web app surface (`/`, `/setup`, `/ws`) and only keeps the board-specific camera stream handler in the binary.
  - Current baseline is camera/network-first, with the shared `owb-core` runtime owning the app shell and command loop.
  - Custom ESP32-S3 profiles run a boot-time board self-test for the shared I2C parts.
  - By default, ESP32-S3 builds keep the shadow executor for safe bring-up; pass `--real-executors` to `cargo xtask build` (or `./owb build`) to enable the GPIO-backed DRV8845 runtime on the supported custom boards before HIL.
  - Current ESP32-S3 profile names:
    - `xiao-seedstudio-sense-cam`
    - `esp32-s3-devkitc-1-n32r16`
    - `owb-esp32s3-v1`
    - `owb-esp32s3-v1-alpha`
    - `owb-esp32s3-v2`

The legacy ESP32 (Xtensa, non-S3) firmware path and its `esp32`/`esp32c*`/`esp32s2`
Cargo features were removed in v0.1.9; see `owb-core/CHANGELOG.md`.

## Target Rules

RP2040/STM32 makers are not yet implemented. The active firmware target is
ESP32-S3 plus the host simulator.

Active targets should:

- Keep board wiring and runtime decisions in the binary crate.
- Reuse `owb-core` for shared protocol, net, math, reusable drivers, and common app/runtime glue.
- Reuse `owb-core`’s `net` helpers for stack bring‑up, DHCP(SoftAP), the shared web shell, and WS.
- Keep the binary crate focused on board initialization (clocks, pins, I2C,
  Wi‑Fi) and spawn tasks via core.

## Building

Build and flash from the repo root with `cargo xtask`, or `./owb` on a machine
that has Docker but no Rust (see the root README). For ESP32‑S3:

```bash
./owb build-flash --profile owb-esp32s3-v1-alpha --real-executors --port /dev/ttyACM0
# or: just build / just flash / just monitor
```

The firmware is cross-compiled in the `espressif/idf-rust` image. Wi‑Fi comes
from the `SSID`/`PASSWORD` env vars, a prompt, or `--force-softap`. Add
`--real-executors` when you want the custom S3 boards to drive their real
GPIO-backed motor runtime instead of the shadow executor.

For the host simulator MVP path, use the repository-level smoke test:

```bash
cargo xtask e2e
```
