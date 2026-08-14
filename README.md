[![OpenSSF Scorecard](https://api.securityscorecards.dev/projects/github.com/Token-Thinker/omni-wheel-bot-pro/badge)](https://securityscorecards.dev/viewer/?uri=github.com/Token-Thinker/omni-wheel-bot-pro)
![crates.io](https://img.shields.io/crates/v/owb-core.svg) ![docs.rs](https://docs.rs/owb-core/badge.svg) ![license](https://img.shields.io/badge/license-MIT%20%2F%20Apache--2.0-blue.svg)

# Omni‑Wheel Bot Workspace

Firmware and libraries for an omnidirectional robot, built on Rust, embedded‑hal, and Embassy.

## Documentation

All design and operations docs live in [`docs/`](docs/) as plain markdown
files — read them directly in your IDE or browse them rendered as an mdBook
site:

- **Rendered site** (after first deploy): https://token-thinker.github.io/omni-wheel-bot/
- **Source markdown**: [`docs/`](docs/) (table of contents in [`docs/SUMMARY.md`](docs/SUMMARY.md))
- **Build locally**: `cargo install mdbook --locked && mdbook serve --open`
- **Backlog**: [`docs/backlog.md`](docs/backlog.md) is the canonical work-item list.

## Block Architecture

```text
[ Web / Mobile Client ]
          |
          v
[ Shared App Surface in owb-core ]
  operator page + setup page + /ws + camera stream alias
          |
          v
[ Shared Command + Runtime Layer in owb-core ]
  transport::protocol -> COMMAND_BUS -> node::command_bus -> motion::omni / motion::pid / motion::encoder
          |
          v
[ Target Binary ]
  board profile + network bring-up + HAL wiring
          |
          v
[ Board Runtime ]
  motors + sensors + camera + power + telemetry
```

## Workspace Layout

- `owb-core/`: no‑std library with onboard device drivers, the shared embedded web app surface, and the ROS-shaped runtime taxonomy (`msg`, `motion`, `node`, `transport`, `board`, `diagnostics`).
  - Features:
    - `net`: picoserve + embassy‑net/time/sync + executor, including shared static assets, `/ws`, the operator/setup pages, camera stream helpers, and command-drain helpers (auto-pulls the `owb-ui` bundle)
    - `owb-ui`: pull only the embedded operator UI bundle from the `owb-ui` workspace crate (auto-enabled by `net`)
    - `leds`: optional LED traits
    - `serde` (default): serialization for commands
  - Onboard device drivers (`lis2dh12`, `hdc2010`, `max17048`, `tca9555`, `ov2640`) are always available. The legacy `drivers` feature (PCA9685 + ICM-42670) was removed in v0.1.9; see `owb-core/CHANGELOG.md`.
  - Also includes reusable pre-HIL primitives for PID loops, quadrature encoder decoding, and multi-ToF port bring-up planning.
  - Docs: https://docs.rs/owb-core

- `owb-bin/owb-esp/`: ESP32-S3 firmware using `esp-hal`, `esp-radio`/`esp-rtos`, and Embassy. `owb-bin/owb-esp/src/profile/mod.rs` defines board behavior (camera, motor, and IO profile data); the default profile is the current v2 custom board (`owb-esp32s3-v2`). Only `esp32s3` is supported; legacy ESP32 (non-S3) targets were removed in v0.1.9.
  Custom S3 profiles perform a boot-time I2C self-test (TCA9555, LIS2DH12, HDC2010, MAX17048). By default the firmware keeps the shadow executor for safe bring-up; `--real-executors` enables the GPIO-backed DRV8845 runtime for the supported ESP32-S3 boards before HIL.
- `owb-bin/owb-host-sim/`: Host simulator (tokio + axum) to test UI + WebSocket without flashing hardware. Ships the `/sim` physics replay viewer (see Quick Start).
- `owb-ui/`: Embedded operator UI assets shipped as a Rust crate. HTML + CSS are hand-written; `assets/script.js` is the compiled output from the TypeScript source under `owb-ui/ts/`. Consumed by `owb-core` via the `owb-ui` feature (auto-enabled by `net`).

## Current Board Matrix

- Test boards:
  - `xiao-seedstudio-sense-cam`
  - `esp32-s3-devkitc-1-n32r16`
- Custom ESP32-S3 boards:
  - `owb-esp32s3-v1`
  - `owb-esp32s3-v1-alpha`
  - `owb-esp32s3-v2`

## Quick Start

### Clone and run the host simulator

```bash
git clone https://github.com/Token-Thinker/omni-wheel-bot.git
cd omni-wheel-bot
cargo run -p owb-host-sim
```

Then open one of:

- `http://127.0.0.1:3000/` — the host simulator console
- `http://127.0.0.1:3000/embedded` — the embedded operator page (same surface as on-device)
- `http://127.0.0.1:3000/sim` — the physics replay viewer (added in v0.1.9; see `owb-bin/owb-host-sim/src/sim_routes.rs`)

### Develop UI + WebSocket on host

```bash
cargo run -p owb-host-sim
# Open http://127.0.0.1:3000/ for the host simulator console
# Or open http://127.0.0.1:3000/embedded for the embedded operator page
```

Quick MVP smoke test:

```bash
./scripts/e2e_host_sim.sh
```

For repeated build/flash/test loops without retyping flags, or via `just`:

```bash
./scripts/run_app.sh
# or: just build / just flash / just monitor / just probe
```

- WebSocket endpoint: `ws://127.0.0.1:3000/ws`
- Accepts either a SystemCommand or a RequestV1; see API below.

### Build + Flash ESP32‑S3 firmware

Only `esp32s3` is supported; legacy ESP32 (non-S3) targets were removed in v0.1.9.

```bash
chmod +x scripts/run_app.sh
./scripts/run_app.sh          # guided build/flash/monitor
# or use ./scripts/run_app.sh --help for flags / shortcuts
```

- The script sets up the xtensa toolchain in Docker and prompts for Wi‑Fi credentials.
- For ESP builds, it prompts for an IO profile first and passes it to the build as `OWB_IO_PROFILE`.
- Add `--real-executors` to enable the hardware-backed `esp32s3` GPIO runtime.
- Add `--hil-low-mode` to build a safer bench-test runtime with reduced motor authority (intended for HIL bring-up on breadboard/new wiring).
- For ESP32-S3 `real-executors`, normal drive mode serves the normal embedded
  frontend; `--hil-low-mode` serves the compact HIL test page instead.
- The HIL test page supports motor identity, encoder sign, `counts -> rad/s`,
  and per-wheel PID probes; see `docs/motor_feedback_hil.md`.
- Requires a serial device (e.g., `/dev/ttyUSB0`).

## Run Application (scripts/run_app.sh)

The run application is modular and extendable via a `scripts/` directory. `scripts/run_app.sh` is the entrypoint; helper scripts live under `scripts/`, and maker‑specific adapters live under `scripts/makers/`. `justfile` wraps the common cases (`just build`, `just flash`, `just monitor`, `just probe`).

- scripts/lib.sh: colors, prompts, board→triple mapping, serial detection.
- scripts/select_target.sh: interactive target selection (supports TUI).
- scripts/prompt_wifi.sh: Wi‑Fi prompt with optional test network.
- scripts/build.sh: dispatches to a maker plugin for building.
- scripts/flash.sh: dispatches to a maker plugin for flashing.
- scripts/monitor.sh: serial monitor helper.
- scripts/info.sh: `--list`, `--probe`, and `--probe-board` commands.
- scripts/select_profile.sh: picks ESP32-S3 IO profiles and exports `OWB_IO_PROFILE` for the build.
- scripts/ui.sh: optional TUI powered by `fzf` or `whiptail`.
- scripts/state.sh: saves last target selection to `.owb_last_target` (no Wi‑Fi saved).

Supported makers and examples:
- owb-esp (ESP32-S3):
  - Interactive: `./scripts/run_app.sh`
  - Non‑interactive: `./scripts/run_app.sh --maker owb-esp --board esp32s3 --bin owb-esp --use-test-network`
  - Uses Docker image `espressif/idf-rust`, prints `rustc --version --verbose` and active toolchain from inside the container.
- owb-host-sim (host simulator):
  - Build only: `./scripts/run_app.sh --maker owb-host-sim --board host --bin owb-host-sim --build`
  - Build + run: `./scripts/run_app.sh --maker owb-host-sim --board host --bin owb-host-sim`
  - Run directly: `cargo run -p owb-host-sim`
  - Prints local `rustc --version --verbose` and `rustup show active-toolchain` before the build.

Discover and probe:
- `./scripts/run_app.sh --list` prints detected makers, supported boards, and available binaries.
- `./scripts/run_app.sh --probe` shows the workspace Rust version, detected serial ports, and tools (docker, espflash, screen, elf2uf2-rs, openocd, dfu-util).
- `./scripts/run_app.sh --probe-board` prints USB board identity and suggested target mapping.

Non‑interactive flags:
- `--maker NAME` `--board NAME` `--bin NAME` `--port PATH`
- `--ssid SSID` `--password PASS` `--use-test-network` (reads `OWB_WIFI_SSID`/`OWB_WIFI_PASSWORD` from env; see `.env.example`)
- `--force-softap` (skip STA credentials and serve OWB-BOT at `192.168.4.1`)
- `--no-attach-monitor` (don't attach serial monitor during flash)
- `--last` (reuse last maker/board/bin selection; saved in `.owb_last_target`)
- `--tui` / `--no-tui` (force enable/disable TUI menus; default auto)
- `--dry-run` (print actions without executing)
- `--real-executors` (enable hardware-backed executors for supported ESP targets)
- `--hil-low-mode` (reduce motor authority and serve the compact HIL test page)
- `--normal-drive-mode` (explicitly disable HIL low mode)
- `--auto-flash-on-test` (automatically flash after build when using the test network)

Interactive quality‑of‑life:
- Before flashing you can choose to flash now, change target, or exit.
- After finishing, you can pick another board or exit to loop runs smoothly.

DevKit breadboard wiring overrides:
- Put local GPIO exports in `config/devkit_wiring.local.env`
- Start from [config/devkit_wiring.local.env.example](config/devkit_wiring.local.env.example)
- The file is loaded automatically by `scripts/run_app.sh`

Notes:
- The deployer never writes Wi‑Fi credentials to disk. The optional test network is public: SSID=`iot`, PASSWORD=`12345678`.
- ESP32-S3 behavior comes from `owb-bin/owb-esp/src/profile/mod.rs` (default: `owb-esp32s3-v2`).
- Non‑ESP makers (e.g., RP2040/STM32) are scaffolded with "coming soon" flash flows; they already build with the correct Rust target triples.

### Core library (no‑std)

```bash
cargo build -p owb-core --features net
cargo test  -p owb-core
cargo fmt -- --check
cargo clippy --all -- -D warnings
```

If a target exposes the standard web shell, the shared core helpers now own the
common route surface:

```rust
owb_core::runtime::spawn_common(spawner, stack, None);
// or, for camera-capable targets:
owb_core::runtime::spawn_camera_web_app(spawner, stack, None);
// board binaries provide only the hardware-specific /camera/stream handler
```

## WebSocket JSON API

Two formats are accepted at `/ws`:

### Canonical Control Map

```text
Right stick   -> {"ct":"i","ic":"t","direction_deg","speed_norm"}
              -> I2CCommand::Translate
              -> 3-wheel translation plan

Left stick X  -> {"ct":"i","ic":"y","s","o":null}
              -> I2CCommand::Yaw
              -> in-place spin / yaw blend

Idle / center -> zero speed + zero yaw through the same schema
              -> stop / hold

Long left     -> {"ct":"p","pc":"motors_toggle"}
              -> PowerCommand::MotorsToggle

Long right    -> {"ct":"p","pc":"doze_toggle"}
              -> PowerCommand::DozeToggle

Double right  -> {"ct":"p","pc":"camera_toggle"} + local browser preview toggle
              -> PowerCommand::CameraToggle
              -> device stream enable/disable + local preview sync

Triple left   -> {"ct":"p","pc":"estop"}
              -> PowerCommand::Estop

Triple right  -> {"ct":"a","ac":"auto_mode"}
              -> AutonomyCommand::AutoMode

LED actions   -> {"ct":"l","lc":"on|off|brightness|sc", ...}
              -> LEDCommand
              -> board LED driver

Calibration   -> {"ct":"i","ic":"wheel_trim","w1":1.0,"w2":1.0,"w3":1.0}
              -> I2CCommand::WheelTrim (per-wheel static trim, clamped to [0.0, 2.0])
              -> {"ct":"i","ic":"heading_hold","enable":true,"kp":0.4}
              -> I2CCommand::HeadingHold (closed-loop encoder-differential straightness, forward-only)
              -> {"ct":"i","ic":"stiction_kick","enable":true,"boost":0.2,"ms":100}
              -> I2CCommand::StictionKick (one-shot startup PWM boost to break static friction)

Schema docs   -> GET /api/schema
```

The `WheelTrim`, `HeadingHold`, and `StictionKick` commands were added in v0.1.9
to support the v1-alpha straight-line tracking work.

1) Direct SystemCommand (serde‑tagged):

```json
{ "ct": "i", "ic": "enable" }
{ "ct": "i", "ic": "t", "direction_deg": 90.0, "speed_norm": 0.5 }
{ "ct": "i", "ic": "wheel_trim", "w1": 1.0, "w2": 0.96, "w3": 1.0 }
{ "ct": "i", "ic": "heading_hold", "enable": true, "kp": 0.4 }
{ "ct": "l", "lc": "brightness", "v": 128 }
```

2) RequestV1 (type‑tagged; mapped to SystemCommand):

```json
{ "type": "i2c", "cmd": "t", "direction_deg": 90.0, "speed_norm": 0.5 }
{ "type": "i2c", "cmd": "o", "d": 45.0, "s": 0.8, "rs": 10.0, "o": 0.0 }
{ "type": "led", "cmd": "brightness", "value": 128 }
```

## MSRV

Tracked by `owb-core/Cargo.toml` (`rust-version`).

## License

Dual‑licensed under MIT OR Apache‑2.0. See [LICENSE-MIT](LICENSE-MIT) and [LICENSE-APACHE](LICENSE-APACHE).

## Roadmap

- Extend binaries to RP2040 and STM32 boards (Arduino‑class) while keeping application logic in `owb-core`.
