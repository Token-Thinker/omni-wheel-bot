# Repository Guidelines

## Project Structure & Modules
- `owb-core/`: no-std library crate with drivers and utils; tests in `owb-core/tests/`.
- `owb-bin/owb-esp/`: ESP32-S3 firmware binary; board aliases in `.cargo/config.toml`. ESP32-S3 board behavior is defined by `owb-bin/owb-esp/src/profile/mod.rs` profiles (camera, motor, and IO behavior), with `owb-esp32s3-v2` as the default profile.
- Top-level workspace managed by `Cargo.toml`; `cargo xtask` (crate `xtask/`) is the build/flash/monitor entry point, with `./owb` running it inside the idf-rust Docker image on machines without Rust (also exposed via `justfile` recipes).

## Build, Test, and Development
- Build core crate: `cargo build -p owb-core`.
- Test core crate: `cargo test -p owb-core` (uses `embedded-hal-mock`).
- Format/lint: `cargo xtask ci check` (pinned nightly rustfmt, since `rustfmt.toml` uses nightly-only options, plus clippy `-D warnings` and tests).
- Build/flash/monitor (`cargo xtask`, or `./owb` without host Rust):
  - `./owb build --profile <io-profile> [--real-executors] [--hil-low-mode] [--force-softap]`
  - `./owb flash --port /dev/ttyACM0 [--no-monitor]`, `./owb build-flash ...`, `./owb monitor --port ... [--reset]`
  - `./owb probe` / `./owb info` to identify boards and list profiles; `./owb ci check` for the CI host checks; `cargo xtask e2e` for the host-sim smoke test.
  - `--last` reuses `.owb_last_target`; `--dry-run` prints the docker/espflash commands. Without a TTY a missing choice errors out naming the flag.
  - Shortcuts via `justfile`: `just build`, `just flash`, `just monitor`, `just probe`.

## Coding Style & Naming
- Language: Rust 2021; MSRV follows `owb-core/Cargo.toml` (`rust-version`).
- Formatting: enforced by repo `rustfmt.toml` (imports reordered, braces next line, vertical params).
- Linting: treat clippy warnings as errors in CI/local (`-D warnings`).
- Naming: `snake_case` for funcs/vars, `CamelCase` for types/traits, `SCREAMING_SNAKE_CASE` for consts.

## Testing Guidelines
- Framework: Rust `#[test]` in `owb-core/tests/` (e.g., `mock_tests.rs`).
- Mocks: `embedded-hal-mock` for I2C and peripheral interactions.
- Conventions: name tests descriptively (`test_*`), prefer unit tests for math/kinematics and controller logic.
- Run: `cargo test -p owb-core`; workspace-level tests: `cargo test --all`.

## Commit & Pull Requests
- Current history is informal; prefer concise, imperative subject lines. Recommendation: Conventional Commits (`feat:`, `fix:`, `chore:`).
- Branches: `feature/<name>`, `fix/<name>`.
- Before PR: run `cargo fmt`, `cargo clippy --all`, `cargo test --all`.
- PRs should include: clear description, linked issues, build/test notes, and (when user-facing) short screenshots/logs.

## Device & Security Notes
- Wi‑Fi credentials are read from `SSID`/`PASSWORD` (or `OWB_WIFI_SSID`/`OWB_WIFI_PASSWORD`) env vars or prompted interactively; they are never written to disk or committed. Copy `.env.example` → `.env` and fill in your network.
- Flashing requires a serial port (e.g., `/dev/ttyACM0`). `espflash` runs on the host, or inside the idf-rust image via `./owb`.
- Last target selection is saved in `.owb_last_target` (profile/port; no Wi‑Fi).

## Build/Flash Architecture (quick reference)
- Entry: `cargo xtask` (`xtask/src/main.rs`), wrapped by `./owb` and the `justfile`.
- Firmware build: `xtask/src/firmware.rs` (docker args, features, image tag); `resolve.rs` picks profile/port; `prompt.rs` only prompts on a TTY.
- Monitor: `xtask/src/monitor.rs` (native serial monitor; `--reset` = DTR low + RTS pulse, never `espflash reset`).
- Probe/info: `xtask/src/probe.rs`. State: `xtask/src/state.rs` persists `.owb_last_target`.
- Host-sim smoke test: `xtask/src/e2e.rs`.
