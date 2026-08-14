# Repository Guidelines

## Project Structure & Modules
- `owb-core/`: no-std library crate with drivers and utils; tests in `owb-core/tests/`.
- `owb-bin/owb-esp/`: ESP32-S3 firmware binary; board aliases in `.cargo/config.toml`. ESP32-S3 board behavior is defined by `owb-bin/owb-esp/src/profile/mod.rs` profiles (camera, motor, and IO behavior), with `owb-esp32s3-v2` as the default profile.
- Top-level workspace managed by `Cargo.toml`; `scripts/run_app.sh` is the build/flash/monitor entry point over the modular helpers under `scripts/` (also exposed via `justfile` recipes).

## Build, Test, and Development
- Build core crate: `cargo build -p owb-core`.
- Test core crate: `cargo test -p owb-core` (uses `embedded-hal-mock`).
- Format/lint: `cargo fmt -- --check` and `cargo clippy --all -- -D warnings`.
- Run application (`scripts/run_app.sh`):
  - Make executable: `chmod +x scripts/run_app.sh`
  - Guided flow: `./scripts/run_app.sh` prompts for maker, board, binary; ESP32-S3 builds also prompt for an IO profile before Wi‑Fi.
  - List/probe: `./scripts/run_app.sh --list` and `./scripts/run_app.sh --probe` (or `--probe-board`).
  - Non‑interactive flags: `--maker/--board/--bin/--port`, `--ssid/--password`, `--use-test-network` (reads `OWB_WIFI_SSID`/`OWB_WIFI_PASSWORD` from env), `--build`, `--flash`, `--monitor`, `--no-attach-monitor`, `--real-executors`, `--last`, `--tui/--no-tui`, `--dry-run`.
  - Shortcuts via `justfile`: `just build`, `just flash`, `just monitor`, `just probe`.
  - ESP (Docker): prints rustc/toolchain details from container; flashing via `espflash`.
  - Host‑sim: prints local rustc/toolchain details; no flashing.
  - Interactive QoL: before flashing choose to change target or exit; after run choose another board or exit.

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
- Wi‑Fi credentials are read from `OWB_WIFI_SSID`/`OWB_WIFI_PASSWORD` env vars or prompted interactively; they are never written to disk or committed. Copy `.env.example` → `.env` and fill in your network.
- Flashing requires a serial port (e.g., `/dev/ttyUSB0`) and `espflash` installed on host for ESP targets.
- Last target selection is saved in `.owb_last_target` (maker/board/bin only).

## Build/Flash Architecture (quick reference)
- Entry: `scripts/run_app.sh` — build/flash/monitor launcher (also wrapped by `justfile`).
- Common helpers: `scripts/lib.sh` (colors, serial, target mapping).
- Selection: `scripts/select_target.sh` (uses `scripts/ui.sh` for TUI when available).
- Wi‑Fi: `scripts/prompt_wifi.sh` (with optional test network).
- Build: `scripts/build.sh` dispatches to `scripts/makers/<maker>.sh` (`maker_build`).
- Flash: `scripts/flash.sh` dispatches to `scripts/makers/<maker>.sh` (`maker_flash`).
- Monitor: `scripts/monitor.sh` (uses `screen`).
- Info: `scripts/info.sh` powers `--list` and `--probe`.
- State: `scripts/state.sh` persists `.owb_last_target`.
