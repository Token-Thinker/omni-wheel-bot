# OWB — just --list for the full cheat-sheet
# Requires: just (https://github.com/casey/just)
#
# Firmware recipes go through ./owb, which runs `cargo xtask ...` on the host
# when Rust is installed and inside the espressif/idf-rust image otherwise
# (Docker-only machines). Extra flags pass through, e.g.
#   just build --profile owb-esp32s3-v1-alpha --real-executors --force-softap

# Default: list all recipes
default:
    @just --list

# --- Simulation ---

# Run the host simulator (axum dev server)
sim:
    cargo run -p owb-host-sim

# Run host-sim and rebuild on changes (requires cargo-watch)
sim-watch:
    cargo watch -x 'run -p owb-host-sim'

# --- Build ---

# Build the ESP32-S3 firmware via Docker cross-compile (no flash)
build *ARGS:
    ./owb build {{ARGS}}

# Build with hardware-backed board executors enabled
build-real *ARGS:
    ./owb build --real-executors {{ARGS}}

# Build with HIL low-authority mode (safer bench testing)
build-hil *ARGS:
    ./owb build --hil-low-mode {{ARGS}}

# --- Flash & Monitor ---

# Serial port (override with: just flash PORT=/dev/ttyUSB0)
PORT := "/dev/ttyACM0"

# Flash ESP32-S3 and attach serial monitor
flash *ARGS:
    ./owb flash --port {{PORT}} {{ARGS}}

# Flash without attaching serial monitor afterwards
flash-quiet *ARGS:
    ./owb flash --port {{PORT}} --no-monitor {{ARGS}}

# Build then flash in one step
build-flash *ARGS:
    ./owb build-flash --port {{PORT}} {{ARGS}}

# Attach serial monitor to a running board (no reset; add --reset for the boot log)
monitor *ARGS:
    ./owb monitor --port {{PORT}} {{ARGS}}

# --- Probe ---

# Probe attached serial targets and suggest board/port mappings
probe:
    ./owb probe

# Toolchain, Docker image, serial ports and IO profiles
info:
    ./owb info

# --- Check / Test ---

# Run host-side checks and tests (no hardware required)
check:
    cargo check -p owb-core --features net,serde
    cargo check -p owb-host-sim
    cargo test -p owb-core --features net,serde

# Run clippy on host crates
lint:
    cargo clippy -p owb-core --features net,serde -- -D warnings
    cargo clippy -p owb-host-sim -- -D warnings

# --- UI ---

# Build the Leptos WASM operator UI (requires trunk: cargo install trunk)
ui-build:
    cd owb-ui && trunk build --release

# Serve the UI locally for development
ui-dev:
    cd owb-ui && trunk serve

# Check the UI typecheck only (no trunk needed)
ui-check:
    cargo check --target wasm32-unknown-unknown -p owb-ui

# Run all host-side checks including WASM UI
check-all: check ui-check

# Local parity for the GitHub Actions host jobs (check|build|publish-dry|esp32s3|all)
ci TARGET="all":
    ./owb ci {{TARGET}}

# Host-sim end-to-end smoke test
e2e:
    ./owb e2e

# --- USB serial control (bot must be plugged in) ---

# Send a JSON command over USB serial (requires usb-init first)
# Example: just usb-send '{"ct":"i","ic":"enable"}'
usb-send CMD:
    printf '%s\n' '{{CMD}}' > {{PORT}}
