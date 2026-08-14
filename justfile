# OWB — just --list for the full cheat-sheet
# Requires: just (https://github.com/casey/just)

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
build:
    ./scripts/run_app.sh --maker owb-esp --board esp32s3 --bin owb-esp --build

# Build with hardware-backed board executors enabled
build-real:
    ./scripts/run_app.sh --maker owb-esp --board esp32s3 --bin owb-esp --build --real-executors

# Build with HIL low-authority mode (safer bench testing)
build-hil:
    ./scripts/run_app.sh --maker owb-esp --board esp32s3 --bin owb-esp --build --hil-low-mode

# --- Flash & Monitor ---

# Serial port (override with: just flash PORT=/dev/ttyUSB0)
PORT := "/dev/ttyACM0"

# Flash ESP32-S3 and attach serial monitor
flash:
    ./scripts/run_app.sh --maker owb-esp --board esp32s3 --bin owb-esp --flash --port {{PORT}}

# Flash without attaching serial monitor afterwards
flash-quiet:
    ./scripts/run_app.sh --maker owb-esp --board esp32s3 --bin owb-esp --flash --port {{PORT}} --no-attach-monitor

# Build then flash in one step
build-flash:
    ./scripts/run_app.sh --maker owb-esp --board esp32s3 --bin owb-esp --build --flash --port {{PORT}}

# Attach serial monitor to a running board (uses screen)
monitor:
    ./scripts/run_app.sh --maker owb-esp --board esp32s3 --bin owb-esp --monitor --port {{PORT}}

# --- Probe ---

# Probe attached serial targets and suggest board/port mappings
probe:
    ./scripts/run_app.sh --probe-board

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

# --- USB serial control (bot must be plugged in) ---

# Send a JSON command over USB serial (requires usb-init first)
# Example: just usb-send '{"ct":"i","ic":"enable"}'
usb-send CMD:
    printf '%s\n' '{{CMD}}' > {{PORT}}
