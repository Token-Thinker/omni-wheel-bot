#!/usr/bin/env bash
set -euo pipefail

# Host simulator plugin
# Expects env: MAKER, BOARD, BIN

maker_build() {
  unset OWB_CARGO_TARGET_DIR
  note "Building host simulator"
  rustc --version --verbose || true
  rustup show active-toolchain || true
  cargo build -p owb-host-sim -q
}

maker_flash() {
  unset OWB_CARGO_TARGET_DIR
  note "Starting host simulator"
  note "Phone UI: http://127.0.0.1:3000/"
  note "Dev console: http://127.0.0.1:3000/dev"
  cargo run -p owb-host-sim
}
