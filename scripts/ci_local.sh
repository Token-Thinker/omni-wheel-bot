#!/usr/bin/env bash
# Local-dev parity for the GitHub Actions CI defined in .github/workflows/ci.yml.
#
# Targets mirror the host-side workflow jobs so a contributor can reproduce
# CI locally before pushing. Container-based esp32s3 build is delegated to
# `scripts/run_app.sh` so the same idf-rust image is used.
#
# Usage:
#   scripts/ci_local.sh check        # fmt + clippy + test
#   scripts/ci_local.sh build        # cargo build -p owb-host-sim
#   scripts/ci_local.sh publish-dry  # cargo publish --dry-run -p owb-core --allow-dirty
#   scripts/ci_local.sh esp32s3      # build firmware via scripts/run_app.sh (Docker cross-compile)
#   scripts/ci_local.sh all          # run check + build + publish-dry sequentially

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# shellcheck source=lib.sh
source "${SCRIPT_DIR}/lib.sh"

cd "${REPO_ROOT}"

usage() {
  cat <<'USAGE'
ci_local.sh — run the same checks GitHub Actions runs, locally.

Targets:
  check        cargo fmt --check, clippy -D warnings, test -p owb-core
  build        cargo build -p owb-host-sim
  publish-dry  cargo publish --dry-run -p owb-core --allow-dirty (informational)
  esp32s3      scripts/run_app.sh --maker owb-esp --board esp32s3 --bin owb-esp --build
  all          check + build + publish-dry (host-side only)
  help         show this message
USAGE
}

run_fmt() {
  section "cargo fmt --check"
  cargo fmt --all -- --check
  ok "fmt clean"
}

run_clippy() {
  section "cargo clippy (owb-core, owb-host-sim)"
  cargo clippy -p owb-core -p owb-host-sim --no-deps -- -D warnings
  ok "clippy clean"
}

run_test() {
  section "cargo test -p owb-core"
  cargo test -p owb-core
  ok "tests passed"
}

run_check() {
  run_fmt
  run_clippy
  run_test
}

run_build() {
  section "cargo build -p owb-host-sim"
  cargo build -p owb-host-sim
  ok "host-sim built"
}

run_publish_dry() {
  section "cargo publish --dry-run -p owb-core --allow-dirty (informational)"
  if cargo publish --dry-run -p owb-core --allow-dirty; then
    ok "publish dry-run succeeded"
  else
    warn "publish dry-run failed (informational only; expected when version already exists on crates.io)"
    return 0
  fi
}

run_esp32s3() {
  section "scripts/run_app.sh --maker owb-esp --board esp32s3 --bin owb-esp --build"
  if [[ ! -x "${SCRIPT_DIR}/run_app.sh" ]]; then
    err "scripts/run_app.sh not found or not executable at ${SCRIPT_DIR}/run_app.sh"
    return 1
  fi
  "${SCRIPT_DIR}/run_app.sh" --maker owb-esp --board esp32s3 --bin owb-esp --build
  ok "esp32s3 firmware build complete"
}

target="${1:-help}"
case "$target" in
  check)        run_check ;;
  build)        run_build ;;
  publish-dry)  run_publish_dry ;;
  esp32s3)      run_esp32s3 ;;
  all)          run_check; run_build; run_publish_dry ;;
  help|-h|--help) usage ;;
  *)
    err "Unknown target: $target"
    usage
    exit 2
    ;;
esac
