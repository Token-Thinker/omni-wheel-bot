#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
HOST=${OWB_E2E_HOST:-127.0.0.1}
PORT=${OWB_E2E_PORT:-3300}
BASE_URL="http://${HOST}:${PORT}"
LOG_FILE="${TMPDIR:-/tmp}/owb-host-sim-e2e.log"

cleanup() {
  if [[ -n "${SERVER_PID:-}" ]] && kill -0 "$SERVER_PID" 2>/dev/null; then
    kill "$SERVER_PID" 2>/dev/null || true
    wait "$SERVER_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

cd "$ROOT_DIR"
OWB_BIND_ADDR="${HOST}:${PORT}" cargo run -p owb-host-sim --bin owb-host-sim >"$LOG_FILE" 2>&1 &
SERVER_PID=$!

for _ in $(seq 1 60); do
  if ! kill -0 "$SERVER_PID" 2>/dev/null; then
    cat "$LOG_FILE" >&2
    exit 1
  fi
  if curl -fsS "${BASE_URL}/healthz" >/dev/null 2>&1; then
    break
  fi
  sleep 0.5
done

if ! curl -fsS "${BASE_URL}/healthz" >/dev/null 2>&1; then
  cat "$LOG_FILE" >&2
  exit 1
fi

# check <path> [grep-pattern] -- names the endpoint on failure, because a bare
# `curl -f` only reports "error: 404" with no clue which of these 11 broke.
check() {
  local path="$1" pattern="${2:-}" body err
  if [[ -n "$pattern" ]]; then
    if ! body=$(curl -fsS "${BASE_URL}${path}" 2>&1); then
      echo "FAIL ${path}: ${body}" >&2
      return 1
    fi
    if ! printf '%s' "$body" | grep -q "$pattern"; then
      echo "FAIL ${path}: body did not match /${pattern}/" >&2
      return 1
    fi
  # -o /dev/null so binary bodies (/app.wasm) are not pulled into a command
  # substitution, which strips null bytes and warns.
  elif ! err=$(curl -fsS -o /dev/null "${BASE_URL}${path}" 2>&1); then
    echo "FAIL ${path}: ${err}" >&2
    return 1
  fi
}

check /healthz '^ok$'
check /
check /app-config.js 'hasCamera'
check /embedded
check /embedded/setup
check /embedded-host
check /embedded-host/setup
# Static assets come from owb-ui/dist/ via the ServeDir fallback. The Leptos
# build emits app.js + app.wasm; there is no script.js (that was the old
# hand-written UI), so asserting it here fails a healthy build.
check /style.css
check /app.js
check /app.wasm
check /api/schema '"system_command"'
check /api/schema '"/ws"'

if command -v websocat >/dev/null 2>&1; then
  WS_OUTPUT=$(printf '%s\n' '{"ct":"i","ic":"enable"}' | websocat -n1 "ws://${HOST}:${PORT}/ws" 2>/dev/null || true)
  printf '%s' "$WS_OUTPUT" | grep -q '"status":"ok"'
else
  echo "websocat not found; skipped websocket smoke test" >&2
fi

echo "host-sim e2e passed (${BASE_URL})"
