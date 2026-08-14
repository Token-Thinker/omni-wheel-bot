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
OWB_BIND_ADDR="${HOST}:${PORT}" cargo run -p owb-host-sim >"$LOG_FILE" 2>&1 &
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

curl -fsS "${BASE_URL}/healthz" | grep -qx 'ok'
curl -fsS "${BASE_URL}/" >/dev/null
curl -fsS "${BASE_URL}/app-config.js" | grep -q 'hasCamera'
curl -fsS "${BASE_URL}/embedded" >/dev/null
curl -fsS "${BASE_URL}/embedded/setup" >/dev/null
curl -fsS "${BASE_URL}/embedded-host" >/dev/null
curl -fsS "${BASE_URL}/embedded-host/setup" >/dev/null
curl -fsS "${BASE_URL}/style.css" >/dev/null
curl -fsS "${BASE_URL}/script.js" >/dev/null
curl -fsS "${BASE_URL}/api/schema" | grep -q '"system_command"'
curl -fsS "${BASE_URL}/api/schema" | grep -q '"/ws"'

if command -v websocat >/dev/null 2>&1; then
  WS_OUTPUT=$(printf '%s\n' '{"ct":"i","ic":"enable"}' | websocat -n1 "ws://${HOST}:${PORT}/ws" 2>/dev/null || true)
  printf '%s' "$WS_OUTPUT" | grep -q '"status":"ok"'
else
  echo "websocat not found; skipped websocket smoke test" >&2
fi

echo "host-sim e2e passed (${BASE_URL})"
