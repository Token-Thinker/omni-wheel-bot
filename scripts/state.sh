#!/usr/bin/env bash
set -euo pipefail

STATE_FILE=".owb_last_target"

save_last_selection() {
  [[ "${OWB_NO_SAVE_LAST_TARGET:-0}" == "1" ]] && return 0
  [[ -n "${MAKER-}" && -n "${BOARD-}" && -n "${BIN-}" ]] || return 0
  {
    printf 'MAKER=%q\n' "$MAKER"
    printf 'BOARD=%q\n' "$BOARD"
    printf 'BIN=%q\n' "$BIN"
    [[ -n "${PORT-}" ]] && printf 'PORT=%q\n' "$PORT"
    [[ -n "${IO_PROFILE-}" ]] && printf 'IO_PROFILE=%q\n' "$IO_PROFILE"
  } > "$STATE_FILE"
}

load_last_selection() {
  [[ -f "$STATE_FILE" ]] || return 1
  # shellcheck disable=SC1090
  source "$STATE_FILE"
  export MAKER BOARD BIN
  [[ -n "${PORT-}" ]] && export PORT
  if [[ -n "${IO_PROFILE-}" ]]; then
    export IO_PROFILE
    export OWB_IO_PROFILE="$IO_PROFILE"
  fi
}
