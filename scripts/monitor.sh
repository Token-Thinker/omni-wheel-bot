#!/usr/bin/env bash
set -euo pipefail

__DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# shellcheck disable=SC1091
source "$__DIR/lib.sh"

monitor_serial() {
  section "Monitor Serial"
  case "$MAKER" in
    owb-host-sim|host|sim)
      warn "Monitor not applicable for host simulator."
      ;;
    owb-esp|esp|espressif|*)
      if ! command -v screen &>/dev/null; then
        if is_dry_run; then
          warn "[dry-run] 'screen' not found; would use it if present."
        else
          err "Install 'screen' to monitor serial."
          return 1
        fi
      fi
      local PORT="${PORT-}"
      if [[ -z "$PORT" ]]; then
        if is_dry_run; then
          PORT="/dev/ttyUSB0"; note "[dry-run] assuming port $PORT"
        else
          if ! PORT=$(detect_serial_port); then
            read -r -p "$(echo -e "${YELLOW}"Serial port:"${RESET}" )" PORT
            [[ -n "$PORT" ]] || return 1
          fi
        fi
      fi
      if is_dry_run; then
        note "[dry-run] screen $PORT 115200"
      else
        screen "$PORT" 115200
      fi
      ;;
  esac
  echo
}
