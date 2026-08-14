#!/usr/bin/env bash
set -euo pipefail

__DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# shellcheck disable=SC1091
source "$__DIR/lib.sh"

flash_firmware() {
  section "Flash (${MAKER}/${BOARD}/${BIN})"

  if [[ ! "$MAKER" =~ ^(owb-host-sim|host|sim)$ ]]; then
    local PORT_INIT="${PORT-}"
    local PORT="$PORT_INIT"
    if [[ -z "$PORT" ]]; then
      if is_dry_run; then
        PORT="/dev/ttyUSB0"
        note "[dry-run] assuming port $PORT"
      else
        if ! PORT=$(detect_serial_port); then
          read -r -p "$(echo -e "${YELLOW}""Serial port (e.g. /dev/ttyUSB0):${RESET} ")" PORT || true
        fi
      fi
    fi
    if ! is_dry_run; then
      [[ -n "$PORT" ]] || { err "No serial port provided or detected."; return 1; }
    fi
  fi

  local plugin="$__DIR/makers/${MAKER}.sh"
  if [[ -f "$plugin" ]]; then
    # shellcheck disable=SC1090
    source "$plugin"
    if is_dry_run; then
      note "[dry-run] maker_flash for $MAKER ($BOARD/$BIN) via $plugin"
    else
      local attempt=1
      local MAX_RETRIES=3
      while :; do
        if (( attempt > 1 )); then
          note "Flash retry #$attempt"
        fi

        if maker_flash; then
          if [[ "${ATTACH_MONITOR:-true}" == true ]]; then
            note "Opening serial monitor. Reset the device if you want to watch boot output."
            if ! monitor_serial; then
              warn "Flash succeeded, but monitor could not be opened."
            fi
          fi
          ok "Flash complete"
          echo
          return 0
        fi

        warn "Flash failed."
        if (( attempt >= MAX_RETRIES )); then
          err "flash: aborting after $attempt failed attempts (non-interactive or retry limit reached)"
          return 1
        fi
        if [[ ! -t 0 ]]; then
          err "flash: aborting after $attempt failed attempts (non-interactive or retry limit reached)"
          return 1
        fi
        local ans=""
        read -r -p "$(echo -e "${YELLOW}Retry flash, change port, monitor, or abort? [R/p/m/a]: ${RESET}")" ans || true
        case "${ans,,}" in
          ""|r|retry)
            attempt=$((attempt + 1))
            ;;
          p|port)
            if ! PORT=$(detect_serial_port); then
              read -r -p "$(echo -e "${YELLOW}""Serial port (e.g. /dev/ttyUSB0):${RESET} ")" PORT || true
            fi
            [[ -n "$PORT" ]] || { err "No serial port provided."; return 1; }
            attempt=$((attempt + 1))
            ;;
          m|monitor)
            if ! monitor_serial; then
              warn "Monitor could not be opened."
            else
              ok "Flash step complete"
              echo
              return 0
            fi
            ;;
          a|abort|e|exit|q|quit)
            err "Flash aborted."
            return 1
            ;;
          *)
            attempt=$((attempt + 1))
            ;;
        esac
      done
    fi
  else
    if [[ -z "${TRIPLE-}" || -z "${BIN-}" ]]; then
      err "Missing TRIPLE or BIN for flashing."
      return 1
    fi
    local elf
    elf=$(cargo_release_artifact_path) || {
      err "Missing build context (TRIPLE/BIN) for flashing."
      return 1
    }
    if ! is_dry_run; then
      [[ -f "$elf" ]] || { err "Missing $elf"; return 1; }
    else
      note "[dry-run] would flash $elf on $PORT"
    fi
    warn "No maker plugin for '$MAKER'; flashing not implemented."
  fi
  if is_dry_run; then
    ok "Flash complete"
    echo
  fi
}
