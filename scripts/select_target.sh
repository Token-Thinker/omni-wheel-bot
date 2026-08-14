#!/usr/bin/env bash
set -euo pipefail

__DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# shellcheck disable=SC1091
source "$__DIR/lib.sh"
source "$__DIR/ui.sh"
source "$__DIR/state.sh"
source "$__DIR/probe.sh"

select_target() {
  if [[ -z "${MAKER-}" ]]; then
    local ports=()
    mapfile -t ports < <(list_serial_ports || true)
    if (( ${#ports[@]} > 0 )); then
      section "Resolve Target"
      local resolution
      resolution=$(choose \
        "target-resolution" \
        "probe-connected-board" \
        "manual-select") || return 1
      case "$resolution" in
        probe-connected-board)
          if probe_and_select_target; then
            return 0
          else
            local probe_rc=$?
            if (( probe_rc != 2 )); then
              return 1
            fi
          fi
          echo
          ;;
        manual-select)
          :
          ;;
      esac
    fi
  fi

  # Maker
  if [[ -z "${MAKER-}" ]]; then
    section "Select chip maker"
    local MAKER_OPTIONS=()
    local d
    for d in owb-bin/*/; do
      [[ -d $d ]] && MAKER_OPTIONS+=("$(basename "$d")")
    done
    local sel
    sel=$(choose "maker" "${MAKER_OPTIONS[@]}") || return 1
    MAKER="$sel"
    echo
  else
    note "Maker: ${BOLD}$MAKER${RESET}"
    [[ -d "owb-bin/$MAKER" ]] || { err "Maker '$MAKER' not found in owb-bin/"; return 1; }
  fi

  # Board
  if [[ -z "${BOARD-}" ]]; then
    section "Select target board"
    # Minimal defaults per maker (extend as needed)
    local BOARD_OPTIONS=()
    case "$MAKER" in
      owb-esp|esp|espressif) BOARD_OPTIONS=(esp32s3) ;;
      owb-host-sim|host|sim) BOARD_OPTIONS=(host) ;;
      *) BOARD_OPTIONS=(unknown) ;;
    esac
    BOARD=$(choose "board" "${BOARD_OPTIONS[@]}") || return 1
  else
    note "Board: ${BOLD}$BOARD${RESET}"
  fi

  if ! TRIPLE=$(target_triple_for "$MAKER" "$BOARD"); then
    err "Unknown board '$BOARD' for maker '$MAKER'"
    return 1
  fi
  echo

  # Bin
  if [[ -z "${BIN-}" ]]; then
    section "Select firmware binary"
    local BIN_OPTIONS=()
    local name
    while IFS= read -r name; do
      [[ -n "$name" ]] && BIN_OPTIONS+=("$name")
    done < <(binaries_for_maker "$MAKER" || true)
    if [[ ${#BIN_OPTIONS[@]} -eq 0 ]]; then
      err "No Cargo bin targets found for '$MAKER'"
      return 1
    fi
    BIN=$(choose "binary" "${BIN_OPTIONS[@]}") || return 1
    echo
  else
    note "Binary: ${BOLD}$BIN${RESET}"
    local found=false
    local name
    while IFS= read -r name; do
      if [[ "$name" == "$BIN" ]]; then
        found=true
        break
      fi
    done < <(binaries_for_maker "$MAKER" || true)
    [[ "$found" == true ]] || {
      err "Binary '$BIN' is not a declared Cargo bin target for '$MAKER'"
      return 1
    }
  fi

  export MAKER BOARD BIN TRIPLE
  save_last_selection || true
}
