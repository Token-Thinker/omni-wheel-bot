#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/lib.sh"

tui_available() {
  [[ "${TUI:-auto}" != "false" ]] || return 1
  [[ -t 0 && -t 1 ]] || return 1
  if command -v fzf &>/dev/null; then return 0; fi
  if command -v whiptail &>/dev/null; then return 0; fi
  return 1
}

choose() {
  local title="$1"; shift
  local opts=("$@")
  if [[ ${#opts[@]} -eq 0 ]]; then return 1; fi

  if command -v fzf &>/dev/null && tui_available; then
    printf '%s\n' "${opts[@]}" | fzf --prompt "${title}> " --height=15 --border --ansi
    return $?
  fi

  if command -v whiptail &>/dev/null && tui_available; then
    local menu_args=() i=1
    for o in "${opts[@]}"; do menu_args+=("$i" "$o"); i=$((i+1)); done
    local sel
    sel=$(whiptail --title "${title}" --menu "${title}" 20 70 10 "${menu_args[@]}" 3>&1 1>&2 2>&3) || return 1
    echo "${opts[$((sel-1))]}"
    return 0
  fi

  # Readable fallback menu for plain terminals.
  local i choice
  for i in "${!opts[@]}"; do
    printf "%d) %s\n" "$((i + 1))" "${opts[$i]}" >&2
  done

  while :; do
    read -r -p "$(echo -e "${YELLOW}Choice [1-${#opts[@]}]:${RESET} ")" choice
    if [[ "$choice" =~ ^[0-9]+$ ]] && (( choice >= 1 && choice <= ${#opts[@]} )); then
      # Add a visual break before the next command/output starts.
      echo >&2
      echo "${opts[$((choice - 1))]}"
      return 0
    fi
    err "Invalid selection. Try again." >&2
  done
}
