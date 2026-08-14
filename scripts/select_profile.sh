#!/usr/bin/env bash
set -euo pipefail

__DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# shellcheck disable=SC1091
source "$__DIR/lib.sh"
# shellcheck disable=SC1091
source "$__DIR/ui.sh"
# shellcheck disable=SC1091
source "$__DIR/state.sh"

profile_list() {
  local root="$__DIR/.."
  local file="$root/owb-bin/owb-esp/src/profile/mod.rs"
  [[ -f "$file" ]] || return 1
  if command -v rg &>/dev/null; then
    rg -n 'name: "[^"]+"' "$file" | sed -E 's/.*name: "([^"]+)".*/\1/'
  else
    grep -E 'name: "[^"]+"' "$file" | sed -E 's/.*name: "([^"]+)".*/\1/'
  fi
}

select_profile() {
  local profiles=()
  local board="${BOARD:-esp32s3}"
  while IFS= read -r name; do
    [[ -n "$name" ]] && profiles+=("$name")
  done < <(profile_list "$board")

  if (( ${#profiles[@]} == 0 )); then
    warn "No IO profiles found for board '$board'."
    return 0
  fi

  if [[ -n "${IO_PROFILE-}" ]]; then
    local found=false
    if command -v rg &>/dev/null; then
      if printf '%s\n' "${profiles[@]}" | rg -q "^${IO_PROFILE}$"; then
        found=true
      fi
    else
      if printf '%s\n' "${profiles[@]}" | grep -Fxq "$IO_PROFILE"; then
        found=true
      fi
    fi
    if [[ "$found" == true ]]; then
      note "IO profile: ${BOLD}$IO_PROFILE${RESET}"
    else
      warn "IO profile '$IO_PROFILE' not in list; using as-is."
    fi
    export OWB_IO_PROFILE="$IO_PROFILE"
    return 0
  fi

  section "Select IO profile"
  local sel
  sel=$(choose "profile" "${profiles[@]}") || return 1
  IO_PROFILE="$sel"
  export IO_PROFILE
  export OWB_IO_PROFILE="$IO_PROFILE"
}
