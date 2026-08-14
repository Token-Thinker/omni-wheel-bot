#!/usr/bin/env bash
set -euo pipefail

__DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# shellcheck disable=SC1091
source "$__DIR/lib.sh"

list_catalog() {
  section "Available Makers"
  local makers=()
  local d
  for d in owb-bin/*/; do
    [[ -d $d ]] && makers+=("$(basename "$d")")
  done
  if ((${#makers[@]}==0)); then
    warn "No makers found under owb-bin/"
    return 0
  fi

  for m in "${makers[@]}"; do
    echo -e "${BOLD}- $m${RESET}"
    local boards bins
    boards=$(boards_for_maker "$m" || true)
    if [[ -n "$boards" ]]; then
      note "Boards: $boards"
    else
      warn "Boards: unknown (add in scripts/lib.sh)"
    fi
    bins=()
    local name
    while IFS= read -r name; do
      [[ -n "$name" ]] && bins+=("$name")
    done < <(binaries_for_maker "$m" || true)
    if ((${#bins[@]}>0)); then
      note "Binaries: ${bins[*]}"
    else
      warn "Binaries: none found"
    fi
  done
}

probe_env() {
  section "Probe Environment"
  note "Rust (workspace): $(sed -n 's/^rust-version *= *"\([^"]\+\)".*/\1/p' owb-core/Cargo.toml 2>/dev/null || echo unknown)"

  local ports
  mapfile -t ports < <(list_serial_ports || true)
  if ((${#ports[@]}==0)); then
    warn "Serial ports: none detected"
  elif ((${#ports[@]}==1)); then
    ok "Serial port: ${ports[0]}"
  else
    note "Serial ports:"
    local p
    for p in "${ports[@]}"; do echo "  - $p"; done
  fi

  section "Installed Tools"
  for t in docker espflash screen elf2uf2-rs openocd dfu-util; do
    if command -v "$t" &>/dev/null; then ok "$t"; else warn "$t not found"; fi
  done
}
