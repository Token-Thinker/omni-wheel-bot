#!/usr/bin/env bash
set -euo pipefail

# Shared colors and helpers
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
BOLD='\033[1m'
RESET='\033[0m'

section() { echo -e "${CYAN}${BOLD}▶ $*${RESET}"; }
ok() { echo -e "${GREEN}✔ $*${RESET}"; }
warn() { echo -e "${YELLOW}⚠ $*${RESET}"; }
err() { echo -e "${RED}✖ $*${RESET}"; }
note() { echo -e "${BLUE}• $*${RESET}"; }

is_dry_run() { [[ "${DRY_RUN:-false}" == true ]]; }

load_devkit_wiring_env() {
  local repo_root="${1:-.}"
  local env_file="${OWB_DEVKIT_WIRING_ENV:-$repo_root/config/devkit_wiring.local.env}"

  if [[ -f "$env_file" ]]; then
    set -a
    # shellcheck disable=SC1090
    source "$env_file"
    set +a
    export OWB_DEVKIT_WIRING_ENV_SOURCE="$env_file"
  fi
}

profile_supports_real_executors() {
  local board="${1:-}"
  local profile="${2:-}"

  case "$board" in
    esp32s3)
      case "$profile" in
        esp32-s3-devkitc-1-n32r16|owb-esp32s3-v1|owb-esp32s3-v1-alpha|owb-esp32s3-v2)
          return 0
          ;;
        *)
          return 1
          ;;
      esac
      ;;
    *)
      return 1
      ;;
  esac
}

profile_has_camera() {
  local board="${1:-}"
  local profile="${2:-}"

  case "$board" in
    esp32s3)
      case "$profile" in
        esp32-s3-devkitc-1-n32r16|owb-esp32s3-v1)
          return 1
          ;;
        xiao-seedstudio-sense-cam|owb-esp32s3-v2)
          return 0
          ;;
        *)
          return 0
          ;;
      esac
      ;;
    *)
      return 0
      ;;
  esac
}

detect_serial_port() {
  local OS candidates ports idx
  OS=$(uname)
  if [[ "$OS" == "Darwin" ]]; then
    candidates=(/dev/cu.* /dev/tty.*)
  else
    candidates=(/dev/ttyUSB* /dev/ttyACM*)
  fi

  ports=()
  for p in "${candidates[@]}"; do
    [[ -e $p ]] && ports+=("$p")
  done

  if (( ${#ports[@]} == 1 )); then
    echo "${ports[0]}"
  elif (( ${#ports[@]} > 1 )); then
    echo -e "${YELLOW}Multiple ports found:${RESET}"
    for i in "${!ports[@]}"; do
      printf "  [%d] %s\n" "$i" "${ports[$i]}"
    done
    read -r -p "$(echo -e "${YELLOW}""Select port #:${RESET} ")" idx
    echo "${ports[$idx]}"
  else
    return 1
  fi
}

list_serial_ports() {
  local OS candidates p
  OS=$(uname)
  if [[ "$OS" == "Darwin" ]]; then
    candidates=(/dev/cu.* /dev/tty.*)
  else
    candidates=(/dev/ttyUSB* /dev/ttyACM*)
  fi
  for p in "${candidates[@]}"; do
    [[ -e $p ]] && echo "$p"
  done
}

# Boards available for a given maker (space-separated)
boards_for_maker() {
  case "${1:-}" in
    owb-esp|esp|espressif)
      echo "esp32s3"
      ;;
    owb-host-sim|host|sim)
      echo "host"
      ;;
    *)
      echo ""
      ;;
  esac
}

# Returns target triple for a given maker + board
# Known makers: owb-esp, rp2040, stm32f4 (extensible)
target_triple_for() {
  local maker="${1:-}" board="${2:-}"
  case "$maker" in
    owb-esp|esp|espressif)
      case "$board" in
        esp32s3) echo xtensa-esp32s3-none-elf ;;
        *) return 1;;
      esac
      ;;
    owb-host-sim|host|sim)
      echo host
      ;;
    *) return 1 ;;
  esac
}

# Back-compat helper used by older code paths
target_triple_for_board() { target_triple_for owb-esp "$1"; }

package_name_for_maker() {
  local maker="${1:-}" cargo_file="owb-bin/${1:-}/Cargo.toml"
  [[ -f "$cargo_file" ]] || return 1
  sed -n 's/^name *= *"\([^"]\+\)".*/\1/p' "$cargo_file" | head -n1
}

# Print one Cargo bin target per line for the given maker crate.
binaries_for_maker() {
  local maker="${1:-}" crate_dir="owb-bin/${1:-}"
  local cargo_file="${crate_dir}/Cargo.toml"
  [[ -f "$cargo_file" ]] || return 1

  local bins=()
  local explicit
  explicit=$(
    awk '
      /^\[\[bin\]\]/ { in_bin=1; next }
      /^\[/ && $0 != "[[bin]]" { in_bin=0 }
      in_bin && /^[[:space:]]*name[[:space:]]*=/ {
        if (match($0, /"[^"]+"/)) {
          print substr($0, RSTART + 1, RLENGTH - 2)
        }
      }
    ' "$cargo_file"
  )

  if [[ -n "$explicit" ]]; then
    while IFS= read -r name; do
      [[ -n "$name" ]] && bins+=("$name")
    done <<<"$explicit"
  fi

  if [[ ${#bins[@]} -eq 0 && -f "${crate_dir}/src/main.rs" ]]; then
    local pkg
    pkg=$(package_name_for_maker "$maker" || true)
    [[ -n "$pkg" ]] && bins+=("$pkg")
  fi

  local f
  for f in "${crate_dir}"/src/bin/*.rs; do
    [[ -f "$f" ]] && bins+=("$(basename "${f%.rs}")")
  done

  if [[ ${#bins[@]} -eq 0 ]]; then
    return 1
  fi

  printf '%s\n' "${bins[@]}" | awk '!seen[$0]++'
}

cargo_target_dir() {
  printf '%s\n' "${OWB_CARGO_TARGET_DIR:-target}"
}

cargo_release_artifact_path() {
  if [[ -z "${TRIPLE-}" || -z "${BIN-}" ]]; then
    return 1
  fi

  printf '%s/%s/release/%s\n' "$(cargo_target_dir)" "$TRIPLE" "$BIN"
}
