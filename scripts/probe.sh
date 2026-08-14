#!/usr/bin/env bash
set -euo pipefail

__DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# shellcheck disable=SC1091
source "$__DIR/lib.sh"
# shellcheck disable=SC1091
source "$__DIR/ui.sh"
# shellcheck disable=SC1091
source "$__DIR/state.sh"

board_registry_file() {
  echo "$__DIR/../config/board_registry.tsv"
}

_sys_attr() {
  local file="${1:-}"
  [[ -f "$file" ]] || return 1
  tr -d '\n' < "$file"
}

_sysfs_tty_path() {
  local port="${1:-}" tty
  tty=$(basename "$port")
  [[ -e "/sys/class/tty/$tty" ]] || return 1
  readlink -f "/sys/class/tty/$tty/device"
}

_usb_parent_for_port() {
  local dir
  dir=$(_sysfs_tty_path "$1" 2>/dev/null || true)
  while [[ -n "${dir:-}" && "$dir" != "/" ]]; do
    if [[ -f "$dir/idVendor" && -f "$dir/idProduct" ]]; then
      echo "$dir"
      return 0
    fi
    dir=$(dirname "$dir")
  done
  return 1
}

_driver_name_for_port() {
  local tty driver_link
  tty=$(basename "$1")
  driver_link="/sys/class/tty/$tty/device/driver"
  [[ -L "$driver_link" ]] || return 1
  basename "$(readlink -f "$driver_link")"
}

_reset_probe_result() {
  PROBE_PORT=""
  PROBE_VENDOR_ID=""
  PROBE_PRODUCT_ID=""
  PROBE_MANUFACTURER=""
  PROBE_PRODUCT_NAME=""
  PROBE_SERIAL=""
  PROBE_DRIVER=""
  PROBE_LABEL="Unknown serial target"
  PROBE_MAKER=""
  PROBE_BOARD=""
  PROBE_BIN=""
  PROBE_PROFILE=""
  PROBE_MATCH_SOURCE=""
  PROBE_MATCH_CONFIDENCE="unknown"
}

_apply_probe_mapping() {
  PROBE_MAKER="${1:-}"
  PROBE_BOARD="${2:-}"
  PROBE_BIN="${3:-}"
  PROBE_PROFILE="${4:-}"
  PROBE_LABEL="${5:-$PROBE_LABEL}"
  PROBE_MATCH_SOURCE="${6:-$PROBE_MATCH_SOURCE}"
  PROBE_MATCH_CONFIDENCE="${7:-$PROBE_MATCH_CONFIDENCE}"
}

_registry_match_probe() {
  local file line kind value maker board bin profile label
  file=$(board_registry_file)
  [[ -f "$file" ]] || return 1

  while IFS= read -r line || [[ -n "$line" ]]; do
    kind=$(awk -F '\t' '{print $1}' <<< "$line")
    value=$(awk -F '\t' '{print $2}' <<< "$line")
    maker=$(awk -F '\t' '{print $3}' <<< "$line")
    board=$(awk -F '\t' '{print $4}' <<< "$line")
    bin=$(awk -F '\t' '{print $5}' <<< "$line")
    profile=$(awk -F '\t' '{print $6}' <<< "$line")
    label=$(awk -F '\t' '{print $7}' <<< "$line")

    [[ -n "${kind:-}" ]] || continue
    [[ "${kind:0:1}" == "#" ]] && continue

    case "$kind" in
      serial)
        [[ -n "$PROBE_SERIAL" && "${PROBE_SERIAL,,}" == "${value,,}" ]] || continue
        ;;
      vidpid)
        [[ -n "$PROBE_VENDOR_ID" && -n "$PROBE_PRODUCT_ID" ]] || continue
        [[ "${PROBE_VENDOR_ID,,}:${PROBE_PRODUCT_ID,,}" == "${value,,}" ]] || continue
        ;;
      port)
        [[ "$PROBE_PORT" == "$value" ]] || continue
        ;;
      product_contains)
        [[ -n "$PROBE_PRODUCT_NAME" && "${PROBE_PRODUCT_NAME,,}" == *"${value,,}"* ]] || continue
        ;;
      manufacturer_contains)
        [[ -n "$PROBE_MANUFACTURER" && "${PROBE_MANUFACTURER,,}" == *"${value,,}"* ]] || continue
        ;;
      *)
        continue
        ;;
    esac

    _apply_probe_mapping "$maker" "$board" "$bin" "$profile" "$label" "registry:$kind" "high"
    return 0
  done < "$file"

  return 1
}

_heuristic_match_probe() {
  local product_lc manufacturer_lc
  product_lc="${PROBE_PRODUCT_NAME,,}"
  manufacturer_lc="${PROBE_MANUFACTURER,,}"

  if [[ "$manufacturer_lc" == *seeed* || "$product_lc" == *xiao* ]]; then
    _apply_probe_mapping \
      "owb-esp" "esp32s3" "owb-esp" "xiao-seedstudio-sense-cam" \
      "Seeed XIAO ESP32S3 Sense" "heuristic" "high"
    return 0
  fi

  if [[ "$product_lc" == *devkitc-1* || "$product_lc" == *esp32-s3-devkitc-1* ]]; then
    _apply_probe_mapping \
      "owb-esp" "esp32s3" "owb-esp" "esp32-s3-devkitc-1-n32r16" \
      "Espressif ESP32-S3-DevKitC-1" "heuristic" "high"
    return 0
  fi

  if [[ "$product_lc" == *usb\ jtag/serial\ debug\ unit* ]]; then
    _apply_probe_mapping \
      "owb-esp" "esp32s3" "owb-esp" "" \
      "ESP32-S3 class device" "heuristic" "medium"
    return 0
  fi

  if [[ "$manufacturer_lc" == *espressif* || "$product_lc" == *esp32-s3* ]]; then
    _apply_probe_mapping \
      "owb-esp" "esp32s3" "owb-esp" "" \
      "ESP32-S3 class device" "heuristic" "medium"
    return 0
  fi

  if [[ "$PROBE_VENDOR_ID" == "10c4" || "$PROBE_VENDOR_ID" == "1a86" ]]; then
    _apply_probe_mapping \
      "owb-esp" "" "owb-esp" "" \
      "Generic USB serial bridge" "heuristic" "low"
    return 0
  fi

  return 1
}

probe_serial_port() {
  local port="${1:-}" usb_dir=""
  [[ -n "$port" ]] || return 1

  _reset_probe_result
  PROBE_PORT="$port"

  if usb_dir=$(_usb_parent_for_port "$port" 2>/dev/null || true); then
    PROBE_VENDOR_ID=$(_sys_attr "$usb_dir/idVendor" 2>/dev/null || true)
    PROBE_PRODUCT_ID=$(_sys_attr "$usb_dir/idProduct" 2>/dev/null || true)
    PROBE_MANUFACTURER=$(_sys_attr "$usb_dir/manufacturer" 2>/dev/null || true)
    PROBE_PRODUCT_NAME=$(_sys_attr "$usb_dir/product" 2>/dev/null || true)
    PROBE_SERIAL=$(_sys_attr "$usb_dir/serial" 2>/dev/null || true)
  fi

  PROBE_DRIVER=$(_driver_name_for_port "$port" 2>/dev/null || true)

  _registry_match_probe || _heuristic_match_probe || true
}

probe_summary() {
  local port="${1:-}" summary
  probe_serial_port "$port"

  summary="$PROBE_PORT"
  if [[ -n "$PROBE_LABEL" ]]; then
    summary+=" | $PROBE_LABEL"
  fi
  if [[ -n "$PROBE_PROFILE" ]]; then
    summary+=" | profile=$PROBE_PROFILE"
  elif [[ -n "$PROBE_BOARD" ]]; then
    summary+=" | board=$PROBE_BOARD"
  fi
  if [[ "$PROBE_MATCH_CONFIDENCE" != "unknown" ]]; then
    summary+=" | confidence=$PROBE_MATCH_CONFIDENCE"
  fi

  printf '%s\n' "$summary"
}

print_probe_report() {
  local port="${1:-}"
  probe_serial_port "$port"

  section "Board Probe"
  note "Port: $PROBE_PORT"
  [[ -n "$PROBE_LABEL" ]] && note "Detected: $PROBE_LABEL"
  [[ -n "$PROBE_MATCH_SOURCE" ]] && note "Match source: $PROBE_MATCH_SOURCE"
  [[ -n "$PROBE_MATCH_CONFIDENCE" ]] && note "Confidence: $PROBE_MATCH_CONFIDENCE"
  [[ -n "$PROBE_VENDOR_ID" || -n "$PROBE_PRODUCT_ID" ]] && note "USB VID:PID: ${PROBE_VENDOR_ID:-????}:${PROBE_PRODUCT_ID:-????}"
  [[ -n "$PROBE_MANUFACTURER" ]] && note "Manufacturer: $PROBE_MANUFACTURER"
  [[ -n "$PROBE_PRODUCT_NAME" ]] && note "Product: $PROBE_PRODUCT_NAME"
  [[ -n "$PROBE_SERIAL" ]] && note "Serial: $PROBE_SERIAL"
  [[ -n "$PROBE_DRIVER" ]] && note "Driver: $PROBE_DRIVER"
  [[ -n "$PROBE_MAKER" ]] && note "Recommended maker: $PROBE_MAKER"
  [[ -n "$PROBE_BOARD" ]] && note "Recommended board: $PROBE_BOARD"
  [[ -n "$PROBE_BIN" ]] && note "Recommended binary: $PROBE_BIN"
  [[ -n "$PROBE_PROFILE" ]] && note "Recommended IO profile: $PROBE_PROFILE"
  return 0
}

probe_connected_boards() {
  local ports=() port
  mapfile -t ports < <(list_serial_ports || true)

  if (( ${#ports[@]} == 0 )); then
    warn "No serial ports detected."
    return 0
  fi

  for port in "${ports[@]}"; do
    print_probe_report "$port"
    echo
  done
}

probe_and_select_target() {
  local ports=() options=() selected summary port action
  mapfile -t ports < <(list_serial_ports || true)

  if (( ${#ports[@]} == 0 )); then
    warn "No serial ports detected for probing."
    return 1
  fi

  if (( ${#ports[@]} == 1 )); then
    port="${ports[0]}"
  else
    for port in "${ports[@]}"; do
      summary=$(probe_summary "$port")
      options+=("$summary")
    done

    section "Probe Connected Board"
    selected=$(choose "probe-board" "${options[@]}") || return 1
    port="${selected%% | *}"
  fi

  print_probe_report "$port"
  PORT="$port"
  export PORT

  if [[ -n "$PROBE_MAKER" && -n "$PROBE_BOARD" && -n "$PROBE_BIN" ]]; then
    action=$(choose "probe-result" "use-probed-target" "manual-override" "cancel") || return 1
    case "$action" in
      use-probed-target)
        MAKER="$PROBE_MAKER"
        BOARD="$PROBE_BOARD"
        BIN="$PROBE_BIN"
        TRIPLE=$(target_triple_for "$MAKER" "$BOARD") || {
          err "Could not resolve target triple for probed target."
          return 1
        }

        if [[ -n "$PROBE_PROFILE" ]]; then
          IO_PROFILE="$PROBE_PROFILE"
          OWB_IO_PROFILE="$IO_PROFILE"
          export IO_PROFILE OWB_IO_PROFILE
        fi

        export MAKER BOARD BIN TRIPLE
        save_last_selection || true
        ok "Using probed target on $PORT"
        return 0
        ;;
      manual-override)
        note "Keeping detected port: $PORT"
        return 2
        ;;
      cancel)
        return 1
        ;;
    esac
  fi

  warn "Probe could not fully resolve a supported target; continuing with manual selection."
  return 2
}
