#!/usr/bin/env bash
set -euo pipefail

# Expects env: RUST_VERSION, MAKER, BOARD, BIN, TRIPLE, SSID, PASSWORD,
# ATTACH_MONITOR, PORT, optionally OWB_REAL_EXECUTORS=1 / OWB_HIL_LOW_MODE=1
# BOARD is always "esp32s3" — the only supported owb-esp target.

select_docker_target_dir() {
  local preferred="target/docker/${MAKER}/${BOARD}"
  local fallback=".owb_target/docker/${MAKER}/${BOARD}"
  local parent

  if [[ -n "${OWB_CARGO_TARGET_DIR:-}" ]]; then
    printf '%s\n' "${OWB_CARGO_TARGET_DIR}"
    return 0
  fi

  parent=$(dirname "$preferred")
  if [[ -d "$parent" ]]; then
    if [[ -w "$parent" ]]; then
      printf '%s\n' "$preferred"
      return 0
    fi
  elif [[ -d "target" && -w "target" ]]; then
    printf '%s\n' "$preferred"
    return 0
  elif [[ ! -e "target" ]]; then
    printf '%s\n' "$preferred"
    return 0
  fi

  mkdir -p "$fallback"
  note "Using fallback Docker cargo target dir: ${fallback}" >&2
  printf '%s\n' "$fallback"
}

maker_build() {
  if ! command -v docker &>/dev/null; then
    err "Docker not found. Please install & start Docker."
    return 1
  fi
  local profile_camera="true"
  local owb_has_camera="true"
  local owb_ui_variant="${OWB_UI_VARIANT:-v1}"
  if [[ "${OWB_IO_PROFILE:-}" == "owb-esp32s3-v1-alpha" && -z "${OWB_UI_VARIANT:-}" ]]; then
    owb_ui_variant="v1-alpha"
  fi
  if ! profile_has_camera "${BOARD:-}" "${OWB_IO_PROFILE:-}"; then
    profile_camera="false"
    owb_has_camera="false"
  fi
  # Frontend assets are now sourced from the `owb-ui` workspace crate
  # (plain HTML/CSS/JS, no gzip step required). The legacy
  # build_frontend_assets.py helper has been removed.
  export OWB_HAS_CAMERA="$owb_has_camera"
  export OWB_UI_VARIANT="$owb_ui_variant"
  note "Frontend UI variant: ${owb_ui_variant}"
  OWB_CARGO_TARGET_DIR="$(select_docker_target_dir)"
  export OWB_CARGO_TARGET_DIR
  local -a docker_env=(
    --rm
    -v "$(pwd)":/workspace
    --group-add "$(id -g)"
    -e SSID="${SSID-}"
    -e PASSWORD="${PASSWORD-}"
    -e CARGO_TARGET_DIR="/workspace/${OWB_CARGO_TARGET_DIR}"
    --entrypoint sh
  )
  note "Docker cargo target dir: ${OWB_CARGO_TARGET_DIR}"
  if [[ -n "${OWB_DEBUG_I2C-}" ]]; then
    docker_env+=(-e OWB_DEBUG_I2C="${OWB_DEBUG_I2C}")
  fi
  if [[ -n "${OWB_LOG_LEVEL-}" ]]; then
    docker_env+=(-e OWB_LOG_LEVEL="${OWB_LOG_LEVEL}")
  fi
  if [[ -n "${OWB_DUMP_PACKETS-}" ]]; then
    # WiFi diagnostic: esp-radio logs every frame (rx/tx) at the driver layer
    # via an info! statement (ESP_RADIO_CONFIG_DUMP_PACKETS). Pairs with the
    # dhcp.rs packet logging to localize the SoftAP unicast-TCP delivery gap —
    # i.e. whether a TCP SYN to 192.168.4.1:80 ever reaches the driver/smoltcp.
    # Very verbose; opt-in only. Surfaces at the default info log level.
    docker_env+=(-e ESP_RADIO_CONFIG_DUMP_PACKETS="true")
  fi
  # Build-time security/AP `option_env!` vars (owb-esp.rs). Passed through only
  # when set so the SoftAP fallback / control-auth can be exercised on the
  # bench without editing source (e.g. OWB_ALLOW_OPEN_AP=1 to bring up an open
  # SoftAP, AP_SSID/AP_PASSWORD for a WPA2 AP, CONTROL_KEY for the auth gate).
  local ap_env_var
  for ap_env_var in OWB_ALLOW_OPEN_AP AP_SSID AP_PASSWORD CONTROL_KEY; do
    if [[ -n "${!ap_env_var-}" ]]; then
      docker_env+=(-e "${ap_env_var}=${!ap_env_var}")
    fi
  done
  if [[ -n "${OWB_IO_PROFILE-}" ]]; then
    docker_env+=(-e OWB_IO_PROFILE="${OWB_IO_PROFILE}")
  fi
  local passthrough_var
  for passthrough_var in \
    OWB_DEVKIT_MTR_SLEEP \
    OWB_DEVKIT_MTR1_PHASE \
    OWB_DEVKIT_MTR1_DRIVE \
    OWB_DEVKIT_MTR2_PHASE \
    OWB_DEVKIT_MTR2_DRIVE \
    OWB_DEVKIT_MTR3_PHASE \
    OWB_DEVKIT_MTR3_DRIVE \
    OWB_DEVKIT_ENC1_A \
    OWB_DEVKIT_ENC1_B \
    OWB_DEVKIT_ENC2_A \
    OWB_DEVKIT_ENC2_B \
    OWB_DEVKIT_ENC3_A \
    OWB_DEVKIT_ENC3_B \
    OWB_DEVKIT_LED_PIN
  do
    if [[ -n "${!passthrough_var-}" ]]; then
      docker_env+=(-e "${passthrough_var}=${!passthrough_var}")
    fi
  done
  local -a cargo_features=()
  if [[ "${OWB_FORCE_NO_CAMERA:-0}" == "1" ]]; then
    cargo_features+=("no-camera")
    note "No-camera compile path forced (OWB_FORCE_NO_CAMERA=1)"
  elif [[ "$profile_camera" == "false" ]]; then
    cargo_features+=("no-camera")
    note "No-camera compile path selected for profile ${OWB_IO_PROFILE:-unknown}"
  fi
  if [[ "${OWB_HIL_LOW_MODE:-}" == "1" ]]; then
    docker_env+=(-e OWB_HIL_LOW_MODE="1")
  fi
  if [[ "${OWB_DEV_TELEMETRY:-}" == "1" ]]; then
    cargo_features+=("dev-telemetry")
    note "Dev telemetry: compiling debug/trace tracing events into the release build"
  fi
  local cargo_extra=""
  if [[ "${OWB_REAL_EXECUTORS:-}" == "1" ]]; then
    if profile_supports_real_executors "${BOARD:-}" "${OWB_IO_PROFILE:-}"; then
      docker_env+=(-e OWB_REAL_EXECUTORS="1")
      cargo_features+=("real-executors")
    else
      note "Skipping real-executors for profile ${OWB_IO_PROFILE:-unknown}; shadow runtime keeps the frontend/network image on the supported path."
    fi
  fi
  if (( ${#cargo_features[@]} > 0 )); then
    local feature_csv
    feature_csv=$(IFS=,; echo "${cargo_features[*]}")
    cargo_extra=" --features ${feature_csv}"
  fi

  docker run "${docker_env[@]}" \
    espressif/idf-rust:all_"${RUST_VERSION}".0 \
    -c "\
      . /home/esp/export-esp.sh && \
      rustc --version --verbose && \
      rustup show active-toolchain && \
      cd /workspace/owb-bin/""$MAKER"" && \
      rustup run esp cargo ""$BOARD"" --bin ""$BIN"" --ignore-rust-version -q --color=always${cargo_extra}\
    "
    # Make sure to remove `-ignore-rust-version` when 1.93 is implemented for container.
}

maker_flash() {
  if ! command -v espflash &>/dev/null; then
    err "Please install espflash: cargo install espflash"
    return 1
  fi
  local FIRMWARE
  FIRMWARE=$(cargo_release_artifact_path) || { err "Missing build context for ESP flash."; return 1; }
  [[ -f "$FIRMWARE" ]] || { err "Missing $FIRMWARE"; return 1; }
  local attach="${ATTACH_MONITOR:-true}"
  if [[ "$attach" == true ]]; then
    espflash flash --monitor --port "$PORT" "$FIRMWARE"
  else
    espflash flash --port "$PORT" "$FIRMWARE"
  fi
}
