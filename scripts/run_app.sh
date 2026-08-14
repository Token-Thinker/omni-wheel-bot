#!/usr/bin/env bash
set -euo pipefail

# Source helpers
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# shellcheck disable=SC1091
source "$SCRIPT_DIR/lib.sh"
source "$SCRIPT_DIR/prompt_wifi.sh"
source "$SCRIPT_DIR/select_profile.sh"
source "$SCRIPT_DIR/select_target.sh"
source "$SCRIPT_DIR/build.sh"
source "$SCRIPT_DIR/flash.sh"
source "$SCRIPT_DIR/monitor.sh"
source "$SCRIPT_DIR/info.sh"
source "$SCRIPT_DIR/probe.sh"
source "$SCRIPT_DIR/ui.sh"
source "$SCRIPT_DIR/state.sh"

load_devkit_wiring_env "$SCRIPT_DIR/.."

echo -e "${BLUE}${BOLD}Omni‑Wheel Bot Deployer${RESET} ${CYAN}— build • flash • monitor${RESET}"


BUILD=true
FLASH=true
MONITOR=false
MODE_BUILD=false
MODE_FLASH=false
MODE_MONITOR=false
DEBUG=false
PWM_DEBUG=false
REAL_EXECUTORS=false
HIL_LOW_MODE=false
HIL_LOW_MODE_SET=false
ATTACH_MONITOR=true
MAKER=""
BOARD=""
BIN=""
PORT=""
SSID="${SSID-}"
PASSWORD="${PASSWORD-}"
DRY_RUN=false
TARGET_FLAGS_SET=false
TUI=auto
USE_TEST_NETWORK=false
AUTO_FLASH_ON_TEST=false
FORCE_SOFTAP=false

usage() {
    echo -e "${BOLD}Usage:${RESET} $0 [options]"
    echo
    echo -e "${BOLD}Modes:${RESET}"
    echo -e "  ${YELLOW}-b, --build${RESET}           Build only (no flash, no monitor)"
    echo -e "  ${YELLOW}-f, --flash${RESET}           Flash only (no build, no monitor)"
    echo -e "  ${YELLOW}-m, --monitor${RESET}         Monitor only (no build, no flash)"
    echo
    echo -e "${BOLD}Non-interactive:${RESET}"
    echo -e "  ${YELLOW}    --maker NAME${RESET}       Maker directory under owb-bin (e.g., owb-esp)"
    echo -e "  ${YELLOW}    --board NAME${RESET}       Board id (e.g., esp32, esp32s3)"
    echo -e "  ${YELLOW}    --bin NAME${RESET}         Binary name (file without .rs)"
    echo -e "  ${YELLOW}    --port PATH${RESET}        Serial port (e.g., /dev/ttyUSB0)"
    echo -e "  ${YELLOW}    --ssid SSID${RESET}        Wi‑Fi SSID"
    echo -e "  ${YELLOW}    --password PASS${RESET}    Wi‑Fi password"
    echo -e "  ${YELLOW}    --force-softap${RESET}     Skip STA credentials and serve OWB-BOT at 192.168.4.1"
    echo -e "  ${YELLOW}    --no-attach-monitor${RESET} Do not attach monitor during flash"
    echo -e "  ${YELLOW}    --list${RESET}             List makers, boards, and binaries"
    echo -e "  ${YELLOW}    --probe${RESET}            Probe environment and serial ports"
    echo -e "  ${YELLOW}    --probe-board${RESET}      Probe connected boards and suggest target mappings"
    echo -e "  ${YELLOW}    --dry-run${RESET}          Print commands without executing"
    echo -e "  ${YELLOW}    --tui${RESET}              Force TUI (fzf/whiptail) menus if available"
    echo -e "  ${YELLOW}    --no-tui${RESET}           Disable TUI; use basic prompts"
    echo -e "  ${YELLOW}    --last${RESET}             Use last selected maker/board/bin"
    echo -e "  ${YELLOW}    --use-test-network${RESET} Use SSID/PASSWORD from environment (requires .env sourced)"
    echo
    echo -e "${BOLD}General:${RESET}"
    echo -e "  ${YELLOW}-d, --debug${RESET}           Confirm Wi‑Fi creds"
    echo -e "  ${YELLOW}    --debug-i2c${RESET}        Enable live I2C/PWM telemetry (sets OWB_DEBUG_I2C=1)"
    echo -e "  ${YELLOW}    --real-executors${RESET}   Enable hardware-backed board executors for supported ESP targets"
    echo -e "  ${YELLOW}    --hil-low-mode${RESET}     Build with reduced motor authority for safer HIL bench tests"
    echo -e "  ${YELLOW}    --normal-drive-mode${RESET} Disable HIL low mode explicitly"
    echo -e "  ${YELLOW}    --auto-flash-on-test${RESET} Automatically flash after build when using test network"
    echo -e "  ${YELLOW}-h, --help${RESET}            Show this help"
    exit 0
}

prompt_hil_mode() {
    [[ "$BUILD" == true ]] || return 0
    [[ "${MAKER}" =~ ^(owb-esp|esp|espressif)$ ]] || return 0
    [[ "$TARGET_FLAGS_SET" == false ]] || return 0
    [[ "$HIL_LOW_MODE_SET" == false ]] || return 0

    local mode
    mode=$(choose "hil-mode" "normal-drive-mode" "hil-low-mode") || return 1
    case "$mode" in
        hil-low-mode)
            HIL_LOW_MODE=true
            ;;
        normal-drive-mode)
            HIL_LOW_MODE=false
            ;;
    esac
}

normalize_runtime_mode_for_profile() {
    [[ "${MAKER}" =~ ^(owb-esp|esp|espressif)$ ]] || return 0
    [[ "$REAL_EXECUTORS" == true ]] || return 0

    local profile_name="${IO_PROFILE:-${OWB_IO_PROFILE:-}}"
    if [[ -z "$profile_name" ]]; then
        return 0
    fi

    if ! profile_supports_real_executors "$BOARD" "$profile_name"; then
        warn "Profile ${profile_name} does not support hardware-backed executors; using shadow runtime for frontend/network bring-up."
        REAL_EXECUTORS=false
    fi
}

while [[ $# -gt 0 ]]; do
    case $1 in
        -b|--build)
            MODE_BUILD=true
            ;;
        -f|--flash)
            MODE_FLASH=true
            ;;
        -m|--monitor)
            MODE_MONITOR=true
            ;;
        -d|--debug)
            DEBUG=true
            ;;
        --debug-i2c)
            PWM_DEBUG=true
            ;;
        --real-executors)
            REAL_EXECUTORS=true
            ;;
        --hil-low-mode)
            HIL_LOW_MODE=true
            HIL_LOW_MODE_SET=true
            ;;
        --normal-drive-mode)
            HIL_LOW_MODE=false
            HIL_LOW_MODE_SET=true
            ;;
        --maker)
            MAKER="$2"; shift
            TARGET_FLAGS_SET=true
            ;;
        --board)
            BOARD="$2"; shift
            TARGET_FLAGS_SET=true
            ;;
        --bin)
            BIN="$2"; shift
            TARGET_FLAGS_SET=true
            ;;
        --port)
            PORT="$2"; shift
            ;;
        --ssid)
            SSID="$2"; shift
            ;;
        --password)
            PASSWORD="$2"; shift
            ;;
        --no-attach-monitor)
            ATTACH_MONITOR=false
            ;;
        --dry-run)
            DRY_RUN=true
            ;;
        --tui)
            TUI=true
            ;;
        --no-tui)
            TUI=false
            ;;
        --last)
            if load_last_selection 2>/dev/null; then
                TARGET_FLAGS_SET=true
            else
                warn "No last selection saved yet."
            fi
            ;;
        --use-test-network)
            USE_TEST_NETWORK=true
            ;;
        --force-softap)
            FORCE_SOFTAP=true
            ;;
        --auto-flash-on-test)
            AUTO_FLASH_ON_TEST=true
            ;;
        --list)
            list_catalog
            exit 0
            ;;
        --probe)
            probe_env
            exit 0
            ;;
        --probe-board)
            probe_connected_boards
            exit 0
            ;;
        -h|--help)
            usage
            ;;
        *)
            echo -e "${RED}Unknown option:${RESET} $1"
            usage
            ;;
    esac
    shift
done

if [[ "$MODE_MONITOR" == true && ( "$MODE_BUILD" == true || "$MODE_FLASH" == true ) ]]; then
    err "--monitor cannot be combined with --build/--flash."
    exit 1
fi

if [[ "$MODE_MONITOR" == true ]]; then
    BUILD=false
    FLASH=false
    MONITOR=true
elif [[ "$MODE_BUILD" == true && "$MODE_FLASH" == true ]]; then
    BUILD=true
    FLASH=true
    MONITOR=false
elif [[ "$MODE_BUILD" == true ]]; then
    BUILD=true
    FLASH=false
    MONITOR=false
elif [[ "$MODE_FLASH" == true ]]; then
    BUILD=false
    FLASH=true
    MONITOR=false
fi

RUST_VERSION=$(sed -n 's/^rust-version *= *"\([^"]\+\)".*/\1/p' owb-core/Cargo.toml)




if [[ -n "${SUDO_USER-}" ]]; then
    USER_HOME=$(eval echo "~$SUDO_USER")
    export PATH="$USER_HOME/.cargo/bin:$PATH"
    export CARGO_HOME="${CARGO_HOME:-$USER_HOME/.cargo}"
    export RUSTUP_HOME="${RUSTUP_HOME:-$USER_HOME/.rustup}"
else
    export PATH="$HOME/.cargo/bin:$PATH"
fi

if $BUILD || $FLASH; then
    select_target
fi

# Only ESP maker needs Wi‑Fi credentials during build
if $BUILD && [[ "${MAKER}" =~ ^(owb-esp|esp|espressif)$ ]]; then
    select_profile
    normalize_runtime_mode_for_profile
    if [[ "$FORCE_SOFTAP" == true ]]; then
        export OWB_FORCE_SOFTAP=1
    fi
    prompt_wifi
    prompt_hil_mode
    if [[ "${USE_TEST_NETWORK}" == true && "${AUTO_FLASH_ON_TEST}" == true ]]; then
        FLASH=true
    fi
fi


NEXT_ACTION=""

run_once() {
    NEXT_ACTION=""
    if $BUILD; then
        if [[ "$PWM_DEBUG" == true ]]; then
            export OWB_DEBUG_I2C=1
        else
            unset OWB_DEBUG_I2C || true
        fi
        if [[ "$REAL_EXECUTORS" == true ]]; then
            export OWB_REAL_EXECUTORS=1
        else
            unset OWB_REAL_EXECUTORS || true
        fi
        if [[ "$HIL_LOW_MODE" == true ]]; then
            export OWB_HIL_LOW_MODE=1
        else
            unset OWB_HIL_LOW_MODE || true
        fi
        build_firmware || return 1
    fi

    if $FLASH; then
        if [[ "$TARGET_FLAGS_SET" == false ]]; then
            read -r -p "$(echo -e "${YELLOW}Flash now, change target, or exit? [F/c/e]: ${RESET}")" ans
            case "${ans,,}" in
                c|change)
                    NEXT_ACTION="restart"; return 0 ;;
                e|exit|q|quit)
                    NEXT_ACTION="exit"; return 0 ;;
                ""|f|flash|y|yes)
                    : ;;
                *) : ;;
            esac
        fi
        flash_firmware || return 1
    fi

    if $MONITOR; then
        monitor_serial || return 1
    fi
}

EXIT_CODE=0

while :; do
    if ! run_once; then
        EXIT_CODE=1
        break
    fi
    case "$NEXT_ACTION" in
        restart)
            unset MAKER BOARD BIN TRIPLE PORT
            select_target
            if $BUILD && [[ "${MAKER}" =~ ^(owb-esp|esp|espressif)$ ]]; then
                select_profile
                normalize_runtime_mode_for_profile
                prompt_wifi
                prompt_hil_mode
            fi
            continue
            ;;
        exit)
            break
            ;;
        *)
            if [[ "$TARGET_FLAGS_SET" == false ]]; then
                read -r -p "$(echo -e "${YELLOW}Pick another board or exit? [a/E]: ${RESET}")" again
                case "${again,,}" in
                    a|again|y|yes)
                        unset MAKER BOARD BIN TRIPLE PORT
                        select_target
                        if $BUILD && [[ "${MAKER}" =~ ^(owb-esp|esp|espressif)$ ]]; then
                            select_profile
                            normalize_runtime_mode_for_profile
                            prompt_wifi
                            prompt_hil_mode
                        fi
                        continue
                        ;;
                    *) ;;
                esac
            fi
            break
            ;;
    esac
done

exit "$EXIT_CODE"
