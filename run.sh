#!/usr/bin/env bash
set -euo pipefail

# Color constants
RED='\033[0;31m' GREEN='\033[0;32m' YELLOW='\033[0;33m'
BLUE='\033[0;34m' BOLD='\033[1m' RESET='\033[0m'


BUILD=true
FLASH=true
MONITOR=false
DEBUG=false

usage() {
    echo -e "${BOLD}Usage:${RESET} $0 [options]"
    echo
    echo -e "${BOLD}Options:${RESET}"
    echo -e "  ${YELLOW}-b, --build${RESET}     Build only (no flash, no monitor)"
    echo -e "  ${YELLOW}-f, --flash${RESET}     Flash only (no build, no monitor)"
    echo -e "  ${YELLOW}-m, --monitor${RESET}   Monitor only (no build, no flash)"
    echo -e "  ${YELLOW}-d, --debug${RESET}     Turn on debug/confirmation of creds"
    echo -e "  ${YELLOW}-h, --help${RESET}      Show this help"
    exit 0
}

while [[ $# -gt 0 ]]; do
    case $1 in
        -b|--build)
            BUILD=true
            FLASH=false
            MONITOR=false
            ;;
        -f|--flash)
            BUILD=false
            FLASH=true
            MONITOR=false
            ;;
        -m|--monitor)
            BUILD=false
            FLASH=false
            MONITOR=true
            ;;
        -d|--debug)
            DEBUG=true
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

RUST_VERSION=$(
  sed -n 's/^rust-version *= *"\([^"]\+\)".*/\1/p' Cargo.toml
)




if [[ -n "${SUDO_USER-}" ]]; then
    USER_HOME=$(eval echo "~$SUDO_USER")
    export PATH="$USER_HOME/.cargo/bin:$PATH"
else
    export PATH="$HOME/.cargo/bin:$PATH"
fi


if $BUILD; then
    echo -e "${BLUE}${BOLD}▶ Why we ask for your Wi-Fi password${RESET}"
    echo -e "  • Injected only into this script’s environment"
    echo -e "  • Never written to disk or git"
    echo

    # Read Wi-Fi credentials from environment or prompt if missing
    if [ -z "${SSID-}" ] || [ -z "${PASSWORD-}" ]; then
        read -p "$(echo -e ${YELLOW}Enter Wi-Fi SSID:${RESET} )" SSID
        read -s -p "$(echo -e ${YELLOW}Enter Wi-Fi Password:${RESET} )" PASSWORD
        echo
    fi
    export SSID PASSWORD

    # Debug confirmation
    if $DEBUG; then
        echo -e "${BLUE}${BOLD}[DEBUG] You entered:${RESET}"
        echo -e "  SSID:     ${GREEN}$SSID${RESET}"
        echo -e "  Password: ${GREEN}(hidden)${RESET}"
        read -p "$(echo -e ${YELLOW}Proceed with these? [y/N]:${RESET} )" CONF
        case "$CONF" in
            [Yy]|[Yy][Ee][Ss])
                echo -e "${GREEN}Continuing…${RESET}"
                ;;
            *)
                echo -e "${RED}Aborted by user.${RESET}"
                exit 1
                ;;
        esac
    fi

    # Check Docker
    if ! command -v docker &>/dev/null; then
        echo -e "${RED}Error:${RESET} Docker not found. Please install & start Docker."
        exit 1
    fi
    echo
fi


if $BUILD || $FLASH; then
    echo -e "${BLUE}${BOLD}▶ Select target board:${RESET}"
    BOARD_OPTIONS=(esp32 esp32s3)
    PS3="$(echo -e "${YELLOW}"Choice [1-${#BOARD_OPTIONS[@]}]:"${RESET}" )"
    select BOARD in "${BOARD_OPTIONS[@]}"; do
        [[ -n "$BOARD" ]] && break
        echo -e "${RED}Invalid selection.${RESET} Try again."
    done

    declare -A TARGET_TRIPLES=(
        [esp32]=xtensa-esp32-none-elf
        [esp32c2]=riscv32imc-unknown-none-elf
        [esp32c3]=riscv32imc-unknown-none-elf
        [esp32c6]=riscv32imac-unknown-none-elf
        [esp32h2]=riscv32imac-unknown-none-elf
        [esp32p4]=xtensa-esp32p4-none-elf
        [esp32s2]=xtensa-esp32s2-none-elf
        [esp32s3]=xtensa-esp32s3-none-elf
    )
    TRIPLE="${TARGET_TRIPLES[$BOARD]}"
    if [[ -z "$TRIPLE" ]]; then
        echo -e "${RED}❌ Unknown board '$BOARD'${RESET}"
        exit 1
    fi
    echo

    echo -e "${BLUE}${BOLD}▶ Select firmware binary:${RESET}"
    mapfile -t BIN_OPTIONS < <(cd src/bin && find -- *.rs | sed 's/\.rs$//')
    PS3="$(echo -e "${YELLOW}"Choice [1-${#BIN_OPTIONS[@]}]:"${RESET}" )"
    select BIN in "${BIN_OPTIONS[@]}"; do
        [[ -n "$BIN" ]] && break
        echo -e "${RED}Invalid selection.${RESET} Try again."
    done
    echo
fi


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


if $BUILD; then
    echo -e "${GREEN}${BOLD}=== BUILD PHASE (${BOARD}/${BIN}) ===${RESET}"

    docker run --rm \
      -v "$(pwd)":/workspace \
      -e SSID="$SSID" \
      -e PASSWORD="$PASSWORD" \
      -e LIBCLANG_PATH="/home/esp/.rustup/toolchains/esp/xtensa-esp32-elf-clang/esp-19.1.2_20250225/esp-clang/lib" \
      -e PATH="/home/esp/.rustup/toolchains/esp/xtensa-esp-elf/esp-14.2.0_20240906/xtensa-esp-elf/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/home/esp/.cargo/bin" \
      --entrypoint sh \
      espressif/idf-rust:esp32_"${RUST_VERSION}".0 \
      -c "\
        rustc --version --verbose && \
        rustup show active-toolchain && \
        cd /workspace && \
        rustup run esp cargo \"$BOARD\" --bin \"$BIN\" -q --color=always\
      "

    echo -e "${GREEN}${BOLD}=== BUILD COMPLETE ===${RESET}"
    echo
fi


if $FLASH; then
    echo -e "${GREEN}${BOLD}=== FLASH PHASE (host) ===${RESET}"
    if ! command -v espflash &>/dev/null; then
        echo -e "${RED}Please install espflash:${RESET} cargo install espflash"
        exit 1
    fi

    PORT=$(detect_serial_port) || {
      read -p "$(echo -e "${YELLOW}""Serial port (e.g. /dev/ttyUSB0):${RESET} ")" PORT
      [[ -n "$PORT" ]] || { echo "No port, abort."; exit 1; }
    }

    FIRMWARE="target/${TRIPLE}/release/${BIN}"
    if [[ ! -f "$FIRMWARE" ]]; then
        echo -e "${RED}Missing $FIRMWARE${RESET}"
        exit 1
    fi

    espflash flash --monitor --port "$PORT" "$FIRMWARE"
    echo -e "${GREEN}${BOLD}=== FLASH COMPLETE ===${RESET}"
    echo
fi


if $MONITOR; then
    echo -e "${GREEN}${BOLD}=== MONITOR ===${RESET}"
    if ! command -v screen &>/dev/null; then
        echo -e "${RED}Install screen to monitor serial.${RESET}"
        exit 1
    fi

    PORT=$(detect_serial_port) || {
        read -p "$(echo -e "${YELLOW}"Serial port:"${RESET}" )" PORT
        [[ -n "$PORT" ]] || exit 1
    }

    screen "$PORT" 115200
    echo
fi