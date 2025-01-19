#!/usr/bin/env bash
set -e

# Function to display usage information
function usage() {
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  -b, --build         Only build the firmware, skip flashing and monitoring."
    echo "  -f, --flash         Only flash the firmware, skip building and monitoring."
    echo "  -m, --monitor  Only start monitoring using screen, skip building and flashing."
    echo "  -h, --help          Display this help message."
    exit 0
}

# Default actions: perform build and flash by default unless specified otherwise
BUILD=true
FLASH=true
MONITOR_ONLY=false

# Parse command-line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            usage
            ;;
        -b|--build)
            BUILD=true
            FLASH=false
            MONITOR_ONLY=false
            shift
            ;;
        -f|--flash)
            BUILD=false
            FLASH=true
            MONITOR_ONLY=false
            shift
            ;;
        -m|--monitor)
            BUILD=false
            FLASH=false
            MONITOR_ONLY=true
            shift
            ;;
        *)
            echo "Unknown option: $1"
            usage
            ;;
    esac
done

# Utility function to check if a command exists
function command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Verify Docker installation
if ! command_exists docker; then
    echo "Error: Docker is not installed or not found in PATH."
    echo "Please install Docker from https://www.docker.com/get-started and ensure it's running."
    exit 1
fi

# Function to detect serial port
function detect_serial_port() {
    local OS_TYPE
    OS_TYPE="$(uname)"
    local PORTS=()

    if [[ "$OS_TYPE" == "Darwin" ]]; then
        # macOS: Look for /dev/cu.usbserial*
        PORTS=($(ls /dev/cu.usbserial* 2>/dev/null))
    elif [[ "$OS_TYPE" == "Linux" ]]; then
        # Linux: Look for /dev/ttyUSB*
        PORTS=($(ls /dev/ttyUSB* 2>/dev/null))
    else
        echo "Unsupported OS: $OS_TYPE"
        echo "Please specify the serial port manually."
        return 1
    fi

    if [[ ${#PORTS[@]} -eq 0 ]]; then
        echo "No serial ports found matching expected patterns."
        return 1
    elif [[ ${#PORTS[@]} -eq 1 ]]; then
        echo "${PORTS[0]}"
        return 0
    else
        echo "Multiple serial ports found:"
        for i in "${!PORTS[@]}"; do
            echo "  [$i] ${PORTS[$i]}"
        done
        read -p "Select the port number to use: " PORT_INDEX
        if [[ "$PORT_INDEX" =~ ^[0-9]+$ ]] && [[ "$PORT_INDEX" -ge 0 && "$PORT_INDEX" -lt ${#PORTS[@]} ]]; then
            echo "${PORTS[$PORT_INDEX]}"
            return 0
        else
            echo "Invalid selection."
            return 1
        fi
    fi
}

# Build Phase
if $BUILD; then
    echo "=============================="
    echo "        Building Project"
    echo "=============================="

    echo "Building Docker Image 'esp-owb'..."
    docker build -q -t esp-owb .

    echo "Running Docker Container to Build Firmware..."
    docker run --rm -it \
      -v "$(pwd)":/owb \
      -w /owb \
      esp-owb \
      /bin/bash -l -c "export PATH=\"/home/esp/.rustup/toolchains/esp/xtensa-esp-elf/esp-14.2.0_20240906/xtensa-esp-elf/bin:/home/esp/.cargo/bin:\$PATH\" && cargo +esp build -q --release --target xtensa-esp32-none-elf"

    echo "=============================="
    echo "         Build Complete"
    echo "=============================="
fi

# Flash Phase
if $FLASH; then
    echo "=============================="
    echo "        Flashing Firmware"
    echo "=============================="

    # Verify espflash installation
    if ! command_exists espflash; then
        echo "Error: 'espflash' is not installed."
        echo "Please install it using one of the following methods:"
        echo "  - Via Cargo: cargo install espflash"
        echo "  - Via Pip: pip install espflash"
        exit 1
    fi

    # Detect serial port
    PORT=""
    PORT=$(detect_serial_port) || {
        echo "Attempting manual port entry."
        read -p "Enter the serial port (e.g., /dev/cu.usbserial-A50285BI or /dev/ttyUSB0): " PORT
        if [[ -z "$PORT" ]]; then
            echo "No serial port specified. Exiting."
            exit 1
        fi
    }

    echo "Using serial port: $PORT"

    # Verify that the firmware file exists
    FIRMWARE_PATH="target/xtensa-esp32-none-elf/release/omni-wheel"
    if [[ ! -f "$FIRMWARE_PATH" ]]; then
        echo "Error: Firmware file '$FIRMWARE_PATH' does not exist."
        echo "Please ensure the build phase completed successfully."
        exit 1
    fi

    # Flash the firmware
    echo "Flashing firmware to ESP32..."
    espflash flash --port "$PORT" "$FIRMWARE_PATH"

    echo "=============================="
    echo "         Flash Complete"
    echo "=============================="
fi

# Monitor Phase
if $MONITOR_ONLY; then
    echo "=============================="
    echo "         Starting Monitor"
    echo "=============================="

    # Verify screen installation
    if ! command_exists screen; then
        echo "Warning: 'screen' is not installed. Install it to view serial output."
        exit 1
    fi

    # Detect serial port for monitoring
    PORT=""
    PORT=$(detect_serial_port) || {
        echo "Attempting manual port entry for monitoring."
        read -p "Enter the serial port for monitoring: " PORT
        if [[ -z "$PORT" ]]; then
            echo "No serial port specified. Exiting."
            exit 1
        fi
    }

    echo "Starting screen monitor on port $PORT at 115200 baud..."
    export TERM=xterm  # Set TERM to a simpler value
    screen "$PORT" 115200
fi
