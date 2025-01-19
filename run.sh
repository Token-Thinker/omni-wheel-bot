#!/usr/bin/env bash
set -e

function usage() {
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  --build     Only build firmware, skip flashing."
    echo "  --flash     Only flash firmware, skip building."
    echo "  -h, --help       Display this help message."
    exit 0
}

# Default actions
BUILD=true
FLASH=true

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            usage
            ;;
        --build)
            BUILD=true
            FLASH=false
            shift
            ;;
        --flash)
            BUILD=false
            FLASH=true
            shift
            ;;
        *)
            echo "Unknown option: $1"
            usage
            ;;
    esac
done

# Build Phase
if $BUILD; then
    echo "Building project..."

    echo "Building Container"
    docker build -t esp-owb .

    echo "Building Image"
    docker run --rm -it \
      -v "$(pwd)":/owb \
      -w /owb \
      esp-owb \
      /bin/bash -l -c "cargo +esp build -q --release --target xtensa-esp32-none-elf"

    echo "Build complete!"
fi

# Flash Phase
if $FLASH; then
    # Check if espflash is installed locally
    if ! command -v espflash >/dev/null 2>&1; then
        echo "espflash is not installed. Please install it (e.g., via pip or cargo)."
        exit 1
    fi

    # Dynamic Port Detection
    PORT=""
    if [[ "$(uname)" == "Darwin" ]]; then
        PORT=$(ls /dev/cu.usbserial* 2>/dev/null | head -n 1)
    elif [[ "$(uname)" == "Linux" ]]; then
        PORT=$(ls /dev/ttyUSB* 2>/dev/null | head -n 1)
    fi

    if [ -z "$PORT" ]; then
        echo "No suitable serial port found automatically. Please specify the port manually."
        exit 1
    fi

    echo "Using serial port: $PORT"

    echo "Flashing firmware to ESP32 on port $PORT..."
    espflash flash --port "$PORT" target/xtensa-esp32-none-elf/release/omni-wheel

    echo "Flashing complete!"
fi
