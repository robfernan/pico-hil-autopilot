#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FIRMWARE_DIR="$SCRIPT_DIR/firmware"
PICO_SDK_PATH="${PICO_SDK_PATH:-$HOME/.pico-sdk/pico-sdk}"

echo "==============================================================="
echo "  Building Aero-RTOS RP2350 Firmware (Pico 2 / Pico 2 W)"
echo "==============================================================="
echo "Using Pico SDK at: $PICO_SDK_PATH"

if [ ! -d "$PICO_SDK_PATH" ]; then
    echo "[ERROR] Pico SDK not found at $PICO_SDK_PATH"
    exit 1
fi

mkdir -p "$FIRMWARE_DIR/build_rp2350"
cd "$FIRMWARE_DIR/build_rp2350"

cmake -DPICO_PLATFORM=rp2350 -DPICO_BOARD=pico2 -DPICO_SDK_PATH="$PICO_SDK_PATH" ..
make -j$(nproc)

echo ""
echo "[SUCCESS] Built firmware binaries:"
ls -lh "$FIRMWARE_DIR/build_rp2350/pico_hil_autopilot.uf2"
echo ""
echo "To flash to Pico 2 W:"
echo "  1. Hold the BOOTSEL button while plugging in the Pico 2 W"
echo "  2. Run: cp \"$FIRMWARE_DIR/build_rp2350/pico_hil_autopilot.uf2\" /media/$USER/RP2350/"
echo "==============================================================="
