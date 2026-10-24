#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/ground_station/build"

if [ ! -f "$BUILD_DIR/aero_ground_station" ]; then
    echo "[BUILD] Compiling Ground Station Cockpit..."
    mkdir -p "$BUILD_DIR"
    cd "$BUILD_DIR"
    cmake ..
    make -j$(nproc)
fi

echo "[RUN] Launching Aero-RTOS Ground Station Cockpit..."
cd "$BUILD_DIR"
exec ./aero_ground_station
