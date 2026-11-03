#!/usr/bin/env bash
# ==============================================================================
# start_zenoh_gcs.sh
# Portable startup script for GCS Zenoh Bridge (ros2dds)
# Works on any user account and any laptop without hardcoded paths.
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

PORT="7447"
if [ "$1" == "--help" ] || [ "$1" == "-h" ]; then
    echo "Usage: $0 [PORT] (default: 7447)"
    exit 0
elif [ -n "$1" ]; then
    PORT="$1"
fi
# Check if a Zenoh instance or another process is already listening on this port
EXISTING_PID=$(lsof -ti "tcp:${PORT}" 2>/dev/null || ss -tulpn 2>/dev/null | grep ":${PORT} " | grep -o 'pid=[0-9]*' | cut -d= -f2 | head -n1)
if [ -n "${EXISTING_PID}" ]; then
    echo "=========================================================="
    echo " [INFO] Zenoh Bridge router is ALREADY RUNNING on port ${PORT}!"
    echo " Process ID (PID): ${EXISTING_PID}"
    echo " You do NOT need to run it again. Everything is ready."
    echo " (If you ever want to restart it: kill ${EXISTING_PID})"
    echo "=========================================================="
    exit 0
fi

# 1. Search in user PATH
if command -v zenoh-bridge-ros2dds &> /dev/null; then
    ZENOH_BIN="$(command -v zenoh-bridge-ros2dds)"
# 2. Search inside package bin/
elif [ -f "${PKG_ROOT}/bin/zenoh-bridge-ros2dds" ]; then
    ZENOH_BIN="${PKG_ROOT}/bin/zenoh-bridge-ros2dds"
# 3. Search in ~/bin
elif [ -f "${HOME}/bin/zenoh-bridge-ros2dds" ]; then
    ZENOH_BIN="${HOME}/bin/zenoh-bridge-ros2dds"
# 4. Search in ~/.local/bin
elif [ -f "${HOME}/.local/bin/zenoh-bridge-ros2dds" ]; then
    ZENOH_BIN="${HOME}/.local/bin/zenoh-bridge-ros2dds"
fi

if [ -z "${ZENOH_BIN}" ]; then
    echo "[ERROR] zenoh-bridge-ros2dds binary not found in PATH, ${PKG_ROOT}/bin, or ${HOME}/bin."
    echo "Please place the zenoh-bridge-ros2dds binary in '${PKG_ROOT}/bin' or '~/bin'."
    exit 1
fi

echo "=========================================================="
echo " Starting Zenoh Bridge GCS Router"
echo " Binary: ${ZENOH_BIN}"
echo " Listening on: tcp/0.0.0.0:${PORT}"
echo " User: $(whoami) | Host: $(hostname)"
echo "=========================================================="

exec "${ZENOH_BIN}" -l "tcp/0.0.0.0:${PORT}"
