#!/usr/bin/env bash
# ==============================================================================
# start_zenoh_usv.sh
# Portable startup script for USV Onboard Zenoh Bridge client
# Connects to the local forwarded reverse SSH tunnel endpoint (tcp/127.0.0.1:7447).
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

TARGET="${1:-tcp/127.0.0.1:7447}"
ZENOH_BIN=""

if command -v zenoh-bridge-ros2dds &> /dev/null; then
    ZENOH_BIN="$(command -v zenoh-bridge-ros2dds)"
elif [ -f "${PKG_ROOT}/bin/zenoh-bridge-ros2dds" ]; then
    ZENOH_BIN="${PKG_ROOT}/bin/zenoh-bridge-ros2dds"
elif [ -f "${HOME}/bin/zenoh-bridge-ros2dds" ]; then
    ZENOH_BIN="${HOME}/bin/zenoh-bridge-ros2dds"
elif [ -f "${HOME}/zenoh-bridge-ros2dds" ]; then
    ZENOH_BIN="${HOME}/zenoh-bridge-ros2dds"
elif [ -f "${HOME}/.local/bin/zenoh-bridge-ros2dds" ]; then
    ZENOH_BIN="${HOME}/.local/bin/zenoh-bridge-ros2dds"
fi

if [ -z "${ZENOH_BIN}" ]; then
    echo "[ERROR] zenoh-bridge-ros2dds binary not found in PATH, ${PKG_ROOT}/bin, or ${HOME}."
    exit 1
fi

echo "=========================================================="
echo " Starting USV Zenoh Bridge Client"
echo " Binary: ${ZENOH_BIN}"
echo " Connecting to: ${TARGET}"
echo " User: $(whoami) | Host: $(hostname)"
echo "=========================================================="

exec "${ZENOH_BIN}" client -e "${TARGET}"
