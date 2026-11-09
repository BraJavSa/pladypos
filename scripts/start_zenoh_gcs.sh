#!/usr/bin/env bash
# ==============================================================================
# start_zenoh_gcs.sh
# Connects the GCS Laptop (ROS_DOMAIN_ID=0) to the central Zenoh Router running
# on rawfie (tcp/10.250.253.1:7447, ROS_DOMAIN_ID=10).
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

TARGET="${1:-tcp/10.250.253.1:7447}"
if [ "$1" == "--help" ] || [ "$1" == "-h" ]; then
    echo "Usage: $0 [TARGET_ENDPOINT] (default: tcp/10.250.253.1:7447)"
    exit 0
fi

ZENOH_BIN=""
if [ -x "${SCRIPT_DIR}/zenoh-bridge-ros2dds" ]; then
    ZENOH_BIN="${SCRIPT_DIR}/zenoh-bridge-ros2dds"
elif [ -x "${PKG_ROOT}/bin/zenoh-bridge-ros2dds" ]; then
    ZENOH_BIN="${PKG_ROOT}/bin/zenoh-bridge-ros2dds"
elif [ -x "${SCRIPT_DIR}/../../share/pladypos/bin/zenoh-bridge-ros2dds" ]; then
    ZENOH_BIN="${SCRIPT_DIR}/../../share/pladypos/bin/zenoh-bridge-ros2dds"
elif command -v zenoh-bridge-ros2dds &> /dev/null; then
    ZENOH_BIN="$(command -v zenoh-bridge-ros2dds)"
elif [ -f "${HOME}/bin/zenoh-bridge-ros2dds" ]; then
    ZENOH_BIN="${HOME}/bin/zenoh-bridge-ros2dds"
elif [ -f "${HOME}/.local/bin/zenoh-bridge-ros2dds" ]; then
    ZENOH_BIN="${HOME}/.local/bin/zenoh-bridge-ros2dds"
fi

if [ -z "${ZENOH_BIN}" ]; then
    echo "[ERROR] zenoh-bridge-ros2dds binary not found in PATH, ${PKG_ROOT}/bin, or ${HOME}/bin."
    exit 1
fi

CONFIG_FILE=""
if [ -f "${PKG_ROOT}/config/zenoh_config.json5" ]; then
    CONFIG_FILE="${PKG_ROOT}/config/zenoh_config.json5"
elif [ -f "${SCRIPT_DIR}/../../share/pladypos/config/zenoh_config.json5" ]; then
    CONFIG_FILE="${SCRIPT_DIR}/../../share/pladypos/config/zenoh_config.json5"
elif [ -f "${PKG_ROOT}/share/pladypos/config/zenoh_config.json5" ]; then
    CONFIG_FILE="${PKG_ROOT}/share/pladypos/config/zenoh_config.json5"
elif [ -f "${SCRIPT_DIR}/../config/zenoh_config.json5" ]; then
    CONFIG_FILE="${SCRIPT_DIR}/../config/zenoh_config.json5"
fi

unset ROS_LOCALHOST_ONLY
unset ROS_AUTOMATIC_DISCOVERY_RANGE
export ROS_DOMAIN_ID=0
export RUST_LOG="zenoh=info,zenoh::net::routing::dispatcher::pubsub=off"

echo "=========================================================="
echo " Starting GCS Zenoh Bridge Client -> rawfie Router"
echo " Binary: ${ZENOH_BIN}"
echo " Connecting to: ${TARGET} (ROS_DOMAIN_ID=0)"
if [ -n "${CONFIG_FILE}" ]; then
    echo " Config: ${CONFIG_FILE}"
fi
echo " User: $(whoami) | Host: $(hostname)"
echo "=========================================================="

if [ -n "${CONFIG_FILE}" ]; then
    exec "${ZENOH_BIN}" client -d 0 --no-multicast-scouting -c "${CONFIG_FILE}" -e "${TARGET}"
else
    exec "${ZENOH_BIN}" client -d 0 --no-multicast-scouting -e "${TARGET}"
fi
