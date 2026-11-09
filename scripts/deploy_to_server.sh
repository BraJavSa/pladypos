#!/usr/bin/env bash
# ==============================================================================
# deploy_to_server.sh
# Deploys all processing, fusion (Camera+IMU with Camera-Only fallback),
# teleoperation, and inverse dynamics control nodes + Zenoh v1.10.2 to the
# camera server (rawfie) and restarts camera .204 (0354).
# ==============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

echo "=========================================================="
echo " 1. Copying nodes & Zenoh v1.10.2 to rawfie:/home/uw/ros2-camera-test/config/"
echo "=========================================================="
ssh rawfie "docker stop ros2_streaming_container >/dev/null 2>&1 || true; rm -f ~/ros2-camera-test/config/zenoh-bridge-ros2dds"
scp \
  "${PKG_ROOT}/src/python_nodes/visual_tag_estimator.py" \
  "${PKG_ROOT}/src/python_nodes/direct_usv_fusion.py" \
  "${PKG_ROOT}/src/python_nodes/teleop_mixer.py" \
  "${PKG_ROOT}/src/python_nodes/pool_boundary_publisher.py" \
  "${PKG_ROOT}/src/python_nodes/inverse_dynamics_controller.py" \
  "${PKG_ROOT}/config/zenoh_config.json5" \
  "${PKG_ROOT}/bin/zenoh-bridge-ros2dds" \
  "${PKG_ROOT}/launch/start_all_cameras.launch.py" \
  "${PKG_ROOT}/scripts/run_controller_8.sh" \
  rawfie:~/ros2-camera-test/config/

echo "=========================================================="
echo " 2. Restarting rawfie container (strictly camera .204 / 0354)"
echo "=========================================================="
ssh rawfie "chmod +x ~/ros2-camera-test/config/*.py ~/ros2-camera-test/config/zenoh-bridge-ros2dds ~/ros2-camera-test/config/run_controller_8.sh && cd ~/ros2-camera-test && ./multicam-streaming.bash 0354"

echo "=========================================================="
echo " Server rawfie deployed and running!"
echo "=========================================================="
