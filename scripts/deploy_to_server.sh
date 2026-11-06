#!/usr/bin/env bash
# ==============================================================================
# deploy_to_server.sh
# Deploys all processing, fusion, teleoperation, and inverse dynamics control
# nodes + Zenoh v1.10.2 to the camera server (rawfie) and restarts camera .204.
# ==============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SCRATCH_LAUNCH="/home/brayan/.gemini/antigravity/brain/934bf9da-7e35-44ff-87a3-e29716eb5a31/scratch/start_all_cameras.launch.py"
SCRATCH_CTRL="/home/brayan/.gemini/antigravity/brain/934bf9da-7e35-44ff-87a3-e29716eb5a31/scratch/run_controller_8.sh"

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
  "${SCRATCH_LAUNCH}" \
  "${SCRATCH_CTRL}" \
  rawfie:~/ros2-camera-test/config/

echo "=========================================================="
echo " 2. Restarting rawfie container (strictly camera .204 / 0354)"
echo "=========================================================="
ssh rawfie "chmod +x ~/ros2-camera-test/config/*.py ~/ros2-camera-test/config/zenoh-bridge-ros2dds ~/ros2-camera-test/config/run_controller_8.sh && cd ~/ros2-camera-test && ./multicam-streaming.bash 0354"

echo "=========================================================="
echo " Server rawfie deployed and running!"
echo "=========================================================="
