#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# install_rosbridge.sh
#
# Optional add-on: installs the ROS 2 to WebSocket bridge (rosbridge_suite)
# so web browsers / external clients can talk to the ROS 2 graph from the
# container (default port 9090).
#
#     bash /root/ros2_ws/src/arm/scripts/install_rosbridge.sh
# ---------------------------------------------------------------------------
set -euo pipefail

if dpkg -s ros-humble-rosbridge-suite >/dev/null 2>&1; then
  echo "[install_rosbridge] ros-humble-rosbridge-suite already installed; nothing to do."
  exit 0
fi

echo "[install_rosbridge] installing ros-humble-rosbridge-suite..."
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
  ros-humble-rosbridge-suite

echo "[install_rosbridge] done. Launch with: ros2 launch rosbridge_server rosbridge_websocket_launch.xml"