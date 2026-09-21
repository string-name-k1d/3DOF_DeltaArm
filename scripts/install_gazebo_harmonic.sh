#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# install_gazebo_harmonic.sh
#
# Optional add-on: installs the Gazebo Harmonic (gz-sim 8) closed-loop 3-D
# simulation stack for the delta arm (the `--view 3d` path of run_arm.sh).
#
#     bash /root/ros2_ws/src/arm/scripts/install_gazebo_harmonic.sh
#
# Installs (in order):
#   1. The OSRF apt repository (gazebo-stable) for the gz-harmonic binaries.
#   2. gz-harmonic (gz-sim 8.15+ server & GUI) plus its tooling.
#   3. ros-humble-ros-gzharmonic  (ros_gz_sim / ros_gz_bridge / interfaces)
#   4. ros-humble-robot-state-publisher (URDF -> TF for the arm model)
#
# Note: gz-transport discovery inside the arm_sim container needs GZ_IP
# pinned to 127.0.0.1 -- run_arm.sh / arm_gazebo.launch.py already do this.
# ---------------------------------------------------------------------------
set -euo pipefail

GZ_STABLE_LIST=/etc/apt/sources.list.d/gazebo-stable.list
GZ_KEYRING=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

if dpkg -s gz-harmonic >/dev/null 2>&1 && dpkg -s ros-humble-ros-gzharmonic >/dev/null 2>&1; then
  echo "[install_gazebo_harmonic] gz-harmonic + ros_gz already installed; nothing to do."
  exit 0
fi

echo "[install_gazebo_harmonic] adding OSRF gazebo-stable apt repository..."
if [ ! -f "$GZ_KEYRING" ]; then
  curl -sSL https://packages.osrfoundation.org/gazebo.key \
    -o "$GZ_KEYRING"
fi
if [ ! -f "$GZ_STABLE_LIST" ]; then
  echo "deb [arch=$(dpkg --print-architecture) signed-by=$GZ_KEYRING] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | tee "$GZ_STABLE_LIST" >/dev/null
fi

echo "[install_gazebo_harmonic] apt-get update + install gz-harmonic, ros_gz, robot_state_publisher..."
apt-get update
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
  gz-harmonic \
  ros-humble-ros-gzharmonic \
  ros-humble-robot-state-publisher

echo "[install_gazebo_harmonic] done. Use: ./run_arm.sh --view 3d"