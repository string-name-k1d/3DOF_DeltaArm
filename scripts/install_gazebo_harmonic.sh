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
# Note: gz-transport discovery inside the container needs GZ_IP
# pinned to 127.0.0.1 -- run_arm.sh / arm_gazebo.launch.py already do this.
# ---------------------------------------------------------------------------
set -euo pipefail

GZ_KEYRING=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
GZ_STABLE_LIST=/etc/apt/sources.list.d/gazebo-stable.list
OSRF_KEYID=67170598AF249743
OSRF_KEY_URL=https://packages.osrfoundation.org/gazebo.key
GZ_KEYSERVER=hkps://keyserver.ubuntu.com

if dpkg -s gz-harmonic >/dev/null 2>&1 && dpkg -s ros-humble-ros-gzharmonic >/dev/null 2>&1; then
  echo "[install_gazebo_harmonic] gz-harmonic + ros_gz already installed; nothing to do."
  exit 0
fi

# Acquire the OSRF/Gazebo apt signing key so the gazebo-stable repo can be
# verified.  The key URL (packages.osrfoundation.org/gazebo.key) may serve an
# ASCII-armoured PGP block, but it has intermittently returned an empty /
# invalid response in build environments, so the script validates the
# downloaded key and falls back to the Ubuntu keyserver (key ID
# 67170598AF249743, "OSRF Repository GPG key") when needed.
if [ ! -f "$GZ_KEYRING" ] || [ ! -s "$GZ_KEYRING" ]; then
  echo "[install_gazebo_harmonic] acquiring OSRF signing key (key $OSRF_KEYID)..."
  rm -f "$GZ_KEYRING"
  if curl -sSL --max-time 20 "$OSRF_KEY_URL" -o "$GZ_KEYRING" 2>/dev/null; then
    if gpg --dearmor < "$GZ_KEYRING" >/dev/null 2>&1 || \
       gpg --show-keys "$GZ_KEYRING" >/dev/null 2>&1; then
      echo "[install_gazebo_harmonic] OSRF key acquired from $OSRF_KEY_URL (validated)."
    else
      echo "[install_gazebo_harmonic] $OSRF_KEY_URL did not return a valid PGP key - falling back to keyserver."
      rm -f "$GZ_KEYRING"
      gpg --no-default-keyring --keyring "$GZ_KEYRING" \
        --keyserver "$GZ_KEYSERVER" --recv-keys "$OSRF_KEYID" >/dev/null 2>&1
      echo "[install_gazebo_harmonic] OSRF key acquired from keyserver $GZ_KEYSERVER."
    fi
  else
    echo "[install_gazebo_harmonic] $OSRF_KEY_URL unreachable - falling back to keyserver."
    gpg --no-default-keyring --keyring "$GZ_KEYRING" \
      --keyserver "$GZ_KEYSERVER" --recv-keys "$OSRF_KEYID" >/dev/null 2>&1
    echo "[install_gazebo_harmonic] OSRF key acquired from keyserver $GZ_KEYSERVER."
  fi
  if [ -s "$GZ_KEYRING" ]; then
    gpg --no-default-keyring --keyring "$GZ_KEYRING" --export >/dev/null 2>&1 || \
      (gpg --dearmor < "$GZ_KEYRING" > "$GZ_KEYRING.dearmored" && mv "$GZ_KEYRING.dearmored" "$GZ_KEYRING")
  fi
fi

if [ ! -f "$GZ_STABLE_LIST" ]; then
  echo "[install_gazebo_harmonic] configuring gazebo-stable apt source..."
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