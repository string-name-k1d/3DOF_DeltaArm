#!/usr/bin/env bash
# One-time setup for the arm package (Ubuntu 22.04 + ROS 2 Humble).
#
#   ./scripts/setup.sh                                    # build with .env
#   ./scripts/setup.sh --event-handlers console_direct+   # extra colcon args
#
# Creates .env from .env.example, installs the optional add-ons selected in
# .env (ARM_INSTALL_GAZEBO / ARM_INSTALL_ROSBRIDGE), and builds the workspace.
# Re-running is safe: existing files are reused.
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.." || exit 1
REPO="$PWD"

# Per-machine configuration.
[ -f .env ] || cp .env.example .env
# shellcheck disable=SC1091
. ./.env

SUDO=""
if [ "$(id -u)" -ne 0 ]; then SUDO=sudo; fi

# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# Workspace root: ARM_WS from .env, else walk up until this repo sits under
# <candidate>/src/ (arm_ws/src/fyp/arm -> arm_ws). A standalone checkout (repo
# root IS the package) falls back to the repo root, which colcon accepts.
if [ -n "${ARM_WS:-}" ]; then
  if [ ! -d "$ARM_WS" ]; then
    echo "setup.sh: ARM_WS='$ARM_WS' is not a directory." >&2
    exit 1
  fi
  WS="$(cd "$ARM_WS" && pwd)"
else
  WS="$REPO"
  candidate="$(dirname "$REPO")"
  while [ "$candidate" != "/" ]; do
    case "$REPO/" in
      "$candidate/src/"*) WS="$candidate"; break ;;
    esac
    candidate="$(dirname "$candidate")"
  done
fi

# Optional: 3-D Gazebo Harmonic stack (the `3d` launch mode).
if [ "${ARM_INSTALL_GAZEBO:-0}" = "1" ]; then
  GZ_KEYRING=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
  GZ_STABLE_LIST=/etc/apt/sources.list.d/gazebo-stable.list
  OSRF_KEYID=67170598AF249743
  OSRF_KEY_URL=https://packages.osrfoundation.org/gazebo.key

  if dpkg -s gz-harmonic >/dev/null 2>&1 && dpkg -s ros-humble-ros-gzharmonic >/dev/null 2>&1; then
    echo "Gazebo Harmonic already installed; skipping."
  else
    if [ ! -s "$GZ_KEYRING" ]; then
      echo "Installing the OSRF signing key..."
      $SUDO rm -f "$GZ_KEYRING"
      if $SUDO curl -sSL --max-time 20 "$OSRF_KEY_URL" -o "$GZ_KEYRING" && \
         (gpg --dearmor < "$GZ_KEYRING" >/dev/null 2>&1 || gpg --show-keys "$GZ_KEYRING" >/dev/null 2>&1); then
        :
      else
        $SUDO rm -f "$GZ_KEYRING"
        $SUDO gpg --no-default-keyring --keyring "$GZ_KEYRING" \
          --keyserver hkps://keyserver.ubuntu.com --recv-keys "$OSRF_KEYID"
      fi
    fi
    if [ ! -f "$GZ_STABLE_LIST" ]; then
      echo "Configuring the gazebo-stable apt source..."
      echo "deb [arch=$(dpkg --print-architecture) signed-by=$GZ_KEYRING] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        | $SUDO tee "$GZ_STABLE_LIST" >/dev/null
    fi
    $SUDO apt-get update
    $SUDO env DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      gz-harmonic \
      ros-humble-ros-gzharmonic \
      ros-humble-robot-state-publisher
  fi
fi

# Optional: rosbridge_suite for browser/WebSocket clients.
if [ "${ARM_INSTALL_ROSBRIDGE:-0}" = "1" ]; then
  if dpkg -s ros-humble-rosbridge-suite >/dev/null 2>&1; then
    echo "rosbridge_suite already installed; skipping."
  else
    $SUDO apt-get update
    $SUDO env DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      ros-humble-rosbridge-suite
  fi
fi

SFML_SIM="${ARM_BUILD_SFML_SIM:-ON}"
PACKAGES="${ARM_BUILD_PACKAGES:-arm}"

echo "setup.sh: workspace = $WS"
echo "setup.sh: ARM_BUILD_SFML_SIM = $SFML_SIM"
echo "setup.sh: packages = $PACKAGES"

cd "$WS"

# Extra arguments come before the package/CMake options so they are parsed as
# colcon options rather than being consumed by --cmake-args.
# shellcheck disable=SC2086  # PACKAGES is intentionally word-split.
colcon build "$@" --symlink-install --packages-select $PACKAGES \
  --cmake-args "-DARM_BUILD_SFML_SIM=$SFML_SIM"

cat <<EOF
Setup complete.
  source .env
  source $WS/install/setup.bash
  ./run_arm.sh simulation
EOF
