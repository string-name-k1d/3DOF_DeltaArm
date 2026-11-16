#!/bin/bash
# Entry point for the 3-DoF delta arm. Works in a local ROS 2 install and in
# the Docker container.
#
#   (no arguments)      real hardware + 2-D reference view, control ON
#   hardware            real hardware: controller + motor driver, control ON
#   fake / simulation   fake hardware: controller only, control ON
#   2d / sim2d          2-D SFML view (driver:=true for real hardware)
#   3d / sim3d          3-D Gazebo simulation
#   manual              fake hardware + manual target-position node
#   <other args>        executed as a shell command (dev shell / builds)
#
# Motion is ON by default; pass control:=false to observe only.
#
# Works in all supported layouts:
#   * native, workspace root : arm_ws/ with this repo under src/ (e.g.
#                              arm_ws/src/fyp/arm); run from arm_ws or from
#                              the package directory
#   * Docker, standalone     : repo mounted at /root/ws (repo root IS the arm
#                              package)
#   * Docker, parent fyp     : sources mounted at /root/ros2_ws/src/arm
#
# The 2-D view needs the optional arm_sim_sfml executable, which is only built
# when SFML is installed (Ubuntu: sudo apt install libsfml-dev). Without it use
# `./run_arm.sh hardware` or `./run_arm.sh simulation`.
#
# Extra launch arguments passed after the mode are forwarded unchanged, e.g.:
#   ./run_arm.sh 2d driver:=true control:=false
#   ./run_arm.sh hardware control:=false
#
# Per-machine overrides come from the sourced environment:
#   source .env && ./run_arm.sh 2d

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ROS 2 environment. Prefer an already-active ROS_DISTRO; otherwise default to
# Humble, which is what the provided Docker image uses.
ROS_SETUP="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
if [ -f "$ROS_SETUP" ]; then
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
elif ! command -v ros2 >/dev/null 2>&1; then
  echo "run_arm.sh: ROS 2 not found (looked for $ROS_SETUP)." >&2
  echo "            Install ROS 2 Humble or source its setup.bash first." >&2
  exit 1
fi

# Source the workspace overlay when one exists. In a native workspace this
# walks up from this script (arm_ws/src/.../arm -> arm_ws/install/setup.bash),
# so the script also works when invoked from the package directory.
find_overlay() {
  local dir="$SCRIPT_DIR"
  while [ "$dir" != "/" ]; do
    if [ -f "$dir/install/setup.bash" ]; then
      printf '%s\n' "$dir/install/setup.bash"
      return 0
    fi
    dir="$(dirname "$dir")"
  done
  return 1
}

if [ -f /root/ros2_ws/install/setup.bash ]; then
  source /root/ros2_ws/install/setup.bash
elif [ -f install/setup.bash ]; then
  source install/setup.bash
elif overlay="$(find_overlay)"; then
  source "$overlay"
fi

# Optional parameter-set override (see arm/config/):
#   ARM_PARAMS_FILE=/path/to/arm_gen0_params.yaml ./run_arm.sh 2d
PARAMS_ARG=()
[ -n "${ARM_PARAMS_FILE:-}" ] && PARAMS_ARG=(params_file:="$ARM_PARAMS_FILE")

if [ $# -eq 0 ]; then
  # Real hardware with the 2-D model reference; motion is on by default.
  set -- 2d driver:=true
fi

MODE="$1"
shift

# The 2-D view is optional: it is built only when SFML is available. Check for
# the executable up front and explain how to enable it instead of letting ros2
# launch fail with a generic "executable not found".
require_sfml_sim() {
  local prefix
  prefix="$(ros2 pkg prefix arm 2>/dev/null || true)"
  if [ -x "$prefix/lib/arm/arm_sim_sfml" ]; then
    return 0
  fi

  echo "run_arm.sh: arm_sim_sfml is not built (SFML was not found)." >&2
  echo "  Install SFML and rebuild to get the 2-D view:" >&2
  echo "    sudo apt install libsfml-dev" >&2
  echo "    colcon build --symlink-install --packages-select arm" >&2
  echo "  Or run without the 2-D view:" >&2
  echo "    ./run_arm.sh hardware       # real hardware" >&2
  echo "    ./run_arm.sh simulation     # fake hardware" >&2
  exit 1
}

# The Gazebo package is a separate (nested) package and may not be built.
require_arm_gazebo() {
  if ros2 pkg prefix arm_gazebo >/dev/null 2>&1; then
    return 0
  fi

  echo "run_arm.sh: arm_gazebo is not built or not on the environment." >&2
  echo "  Build it as a sibling of arm (see AGENT.md), then retry:" >&2
  echo "    colcon build --symlink-install --packages-select arm arm_gazebo" >&2
  exit 1
}

case "$MODE" in
  hardware)
    exec ros2 launch arm hardware.launch.py \
      ${PARAMS_ARG[@]+"${PARAMS_ARG[@]}"} "$@"
    ;;
  fake|simulation)
    exec ros2 launch arm hardware.launch.py driver:=false \
      ${PARAMS_ARG[@]+"${PARAMS_ARG[@]}"} "$@"
    ;;
  2d|sim2d|sim-2d)
    require_sfml_sim
    exec ros2 launch arm sim_2d.launch.py \
      ${PARAMS_ARG[@]+"${PARAMS_ARG[@]}"} "$@"
    ;;
  3d|sim3d|sim-3d)
    require_arm_gazebo
    exec ros2 launch arm_gazebo sim_3d.launch.py \
      ${PARAMS_ARG[@]+"${PARAMS_ARG[@]}"} "$@"
    ;;
  manual)
    exec ros2 launch arm hardware.launch.py driver:=false manual:=true \
      ${PARAMS_ARG[@]+"${PARAMS_ARG[@]}"} "$@"
    ;;
  *)
    exec "$MODE" "$@"
    ;;
esac
