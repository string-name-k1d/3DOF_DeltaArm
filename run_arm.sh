#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ROS_SETUP="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
if [ -f "$ROS_SETUP" ]; then
  source "$ROS_SETUP"
elif ! command -v ros2 >/dev/null 2>&1; then
  echo "run_arm.sh: ROS 2 not found (looked for $ROS_SETUP)." >&2
  echo "            Install ROS 2 Humble or source its setup.bash first." >&2
  exit 1
fi

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

PARAMS_ARG=()
[ -n "${ARM_PARAMS_FILE:-}" ] && PARAMS_ARG=(params_file:="$ARM_PARAMS_FILE")

if [ $# -eq 0 ]; then
  set -- 2d driver:=true
fi

MODE="$1"
shift

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
