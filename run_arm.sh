#!/bin/bash
# Container entrypoint for the 3-DoF delta arm.
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
# Works in BOTH layouts:
#   * standalone   : workspace mounted at /root/ws (repo root IS the arm package)
#   * parent fyp   : sources mounted at /root/ros2_ws/src/arm
#
# Extra launch arguments passed after the mode are forwarded unchanged, e.g.:
#   ./run_arm.sh 2d driver:=true control:=false
#   ./run_arm.sh hardware control:=false

set -e

source /opt/ros/humble/setup.bash
if [ -f /root/ros2_ws/install/setup.bash ]; then
  source /root/ros2_ws/install/setup.bash
elif [ -f install/setup.bash ]; then
  source install/setup.bash
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
    exec ros2 launch arm sim_2d.launch.py \
      ${PARAMS_ARG[@]+"${PARAMS_ARG[@]}"} "$@"
    ;;
  3d|sim3d|sim-3d)
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
