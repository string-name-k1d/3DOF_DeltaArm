#!/bin/bash
# Container launcher for the 3-DoF delta arm.
#
#   default            real hardware - controller + motor driver (arm_system)
#   simulation         controller only (arm_controller); the arm simulation is
#                      provided externally (e.g. the arm_gazebo package)
#   <other args>       executed as a shell command (dev shell / builds)
#
# Works in BOTH layouts:
#   * standalone   : workspace mounted at /root/ws (repo root IS the arm package,
#                    sees install/setup.bash in the current directory)
#   * parent fyp   : sources mounted at /root/ros2_ws/src/arm (install at
#                    /root/ros2_ws/install — the shared fyp_sim container)

set -e

source /opt/ros/humble/setup.bash
if [ -f /root/ros2_ws/install/setup.bash ]; then
  source /root/ros2_ws/install/setup.bash
elif [ -f install/setup.bash ]; then
  source install/setup.bash
fi

# Optional parameter-set override: ARM_PARAMS_FILE=/path/to/arm_gen0_params.yaml
# selects an alternate geometry/config set (see arm/config/).
PARAMS_ARG=()
[ -n "${ARM_PARAMS_FILE:-}" ] && PARAMS_ARG=(params_file:="$ARM_PARAMS_FILE")

if [ "${1:-}" = "simulation" ]; then
  echo "[arm] launching arm_controller (simulation mode - external arm sim expected)"
  exec ros2 launch arm delta_arm.launch simulation:=true ${PARAMS_ARG[@]+"${PARAMS_ARG[@]}"}
fi

if [ -n "${1:-}" ]; then
  exec "$@"
fi

echo "[arm] launching arm_system (real hardware)"
exec ros2 launch arm delta_arm.launch ${PARAMS_ARG[@]+"${PARAMS_ARG[@]}"}