#!/bin/bash
# Container launcher for the 3-DoF delta arm.
#
#   default            real hardware - controller + motor driver (arm_system)
#   simulation         controller only (arm_controller); the arm simulation is
#                      provided externally (e.g. the arm_gazebo package)
#   <other args>       executed as a shell command (dev shell / builds)
#
# The workspace is mounted by compose at /root/ws (the repo root IS the arm
# package), so this script sources the colcon install from the current dir.

set -e

source /opt/ros/humble/setup.bash
if [ -f install/setup.bash ]; then
  source install/setup.bash
fi

if [ "${1:-}" = "simulation" ]; then
  echo "[arm] launching arm_controller (simulation mode - external arm sim expected)"
  exec ros2 launch arm delta_arm.launch simulation:=true
fi

if [ -n "${1:-}" ]; then
  exec "$@"
fi

echo "[arm] launching arm_system (real hardware)"
exec ros2 launch arm delta_arm.launch