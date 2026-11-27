#!/bin/bash

KEEP_GZ=0
WATCH=""
while [ $# -gt 0 ]; do
  case "$1" in
    --keep-gz) KEEP_GZ=1; shift ;;
    --watch)    WATCH="${2:-}"; shift 2 ;;
    *) echo "stop_sim.sh: unknown argument '$1'" >&2; exit 2 ;;
  esac
done

_watch_alive() {
  ps -eo stat=,comm= | awk -v p="$1" \
    '$1 !~ /^Z/ && index($2, p) {f=1} END {exit !f}'
}

if [ -n "$WATCH" ]; then
  for _ in $(seq 1 20); do
    _watch_alive "$WATCH" && break
    sleep 1
  done
  while _watch_alive "$WATCH"; do
    sleep 2
  done
  echo "stop_sim.sh: '$WATCH' window closed -> stopping the simulation stack"
fi

set +e

pkill -9 -x arm_controller
pkill -9 -x arm_sim_sfml
pkill -9 -x arm_manual
pkill -9 -x arm_motor_driver
pkill -9 -f "install/arm/lib/arm/arm_motor_driver"
pkill -9 -f "install/arm/lib/arm/arm_sim_sfml"
pkill -9 -f "install/arm/lib/arm/arm_controller"
pkill -9 -f "install/arm/lib/arm/arm_manual"
if [ "$KEEP_GZ" -eq 0 ]; then
  pkill -9 -x gzserver
  pkill -9 -x gzclient
  pkill -9 -f "[g]z sim"
else
  echo "stop_sim.sh: --keep-gz, leaving gzserver/gzclient alive."
fi
pkill -9 -f "install/arm_gazebo/lib/arm_gazebo/arm_cmd_bridge"
pkill -9 -f "lib/ros_gz_sim/create"
pkill -9 -f "parameter_bridge.*delta_arm"
pkill -9 -f "lib/[r]obot_state_publisher/robot_state_publisher"
sleep 1

pkill -9 -f "ros2 launch arm"
pkill -9 -f "sim_2d.launch.py"
pkill -9 -f "sim_3d.launch.py"
pkill -9 -f "arm_sim_sfml --ros-args"
sleep 1

if command -v ros2 >/dev/null 2>&1; then
  ros2 daemon stop 2>/dev/null
fi

exit 0
