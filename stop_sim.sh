#!/bin/bash
# stop_sim.sh — stop any running arm simulation/controller processes (2-D SFML,
# 3-D Gazebo or real-hardware stack) inside the container.
#
# Called automatically by run_arm_sim_2d.sh before each (re)launch so a fresh
# launch never clashes on ROS node names (e.g. two /arm_controller nodes). Being
# a script FILE (not an inline `bash -lc` command) keeps the "ros2 launch arm"
# pkill pattern from matching this script's own command line.
#
# Usage (inside the container):
#   bash /root/ros2_ws/src/arm/stop_sim.sh

set +e

# Executables (2-D SFML, controller, manual, real-HW, Gazebo). SIGKILL so an
# ignored SIGTERM can never leave an orphaned window/node behind.
pkill -9 -x arm_controller
pkill -9 -x arm_sim_sfml
pkill -9 -x arm_manual
pkill -9 -x arm_system
pkill -9 -x arm_motor_driver
pkill -9 -x gzserver
pkill -9 -x gzclient
sleep 1

# ros2 launch wrapper processes left behind by the above (child death makes
# them exit on their own, but kill them anyway so the DDS graph goes quiet).
pkill -9 -f "ros2 launch arm"
pkill -9 -f "arm_sim_2d.launch.py"
pkill -9 -f "arm_sim_sfml --ros-args"
sleep 1

# Clear the ROS discovery daemon cache so `ros2 node list` is not stale.
if command -v ros2 >/dev/null 2>&1; then
  ros2 daemon stop 2>/dev/null
fi

exit 0