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

# Executables (2-D SFML, controller, manual, real-HW, Gazebo).
pkill -x arm_controller
pkill -x arm_sim_sfml
pkill -x arm_manual
pkill -x arm_system
pkill -x arm_motor_driver
pkill -x gzserver
pkill -x gzclient
sleep 1

# ros2 launch wrapper processes left behind by the above (child death makes
# them exit on their own, but kill them anyway so the DDS graph goes quiet).
pkill -f "ros2 launch arm"
pkill -f "arm_sim_2d.launch.py"
sleep 1

# Clear the ROS discovery daemon cache so `ros2 node list` is not stale.
if command -v ros2 >/dev/null 2>&1; then
  ros2 daemon stop 2>/dev/null
fi

exit 0