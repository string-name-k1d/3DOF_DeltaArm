#!/bin/bash
# stop_sim.sh — stop any running arm simulation/controller processes (2-D SFML,
# 3-D Gazebo or real-hardware stack) inside the container.
#
# Called automatically by the host ./run_arm.sh before each (re)launch so a
# fresh launch never clashes on ROS node names (e.g. two /arm_controller nodes). Being
# a script FILE (not an inline `bash -lc` command) keeps the "ros2 launch arm"
# pkill pattern from matching this script's own command line.
#
# Usage (inside the container):
#   bash /root/ros2_ws/src/arm/stop_sim.sh
#   bash /root/ros2_ws/src/arm/stop_sim.sh --keep-gz
#   bash /root/ros2_ws/src/arm/stop_sim.sh --watch arm_sim_sfml
#
# --keep-gz: kill only the ARM stack, leave Gazebo itself (gzserver/gzclient)
#   running. Required when attaching the arm to an already-running world (the
#   drone's): otherwise this script kills the very server the arm is about to be
#   spawned into, and run_arm.sh then fails its "is the world up?" precondition.
#
# --watch PATTERN: block until no live process matches PATTERN, then run the
#   normal teardown. run_arm.sh launches this detached so that CLOSING the
#   simulation window shuts the whole stack down; without it the window process
#   would exit while the controller, bridges and Gazebo server stayed orphaned.

KEEP_GZ=0
WATCH=""
while [ $# -gt 0 ]; do
  case "$1" in
    --keep-gz) KEEP_GZ=1; shift ;;
    --watch)    WATCH="${2:-}"; shift 2 ;;
    *) echo "stop_sim.sh: unknown argument '$1'" >&2; exit 2 ;;
  esac
done

# Is a LIVE (non-zombie) process whose comm matches $1 running?
# Two subtleties, both learned the hard way:
#  * The stat filter matters: this container's PID 1 is 'tail -f /dev/null',
#    which never reaps orphans, so every killed process lingers forever as
#    'Z [name] <defunct>' and a plain pgrep/grep would keep reporting it as
#    present — the watchdog would then never fire.
#  * Match comm=, NOT args=. This script's own command line is
#    "stop_sim.sh --watch arm_sim_sfml", so an args= match finds ITSELF and
#    waits forever. comm= is the bare executable name (bash / stop_sim.sh here),
#    which can never equal the window process it is watching. Note the kernel
#    truncates comm to 15 chars, so PATTERN must be <= 15 characters.
_watch_alive() {
  ps -eo stat=,comm= | awk -v p="$1" \
    '$1 !~ /^Z/ && index($2, p) {f=1} END {exit !f}'
}

if [ -n "$WATCH" ]; then
  # Wait for the window to show up first, so we never arm on a pattern that has
  # not spawned yet and immediately tear the fresh stack down.
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

# Executables (2-D SFML, controller, manual, real-HW, Gazebo). SIGKILL so an
# ignored SIGTERM can never leave an orphaned window/node behind. The kernel
# truncates some comm() names (e.g. "arm_motor_drive"), so also kill by the
# full install path which never truncates.
pkill -9 -x arm_controller
pkill -9 -x arm_sim_sfml
pkill -9 -x arm_manual
pkill -9 -x arm_system
pkill -9 -x arm_motor_driver
pkill -9 -x arm_endpoint
pkill -9 -f "install/arm/lib/arm/arm_motor_driver"
pkill -9 -f "install/arm/lib/arm/arm_sim_sfml"
pkill -9 -f "install/arm/lib/arm/arm_controller"
pkill -9 -f "install/arm/lib/arm/arm_manual"
pkill -9 -f "install/arm/lib/arm/arm_endpoint"
if [ "$KEEP_GZ" -eq 0 ]; then
  pkill -9 -x gzserver
  pkill -9 -x gzclient
  # Gazebo Harmonic (gz-sim 8) server + the arm_gazebo launch stack.
  pkill -9 -f "[g]z sim"
else
  echo "stop_sim.sh: --keep-gz, leaving gzserver/gzclient alive."
fi
pkill -9 -f "install/arm_gazebo/lib/arm_gazebo/arm_cmd_bridge"
pkill -9 -f "lib/ros_gz_sim/create"
pkill -9 -f "lib/ros_gz_bridge/parameter_bridge"
pkill -9 -x robot_state_publisher
sleep 1

# ros2 launch wrapper processes left behind by the above (child death makes
# them exit on their own, but kill them anyway so the DDS graph goes quiet).
pkill -9 -f "ros2 launch arm"
pkill -9 -f "arm_sim_2d.launch.py"
pkill -9 -f "arm_gazebo.launch.py"
pkill -9 -f "arm_sim_sfml --ros-args"
sleep 1

# Clear the ROS discovery daemon cache so `ros2 node list` is not stale.
if command -v ros2 >/dev/null 2>&1; then
  ros2 daemon stop 2>/dev/null
fi

exit 0