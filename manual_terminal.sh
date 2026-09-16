#!/bin/bash
# Manual (WASD) control terminal for the running arm simulation.
#
# Runs the arm_manual node connected to the arm controller that the arm_gazebo
# launch started (action /arm/set_pos + position stream /arm/get_pos). Intended
# to be started inside an interactive container terminal (docker exec -it); the
# run_arm.sh script auto-opens such a terminal ($TERMINAL_CMD, konsole,
# gnome-terminal, alacritty, or Windows Terminal on WSL).
#
#   W/S : +/- z    A/D : +/- y     Q : quit

set -e

source /opt/ros/humble/setup.bash
if [ -d /root/ros2_ws ]; then
  source /root/ros2_ws/install/setup.bash
elif [ -f install/setup.bash ]; then
  source install/setup.bash
fi

cd /root/ros2_ws/src/arm 2>/dev/null || cd "$(dirname "$0")"
exec ros2 run arm arm_manual --ros-args --params-file config/arm_params.yaml