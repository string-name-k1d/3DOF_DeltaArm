# 2D SFML delta-arm simulator launch.
#
# Launches the controller + the SFML 2D visualizer. The controller runs in
# simulation mode (no motor driver); the SFML window renders the arm and
# forwards WASD targets via the standard set_pos action.
#
# Usage:
#   ros2 launch arm arm_sim_2d.launch.py
#   ros2 launch arm arm_sim_2d.launch.py verbose:=true

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('arm')
    params_file = os.path.join(pkg_share, 'config', 'arm_params.yaml')

    return LaunchDescription([
        # Controller (simulation mode: no motor driver).
        Node(
            package='arm',
            executable='arm_controller',
            name='arm_controller',
            output='screen',
            parameters=[params_file],
        ),

        # SFML 2D visualizer.
        Node(
            package='arm',
            executable='arm_sim_sfml',
            name='arm_sim_sfml',
            output='screen',
            parameters=[params_file],
        ),
    ])