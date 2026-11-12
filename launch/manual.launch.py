"""Manual target-position control (WASD + Z/X).

Attaches to an already-running arm_controller (sim_2d, sim_3d or hardware).
To start the controller and manual node together, pass manual:=true to those
launches instead.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('arm')
    default_params_file = os.path.join(pkg_share, 'config', 'arm_params.yaml')

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params_file,
            description='Arm parameter set to load.'),
        Node(
            package='arm',
            executable='arm_manual',
            name='arm_manual_control',
            output='screen',
            parameters=[params_file],
        ),
    ])
