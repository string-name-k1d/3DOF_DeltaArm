"""Start only the high-level arm state-machine interface."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('arm')
    params_file = LaunchConfiguration('params_file')
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(share, 'config', 'arm_params.yaml'),
            description='Arm parameter set.'),
        Node(
            package='arm',
            executable='arm_interface_node',
            name='arm_interface_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
