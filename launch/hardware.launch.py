"""Real or fake hardware bringup.

Usage:
  ros2 launch arm hardware.launch.py
  ros2 launch arm hardware.launch.py control:=false
  ros2 launch arm hardware.launch.py driver:=false
  ros2 launch arm hardware.launch.py manual:=true

driver:=true drives the real servos. control defaults to true; pass
control:=false to read the servos without commanding them.
driver:=false runs the controller without any driver (fake hardware).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('arm')
    default_params_file = os.path.join(pkg_share, 'config', 'arm_params.yaml')
    bringup_file = os.path.join(
        pkg_share, 'launch', 'arm_bringup.launch.py')

    params_file = LaunchConfiguration('params_file')
    driver = LaunchConfiguration('driver')
    control = LaunchConfiguration('control')
    manual = LaunchConfiguration('manual')

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_file),
        launch_arguments={
            'params_file': params_file,
            'driver': driver,
            'control': control,
            'manual': manual,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params_file,
            description='Arm parameter set to load.'),
        DeclareLaunchArgument(
            'driver', default_value='true',
            description='Start the real motor driver (false = fake hardware).'),
        DeclareLaunchArgument(
            'control', default_value='true',
            description='Enable set_pos motion; false is feedback-only.'),
        DeclareLaunchArgument(
            'manual', default_value='false',
            description='Start the manual target-position node.'),
        bringup,
    ])
