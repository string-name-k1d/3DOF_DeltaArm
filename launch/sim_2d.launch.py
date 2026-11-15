"""2-D SFML delta-arm simulation.

Usage:
  ros2 launch arm sim_2d.launch.py
  ros2 launch arm sim_2d.launch.py control:=false
  ros2 launch arm sim_2d.launch.py driver:=true
  ros2 launch arm sim_2d.launch.py driver:=true control:=false

driver:=true visualises the real servos as a model reference. control defaults
to true; pass control:=false to observe the real arm without commanding it.
"""

import os

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _require_sfml_simulator():
    """Fail with an actionable message when SFML was not available at build time.

    arm_sim_sfml is optional (SFML is not a rosdep dependency), so a build on a
    machine without libsfml-dev succeeds but silently omits this executable.
    Without this check `ros2 launch` would only report "executable not found".
    """
    sim_binary = os.path.join(
        get_package_prefix('arm'), 'lib', 'arm', 'arm_sim_sfml')
    if os.path.isfile(sim_binary):
        return

    raise RuntimeError(
        'arm_sim_sfml was not built because SFML is not installed.\n'
        'The rest of the package builds and runs without it.\n'
        '\n'
        'Install SFML and rebuild to get the 2-D view:\n'
        '  sudo apt install libsfml-dev\n'
        '  colcon build --symlink-install --packages-select arm\n'
        '\n'
        'Or use a launch that needs no 2-D window:\n'
        '  ros2 launch arm hardware.launch.py driver:=false  # fake hardware\n'
        '  ros2 launch arm hardware.launch.py                # real hardware\n'
        '  ros2 launch arm_gazebo sim_3d.launch.py           # 3-D Gazebo')


def generate_launch_description():
    _require_sfml_simulator()

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
            'publish_joints': 'true',
        }.items(),
    )

    sim = Node(
        package='arm',
        executable='arm_sim_sfml',
        name='arm_sim_sfml',
        output='screen',
        parameters=[params_file],
        on_exit=Shutdown(reason='arm_sim_sfml exited'),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params_file,
            description='Arm parameter set to load.'),
        DeclareLaunchArgument(
            'driver', default_value='false',
            description='Include the real motor driver.'),
        DeclareLaunchArgument(
            'control', default_value='true',
            description='Enable set_pos motion; false is feedback-only.'),
        DeclareLaunchArgument(
            'manual', default_value='false',
            description='Start the manual target-position node.'),
        bringup,
        sim,
    ])
