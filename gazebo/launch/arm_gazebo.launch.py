"""Gazebo (Classic) simulation of the 3DOF delta arm.

Launches:
  * the world SDF generated at runtime from the xacro descriptor
    urdf/delta_arm.world.xacro (the DeltaArmGazeboPlugin inside the model
    subscribes to arm/motor_targets and drives the shoulder joints).
  * the arm controller (simulation mode - no motor driver), which exposes the
    set_pos action / get_pos streaming on the standard /arm topics.
  * optionally the WASD manual-control node ('manual:=true') so the simulated
    arm can be jogged from the keyboard in the launching terminal.

The world xacro is built with the SAME mechanism parameters and maths as the
2-D simulator (values in config/arm_params.yaml under "geometry"): edit that
file to change link lengths, then re-run xacro + gzserver.

Usage:
    ros2 launch arm_gazebo arm_gazebo.launch.py                     (headless)
    ros2 launch arm_gazebo arm_gazebo.launch.py gui:=true           (Gzclient)
    ros2 launch arm_gazebo arm_gazebo.launch.py manual:=true        (+ WASD)
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    arm_share = get_package_share_directory('arm')
    arm_gazebo_share = get_package_share_directory('arm_gazebo')

    # <prefix>/lib holding libdelta_arm_gazebo_plugin.so next to the share dir.
    plugin_lib = os.path.normpath(os.path.join(arm_gazebo_share, '..', '..', 'lib'))
    existing = os.environ.get('GAZEBO_PLUGIN_PATH', '')
    plugin_path = plugin_lib + (os.pathsep + existing if existing else '')

    # gzserver resolves <plugin filename> through GAZEBO_PLUGIN_PATH, so make
    # our plugin library discoverable before the server is started.
    os.environ['GAZEBO_PLUGIN_PATH'] = plugin_path

    # The parameterised world descriptor (source of truth for the 3-D model).
    # worlds/delta_arm.world is a committed copy of its output.
    world_xacro = os.path.join(arm_gazebo_share, 'urdf', 'delta_arm.world.xacro')
    world = os.path.join(tempfile.gettempdir(), 'delta_arm.generated.world')
    params_file = os.path.join(arm_share, 'config', 'arm_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('verbose', default_value='true'),
        DeclareLaunchArgument('manual', default_value='false'),
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', plugin_path),

        ExecuteProcess(
            cmd=['xacro', world_xacro, '-o', world],
            name='xacro_world',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')),
            launch_arguments={
                'world': world,
                'verbose': LaunchConfiguration('verbose'),
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py')),
            condition=IfCondition(LaunchConfiguration('gui')),
        ),

        Node(
            package='arm',
            executable='arm_controller',
            name='arm_controller',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='arm',
            executable='arm_manual',
            name='arm_manual_control',
            output='screen',
            parameters=[params_file],
            condition=IfCondition(LaunchConfiguration('manual')),
        ),
    ])