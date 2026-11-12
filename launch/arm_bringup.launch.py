"""Reusable controller / driver / manual bringup.

Included by the user-facing launch files; not normally launched directly.

Arguments:
  params_file         arm parameter YAML (default config/arm_params.yaml)
  driver              start the real motor driver (default false)
  control             enable motion/action server (default true); this is a
                      runtime switch, never a config/*.yaml parameter
  manual              start the keyboard target-position node (default false)
  publish_joints      publish the optional arm/joints debug topic (default false)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('arm')
    default_params_file = os.path.join(pkg_share, 'config', 'arm_params.yaml')

    params_file = LaunchConfiguration('params_file')
    driver = LaunchConfiguration('driver')
    control = LaunchConfiguration('control')
    manual = LaunchConfiguration('manual')
    publish_joints = LaunchConfiguration('publish_joints')

    enable_motion = ParameterValue(control, value_type=bool)
    publish_joints_flag = ParameterValue(publish_joints, value_type=bool)

    controller_params = [params_file, {
        'enable_motion': enable_motion,
        'publish_joints': publish_joints_flag,
    }]
    driver_params = [params_file, {
        'enable_motion': enable_motion,
    }]

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params_file,
            description='Arm parameter set to load.'),
        DeclareLaunchArgument(
            'driver', default_value='false',
            description='Start the real motor driver.'),
        DeclareLaunchArgument(
            'control', default_value='true',
            description='Enable set_pos motion; false is feedback-only.'),
        DeclareLaunchArgument(
            'manual', default_value='false',
            description='Start the manual target-position node.'),
        DeclareLaunchArgument(
            'publish_joints', default_value='false',
            description='Publish the 2-D debug arm/joints topic.'),
        Node(
            package='arm',
            executable='arm_controller',
            name='arm_controller',
            output='screen',
            parameters=controller_params,
            on_exit=Shutdown(reason='arm_controller exited'),
        ),
        GroupAction(
            condition=IfCondition(driver),
            actions=[
                Node(
                    package='arm',
                    executable='arm_motor_driver',
                    name='arm_motor_driver',
                    output='screen',
                    parameters=driver_params,
                    on_exit=Shutdown(reason='arm_motor_driver exited'),
                ),
            ],
        ),
        Node(
            package='arm',
            executable='arm_manual',
            name='arm_manual_control',
            output='screen',
            parameters=[params_file],
            condition=IfCondition(manual),
        ),
    ])
