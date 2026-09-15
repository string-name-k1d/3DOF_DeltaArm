# 2D SFML delta-arm simulator launch.
#
# Launches the SFML 2D visualizer plus controller and/or real motor driver:
#
#   arm_sim_2d.launch.py                       virtual sim (controller + window)
#   arm_sim_2d.launch.py motor:=true           real hardware: driver + controller
#                                              + window (WASD jogs real servos)
#   arm_sim_2d.launch.py motor:=true control:=false
#                                              REAL-FEEDBACK VISUALISATION: the
#                                              driver runs feedback-only (no
#                                              set_pos control, no motor_targets)
#                                              and the window draws the measured
#                                              servo angles via FK. No controller
#                                              node is started.
#
# Args:
#   motor    (false)  include the real arm_motor_driver node.
#   control  (true)   enable set_pos motor control (only meaningful with motor).
#                      false = visualization-only using motor feedbacks.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('arm')
    params_file = os.path.join(pkg_share, 'config', 'arm_params.yaml')

    motor = LaunchConfiguration('motor')
    control = LaunchConfiguration('control')

    return LaunchDescription([
        DeclareLaunchArgument(
            'motor', default_value='false',
            description='Include the real motor driver (arm/motor_feedback).'),
        DeclareLaunchArgument(
            'control', default_value='true',
            description='Enable set_pos motor control (motor mode only); '
                        'false = visualization-only using motor feedbacks.'),

        # Virtual-sim controller (no motor driver): the two-in-one behaviour
        # that arm_sim_2d.launch.py had before the motor/control args.
        Node(
            package='arm',
            executable='arm_controller',
            name='arm_controller',
            output='screen',
            parameters=[params_file],
            condition=UnlessCondition(motor),
        ),

        # SFML 2D visualizer (always on - the window).
        Node(
            package='arm',
            executable='arm_sim_sfml',
            name='arm_sim_sfml',
            output='screen',
            parameters=[params_file],
        ),

        # Motor mode, control ON: driver + controller so set_pos jogs the real
        # servos (WASD in the window, arm/motor_feedback draws the real arm).
        GroupAction(
            condition=IfCondition(motor),
            actions=[
                GroupAction(
                    condition=IfCondition(control),
                    actions=[
                        Node(
                            package='arm',
                            executable='arm_motor_driver',
                            name='arm_motor_driver',
                            output='screen',
                            parameters=[params_file],
                        ),
                        Node(
                            package='arm',
                            executable='arm_controller',
                            name='arm_controller',
                            output='screen',
                            parameters=[params_file],
                        ),
                    ],
                ),
                GroupAction(
                    condition=UnlessCondition(control),
                    actions=[
                        # Motor mode, control OFF = feedback-only visualisation:
                        # the driver just pings + reports arm/motor_feedback
                        # (enable_motion=false) and the window draws it; no
                        # controller, nothing can move.
                        Node(
                            package='arm',
                            executable='arm_motor_driver',
                            name='arm_motor_driver',
                            output='screen',
                            parameters=[params_file, {'enable_motion': False}],
                        ),
                    ],
                ),
            ],
        ),
    ])