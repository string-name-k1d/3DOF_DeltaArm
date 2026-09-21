"""Gazebo Harmonic (gz-sim 8) closed-loop simulation of the 3-DoF delta arm.

Launches:
  * the delta arm as an open kinematic tree closed at runtime by Gazebo
    DetachableJoint system plugins (see urdf/delta_arm.urdf.xacro).
  * the world worlds/arm_world.sdf (ODE physics, quick solver — same engine
    profile as the drone's apriltag1 world: 0.004 s step @ 250 Hz).
  * robot_state_publisher with the xacro-generated robot_description.
  * the ros_gz_sim entity spawner + the ros_gz_bridge parameter bridge
    (/clock, the three shoulder cmd_pos topics, /joint_states).
  * arm_cmd_bridge: converts the arm controller's arm/motor_targets (degrees)
    into the three cmd_pos topics (radians) the joint servos follow.
  * the arm controller (simulation mode - no motor driver), which exposes the
    set_pos action / get_pos streaming on the standard /arm topics.
  * optionally the WASD manual-control node ('manual:=true').

The mechanism is built from the SAME geometry parameters as the 2-D simulator
(config/arm_params.yaml -> "geometry"): edit that file to change link lengths,
then sync the xacro properties at the top of urdf/delta_arm.urdf.xacro.

Usage:
    ros2 launch arm_gazebo arm_gazebo.launch.py                     (headless)
    ros2 launch arm_gazebo arm_gazebo.launch.py gui:=true           (Gazebo GUI)
    ros2 launch arm_gazebo arm_gazebo.launch.py manual:=true        (+ WASD)
    # Attach to an ALREADY-RUNNING gz server (e.g. the PX4 drone world) and
    # spawn the arm into it — no second Gazebo server is started:
    ros2 launch arm_gazebo arm_gazebo.launch.py \
        use_existing_gz:=true world_name:=apriltag1
"""

import os

import xacro as xacro_lib

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    arm_share = get_package_share_directory('arm')
    arm_gazebo_share = get_package_share_directory('arm_gazebo')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    urdf_xacro = os.path.join(arm_gazebo_share, 'urdf', 'delta_arm.urdf.xacro')
    world = os.path.join(arm_gazebo_share, 'worlds', 'arm_world.sdf')
    params_file = os.path.join(arm_share, 'config', 'arm_params.yaml')

    robot_description_content = xacro_lib.process_file(urdf_xacro).toxml()

    # The bridge's joint-state source topic depends on the world the model is
    # spawned into: '/world/<world_name>/model/delta_arm/joint_state'.
    js_topic = PythonExpression([
        "'/world/' + '", LaunchConfiguration('world_name'),
        "' + '/model/delta_arm/joint_state'",
    ])
    js_bridge_arg = PythonExpression([
        "'/world/' + '", LaunchConfiguration('world_name'),
        "' + '/model/delta_arm/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'",
    ])

    # gz sim args: world + verbosity + run; add '-s' (server-only, no GUI) when
    # launched headless. The gz server itself is skipped entirely with
    # use_existing_gz:=true — the arm is spawned into a server that is already
    # running (e.g. the PX4 drone world). For transport to work there, that
    # server must run with the same GZ_IP (127.0.0.1; the combined launcher
    # exports it for the PX4 process too).
    gz_args = PythonExpression([
        "'", world, " -v 4 -r' + (' -s' if '",
        LaunchConfiguration('gui'), "' == 'false' else '')",
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args}.items(),
        condition=UnlessCondition(LaunchConfiguration('use_existing_gz')),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'delta_arm',
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/delta_arm/shoulder_0/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/delta_arm/shoulder_1/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/delta_arm/shoulder_2/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            js_bridge_arg,
        ],
        remappings=[(js_topic, '/joint_states')],
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content, 'use_sim_time': True}],
    )

    cmd_bridge = Node(
        package='arm_gazebo',
        executable='arm_cmd_bridge',
        name='arm_cmd_bridge',
        output='screen',
    )

    controller = Node(
        package='arm',
        executable='arm_controller',
        name='arm_controller',
        output='screen',
        parameters=[params_file],
    )

    manual = Node(
        package='arm',
        executable='arm_manual',
        name='arm_manual_control',
        output='screen',
        parameters=[params_file],
        condition=IfCondition(LaunchConfiguration('manual')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('verbose', default_value='true'),
        DeclareLaunchArgument('manual', default_value='false'),
        DeclareLaunchArgument(
            'use_existing_gz', default_value='false',
            description='Do NOT start a Gazebo server; spawn the arm into an '
                        'already-running one (e.g. the PX4 drone world). The '
                        'server must use the same GZ_IP=127.0.0.1 transport.'),
        DeclareLaunchArgument(
            'world_name', default_value='arm_world',
            description='Name of the world the arm is spawned into; used for '
                        'the joint-state bridge topic '
                        "'/world/<world_name>/model/delta_arm/joint_state'."),
        # gz-transport topic/service discovery is unreliable in containers with
        # host networking; forcing a loopback-only transport makes every gz
        # process (server, bridge, spawner) discover each other.
        SetEnvironmentVariable('GZ_IP', '127.0.0.1'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', arm_gazebo_share),
        rsp,
        gz_sim,
        spawn_entity,
        bridge,
        cmd_bridge,
        controller,
        manual,
    ])