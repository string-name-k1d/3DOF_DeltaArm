"""Gazebo Harmonic (gz-sim 8) closed-loop simulation of the 3-DoF delta arm.

Launches:
  * the payload as an open kinematic tree closed at runtime by Gazebo
    DetachableJoint system plugins (urdf/delta_arm.urdf.xacro), OR the
    simplified rigid endpoint (urdf/endpoint.urdf.xacro) that only carries
    the base_link / tool0 frames (no mechanism internals).
  * the world worlds/arm_world.sdf (ODE physics, quick solver — same engine
    profile as the drone's apriltag1 world: 0.004 s step @ 250 Hz).
  * robot_state_publisher with the xacro-generated robot_description.
  * the ros_gz_sim entity spawner + the ros_gz_bridge parameter bridge
    (/clock, the three shoulder cmd_pos topics, /joint_states).
  * arm_cmd_bridge: converts the arm controller's arm/motor_targets (degrees)
    into the three cmd_pos topics (radians) the joint servos follow. Only
    launched for the delta_arm payload.
  * the arm controller (simulation mode - no motor driver), which exposes the
    set_pos action / get_pos streaming on the standard /arm topics. Only
    launched for the delta_arm payload.
  * mount_welder (mount:=true): at runtime welds the payload onto the drone's
    base_link with a fixed joint (see scripts/mount_welder.py) — the "arm
    mounted on the drone" mode. The URDF is then processed with mount=true,
    dropping the artificial `world` link so base_link has exactly one parent.
  * optionally the WASD manual-control node ('manual:=true').

Payload + mounting are selected by:
  * ARM_PAYLOAD=delta_arm|endpoint      (env; default delta_arm)
  * mount:=true|false                   (launch arg; default off)
  * mount_offset_z:="..."               (launch arg; offset of the payload
                                          base below/above the drone base)
Runs one of:
    ros2 launch arm_gazebo arm_gazebo.launch.py                     (headless)
    ros2 launch arm_gazebo arm_gazebo.launch.py gui:=true           (Gazebo GUI)
    ros2 launch arm_gazebo arm_gazebo.launch.py manual:=true        (+ WASD)
    # Attach to an ALREADY-RUNNING gz server (e.g. the PX4 drone world) and
    # spawn the arm into it — no second Gazebo server is started:
    ros2 launch arm_gazebo arm_gazebo.launch.py \
        use_existing_gz:=true world_name:=apriltag1
    # Attach AND weld the payload under the drone:
    ros2 launch arm_gazebo arm_gazebo.launch.py \
        use_existing_gz:=true world_name:=apriltag1 mount:=true \
        mount_offset_z:=-0.05
    # Same, but the simplified endpoint-only payload:
    ARM_PAYLOAD=endpoint ros2 launch arm_gazebo arm_gazebo.launch.py \
        use_existing_gz:=true world_name:=apriltag1 mount:=true

The mechanism is built from the SAME geometry parameters as the 2-D simulator
(config/arm_params.yaml -> "geometry"): edit that file to change link lengths,
then sync the xacro properties at the top of urdf/delta_arm.urdf.xacro.
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

    # Payload pick: read the env var at description-build time (run_arm.sh /
    # sim.sh export ARM_PAYLOAD, so it is known before the graph builds).
    payload = os.environ.get('ARM_PAYLOAD', 'delta_arm').strip().lower()
    if payload not in ('delta_arm', 'endpoint'):
        raise RuntimeError(
            f"ARM_PAYLOAD must be 'delta_arm' or 'endpoint' (got '{payload}')")
    mounted = os.environ.get('ARM_MOUNT', '0') == '1'
    is_delta = payload == 'delta_arm'

    model_name = 'delta_arm' if is_delta else 'delta_arm_endpoint'
    urdf_xacro = os.path.join(
        arm_gazebo_share, 'urdf',
        'delta_arm.urdf.xacro' if is_delta else 'endpoint.urdf.xacro')
    world = os.path.join(arm_gazebo_share, 'worlds', 'arm_world.sdf')
    params_file = os.path.join(arm_share, 'config', 'arm_params.yaml')

    # mount=true drops the payload's fake `world` link so base_link can be
    # welded onto the drone with a single parent (see module docstring).
    # Always passed explicitly (never a bare-file default).
    mappings = {'mount': 'true' if mounted else 'false'}
    robot_description_content = xacro_lib.process_file(
        urdf_xacro, mappings=mappings).toxml()

    # The bridge's joint-state source topic depends on the world the model is
    # spawned into: '/world/<world_name>/model/<model_name>/joint_state'.
    js_topic = PythonExpression([
        "'/world/' + '", LaunchConfiguration('world_name'),
        "' + '/model/", model_name, "/joint_state'",
    ])
    js_bridge_arg = PythonExpression([
        "'/world/' + '", LaunchConfiguration('world_name'),
        "' + '/model/", model_name, "/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'",
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
            '-name', model_name,
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

    # Runtime weld of the mounted payload onto the drone base (see
    # scripts/mount_welder.py). Exits after the create service confirms.
    welder = Node(
        package='arm_gazebo',
        executable='mount_welder.py',
        name='mount_welder',
        output='screen',
        arguments=[
            '--world', LaunchConfiguration('world_name'),
            '--child-model', model_name,
            '--offset-z', LaunchConfiguration('mount_offset_z'),
        ],
        condition=IfCondition(LaunchConfiguration('mount')),
    )

    # delta_arm payload only: the command bridge + controller + manual node
    # driving the three JPC shoulder servos. The endpoint payload is a pure
    # passive stand-in (no joints to command).
    is_endpoint = PythonExpression(["'", LaunchConfiguration('payload'), "' == 'endpoint'"])

    cmd_bridge = Node(
        package='arm_gazebo',
        executable='arm_cmd_bridge',
        name='arm_cmd_bridge',
        output='screen',
        condition=UnlessCondition(is_endpoint),
    )

    controller = Node(
        package='arm',
        executable='arm_controller',
        name='arm_controller',
        output='screen',
        parameters=[params_file],
        condition=UnlessCondition(is_endpoint),
    )

    manual = Node(
        package='arm',
        executable='arm_manual',
        name='arm_manual_control',
        output='screen',
        parameters=[params_file],
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('manual'),
                "' == 'true' and '", LaunchConfiguration('payload'), "' != 'endpoint'",
            ])),
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('verbose', default_value='true'),
        DeclareLaunchArgument('manual', default_value='false'),
        DeclareLaunchArgument('payload', default_value=payload),
        DeclareLaunchArgument('mount', default_value='true' if mounted else 'false'),
        DeclareLaunchArgument('mount_offset_z', default_value='-0.05'),
        DeclareLaunchArgument(
            'use_existing_gz', default_value='false',
            description='Do NOT start a Gazebo server; spawn the arm into an '
                        'already-running one (e.g. the PX4 drone world). The '
                        'server must use the same GZ_IP=127.0.0.1 transport.'),
        DeclareLaunchArgument(
            'world_name', default_value='arm_world',
            description='Name of the world the arm is spawned into; used for '
                        'the joint-state bridge topic '
                        "'/world/<world_name>/model/<model_name>/joint_state'."),
        # gz-transport topic/service discovery is unreliable in containers with
        # host networking; forcing a loopback-only transport makes every gz
        # process (server, bridge, spawner) discover each other.
        SetEnvironmentVariable('GZ_IP', '127.0.0.1'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', arm_gazebo_share),
        rsp,
        gz_sim,
        spawn_entity,
        bridge,
        welder,
        cmd_bridge,
        controller,
        manual,
    ])