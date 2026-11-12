"""Gazebo Harmonic (gz-sim 8) 3-D simulation of the 3-DoF delta arm.

Usage:
  ros2 launch arm_gazebo sim_3d.launch.py
  ros2 launch arm_gazebo sim_3d.launch.py control:=false
  ros2 launch arm_gazebo sim_3d.launch.py gui:=true
  ros2 launch arm_gazebo sim_3d.launch.py manual:=true
  ros2 launch arm_gazebo sim_3d.launch.py payload:=endpoint
  ros2 launch arm_gazebo sim_3d.launch.py \
      use_existing_gz:=true world_name:=apriltag1 mount:=true
  ros2 launch arm_gazebo sim_3d.launch.py \
      use_existing_gz:=true world_name:=apriltag1 mount:=true \
      bridge_clock:=false

The arm controller and manual node are included from the arm package's
arm_bringup.launch.py so the 2-D, 3-D and hardware launches share one node
definition. The endpoint payload is a passive stand-in without joints, so it
runs without a controller.

ARM_PAYLOAD, ARM_MOUNT and ARM_PARAMS_FILE provide defaults for payload, mount
and params_file when the launch arguments are not given.
"""

import os

import yaml
import xacro as xacro_lib

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _geometry_mappings(params_path):
    try:
        with open(params_path) as fh:
            doc = yaml.safe_load(fh) or {}
    except OSError:
        return {}
    geometry = (doc.get('/**', {}).get('ros__parameters', {}) or {}).get(
        'geometry', {}) or {}
    mappings = {}
    for key in ('base_radius', 'platform_radius', 'upper_arm_len',
                'lower_arm_len'):
        if key in geometry:
            mappings[key] = str(float(geometry[key]) / 1000.0)
    return mappings


def _launch_setup(context, *args, **kwargs):
    arm_share = get_package_share_directory('arm')
    arm_gazebo_share = get_package_share_directory('arm_gazebo')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    payload = LaunchConfiguration('payload').perform(context).strip().lower()
    if payload not in ('delta_arm', 'endpoint'):
        raise RuntimeError(
            f"payload must be 'delta_arm' or 'endpoint' (got '{payload}')")
    is_delta = payload == 'delta_arm'

    mounted = LaunchConfiguration('mount').perform(context).lower() == 'true'
    manual = LaunchConfiguration('manual').perform(context).lower() == 'true'
    control = LaunchConfiguration('control').perform(context).lower()
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    verbose = LaunchConfiguration('verbose').perform(context).lower() == 'true'
    use_existing_gz = (
        LaunchConfiguration('use_existing_gz').perform(context).lower()
        == 'true')
    bridge_clock = (
        LaunchConfiguration('bridge_clock').perform(context).lower()
        == 'true')
    world_name = LaunchConfiguration('world_name').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    mount_offset_z = LaunchConfiguration('mount_offset_z').perform(context)

    model_name = 'delta_arm' if is_delta else 'delta_arm_endpoint'
    urdf_xacro = os.path.join(
        arm_gazebo_share, 'urdf',
        'delta_arm.urdf.xacro' if is_delta else 'endpoint.urdf.xacro')
    world = os.path.join(arm_gazebo_share, 'worlds', 'arm_world.sdf')

    mappings = {'mount': 'true' if mounted else 'false'}
    if is_delta:
        mappings.update(_geometry_mappings(params_file))
    robot_description_content = xacro_lib.process_file(
        urdf_xacro, mappings=mappings).toxml()

    js_topic = f'/world/{world_name}/model/{model_name}/joint_state'
    bridge_args = []
    if bridge_clock:
        bridge_args.append('/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock')
    if is_delta:
        bridge_args.extend([
            '/delta_arm/shoulder_0/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/delta_arm/shoulder_1/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/delta_arm/shoulder_2/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        ])
    bridge_args.append(
        f'{js_topic}@sensor_msgs/msg/JointState[gz.msgs.Model')

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        output='screen',
        arguments=bridge_args,
        remappings=[(js_topic, '/joint_states')],
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True,
        }],
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', model_name],
    )

    actions = [
        SetEnvironmentVariable('GZ_IP', '127.0.0.1'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', arm_gazebo_share),
        rsp,
    ]

    if not use_existing_gz:
        verbosity = '4' if verbose else '2'
        gz_args = f'{world} -v {verbosity} -r'
        if not gui:
            gz_args += ' -s'
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_share, 'launch',
                             'gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args}.items(),
        ))

    actions.extend([spawn_entity, bridge])

    if mounted:
        actions.append(Node(
            package='arm_gazebo',
            executable='mount_welder.py',
            name='mount_welder',
            output='screen',
            arguments=[
                '--world', world_name,
                '--child-model', model_name,
                '--offset-z', mount_offset_z,
            ],
        ))

    if is_delta:
        actions.append(Node(
            package='arm_gazebo',
            executable='arm_cmd_bridge',
            name='arm_cmd_bridge',
            output='screen',
        ))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(arm_share, 'launch',
                             'arm_bringup.launch.py')),
            launch_arguments={
                'params_file': params_file,
                'driver': 'false',
                'control': control,
                'manual': str(manual).lower(),
            }.items(),
        ))

    return actions


def generate_launch_description():
    arm_share = get_package_share_directory('arm')
    default_params_file = (
        os.environ.get('ARM_PARAMS_FILE', '').strip()
        or os.path.join(arm_share, 'config', 'arm_params.yaml'))
    default_payload = os.environ.get('ARM_PAYLOAD', 'delta_arm').strip().lower()
    default_mount = 'true' if os.environ.get('ARM_MOUNT', '0') == '1' else 'false'

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='false',
            description='Start the Gazebo GUI.'),
        DeclareLaunchArgument(
            'verbose', default_value='true',
            description='High Gazebo verbosity.'),
        DeclareLaunchArgument(
            'manual', default_value='false',
            description='Start the manual target-position node.'),
        DeclareLaunchArgument(
            'control', default_value='true',
            description='Enable set_pos motion; false is feedback-only.'),
        DeclareLaunchArgument(
            'payload', default_value=default_payload,
            description="'delta_arm' or 'endpoint'."),
        DeclareLaunchArgument(
            'params_file', default_value=default_params_file,
            description='Arm parameter set to load.'),
        DeclareLaunchArgument(
            'mount', default_value=default_mount,
            description='Weld the payload onto an existing drone base.'),
        DeclareLaunchArgument(
            'mount_offset_z', default_value='-0.05',
            description='Mount offset below the drone base, metres.'),
        DeclareLaunchArgument(
            'bridge_clock', default_value='true',
            description='Bridge the Gazebo clock; set false when attaching to '
                        'a world that already publishes /clock.'),
        DeclareLaunchArgument(
            'use_existing_gz', default_value='false',
            description='Spawn into an already-running Gazebo server.'),
        DeclareLaunchArgument(
            'world_name', default_value='arm_world',
            description='World name used for joint-state and mount topics.'),
        OpaqueFunction(function=_launch_setup),
    ])
