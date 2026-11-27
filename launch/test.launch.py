"""Run the arm GTest binary."""
import os

from ament_index_python.packages import get_package_prefix

from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo


def generate_launch_description():
    prefix = get_package_prefix('arm')


    name = 'test_arm'
    candidates = [
        os.path.join(prefix, 'lib', 'arm', name),
        os.path.join(prefix, 'bin', name),
    ]
    binary = next((p for p in candidates if os.path.exists(p)), candidates[0])

    return LaunchDescription([
        LogInfo(msg='Executing arm tests: {}'.format(binary)),
        ExecuteProcess(
            cmd=[binary],
            output='screen',
            name='arm_tests',
        ),
    ])
