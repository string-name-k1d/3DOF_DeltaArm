"""Execute the arm_hardware unit tests.

Runs the GTest binary produced by the package (built from test/). The test
binary is installed to <prefix>/lib/arm_hardware, so it is located from the
package prefix and executed as a plain process (it is not a ROS node).

Usage:
    ros2 launch arm_bringup test.launch.py
"""
import os

from ament_index_python.packages import get_package_prefix

from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo


def generate_launch_description():
    prefix = get_package_prefix('arm_hardware')

    # Standard ament install locations for the test executable.
    name = 'test_arm'
    candidates = [
        os.path.join(prefix, 'lib', 'arm_hardware', name),
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
