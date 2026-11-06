#!/usr/bin/env python3
"""Dummy/example: map PX4 manual-control/joystick inputs to arm set_pos goals.

This is intentionally minimal and NOT safety-certified. It demonstrates the
integration shape requested during development:
- subscribe to `px4_msgs/msg/ManualControlSetpoint` (ROS 2, XRCE-DDS bridged)
- gate arm control with a togglable manual flag
- send `arm/action/SetPosition` goals while the arm is alive and not e-stopped
- optionally map a configured button/aux edge to `arm/srv/EmergencyStop` toggle

PX4 mapping varies by radio/QGC setup. Keep button/axis mappings configurable and
verify on hardware/sim before use. This example does not own vehicle mode/arm;
`drone_mission` remains responsible for mission takeover semantics.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

try:
    from px4_msgs.msg import ManualControlSetpoint
except Exception:  # pragma: no cover - example may run without px4_msgs
    ManualControlSetpoint = None

from arm.action import SetPosition
from arm.msg import ControlState
from arm.srv import EmergencyStop


PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


@dataclass
class AxisMap:
    x_axis: str = 'pitch'       # ManualControlSetpoint field name
    y_axis: str = 'roll'
    z_axis: str = 'throttle'
    e_stop_button: str = 'buttons'  # bitmask source field
    e_stop_bit: int = 0
    manual_enable_bit: int = 1


class Px4JoystickArmControl(Node):
    def __init__(self) -> None:
        super().__init__('px4_joystick_arm_control_example')

        self.declare_parameter('manual_control_topic', 'fmu/out/manual_control_setpoint')
        self.declare_parameter('home_x', 0.0)
        self.declare_parameter('home_y', 0.0)
        self.declare_parameter('home_z', -0.18)
        self.declare_parameter('x_range_m', 0.06)
        self.declare_parameter('y_range_m', 0.06)
        self.declare_parameter('z_range_m', 0.06)
        self.declare_parameter('publish_period_sec', 0.1)
        self.declare_parameter('goal_min_delta_m', 0.005)
        self.declare_parameter('e_stop_bit', -1)         # -1 disables mapping
        self.declare_parameter('manual_enable_bit', -1)  # -1 => always manual-allowed

        topic = self.get_parameter('manual_control_topic').value
        self._home = (
            float(self.get_parameter('home_x').value),
            float(self.get_parameter('home_y').value),
            float(self.get_parameter('home_z').value),
        )
        self._range = (
            float(self.get_parameter('x_range_m').value),
            float(self.get_parameter('y_range_m').value),
            float(self.get_parameter('z_range_m').value),
        )
        self._period = float(self.get_parameter('publish_period_sec').value)
        self._min_delta = float(self.get_parameter('goal_min_delta_m').value)
        self._e_stop_bit = int(self.get_parameter('e_stop_bit').value)
        self._manual_enable_bit = int(self.get_parameter('manual_enable_bit').value)

        self._manual = None
        self._control_state = None
        self._manual_enabled = self._manual_enable_bit < 0
        self._last_buttons = 0
        self._last_goal = None
        self._goal_inflight = False

        self._set_pos = ActionClient(self, SetPosition, 'arm/set_pos')
        self._estop = self.create_client(EmergencyStop, 'arm/emergency_stop')
        self.create_subscription(ControlState, 'arm/control_state', self._on_control_state, 10)
        if ManualControlSetpoint is not None:
            self.create_subscription(ManualControlSetpoint, topic, self._on_manual, PX4_QOS)
        else:
            self.get_logger().warning('px4_msgs not available; example cannot subscribe to PX4 manual control')
        self.create_timer(self._period, self._tick)
        self.get_logger().info(
            f'PX4 joystick arm example started (topic={topic}, home={self._home}, manual_enabled={self._manual_enabled})'
        )

    def _on_control_state(self, msg: ControlState) -> None:
        self._control_state = msg

    def _on_manual(self, msg) -> None:
        self._manual = msg
        buttons = int(getattr(msg, 'buttons', 0) or 0)
        rising = buttons & ~self._last_buttons
        self._last_buttons = buttons

        if self._manual_enable_bit >= 0 and (rising >> self._manual_enable_bit) & 1:
            self._manual_enabled = not self._manual_enabled
            self.get_logger().info(f'manual arm control {"ENABLED" if self._manual_enabled else "DISABLED"}')

        if self._e_stop_bit >= 0 and (rising >> self._e_stop_bit) & 1 and self._estop.service_is_ready():
            req = EmergencyStop.Request()
            req.action = EmergencyStop.Request.TOGGLE
            self._estop.call_async(req)
            self.get_logger().warn('requested arm emergency-stop toggle from PX4 button edge')

    def _arm_alive(self) -> bool:
        return self._control_state is not None

    def _axis(self, name: str) -> float:
        if self._manual is None:
            return 0.0
        value = float(getattr(self._manual, name, 0.0) or 0.0)
        return max(-1.0, min(1.0, value))

    def _target(self):
        # Dummy mapping: adjust axes/sign/ranges to your QGC/PX4 joystick setup.
        x = self._home[0] + self._axis('pitch') * self._range[0]
        y = self._home[1] + self._axis('roll') * self._range[1]
        # throttle is often [-1,1] in PX4 manual setpoint; map around home Z.
        z = self._home[2] + self._axis('throttle') * self._range[2]
        return (x, y, z)

    def _goal_response_cb(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # pragma: no cover
            self._goal_inflight = False
            self.get_logger().warning(f'set_pos goal failed: {exc}')
            return
        if not goal_handle.accepted:
            self._goal_inflight = False
            self._last_goal = None
            return
        goal_handle.get_result_async().add_done_callback(lambda _: setattr(self, '_goal_inflight', False))

    def _tick(self) -> None:
        if not self._manual_enabled or not self._arm_alive() or self._manual is None:
            return
        if self._control_state is not None and self._control_state.emergency_active:
            return
        if not getattr(self._manual, 'valid', True):
            return
        if self._goal_inflight or not self._set_pos.server_is_ready():
            return

        target = self._target()
        if self._last_goal is not None and all(abs(a - b) < self._min_delta for a, b in zip(target, self._last_goal)):
            return

        goal = SetPosition.Goal()
        goal.target = Twist()
        goal.target.linear.x, goal.target.linear.y, goal.target.linear.z = target
        self._goal_inflight = True
        self._last_goal = target
        self._set_pos.send_goal_async(goal).add_done_callback(self._goal_response_cb)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Px4JoystickArmControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
