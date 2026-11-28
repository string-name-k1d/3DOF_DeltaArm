"""State-machine interface for the delta arm.

The node keeps the high-level arm state and exposes it through two services:

  arm/set_state  request a transition to stop / manual / home / hold
  arm/get_state  read the current state plus pose and servo feedback

MANUAL is the default state. In MANUAL the interface does not send arm/set_pos
goals; the existing raw position-control action remains the active control path.
"""

import threading
import time

from geometry_msgs.msg import Point, Twist
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from arm.action import SetPosition
from arm.msg import ArmPosition, MotorFeedback
from arm.srv import GetState, SetState

STATE_STOP = 0
STATE_MANUAL = 1
STATE_HOME = 2
STATE_HOLD = 3
STATE_NAMES = {
    STATE_STOP: 'stop',
    STATE_MANUAL: 'manual',
    STATE_HOME: 'home',
    STATE_HOLD: 'hold',
}
STATE_BY_NAME = {name: value for value, name in STATE_NAMES.items()}
DEFAULT_STATE = STATE_MANUAL

STATUS_OK = getattr(SetPosition.Result, 'STATUS_OK', 0)
STATUS_UNREACHABLE = getattr(SetPosition.Result, 'STATUS_UNREACHABLE', 1)
STATUS_POS_NOT_REACHED = getattr(SetPosition.Result, 'STATUS_POS_NOT_REACHED', 2)
STATUS_ACTION_UNAVAILABLE = 3
STATUS_CANCELED = 4
STATUS_ERROR = 5

STATUS_MESSAGE = {
    STATUS_OK: 'arrived',
    STATUS_UNREACHABLE: 'target rejected by IK',
    STATUS_POS_NOT_REACHED: 'pose not reached before timeout',
    STATUS_ACTION_UNAVAILABLE: 'arm/set_pos action server unavailable',
    STATUS_CANCELED: 'move canceled or timed out',
    STATUS_ERROR: 'interface error',
}


def _wait(future, timeout):
    deadline = time.monotonic() + max(0.0, timeout)
    while not future.done():
        if time.monotonic() >= deadline:
            return False
        time.sleep(0.005)
    return True


class ArmInterfaceNode(Node):
    def __init__(self):
        super().__init__('arm_interface_node')
        self.declare_parameter('home_position', [0.0, 0.0, -0.28])
        self.declare_parameter('move_timeout', 10.0)
        self.declare_parameter('action_wait_timeout', 2.0)
        self.declare_parameter('cancel_timeout', 2.0)

        self._state_lock = threading.Lock()
        self._goal_lock = threading.Lock()
        self._state = DEFAULT_STATE
        self._last_pos = ArmPosition()
        self._last_feedback = MotorFeedback()
        self._has_pos = False
        self._has_feedback = False
        self._goal_handle = None

        self._action_client = ActionClient(self, SetPosition, 'arm/set_pos')
        self.create_subscription(ArmPosition, 'arm/pos', self._on_pos, 10)
        self.create_subscription(MotorFeedback, 'arm/MotorFeedback', self._on_feedback, 10)

        self.create_service(SetState, 'arm/set_state', self._on_set_state)
        self.create_service(GetState, 'arm/get_state', self._on_get_state)
        self.get_logger().info(
            'arm_interface state machine ready (state=%s)' % STATE_NAMES[self._state])

    def _on_pos(self, msg):
        with self._state_lock:
            self._last_pos = msg
            self._has_pos = True

    def _on_feedback(self, msg):
        with self._state_lock:
            self._last_feedback = msg
            self._has_feedback = True

    def _get_state(self):
        with self._state_lock:
            return self._state

    def _set_state(self, state):
        with self._state_lock:
            self._state = state

    def _current_point(self):
        with self._state_lock:
            pos = self._last_pos
            if self._has_pos:
                return Point(x=pos.position.x, y=pos.position.y, z=pos.position.z)
            return Point()

    def _fill_state(self, response):
        state = self._get_state()
        with self._state_lock:
            pos = self._last_pos
            fb = self._last_feedback
            has_pos = self._has_pos
            has_feedback = self._has_feedback
        response.state = state
        response.state_name = STATE_NAMES.get(state, 'unknown')
        response.position = pos.position if has_pos else Point()
        response.target_position = pos.target_position if has_pos else Point()
        response.motor_angles_current = list(pos.motor_angles_current) if has_pos else [0.0, 0.0, 0.0]
        response.motor_angles_target = list(pos.motor_angles_target) if has_pos else [0.0, 0.0, 0.0]
        response.servo_online = list(fb.servo_online) if has_feedback else [0, 0, 0]
        response.servo_error = list(fb.servo_error) if has_feedback else [0, 0, 0]
        response.has_feedback = has_feedback

    def _on_get_state(self, request, response):
        self._fill_state(response)

    def _on_set_state(self, request, response):
        requested = int(request.state)
        if requested not in STATE_NAMES:
            current = self._get_state()
            response.success = False
            response.status = STATUS_ERROR
            response.state = current
            response.state_name = STATE_NAMES[current]
            response.message = 'unknown state %d' % requested
            response.reached_position = self._current_point()
            return

        if requested == STATE_HOME:
            success, status, message, reached = self._move_home()
            if success:
                self._set_state(STATE_HOME)
        else:
            self._cancel_active_goal()
            self._set_state(requested)
            success = True
            status = STATUS_OK
            message = 'state -> %s' % STATE_NAMES[requested]
            reached = self._current_point()

        current = self._get_state()
        response.success = success
        response.status = status
        response.state = current
        response.state_name = STATE_NAMES[current]
        response.message = message
        response.reached_position = reached

    def _move_home(self):
        values = self.get_parameter('home_position').value
        if len(values) != 3:
            point = self._current_point()
            return False, STATUS_ERROR, 'home_position must contain 3 values', point
        return self._move(values[0], values[1], values[2], 0.0)

    def _move(self, x, y, z, timeout):
        self._cancel_active_goal()
        wait_timeout = float(self.get_parameter('action_wait_timeout').value)
        if not self._action_client.wait_for_server(timeout_sec=wait_timeout):
            point = self._current_point()
            return False, STATUS_ACTION_UNAVAILABLE, STATUS_MESSAGE[STATUS_ACTION_UNAVAILABLE], point

        goal = SetPosition.Goal()
        goal.target = Twist()
        goal.target.linear.x = float(x)
        goal.target.linear.y = float(y)
        goal.target.linear.z = float(z)

        send_future = self._action_client.send_goal_async(goal)
        if not _wait(send_future, wait_timeout):
            point = self._current_point()
            return False, STATUS_ERROR, 'timed out sending arm/set_pos goal', point
        goal_handle = send_future.result()
        if not getattr(goal_handle, 'accepted', True):
            point = self._current_point()
            return False, STATUS_UNREACHABLE, 'arm/set_pos rejected the goal', point
        with self._goal_lock:
            self._goal_handle = goal_handle

        if float(timeout) <= 0.0:
            timeout = float(self.get_parameter('move_timeout').value)
        result_future = goal_handle.get_result_async()
        if not _wait(result_future, timeout):
            self._cancel_active_goal()
            point = self._current_point()
            return False, STATUS_CANCELED, 'move timed out waiting for result', point

        result = result_future.result().result
        status = int(result.status)
        message = STATUS_MESSAGE.get(status, 'action status %d' % status)
        point = self._current_point()
        if hasattr(result, 'current'):
            point = Point(x=result.current.x, y=result.current.y, z=result.current.z)
        with self._goal_lock:
            self._goal_handle = None
        return status == STATUS_OK, status, message, point

    def _cancel_active_goal(self):
        with self._goal_lock:
            goal_handle = self._goal_handle
            self._goal_handle = None
        if goal_handle is None:
            return
        cancel_future = goal_handle.cancel_goal_async()
        _wait(cancel_future, float(self.get_parameter('cancel_timeout').value))


def main():
    rclpy.init()
    node = ArmInterfaceNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
