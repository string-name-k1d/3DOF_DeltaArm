"""Synchronous client for the arm state-machine interface."""

from dataclasses import dataclass
import time

import rclpy
from geometry_msgs.msg import Point

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

STATUS_OK = 0
STATUS_ERROR = 5


@dataclass
class ArmResult:
    success: bool
    status: int
    message: str
    reached_position: tuple


@dataclass
class ArmState:
    state: int
    state_name: str
    position: tuple
    target_position: tuple
    motor_angles_current: tuple
    motor_angles_target: tuple
    servo_online: tuple
    servo_error: tuple
    has_feedback: bool


def _point(msg):
    return (float(msg.x), float(msg.y), float(msg.z))


class ArmClient:
    def __init__(self, node=None, node_name='arm_interface_client'):
        self._owns_node = node is None
        self._node = node if node is not None else rclpy.create_node(node_name)
        self._set_state = self._node.create_client(SetState, 'arm/set_state')
        self._get_state = self._node.create_client(GetState, 'arm/get_state')

    def wait_for_services(self, timeout_sec=5.0):
        deadline = time.monotonic() + timeout_sec
        clients = [self._set_state, self._get_state]
        while rclpy.ok():
            if all(client.service_is_ready() for client in clients):
                return True
            if time.monotonic() >= deadline:
                return False
            rclpy.spin_once(self._node, timeout_sec=0.1)
        return False

    def set_state(self, state):
        if isinstance(state, str):
            value = STATE_BY_NAME.get(state.strip().lower())
            if value is None:
                return ArmResult(False, STATUS_ERROR, 'unknown state %r' % state, (0.0, 0.0, 0.0))
        else:
            value = int(state)

        request = SetState.Request()
        request.state = value
        response = self._call(self._set_state, request)
        if response is None:
            return ArmResult(False, STATUS_ERROR, 'no response from arm/set_state', (0.0, 0.0, 0.0))
        return ArmResult(bool(response.success), int(response.status), response.message,
                         _point(response.reached_position))

    def stop(self):
        return self.set_state(STATE_STOP)

    def manual(self):
        return self.set_state(STATE_MANUAL)

    def home(self):
        return self.set_state(STATE_HOME)

    def hold(self):
        return self.set_state(STATE_HOLD)

    def get_state(self):
        response = self._call(self._get_state, GetState.Request())
        if response is None:
            return ArmState(STATE_MANUAL, 'manual', (0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
                            (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (False, False, False),
                            (0, 0, 0), False)
        return ArmState(
            int(response.state),
            response.state_name,
            _point(response.position),
            _point(response.target_position),
            tuple(float(v) for v in response.motor_angles_current),
            tuple(float(v) for v in response.motor_angles_target),
            tuple(bool(v) for v in response.servo_online),
            tuple(int(v) for v in response.servo_error),
            bool(response.has_feedback),
        )

    def _call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future)
        if not future.done():
            return None
        return future.result()

    def destroy(self):
        if self._owns_node:
            self._node.destroy_node()
