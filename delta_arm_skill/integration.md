# ROS interfaces and high-level API

## Low-level interfaces

| Name | Type | Direction | Purpose |
|---|---|---|---|
| `arm/set_pos` | action `SetPosition` | client -> controller | goal payload is `geometry_msgs/Twist`; `linear.x/y/z` is metres. |
| `arm/pos` | topic `ArmPosition` | controller -> consumers | current pose, target pose, current/target motor angles. |
| `arm/joints` | topic `JointState` | controller -> consumers | optional 2-D debug stream. |
| `arm/motor_targets` | topic `MotorTargets` | controller -> driver | three motor angles in degrees. |
| `arm/MotorFeedback` | topic `MotorFeedback` | driver -> consumers | measured angle, online flag, error code. |

## High-level services

`arm_interface_node` bridges the action/topic layer to simple services. Start it
with `interface:=true` on any bringup.

| Service | Type | Purpose |
|---|---|---|
| `arm/move_to` | `arm/srv/MoveTo` | move to a `Point` and wait for the action result. |
| `arm/get_state` | `arm/srv/GetState` | snapshot of `arm/pos` and `arm/MotorFeedback`. |
| `arm/home` | `arm/srv/Home` | move to `arm_interface.home_position`. |
| `arm/stop` | `arm/srv/Stop` | cancel the active move. |

Response status constants are shared by `MoveTo` and `Home`:

```text
STATUS_OK = 0
STATUS_UNREACHABLE = 1
STATUS_POS_NOT_REACHED = 2
STATUS_ACTION_UNAVAILABLE = 3
STATUS_CANCELED = 4
STATUS_ERROR = 5
```

When `control:=false` the action server is absent; state queries still work, but
move/home return `STATUS_ACTION_UNAVAILABLE`.

## Python client

```python
from arm_interface import ArmClient

arm = ArmClient(node_name='dock_arm_client')
arm.wait_for_services(timeout_sec=5.0)

arm.move_to(0.0, 0.0, -0.28, timeout=10.0)
state = arm.get_state()
print(state.position.x, state.motor_angles_current)

arm.stop()
arm.home()
```

`ArmClient` uses only the service layer, so drone nodes do not need to know the
action goal shape or subscribe to low-level motor topics.

## Drone usage note

`drone_bringup` already includes `arm_bringup.launch.py`. Pass
`interface:=true` there when the drone docking/landing sequence needs arm
commands. Keep motion disabled during VIO/AprilTag calibration with
`control:=false`; the interface will still answer `arm/get_state`.
