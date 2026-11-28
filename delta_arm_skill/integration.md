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

`arm_interface_node` bridges the action/topic layer to a small state machine.
Start it with `interface:=true` on any bringup. The default state is `manual`.

| Service | Type | Purpose |
|---|---|---|
| `arm/set_state` | `arm/srv/SetState` | transition to `stop` / `manual` / `home` / `hold`. |
| `arm/get_state` | `arm/srv/GetState` | current state plus snapshot of `arm/pos` and `arm/MotorFeedback`. |

State values: `stop=0`, `manual=1`, `home=2`, `hold=3`.

- `manual`: cancel any interface-owned move and leave `arm/set_pos` to the raw
  controller/manual node.
- `home`: drive to `arm_interface.home_position` via `arm/set_pos`, then enter
  `home`.
- `stop`: cancel the interface-owned move and enter `stop`.
- `hold`: placeholder state (TBD); no new command is issued.

Response status constants:

```text
STATUS_OK = 0
STATUS_UNREACHABLE = 1
STATUS_POS_NOT_REACHED = 2
STATUS_ACTION_UNAVAILABLE = 3
STATUS_CANCELED = 4
STATUS_ERROR = 5
```

When `control:=false` the action server is absent; state queries still work, but
requesting `home` returns `STATUS_ACTION_UNAVAILABLE` and the state is unchanged.

## Python client

```python
from arm_interface import ArmClient

arm = ArmClient(node_name='dock_arm_client')
arm.wait_for_services(timeout_sec=5.0)

arm.manual()
arm.home()
state = arm.get_state()
print(state.state_name, state.position, state.motor_angles_current)

arm.stop()
```

`ArmClient` uses only the service layer, so drone nodes do not need to know the
action goal shape or subscribe to low-level motor topics.

## Drone usage note

`drone_bringup` already includes `arm_bringup.launch.py`. Pass
`interface:=true` there when the drone docking/landing sequence needs arm
commands. Keep motion disabled during VIO/AprilTag calibration with
`control:=false`; the interface will still answer `arm/get_state`.
