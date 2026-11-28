# Delta Arm Skill

Detailed notes for the `arm` ROS 2 package. Source files keep only short
descriptions; design notes, derivations and diagnostics live here.

## Read first

- [kinematics.md](kinematics.md) — geometry conventions, two-stage IK, 4-bar solve, FK.
- [parameters.md](parameters.md) — every runtime parameter and which file owns it.
- [motion_limits.md](motion_limits.md) — measured motor range, derived bands, fold/branch issue.
- [integration.md](integration.md) — ROS interfaces and the high-level `arm_interface` API.
- [implementation_notes.md](implementation_notes.md) — node-by-node behaviour and launch details.

## High-level usage

```bash
ros2 launch arm hardware.launch.py interface:=true
ros2 service call /arm/set_state arm/srv/SetState "{state: 1}"  # manual
ros2 service call /arm/set_state arm/srv/SetState "{state: 2}"  # home
ros2 service call /arm/set_state arm/srv/SetState "{state: 0}"  # stop
ros2 service call /arm/set_state arm/srv/SetState "{state: 3}"  # hold
ros2 service call /arm/get_state arm/srv/GetState "{}"
```

Python clients can use:

```python
from arm_interface import ArmClient

arm = ArmClient()
arm.wait_for_services()
arm.manual()
arm.home()
state = arm.get_state()
print(state.state_name, state.position)
```

The interface starts in `manual`, leaving the raw `arm/set_pos` action as the
default control path. Motion is disabled by `control:=false`; the interface then
still answers state queries but `home` reports `STATUS_ACTION_UNAVAILABLE`.
