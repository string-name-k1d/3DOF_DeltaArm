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
ros2 service call /arm/move_to arm/srv/MoveTo \
  "{target: {x: 0.0, y: 0.0, z: -0.28}, timeout: 10.0}"
ros2 service call /arm/get_state arm/srv/GetState "{}"
ros2 service call /arm/home arm/srv/Home "{}"
ros2 service call /arm/stop arm/srv/Stop "{}"
```

Python clients can use:

```python
from arm_interface import ArmClient

arm = ArmClient()
arm.wait_for_services()
arm.move_to(0.0, 0.0, -0.28)
state = arm.get_state()
```

Motion is disabled by `control:=false`; the interface then still answers state
queries but refuses move commands.
