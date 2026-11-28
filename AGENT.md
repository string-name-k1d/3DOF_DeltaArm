# AGENT.md

The package root is the repository root. Native layout: `arm_ws/src/fyp/arm`.
Detailed notes are in [`delta_arm_skill/`](delta_arm_skill/SKILL.md).

## Layout

| Concern | Location |
|---|---|
| Interfaces | `action/`, `msg/`, `srv/` |
| Kinematics | `include/arm/delta_arm.hpp`, `src/controller/delta-arm_dynamics.cpp` |
| Controller | `src/controller/`, `include/arm/delta_arm_controller.hpp` |
| Driver | `src/driver/`, `include/arm/motor_driver*.hpp` |
| Manual control | `src/manual/` |
| State interface | `arm_interface/`, `launch/interface.launch.py` |
| Simulators | `src/sim/`, `gazebo/` (separate `arm_gazebo` package) |
| Launch / params | `launch/`, `config/` |
| Tests | `test/test_delta_arm.cpp` |
| Deep notes | `delta_arm_skill/` |

## Build & test

Run from the workspace root:

```bash
source /opt/ros/humble/setup.bash
git -C src/fyp/arm submodule update --init --recursive
cp src/fyp/arm/.env.example src/fyp/arm/.env  # once, then edit
./src/fyp/arm/scripts/setup.sh
source install/setup.bash
colcon test --packages-select arm
```

Plain `colcon build` ignores `.env`; pass `--cmake-args
-DARM_BUILD_SFML_SIM=OFF` explicitly to skip SFML. Never make SFML a hard
dependency.

## Conventions

- C++17; namespaces `DeltaArm`, `DeltaArmRos`, `DeltaArmDriver`,
  `DeltaArmDriverNode`, `DeltaArmSim`.
- Public headers use `#include "arm/<header>.hpp"`.
- `enable_motion`, `publish_joints`, `driver`, `manual`, `interface` are launch
  switches, never YAML keys.
- High-level control uses `arm/set_state` (`stop`/`manual`/`home`/`hold`) and
  feedback uses `arm/get_state`; `manual` is the default state and leaves the
  raw `arm/set_pos` action as the active control path.
- `SERIAL_DEVICE` is applied by `arm_bringup.launch.py` after params loading.
- Source comments are short; design rationale belongs in `delta_arm_skill/`.

## Gazebo

`gazebo/` is a separate package not discovered from this repo root. Stage it as
a sibling of `arm` under the workspace `src/` before building `arm_gazebo`.
