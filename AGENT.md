# AGENT.md — Guide for AI Coding Agents

This repository is a single ROS 2 package named `arm`. The package root **is**
the repository root. When integrated as a submodule, it is mounted/copied into
`ros2_ws/src/arm`.

## Layout

| Concern | Location |
|---|---|
| Interfaces | `action/SetPosition.action`, `msg/{ArmPosition,MotorFeedback,MotorTargets}.msg` |
| Kinematics library | `src/controller/delta-arm_dynamics.cpp`, `include/arm/delta_arm.hpp` |
| Controller node | `src/controller/`, `include/arm/delta_arm_controller.hpp` |
| Servo driver | `src/driver/`, `include/arm/motor_driver*.hpp` |
| Manual control | `src/manual/`, `include/arm/delta_arm_manual.hpp` |
| SFML 2-D simulator | `src/sim/`, `include/arm/arm_sim_sfml.hpp` |
| Launch / params | `launch/{sim_2d,hardware,manual}.launch.py` (shared `arm_bringup.launch.py`), `gazebo/launch/sim_3d.launch.py`, `config/` |
| Unit tests | `test/test_delta_arm.cpp` |
| FashionStar SDK (submodule) | `motor_driver/fashionstart-uart-servo/` |
| Gazebo package (separate) | `gazebo/` |

## Build & test

```bash
git submodule update --init --recursive
colcon build --symlink-install
colcon test --packages-select arm
```

A Docker image with ROS 2 Humble is provided (`Dockerfile`, `compose.yaml`).
On a host without a local ROS installation, build with:

```bash
docker run --rm --userns=keep-id -e HOME=/tmp \
  -v "$PWD":/ws:Z -w /ws --entrypoint bash \
  docker.io/library/fyp:latest -lc \
  'source /opt/ros/humble/setup.bash && colcon build'
```

For the container plus an interactive shell use `scripts/launch_container.sh`;
it forwards `DISPLAY`/`XAUTHORITY` only when the host paths exist, so the SFML
window works on a desktop and headless runs still start.

## Conventions

- C++17.
- Namespaces: `DeltaArm` (kinematics), `DeltaArmRos` (controller/manual),
  `DeltaArmDriver` / `DeltaArmDriverNode` (driver), `DeltaArmSim` (SFML sim).
- Public headers live in `include/arm/` and are included as
  `#include "arm/<header>.hpp"`.
- ROS names: `arm/set_pos` action, `arm/pos` feedback topic,
  `arm/motor_targets` (controller → driver), `arm/MotorFeedback`
  (driver → consumers), optional 2-D debug `arm/joints`.
- `control:=true` is the launch default (`manual:=false`). `enable_motion` and
  `publish_joints` are runtime launch parameters, never keys in `config/*.yaml`.

## Gazebo (`gazebo/` = `arm_gazebo`)

`gazebo/` is a **separate package nested inside** the `arm` package, so colcon
will not discover it from the repo root. To build it, stage `arm` and `gazebo/`
as siblings under a parent workspace's `src/`:

```bash
mkdir -p /tmp/ws/src
ln -s <repo>        /tmp/ws/src/arm
ln -s <repo>/gazebo /tmp/ws/src/arm_gazebo
cd /tmp/ws && colcon build --symlink-install
```

The Gazebo bridge consumes `arm/motor_targets`; the kinematics are reused from
the exported `arm_kinematics` library.
