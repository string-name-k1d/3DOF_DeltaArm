# AGENT.md — Guide for AI Coding Agents

This repository is a single ROS 2 package named `arm`. The package root **is**
the repository root. When integrated as a submodule it is vendored into a
workspace, for example:

- native install: `arm_ws/src/fyp/arm` (Ubuntu 22.04 + ROS 2 Humble, no Docker)
- Docker: `/root/ros2_ws/src/arm`, or `/root/ws` when built standalone

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

Native (preferred on the target device: Ubuntu 22.04 + ROS 2 Humble, no
Docker). Run colcon from the workspace root, e.g. `~/arm_ws`:

```bash
source /opt/ros/humble/setup.bash
cd ~/arm_ws
git -C src/fyp/arm submodule update --init --recursive   # FashionStar SDK
rosdep install --from-paths src --ignore-src -r -y
./src/fyp/arm/scripts/setup.sh                           # creates .env, builds
. ./.env
source install/setup.bash
colcon test --packages-select arm
```

`scripts/setup.sh` creates/sources `.env`, auto-detects the workspace root,
and passes `-DARM_BUILD_SFML_SIM=...` from it. Plain `colcon build` ignores
`.env`; pass `--cmake-args -DARM_BUILD_SFML_SIM=OFF` manually instead.

SFML is optional and is **not** declared in `package.xml`, so `rosdep` and the
build work without it. `arm_sim_sfml` (2-D view) is built only when
`libsfml-dev` is installed. Never make SFML a hard build dependency: the
hardware/controller/driver/test targets must configure and build on a headless
machine without it.

A Docker image with ROS 2 Humble is also provided (`Dockerfile`,
`compose.yaml`). On a host without a local ROS installation, build with:

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
