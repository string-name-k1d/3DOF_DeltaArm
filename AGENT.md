# AGENT.md — Guide for AI Coding Agents

This file gives AI agents (and human collaborators) the context needed to work on
the `arm` delta-arm package without guessing project conventions.

---

## 1. Workspace snapshot

This repository is a **single ROS 2 package** named `arm`. The package root **is**
the repo root (not a `src/<pkg>` wrapper). When integrated as a submodule in
another project (e.g. `fyp`), the repo is mounted/copied into the container's
`ros2_ws/src/arm` (same pattern as `drone_ctrl` / `apriltag_ros`).

| Concern | Where it lives |
|---|---|
| Custom interfaces (action/msg/srv) | `action/` (SetPosition), `msg/` (ArmPosition, MotorTargets), `srv/` (EmergencyStop, MotorParamQuery, TogglePositionStream) |
| Kinematics + controller + driver code | `src/{axis,controller,driver,manual,sim,system}/` |
| Public headers | `include/arm/` (included as `#include "arm/<header>.hpp"`) |
| GTest unit tests | `test/test_delta_arm.cpp` |
| Launch + params | `launch/`, `config/arm_params.yaml` |
| FashionStart SDK (submodule) | `motor_driver/fashionstart-uart-servo/` |
| Gazebo simulation package | `gazebo/` — a separate package `arm_gazebo` (see §6) |
| Build container | `Dockerfile` (ROS 2 Humble) |

> **Terminology:** the package name is `arm`. C++ namespaces per component:
> `DeltaArm` (kinematics library), `DeltaArmDriver` (driver lib) /
> `DeltaArmDriverNode` (driver ROS node), `DeltaArmRos` (controller + manual),
> `DeltaArmSim` (SFML sim).

## 2. Build & test

This host (Windows) has **no ROS 2 toolchain** — only `g++ -std=c++17` is
available for standalone library/validation checks. The full colcon build uses
the Docker image (see `README.md`, `build_arm.sh`).

### Quick on-host library validation (no ROS)
```bash
# arm kinematics library + IK skeleton
g++ -std=c++17 -Iinclude -fsyntax-only src/axis/delta-arm.cpp

# motor-driver wrapper (needs FashionStar + CSerialPort include dirs)
g++ -std=c++17 \
  -Iinclude \
  -Imotor_driver/fashionstart-uart-servo/include \
  -Imotor_driver/fashionstart-uart-servo/dependency/CSerialPort/include \
  -fsyntax-only src/driver/motor_driver.cpp
```

### Full build (Docker / native ROS 2 Humble)
See `README.md → Building`.

## 3. Coding conventions

- **Language standard:** C++17 (`CMAKE_CXX_STANDARD 17`).
- **Namespaces:** `DeltaArm` for the kinematics library classes (`Arm`, `Motor`, `Vec3`);
  `DeltaArmDriver`/`DeltaArmDriverNode`, `DeltaArmRos`, `DeltaArmSim` for the other nodes.
- **Headers:** installed under `include/arm/` (package-style path), so source files
  use `#include "arm/<header>.hpp"`.
- **ROS style:** `rclcpp`/`rclcpp_action`, action server for `set_pos`,
  service for `emergency_stop` / `get_pos` / `motor_param_query`, topic for
  `motor_targets` and `pos`.
- **Parameters:** namespaced per node in `config/arm_params.yaml`
  (`arm_controller`, `arm_motor_driver`, `arm_manual_control`).
- **Naming:** executables targets include `arm_system`, `arm_controller`,
  `arm_motor_driver`, `arm_manual`, `arm_sim_sfml`; `arm_kinematics` is the
  exported kinematics shared library (in the `arm` package, linked by the
  gazebo sim).

## 4. What is intentionally a skeleton

The inverse kinematics math is **placeholder**. Fill in:
- `Arm::ik_stage1(const Vec3 &)` — task-space target → per-limb plane orientation
- `Arm::ik_stage2(float theta, int leg)` — limb plane orientation → motor angle
- `Arm::apply()` — optional forward-kinematics estimate of `cur_pos_`

The tests in `test/test_delta_arm.cpp` assert the current skeleton behavior
(IK returns zeros, e-stop zeroes outputs) — update them when you implement real
math.

## 5. Before asking for help

1. Does `git status` show only intended changes?
2. Did you `git submodule update --init --recursive`?
3. If editing the driver: re-run the motor_driver syntax check (needs both
   FashionStar and CSerialPort include dirs).
4. If editing interfaces: `ament_generate_interfaces` → bump nothing; just rebuild.

## 6. Gazebo simulation (`gazebo/` = `arm_gazebo` package)

The optional Gazebo sim lives in `gazebo/`, which is its **own ROS 2 package**
named `arm_gazebo` (`package.xml`, `CMakeLists.txt`, `include/arm_gazebo/`,
`src/`, `launch/`, `worlds/`, `urdf/`, `meshes/`, `config/`). It re-uses the
delta-arm kinematics from `arm` instead of recompiling it.

- The kinematics implementation lives in the `arm` package as the exported
  shared library `arm_kinematics` (from `src/axis/delta-arm.cpp`), exported to
  consumers as `arm::arm_kinematics`. `arm` installs its public headers, so an
  external package can `find_package(arm REQUIRED)` and `target_link_libraries`
  with `arm::arm_kinematics`.
- `gazebo/` is **nested inside the `arm` package**, so colcon will not discover
  it from the repo root (a `colcon build` here builds only `arm`). To build
  `arm_gazebo`, stage the repo and `gazebo/` as **siblings** in a parent
  workspace's `src/` (e.g. symlink `src/arm` → repo and `src/arm_gazebo` →
  `gazebo/`), then `colcon build` there. `arm_gazebo` depends on `arm`, so
  colcon orders it correctly.
- `gazebo_ros` / `glm` are found via `find_package(... QUIET)` and the
  `arm_sim_gazebo` executable is only built when they are present, so a
  headless build of `arm` stays unaffected.

### Verify the sibling build (Docker)
```bash
source /opt/ros/humble/setup.bash
mkdir -p /tmp/ws/src
ln -s <repo>         /tmp/ws/src/arm
ln -s <repo>/gazebo  /tmp/ws/src/arm_gazebo
cd /tmp/ws && colcon build --symlink-install
```
