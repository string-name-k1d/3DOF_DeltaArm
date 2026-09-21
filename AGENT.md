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
- The 3-D model is generated **at launch time** from the URDF xacro
  descriptors `gazebo/urdf/delta_arm.urdf.xacro` (+ the plugin block in
  `delta_arm.gazebo.xacro`); `arm_gazebo.launch.py` runs `xacro` to a robot
  description and spawns it into `gazebo/worlds/arm_world.sdf` (DART `pgs`).
- The model uses the SAME mechanism parameters and IK/FK maths as the 2-D sim
  (shared `geometry` values in `arm/config/arm_params.yaml`): shoulder pivots
  at `base_radius` = 100 mm, upper arms `upper_arm_len` = 120 mm (revolute,
  actuated), lower rods `lower_arm_len` = 240 mm, platform at
  `platform_radius` = 32.5 mm hanging below the base. The mechanism is an
  **open kinematic tree** — each leg is an elbow/wrist pair of 3-revolute
  (X,Y,Z) chains whose rod tip is welded to the platform dummy at runtime by a
  `DetachableJoint` plugin, closing the three loops in physics. Each shoulder
  is servoed by a `JointPositionController` on `/delta_arm/shoulder_<i>/cmd_pos`
  (rad); the `arm_cmd_bridge` node maps the controller's `arm/motor_targets`
  (deg) onto those three topics (and publishes zeros shortly after start so
  the servos latch the home pose). Verified: commands track (e.g. 20°/-10°/5°
  → ≈0.38/-0.11/0.13 rad at the joints) and the platform follows in 3-D
  (tool0 moves with asymmetric commands); the shoulders rest a few degrees
  off zero (~0.117 rad) — the soft static equilibrium of the pgs-closed loop,
  not a true home error. Targets needing a negative arm angle swing the upper
  arm UP into the base plate (solid disc in the model) → self-blocking; the
  nominal workspace is positive arm angles (servo range [0, 145] deg).
- The Harmonic stack is NOT baked into `arm:latest`; install it on demand with
  `arm/scripts/install_gazebo_harmonic.sh` (OSRF repo + gz-harmonic +
  `ros-humble-ros-gzharmonic` + `robot_state_publisher`). The `--view 3d`
  launcher (`run_arm.sh`) detects a missing install and prints this command.
- The legacy Classic/torque-plugin path (`delta_arm_gazebo_plugin`,
  `arm_sim_gazebo`, `worlds/delta_arm.world`) was removed in the Harmonic
  rewrite.

### Verify the sibling build (Docker)
```bash
source /opt/ros/humble/setup.bash
mkdir -p /tmp/ws/src
ln -s <repo>         /tmp/ws/src/arm
ln -s <repo>/gazebo  /tmp/ws/src/arm_gazebo
cd /tmp/ws && colcon build --symlink-install
```
