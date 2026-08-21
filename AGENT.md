# AGENT.md — Guide for AI Coding Agents

This file gives AI agents (and human collaborators) the context needed to work on
the `arm` delta-arm workspace without guessing project conventions.

---

## 1. Workspace snapshot

| Concern | Where it lives |
|---|---|
| Custom interfaces (action/msg/srv) | `src/arm_msgs/`  (`SetPosition`, `ArmPosition`, `MotorTargets`, `TogglePositionStream`, `MotorParamQuery`, `EmergencyStop`) |
| Controller + driver library + nodes | `src/arm_hardware/` |
| Source files categorized by node | `src/arm_hardware/src/{axis,controller,driver,manual,system}/` |
| GTest unit tests | `src/arm_hardware/test/test_delta_arm.cpp` |
| Launch + params | `src/arm_bringup/` |
| FashionStart SDK (submodule) | `src/arm_hardware/motor_driver/fashionstart-uart-servo/` |
| Robot model / URDF | `src/arm_description/` |
| MoveIt config | `src/arm_moveit_config/` |
| Build container | `Dockerfile` (ROS 2 Humble) |

> **Terminology:** the package/library names use `arm_*`. The C++ *namespace*
> inside the kinematics library is `DeltaArm` (unchanged by the `arm_*` naming
> refactor — that was a package/directory rename, not a namespace rename).

## 2. Build & test

This host (Windows) has **no ROS 2 toolchain** — only `g++ -std=c++17` is
available for standalone library/validation checks. The full colcon build uses
the Docker image.

### Quick on-host library validation (no ROS)
```bash
# arm kinematics library + IK skeleton
g++ -std=c++17 -Isrc/arm_hardware/include -fsyntax-only src/arm_hardware/src/axis/delta-arm.cpp

# motor-driver wrapper (needs FashionStar + CSerialPort include dirs)
g++ -std=c++17 \
  -Isrc/arm_hardware/include \
  -Isrc/arm_hardware/motor_driver/fashionstart-uart-servo/include \
  -Isrc/arm_hardware/motor_driver/fashionstart-uart-servo/dependency/CSerialPort/include \
  -fsyntax-only src/arm_hardware/src/driver/motor_driver.cpp
```

### Full build (Docker / native ROS 2 Humble)
See `README.md → Building`.

## 3. Coding conventions

- **Language standard:** C++17 (`CMAKE_CXX_STANDARD 17`).
- **Namespace:** `DeltaArm` for the kinematics library classes (`Arm`, `Motor`, `Vec3`).
- **Headers:** installed under `include/arm_hardware/` (package-style path), so
  source files use `#include "arm_hardware/<header>.hpp"`.
- **ROS style:** `rclcpp`/`rclcpp_action`, action server for `set_pos`,
  service for `emergency_stop` / `get_pos` / `motor_param_query`, topic for
  `motor_targets` and `pos`.
- **Parameters:** namespaced per node in `src/arm_bringup/config/arm_params.yaml`
  (`arm_controller`, `arm_motor_driver`, `arm_manual_control`).
- **Naming:** executables target names match package+role
  (`arm_system`, `arm_controller`, `arm_motor_driver`, `arm_manual`).

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
