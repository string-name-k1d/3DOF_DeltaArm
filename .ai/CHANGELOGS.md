# Changelog

All notable changes to the `arm` delta-arm workspace are documented here.
Entries are grouped by major phase. The working tree is **staged but uncommitted**.

---

## [Unreleased] — Workspace reorganization: `my_arm` → `arm` + source categorization

### Added
- `AGENT.md` — instructions for AI agents (workspace snapshot, build/test,
  conventions, what is a skeleton, pre-help checklist).
- `.ai/ARCHITECTURE.md` — full architecture document (packages, interfaces,
  source layout by node, data flow, process modes, design decisions).
- `.ai/CHANGELOGS.md` — this file.
- Two-stage IK skeleton with `Arm::ik_stage1` / `ik_stage2` and `Arm::apply()`.
- Emergency-stop service (`arm/emergency_stop`): cancels active goals, rejects
  new goals, zeroes all motor outputs + pose.
- Generic hardware query service (`arm/motor_param_query`) with request-type
  dispatch (angle, voltage, current, power, temperature, ping).
- WASD manual-control node (`arm_manual`) + `manual.launch`.
- `test.launch.py` — launches the GTest binaries from a colcon install.
- GTest `test_arm` unit tests: MotorBookkeeping, SetTarPosRunsPipeline,
  EmergencyStopZeroesOutputs.
- Simulation launch mode (`simulation:=true`): launches only the controller
  node; the driver is provided externally over the same interface names.
- `Dockerfile` (Ubuntu 22.04 / ROS 2 Humble) with build tools, USB/serial
  utilities, and a `rosuser` in the `dialout`/`plugdev` groups.

### Changed
- **Package names**: `my_arm_msgs` → `arm_msgs`, `my_arm_hardware` →
  `arm_hardware`, `my_arm_bringup` → `arm_bringup`, `my_arm_description` →
  `arm_description`.
- **ROS graph names** renamed for consistency: all topics/services/actions/
  parameters now use the `arm/` prefix (e.g. `arm/set_pos`, `arm/pos`,
  `arm/emergency_stop`, `arm/motor_param_query`).
- **Node exec names**: `delta_arm_controller` → `arm_controller`,
  `motor_driver` → `arm_motor_driver`, `delta_arm_system` → `arm_system`,
  manual control → `arm_manual`.
- **Source files reorganized by node** under `src/arm_hardware/src/`:
  - `axis/` — shared kinematics library (`delta-arm.cpp`)
  - `controller/` — arm controller node sources
  - `driver/` — motor-driver node sources (+ FSUS wrapper)
  - `manual/` — WASD manual-control sources
  - `system/` — combined controller+driver main
- `CMakeLists.txt` source paths updated to match the new categorized layout.
- `config/my_arm_params.yaml` → `config/arm_params.yaml`
  (per-node parameter namespaces: `arm_controller`, `arm_motor_driver`,
  `arm_manual_control`).
- `.vscode/c_cpp_properties.json` include paths updated to the new structure.

### Removed
- Stale root-level `include/` and `src/` (monolithic `delta_arm` package) —
  superseded by the multi-package `src/arm_*` layout.
- Top-level `motor_driver/` (FashionStart submodule relocated into
  `src/arm_hardware/motor_driver/`).

---

## History (pre-reorg)

### `delta_arm` (original monolithic package)
- Single ROS 2 package with action (`SetPosition`), messages (`ArmPosition`,
  `MotorTargets`), services (`TogglePositionStream`, `MotorParamQuery`), and
  emergency-stop support.
- `delta_arm_system` executable ran controller + driver in one process.
- `.gitmodules` referenced the FashionStart submodule at repo root.
