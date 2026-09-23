# Changelog

All notable changes to the `arm` delta-arm workspace are documented here.
Entries are grouped by major phase. The working tree is **staged but uncommitted**.

---

## [Unreleased] — Fold the control augmenter into the controller (logic-only)

The control augmenter is no longer a ROS node. It was rewritten as a
**logic-only** class (`ControlAugmenter`, `include/arm/control_augmenter.hpp` +
`src/system/control_augmenter.cpp` — no publishers/subscriptions, no rclcpp
dependency beyond the generated message type) and **combined into the
controller node**, which runs it on every `MotorTargets` message in
`publishTargets()` before publishing.

### Changed
- Removed the `arm_control_augmenter` node and its relay
  (`arm/motor_targets` → `arm/motor_targets_cmd` → driver). The controller now
  publishes the already-augmented stream on `arm/motor_targets`, and the
  driver consumes it directly (its default `targets_topic`).
- Folded the augmenter parameters (`offset_deg`, `enable_feedforward`,
  `max_delta_deg`) from the `arm_control_augmenter` block into `arm_controller`
  in `config/arm_params.yaml`.
- `arm_system` (`delta_arm_system_main.cpp`) no longer registers a third node:
  controller + motor driver on one executor, driver on the default topic.
- `ControlAugmenter` computes the message-interval `dt` from
  `std::chrono::steady_clock` internally (no rclcpp `now()` in the logic file)
  and exposes `reset()`, which the controller calls on emergency stop so the
  zeroed output is not slew-limited.

### Note
The pipeline remains an identity passthrough by default (offsets `[0,0,0]`,
feedforward off, slew `0`), so behaviour is unchanged with the stock config.
Because the augmenter is now inside the controller, the configured
`offset_deg`/`max_delta_deg` also apply to every controller-driven path
(full-hardware `arm_system`, the 2-D mirror `arm_sim_2d.launch.py motor:=true`,
and standalone `arm_controller`), not only the single-process `arm_system`.

---

## [Unreleased] — Isolate Gazebo simulation into an independent `arm_gazebo` package

The optional Gazebo sim was moved out of the `arm` package into its own ROS 2
package `arm_gazebo`, physically under `gazebo/`, so the core `arm` package
builds without any Gazebo dependencies.

### Added
- New package `gazebo/` (`arm_gazebo`) with its own `package.xml`,
  `CMakeLists.txt`, `include/arm_gazebo/`, `src/`, `launch/`, `worlds/`,
  `urdf/`, `meshes/`, `config/`.
  - `ArmGazebo::ArmSimGazeboNode` (`src/arm_sim_gazebo.cpp`) — a Gazebo-side
    delta-arm simulator that re-uses `DeltaArm::Arm` kinematics from `arm`:
    subscribes `arm/target_position`, publishes `arm/current_position`.
  - `launch/arm_gazebo.launch` — controller + gazebo sim nodes.
- The `arm` package now builds an exported shared **`arm_kinematics`** library
  from `src/axis/delta-arm.cpp`, exported as `arm::arm_kinematics`, so both the
  `arm` executables and the external `arm_gazebo` package link the same IK code
  instead of recompiling it.
- `arm` now installs its public headers (`install(DIRECTORY include/ ...)`) so
  `find_package(arm)` provides `arm/delta_arm.hpp` to downstream packages.

### Changed
- `arm` `CMakeLists.txt`: removed `find_package(gazebo_ros/glm QUIET)`, the
  `arm_sim_gazebo` target, and its installs; executables/tests now link
  `arm_kinematics` instead of compiling `src/axis/delta-arm.cpp` directly.
- `arm` `package.xml`: removed Gazebo-only deps (gazebo_ros,
  gazebo_ros2_control, robot_state_publisher, xacro, joint_state_publisher,
  controller_manager); kept `sfml`, `ros2launch`.
- Removed empty root `worlds/`, `urdf/`, `meshes/` directories (now under
  `gazebo/`).

### Build note
`gazebo/` is a package nested inside the `arm` package, so colcon does **not**
auto-discover it. To build it, stage the repo and `gazebo/` as **siblings**
under a parent `src/` (e.g. symlink `src/arm` → repo, `src/arm_gazebo` →
`gazebo/`), then run `colcon build` there. Verified: both `arm` and
`arm_gazebo` compile, link, and install cleanly in that layout.

---

## [Unreleased] — Flatten `src/arm/*` multi-package workspace → single `arm` package at repo root

The repo had become a colcon **workspace** with a single `src/arm` package
inside it. To match the integration pattern used by sibling projects (packages
live at the repo root and get mounted/copied into the container's
`ros2_ws/src/<pkg>`), the package was flattened so the **package root is the
repo root**.

### Changed
- Moved everything from `src/arm/*` up to the repo root, dropping the
  `src/arm/` wrapper:
  - `src/arm/{package.xml,CMakeLists.txt}` → root
  - `src/arm/src/{axis,controller,driver,manual,sim,system}/` → `src/...`
  - `src/arm/include/arm/` → `include/arm/`
  - `src/arm/{action,msg,srv,launch,config,test,urdf,meshes,worlds}` → root
  - `src/arm/motor_driver/` → `motor_driver/`
- **Submodule relocated**: `fashionstart-uart-servo` moved from
  `src/arm/motor_driver/` to `motor_driver/`. Updated `.gitmodules` path, the
  submodule `.git` gitdir pointer, and its internal `core.worktree`.
- **Build / runtime paths**: `build_arm.sh` colcon `--base-paths` now targets
  the repo root; the params file moved to `config/arm_params.yaml`.
- **Docs**: `README.md` params/IK paths, `AGENT.md` workspace snapshot +
  validation commands, and `.vscode/c_cpp_properties.json` include paths all
  updated to the repo-root layout.
- The `src/arm` gitlinks/entries were removed from the index; git detects the
  flatten as a series of renames.

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
