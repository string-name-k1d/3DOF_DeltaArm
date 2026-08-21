# Architecture

The `arm` delta-arm workspace is a **ROS 2 (C++) colcon workspace** split into
five packages. It drives a 3-DOF delta arm whose actuators are FashionStar
UART bus-servos.

---

## Packages

```
src/
├── arm_description/      # URDF + meshes (scaffolding)
├── arm_msgs/             # Custom actions / messages / services
├── arm_hardware/         # Library + nodes + driver binding + tests
├── arm_bringup/          # Params + launch files
└── arm_moveit_config/    # MoveIt config (scaffolding)
```

### `arm_msgs`

Custom interfaces (all namespaced under the ROS graph as `arm/...`):

| Name | Type | Direction | Purpose |
|---|---|---|---|
| `arm/set_pos` | Action (`SetPosition`) | client→controller | Goal `geometry_msgs/Twist` (linear.x/y/z = target); runs IK. |
| `arm/emergency_stop` | Service (`EmergencyStop`) | client→controller | `activate=true`: cancel active goal, reject new goals, zero all motor outputs, pause stream. `false`: release. |
| `arm/pos` | Topic (`ArmPosition`) | controller→consumer | Current pose + motor angles (continuous only while streaming). |
| `arm/get_pos` | Service (`TogglePositionStream`) | client→controller | Enable / disable the `pos` stream. |
| `arm/motor_targets` | Topic (`MotorTargets`) | controller→driver | Three joint-angle commands (deg). |
| `arm/motor_param_query` | Service (`MotorParamQuery`) | client→driver | Generic hardware query: `servo_id` + `request_type` → `response_type` + `value`. |

`request_type` mapping: `0`=angle(deg), `1`=voltage(mV), `2`=current(mA),
`3`=power(mW), `4`=temperature(°C), `5`=ping/online.

### `arm_hardware`

Kinematics library + ROS 2 nodes + FashionStar driver binding.

#### Source layout (by node)

```
arm_hardware/src/
├── axis/                 # Shared kinematics library (not a node)
│   └── delta-arm.cpp     # DeltaArm::Arm, DeltaArm::Motor, IK skeleton
├── controller/           # Controller node (set_pos action + get_pos stream)
│   ├── delta_arm_controller_node.cpp
│   └── delta_arm_controller_main.cpp
├── driver/               # Motor-driver node (FSUS binding) + shared lib entry
│   ├── motor_driver.cpp        # FSUS wrapper lib (ROS-neutral)
│   ├── motor_driver_node.cpp   # driver node: motor_targets -> servos
│   └── motor_driver_main.cpp   # standalone driver main
├── manual/               # WASD manual-control action client
│   ├── delta_arm_manual.cpp
│   └── delta_arm_manual_main.cpp
└── system/               # Combined controller+driver (one process)
    └── delta_arm_system_main.cpp
```

#### Build targets

| Target | Type | Sources |
|---|---|---|
| `arm_hardware` (lib) | SHARED library | `src/driver/motor_driver.cpp` |
| `arm_system` | Executable | axis/ + controller/ + driver/node + system/main |
| `arm_controller` | Executable | axis/ + controller/ (+ controller main) |
| `arm_motor_driver` | Executable | driver/ (standalone) |
| `arm_manual` | Executable | manual/ |
| `test_arm` | GTest | axis/ + `test/test_delta_arm.cpp` |

The FashionStar SDK (`motor_driver/fashionstart-uart-servo/`) is built via
`add_subdirectory(motor_driver)` using its own CMakeLists (which builds
`fsuartservo` on top of `cserialport`).

## Data flow

```
  action goal (Twist x/y/z) -> arm_controller -> ik_stage1/ik_stage2 -> motor_targets (deg)
                                                                           |
                                                                           v
  arm/pos (feedback) <- get_pos stream        arm_motor_driver -> FSUS servos (UART)
```

## Node / process modes

| Mode | Exec | What runs |
|---|---|---|
| Real hardware | `arm_system` | Controller + driver in **one process**. Default. |
| Simulation | `arm_controller` | Controller only. Driver provided externally (same interface). |
| Standalone driver | `arm_motor_driver` | Driver node only (debugging / external controller). |
| Manual jog | `arm_manual` | WASD client for `arm/set_pos`; prints `arm/pos` changes. |

## Key design decisions

1. **Two-stage IK skeleton** — `ik_stage1` (task-space → limb plane) and
   `ik_stage2` (plane → motor angle) in `src/axis/delta-arm.cpp`. Math is
   deliberately placeholder (returns zeros) so the full pipeline can be
   validated before kinematics are filled in.
2. **Single-process coupling** — `arm_system` runs controller + driver together.
3. **Simulation-friendly interface** — `simulation:=true` launches only
   `arm_controller`; an external sim uses the same topic/service/action names.
4. **Generic hardware query** — `motor_param_query` serves all query types
   through one service contract.
5. **Emergency stop** — latched flag that cancels goals, rejects new ones, and
   zeroes all motor outputs + pose, keeping a safe state.
