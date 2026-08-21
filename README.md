# arm — 3DOF Delta Arm (ROS 2 workspace)

3 Degree of Freedom Delta Arm driven by FashionStar UART-servos.

This repo is a **ROS 2 (C++) workspace** (colcon). The original monolithic
`delta_arm` package has been split into a multi-package layout under `src/`.
For real hardware the controller and motor-driver nodes run in a **single
process** (tight coupling between the ROS application and the serial-bus
hardware); they can also be launched in **simulation** mode that runs only the
controller (the arm simulation is provided externally, in another
workspace/package):

| Node                | Exec / executable      | Responsibility                                                          |
|---------------------|------------------------|-------------------------------------------------------------------------|
| Controller          | `arm_controller`    | Task-space `set_pos` action + `get_pos` streaming (two-stage IK).       |
| Motor driver        | `arm_motor_driver`  | Bridges ROS messages to the FashionStar bus-servo hardware (`fsuartservo`). |
| Combined (HW)       | `arm_system`        | Controller + Motor driver, one process (default for real hardware).     |
| Manual control      | `arm_manual`        | Keyboard (WASD) jogging of `set_pos`; prints `get_pos` position changes. |

## Workspace layout

```
3DOF_DeltaArm/                     <-- Git Repository Root
├── README.md
├── .gitignore
├── .gitmodules                    # FashionStart UART servo SDK submodule
├── arm_description/            # Package 1: URDF/meshes (model + frames)
│   ├── package.xml
│   ├── CMakeLists.txt
│   ├── urdf/                      # robot description (Xacro/URDF)
│   └── meshes/                    # STL/DAE visual & collision models
├── arm_msgs/                   # Package 2: Custom ROS interfaces
│   ├── package.xml
│   ├── CMakeLists.txt
│   ├── action/                    # SetPosition.action  (set_pos)
│   ├── msg/                       # ArmPosition.msg, MotorTargets.msg
│   └── srv/                       # TogglePositionStream.srv, MotorParamQuery.srv, EmergencyStop.srv
├── arm_hardware/               # Package 3: Drivers & controllers (C++ library + nodes)
│   ├── package.xml
│   ├── CMakeLists.txt
│   ├── include/arm_hardware/   # C++ headers (delta_arm, controller, manual, motor_driver)
│   ├── src/                       # ros2_control/hardware interface + node mains
│   ├── test/                     # GTest unit tests
│   └── motor_driver/              # FashionStart C++ SDK (git submodule) -> builds fsuartservo + cserialport
├── arm_bringup/                # Package 4: High-level launch + config
│   ├── package.xml
│   ├── CMakeLists.txt
│   ├── config/                   # arm_params.yaml (per-node parameters)
│   └── launch/                   # delta_arm.launch, manual.launch, test.launch.py
├── arm_moveit_config/          # Package 5: MoveIt kinematics & motion planning
│   ├── package.xml
│   ├── CMakeLists.txt
│   ├── config/                   # SRDF, joint_limits, kinematics.yaml
│   └── launch/                   # MoveIt planning & execution launches
└── Dockerfile                     # ROS 2 Humble dev container (Windows build path)
```

> The `arm_description`, `arm_moveit_config`, and the model-side of
> `arm_bringup` packages are scaffolding (their `urdf/`, `meshes/`,
> `config/`, and `launch/` dirs are `.gitkeep`-placeholdered). Fill them in as
> the robot model + MoveIt side of the arm is added. The parts you actually use
> today — `arm_msgs` (interfaces), `arm_hardware` (controller/driver/manual
> nodes, library, tests, and the FashionStart submodule), and
> `arm_bringup` (params + the three working launch files) — are fully
> functional.

## ROS interfaces (`arm_msgs`)

All topics/services/actions are namespaced under `arm/` in node code, but the
canonical **ROS-level** names (used by clients in other packages) are listed
below.

* **`arm/set_pos`** *(action `arm_msgs/action/SetPosition`)*
  Goal is a `geometry_msgs/Twist`; the controller reads the target as
  `linear.x/y/z`, runs two-stage IK and publishes motor commands.

* **`arm/emergency_stop`** *(service `arm_msgs/srv/EmergencyStop`)*
  Emergency interrupt: `activate=true` cancels any in-flight `set_pos` goal,
  rejects new goals, zeroes the motor outputs and republishes them (and pauses
  the `get_pos` stream). `activate=false` releases the stop.

* **`arm/pos`** *(topic `arm_msgs/msg/ArmPosition`)*
  Current end-effector pose + motor angles. Published continuously only while
  streaming is enabled.

* **`arm/get_pos`** *(service `arm_msgs/srv/TogglePositionStream`)*
  `enable=true` → start continuous publishing; `enable=false` → stop.

* **`arm/motor_targets`** *(topic `arm_msgs/msg/MotorTargets`, controller → driver)*
  The three joint-angle commands (degrees).

* **`arm/motor_param_query`** *(service `arm_msgs/srv/MotorParamQuery`)*
  Generic "same interface" hardware query: `servo_id` + `request_type` →
  `response_type` + `value`.

  | request_type | meaning          | value unit |
  |--------------|------------------|------------|
  | 0            | joint angle      | deg        |
  | 1            | voltage          | mV         |
  | 2            | current          | mA         |
  | 3            | power            | mW         |
  | 4            | temperature      | °C         |
  | 5            | ping / online    | 0 or 1     |

## Building (Linux / Ubuntu with ROS 2)

The FashionStart driver is a git submodule:

```bash
git submodule update --init --recursive
```

Place the workspace, install deps, and build:

```bash
# From the repo root (3DOF_DeltaArm)
#   If using the provided Dockerfile (ROS 2 Humble):
#     docker build -t arm .   # builds a Humble image
#     docker run  --rm -it arm
#   Inside the container (or on a native ROS 2 Humble host):
sudo apt-get update && sudo apt-get install -y \
    ros-humble-ament-cmake \
    ros-humble-rclcpp \
    ros-humble-rclcpp_action \
    ros-humble-geometry-msgs \
    ros-humble-std-msgs

colcon build --symlink-install
```

## Running

Launch-file usage (these live in `arm_bringup`):

### Real hardware

```bash
ros2 launch arm_bringup delta_arm.launch                      # launch file
ros2 launch arm_bringup delta_arm.launch simulation:=false    # force hardware
# or, without a launch file:
ros2 run arm_hardware arm_system \
    --ros-args --params-file src/arm_bringup/config/arm_params.yaml
```

### Simulation (no motor driver)

```bash
ros2 launch arm_bringup delta_arm.launch simulation:=true
```

This starts **only the controller node** (`arm_controller`); the motor
driver is **not** run — the arm simulation is provided externally, in another
workspace/package, over the same topic/service/action names
(`arm/set_pos`, `arm/motor_targets`, `arm/pos`, `arm/get_pos`).
Because the interface is identical, the controller code needs no changes
between simulation and hardware.

### Example: drive the end-effector

```bash
# move to (0.15, 0.0, 0.20) via the set_pos action
ros2 action send_goal arm/set_pos arm_msgs/action/SetPosition \
    "{target: {linear: {x: 0.15, y: 0.0, z: 0.20}}}"

# start stream, then watch the pose
ros2 service call arm/get_pos arm_msgs/srv/TogglePositionStream "{enable: true}"
ros2 topic echo arm/pos

# generic hardware query: joint angle of servo 0 (request_type 0)
ros2 service call arm/motor_param_query arm_msgs/srv/MotorParamQuery \
    "{servo_id: 0, request_type: 0}"

# emergency interrupt: halt everything and zero the motor output
ros2 service call arm/emergency_stop arm_msgs/srv/EmergencyStop \
    "{activate: true}"
# release the stop (set_pos accepted again)
ros2 service call arm/emergency_stop arm_msgs/srv/EmergencyStop \
    "{activate: false}"
```

### Run the tests

```bash
ros2 launch arm_bringup test.launch.py
# or, equivalently via colcon:
colcon test --packages-select arm_hardware
```

`test.launch.py` locates and executes the GTest binaries built from
`arm_hardware/test/` (currently `test_arm`, covering arm bookkeeping +
emergency-stop zeroing).

### Manual control (WASD)

```bash
ros2 launch arm_bringup manual.launch                  # simulation (default)
ros2 launch arm_bringup manual.launch simulation:=false   # real hardware
```

This starts the controller (plus the manual-control node) and lets you jog the
arm. `arm_manual` publishes `set_pos` action goals from the keyboard and
prints current-position changes as they come in on the `get_pos` stream:

```
W/S : +/- z    A/D : +/- y     Q : quit
```

The step size and the two axes are configurable:

```bash
ros2 run arm_hardware arm_manual --ros-args \
    -p step:=0.005 -p axis0:=x -p axis1:=z
```

> Note: the WASD axes default to `z` (W/S) and `y` (A/D) in a plane; choose
> `axis0`/`axis1` from `x`, `y`, `z` to jog a different plane.

## Where to implement the math

The inverse kinematics is intentionally left as a **skeleton**
in `src/arm_hardware/src/axis/delta-arm.cpp`:

* `Arm::ik_stage1(const Vec3 &)` — task-space target → per-limb plane orientation.
* `Arm::ik_stage2(float theta, int leg)` — limb plane orientation → motor angle.
* `Arm::apply()` — optional forward-kinematics estimate of `cur_pos_`.

Implement those, then the set_pos action / get_pos stream will reflect real
motion.

## References

* [3DOF Delta Arm](https://people.ohio.edu/williams/html/PDF/DeltaKin.pdf)
* [FashionStart UART Servo C++ Driver](https://github.com/servodevelop/fashionstar-uart-servo-cpp.git)
