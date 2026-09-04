# arm — 3DOF Delta Arm (ROS 2 workspace)

3 Degree of Freedom Delta Arm driven by FashionStar UART-servos.

This repo is a **single ROS 2 (C++) package** named `arm`. The package root
**is** the repo root (no `src/<pkg>` wrapper). When integrated into another
project, the repo is mounted/copied into the container's `ros2_ws/src/arm`. An
optional, independent Gazebo simulation package (`arm_gazebo`) lives under
`gazebo/` and is built as a sibling (see below).

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
3DOF_DeltaArm/                     <-- Git Repository Root (= the `arm` package)
├── README.md
├── AGENT.md                          # Guide for AI agents (build/test/conventions)
├── .gitignore
├── .gitmodules                    # FashionStart UART servo SDK submodule
├── .ai/                             # Agent docs: ARCHITECTURE.md, CHANGELOGS.md
├── action/                          # SetPosition.action  (set_pos)
├── msg/                             # ArmPosition.msg, MotorTargets.msg
├── srv/                             # TogglePositionStream.srv, MotorParamQuery.srv, EmergencyStop.srv
├── include/arm/                     # C++ headers (delta_arm, controller, manual, motor_driver)
├── src/{axis,controller,driver,manual,sim,system}/   # node sources + kinematics
├── launch/                          # delta_arm.launch, manual.launch, test.launch.py
├── config/                          # arm_params.yaml (per-node parameters)
├── test/                            # GTest unit tests (test_delta_arm.cpp)
├── motor_driver/                    # FashionStart C++ SDK (git submodule) -> fsuartservo + cserialport
├── gazebo/                          # SEPARATE package `arm_gazebo` (Gazebo sim)
│   ├── package.xml
│   ├── CMakeLists.txt
│   ├── include/arm_gazebo/
│   ├── src/                         # arm_sim_gazebo node
│   ├── launch/                      # arm_gazebo.launch
│   ├── worlds/ urdf/ meshes/ config/
└── Dockerfile                       # ROS 2 Humble build container (USB/serial for HW access)
```

> The kinematics implementation is built into the exported `arm_kinematics`
> shared library (from `src/axis/delta-arm.cpp`), exposed to downstream
> packages as `arm::arm_kinematics`. The `arm_gazebo` package re-uses it rather
> than recompiling the IK.
>
> `gazebo/` is a package **nested inside** the `arm` package, so colcon does not
> discover it from the repo root. To build it, stage `arm` and `gazebo/` as
> **siblings** in a parent workspace's `src/` (e.g. symlink `src/arm` → repo,
> `src/arm_gazebo` → `gazebo/`), then `colcon build`. See `AGENT.md §6`.

## ROS interfaces (`arm`)

All topics/services/actions are namespaced under `arm/` in node code, but the
canonical **ROS-level** names (used by clients in other packages) are listed
below.

* **`arm/set_pos`** *(action `arm/action/SetPosition`)*
  Goal is a `geometry_msgs/Twist`; the controller reads the target as
  `linear.x/y/z`, runs two-stage IK and publishes motor commands.

* **`arm/emergency_stop`** *(service `arm/srv/EmergencyStop`)*
  Emergency interrupt: `activate=true` cancels any in-flight `set_pos` goal,
  rejects new goals, zeroes the motor outputs and republishes them (and pauses
  the `get_pos` stream). `activate=false` releases the stop.

* **`arm/pos`** *(topic `arm/msg/ArmPosition`)*
  Current end-effector pose + motor angles. Published continuously only while
  streaming is enabled. The controller also publishes `arm/joints`
  (`sensor_msgs/JointState`) for the three joint angles.

* **`arm/get_pos`** *(service `arm/srv/TogglePositionStream`)*
  `enable=true` → start continuous publishing; `enable=false` → stop.

* **`arm/motor_targets`** *(topic `arm/msg/MotorTargets`, controller → driver)*
  The three joint-angle commands (degrees).

* **`arm/motor_feedback`** *(topic `arm/msg/ArmFeedback`, driver → monitor)*
  Periodic servo-bus feedback: measured joint angle per servo (`-1` if no
  response), plus an `online` flag and an `error` code per servo:
  `0` = OK, `1` = communication/timeout fault, `2` = offline.

* **`arm/motor_param_query`** *(service `arm/srv/MotorParamQuery`)*
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

## Docker (Windows WSL2) workflow

The recommended way to build/run on Windows is Docker Desktop with **WSL2
integration** enabled — the ROS Humble container runs inside the WSL2
`Ubuntu-22.04` distro, and the USB serial adapter is bridged into that distro
with `usbipd`.

### 1. One-time serial setup (WSL2 USB→container bridge)

The FashionStar bus-servo adapter is a USB UART (CH340 → `COMx` on Windows).
To use it from the container you must share the USB device into WSL2:

```powershell
# Windows PowerShell (admin):
usbipd bind    --hardware-id 1a86:7523 --force     # make COMx shareable
usbipd attach  --wsl Ubuntu-22.04 --busid 3-9 --auto-attach
```

After attach, the device appears in the WSL2 distro:

```bash
ls -l /dev/ttyUSB0    # crw-rw---- root dialout ...  → CH340 (1a86:7523)
```

(Find your bus id with `usbipd list`.) Re-attach after reboot or replug with
the same `attach` command.

### 2. Build the container

```bash
cd C:\Users\admin\Programs\Fyp\3DOF_DeltaArm
git submodule update --init --recursive    # FashionStar SDK
docker build -t arm:latest .
```

### 3. Build the package (colcon) inside the container

```bash
# From the repo root (PowerShell), the workspace is mounted at /home/rosuser/ws:
docker run --rm -it `
  -v "${PWD}:/home/rosuser/ws" -w /home/rosuser/ws `
  arm:latest bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install && source install/setup.bash"
```

> The repo root **is** the `arm` package, so the above builds only `arm`. To
> also build the separate `arm_gazebo` (Gazebo sim) package, stage `arm` and
> `gazebo/` as siblings under a parent `src/`:
>
> ```bash
> source /opt/ros/humble/setup.bash
> mkdir -p /tmp/ws/src
> ln -s <repo>        /tmp/ws/src/arm
> ln -s <repo>/gazebo /tmp/ws/src/arm_gazebo
> cd /tmp/ws && colcon build --symlink-install
> ```
>
> `arm_gazebo` depends on `arm`, so colcon builds `arm` first and `arm_gazebo`
> links the exported `arm::arm_kinematics` library. (See `AGENT.md §6`.)

## Launch options — quick reference menu

Everything below runs **inside the container** after
`source install/setup.bash` (see the Docker workflow above). The container
must be started with the serial device when hardware is attached:

```bash
# Hardware container (pass the serial port + dialout group):
docker run --rm -it --device /dev/ttyUSB0 --group-add 20 `
  -v "${PWD}:/home/rosuser/ws" -w /home/rosuser/ws arm:latest bash
```

| Purpose                                | Command                                                        |
|----------------------------------------|----------------------------------------------------------------|
| **Motor driver alone** (bus-servo HW)  | `ros2 run arm arm_motor_driver --ros-args --params-file config/arm_params.yaml` |
| **Controller only** (sim/external drv) | `ros2 launch arm delta_arm.launch simulation:=true`            |
| **Full system** (controller+driver HW) | `ros2 launch arm delta_arm.launch`                             |
| **Manual WASD jog** (sim)              | `ros2 launch arm manual.launch`                                |
| **Manual WASD jog** (hardware)         | `ros2 launch arm manual.launch simulation:=false`              |
| **Unit tests**                         | `ros2 launch arm test.launch.py`                               |

### Monitoring the motor bus (feedback)

In a second terminal/container run:

```bash
# Stream the periodic servo feedback (angle/online/error):
ros2 topic echo arm/motor_feedback

# One-shot read of servo 0 joint angle (request_type 0):
ros2 service call arm/motor_param_query arm/srv/MotorParamQuery \
    "{servo_id: 0, request_type: 0}"
```

> With the board connected but **no motors attached**, the driver starts, opens
> the port, and the feedback shows `servo_angle_current: [-1, -1, -1]`,
> `servo_online: [0,0,0]`, `servo_error: [2,2,2]` — i.e. the port works and the
> driver is healthy; the servos are simply offline.

### Real hardware

```bash
ros2 launch arm delta_arm.launch                      # controller + driver (one process)
ros2 launch arm delta_arm.launch simulation:=false    # force hardware
```

### Simulation (no motor driver)

```bash
ros2 launch arm delta_arm.launch simulation:=true
```

This starts **only the controller node** (`arm_controller`); the motor
driver is **not** run — the arm simulation is provided externally over the
same topic/service/action names (`arm/set_pos`, `arm/motor_targets`, `arm/pos`,
`arm/get_pos`). Because the interface is identical, the controller code needs no
changes between simulation and hardware.

### Gazebo simulation (`arm_gazebo`)

The optional 3D Gazebo sim is a separate package (`.ai` `/gazebo` =
`arm_gazebo`, see `AGENT.md §6`). It provides the simulation-side model that
fills in for the motor driver when the controller runs in `simulation:=true`.
Launch it (after building as a sibling) with:

```bash
ros2 launch arm_gazebo arm_gazebo.launch
```

The sim node subscribes `arm/target_position` and publishes
`arm/current_position`, and re-uses the delta-arm IK via the exported
`arm::arm_kinematics` library.

### Example: drive the end-effector

```bash
# move to (0.15, 0.0, 0.20) via the set_pos action
ros2 action send_goal arm/set_pos arm/action/SetPosition \
    "{target: {linear: {x: 0.15, y: 0.0, z: 0.20}}}"

# start stream, then watch the pose
ros2 service call arm/get_pos arm/srv/TogglePositionStream "{enable: true}"
ros2 topic echo arm/pos

# generic hardware query: joint angle of servo 0 (request_type 0)
ros2 service call arm/motor_param_query arm/srv/MotorParamQuery \
    "{servo_id: 0, request_type: 0}"

# emergency interrupt: halt everything and zero the motor output
ros2 service call arm/emergency_stop arm/srv/EmergencyStop \
    "{activate: true}"
# release the stop (set_pos accepted again)
ros2 service call arm/emergency_stop arm/srv/EmergencyStop \
    "{activate: false}"
```

### Run the tests

```bash
ros2 launch arm test.launch.py
# or, equivalently via colcon:
colcon test --packages-select arm
```

### Manual control (WASD)

```bash
ros2 launch arm manual.launch                  # simulation (default)
ros2 launch arm manual.launch simulation:=false   # real hardware
```

This starts the controller (plus the manual-control node) and lets you jog the
arm. `arm_manual` publishes `set_pos` action goals from the keyboard and
prints current-position changes as they come in on the `get_pos` stream:

```
W/S : +/- z    A/D : +/- y     Q : quit
```

The step size and the two axes are configurable:

```bash
ros2 run arm arm_manual --ros-args \
    -p step:=0.005 -p axis0:=x -p axis1:=z
```

> Note: the WASD axes default to `z` (W/S) and `y` (A/D) in a plane; choose
> `axis0`/`axis1` from `x`, `y`, `z` to jog a different plane.

## Where to implement the math

The inverse kinematics lives in `src/axis/delta-arm.cpp`:

* `Arm::ik_stage1(const Vec3 &)` — task-space target → per-limb plane orientation.
* `Arm::ik_stage2(float theta, int leg)` — limb plane orientation → motor angle.
* `Arm::apply()` — forward-kinematics estimate of `cur_pos_`.

## References

* [3DOF Delta Arm](https://people.ohio.edu/williams/html/PDF/DeltaKin.pdf)
* [FashionStart UART Servo C++ Driver](https://github.com/servodevelop/fashionstar-uart-servo-cpp.git)
