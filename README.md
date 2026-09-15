# 3DOF Delta Arm (ROS 2 package)

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
workspace/package).

## Content Overview

| Table of Contents |  |
| :--- | :--- |
| [1. Prerequisites](#1-prerequisites) | Prerequisites (serial / tooling) |
| [2. Setup](#2-setup) | Workspace layout, submodule, build |
| [3. Configurations](#3-configurations) | Parameter files & motor limits |
| [4. ROS Components](#4-ros-components) | Nodes & ROS interfaces |
| [5. Launch](#5-launch) | Launch commands (HW / sim / tests) |
| [6. Controls](#6-controls) | Jogging (WASD), actions, driving poses |
| [Component: Gazebo Simulation](#component-gazebo-simulation-arm_gazebo) | Gazebo standalone sim |
| [7. Inverse Kinematics](#7-inverse-kinematics) | Where to implement the IK math |
| [8. References](#8-references) | External references |

## 1. Prerequisites

- **Build toolchain**: ROS 2 Humble. Either build natively on Ubuntu 22.04 or —
  on Windows — build inside the packaged Docker container (see [2. Setup](#2-setup)).
- **FashionStar SDK submodule**: checked out recursively (see [2. Setup](#2-setup)).
- **Hardware** (only for the motor driver): the FashionStar bus-servo adapter is a
  USB UART (CH340 → `COMx` on Windows). See [5. Launch](#5-launch) → *Real
  hardware* and *Docker (Windows WSL2) workflow* for the USB→container bridge.

## 2. Setup

### Workspace layout

```
3DOF_DeltaArm/                     <-- Git Repository Root (= the `arm` package)
├── README.md
├── AGENT.md                          # Guide for AI agents (build/test/conventions)
├── .gitignore
├── .gitmodules                    # FashionStart UART servo SDK submodule
├── .ai/                             # Agent docs: ARCHITECTURE.md, CHANGELOGS.md
├── action/                          # SetPosition.action  (set_pos)
├── msg/                             # ArmPosition.msg, ArmFeedback.msg, MotorTargets.msg
├── srv/                             # TogglePositionStream.srv, MotorParamQuery.srv, EmergencyStop.srv
├── include/arm/                     # C++ headers (delta_arm, delta_arm_controller,
│                                    #   delta_arm_manual, motor_driver, motor_driver_node, arm_sim_sfml)
├── src/{axis,controller,driver,manual,sim,system}/   # node sources + kinematics
├── launch/                          # delta_arm.launch, manual.launch, test.launch.py
├── config/                          # arm_params.yaml (per-node parameters)
├── test/                            # GTest unit tests (test_delta_arm.cpp)
├── motor_driver/                    # FashionStart C++ SDK (git submodule) -> fsuartservo + cserialport
├── run_arm.sh                       # container launcher (HW default; "simulation" arg; dev shell)
├── compose.yaml                     # docker compose defaults (root container, HW config, TTY)
├── compose.sim.yaml                 # simulation override (clears HW bits, runs controller only)
├── gazebo/                          # SEPARATE package `arm_gazebo` (Gazebo sim)
│   ├── package.xml
│   ├── CMakeLists.txt
│   ├── src/                         # delta_arm_gazebo_plugin (model plugin)
│   ├── launch/                      # arm_gazebo.launch.py (sim + optional WASD)
│   ├── worlds/                      # delta_arm.world
│   └── config/
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
>
> A stale, empty `src/arm_hardware/` folder may linger from the pre-flatten
> layout; it is no longer built (all sources live at `src/` directly). Keep it
> out of git (it is untracked).

### Pulling in the submodules

```bash
git submodule update --init --recursive    # FashionStart SDK
```

### Build the container (Docker, Windows WSL2)

The recommended way to build/run on Windows is Docker Desktop with **WSL2
integration** enabled — the ROS Humble container runs inside the WSL2
`Ubuntu-22.04` distro, and the USB serial adapter is bridged into that distro
with `usbipd`.

```bash
cd C:\Users\admin\Programs\Fyp\3DOF_DeltaArm
git submodule update --init --recursive    # FashionStart SDK
docker build -t arm:latest .
# or, using the compose defaults in compose.yaml:
docker compose build arm
```

> **`compose.yaml`** bakes in the always-needed `docker run` arguments (the
> container runs as **root**, the repo is mounted at `/root/ws`, interactive
> TTY) plus the hardware configuration (**USB serial binding** and **dialout
> group**), so no ad-hoc CLI flags are needed. The default launch is the **real
> hardware** arm; the pure-simulation launch requires the extra
> `-f compose.sim.yaml` override file (see [5. Launch](#5-launch)).

### Build the package (colcon) inside the container

```bash
# From the repo root (PowerShell), the workspace is mounted at /root/ws:
docker compose run --rm arm bash -lc \
  "source /opt/ros/humble/setup.bash && colcon build --symlink-install && source install/setup.bash"
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

## 3. Configurations

All node parameters are read from `config/arm_params.yaml`, namespaced per node
(`arm_controller`, `arm_motor_driver`, `arm_manual_control`). Launch files point
at this file automatically; a standalone node can be pointed at it with
`--ros-args --params-file config/arm_params.yaml`.

### Controller (`arm_controller`)

| Parameter | Default | Meaning |
|---|---|---|
| `feedback_rate` | `20.0` | Continuous `get_pos` stream rate on topic `arm/pos` (Hz). |
| `feedback_frame` | `base_link` | Coordinate frame attached to published `ArmPosition` messages. |
| `enable_motion` | `true` | Disable to skip creating the `set_pos` action server — the controller can never produce `arm/motor_targets` (feedback-only runs). |

### Motor driver (`arm_motor_driver`)

The FashionStar UART-servos are angle servos; their "angle" is the raw
output-shaft position in degrees. The control outputs are mapped onto the
arm's **joint-angle convention** and bounded before they reach the servo:

| Parameter | Default | Meaning |
|---|---|---|
| `port_name` | `/dev/ttyUSB0` | UART/serial port of the bus-servo adapter (Windows: `COM8`). |
| `baudrate` | `115200` | Serial baud rate. |
| `servo_ids` | `[0, 1, 2]` | Binary ids of the three delta-arm servos. |
| `start_angles` | `[0, 0, 0]` | **Installation offset** (deg) per motor: the physical servo angle when the joint sits at the arm's zero/home pose. A commanded joint angle is sent as `joint + start_angles`, so `start_angles` **is** the per-motor installation offset of the servos. |
| `angle_min` | `[0, 0, 0]` | Physical position limits (deg), **0 = limb fully extended** (away from centre). |
| `angle_max` | `[145, 145, 145]` | Physical position limits (deg), **145 = most retracted**. |
| `max_speed` | `100.0` | Velocity limit (deg/s) used by the servo's trajectory profiling (move interval = dAngle/speed). |
| `auto_init` | `true` | Ping + sync servos at startup. |
| `enable_motion` | `true` | Disable to skip the `arm/motor_targets` subscription entirely — motors can never be commanded (feedback-only runs). Ping/telemetry still work. |
| `feedback_rate` | `10.0` | Periodic servo-bus feedback rate on `arm/motor_feedback` (Hz). |
| `feedback_frame` | `base_link` | Frame ID of the feedback messages. |

The driver clamps every command into `[angle_min, angle_max]` before sending it,
and reports measured angles back in the **joint frame** (`servo_angle −
start_angles`), so the feedback is directly comparable to the commanded value.
The SDK-side position range (`setAngleRange`) and speed (`setSpeed`) are also
configured for trajectory profiling on every servo at construction.

> These are SDK-side limits — the physical **position/velocity limits** are
> enforced in software by the driver on every command, not by a fixed hardware
> register. `angle_min`/`angle_max`/`start_angles`/`max_speed` are **per-motor**
> arrays (one entry per servo).

### Manual control (`arm_manual_control`)

| Parameter | Default | Meaning |
|---|---|---|
| `step` | `0.01` | Position step per keypress (m). |
| `axis0` | `z` | WASD primary axis (`x`, `y`, or `z`). |
| `axis1` | `y` | WASD secondary axis. |

## 4. ROS Components

### Nodes

| Node                | Exec / executable      | Responsibility                                                          |
|---------------------|------------------------|-------------------------------------------------------------------------|
| Controller          | `arm_controller`    | Task-space `set_pos` action + `get_pos` streaming (two-stage IK).       |
| Motor driver        | `arm_motor_driver`  | Bridges ROS messages to the FashionStar bus-servo hardware (`fsuartservo`). |
| Combined (HW)       | `arm_system`        | Controller + Motor driver, one process (default for real hardware).     |
| Manual control      | `arm_manual`        | Keyboard (WASD) jogging of `set_pos`; prints `get_pos` position changes. |
| 2-D visualiser      | `arm_sim_sfml`       | SFML top/side view of the arm. Draws the `arm/pos` stream; when the real driver's `arm/motor_feedback` is fresh and all servos online, it draws the **measured** servos instead (real-machine mirror). |

All topics/services/actions are namespaced under `arm/` in node code, but the
canonical **ROS-level** names (used by clients in other packages) are listed
below.

### Interfaces

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

### Data flow

```
  action goal (Twist x/y/z) -> arm_controller -> ik_stage1/ik_stage2 -> motor_targets (deg)
                                     |                                          |
                                     v                                          v
      arm/pos (feedback) <- get_pos stream        arm_motor_driver -> FSUS servos (UART)
```

## 5. Launch

Everything below runs **inside the container** after
`source install/setup.bash` (see [2. Setup](#2-setup)). Start the container
with the compose defaults. The default launch is the **real hardware** arm;
simulation requires the extra `-f compose.sim.yaml` flag:

```bash
# Real hardware (default) - controller + motor driver; USB serial and dialout
# group are already configured in compose.yaml:
docker compose run --rm arm

# Simulation - controller only (arm sim provided externally), no serial config:
docker compose -f compose.yaml -f compose.sim.yaml run --rm arm

# Interactive shell / build inside the repo-mounted workspace:
docker compose run --rm arm bash
```

> On a host without the serial adapter attached, `compose.yaml` cannot bind
> `/dev/ttyUSB0` — use the simulation override for shells/builds too:
> `docker compose -f compose.yaml -f compose.sim.yaml run --rm arm bash`.
>
> The USB serial path is configurable without touching the docker files:
> `SERIAL_DEVICE=/dev/ttyUSB1 docker compose run --rm arm` (default
> `/dev/ttyUSB0`).

| Purpose                                | Command                                                        |
|----------------------------------------|----------------------------------------------------------------|
| **Motor driver alone** (bus-servo HW)  | `ros2 run arm arm_motor_driver --ros-args --params-file config/arm_params.yaml` |
| **Controller only** (sim/external drv) | `ros2 launch arm delta_arm.launch simulation:=true`            |
| **Full system** (controller+driver HW) | `ros2 launch arm delta_arm.launch`                             |
| **Feedback only** (HW, no motion)      | `ros2 launch arm delta_arm.launch motion:=false`               |
| **Manual WASD jog** (sim)              | `ros2 launch arm manual.launch`                                |
| **Manual WASD jog** (hardware)         | `ros2 launch arm manual.launch simulation:=false`              |
| **Unit tests**                         | `ros2 launch arm test.launch.py`                               |

### Real hardware

```bash
ros2 launch arm delta_arm.launch                      # controller + driver (one process)
ros2 launch arm delta_arm.launch simulation:=false    # force hardware

# Feedback-only: boot the full stack but disable all motion (servos will not
# move). The driver opens the port, pings the servos, and publishes the
# periodic arm/motor_feedback topic. Nothing is commanded.
ros2 launch arm delta_arm.launch motion:=false

# Then in a second terminal, watch the servo bus feedback:
ros2 topic echo arm/motor_feedback
# or do a one-shot query of servo 0's joint angle (request_type 0):
ros2 service call arm/motor_param_query arm/srv/MotorParamQuery \
    "{servo_id: 0, request_type: 0}"
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

### 2-D visualiser (`arm_sim_sfml`)

```bash
ros2 launch arm arm_sim_2d.launch.py            # controller (sim) + SFML window
# ...or together with the real hardware stack:
ros2 launch arm delta_arm.launch motion:=true   # real controller + motor driver
ros2 launch arm arm_sim_2d.launch.py            # SFML window only (see below)
ros2 launch arm arm_sim_2d.launch.py motor:=true   # driver + controller + window
ros2 launch arm arm_sim_2d.launch.py motor:=true control:=false
#   feedback-only VISUALISATION: driver runs enable_motion=false (no
#   arm/motor_targets subscription, nothing can move) and the window draws the
#   measured servo angles. No controller node is started.
```

The SFML window shows the **top view (XY)** and **side view (XZ)** of the arm
with two sources:

* **sim stream** — the controller's `arm/pos` endpoint (default);
* **live servo feedback** — while the real driver is publishing fresh
  `arm/motor_feedback` with all three servos **online**, the top-left status
  turns green ("SOURCE: real servo feedback") and the arm is redrawn from the
  **measured** servo angles via FK, so the window mirrors the physical machine
  servo-by-servo. It drops back to the sim stream if any servo is offline or
  feedback stops for more than 0.5 s.

For a real-hardware mirror run `motor:=true` (from `run_arm.sh --view 2d
--mode motor`; add `--control on`/`control:=true` for WASD to jog the physical
arm through the same `arm/set_pos` action the manual node uses).

### One-time serial setup (WSL2 USB→container bridge)

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

## 6. Controls

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

## Component: Gazebo Simulation (`arm_gazebo`)

The optional 3D Gazebo sim is a **separate package** under `gazebo/` (see
`AGENT.md §6`). It provides the simulation-side model that fills in for the
motor driver when the controller runs in `simulation:=true`.

The sim is a **model plugin** (`delta_arm_gazebo_plugin` in
`worlds/delta_arm.world`): a closed-loop delta-arm model whose shoulder joints
are servoed by a PD+I torque controller on the upper-arm links. It subscribes
the controller's `arm/motor_targets` (degrees) and is verified to track the
commanded angles to well under a degree. Because a Gazebo/ODE loop joint rejects
kinematic `SetPosition`, the plugin drives the mechanism with body torques
instead (`AddRelativeTorque`); gains are SDF-configurable (`kp`/`ki`/`kd`/
`tau_max`).

Launch it (after building `arm` and `arm_gazebo` as siblings) with:

```bash
ros2 launch arm_gazebo arm_gazebo.launch.py                     # headless
ros2 launch arm_gazebo arm_gazebo.launch.py gui:=true           # + Gzclient GUI
ros2 launch arm_gazebo arm_gazebo.launch.py manual:=true        # + WASD terminal
```

`manual:=true` also starts the `arm_manual` WASD node, so you can jog the
simulated end-effector from the launching terminal:

```
W/S : +/- z    A/D : +/- y     Q : quit
```

The controller can also run standalone with the sim, or be replaced by the
WASD jog entirely:

```bash
ros2 launch arm delta_arm.launch simulation:=true   # controller only
ros2 launch arm manual.launch                       # controller + WASD (no sim)
```

## 7. Inverse Kinematics

The inverse kinematics lives in `src/axis/delta-arm.cpp`:

* `Arm::ik_stage1(const Vec3 &)` — task-space target → per-limb plane orientation.
* `Arm::ik_stage2(float theta, int leg)` — limb plane orientation → motor angle.
* `Arm::apply()` — forward-kinematics estimate of `cur_pos_`.

## 8. References

* [3DOF Delta Arm](https://people.ohio.edu/williams/html/PDF/DeltaKin.pdf)
* [FashionStart UART Servo C++ Driver](https://github.com/servodevelop/fashionstar-uart-servo-cpp.git)