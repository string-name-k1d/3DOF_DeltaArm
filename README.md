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

> **Platform labels.** `[Win/WSL2]` = applies only on a **Windows** host
> (Docker Desktop + WSL2, `usbipd` USB bridging); `[Linux]` = only on a
> **native Linux** host; unlabelled = both.

- **Build toolchain**: ROS 2 Humble. Either build natively on Ubuntu 22.04 `[Linux]` or —
  on Windows — build inside the packaged Docker container `[Win/WSL2]` (see [2. Setup](#2-setup)).
- **FashionStar SDK submodule**: checked out recursively (see [2. Setup](#2-setup)).
- **Hardware** (only for the motor driver): the FashionStar bus-servo adapter is a
  USB UART (CH340 → `COMx` on Windows `[Win/WSL2]`). See [5. Launch](#5-launch) → *Real
  hardware* and *Docker (Windows WSL2) workflow* `[Win/WSL2]` for the USB→container bridge.

## 2. Setup

### Workspace layout

```
3DOF_DeltaArm/                     <-- Git Repository Root (= the `arm` package)
├── README.md
├── AGENT.md                          # Guide for AI agents (build/test/conventions)
├── .gitignore
├── .gitmodules                    # FashionStart UART servo SDK submodule
├── .ai/                             # Agent docs: ARCHITECTURE.md, CHANGELOGS.md
├── docs/                            # KINEMATICS.md (IK + 4-bar derivations)
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
├── compose.yaml                     # docker compose (root container, TTY, HW device binding)
├── gazebo/                          # SEPARATE package `arm_gazebo` (Gazebo sim)
│   ├── package.xml
│   ├── CMakeLists.txt
│   ├── src/                         # arm_cmd_bridge (deg targets -> shoulder cmd_pos)
│   ├── launch/                      # arm_gazebo.launch.py (world + create + bridge + nodes)
│   ├── urdf/                        # delta_arm.urdf.xacro (+ .gazebo.xacro plugins)
│   ├── worlds/                      # arm_world.sdf (DART/pgs)
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

### Build the container (Docker, Windows WSL2) `[Win/WSL2]`

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
> TTY) plus the hardware device binding, so no ad-hoc CLI flags are needed.
> Which part runs (real hardware vs simulation) is chosen **at launch with an
> entrypoint argument**, not with a separate compose file — see [5. Launch](#5-launch).

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
| `port_name` | `/dev/ttyUSB0` | UART/serial port of the bus-servo adapter. The WSL number is not fixed: the CH340 enumerates as `COM4`, `COM8`, ... depending on what else is plugged in — check with `usbipd list` on Windows, not by hard-coding. |
| `baudrate` | `115200` | Serial baud rate. |
| `servo_ids` | `[0, 1, 2]` | Binary ids of the three delta-arm servos. |
| `start_angles` | `[0, 0, 0]` | **Installation offset** (deg) per motor: the physical servo angle when the joint sits at the arm's zero/home pose. A commanded joint angle is sent as `joint + start_angles`, so `start_angles` **is** the per-motor installation offset of the servos. |
| `angle_min` | *(unset)* | Physical position limits (deg), **0 = limb fully extended** (away from centre). Left unset in `arm_params.yaml` so it **inherits `geometry.angle_min`** — one source of truth for the servo's travel. Set it here only for hardware whose travel differs from the modelled geometry (this is how `arm_gen0_params.yaml` overrides it). |
| `angle_max` | *(unset)* | Physical position limits (deg), **145 = most retracted**. Same inheritance as `angle_min`; the controller reads the shared `geometry.angle_max` when this is absent, so the pose it accepts and the command the driver clamps to always agree. |
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

Key mapping: `W`/`S` jog target `y`, `A`/`D` jog `x`, `Z`/`X` jog `z`.

## 4. ROS Components

### Nodes

| Node                | Exec / executable      | Responsibility                                                          |
|---------------------|------------------------|-------------------------------------------------------------------------|
| Controller          | `arm_controller`    | Task-space `set_pos` action + `get_pos` streaming (two-stage IK).       |
| Motor driver        | `arm_motor_driver`  | Bridges ROS messages to the FashionStar bus-servo hardware (`fsuartservo`). |
| Combined (HW)       | `arm_system`        | Controller + Motor driver, one process (default for real hardware).     |
| Manual control      | `arm_manual`        | Keyboard (WASD + Z/X) jogging of `set_pos`; prints `get_pos` position changes. |
| 2-D visualiser      | `arm_sim_sfml`       | SFML top/side view of the arm. Draws the `arm/pos` stream; when the real driver's `arm/motor_feedback` is fresh and all servos online, it draws the **measured** servos instead (real-machine mirror). Also marks the controller's commanded target (`arm/pos.target_position`) as `TGT`. |

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
  Current end-effector pose (`position`), the controller's commanded
  task-space target (`target_position`) and the current/target motor angles.
  Published continuously only while streaming is enabled. The controller also publishes `arm/joints`
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
with the compose defaults; the part to run is sent as the **first argument to
the `run_arm.sh` entrypoint** (`simulation` for the controller-only sim, no
argument for real hardware, `bash` for a shell):

```bash
# Real hardware (default) - controller + motor driver; USB serial and dialout
# group are already configured in compose.yaml:
docker compose run --rm arm

# Simulation - controller only (arm sim provided externally):
docker compose run --rm arm simulation

# Interactive shell / build inside the repo-mounted workspace:
docker compose run --rm arm bash
```

> On a host without the serial adapter attached, `compose.yaml` cannot bind
> `/dev/ttyUSB0` — point the bind somewhere harmless for shells/sims:
> `SERIAL_DEVICE=/dev/null docker compose run --rm arm simulation` (or use the
> wrapped workflow in the parent workspace, which hot-plugs the device).
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
# Motion is disabled by loading config/arm_feedback_only.yaml into both the
# controller and the driver; see "Feedback-only mode" below.
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
#   feedback-only VISUALISATION: driver runs enable_motion=false and the
#   window draws the measured servo angles. No controller node is started.
```

#### Feedback-only mode: what is (and is not) guaranteed

`control:=false` is the safe way to look at real hardware. In that mode the
launch starts **no controller**, and it loads `config/arm_feedback_only.yaml`
so the driver runs with `enable_motion=false`. The resulting guarantees:

| Property | In `control:=false` |
| --- | --- |
| `arm_controller` node | not started |
| `set_pos` action **server** | none (the 2-D window is only an action *client*) |
| `arm/motor_targets` subscription | **not created** — the topic has zero endpoints |
| Servo commands reachable | **none** |
| `arm/motor_feedback` published | yes |
| `arm/motor_param_query` served | yes |

Because the driver never subscribes to `arm/motor_targets`, no `MotorTargets`
message can reach the bus servos even if some other node published one.

> **Why this uses a params file, not an inline launch dict.** Passing
> `parameters=[params_file, {'enable_motion': False}]` looks correct but is
> **silently unsafe**: rcl gives an exact `arm_motor_driver:` block inside a
> params file precedence over a `/**:` wildcard override, so the driver's
> `enable_motion: true` from `arm_params.yaml` wins and the driver *does* start
> its `arm/motor_targets` subscription. `config/arm_feedback_only.yaml` is
> passed **after** `params_file` and is node-specific, so it wins. The same trap
> applies to `<param name="enable_motion" value="false"/>` in XML launch files,
> so `delta_arm.launch` also loads this file.

**Always confirm the guarantee on the running system before trusting it with
hardware** (duplicate node names from a previous run can make you query the
wrong process, so check the process count too):

```bash
# exactly one driver instance
ps -eo args | grep "install/arm/lib/arm/arm_motor_driver" | grep -vc defunct

# must print: Boolean value is: False
ros2 param get /arm_motor_driver enable_motion

# must print: motion control DISABLED (enable_motion=false) ...
grep "motion control DISABLED" <launch-log>

# must show no endpoints at all
ros2 topic info /arm/motor_targets

# must show no arm_controller
ros2 node list | grep arm
```

If `enable_motion` reports `True` while you passed `control:=false`, stop —
do not attach the arm. See
[Troubleshooting](#8-troubleshooting).

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

### One-time serial setup (WSL2 USB→container bridge) `[Win/WSL2]`

The bus-servo adapter is a **CH340** (`1a86:7523`). Bring it into WSL2 with
`usbipd-win`, which needs **Administrator** — it cannot be done from an
unelevated shell.

```powershell
# Windows PowerShell (Administrator):
usbipd list                                # find the CH340 and note its BUSID
usbipd bind    --busid 3-9                 # only needed once per device
usbipd attach  --wsl --busid 3-9
```

> `bind` makes the device shareable and `attach` attaches it to WSL. Both
> report "Shared"/"Not shared" in `usbipd list`; the CH340 shows as
> `USB-SERIAL CH340 (COM4)`. The COM number is assigned by Windows and is
> **not stable** — it is unrelated to the WSL device name, which is always
> `/dev/ttyUSB0`.

After attach, the device appears in the WSL2 distro **and** inside the
container (the service is `privileged: true` and bind-mounts `/dev`):

```bash
ls -l /dev/ttyUSB0   # crw-rw---- root dialout ...  → CH340 (1a86:7523)
```

Re-attach after a reboot or replug with the same `attach` command. Detach with
`usbipd detach --busid 3-9`.

**usbipd-win 5.x** uses the syntax above. Version 4 used
`usbipd attach --wsl <distro> --busid <id> --auto-attach`; check
`usbipd --version` if a command is rejected.

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

### Manual control (WASD + Z/X)

```bash
ros2 launch arm manual.launch                  # simulation (default)
ros2 launch arm manual.launch simulation:=false   # real hardware
```

This starts the controller (plus the manual-control node) and lets you jog the
arm. `arm_manual` publishes `set_pos` action goals from the keyboard and
prints current-position changes as they come in on the `get_pos` stream:

```
W/S : +/- y    A/D : -/+ x    Z/X : +/- z    Q : quit
```

`WASD` drives the horizontal (XY) plane and `Z/X` the vertical axis, so the
whole reachable workspace can be jogged from one key set.

The step size is configurable:

```bash
ros2 run arm arm_manual --ros-args -p step:=0.005
```

## Component: Gazebo Simulation (`arm_gazebo`)

The optional 3-D Gazebo sim is a **separate package** under `gazebo/` (see
`AGENT.md §6`). It provides the simulation-side model that fills in for the
motor driver when the controller runs in `simulation:=true`.

It targets **Gazebo Harmonic (gz-sim 8)**: the delta arm is a URDF **open
kinematic tree** (`urdf/delta_arm.urdf.xacro` + `delta_arm.gazebo.xacro`)
whose three leg loops are closed at runtime by `DetachableJoint` plugin welds
(tip ↔ dummy). Each shoulder is servoed by a `JointPositionController`
(subscribes `/delta_arm/shoulder_<i>/cmd_pos`, radians); the `arm_cmd_bridge`
node feeds it from the controller's `arm/motor_targets` (degrees). Physics is
DART `pgs` in `worlds/arm_world.sdf`; joint state is bridged over
`parameter_bridge` to ROS `/joint_states`.

Navigation/notes:

```bash
docker exec fyp_sim bash /root/install/03-gz-harmonic.sh                    # parent fyp workspace (shared container)
docker exec fyp_sim bash /root/ros2_ws/src/arm/scripts/install_gazebo_harmonic.sh   # ...or this repo's own script
ros2 launch arm_gazebo arm_gazebo.launch.py                     # headless
ros2 launch arm_gazebo arm_gazebo.launch.py gui:=true           # + Gazebo GUI
ros2 launch arm_gazebo arm_gazebo.launch.py manual:=true        # + WASD/Z-X terminal
```

`manual:=true` also starts the `arm_manual` WASD/Z-X node, so you can jog the
simulated end-effector from the launching terminal:

```
W/S : +/- y    A/D : -/+ x    Z/X : +/- z    Q : quit
```

The controller can also run standalone with the sim, or be replaced by the
keyboard jog entirely:

```bash
ros2 launch arm delta_arm.launch simulation:=true   # controller only
ros2 launch arm manual.launch                       # controller + WASD (no sim)
```

## 7. Inverse Kinematics

The inverse kinematics lives in `src/axis/delta-arm.cpp` (pipe:

* `Arm::ik_stage1(const Vec3 &)` — task-space target → per-limb plane orientation
  (the classic delta IK of R. L. Williams II, `delta_inverse_kinematics_arm`).
* `Arm::ik_stage2(float theta, int leg)` — limb plane orientation → motor angle
  through the **4-bar servo linkage** (`motor_from_arm`).
* `Arm::apply()` — forward-kinematics estimate of `cur_pos_`, backed by
  `arm_from_motor` (bisection) + classic delta FK.

### The reachable envelope

The arm can only be in a small region of task space, and both directions of the
solve are driven from **one** geometry-derived band so they can never disagree.
`Arm::compute_linkage_bands()` resolves it once per `set_geometry()` as the
intersection of four constraints:

1. the four-bar linkage can physically close;
2. the motor angle is non-decreasing in the arm angle — past the linkage's
   transmission-angle fold the map folds back on itself and no single-valued
   inverse exists, so that region is unusable;
3. the commanded motor angle is inside the servo's mechanical travel
   (`geometry.angle_min` / `geometry.angle_max`);
4. the arm angle is inside `geometry.arm_angle_min` / `geometry.arm_angle_max`.

The band is logged at startup, per leg:

```
reachable arm band, leg 0: [31.81, 99.69] deg (servo travel [0.0, 145.0] deg)
```

| Assembly | Resolved band | Upper edge set by |
|---|---|---|
| default (`arm_params.yaml`) | ~`[31.8°, 99.7°]` | the 145° servo stop |
| gen0 (`arm_gen0_params.yaml`) | ~`[0.05°, 101.1°]` | the linkage itself |

The top of the band is **not** 90°. The arm sweeps past vertical, and the servo
stop is what finally bounds it (on the default assembly, relaxing the stop to
200° opens the band to ~127°). Lowering the top edge is how you buy workspace
once the real servo travel is measured — roughly 3.4° of arm per 5° of servo.

**Unreachable goals are refused, not silently adjusted.** `set_tar_pos()` returns
a `TargetResult{reached, reason}`; on failure it commits *nothing*, so the arm
holds position and the streamed target marker stays where the arm actually is.
The action server aborts with the offending limb named:

```
target outside the reachable envelope: leg 1 needs arm angle 120.815 deg,
outside the reachable band [31.81, 99.69] deg
```

This matters because the old behaviour was invisible: the forward kinematics used
to clamp any arm angle to 90°, so poses needing more were *accepted* with
perfectly plausible motor angles while the arm sat somewhere else entirely — up to
64 mm from the command on the default assembly, affecting ~37% of all accepted
goals. A wrong-looking motor angle is a far better failure than a silent
disagreement between the command and the arm.

`arm_angle_from_motor_deg()` is total by construction (it bisects inside the band
and clamps at its edges), so feedback from a servo that is not currently inside
the resolved envelope can never throw on the control path.

### Notes on the 3-D model

`gazebo/urdf/delta_arm.urdf.xacro` drives the shoulder joint **directly** and does
not model the four-bar, so its joint angle is the arm angle rather than the servo
angle. Its joint limits are deliberately permissive (`shoulder_lower`/
`shoulder_upper`, `-0.10`/`3.20` rad): the controller is the single authority on
reachability, and a tighter limit in the URDF would clamp poses the controller had
already accepted — reintroducing the same silent disagreement in 3-D.

### Integration checks (need the sims running)

The unit tests cover the library. These two cover the whole ROS stack — that what
the 2-D simulator *draws* is what was *commanded* — and are run by hand:

```bash
# 2-D: controller + arm_sim_sfml up, then sweep the whole reachable workspace
python3 arm/test/e2e_sweep.py          # expects worst error 0.000 mm, 3 mm tolerance

# 3-D: ros2 launch arm_gazebo arm_gazebo.launch.py params_file:=... up, then
#      <band_hi_deg> <label> <reachable x y z> <refused x y z>
python3 arm/test/gz_joint_check.py 99.69 default -0.12 -0.07 -0.25 -0.12 -0.10 -0.28
python3 arm/test/gz_joint_check.py 101.13 gen0    0.0 0.0 -0.32  0.0 0.0 -0.14
```

`e2e_sweep.py` also checks that a refused goal neither moves the arm nor advances
the target marker, and that no motor is ever commanded past its stop.

See **[`docs/KINEMATICS.md`](docs/KINEMATICS.md)** for the full derivation (the
Williams stage-1 `A/B/C/D` solve and its half-angle quadratic, the exact 4-bar
circle-circle intersection and closure test, the band resolution, the bisection
inverse, FK, and worked numbers for both shipped assemblies).

### Known geometry gap: the gen0 servo linkage is still uncalibrated

**The `gen0` horn/rod lengths are not yet confirmed against the real arm, so
absolute servo angles and gen0 FK should be treated as approximate.**

With the exact 4-bar solve in place, the modelled linkage closes across nearly the
whole servo travel on both assemblies:

| | servo travel where the 4-bar closes | driver nominal travel |
| --- | --- | --- |
| `gen0` | **15.9° – 144.9°** | 0° – 145° |
| `default` | **20.2° – 133.1°** | 0° – 145° |

> An earlier version of this file reported gen0 closing over only 15.1° – 93.0°,
> leaving most of the travel unusable. That span was an artefact of the old 4-bar
> closed form, which did not describe a rigid linkage (see
> [docs/KINEMATICS.md](docs/KINEMATICS.md) §3.2). The renderer still draws a limb
> whose linkage cannot close in **orange**, rings its servo shaft, and prints
> `ROD CANNOT REACH (orange)` with each affected limb's raw `θ`, so genuine
> out-of-closure feedback stays visible instead of being silently clamped.

**Calibration datum** (gen0, all three servos commanded to 90°):

| | value |
| --- | --- |
| measured | `z ≈ -300 mm` |
| model predicts | `z ≈ -280 mm` (arm φ 43.8°) |

So the model now lands within ~20 mm of the datum. The remaining error is the
**servo linkage**: horn length, rod length, `arm_attach_dist`, and the horn's
mounting zero (`home_offset`, still `0`). Until those are measured on the real
arm, do not use gen0 absolute FK as a position reference.

For scale, the two hardware anchors and what the model says for them:

| anchor | model |
| --- | --- |
| servo 30° ↔ arm 0° | arm ≈ 8.0° |
| servo 145° ↔ arm 80° | arm ≈ 78.1° (the band's top edge) |

## 8. Troubleshooting

### `enable_motion` is `True` even though I passed `control:=false`

This is the dangerous one, because the driver *is* subscribed to
`arm/motor_targets` and a stray `MotorTargets` message would be sent to the
servos. Do not connect the arm until it reads `False`.

It is usually **not** a build problem — it is a parameter-precedence trap plus
stale ROS state. Check in this order:

1. **A stale node is answering the query.** Duplicate `arm_motor_driver`
   instances from an earlier launch will make `ros2 param get` report the wrong
   process (a `drv_min`-style uniquely-named probe also gives a misleading
   result, because it does not match the `arm_motor_driver:` block in
   `arm_params.yaml`). Verify with
   `ps -eo args | grep "install/arm/lib/arm/arm_motor_driver" | grep -vc defunct`,
   kill leftovers, then `ros2 daemon stop`.
2. **An inline override was used instead of a params file.** Only
   `config/arm_feedback_only.yaml` disables motion reliably. Re-add it to
   `parameters=[...]` *after* `params_file`, or rebuild if it is missing from
   `install/arm/share/arm/config/`.
3. **Confirm the driver logged the decision** — `motion control DISABLED
   (enable_motion=false) - arm/motor_targets subscription not started`.

### Serial device missing

```
serial device /dev/ttyUSB0 not present/accessible - continuing in
feedback-only mode with all servos reported offline.
```

Expected when the bus-servo adapter is not attached, and harmless in
`control:=false` mode. See "One-time serial setup" in
[Real hardware](#real-hardware) above.

## 9. References

* R. L. **Williams II**, *The Delta Parallel Robot: Kinematics Solutions*,
  Ohio University — <https://people.ohio.edu/williams/html/PDF/DeltaKin.pdf>.
  This is the source of the stage-1 inverse kinematics
  (`delta_inverse_kinematics_arm()`); its `A`/`B`/`C`/`D` link notation and its
  per-limb "pick the root with the smaller |θ|" assembly-mode heuristic are kept
  verbatim in the code. Derivation and notation table in
  [docs/KINEMATICS.md](docs/KINEMATICS.md) §2.
* [FashionStart UART Servo C++ Driver](https://github.com/servodevelop/fashionstar-uart-servo-cpp.git)
