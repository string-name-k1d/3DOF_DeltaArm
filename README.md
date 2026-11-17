# 3DOF Delta Arm (ROS 2 package)

Minimal ROS 2 (C++) package for a 3-DOF delta arm driven by FashionStar UART
bus-servos.

See [`docs/KINEMATICS.md`](docs/KINEMATICS.md). The implementation lives in
`src/controller/delta-arm_dynamics.cpp` and is built as the exported
`arm_kinematics` shared library, linked by the controller, visualiser and tests.

## TOC

1. [Build (local ROS 2)](#build-local-ros-2)
2. [Docker (optional)](#docker-optional)
3. [Run](#run)
4. [Configs](#configuration)
5. [ROS Archi](#ros-interfaces)

## Build (local ROS 2)

Target environment: Ubuntu 22.04 + ROS 2 Humble installed **natively** (no
Docker). This repository is the `arm` package; in an integrated workspace it is
usually vendored under a parent package, for example:

```text
arm_ws/
└── src/
    └── fyp/
        └── arm/          <-- this repository
```

Run `colcon` from the workspace root (`arm_ws/`), not from this directory.

### Prerequisites

```bash
source /opt/ros/humble/setup.bash

sudo apt install -y \
    python3-colcon-common-extensions python3-rosdep \
    ros-humble-rclcpp-action ros-humble-ament-cmake-gtest

# once per machine
sudo rosdep init 2>/dev/null || true
rosdep update
```

```bash
# FashionStar UART Servo Driver (run inside this package; if the parent repo
# owns the submodule, use: git -C src/fyp/arm submodule update --init --recursive)
git submodule update --init --recursive
```

### Optional: 2-D SFML visualiser

`arm_sim_sfml` (the `sim_2d.launch.py` window) is optional and is **not** a
rosdep dependency.

```bash
sudo apt install libsfml-dev        # enables arm_sim_sfml
```

Without SFML the configure step prints
`arm: SFML not found - skipping the 2-D visualiser (arm_sim_sfml)` and the build
continues. To skip it explicitly even when SFML is present, set
`ARM_BUILD_SFML_SIM=OFF` in `.env` (see below), or pass the CMake flag directly:

```bash
colcon build --symlink-install --cmake-args -DARM_BUILD_SFML_SIM=OFF
```

### Environment overrides (`.env`)

Machine-specific defaults live in a gitignored `.env` at the package root.
Copy the template and edit it; load it in every shell that runs the scripts:

```bash
cp .env.example .env
source .env
```

`./scripts/setup.sh` refuses to run until `.env` exists.

Docker Compose also reads `.env` automatically for variable substitution.

| Key | Default | Used by |
| --- | --- | --- |
| `ROS_DISTRO` | `humble` | `setup.sh`, `run_arm.sh` |
| `ARM_WS` | auto-detected | `setup.sh` |
| `ARM_BUILD_SFML_SIM` | `ON` | `setup.sh` |
| `ARM_BUILD_PACKAGES` | `arm` | `setup.sh` |
| `ARM_INSTALL_GAZEBO` | `0` | `setup.sh` |
| `ARM_INSTALL_ROSBRIDGE` | `0` | `setup.sh` |
| `ARM_PARAMS_FILE` | `config/arm_params.yaml` | `run_arm.sh` |
| `ARM_TMUX_SESSION` | `arm` | `start_arm_tmux.sh` |
| `SERIAL_DEVICE` | `/dev/ttyUSB0` | Docker Compose / `launch_container.sh` |
| `DISPLAY`, `XAUTHORITY` | live session | override only |

Keep `DISPLAY` out of `.env` normally: Compose and native runs inherit the
live session value.

### Build the workspace

```bash
cp .env.example .env            # once, then edit
./scripts/setup.sh              # builds with .env
source .env
source ~/arm_ws/install/setup.bash
```

`scripts/setup.sh` can be run from anywhere, auto-detects the workspace root,
honours `ARM_BUILD_SFML_SIM`/`ARM_BUILD_PACKAGES`, installs the optional
add-ons selected with `ARM_INSTALL_*`, and forwards extra arguments to
`colcon`. Equivalent manual build:

```bash
colcon build --symlink-install --packages-select arm
source install/setup.bash
```

Add the optional 3-D Gazebo package when it is staged as a sibling of `arm`
(see [`AGENT.md`](AGENT.md)), either with
`ARM_BUILD_PACKAGES="arm arm_gazebo"` in `.env` or explicitly:

```bash
colcon build --symlink-install --packages-select arm arm_gazebo
```

After sourcing, `ros2 pkg prefix arm` should print `<arm_ws>/install/arm`.

## Docker (optional)

The bundled `compose.yaml` keeps the USB-serial adapter out of the
default `arm` service, so simulation and development work without hardware:

```bash
docker compose run --rm arm              # fake 2-D reference, control on
docker compose run --rm arm simulation   # fake hardware: controller only
docker compose run --rm arm sim2d        # 2-D SFML view, control on
docker compose run --rm arm sim3d        # 3-D Gazebo simulation
docker compose run --rm arm bash         # dev shell / build
docker compose run --rm arm-hardware     # real hardware + 2-D reference,
                                         # control on; binds /dev/ttyUSB0
```

Docker Compose reads `.env` from this directory automatically for variable
substitution (shell variables still win), so `DISPLAY`, `SERIAL_DEVICE` and
`XAUTHORITY` can be overridden there.

For a one-step launch that starts the stack and leaves you in a shell inside
the container:

```bash
./scripts/launch_container.sh          # real hardware + 2-D reference, control on
./scripts/launch_container.sh --sim    # fake 2-D simulation, control on
```

The helper also forwards the host X11 display when available, so the SFML
window works from the shell it opens. `arm_sim_sfml` checks the display
before creating the window:
on a headless host it reports the missing `DISPLAY`,
X socket or `XAUTHORITY` and exits, and the launch shuts the remaining nodes
down instead of hanging.

To do the same forwarding with plain compose:

```bash
docker compose run --rm \
    -e DISPLAY="$DISPLAY" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "${XAUTHORITY:-$HOME/.Xauthority}:/tmp/.Xauthority:ro" \
    arm bash
```

(omit the cookie mount on access-control-disabled displays such as WSLg, or
when the `XAUTHORITY`/`~/.Xauthority` file does not exist).

## Run

[run_arm.sh](run_arm.sh) provides unified launch commands for a native install
and for the Docker container. After `source .env`, it can be run from the
workspace root or this package directory, and sets up ROS/overlay sourcing
itself. Refer to its comments for all modes.

| Launch                        | Purpose                                                   |
| ----------------------------- | --------------------------------------------------------- |
| `arm sim_2d.launch.py`        | 2-D SFML view; add `driver:=true` for real hardware.      |
| `arm_gazebo sim_3d.launch.py` | 3-D Gazebo simulation.                                    |
| `arm hardware.launch.py`      | Real hardware, or fake/controller-only hardware.          |
| `arm manual.launch.py`        | Manual target-position node for an already-running stack. |

Every bringup accepts `control:=true|false` to enable or disable motor motion,
and `manual:=true` to add keyboard (WASD + Z/X) target control. Control defaults
to **true**; manual defaults to **false**.

`./run_arm.sh` with no arguments starts real hardware plus the 2-D model
reference, with control on.

```bash
# 2-D simulation (motion on by default)
ros2 launch arm sim_2d.launch.py

# Real hardware + 2-D model reference, feedback only
ros2 launch arm sim_2d.launch.py driver:=true control:=false

# Real hardware + 2-D model reference with motion enabled (the default)
ros2 launch arm sim_2d.launch.py driver:=true

# 3-D simulation (headless by default; gui:=true for the Gazebo window)
ros2 launch arm_gazebo sim_3d.launch.py gui:=true

# Real hardware: controller + motor driver, motion on
ros2 launch arm hardware.launch.py

# Real hardware, feedback only
ros2 launch arm hardware.launch.py control:=false

# Fake hardware: controller only, no motor driver
ros2 launch arm hardware.launch.py driver:=false

# Manual target-position control, with any of the bringups
ros2 launch arm sim_2d.launch.py manual:=true
ros2 launch arm_gazebo sim_3d.launch.py manual:=true
ros2 launch arm hardware.launch.py manual:=true

# Attach the manual node to a stack that is already running
ros2 launch arm manual.launch.py
```

Manual keys: W/S move y, A/D move x, Z/X move z, Q quits. The `step` parameter
sets metres per keypress.

For real hardware, `scripts/start_arm_tmux.sh` starts everything in tmux: the
hardware launch, the manual node, and a `feedback` window echoing `/arm/pos`
and `/arm/MotorFeedback`. It uses the local `.env` and `config/arm_params.yaml`
(or `ARM_PARAMS_FILE`):

```bash
./scripts/start_arm_tmux.sh
./scripts/start_arm_tmux.sh control:=false   # observe only, no motion
tmux kill-session -t arm                     # stop; then ./stop_sim.sh
```

The fyp workspace mirrors this helper into `scripts/`, plus
`scripts/start_arm_remote.sh`, which SSHes to another PC and starts/attaches
the same session there.

In `sim_2d.launch.py` every required process is fatal: closing the window or
losing the controller/driver shuts the whole launch down, so a failed node
cannot leave the launch hanging.

Drive a pose and watch feedback:

```bash
ros2 action send_goal arm/set_pos arm/action/SetPosition \
    "{target: {linear: {x: 0.04, y: 0.0, z: -0.27}}}"
ros2 topic echo arm/pos
ros2 topic echo arm/MotorFeedback
```

## Configuration

`config/arm_params.yaml` holds the per-node parameters:

- `arm_controller`: `feedback_rate`, `feedback_frame`, `reach_tolerance`,
  `reach_timeout`, plus the shared `geometry.*` and `sim.*` blocks.
- `arm_motor_driver`: `port_name`, `baudrate`, `servo_ids`, `start_angles`,
  `angle_min`/`angle_max`, `max_speed`, `auto_init`, `targets_topic`,
  `offline_streak`.
- `arm_manual_control`: `step`.

`enable_motion` is deliberately **not** a config-file parameter. The launch
`control:=true|false` argument sets it for the controller and driver at runtime;
`false` disables the `set_pos` action server and the driver's motor-target
subscription. It defaults to `true`. `publish_joints` is likewise launch-only
and is enabled by `sim_2d.launch.py` for its debug `arm/joints` stream.

`config/arm_gen0_params.yaml` is the alternate gen-0 geometry set.

## Components

| Node               | Purpose                                                                                                                                                   |
| ------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `arm_controller`   | `arm/set_pos` action (task-space target) → two-stage IK → `arm/motor_targets`. Publishes `arm/pos`; `arm/joints` is available only as a 2-D debug stream. |
| `arm_motor_driver` | Subscribes `arm/motor_targets`, drives the three bus-servos, publishes `arm/MotorFeedback`.                                                               |
| `arm_manual`       | Keyboard (WASD + Z/X) client for `arm/set_pos`.                                                                                                           |
| `arm_sim_sfml`     | Optional SFML 2-D top/side visualiser; draws `arm/pos`, or the measured `arm/MotorFeedback` when live.                                                    |

A separate Gazebo package (`arm_gazebo`) lives under `gazebo/` and is built as
a sibling package (see `AGENT.md`).

## ROS interfaces

| Name                | Type                           | Direction              | Purpose                                                                                                                          |
| ------------------- | ------------------------------ | ---------------------- | -------------------------------------------------------------------------------------------------------------------------------- |
| `arm/set_pos`       | action `SetPosition`           | client → controller    | Goal is a `geometry_msgs/Twist`; `linear.x/y/z` is the target in metres. Result `status` is `STATUS_OK`, `STATUS_UNREACHABLE` or `STATUS_POS_NOT_REACHED`. |
| `arm/pos`           | topic `ArmPosition`            | controller → consumers | Current pose, commanded target, current/target motor angles.                                                                     |
| `arm/joints`        | topic `sensor_msgs/JointState` | controller → consumers | Optional 2-D debug stream (three motor angles, degrees).                                                                         |
| `arm/motor_targets` | topic `MotorTargets`           | controller → driver    | Three motor-angle commands (degrees).                                                                                            |
| `arm/MotorFeedback` | topic `MotorFeedback`          | driver → consumers     | Measured angle, online flag and error code per servo.                                                                            |

## Tests

```bash
colcon test --packages-select arm
# or
ros2 launch arm test.launch.py
```

The GTest suite in `test/test_delta_arm.cpp` covers kinematics and the reachable
envelope.
