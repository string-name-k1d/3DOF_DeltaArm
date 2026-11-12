# 3DOF Delta Arm (ROS 2 package)

Minimal ROS 2 (C++) package for a 3-DOF delta arm driven by FashionStar UART
bus-servos.

See [`docs/KINEMATICS.md`](docs/KINEMATICS.md). The implementation lives in
`src/controller/delta-arm_dynamics.cpp` and is built as the exported
`arm_kinematics` shared library, linked by the controller, visualiser and tests.

## TOC

1. [Build](#build)
2. [Run](#run)
3. [Configs](#configuration)
4. [ROS Archi](#ros-interfaces)

## Build

```bash
# for FashionStart SDK git submodule:
git submodule update --init --recursive

# From the repository root, in a ROS 2 Humble environment:
colcon build --symlink-install
source install/setup.bash
```

The bundled `compose.yaml` keeps the USB-serial adapter out of the default
`arm` service, so simulation and development work without hardware:

```bash
docker compose run --rm arm              # fake 2-D reference, control on
docker compose run --rm arm simulation   # fake hardware: controller only
docker compose run --rm arm sim2d        # 2-D SFML view, control on
docker compose run --rm arm sim3d        # 3-D Gazebo simulation
docker compose run --rm arm bash         # dev shell / build
docker compose run --rm arm-hardware     # real hardware + 2-D reference,
                                         # control on; binds /dev/ttyUSB0
```

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

Use [run_arm.sh](run_arm.sh) for unified launch commands. Refer to comments there.

| Launch                        | Purpose                                                   |
| ----------------------------- | --------------------------------------------------------- |
| `arm sim_2d.launch.py`        | 2-D SFML view; add `driver:=true` for real hardware.      |
| `arm_gazebo sim_3d.launch.py` | 3-D Gazebo simulation.                                    |
| `arm hardware.launch.py`      | Real hardware, or fake/controller-only hardware.          |
| `arm manual.launch.py`        | Manual target-position node for an already-running stack. |

Every bringup accepts `control:=true|false` to enable or disable motor motion,
and `manual:=true` to add keyboard (WASD + Z/X) target control. Control defaults
to **true**; manual defaults to **false**.

`./run_arm.sh` with no arguments starts real hardware plus the 2-D
model reference, with control on.

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

- `arm_controller`: `feedback_rate`, `feedback_frame`, plus the shared
  `geometry.*` and `sim.*` blocks.
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
| `arm/set_pos`       | action `SetPosition`           | client → controller    | Goal is a `geometry_msgs/Twist`; `linear.x/y/z` is the target in metres. Result `status` is `STATUS_OK` or `STATUS_UNREACHABLE`. |
| `arm/pos`           | topic `ArmPosition`            | controller → consumers | Current pose, commanded target, current/target motor angles.                                                                     |
| `arm/joints`        | topic `sensor_msgs/JointState` | controller → consumers | Optional 2-D debug stream (three motor angles, degrees).                                                                         |
| `arm/motor_targets` | topic `MotorTargets`           | controller → driver    | Three motor-angle commands (degrees).                                                                                            |
| `arm/MotorFeedback` | topic `ArmFeedback`            | driver → consumers     | Measured angle, online flag and error code per servo.                                                                            |

## Tests

```bash
colcon test --packages-select arm
# or
ros2 launch arm test.launch.py
```

The GTest suite in `test/test_delta_arm.cpp` covers kinematics and the reachable
envelope.
