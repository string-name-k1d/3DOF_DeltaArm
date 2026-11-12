# 3DOF Delta Arm (ROS 2 package)

Minimal ROS 2 (C++) package for a 3-DOF delta arm driven by FashionStar UART
bus-servos. The package root **is** the repository root.

## Components

| Node | Purpose |
|---|---|
| `arm_controller` | `arm/set_pos` action (task-space target) → two-stage IK → `arm/motor_targets`. Publishes `arm/pos` + `arm/joints` feedback. |
| `arm_motor_driver` | Subscribes `arm/motor_targets`, drives the three bus-servos, publishes `arm/motor_feedback`. |
| `arm_manual` | Keyboard (WASD + Z/X) client for `arm/set_pos`. |
| `arm_sim_sfml` | Optional SFML 2-D top/side visualiser; draws `arm/pos`, or the measured `arm/motor_feedback` when live. |

A separate Gazebo package (`arm_gazebo`) lives under `gazebo/` and is built as
a sibling package (see `AGENT.md`).

## ROS interfaces

| Name | Type | Direction | Purpose |
|---|---|---|---|
| `arm/set_pos` | action `SetPosition` | client → controller | Goal is a `geometry_msgs/Twist`; `linear.x/y/z` is the target in metres. |
| `arm/pos` | topic `ArmPosition` | controller → consumers | Current pose, commanded target, current/target motor angles. |
| `arm/joints` | topic `sensor_msgs/JointState` | controller → consumers | Three motor angles (degrees). |
| `arm/motor_targets` | topic `MotorTargets` | controller → driver | Three motor-angle commands (degrees). |
| `arm/motor_feedback` | topic `ArmFeedback` | driver → consumers | Measured angle, online flag and error code per servo. |

## Build

```bash
# From the repository root, in a ROS 2 Humble environment:
colcon build --symlink-install
source install/setup.bash
```

The FashionStart SDK is a git submodule:

```bash
git submodule update --init --recursive
```

The bundled `compose.yaml` keeps the USB-serial adapter out of the default
`arm` service, so simulation and development work without hardware:

```bash
docker compose run --rm arm simulation   # controller only, no USB needed
docker compose run --rm arm bash         # dev shell / build
docker compose run --rm arm-hardware     # real servos (binds /dev/ttyUSB0)
```

For a one-step launch that starts the stack and leaves you in a shell inside
the container:

```bash
./scripts/launch_container.sh              # simulation + bash
./scripts/launch_container.sh --hardware   # real servos + bash
```

The helper also forwards the host X11 display when available, so the SFML
window works from the shell it opens. `arm_sim_sfml` checks the display before
creating the window: on a headless host it reports the missing `DISPLAY`,
X socket or `XAUTHORITY` and exits, and the launch shuts the remaining nodes
down instead of hanging. To do the same forwarding with plain compose:

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

```bash
# Real hardware: controller + motor driver
ros2 launch arm delta_arm.launch

# Feedback only (no motion; servos are pinged and reported)
ros2 launch arm delta_arm.launch motion:=false

# Simulation: controller only
ros2 launch arm delta_arm.launch simulation:=true

# Controller + manual keyboard jogging (simulation by default)
ros2 launch arm manual.launch
ros2 launch arm manual.launch simulation:=false

# 2-D SFML visualiser (controller + window)
ros2 launch arm arm_sim_2d.launch.py
```

In `arm_sim_2d.launch.py` every required process is fatal: closing the window
or losing the controller/driver shuts the whole launch down, so a failed node
cannot leave the launch hanging.

Drive a pose and watch feedback:

```bash
ros2 action send_goal arm/set_pos arm/action/SetPosition \
    "{target: {linear: {x: 0.04, y: 0.0, z: -0.27}}}"
ros2 topic echo arm/pos
ros2 topic echo arm/motor_feedback
```

## Configuration

`config/arm_params.yaml` holds the per-node parameters:

- `arm_controller`: `feedback_rate`, `feedback_frame`, `enable_motion`, plus the
  shared `geometry.*` and `sim.*` blocks.
- `arm_motor_driver`: `port_name`, `baudrate`, `servo_ids`, `start_angles`,
  `angle_min`/`angle_max`, `max_speed`, `auto_init`, `enable_motion`,
  `targets_topic`, `offline_streak`.
- `arm_manual_control`: `step`.

`config/arm_feedback_only.yaml` is a node-specific override used by
`motion:=false` runs; it wins over `arm_params.yaml` so the driver never creates
its `arm/motor_targets` subscription.

`config/arm_gen0_params.yaml` is the alternate gen-0 geometry set.

## Inverse kinematics

See [`docs/KINEMATICS.md`](docs/KINEMATICS.md). The library is built as the
exported `arm_kinematics` shared library and linked by the controller, the
visualiser and the external Gazebo package.

## Tests

```bash
colcon test --packages-select arm
# or
ros2 launch arm test.launch.py
```

The GTest suite in `test/test_delta_arm.cpp` covers kinematics and the reachable
envelope.
