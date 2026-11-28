# 3DOF Delta Arm (ROS 2)

C++ controller + FashionStar UART driver for a 3-DOF delta arm.

Detailed design notes live in [`delta_arm_skill/`](delta_arm_skill/SKILL.md).

## Contents

1. [Build](#build)
2. [Run](#run)
3. [Interfaces](#interfaces)
4. [Tests](#tests)

## Build

Native target: Ubuntu 22.04 + ROS 2 Humble.

```bash
source /opt/ros/humble/setup.bash
git submodule update --init --recursive
cp .env.example .env            # then edit SERIAL_DEVICE etc.
source .env
cd ~/arm_ws
colcon build --symlink-install --packages-select arm
source install/setup.bash
```

SFML is optional; install `libsfml-dev` to enable `arm_sim_sfml`, or pass
`-DARM_BUILD_SFML_SIM=OFF`. `./scripts/setup.sh` builds with `.env` values.

## Run

```bash
ros2 launch arm hardware.launch.py                 # real hardware
ros2 launch arm hardware.launch.py control:=false  # feedback only
ros2 launch arm hardware.launch.py driver:=false   # fake hardware
ros2 launch arm sim_2d.launch.py                   # 2-D visualiser
ros2 launch arm_gazebo sim_3d.launch.py            # 3-D sim (separate pkg)
ros2 launch arm manual.launch.py                   # attach keyboard control
```

On the miniPC, `scripts/start_arm_tmux.sh` runs the whole stack locally (no
SSH) in tmux. `SERIAL_DEVICE` always comes from the miniPC's `.env`.

`control:=false` disables the action server and driver target subscription;
`manual:=true` adds `arm_manual`. See
[implementation_notes.md](delta_arm_skill/implementation_notes.md) for node
behaviour.

## Interfaces

| Name | Type | Purpose |
|---|---|---|
| `arm/set_pos` | action `SetPosition` | task-space goal (`Twist.linear`) |
| `arm/pos` | topic `ArmPosition` | pose + motor angles |
| `arm/motor_targets` | topic `MotorTargets` | controller -> driver |
| `arm/MotorFeedback` | topic `MotorFeedback` | measured servo angles/status |

`arm/set_pos` goals are smoothed by a straight-line endpoint modifier with
speed/acceleration limits (`trajectory.*`), and the resulting motor targets are
rate-limited per servo (`motor_limits.*`). Set `trajectory.enabled: false` to
send the goal directly; motor limits still apply.

`arm/joints` is an optional 2-D debug stream. See
[kinematics.md](delta_arm_skill/kinematics.md) and
[motion_limits.md](delta_arm_skill/motion_limits.md) for the IK model and
measured travel limits.

## Tests

```bash
colcon test --packages-select arm
ros2 launch arm test.launch.py
```
