# Implementation notes

## Controller

- Creates `arm/set_pos` only when `enable_motion=true`.
- Publishes `arm/motor_targets` and `arm/pos`; `arm/joints` is opt-in.
- On each goal: run IK, publish motor targets, then wait for the measured pose
  to enter `reach_tolerance` before reporting success.
- A newer goal supersedes the old one; the old goal is aborted.
- With no motor feedback and `sim.simulate_arrival=true`, the commanded pose is
  promoted to the current pose so simulation/visualisation can run headless.
- Motor feedback with any offline/invalid servo is ignored for the forward
  estimate.

## Motor driver

- Subscribes to `arm/motor_targets` only when motion is enabled.
- `port_name` comes from `SERIAL_DEVICE` via the launch override.
- Serial faults are debounced over `offline_streak` samples before a servo is
  reported offline.
- `start_angles` are installation offsets; controller angles are joint-frame
  values and the driver adds the offset on the wire.

## Manual node

- Action client to `arm/set_pos`, subscriber to `arm/pos`.
- Each keypress starts from the last measured pose, not an accumulated command,
  so rejected targets cannot push the next step deeper into an invalid area.
- Uses raw terminal keypress mode; start with `ros2 run arm arm_manual`, not
  through `ros2 launch`, because launch does not forward stdin.

## Simulators

- `arm_sim_sfml` is optional (SFML is not a rosdep dependency).
- It checks the X display before opening a window and exits cleanly when
  `DISPLAY`, the X socket or `XAUTHORITY` is missing.
- The Gazebo bridge in `gazebo/` is a separate package and consumes
  `arm/motor_targets`.

## Launch

- `arm_bringup.launch.py` is shared by all user-facing launch files.
- `SERIAL_DEVICE` is appended after the params file so installed YAML cannot
  override it.
- Required processes in `sim_2d.launch.py` are fatal on exit to avoid a hung
  launch after a window close or node crash.
- `scripts/start_arm_tmux.sh` runs entirely on the miniPC: no SSH, no remote
  tmux. Each pane clears inherited ROS variables, sources `.env`, then sources
  `<workspace>/install/setup.bash`.

## Build

```bash
cd ~/arm_ws
colcon build --symlink-install --packages-select arm
source install/setup.bash
```

Rebuild after changing C++ sources, `.srv`/`.action`/`.msg` files, or installed
YAML. `SERIAL_DEVICE` alone does not require a rebuild because the launch file
applies it at runtime.
