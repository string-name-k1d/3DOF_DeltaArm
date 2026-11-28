# Parameters

## Ownership

- `config/arm_params.yaml`: default geometry, limits, sim startup, controller,
  driver and manual settings.
- `config/arm_gen0_params.yaml`: alternate gen-0 geometry.
- `SERIAL_DEVICE` in `.env`: serial port. `arm_bringup.launch.py` applies it to
  the driver **after** the YAML so a stale installed `port_name` cannot win.
- `enable_motion`, `publish_joints`, `manual`, `driver`, `interface`: launch-only
  switches. Never put them in YAML.

## `arm_controller`

| Parameter | Default | Meaning |
|---|---:|---|
| `feedback_rate` | 20.0 | Hz for `arm/pos`. |
| `feedback_frame` | `base_link` | frame id in published headers. |
| `reach_tolerance` | 0.005 | metres; pose error considered arrived. |
| `reach_timeout` | 10.0 | seconds before a move reports `STATUS_POS_NOT_REACHED`. |
| `enable_motion` | true | start `arm/set_pos` action server. |
| `publish_joints` | false | publish debug `arm/joints`. |

## `geometry.*`

Lengths are millimetres; angles are degrees except `arm_angle_min/max`, which
are radians inside C++ but degrees in YAML.

| Parameter | Main default | Gen0 | Meaning |
|---|---:|---:|---|
| `base_radius` | 100.0 | 100.0 | shoulder pivot radius. |
| `platform_radius` | 32.5 | 32.5 | platform joint radius. |
| `upper_arm_len` | 120.0 | 120.0 | shoulder -> elbow. |
| `lower_arm_len` | 240.0 | 250.0 | elbow -> platform joint. |
| `servo_radius` | 57.65 | 60.0 | servo shaft radius. |
| `servo_z` | -22.5 | -25.0 | servo shaft height below shoulder plane. |
| `upper_rod_len` | 60.0 | 35.0 | servo horn / crank. |
| `servo_rod_len` | 35.0 | 60.0 | connecting rod. |
| `arm_attach_dist` | 68.5 | 52.0 | shoulder -> arm bracket along arm. |
| `arm_attach_offset` | 20.5 | 20.5 | perpendicular bracket standoff. |
| `angle_min` | `[0,0,0]` | `[0,0,0]` | raw servo lower travel. |
| `angle_max` | `[145,145,145]` | `[145,145,145]` | raw servo upper travel. |
| `arm_angle_min` | 0.0 | 0.0 | practical upper-arm lower bound, deg. |
| `arm_angle_max` | 180.0 | 180.0 | practical upper-arm upper bound, deg. |
| `bypass_reachability` | false | true | suppress stage-1/band refusals. |
| `raw_motor_clamp` | true | true | temporary raw motor clamp mode. |

`raw_motor_clamp: true` means the derived reachable band is **diagnostic only**:
stage-1 circle misses are tangent-clamped and the solved motor command is
clamped only to `angle_min..angle_max`. Set it to `false` to restore the
band-based reachability guard.

See [motion_limits.md](motion_limits.md) for why `0..145` is not representative
of the measured mechanism.

## `arm_motor_driver`

`port_name` is omitted from source YAML on purpose. Other keys: `baudrate`,
`servo_ids`, `start_angles`, `max_speed`, `auto_init`, `targets_topic`,
`offline_streak`, `feedback_rate`, `feedback_frame`, `enable_motion`.

## `arm_manual_control`

`step` is metres per keypress. Optional `pos_min` / `pos_max` add a task-space
clamp. Each step starts from the last measured pose on `arm/pos`.

## `arm_interface`

| Parameter | Default | Meaning |
|---|---:|---|
| `home_position` | `[0,0,-0.28]` | target used when entering the `home` state. |
| `move_timeout` | 10.0 | fallback timeout when a service request uses 0. |
| `action_wait_timeout` | 2.0 | seconds to wait for `arm/set_pos` server. |
| `cancel_timeout` | 2.0 | seconds to wait for action cancellation. |
