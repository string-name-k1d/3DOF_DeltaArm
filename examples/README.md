# `arm/examples`

Dummy/reference examples for arm integration workflows.

## `px4_joystick_arm_control.py`

Maps PX4 manual-control/joystick inputs (`px4_msgs/msg/ManualControlSetpoint`,
typically bridged as `fmu/out/manual_control_setpoint`) to `arm/action/SetPosition`
goals.

This is a **dummy implementation** for integration testing and button/axis
mapping discovery:
- axis signs/ranges/mappings are vehicle/QGC/radio specific
- it does not own PX4 mode/arming or `drone_mission` takeover behavior
- manual arm control is gated by a togglable flag and blocked while arm
  emergency state is active
- optional PX4 button bit edges can call `arm/srv/EmergencyStop` with
  `action=TOGGLE` (mapping left configurable)

Run in an environment with `rclpy`, `arm` interfaces, and `px4_msgs` sourced:

```bash
python3 arm/examples/px4_joystick_arm_control.py --ros-args \
  -p manual_control_topic:=fmu/out/manual_control_setpoint \
  -p home_x:=0.0 -p home_y:=0.0 -p home_z:=-0.18 \
  -p manual_enable_bit:=1 -p e_stop_bit:=0
```

Safety: validate mappings with motion disabled / in simulation first.
