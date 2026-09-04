# Motor-driver feedback on the servo bus, published by the motor-driver node on
# topic "arm/motor_feedback" at the configured rate.
#
# Carries the measured (not just commanded) joint angle of each servo plus
# status/telemetry so a monitor, the controller, or an operator can verify the
# arm actually moved and detect faults.
std_msgs/Header header

# Measured joint angles (degrees), indexed exactly like arm/motor_targets.
# -1 indicates the servo did not report (offline / error).
float32[3] servo_angle_current

# Per-servo online flag (1 = responding on the bus, 0 = not reachable).
int8[3]    servo_online

# Per-servo error code:
#   0 = OK, 1 = communication/timeout fault, 2 = offline, -1 = unknown type
int8[3]    servo_error
