# Whole-arm health snapshot published by arm_health_monitor.
std_msgs/Header header

# Overall status: "ok", "degraded", "fault", or "unknown".
string status
bool healthy
bool ready

# Mode hints inferred from observed interfaces.
# True when the arm stack appears configured for position-target control
# (controller/action present). This does not imply goals are currently accepted.
bool motion_enabled
bool controller_seen
bool driver_seen
bool endpoint_seen
bool control_state_seen
bool motor_feedback_seen

# Controller/control state (when available).
string control_state
bool emergency_active
bool accepts_goals

# Motor summary (from arm/motor_feedback when available).
uint8 motor_count
uint8 motors_online
uint8 motors_faulted
int8[3] servo_online
int8[3] servo_error

# Timing/freshness (seconds, -1 if never seen).
float32 control_state_age_sec
float32 motor_feedback_age_sec

# Human-readable issues/warnings.
string[] issues
