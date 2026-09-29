# High-level arm control state published by the controller on topic
# "arm/control_state". Reports which of the control states the arm is in and
# why it last changed. The state governs goal acceptance: a latched
# emergency/zero state rejects new set_pos goals.
std_msgs/Header header

# One of: "zero_emergency", "home", "position_control", "unknown".
string state

# True while the emergency/zero state is latched (goals rejected).
bool emergency_active

# True when the state permits new position goals.
bool accepts_goals

# Free-form reason for the most recent state transition.
string last_reason

# Task-space home posture in meters (the pose the arm returns to).
float32[3] home_position

# Current task-space pose in meters (diagnostics only).
float32[3] current_position
