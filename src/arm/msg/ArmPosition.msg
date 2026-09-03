# Current/Target top-level pose + the associated joint angles (degrees).
# Published by the controller on topic "delta_arm/pos".
std_msgs/msg/Header header
geometry_msgs/msg/Point position
float32[3] motor_angles_current
float32[3] motor_angles_target