# Current/Target top-level pose + the associated joint angles (degrees).
# Published by the controller on topic "delta_arm/pos".
std_msgs/Header header
geometry_msgs/Point position
float32[3] motor_angles_current
float32[3] motor_angles_target