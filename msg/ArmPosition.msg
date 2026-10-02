# Current/Target top-level pose + the associated joint angles (degrees).
# Published by the controller on topic "delta_arm/pos".
std_msgs/Header header
geometry_msgs/Point position
# Commanded ("control") target in task space, metres. Published by the controller
# from the last accepted set_pos goal (Arm::get_tar_pos); equals `position` once
# the motion has settled. Drawn by the 2-D visualiser as the target marker.
geometry_msgs/Point target_position
float32[3] motor_angles_current
float32[3] motor_angles_target