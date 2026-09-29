# Estimated torque required at the arm joints / end-effector to hold or move
# the attached payload, published by the arm_endpoint node on topic
# "arm/endpoint/torque_estimate".
#
# Skeleton message: the null endpoint module estimates a gravity-hold torque
# via the cross product r x F. A real module can replace this with a dynamic
# estimate; the fields are already shaped for that.
std_msgs/Header header

# Estimated end-effector hold torque vector in Nm.
geometry_msgs/Vector3 torque

# Lever arm from the joint axis to the payload CoG, meters (the r in r x F).
geometry_msgs/Vector3 lever_arm

# Force acting at the payload CoG, N (weight for the stub).
geometry_msgs/Vector3 force

# Per-joint torque estimate in Nm, indexed by the delta arm's 3 legs.
float32[3] joint_torque
