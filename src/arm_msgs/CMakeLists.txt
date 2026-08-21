cmake_minimum_required(VERSION 3.8)
project(arm_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ---------------------------------------------------------------------------
# Custom ROS interfaces
# ---------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmPosition.msg"
  "msg/MotorTargets.msg"
  "srv/TogglePositionStream.srv"
  "srv/MotorParamQuery.srv"
  "srv/EmergencyStop.srv"
  "action/SetPosition.action"
  ADD_DEPENDENCIES std_msgs geometry_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
