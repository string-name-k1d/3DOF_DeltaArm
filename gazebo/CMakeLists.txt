cmake_minimum_required(VERSION 3.8)
project(arm_gazebo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# The arm package provides the MotorTargets message the sim bridges from.
find_package(arm REQUIRED)

# ---------------------------------------------------------------------------
# arm_cmd_bridge - Gazebo Harmonic closed-loop simulator bridge node
#
# Bridges the arm controller into the simulated 3-DoF delta arm:
#   * subscribes "arm/motor_targets" (arm/msg/MotorTargets) - commanded shoulder
#     angles in DEGREES
#   * publishes "/delta_arm/shoulder_<i>/cmd_pos" (std_msgs/Float64, RADIANS)
#     for the JointPositionController plugins in the gz-sim model.
# ---------------------------------------------------------------------------
add_executable(arm_cmd_bridge
  src/arm_cmd_bridge.cpp
)
ament_target_dependencies(arm_cmd_bridge
  rclcpp std_msgs arm
)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS arm_cmd_bridge
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/mount_welder.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()