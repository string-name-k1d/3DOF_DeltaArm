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
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(glm QUIET)
find_package(gazebo REQUIRED)
find_package(gazebo_ros QUIET)

# The arm package exposes the delta-arm kinematics (DeltaArm::Arm) that the
# gazebo sim re-uses to convert target -> joint angles.
find_package(arm REQUIRED)

# ---------------------------------------------------------------------------
# arm_sim_gazebo - Gazebo 3D delta-arm simulator node
#
# Bridges the arm controller into a simulated delta-arm model:
#   * subscribes "arm/motor_targets" (arm/msg/MotorTargets) - commanded joints
#   * publishes "arm/joints" (sensor_msgs/JointState) - simulated feedback
#   * publishes "arm/pos" (arm/msg/ArmPosition) - simulated end-effector pose
#
# Built only when gazebo_ros + glm are available (same optional gate the arm
# package used) so the sim never blocks a headless build.
# ---------------------------------------------------------------------------
if(gazebo_ros_FOUND AND glm_FOUND
   AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/arm_sim_gazebo.cpp"
   AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/arm_sim_gazebo_main.cpp")
  add_executable(arm_sim_gazebo
    src/arm_sim_gazebo.cpp
    src/arm_sim_gazebo_main.cpp
  )
  target_include_directories(arm_sim_gazebo PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(arm_sim_gazebo
    rclcpp sensor_msgs geometry_msgs std_msgs arm
  )
  target_link_libraries(arm_sim_gazebo arm::arm_kinematics glm::glm)
endif()

# ---------------------------------------------------------------------------
# delta_arm_gazebo_plugin - Gazebo (Classic) model plugin
#
# Loaded by the delta_arm model in worlds/delta_arm.world. Subscribes to the
# arm controller's arm/motor_targets (degrees) and servoes the three shoulder
# revolute joints. Requires gazebo_ros for the shared ROS node.
# ---------------------------------------------------------------------------
if(gazebo_ros_FOUND
   AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/delta_arm_gazebo_plugin.cpp")
  add_library(delta_arm_gazebo_plugin SHARED
    src/delta_arm_gazebo_plugin.cpp
  )
  target_include_directories(delta_arm_gazebo_plugin PRIVATE
    ${GAZEBO_INCLUDE_DIRS}
    ${gazebo_ros_INCLUDE_DIRS}
  )
  ament_target_dependencies(delta_arm_gazebo_plugin
    rclcpp
    arm
  )
  target_link_libraries(delta_arm_gazebo_plugin
    ${gazebo_ros_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    arm::arm__rosidl_typesupport_cpp
  )
  set_target_properties(delta_arm_gazebo_plugin PROPERTIES
    BUILD_RPATH_USE_ORIGIN TRUE
    INSTALL_RPATH_USE_LINK_PATH TRUE
  )
endif()

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
if(TARGET arm_sim_gazebo)
  install(TARGETS arm_sim_gazebo
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

if(TARGET delta_arm_gazebo_plugin)
  install(TARGETS delta_arm_gazebo_plugin
    LIBRARY DESTINATION lib
  )
endif()

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
