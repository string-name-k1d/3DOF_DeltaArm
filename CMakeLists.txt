cmake_minimum_required(VERSION 3.8)
project(arm)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ---------------------------------------------------------------------------
# ROS 2 + external dependencies
# ---------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Optional: SFML (2D visual simulator). Optional so the hardware/controller/
# driver build succeeds headless even when the simulator is not installed.
# (The Gazebo 3D simulator lives in the separate arm_gazebo package.)
find_package(SFML 2.5 COMPONENTS graphics window system QUIET)

# ---------------------------------------------------------------------------
# Custom ROS interfaces defined and used inside this same package.
# ---------------------------------------------------------------------------
set(arm_msg_files
  "msg/ArmPosition.msg"
  "msg/ArmFeedback.msg"
  "msg/MotorTargets.msg"
)
set(arm_srv_files
  "srv/TogglePositionStream.srv"
  "srv/MotorParamQuery.srv"
  "srv/EmergencyStop.srv"
)
set(arm_action_files
  "action/SetPosition.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${arm_msg_files}
  ${arm_srv_files}
  ${arm_action_files}
  DEPENDENCIES std_msgs geometry_msgs action_msgs
)

# ---------------------------------------------------------------------------
# FashionStart C++ serial-bus servo driver (git submodule). Builds the
# "fsuartservo" and "cserialport" shared libraries from the checked-out
# submodule sources (self-contained; see motor_driver/CMakeLists.txt).
# ---------------------------------------------------------------------------
add_subdirectory(motor_driver)

# Local FSUS + CSerialPort include dirs used by the motor-driver code.
set(FSUS_INC
  ${CMAKE_CURRENT_SOURCE_DIR}/motor_driver/fashionstart-uart-servo/include
  ${CMAKE_CURRENT_SOURCE_DIR}/motor_driver/fashionstart-uart-servo/dependency/CSerialPort/include
)

# ---------------------------------------------------------------------------
# Kinematics library (ROS-neutral). Exported so the separate arm_gazebo
# package can re-use DeltaArm::Arm without recompiling the IK source.
# ---------------------------------------------------------------------------
add_library(arm_kinematics SHARED src/axis/delta-arm.cpp)
target_include_directories(arm_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
install(TARGETS arm_kinematics
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

# ---------------------------------------------------------------------------
# Motor-driver wrapper library (FSUS binding, ROS-neutral)
# ---------------------------------------------------------------------------
add_library(arm_lib SHARED src/driver/motor_driver.cpp)
target_include_directories(arm_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${FSUS_INC}
)
target_link_libraries(arm_lib fsuartservo)

# ---------------------------------------------------------------------------
# Executables
# ---------------------------------------------------------------------------
# Helper: link a ROS target to the interfaces generated in this package plus
# the standard ROS deps. On Humble we use rosidl_get_typesupport_target.
function(arm_link_ros TARGET)
  ament_target_dependencies(${TARGET}
    rclcpp rclcpp_action std_msgs geometry_msgs action_msgs sensor_msgs
  )
  rosidl_get_typesupport_target(cpp_typesupport_target
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(${TARGET} "${cpp_typesupport_target}")
endfunction()

function(arm_ros_target TARGET)
  target_include_directories(${TARGET} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${FSUS_INC}
  )
  arm_link_ros(${TARGET})
endfunction()

# Single-process: controller + motor driver (real hardware, tight coupling).
# The control augmenter is a logic-only class (src/system/control_augmenter.cpp)
# folded into the controller node - it is applied to every published
# MotorTargets message, so no separate relay node is needed.
add_executable(arm_system
  src/controller/delta_arm_controller_node.cpp
  src/driver/motor_driver_node.cpp
  src/system/control_augmenter.cpp
  src/system/delta_arm_system_main.cpp
)
arm_ros_target(arm_system)
target_link_libraries(arm_system arm_lib arm_kinematics)

# Standalone controller (simulation / external driver). The logic-only control
# augmenter (src/system/control_augmenter.cpp) is compiled in here too, since
# the controller node applies it to every published MotorTargets message.
add_executable(arm_controller
  src/controller/delta_arm_controller_node.cpp
  src/controller/delta_arm_controller_main.cpp
  src/system/control_augmenter.cpp
)
arm_ros_target(arm_controller)
target_link_libraries(arm_controller arm_kinematics)

# Standalone motor driver.
add_executable(arm_motor_driver
  src/driver/motor_driver.cpp
  src/driver/motor_driver_node.cpp
  src/driver/motor_driver_main.cpp
)
arm_ros_target(arm_motor_driver)
target_link_libraries(arm_motor_driver fsuartservo)

# Manual (WASD) control.
add_executable(arm_manual
  src/manual/delta_arm_manual.cpp
  src/manual/delta_arm_manual_main.cpp
)
target_include_directories(arm_manual PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
arm_link_ros(arm_manual)

# ---------------------------------------------------------------------------
# Simulators (all OPTIONAL - never block the hardware/controller/driver build)
# ---------------------------------------------------------------------------
# SFML 2D visual simulator - built only when SFML is available AND the main
# file is present (requires a display for visualization).
if(SFML_FOUND AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/sim/arm_sim_sfml_main.cpp")
  add_executable(arm_sim_sfml
    src/sim/arm_sim_sfml.cpp
    src/sim/arm_sim_sfml_main.cpp
  )
  target_include_directories(arm_sim_sfml PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  arm_link_ros(arm_sim_sfml)
  target_link_libraries(arm_sim_sfml
    arm_kinematics
    sfml-graphics sfml-window sfml-system
  )
endif()

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS
  arm_lib
  cserialport
  fsuartservo
  arm_system
  arm_controller
  arm_motor_driver
  arm_manual
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
if(TARGET arm_sim_sfml)
  install(TARGETS arm_sim_sfml
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_arm
    test/test_delta_arm.cpp
  )
  target_include_directories(test_arm PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(test_arm arm_kinematics)

  install(TARGETS test_arm
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
