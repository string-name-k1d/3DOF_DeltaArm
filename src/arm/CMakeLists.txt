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

# Optional: SFML (2D visual simulator), Gazebo (3D simulator) and GLM (used by
# the Gazebo sim). All are OPTIONAL so the hardware/controller/driver build
# succeeds headless even when the simulators are not installed.
find_package(SFML 2.5 COMPONENTS graphics window system QUIET)
find_package(gazebo_ros QUIET)
find_package(glm QUIET)

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
add_executable(arm_system
  src/axis/delta-arm.cpp
  src/controller/delta_arm_controller_node.cpp
  src/driver/motor_driver_node.cpp
  src/system/delta_arm_system_main.cpp
)
arm_ros_target(arm_system)
target_link_libraries(arm_system arm_lib)

# Standalone controller (simulation / external driver).
add_executable(arm_controller
  src/axis/delta-arm.cpp
  src/controller/delta_arm_controller_node.cpp
  src/controller/delta_arm_controller_main.cpp
)
arm_ros_target(arm_controller)

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
    src/axis/delta-arm.cpp
    src/sim/arm_sim_sfml.cpp
    src/sim/arm_sim_sfml_main.cpp
  )
  target_include_directories(arm_sim_sfml PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  arm_link_ros(arm_sim_sfml)
  target_link_libraries(arm_sim_sfml
    sfml-graphics sfml-window sfml-system
  )
endif()

# Gazebo 3D simulator - built only when gazebo_ros + glm are available AND the
# source files are present.
if(gazebo_ros_FOUND AND glm_FOUND
   AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/sim/arm_sim_gazebo.cpp"
   AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/sim/arm_sim_gazebo_main.cpp")
  add_executable(arm_sim_gazebo
    src/axis/delta-arm.cpp
    src/sim/arm_sim_gazebo.cpp
    src/sim/arm_sim_gazebo_main.cpp
  )
  target_include_directories(arm_sim_gazebo PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  arm_link_ros(arm_sim_gazebo)
  target_link_libraries(arm_sim_gazebo glm)
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
if(TARGET arm_sim_gazebo)
  install(TARGETS arm_sim_gazebo
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_arm
    src/axis/delta-arm.cpp
    test/test_delta_arm.cpp
  )
  target_include_directories(test_arm PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

  install(TARGETS test_arm
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
