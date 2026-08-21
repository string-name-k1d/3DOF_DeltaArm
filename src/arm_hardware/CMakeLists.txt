cmake_minimum_required(VERSION 3.8)
project(arm_hardware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# ---------------------------------------------------------------------------
# ROS 2 + interfaces
# ---------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(arm_msgs REQUIRED)

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
# Motor driver wrapper library (FSUS binding, ROS-neutral)
# ---------------------------------------------------------------------------
add_library(arm_hardware SHARED src/driver/motor_driver.cpp)
target_include_directories(arm_hardware PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${FSUS_INC}
)
target_link_libraries(arm_hardware fsuartservo)

# ---------------------------------------------------------------------------
# Executables
# ---------------------------------------------------------------------------
# Helper: ROS deps + include dirs common to the ROS nodes.
function(arm_ros_target TARGET)
  target_include_directories(${TARGET} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${FSUS_INC}
  )
  ament_target_dependencies(${TARGET}
    arm_msgs rclcpp rclcpp_action std_msgs geometry_msgs action_msgs
  )
endfunction()

# Combined: controller + driver in one process (tight coupling).
add_executable(arm_system
  src/axis/delta-arm.cpp
  src/controller/delta_arm_controller_node.cpp
  src/driver/motor_driver_node.cpp
  src/system/delta_arm_system_main.cpp
)
arm_ros_target(arm_system)
target_link_libraries(arm_system arm_hardware)

# Controller only (used by the simulation launch - driver provided externally).
add_executable(arm_controller
  src/axis/delta-arm.cpp
  src/controller/delta_arm_controller_node.cpp
  src/controller/delta_arm_controller_main.cpp
)
arm_ros_target(arm_controller)

# Driver only (standalone / hardware bring-up / debugging).
add_executable(arm_motor_driver
  src/driver/motor_driver.cpp
  src/driver/motor_driver_node.cpp
  src/driver/motor_driver_main.cpp
)
arm_ros_target(arm_motor_driver)
target_link_libraries(arm_motor_driver fsuartservo)

# Manual-control (WASD) action client.
add_executable(arm_manual
  src/manual/delta_arm_manual.cpp
  src/manual/delta_arm_manual_main.cpp
)
target_include_directories(arm_manual PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(arm_manual arm_msgs rclcpp rclcpp_action)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS
  arm_hardware
  cserialport
  fsuartservo
  arm_system
  arm_controller
  arm_motor_driver
  arm_manual
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
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

  # Install the test binary so a launch file can execute it (test.launch.py).
  install(TARGETS test_arm
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

ament_package()
