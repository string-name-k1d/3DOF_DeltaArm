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

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

option(ARM_BUILD_SFML_SIM "Build the optional 2-D SFML visualiser" ON)
if(ARM_BUILD_SFML_SIM)
  find_package(SFML 2.5 COMPONENTS graphics window system QUIET)
  if(SFML_FOUND)
    message(STATUS "arm: building arm_sim_sfml")
  else()
    message(STATUS "arm: SFML not found - skipping arm_sim_sfml")
  endif()
else()
  message(STATUS "arm: ARM_BUILD_SFML_SIM=OFF - skipping arm_sim_sfml")
endif()

set(arm_msg_files
  "msg/ArmPosition.msg"
  "msg/MotorFeedback.msg"
  "msg/MotorTargets.msg"
)
set(arm_srv_files
  "srv/GetState.srv"
  "srv/SetState.srv"
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

if(NOT EXISTS
   "${CMAKE_CURRENT_SOURCE_DIR}/motor_driver/fashionstart-uart-servo/CMakeLists.txt")
  message(FATAL_ERROR
    "FashionStar SDK submodule missing: motor_driver/fashionstart-uart-servo\n"
    "Run: git submodule update --init --recursive")
endif()
add_subdirectory(motor_driver)

set(FSUS_INC
  ${CMAKE_CURRENT_SOURCE_DIR}/motor_driver/fashionstart-uart-servo/include
  ${CMAKE_CURRENT_SOURCE_DIR}/motor_driver/fashionstart-uart-servo/dependency/CSerialPort/include
)

add_library(arm_kinematics SHARED src/controller/delta-arm_dynamics.cpp)
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

add_library(arm_lib SHARED src/driver/motor_driver.cpp)
target_include_directories(arm_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${FSUS_INC}
)
target_link_libraries(arm_lib fsuartservo)

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

add_executable(arm_controller
  src/controller/delta_arm_controller_node.cpp
  src/controller/delta_arm_controller_main.cpp
)
arm_ros_target(arm_controller)
target_link_libraries(arm_controller arm_kinematics)

add_executable(arm_motor_driver
  src/driver/motor_driver_node.cpp
  src/driver/motor_driver_main.cpp
)
arm_ros_target(arm_motor_driver)
target_link_libraries(arm_motor_driver arm_lib)

add_executable(arm_manual
  src/manual/delta_arm_manual.cpp
  src/manual/delta_arm_manual_main.cpp
)
target_include_directories(arm_manual PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
arm_link_ros(arm_manual)

if(ARM_BUILD_SFML_SIM AND SFML_FOUND
   AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/sim/arm_sim_sfml_main.cpp")
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
  find_package(X11 QUIET)
  if(X11_FOUND)
    target_link_libraries(arm_sim_sfml X11::X11)
    target_compile_definitions(arm_sim_sfml PRIVATE ARM_SIM_HAVE_X11=1)
  endif()
endif()

install(TARGETS
  arm_lib
  cserialport
  fsuartservo
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
ament_python_install_package(arm_interface)
install(PROGRAMS arm_interface/interface_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME arm_interface_node
)
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_arm test/test_delta_arm.cpp)
  target_include_directories(test_arm PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(test_arm arm_kinematics)
  install(TARGETS test_arm
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
