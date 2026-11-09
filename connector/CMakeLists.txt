cmake_minimum_required(VERSION 3.16)
project(arm_connector LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(arm_connector STATIC
  src/protocol.cpp
  src/can_transport.cpp
  src/loopback_transport.cpp
  src/connector.cpp
)

target_include_directories(arm_connector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(arm_connector PRIVATE -Wall -Wextra)
endif()

option(ARM_CONNECTOR_BUILD_TESTS "Build the standalone arm_connector smoke test" ON)
if(ARM_CONNECTOR_BUILD_TESTS)
  enable_testing()
  add_executable(arm_connector_smoke_test test/smoke_test.cpp)
  target_link_libraries(arm_connector_smoke_test PRIVATE arm_connector)
  add_test(NAME arm_connector_smoke_test COMMAND arm_connector_smoke_test)
endif()

install(TARGETS arm_connector
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
