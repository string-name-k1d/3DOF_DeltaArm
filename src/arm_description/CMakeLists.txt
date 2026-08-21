cmake_minimum_required(VERSION 3.8)
project(arm_description)

find_package(ament_cmake REQUIRED)

# Install the model/assets directories so they are available at runtime
# (e.g. $(find-pkg-share arm_description)/urdf/...).
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
