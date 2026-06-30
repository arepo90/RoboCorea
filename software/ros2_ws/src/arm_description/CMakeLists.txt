cmake_minimum_required(VERSION 3.10)
project(arm_description)

find_package(ament_cmake REQUIRED)

# Install the description, meshes and launch files so package:// URIs resolve and
# the digital-twin / collision pipeline can find them at runtime.
install(
  DIRECTORY urdf meshes launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
