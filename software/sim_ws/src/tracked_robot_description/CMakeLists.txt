cmake_minimum_required(VERSION 3.8)
project(tracked_robot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
  FILES_MATCHING
    PATTERN "*.urdf"
    PATTERN "*.xacro"
)

ament_package()
