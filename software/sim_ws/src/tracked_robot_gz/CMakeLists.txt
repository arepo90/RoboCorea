cmake_minimum_required(VERSION 3.8)
project(tracked_robot_gz)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY worlds launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
