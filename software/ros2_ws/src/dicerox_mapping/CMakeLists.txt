cmake_minimum_required(VERSION 3.10)
project(dicerox_mapping)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/scan_frame_republisher.py
  scripts/zed_planar_odom_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
