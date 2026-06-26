cmake_minimum_required(VERSION 3.10)
project(rescue_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SavePose.srv"
  "srv/GoToPose.srv"
  "srv/DeletePose.srv"
  "srv/ListPoses.srv"
  "srv/SaveMap.srv"
  "srv/ListMaps.srv"
  "srv/LoadMap.srv"
  "srv/DeleteMap.srv"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
