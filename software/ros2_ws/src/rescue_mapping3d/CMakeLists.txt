cmake_minimum_required(VERSION 3.10)
project(rescue_mapping3d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(octomap_node src/octomap_node.cpp)
target_compile_features(octomap_node PUBLIC cxx_std_17)
target_include_directories(octomap_node PRIVATE ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(octomap_node ${OCTOMAP_LIBRARIES})
ament_target_dependencies(octomap_node
  rclcpp sensor_msgs geometry_msgs octomap_msgs tf2 tf2_ros)

install(TARGETS octomap_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
