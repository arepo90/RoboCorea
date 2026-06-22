cmake_minimum_required(VERSION 3.8)
project(tracked_robot_nav2)

find_package(ament_cmake REQUIRED)

# Generate the occupancy map at configure time so it is available for install.
# Re-run cmake if you change the world layout.
find_program(PYTHON3 python3 REQUIRED)
execute_process(
  COMMAND ${PYTHON3} ${CMAKE_CURRENT_SOURCE_DIR}/maps/generate_map.py
  RESULT_VARIABLE _map_result
)
if(NOT _map_result EQUAL 0)
  message(WARNING
    "tracked_robot_nav2: map generation failed (exit ${_map_result}). "
    "Make sure python3 is available and run cmake again.")
endif()

install(
  DIRECTORY config maps rviz launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
