cmake_minimum_required(VERSION 3.10)
project(gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# ── Dependencies ──────────────────────────────────────────────────────────────
# Audio playback is via GStreamer (autoaudiosink), so no direct PulseAudio dep.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)                             # arm lifecycle Trigger services
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)                            # map window: NavigateToPose goal
find_package(rclcpp_action REQUIRED)                        # map window: NavigateToPose action client
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(tf2 REQUIRED)                                   # map window: robot pose
find_package(tf2_ros REQUIRED)                              # map window: TF listener
find_package(rescue_interfaces REQUIRED)                     # saved-pose services
find_package(ament_index_cpp REQUIRED)
find_package(urdf REQUIRED)                                  # digital twin: URDF parse
find_package(Qt6 REQUIRED COMPONENTS Widgets OpenGL OpenGLWidgets)  # twin: GL view
find_package(OpenCV REQUIRED)
find_package(assimp REQUIRED)                                # twin: mesh loading

# ONNX Runtime (YOLO hazmat inference, CUDA EP with CPU fallback). Installed
# manually under /usr/local (see the legacy gui README).
find_library(ONNXRUNTIME_LIB onnxruntime HINTS /usr/local/lib REQUIRED)
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  HINTS /usr/local/include/onnxruntime REQUIRED)

# ZBar (QR/barcode decode path). Header is the standard /usr/include/zbar.h.
find_library(ZBAR_LIB zbar REQUIRED)

# GStreamer (native A/V SRT receiver). Optional: if the dev packages are not
# installed the GUI still builds, just without the C920 A/V stream + its audio.
#   sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST
  gstreamer-1.0
  gstreamer-app-1.0
  gstreamer-video-1.0)

# Vosk (speech-to-text). Header + lib are installed manually under /usr/local
# (see gui/assets/audio/README.md and the legacy gui README).
find_library(VOSK_LIB vosk HINTS /usr/local/lib REQUIRED)
find_path(VOSK_INCLUDE_DIR vosk_api.h HINTS /usr/local/include REQUIRED)

set(GUI_SOURCES
  src/gui.cpp
  src/main_window.cpp
  src/app_settings.cpp
  src/camera_hub.cpp
  src/source_manager.cpp
  src/video_widget.cpp
  src/video_panel.cpp
  src/dashboard_panel.cpp
  src/odometry_panel.cpp
  src/speech_processor.cpp
  src/urdf_viewer.cpp
  src/arm_pose_panel.cpp
  src/settings_dialog.cpp
  src/ppm_calib_dialog.cpp
  src/map_window.cpp
  src/systems_window.cpp
  src/perception_panel.cpp
  src/maps_panel.cpp
  src/routine_programmer.cpp
  src/filters.cpp
  # Headers listed so AUTOMOC processes the Q_OBJECT classes.
  include/gui/main_window.hpp
  include/gui/source_manager.hpp
  include/gui/video_widget.hpp
  include/gui/video_panel.hpp
  include/gui/dashboard_panel.hpp
  include/gui/odometry_panel.hpp
  include/gui/speech_processor.hpp
  include/gui/urdf_viewer.hpp
  include/gui/arm_pose_panel.hpp
  include/gui/digital_twin_panel.hpp
  include/gui/settings_dialog.hpp
  include/gui/ppm_calib_dialog.hpp
  include/gui/map_window.hpp
  include/gui/systems_window.hpp
  include/gui/perception_panel.hpp
  include/gui/maps_panel.hpp
  include/gui/routine_programmer.hpp
)
if(GST_FOUND)
  list(APPEND GUI_SOURCES src/gst_av_stream.cpp)
endif()

add_executable(gui ${GUI_SOURCES})

target_include_directories(gui PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${VOSK_INCLUDE_DIR}
  ${ASSIMP_INCLUDE_DIRS}
  ${ONNXRUNTIME_INCLUDE_DIR}
  ${OCTOMAP_INCLUDE_DIRS}
)

target_compile_features(gui PUBLIC cxx_std_17)

target_link_libraries(gui
  Qt6::Widgets
  Qt6::OpenGL
  Qt6::OpenGLWidgets
  ${OpenCV_LIBS}
  ${VOSK_LIB}
  ${ASSIMP_LIBRARIES}
  ${ONNXRUNTIME_LIB}
  ${ZBAR_LIB}
  ${OCTOMAP_LIBRARIES}
)

if(GST_FOUND)
  target_compile_definitions(gui PRIVATE HAVE_GSTREAMER)
  target_include_directories(gui PRIVATE ${GST_INCLUDE_DIRS})
  target_link_directories(gui PRIVATE ${GST_LIBRARY_DIRS})
  target_link_libraries(gui ${GST_LIBRARIES})
else()
  message(WARNING
    "GStreamer dev packages not found — the C920 A/V SRT stream + its Opus audio "
    "(speech transcription feed) will be DISABLED. Install with:\n"
    "  sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev")
endif()

ament_target_dependencies(gui
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  nav_msgs
  nav2_msgs
  octomap_msgs
  tf2
  tf2_ros
  rescue_interfaces
  urdf
  ament_index_cpp
)

install(TARGETS gui
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/c920_srt_stream.sh
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch assets
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
