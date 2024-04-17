cmake_minimum_required(VERSION 3.5)
project(detect_ar_marker)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(target_detect_ar_marker SHARED
  DIRECTORY src
)

target_include_directories(target_detect_ar_marker
  SYSTEM PUBLIC
)

rclcpp_components_register_node(target_detect_ar_marker
  PLUGIN "detect_ar_marker::detectARMakerNode"
  EXECUTABLE detect_ar_marker_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)