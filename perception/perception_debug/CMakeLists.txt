cmake_minimum_required(VERSION 3.5)
project(perception_debug)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(target_perception_debug SHARED
  DIRECTORY src
)

target_include_directories(target_perception_debug
  SYSTEM PUBLIC
)

rclcpp_components_register_node(target_perception_debug
  PLUGIN "perception_debug::bbox2foxgloveNode"
  EXECUTABLE perception_debug_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)