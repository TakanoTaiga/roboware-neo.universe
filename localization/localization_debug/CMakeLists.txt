cmake_minimum_required(VERSION 3.5)
project(localization_debug)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(target_localization_debug SHARED
  DIRECTORY src
)

target_include_directories(target_localization_debug
  SYSTEM PUBLIC
)

rclcpp_components_register_node(target_localization_debug
  PLUGIN "localization_debug::localizationDebugNode"
  EXECUTABLE localization_debug_node
)

rclcpp_components_register_node(target_localization_debug
  PLUGIN "localization_debug::quat_to_euler_node"
  EXECUTABLE quat_to_euler_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)