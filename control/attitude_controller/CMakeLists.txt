cmake_minimum_required(VERSION 3.5)
project(attitude_controller)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(target_attitude_controller SHARED
  DIRECTORY src
)

target_include_directories(target_attitude_controller
  SYSTEM PUBLIC
)

rclcpp_components_register_node(target_attitude_controller
  PLUGIN "attitude_controller::attitude_controller_node"
  EXECUTABLE attitude_controller_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)