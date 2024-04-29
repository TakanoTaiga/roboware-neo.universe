cmake_minimum_required(VERSION 3.5)
project(path_follower)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(target_path_follower SHARED
  DIRECTORY src
)

target_include_directories(target_path_follower
  SYSTEM PUBLIC
)

rclcpp_components_register_node(target_path_follower
  PLUGIN "path_follower::PathFollowerNode"
  EXECUTABLE path_follower_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)