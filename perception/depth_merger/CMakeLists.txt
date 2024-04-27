cmake_minimum_required(VERSION 3.5)
project(depth_merger)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(target_depth_merger SHARED
  DIRECTORY src
)

target_include_directories(target_depth_merger
  SYSTEM PUBLIC
)

rclcpp_components_register_node(target_depth_merger
  PLUGIN "depth_merge_node::DepthMergerNode"
  EXECUTABLE depth_merger_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)