cmake_minimum_required(VERSION 3.5)
project(rw_common_util)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(target_rw_common_util SHARED
  DIRECTORY src
)

target_include_directories(target_rw_common_util
  SYSTEM PUBLIC
)

rclcpp_components_register_node(target_rw_common_util
  PLUGIN "rw_common_util::RWCommonUtilNode"
  EXECUTABLE rw_common_util_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)