cmake_minimum_required(VERSION 3.5)
project(thrown_object_estimator)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(target_thrown_object_estimator SHARED
  DIRECTORY src
)

target_include_directories(target_thrown_object_estimator
  SYSTEM PUBLIC
)

rclcpp_components_register_node(target_thrown_object_estimator
  PLUGIN "thrown_object_estimator::thrown_object_estimator_node"
  EXECUTABLE thrown_object_estimator_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)