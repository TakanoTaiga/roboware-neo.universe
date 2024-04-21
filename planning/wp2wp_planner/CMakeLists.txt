cmake_minimum_required(VERSION 3.5)
project(wp2wp_planner)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(target_wp2wp_planner SHARED
  DIRECTORY src
)

target_include_directories(target_wp2wp_planner
  SYSTEM PUBLIC
)

rclcpp_components_register_node(target_wp2wp_planner
  PLUGIN "wp2wp_planner::WP2WPPlannerNode"
  EXECUTABLE wp2wp_planner_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)