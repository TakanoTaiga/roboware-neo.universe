cmake_minimum_required(VERSION 3.5)
project(mission_manager)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(target_mission_manager SHARED
  DIRECTORY src
)

target_include_directories(target_mission_manager
  SYSTEM PUBLIC
)

rclcpp_components_register_node(target_mission_manager
  PLUGIN "mission_manager::MissionManagerNode"
  EXECUTABLE mission_manager_node
)

rclcpp_components_register_node(target_mission_manager
  PLUGIN "mission_manager::ARMarkerStrategyNode"
  EXECUTABLE ar_marker_strategy_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)