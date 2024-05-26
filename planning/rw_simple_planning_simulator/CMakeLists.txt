cmake_minimum_required(VERSION 3.5)
project(rw_simple_planning_simulator)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(target_rw_simple_planning_simulator SHARED
  DIRECTORY src
)

target_include_directories(target_rw_simple_planning_simulator
  SYSTEM PUBLIC
)

rclcpp_components_register_node(target_rw_simple_planning_simulator
  PLUGIN "rw_simple_planning_simulator::RWSimplePlanningSimulatorNode"
  EXECUTABLE rw_simple_planning_simulator_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)