cmake_minimum_required(VERSION 3.5)
project(occupancy_grid_map_maker)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(target_occupancy_grid_map_maker SHARED
  DIRECTORY src
)

target_include_directories(target_occupancy_grid_map_maker
  SYSTEM PUBLIC
)

rclcpp_components_register_node(target_occupancy_grid_map_maker
  PLUGIN "occupancy_grid_map_maker_node::OccupancyGridMapMakerNode"
  EXECUTABLE occupancy_grid_map_maker_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)