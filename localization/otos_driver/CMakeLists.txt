cmake_minimum_required(VERSION 3.5)
project(otos_driver)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(target_otos_driver SHARED
  DIRECTORY src
)

target_include_directories(target_otos_driver
  SYSTEM PUBLIC
)

rclcpp_components_register_node(target_otos_driver
  PLUGIN "otos_driver::otos_driver_node"
  EXECUTABLE otos_driver_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)