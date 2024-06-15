cmake_minimum_required(VERSION 3.5)
project(mcu_ros_driver)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(target_mcu_ros_driver SHARED
  DIRECTORY src
)

target_include_directories(target_mcu_ros_driver
  SYSTEM PUBLIC
)

rclcpp_components_register_node(target_mcu_ros_driver
  PLUGIN "mcu_ros_driver::mcu_ros_driver_node"
  EXECUTABLE mcu_ros_driver_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)