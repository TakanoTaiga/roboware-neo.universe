cmake_minimum_required(VERSION 3.5)
project(joy_to_twist)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(target_joy_to_twist SHARED
  DIRECTORY src
)

target_include_directories(target_joy_to_twist
  SYSTEM PUBLIC
)

rclcpp_components_register_node(target_joy_to_twist
  PLUGIN "joy_to_twist::JoyToTwistNode"
  EXECUTABLE joy_to_twist_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)