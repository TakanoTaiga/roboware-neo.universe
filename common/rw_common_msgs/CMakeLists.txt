cmake_minimum_required(VERSION 3.14)
project(rw_common_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

file(GLOB _all_msg_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" msg/*)
message(STATUS "Using msg files: ${_all_msg_files}")
if(NOT _all_msg_files)
message(FATAL_ERROR "No msg files found")
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  ${_all_msg_files}
  DEPENDENCIES
    geometry_msgs
)

ament_auto_package()