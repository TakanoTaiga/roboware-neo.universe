cmake_minimum_required(VERSION 3.8)
project(rw_common_util)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(rw_common_util SHARED DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src)

ament_auto_package()
