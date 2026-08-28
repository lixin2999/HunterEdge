cmake_minimum_required(VERSION 3.8)
project(hunter_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Placeholder for actual hunter_utils implementation
# This package contains common utility functions for HUNTER autonomous vehicle system

install(DIRECTORY include/
  DESTINATION include/hunter_utils
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE)

ament_package()