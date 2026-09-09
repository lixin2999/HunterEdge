cmake_minimum_required(VERSION 3.8)
project(hunter_safety)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(action_msgs REQUIRED)

add_executable(safety_guard src/safety_guard.cpp)
target_include_directories(safety_guard PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(safety_guard PUBLIC c_std_17 cxx_std_17)
ament_target_dependencies(safety_guard
  rclcpp
  rclcpp_action
  geometry_msgs
  sensor_msgs
  std_msgs
  nav_msgs
  nav2_msgs
  action_msgs
)

install(TARGETS safety_guard
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
