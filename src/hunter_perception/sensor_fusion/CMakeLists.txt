cmake_minimum_required(VERSION 3.8)
project(sensor_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(hunter_msgs REQUIRED)

add_executable(sensor_fusion_node
  src/sensor_fusion.cpp
  src/main.cpp
)
target_include_directories(sensor_fusion_node PRIVATE include)
target_compile_features(sensor_fusion_node PUBLIC cxx_std_17)
ament_target_dependencies(sensor_fusion_node
  rclcpp
  nav_msgs
  hunter_msgs
)

install(TARGETS sensor_fusion_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
