cmake_minimum_required(VERSION 3.8)
project(lidar_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(hunter_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

add_executable(lidar_perception_node
  src/lidar_perception.cpp
  src/tracker.cpp
  src/main.cpp
)
target_include_directories(lidar_perception_node PRIVATE include)
target_compile_features(lidar_perception_node PUBLIC cxx_std_17)
ament_target_dependencies(lidar_perception_node
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  visualization_msgs
  hunter_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
)

install(TARGETS lidar_perception_node
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
