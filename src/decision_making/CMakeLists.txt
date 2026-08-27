cmake_minimum_required(VERSION 3.8)
project(decision_making)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hunter_msgs REQUIRED)

add_executable(decision_making_node
  src/decision_making.cpp
  src/main.cpp
)
target_include_directories(decision_making_node PRIVATE include)
target_compile_features(decision_making_node PUBLIC cxx_std_17)
ament_target_dependencies(decision_making_node
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  std_msgs
  hunter_msgs
)

install(TARGETS decision_making_node
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
