cmake_minimum_required(VERSION 3.8)
project(ch10x_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(ch10x_driver_node
  src/ch10x_driver.cpp
  src/main.cpp
)
target_include_directories(ch10x_driver_node PRIVATE include)
target_compile_features(ch10x_driver_node PUBLIC cxx_std_17)
ament_target_dependencies(ch10x_driver_node
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
)

install(TARGETS ch10x_driver_node
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
