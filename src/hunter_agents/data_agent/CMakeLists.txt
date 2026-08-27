cmake_minimum_required(VERSION 3.8)
project(data_agent)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(hunter_msgs REQUIRED)

# Kafka (librdkafka C++) 与 SQLite3 系统库
find_library(RDKAFKA_LIB rdkafka++)
find_library(SQLITE3_LIB sqlite3)

add_executable(data_agent_node
  src/data_agent.cpp
  src/main.cpp
)
target_include_directories(data_agent_node PRIVATE include)
target_compile_features(data_agent_node PUBLIC cxx_std_17)
ament_target_dependencies(data_agent_node
  rclcpp
  sensor_msgs
  nav_msgs
  hunter_msgs
)
target_link_libraries(data_agent_node
  ${RDKAFKA_LIB}
  ${SQLITE3_LIB}
)

install(TARGETS data_agent_node
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
