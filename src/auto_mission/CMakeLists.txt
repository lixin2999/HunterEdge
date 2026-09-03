cmake_minimum_required(VERSION 3.8)
project(auto_mission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- 依赖包 ----
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(hunter_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(DEPS
  rclcpp
  rclcpp_action
  rclcpp_components
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  nav2_msgs
  hunter_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ============================================================
# C++ 可执行文件：auto_mission_node
# ============================================================
add_executable(auto_mission_node
  src/auto_mission_node.cpp
  src/main.cpp
)
target_include_directories(auto_mission_node PRIVATE include)
target_compile_features(auto_mission_node PUBLIC cxx_std_17)
ament_target_dependencies(auto_mission_node ${DEPS})

install(TARGETS auto_mission_node
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================================
# Python 工具节点：waypoint_recorder / pcd_to_map / fast_lio2_param_injector
#
# 安装为 lib/<pkg>/<name>（无 .py 后缀），
# 使 ros2 run auto_mission waypoint_recorder 等命令可用。
# ============================================================
install(PROGRAMS
  scripts/waypoint_recorder.py
  RENAME waypoint_recorder
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/pcd_to_map.py
  RENAME pcd_to_map
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/fast_lio2_param_injector.py
  RENAME fast_lio2_param_injector
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================================
# 资源目录
# ============================================================
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
