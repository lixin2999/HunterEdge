cmake_minimum_required(VERSION 3.8)
project(hunter_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  rviz
  urdf
  behavior_trees
  scripts
  systemd
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

ament_package()
