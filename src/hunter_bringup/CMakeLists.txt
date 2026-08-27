cmake_minimum_required(VERSION 3.8)
project(hunter_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  urdf
  behavior_trees
  scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
