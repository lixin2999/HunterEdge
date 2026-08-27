cmake_minimum_required(VERSION 3.8)
project(hunter_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
  "msg/DetectedObject.msg"
  "msg/DetectedObjectArray.msg"
  "msg/TrajectoryPoint.msg"
  "msg/Trajectory.msg"
  "msg/ChassisState.msg"
  "msg/ChassisCommand.msg"
  "msg/BehaviorState.msg"
  "msg/SystemHealth.msg"
  "msg/HunterStatus.msg"
  "msg/HunterActuatorState.msg"
  "msg/HunterLightCmd.msg"
  "msg/HunterLightState.msg"
  "msg/HunterRCState.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
