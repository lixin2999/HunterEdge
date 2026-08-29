cmake_minimum_required(VERSION 3.8)
project(vision_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(hunter_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# CUDA / TensorRT（NVIDIA Jetson JetPack）
find_package(CUDA REQUIRED)
find_library(TENSORRT_NVINFER nvinfer)
find_library(TENSORRT_NVINFER_PLUGIN nvinfer_plugin)

add_executable(vision_perception_node
  src/vision_perception.cpp
  src/tensorrt_inference.cpp
  src/main.cpp
)
target_include_directories(vision_perception_node PRIVATE include)
target_compile_features(vision_perception_node PUBLIC cxx_std_17)
ament_target_dependencies(vision_perception_node
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  nav_msgs
  cv_bridge
  hunter_msgs
)
target_link_libraries(vision_perception_node
  ${OpenCV_LIBS}
  ${TENSORRT_NVINFER}
  ${TENSORRT_NVINFER_PLUGIN}
  cudart
)

install(TARGETS vision_perception_node
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
