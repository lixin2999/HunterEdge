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

# 检测 OpenCV 是否包含 CUDA 模块（cudaimgproc / cudawarping）
# 有些环境（无 GPU 或仅安装 CPU-only OpenCV）不包含这些模块
set(OPENCV_CUDA_AVAILABLE FALSE)
if(OpenCV_FOUND)
  foreach(_mod IN ITEMS opencv_cudaimgproc opencv_cudawarping)
    if(_mod IN_LIST OpenCV_LIBS OR TARGET ${_mod})
      set(OPENCV_CUDA_AVAILABLE TRUE)
    endif()
  endforeach()
  # 额外检查：确认头文件实际存在（防止库存在但头文件路径缺失导致编译失败）
  if(OPENCV_CUDA_AVAILABLE)
    # find_file 需要在 hint 目录的子目录下找，所以把 opencv4/opencv2 目录都加进来
    set(_opencv_hint_dirs "")
    foreach(_inc_dir IN LISTS OpenCV_INCLUDE_DIRS)
      list(APPEND _opencv_hint_dirs
        "${_inc_dir}"
        "${_inc_dir}/opencv2"
      )
    endforeach()
    find_file(_cudaimgproc_header
      NAMES "cudaimgproc.hpp"
      HINTS ${_opencv_hint_dirs}
            /usr/local/include/opencv4/opencv2
            /usr/include/opencv4/opencv2
      NO_DEFAULT_PATH
    )
    if(NOT _cudaimgproc_header)
      message(STATUS "vision_perception: opencv_cudaimgproc target found but header missing "
        "in OpenCV_INCLUDE_DIRS=${OpenCV_INCLUDE_DIRS} — disabling GPU path")
      set(OPENCV_CUDA_AVAILABLE FALSE)
    endif()
    unset(_cudaimgproc_header CACHE)
  endif()
endif()
if(OPENCV_CUDA_AVAILABLE)
  message(STATUS "vision_perception: OpenCV CUDA modules found — GPU preprocessing enabled")
else()
  message(STATUS "vision_perception: OpenCV CUDA modules NOT found — falling back to CPU preprocessing")
endif()

# CUDA / TensorRT（NVIDIA Jetson JetPack）
# JetPack 安装路径为 /usr/local/cuda-x.y，CMake find_package(CUDA) 依赖符号链接
# /usr/local/cuda，若不存在则 CUDA_INCLUDE_DIRS 为空；此处手动补充 fallback。
find_package(CUDA QUIET)
if(NOT CUDA_FOUND OR NOT CUDA_INCLUDE_DIRS)
  # 搜索 /usr/local/cuda-* 目录，取最新版本
  file(GLOB _cuda_dirs "/usr/local/cuda-*/targets/aarch64-linux/include"
                       "/usr/local/cuda-*/include")
  if(_cuda_dirs)
    list(SORT _cuda_dirs ORDER DESCENDING)
    list(GET _cuda_dirs 0 CUDA_INCLUDE_DIRS)
    message(STATUS "vision_perception: CUDA headers fallback -> ${CUDA_INCLUDE_DIRS}")
  else()
    message(FATAL_ERROR "vision_perception: Cannot find CUDA headers. "
      "Install CUDA or create symlink: sudo ln -s /usr/local/cuda-12.6 /usr/local/cuda")
  endif()
  # 同步查找 cudart 库
  file(GLOB _cuda_lib_dirs "/usr/local/cuda-*/targets/aarch64-linux/lib"
                           "/usr/local/cuda-*/lib64")
  if(_cuda_lib_dirs)
    list(SORT _cuda_lib_dirs ORDER DESCENDING)
    list(GET _cuda_lib_dirs 0 _cuda_lib_dir)
    find_library(CUDA_CUDART_LIBRARY cudart PATHS "${_cuda_lib_dir}" NO_DEFAULT_PATH)
  endif()
endif()
if(NOT CUDA_CUDART_LIBRARY)
  find_library(CUDA_CUDART_LIBRARY cudart)
endif()
find_library(TENSORRT_NVINFER nvinfer)
find_library(TENSORRT_NVINFER_PLUGIN nvinfer_plugin)

add_executable(vision_perception_node
  src/vision_perception.cpp
  src/tensorrt_inference.cpp
  src/main.cpp
)
target_include_directories(vision_perception_node PRIVATE
  include
  ${CUDA_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
target_compile_features(vision_perception_node PUBLIC cxx_std_17)
# 将 OpenCV CUDA 可用性传递给源码
if(OPENCV_CUDA_AVAILABLE)
  target_compile_definitions(vision_perception_node PRIVATE HAVE_OPENCV_CUDA)
endif()
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
  ${CUDA_CUDART_LIBRARY}
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
