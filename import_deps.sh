#!/usr/bin/env bash
# import_deps.sh — 导入第三方依赖包（文档 4.2 vendor 包）
# 在 Ubuntu + ROS2 Humble 环境执行（GitHub 网络正常）
set -euo pipefail

# 工作空间 src 目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$SCRIPT_DIR/src"
mkdir -p "$SRC_DIR"
cd "$SRC_DIR"

echo "=============================================="
echo "导入 AgileX 底盘驱动（hunter_ros2 / ugv_sdk）"
echo "=============================================="
[ -d ugv_sdk ] || git clone https://github.com/agilexrobotics/ugv_sdk.git
[ -d hunter_ros2 ] || git clone -b humble https://github.com/agilexrobotics/hunter_ros2.git

echo "=============================================="
echo "导入传感器驱动（rslidar_sdk / realsense-ros）"
echo "=============================================="
[ -d rslidar_sdk ] || git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git
[ -d realsense-ros ] || git clone -b ros2-development https://github.com/IntelRealSense/realsense-ros.git

echo "=============================================="
echo "导入定位/融合（fast_lio2 / robot_localization）"
echo "=============================================="
# FAST-LIO2（hku-mars 官方，ROS2 分支）
[ -d fast_lio2 ] || git clone -b ROS2 https://github.com/hku-mars/FAST_LIO.git fast_lio2
# robot_localization（humble-devel 分支）
[ -d robot_localization ] || git clone -b humble-devel https://github.com/cra-ros-pkg/robot_localization.git

echo "=============================================="
echo "导入导航栈（navigation2）"
echo "=============================================="
[ -d navigation2 ] || git clone -b humble https://github.com/ros-navigation/navigation2.git

echo "=============================================="
echo "导入 YOLO TensorRT 封装（可选，yolo_trt_ros）"
echo "=============================================="
# 注：yolo_trt_ros 为可选包，仓库地址待确认，按需补充：
[ -d yolo_trt_ros ] || git clone https://github.com/linClubs/YOLOv8-ROS-TensorRT.git

echo ""
echo "第三方依赖包导入完成，执行构建："
echo "  cd ~/HunterEdge && colcon build --symlink-install"
