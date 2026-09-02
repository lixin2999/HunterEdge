#!/usr/bin/env bash
# import_vendor.sh — 导入第三方依赖包（文档 4.2 vendor 包 + 11.6 安装配置）
# 用法: ./import_vendor.sh [工作空间src路径]（默认 ~/HunterEdge/src）
set -euo pipefail

WS_SRC="${1:-$HOME/HunterEdge/src}"

log() { echo "[vendor] $*"; }

if [ ! -d "$WS_SRC" ]; then
  echo "错误: 工作空间 src 目录不存在: $WS_SRC"
  exit 1
fi

# ============ AgileX 官方（文档 11.6：ugv_sdk + hunter_ros2） ============
log "导入 AgileX 官方驱动..."
if [ -d "$WS_SRC/ugv_sdk/.git" ]; then
  log "  已存在: ugv_sdk"
else
  git clone https://github.com/agilexrobotics/ugv_sdk.git "$WS_SRC/ugv_sdk"
fi

if [ -d "$WS_SRC/hunter_ros2/.git" ]; then
  log "  已存在: hunter_ros2"
else
  git clone -b humble https://github.com/agilexrobotics/hunter_ros2.git "$WS_SRC/hunter_ros2"
fi

# hunter_ros2 自带 hunter_msgs，与 hunter_common/hunter_msgs 同名。
# 5 条底盘消息已并入自定义包；跳过 vendor 包，避免 colcon Duplicate package names。
if [ -d "$WS_SRC/hunter_ros2/hunter_msgs" ]; then
  touch "$WS_SRC/hunter_ros2/hunter_msgs/COLCON_IGNORE"
  log "  已忽略重复包: hunter_ros2/hunter_msgs"
fi

# ============ RoboSense LiDAR（rslidar_sdk） ============
log "导入 rslidar_sdk..."
if [ -d "$WS_SRC/rslidar_sdk/.git" ]; then
  log "  已存在: rslidar_sdk"
else
  git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git "$WS_SRC/rslidar_sdk"
  git clone https://github.com/RoboSense-LiDAR/rs_driver.git "$WS_SRC/rslidar_sdk/src/rs_driver"
fi

# ============ Intel RealSense（realsense-ros） ============
log "导入 realsense-ros..."
if [ -d "$WS_SRC/realsense-ros/.git" ]; then
  log "  已存在: realsense-ros"
else
  git clone https://github.com/realsenseai/realsense-ros.git "$WS_SRC/realsense-ros"
fi

# ============ FAST-LIO2（港大 hku-mars 雷达惯导里程计） ============
log "导入 fast_lio2..."
if [ -d "$WS_SRC/fast_lio2/.git" ]; then
  log "  已存在: fast_lio2"
else
  # ROS2 版本（若仓库默认分支为 ROS1，可改用 -b ros2 分支）
  git clone -b ROS2 https://github.com/hku-mars/FAST_LIO.git "$WS_SRC/fast_lio2"
  # 递归拉取 ikd-Tree 子模块（ikd_Tree.cpp 编译必须）
  git -C "$WS_SRC/fast_lio2" submodule update --init --recursive
  log "  已初始化子模块: fast_lio2/include/ikd-Tree"
  git clone https://github.com/Livox-SDK/livox_ros_driver2.git "$WS_SRC/livox_ros_driver2"
  # CMakeLists.txt 将在下方统一修改，使 SDK 缺失时只跳过驱动节点编译，消息接口照常生成。
fi

# fast_lio2 源码实际使用了 livox_ros_driver2/msg/custom_msg.hpp，
# livox_ros_driver2 必须编译以生成消息头文件。
# CMakeLists.txt 已在源码侧修改：find_library/find_path 去掉 REQUIRED，
# 驱动编译块用 if(LIVOX_LIDAR_SDK_LIBRARY AND LIVOX_LIDAR_SDK_INCLUDE_DIR) 守卫，
# SDK 缺失时只跳过驱动节点编译，消息接口照常生成。

# livox_ros_driver2 不携带 package.xml（ROS1/ROS2 分开存放），colcon 编译消息接口时需要它。
# 将 package_ROS2.xml 复制为 package.xml（幂等：已存在则跳过）。
if [ -f "$WS_SRC/livox_ros_driver2/package_ROS2.xml" ] && \
   [ ! -f "$WS_SRC/livox_ros_driver2/package.xml" ]; then
  cp "$WS_SRC/livox_ros_driver2/package_ROS2.xml" "$WS_SRC/livox_ros_driver2/package.xml"
  log "  已创建: livox_ros_driver2/package.xml（来源: package_ROS2.xml）"
fi

# ============ robot_localization（EKF 传感器融合） ============
log "导入 robot_localization..."
if [ -d "$WS_SRC/robot_localization/.git" ]; then
  log "  已存在: robot_localization"
else
  git clone -b humble-devel https://github.com/cra-ros-pkg/robot_localization.git "$WS_SRC/robot_localization"
fi

# ============ navigation2（Nav2 导航栈） ============
# 若环境中已通过 apt 安装 Nav2，则跳过源码导入/编译（避免与二进制包同名冲突）
if dpkg -s ros-humble-navigation2 >/dev/null 2>&1 || ros2 pkg prefix navigation2 >/dev/null 2>&1; then
  log "检测到 apt 已安装 Nav2，跳过 navigation2 源码导入/编译"
else
  log "导入 navigation2..."
  if [ -d "$WS_SRC/navigation2/.git" ]; then
    log "  已存在: navigation2"
  else
    git clone -b humble https://github.com/ros-navigation/navigation2.git "$WS_SRC/navigation2"
  fi
fi

# ============ yolo_trt_ros（YOLO TensorRT 推理） ============
log "导入 yolo_trt_ros..."
if [ -d "$WS_SRC/yolo_trt_ros/.git" ]; then
  log "  已存在: yolo_trt_ros"
else
  # 根据实际仓库地址调整（本行失败不影响整体导入）
  git clone https://github.com/wyf-yfw/TensorRT_YOLO_ROS2.git "$WS_SRC/yolo_trt_ros"
fi

 # 先恢复误禁用的 package.xml（幂等；当前上游无 src/jetson）
find "$WS_SRC/yolo_trt_ros" -name 'package.xml.disabled' | while read -r f; do
  mv "$f" "${f%.disabled}"
  log "  已恢复: ${f#$WS_SRC/yolo_trt_ros/}"
done

if [ -d "$WS_SRC/yolo_trt_ros/src/jetson" ]; then
  log "yolo_trt_ros：检测到 src/jetson，禁用其余同名包"
  find "$WS_SRC/yolo_trt_ros" -name package.xml | while read -r f; do
    case "$f" in
      *src/jetson*) ;;
      *)
        mv "$f" "$f.disabled"
        log "  已禁用: ${f#$WS_SRC/yolo_trt_ros/}"
        ;;
    esac
  done
else
  log "yolo_trt_ros：无 src/jetson，保持 tensorrt_yolo_core / tensorrt_yolo_msg 不变"
fi

log "第三方依赖包导入完成"
echo ""
echo "提示：navigation2 / robot_localization / realsense 也可用 apt 安装（更快更稳）："
echo "  sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup"
echo "  sudo apt install -y ros-humble-robot-localization"
echo "  sudo apt install -y ros-humble-realsense2-camera"
echo ""
echo "编译："
echo "  cd $(dirname "$WS_SRC") && colcon build --symlink-install"
