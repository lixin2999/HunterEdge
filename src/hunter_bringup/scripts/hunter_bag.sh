#!/usr/bin/env bash
# hunter_bag.sh — ROS Bag 录制/回放工具（文档 20.3 / 14.4）
set -euo pipefail

BAG_DIR="${BAG_DIR:-/data/rosbag}"
# 录制话题列表（文档 14.4.2）
TOPICS=(
  /lidar_points
  /camera/color/image_raw
  /camera/depth/image_rect_raw
  /imu/data
  /chassis/state
  /localization/odom
  /perception/fused_objects
  /planning/trajectory
  /control/command
  /tf
)

usage() {
  echo "用法: $0 {record|play|info} [参数]"
  echo "  record [name]  录制默认话题（name 可选，默认时间戳）"
  echo "  play <bag>     回放 bag"
  echo "  info <bag>     查看 bag 信息"
  echo "  list           列出已有 bag"
  echo ""
  echo "环境变量: BAG_DIR（默认 /data/rosbag）"
}

if [ "$#" -lt 1 ]; then
  usage
  exit 1
fi

case "$1" in
  record)
    mkdir -p "$BAG_DIR"
    name="${2:-$(date +%Y%m%d_%H%M%S)}"
    output="$BAG_DIR/$name"
    echo "录制到: $output"
    echo "话题: ${TOPICS[*]}"
    ros2 bag record -o "$output" "${TOPICS[@]}"
    ;;
  play)
    if [ "$#" -lt 2 ]; then
      echo "请指定 bag 路径"
      usage
      exit 1
    fi
    echo "回放: $2"
    ros2 bag play "$2"
    ;;
  info)
    if [ "$#" -lt 2 ]; then
      echo "请指定 bag 路径"
      usage
      exit 1
    fi
    ros2 bag info "$2"
    ;;
  list)
    echo "--- $BAG_DIR 下的 bag ---"
    if [ -d "$BAG_DIR" ]; then
      ls -lt "$BAG_DIR"
    else
      echo "  (目录不存在)"
    fi
    ;;
  *)
    usage
    exit 1
    ;;
esac
