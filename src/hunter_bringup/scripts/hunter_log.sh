#!/usr/bin/env bash
# hunter_log.sh — 查看和导出系统日志（文档 20.3 / 15.5）
set -euo pipefail

ROS_LOG_DIR="${ROS_LOG_DIR:-$HOME/.ros/log}"
SYS_LOG_DIR="/var/log/hunter"
EXPORT_DIR="${1:-$HOME/hunter_logs_$(date +%Y%m%d_%H%M%S)}"

usage() {
  echo "用法: $0 [导出目录]"
  echo "  不带参数  查看最新日志"
  echo "  带参数    导出日志到指定目录（默认 ~/hunter_logs_时间戳，并打包 tar.gz）"
}

if [ "$#" -gt 1 ]; then
  usage
  exit 1
fi

if [ "$#" -eq 1 ]; then
  # ========== 导出模式 ==========
  echo "导出日志到: $EXPORT_DIR"
  mkdir -p "$EXPORT_DIR"

  # ROS 日志（文档 15.5：~/.ros/log/）
  if [ -d "$ROS_LOG_DIR" ]; then
    latest=$(ls -t "$ROS_LOG_DIR" | head -1)
    if [ -n "$latest" ]; then
      cp -r "$ROS_LOG_DIR/$latest" "$EXPORT_DIR/ros_log_$latest"
      echo "  已导出 ROS 日志: $latest"
    fi
  fi

  # 系统日志（文档 15.5：/var/log/hunter/）
  if [ -d "$SYS_LOG_DIR" ]; then
    cp -r "$SYS_LOG_DIR" "$EXPORT_DIR/system_log"
    echo "  已导出系统日志: $SYS_LOG_DIR"
  fi

  # 打包
  tar -czf "${EXPORT_DIR}.tar.gz" -C "$(dirname "$EXPORT_DIR")" "$(basename "$EXPORT_DIR")"
  echo "已打包: ${EXPORT_DIR}.tar.gz"
else
  # ========== 查看模式 ==========
  echo "--- 最新 ROS 日志目录 ---"
  if [ -d "$ROS_LOG_DIR" ]; then
    latest=$(ls -t "$ROS_LOG_DIR" | head -1)
    echo "  $ROS_LOG_DIR/$latest"
    echo ""
    echo "--- 日志文件 ---"
    find "$ROS_LOG_DIR/$latest" -name "*.log" -type f 2>/dev/null | head -5
  else
    echo "  (无 ROS 日志目录)"
  fi

  echo ""
  echo "--- 系统日志 ---"
  if [ -d "$SYS_LOG_DIR" ]; then
    ls -t "$SYS_LOG_DIR" | head -10
  else
    echo "  (无系统日志目录)"
  fi
fi
