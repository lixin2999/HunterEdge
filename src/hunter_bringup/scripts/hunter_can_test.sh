#!/usr/bin/env bash
# hunter_can_test.sh — CAN 通信测试工具（文档 20.3 / 11.3）
set -euo pipefail

CAN_IF="${CAN_IF:-can0}"
BITRATE="${BITRATE:-500000}"

usage() {
  echo "用法: $0 {up|down|dump|send|test}"
  echo "  up      配置并启用 CAN 接口（$CAN_IF @ ${BITRATE}bps）"
  echo "  down    关闭 CAN 接口"
  echo "  dump    监听 CAN 报文（candump）"
  echo "  send    发送 0x111 停车指令"
  echo "  test    检测底盘反馈报文（0x211/0x221，文档附录A）"
  echo ""
  echo "环境变量: CAN_IF（默认 can0）、BITRATE（默认 500000）"
}

if [ "$#" -lt 1 ]; then
  usage
  exit 1
fi

case "$1" in
  up)
    if ! ip link show "$CAN_IF" >/dev/null 2>&1; then
      echo "接口 $CAN_IF 不存在，尝试加载 gs_usb（仅 USB-CAN 适配器需要；"
      echo "Jetson 板载 mttcan 控制器内核自带，无需此步）..."
      sudo modprobe gs_usb || echo "gs_usb 加载失败（若为板载 CAN 控制器可忽略）"
    fi
    echo "配置 $CAN_IF @ ${BITRATE}bps ..."
    sudo ip link set "$CAN_IF" type can bitrate "$BITRATE" || true
    sudo ip link set "$CAN_IF" up
    echo "--- 接口状态 ---"
    ip -details link show "$CAN_IF"
    ;;
  down)
    sudo ip link set "$CAN_IF" down
    echo "$CAN_IF 已关闭"
    ;;
  dump)
    if ! command -v candump >/dev/null 2>&1; then
      echo "错误: candump 不可用（安装 can-utils）"
      exit 1
    fi
    echo "监听 $CAN_IF（Ctrl+C 退出）..."
    candump "$CAN_IF"
    ;;
  send)
    if ! command -v cansend >/dev/null 2>&1; then
      echo "错误: cansend 不可用（安装 can-utils）"
      exit 1
    fi
    # 0x111 运动控制（8 字节全 0 = 停车，文档附录A）
    cansend "$CAN_IF" "111#0000000000000000"
    echo "已发送 0x111 停车指令"
    ;;
  test)
    if ! command -v candump >/dev/null 2>&1; then
      echo "错误: candump 不可用（安装 can-utils）"
      exit 1
    fi
    echo "检测底盘反馈报文（5 秒，文档 0x211/0x221）..."
    out=$(timeout 5 candump "$CAN_IF" 2>/dev/null | grep -E "211|221" | head -10 || true)
    if [ -n "$out" ]; then
      echo "$out"
      echo "✓ 检测到底盘反馈报文"
    else
      echo "✗ 未检测到反馈报文（检查接线/波特率/接口）"
    fi
    ;;
  *)
    usage
    exit 1
    ;;
esac
