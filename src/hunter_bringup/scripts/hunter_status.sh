#!/usr/bin/env bash
# hunter_status.sh — 查看系统状态、节点存活、资源占用（文档 20.3）
set -euo pipefail

echo "========== HUNTER 系统状态 =========="

# 1. ROS2 节点列表
echo ""
echo "--- ROS2 节点 ---"
if command -v ros2 >/dev/null 2>&1; then
  ros2 node list 2>/dev/null || echo "  (ROS2 daemon 未运行)"
else
  echo "  (ros2 命令不可用)"
fi

# 2. 关键节点存活检查
echo ""
echo "--- 关键节点存活检查 ---"
critical_nodes=(
  lidar_perception vision_perception sensor_fusion
  fast_lio2 ekf_filter_node hunter_ros2
  decision_making health_monitor data_agent command_agent
)
nodes=$(ros2 node list 2>/dev/null || true)
for n in "${critical_nodes[@]}"; do
  if echo "$nodes" | grep -q "$n"; then
    printf "  [OK]   %s\n" "$n"
  else
    printf "  [FAIL] %s\n" "$n"
  fi
done

# 3. 话题列表
echo ""
echo "--- ROS2 话题 ---"
ros2 topic list 2>/dev/null || true

# 4. CPU / 内存
echo ""
echo "--- CPU / 内存 ---"
top -bn1 2>/dev/null | head -5 || echo "  (top 不可用)"

# 5. 磁盘
echo ""
echo "--- 磁盘 ---"
df -h / 2>/dev/null || echo "  (df 不可用)"

# 6. 温度
echo ""
echo "--- 温度 ---"
found=0
for zone in /sys/class/thermal/thermal_zone*/temp; do
  [ -f "$zone" ] || continue
  found=1
  temp=$(( $(cat "$zone") / 1000 ))
  printf "  %s: %d°C\n" "$(basename "$(dirname "$zone")")" "$temp"
done
[ "$found" -eq 0 ] && echo "  (无温度传感器节点)"

# 7. 话题频率（IMU / 定位）
echo ""
echo "--- 话题频率（抽样）---"
if command -v timeout >/dev/null 2>&1; then
  timeout 2 ros2 topic hz /imu/data 2>/dev/null | head -3 || echo "  /imu/data 无数据"
else
  echo "  (timeout 不可用)"
fi

# 8. HunterCore 接入状态（文档 §5.6/§5.7）
# 只查“接入包在不在、服务在不在跑”，不读 kafka.properties 内容（避免口令进日志）
echo ""
echo "--- HunterCore 接入 ---"
if [ -d /etc/hunter/kafka ]; then
  for f in kafka.properties ca-cert.pem client-cert.pem client-key.pem; do
    if [ -f "/etc/hunter/kafka/$f" ]; then
      printf "  [OK]   /etc/hunter/kafka/%s (%s)\n" "$f" "$(stat -c '%a' "/etc/hunter/kafka/$f")"
    else
      printf "  [FAIL] 缺少 /etc/hunter/kafka/%s（重跑 hunter_core_setup.sh --bundle ...）\n" "$f"
    fi
  done
else
  echo "  [FAIL] /etc/hunter/kafka 不存在：尚未部署 HunterCore 接入包"
fi
if command -v systemctl >/dev/null 2>&1; then
  for s in ota-agent remote-agent; do
    printf "  %-14s %s\n" "$s:" "$(systemctl is-active "$s" 2>/dev/null || echo unknown)"
  done
fi
if command -v ros2 >/dev/null 2>&1 && command -v timeout >/dev/null 2>&1; then
  timeout 3 ros2 topic echo /health --once >/dev/null 2>&1 \
    && echo "  [OK]   /health 有数据（data_agent 可上报 health Topic）" \
    || echo "  [WARN] /health 无数据：health_monitor 未起或 data_agent 未订阅到"
fi
