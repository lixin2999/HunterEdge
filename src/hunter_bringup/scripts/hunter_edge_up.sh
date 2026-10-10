#!/usr/bin/env bash
# hunter_edge_up.sh — HunterEdge 整栈启动包装（需求②③；供 hunter-edge.service 调用）
#
# 职责（顺序即上电后的真实时序）：
#   ① CAN 复查：再跑一次幂等 hunter_can_up.sh --quick（底盘通信是 §1 CAN 驱动的前提）
#   ② 运营端连通性预检：hunter-kafka-check（失败**不阻断**启动——data_agent 会
#      把 telemetry/health/event 落 SQLite 缓存，链路恢复后按序回放，见 §13.9 第 2 条）
#   ③ 进入自动驾驶准备状态：ros2 launch hunter_bringup hunter_edge.launch.py
#      （默认 use_autonomous_nav:=true → 定位/感知/Nav2/auto_mission 全部就绪待命，
#        收到平台指令后按指令进入相应模式，见 command_agent）
#
# 为什么用脚本而不是把 ros2 launch 直接写进 unit：
#   systemd 不 source ROS/工作空间；且“CAN 就绪校验 + 连通性预检 + 启动参数整形”
#   三件事需要可读的日志与可现场覆盖的环境变量（agent_env.sh）。
#
# 用法（现场手工排查）:
#   bash hunter_edge_up.sh                 # 与开机自启完全同一条路径
#   bash hunter_edge_up.sh --no-check      # 跳过运营端预检（无网络现场）
#   HUNTER_EDGE_NAV_MODE=mapping bash hunter_edge_up.sh   # 建图模式
#
# 环境变量（缺省值可由 /etc/hunter/agent_env.sh 覆盖）:
#   HUNTER_WS_PREFIX           colcon install 目录（默认 ~/HunterEdge/install）
#   HUNTER_ROS_SETUP           ROS2 setup.bash（默认 /opt/ros/humble/setup.bash）
#   HUNTER_EDGE_AUTONOMOUS_NAV 是否进入自主导航待命（默认 true = 自动驾驶准备状态）
#   HUNTER_EDGE_NAV_MODE       nav（导航巡航，默认）| mapping（建图）
#   HUNTER_EDGE_MAP_YAML       静态地图 .yaml（默认 ~/HunterEdge/maps/hunter_map.yaml）
#   HUNTER_EDGE_MAP_FILE       先验点云 .pcd（默认 ~/HunterEdge/maps/hunter_map.pcd）
#   HUNTER_EDGE_EXTRA_ARGS     追加给 ros2 launch 的参数串（现场应急）
#   HUNTER_RMW_IMPLEMENTATION  可选：DDS 实现（如 rmw_cyclonedds_cpp）。
#                              留空 = 用 ROS 默认；systemd 读不到 ~/.bashrc，
#                              现场若用 CycloneDDS 必须在此继承，否则登录终端
#                              里的 ros2 CLI / rviz2 看不到开机自启的节点
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

RUN_CHECK=1
while [ $# -gt 0 ]; do
  case "$1" in
    --no-check) RUN_CHECK=0; shift ;;
    -h|--help)  sed -n '/^set -uo pipefail/q; p' "$0"; exit 0 ;;
    *) echo "[edge] 未知参数: $1（--help 看用法）" >&2; exit 1 ;;
  esac
done

log()  { echo "[edge] $*"; }
warn() { echo "[edge] ! $*" >&2; }
die()  { echo "[edge] × $*" >&2; exit "${2:-1}"; }

log "════════ HunterEdge 启动序列（上电自启路径）════════"

# ---- ① 底盘 CAN 通信复查（与 hunter-can.service 同一脚本，幂等）----
log "① 底盘 CAN 通信复查（${CAN_IF:-can2}）"
if bash "$SCRIPT_DIR/hunter_can_up.sh" --quick; then
  log "   CAN 已就绪"
else
  rc=$?
  # --quick 只校验接口存在与启用；返回非 0 说明接口都没起来（适配器/驱动问题）
  warn "   CAN 复查失败（退出码 $rc）：hunter_base 将无法与底盘通信"
  warn "   处置：sudo bash $SCRIPT_DIR/hunter_can_up.sh   # 看完整诊断"
fi

# ---- 运行环境（工作空间 / ROS 由 hunter_core_setup.sh 生成的 agent_env.sh 提供）----
WS_PREFIX="${HUNTER_WS_PREFIX:-$HOME/HunterEdge/install}"
ROS_SETUP="${HUNTER_ROS_SETUP:-/opt/ros/humble/setup.bash}"

# ---- ② 运营端连通性预检（不阻断：断网时车端缓存、恢复后回放）----
if [ "$RUN_CHECK" -eq 1 ]; then
  log "② 运营端接入预检（hunter-kafka-check）"
  # 优先级（V0.1.06 实机教训：setuptools 入口包装在 --symlink-install 下会抛
  # PackageNotFoundError，故入口只作最后兜底）：
  #   ① PATH 上的命令包装（hunter_core_setup.sh 装的 /usr/local/bin 或 ~/.local/bin，
  #      内容即 `python3 -m hunter_kafka.diagnose`，不依赖包元数据）
  #   ② 源码方式（HUNTER_SRC_DIR 已知，永远可用）
  #   ③ install/ 下的入口包装
  CHECK_CMD=""
  if command -v hunter-kafka-check >/dev/null 2>&1; then
    CHECK_CMD="hunter-kafka-check"
  elif [ -f "${HUNTER_SRC_DIR:-$HOME/HunterEdge}/src/hunter_common/hunter_kafka/hunter_kafka/diagnose.py" ]; then
    CHECK_CMD="SOURCE"
  elif [ -x "$WS_PREFIX/hunter_kafka/lib/hunter_kafka/hunter-kafka-check" ]; then
    # 隔离安装（colcon 默认）：<安装根>/<pkg>/lib/<pkg>/
    CHECK_CMD="$WS_PREFIX/hunter_kafka/lib/hunter_kafka/hunter-kafka-check"
  elif [ -x "$WS_PREFIX/lib/hunter_kafka/hunter-kafka-check" ]; then
    # 合并安装（--merge-install）：<安装根>/lib/<pkg>/
    CHECK_CMD="$WS_PREFIX/lib/hunter_kafka/hunter-kafka-check"
  fi

  if [ "$CHECK_CMD" = "SOURCE" ]; then
    HK_SRC="${HUNTER_SRC_DIR:-$HOME/HunterEdge}/src/hunter_common/hunter_kafka"
    log "   以源码方式运行自检（PYTHONPATH=$HK_SRC，不依赖包元数据）"
    if PYTHONPATH="$HK_SRC${PYTHONPATH:+:$PYTHONPATH}" timeout 60 \
        python3 -m hunter_kafka.diagnose >/tmp/hunter_edge_kafka_check.log 2>&1; then
      log "   接入链路可用（自检全过）"
    else
      rc=$?
      warn "   自检未全过（退出码 $rc）：10 配置 / 20 证书 / 30 认证 / 40 网络 / 50 Topic / 60 投递"
      warn "   详见 /tmp/hunter_edge_kafka_check.log；车端会先落 SQLite 缓存，链路恢复后自动回放"
    fi
  elif [ -n "$CHECK_CMD" ]; then
    if timeout 60 "$CHECK_CMD" >/tmp/hunter_edge_kafka_check.log 2>&1; then
      log "   接入链路可用（自检全过）"
    else
      rc=$?
      warn "   自检未全过（退出码 $rc）：10 配置 / 20 证书 / 30 认证 / 40 网络 / 50 Topic / 60 投递"
      warn "   详见 /tmp/hunter_edge_kafka_check.log；车端会先落 SQLite 缓存，链路恢复后自动回放"
    fi
  else
    warn "   未找到 hunter-kafka-check（先跑 hunter_core_setup.sh，或 --no-check 跳过）"
  fi
else
  log "② 跳过运营端预检（--no-check）"
fi

# ---- ③ 进入自动驾驶准备状态 ----
[ -f "$ROS_SETUP" ] || die "未找到 $ROS_SETUP（先装 ROS2 Humble，或用 HUNTER_ROS_SETUP 指定）"
[ -f "$WS_PREFIX/setup.bash" ] || die "未找到 $WS_PREFIX/setup.bash（先 colcon build，或用 HUNTER_WS_PREFIX 指定）"

# DDS 实现继承（可选）：systemd 读不到登录 shell 的 ~/.bashrc，若现场用 CycloneDDS
# 而这里不继承，开机自启的节点会落到另一套 DDS → 登录终端里的 ros2 CLI / rviz2 看不见它们。
if [ -n "${HUNTER_RMW_IMPLEMENTATION:-}" ]; then
  export RMW_IMPLEMENTATION="$HUNTER_RMW_IMPLEMENTATION"
  log "DDS 实现：RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION（来自 agent_env.sh）"
else
  log "DDS 实现：未指定，使用 ROS 默认（如需与现场一致，把 RMW_IMPLEMENTATION 写进 /etc/hunter/agent_env.sh 的 HUNTER_RMW_IMPLEMENTATION）"
fi

AUTONOMOUS="${HUNTER_EDGE_AUTONOMOUS_NAV:-true}"
NAV_MODE="${HUNTER_EDGE_NAV_MODE:-nav}"
MAP_YAML="${HUNTER_EDGE_MAP_YAML:-$HOME/HunterEdge/maps/hunter_map.yaml}"
MAP_FILE="${HUNTER_EDGE_MAP_FILE:-$HOME/HunterEdge/maps/hunter_map.pcd}"
EXTRA_ARGS="${HUNTER_EDGE_EXTRA_ARGS:-}"

log "③ 启动整栈（use_autonomous_nav=$AUTONOMOUS, mode=$NAV_MODE）"

# shellcheck disable=SC1090
set +u
source "$ROS_SETUP"
# shellcheck disable=SC1090
source "$WS_PREFIX/setup.bash"
set -u

# shellcheck disable=SC2086
exec ros2 launch hunter_bringup hunter_edge.launch.py \
  use_autonomous_nav:="$AUTONOMOUS" \
  autonomous_nav_mode:="$NAV_MODE" \
  map_yaml_path:="$MAP_YAML" \
  map_file_path:="$MAP_FILE" \
  $EXTRA_ARGS
