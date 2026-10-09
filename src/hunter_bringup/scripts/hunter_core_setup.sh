#!/usr/bin/env bash
# hunter_core_setup.sh — HunterCore 车端接入一键部署（文档 §5.6 / §5.7）
#
# 做五件事（幂等，可重复执行）：
#   ① 依赖安装：librdkafka(C++/Python)、cyrus-sasl-scram(SCRAM-SHA-512 机制)、sqlite3
#   ② 接入包落盘：把 HUNTER-001 bundle 整目录装到 /etc/hunter/kafka/，私钥 0600
#   ③ 运行期配置：Agent 参数文件落到 /etc/hunter/，生成 /etc/hunter/agent_env.sh
#   ④ 编译 + 自检：colcon build 相关包，然后跑 hunter-kafka-check（接入链路 6 层自检）
#   ⑤ systemd：安装并启用 ota-agent / remote-agent（data/command agent 由 bring-up 拉起）
#
# 凭据红线：kafka.properties 只落盘 /etc/hunter/kafka/，不进工作空间、不进版本库、不进日志。
#
# 用法:
#   sudo bash hunter_core_setup.sh --bundle ~/下载/HUNTER-001-bundle/HUNTER-001 \
#        [--ws ~/HunterEdge] [--user hunter] [--skip-deps] [--skip-build]
#        [--offline] [--no-systemd] [--force-config] [--force-key]
#
# 参数:
#   --bundle       接入包目录（含 kafka.properties / ca-cert.pem / client-cert.pem /
#                  client-key.pem，可选 kafka-client.p12）。**必填**
#   --ws           HunterEdge 工作空间根（默认 $HOME/HunterEdge）
#   --user         运行 Agent 的用户（默认 SUDO_USER，其次当前用户）
#   --skip-deps    跳过 apt/pip 安装（离线源或已装齐时用）
#   --skip-build   跳过 colcon 编译
#   --offline      自检只查本地配置/证书，不连 broker（现场无云端网络时用）
#   --no-systemd   不安装/启用 systemd 服务
#   --force-config 用仓内参数文件覆盖 /etc/hunter/ 下已有配置（默认保留现场修改）
#   --force-key    允许 client-key.pem 权限非 0600 时继续（默认直接失败）
set -euo pipefail

BUNDLE_DIR=""
WS_DIR="${HOME}/HunterEdge"
RUN_USER="${SUDO_USER:-$(id -un)}"
SKIP_DEPS=0 SKIP_BUILD=0 OFFLINE=0 NO_SYSTEMD=0 FORCE_CONFIG=0 FORCE_KEY=0

log()  { echo "[hunter-core] $*"; }
warn() { echo "[hunter-core] ! $*" >&2; }
die()  { echo "[hunter-core] × $*" >&2; exit 1; }

while [ $# -gt 0 ]; do
  case "$1" in
    --bundle)       BUNDLE_DIR="${2:?--bundle 需要目录路径}"; shift 2 ;;
    --ws)           WS_DIR="${2:?--ws 需要目录路径}"; shift 2 ;;
    --user)         RUN_USER="${2:?--user 需要用户名}"; shift 2 ;;
    --skip-deps)    SKIP_DEPS=1; shift ;;
    --skip-build)   SKIP_BUILD=1; shift ;;
    --offline)      OFFLINE=1; shift ;;
    --no-systemd)   NO_SYSTEMD=1; shift ;;
    --force-config) FORCE_CONFIG=1; shift ;;
    --force-key)    FORCE_KEY=1; shift ;;
    -h|--help)      sed -n '2,30p' "$0"; exit 0 ;;
    *) die "未知参数: $1（--help 看用法）" ;;
  esac
done

[ "$(id -u)" -eq 0 ] || die "需要 root 权限：sudo bash $0 --bundle <接入包目录>"

# ---- 0. 前置检查 ----
[ -n "$BUNDLE_DIR" ] || die "必须指定 --bundle <接入包目录>"
BUNDLE_DIR="$(cd "$BUNDLE_DIR" && pwd)" || die "接入包目录不存在: $BUNDLE_DIR"

KAFKA_DIR="/etc/hunter/kafka"
ETC_DIR="/etc/hunter"
PROPS="$KAFKA_DIR/kafka.properties"
RUN_GROUP="$(id -gn "$RUN_USER" 2>/dev/null || echo "$RUN_USER")"   # 用户主组（Ubuntu 默认为同名私有组）
REQUIRED_FILES=(kafka.properties ca-cert.pem client-cert.pem client-key.pem)
for f in "${REQUIRED_FILES[@]}"; do
  [ -f "$BUNDLE_DIR/$f" ] || die "接入包缺少 $f（应在 $BUNDLE_DIR 下）"
done
[ -f "$BUNDLE_DIR/kafka-client.p12" ] || warn "包内无 kafka-client.p12（用 PEM 证书链即可，不影响）"

AGENT_PKGS=(command_agent data_agent ota_agent remote_agent)
SRC_ROOT="$WS_DIR/src"
[ -d "$SRC_ROOT/hunter_agents" ] || die "工作空间不对，未找到 $SRC_ROOT/hunter_agents: $WS_DIR"
[ -d "$SRC_ROOT/hunter_common/hunter_kafka" ] || die "缺少公共包 src/hunter_common/hunter_kafka"

log "接入包: $BUNDLE_DIR"
log "工作空间: $WS_DIR   运行用户: $RUN_USER"

# ---- ① 依赖 ----
if [ "$SKIP_DEPS" -eq 0 ]; then
  log "① 安装依赖（apt + pip）"
  export DEBIAN_FRONTEND=noninteractive
  apt-get update -qq
  # cyrus-sasl-scram：librdkafka 的 SCRAM-SHA-512 机制由 Cyrus SASL 插件提供，
  # 缺失时表现为 sasl init failed / No worthy mechs found（与口令错误表象不同，别混）
  apt-get install -y -qq librdkafka++1 librdkafka-dev libsqlite3-dev \
                cyrus-sasl-scram libsasl2-modules ca-certificates
  if python3 -c "import confluent_kafka" 2>/dev/null; then
    log "  confluent-kafka 已安装：$(python3 -c 'import confluent_kafka as c; print(c.version()[0])')"
  else
    pip3 install --quiet confluent-kafka || die "pip3 安装 confluent-kafka 失败（离线环境请预先准备 wheel）"
  fi
else
  log "① 跳过依赖安装（--skip-deps）"
fi

# ---- ② 接入包落盘 ----
log "② 安装接入包到 $KAFKA_DIR"
install -d -m 0750 -o root -g "$RUN_GROUP" "$KAFKA_DIR"
cp -f "$BUNDLE_DIR"/kafka.properties "$BUNDLE_DIR"/ca-cert.pem "$BUNDLE_DIR"/client-cert.pem "$BUNDLE_DIR"/client-key.pem "$KAFKA_DIR/"
if [ -f "$BUNDLE_DIR/kafka-client.p12" ]; then
  # p12 仅作备用（PEM 链已可用：librdkafka 无需 keystore 即可建 mTLS）
  cp -f "$BUNDLE_DIR/kafka-client.p12" "$KAFKA_DIR/kafka-client.p12"
fi
chown root:"$RUN_GROUP" "$KAFKA_DIR"/*
chmod 0640 "$KAFKA_DIR"/kafka.properties "$KAFKA_DIR"/ca-cert.pem "$KAFKA_DIR"/client-cert.pem
if [ -f "$KAFKA_DIR/kafka-client.p12" ]; then
  chmod 0640 "$KAFKA_DIR/kafka-client.p12"
fi
chmod 0600 "$KAFKA_DIR/client-key.pem"

# 私钥权限是接入 README 的硬要求：0600 之外的任何值都说明曾被别的过程改过
KEY_MODE=$(stat -c '%a' "$KAFKA_DIR/client-key.pem")
if [ "$KEY_MODE" != "600" ]; then
  [ "$FORCE_KEY" -eq 1 ] || die "client-key.pem 权限为 $KEY_MODE（要求 0600）"
  warn "client-key.pem 权限 $KEY_MODE，已按 --force-key 继续"
fi

# SCRAM 口令占位符检查：模板里 password="<SCRAM_PASSWORD>" 未替换时，
# 认证必然失败且报错像“网络问题”，所以在这里提前拦下（口令由运营手工写入，脚本不经手）
if grep -q '<SCRAM_PASSWORD>' "$PROPS"; then
  warn "kafka.properties 的 sasl.jaas.config 仍是占位口令 <SCRAM_PASSWORD>"
  warn "请由运维人员手工写入真实 SCRAM 口令后重跑本脚本（脚本不会读取/记录该口令）"
fi

# ---- ③ 运行期配置 ----
log "③ 安装 Agent 参数文件到 $ETC_DIR"
install -d -m 0755 -o root -g "$RUN_GROUP" "$ETC_DIR"
for pkg in "${AGENT_PKGS[@]}"; do
  src_yaml="$SRC_ROOT/hunter_agents/$pkg/config/${pkg}_params.yaml"
  [ -f "$src_yaml" ] || { warn "  未找到 $src_yaml，跳过"; continue; }
  dst="$ETC_DIR/${pkg}_params.yaml"
  if [ -f "$dst" ] && [ "$FORCE_CONFIG" -eq 0 ]; then
    log "  保留现场配置：$dst（需覆盖请加 --force-config）"
  else
    if [ -f "$dst" ]; then
      cp -f "$dst" "$dst.bak.$(date +%Y%m%d%H%M%S)"
    fi
    install -m 0644 -o root -g "$RUN_GROUP" "$src_yaml" "$dst"
    log "  写入 $dst"
  fi
done

# systemd 单元 source 此文件取工作空间路径（单元里写死路径会挡住换机/换目录）
cat > "$ETC_DIR/agent_env.sh" <<EOF
# 由 hunter_core_setup.sh 生成于 $(date '+%F %T')；改工作空间位置后重跑脚本即可
HUNTER_WS_PREFIX=$WS_DIR/install
HUNTER_ROS_SETUP=/opt/ros/humble/setup.bash
HUNTER_OTA_CONFIG=$ETC_DIR/ota_agent_params.yaml
HUNTER_REMOTE_CONFIG=$ETC_DIR/remote_agent_params.yaml
EOF
chmod 0644 "$ETC_DIR/agent_env.sh"
log "  写入 $ETC_DIR/agent_env.sh（HUNTER_WS_PREFIX=$WS_DIR/install）"

# systemd 单元的 Documentation= 指向此文件（现场只需这一个说明就知道改哪里）
cat > "$ETC_DIR/README_agent_env" <<EOF
Agent 运行环境说明（由 hunter_core_setup.sh 生成于 $(date '+%F %T')）

  $ETC_DIR/agent_env.sh              工作空间/ROS 路径与参数文件位置（systemd 单元 source 它）
  $ETC_DIR/<agent>_params.yaml        四个 Agent 的运行期参数（不含凭据）
  $KAFKA_DIR/                         HunterCore 接入包（properties + 证书，私钥 0600）

常见动作：
  换工作空间目录 / 换车 / 接入包重发  →  sudo bash <仓>/src/hunter_bringup/scripts/hunter_core_setup.sh --bundle <目录>
  改业务参数                        →  vi $ETC_DIR/<agent>_params.yaml && sudo systemctl restart ota-agent remote-agent
  接入不通                          →  hunter-kafka-check（看退出码：10 配置 20 证书 30 认证 40 网络 50 Topic 60 投递）
  严禁                              →  把凭据写进任何 YAML、拷进仓库/镜像层、或在日志里打印口令
EOF
chmod 0644 "$ETC_DIR/README_agent_env"

# ---- ④ 编译 + 自检 ----
if [ "$SKIP_BUILD" -eq 0 ]; then
  log "④ colcon 编译（hunter_kafka + 四个 Agent）"
  [ -f /opt/ros/humble/setup.bash ] || die "未找到 /opt/ros/humble/setup.bash（先装 ROS2 Humble）"
  # 编译必须用普通用户身份：root 编译会在 build/install 留下 root 属主文件，
  # 下次用户自己 colcon build 就报 Permission denied（现场高频坑）
  if [ "$RUN_USER" = "root" ]; then
    BUILD_CMD="source /opt/ros/humble/setup.bash; cd '$WS_DIR'; colcon build --symlink-install --packages-select hunter_msgs hunter_kafka ${AGENT_PKGS[*]}"
    bash -c "$BUILD_CMD" || die "colcon 编译失败（先修编译错误，再谈接入联调）"
  else
    sudo -u "$RUN_USER" bash -c "source /opt/ros/humble/setup.bash; cd '$WS_DIR'; colcon build --symlink-install --packages-select hunter_msgs hunter_kafka ${AGENT_PKGS[*]}" \
      || die "colcon 编译失败（先修编译错误，再谈接入联调）"
  fi
else
  log "④ 跳过编译（--skip-build）"
fi

# 自检：能离线判定的失败原因（口令占位、证书 CN 与 vehicle_id 不符、时间漂移、
# 私钥权限、broker 不可达、Topic 缺失）一次跑完，不依赖 ROS 环境
CHECK_BIN="$WS_DIR/install/hunter_kafka/bin/hunter-kafka-check"
CHECK_ARGS=(--properties "$PROPS" --bundle-dir "$KAFKA_DIR")
if [ "$OFFLINE" -eq 1 ]; then
  CHECK_ARGS+=(--offline)
fi
SELFTEST=0
if [ -x "$CHECK_BIN" ]; then
  log "自检：hunter-kafka-check（配置/证书/网络/认证/Topic/投递 六层）"
  "$CHECK_BIN" "${CHECK_ARGS[@]}" || SELFTEST=$?
elif python3 -c "import confluent_kafka" 2>/dev/null; then
  # 未编译（--skip-build）时退回源码模式，仍可做本地配置/证书核查
  log "自检：以源码方式运行 hunter-kafka-check（未找到编译产物）"
  PYTHONPATH="$SRC_ROOT/hunter_common/hunter_kafka" python3 -m hunter_kafka.diagnose "${CHECK_ARGS[@]}" \
    || SELFTEST=$?
else
  warn "未编译且无 confluent_kafka，跳过自检（编好后手动补跑 hunter-kafka-check）"
fi
if [ "$SELFTEST" -ne 0 ]; then
  warn "自检未全通过（退出码 $SELFTEST）：按上面每一条的处置提示修，修完重跑 hunter-kafka-check"
fi

# ---- ⑤ systemd ----
if [ "$NO_SYSTEMD" -eq 0 ]; then
  log "⑤ 安装 systemd 服务（ota-agent / remote-agent）"
  for pkg in ota_agent remote_agent; do
    unit="${pkg/_agent/-agent}.service"
    unit_src="$SRC_ROOT/hunter_agents/$pkg/scripts/$unit"
    if [ ! -f "$unit_src" ]; then
      warn "  未找到 $unit_src，跳过"
      continue
    fi
    install -m 0644 "$unit_src" "/etc/systemd/system/$unit"
    log "  安装 $unit"
  done
  systemctl daemon-reload
  systemctl enable ota-agent.service remote-agent.service
  if [ "$OFFLINE" -eq 0 ] && [ "$SELFTEST" -eq 0 ]; then
    systemctl restart ota-agent.service remote-agent.service
    log "  已启动并设为开机自启（journalctl -u ota-agent -u remote-agent -f 看日志）"
  else
    warn "  自检未通过，已 enable 但**不**自动启动（避免坏配置反复重启刷日志）"
  fi
else
  log "⑤ 跳过 systemd（--no-systemd）"
fi

echo
log "完成。后续步骤："
log "  1) 启动整栈：ros2 launch hunter_bringup hunter_full.launch.py（含 data_agent + command_agent）"
log "  2) 现场改参：vi $ETC_DIR/<agent>_params.yaml，再按 params_file 指向它启动"
log "  3) 联调判据：平台侧该车辆 last_online_time 刷新；车端 hunter-kafka-check 退出码 0"
log "  4) 严禁：把 $KAFKA_DIR 下任何文件拷进工作空间/版本库，或在日志中打印口令"
exit "$SELFTEST"
