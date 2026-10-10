#!/usr/bin/env bash
# hunter_core_setup.sh — HunterCore 车端接入一键部署（文档 §5.6 / §5.7）
#
# 做七件事（幂等，可重复执行；步骤号与运行日志一致）：
#   ⓪ 前置校参：运行用户与其主组存在、接入包四件套齐、工作空间存在（失败文本带下一步动作）
#   ① 依赖安装：librdkafka(C++/Python)、SCRAM-SHA-512 机制插件、sqlite3
#   ② 接入包落盘：把 HUNTER-001 bundle 装到 /etc/hunter/kafka/；已写入口令的 properties 不覆盖
#        （未传 --bundle 而现场四件套已齐时整段跳过，不碰已部署的凭据）
#   ②.1 凭据属主收敛：私钥与 p12 置为 `运行用户:其主组 0600` 并实测运行用户真能打开
#        （0600 只授予属主：装成 root:组 0600 时 Agent 打不开私钥，TLS 阶段报 Permission denied）
#   ③ 运行期配置：Agent 参数文件落到 /etc/hunter/，生成 agent_env.sh 与 README_agent_env
#   ④ 编译：以运行用户身份 colcon build 相关包
#   ⑤ 自检：hunter-kafka-check（接入链路七层自检，退出码非 0 不自启服务）；并给运行用户装
#        ~/.local/bin/hunter-kafka-check 命令包装（ament 不把 lib/<pkg>/ 加进 PATH，直接 source 后用不了）
#   ⑥ systemd：安装并启用 hunter-can / hunter-edge / ota-agent / remote-agent
#        （hunter-can 上电启用底盘 CAN；hunter-edge 依赖它启动整栈——
#         data_agent/command_agent 上报运营端并待命自动驾驶，见 §13.10）
#
# 凭据红线：kafka.properties 只落盘 /etc/hunter/kafka/，不进工作空间、不进版本库、不进日志。
#
# 用法:
#   sudo bash hunter_core_setup.sh --bundle ~/HUNTER-001-bundle/HUNTER-001 \
#        [--ws <工作空间>] [--user <登录用户>] [--skip-deps] [--skip-build]
#        [--offline] [--no-systemd] [--no-start] [--force-config] [--force-key]
#   # --ws / --user 通常省写：默认取 sudo 调用者的家目录 + ~/HunterEdge
#   # 改完口令/参数后的“重跑”可以不传 --bundle（/etc/hunter/kafka 四件套齐即自动复用）：
#   sudo bash hunter_core_setup.sh --skip-deps
#
# 参数:
#   --bundle       接入包目录（含 kafka.properties / ca-cert.pem / client-cert.pem /
#                  client-key.pem，可选 kafka-client.p12）。**首次接入必填**；
#                  /etc/hunter/kafka 已有四件套时可省写（脚本复用现场凭据并跳过步骤②）。
#                  注意：接入包不在仓库内，需先从开发机拷到车上（scp / U 盘）；
#                  传错层级（如传到 HUNTER-001-bundle 而非内层 HUNTER-001）会自动下钻。
#   --ws           HunterEdge 工作空间根（默认取**运行用户**家目录下的 HunterEdge，
#                  即 sudo 时不会变成 /root/HunterEdge）
#   --user         运行 Agent 的用户（默认 SUDO_USER，其次当前用户；可省略）
#   --skip-deps    跳过 apt/pip 安装（离线源或已装齐时用）
#   --skip-build   跳过 colcon 编译
#   --offline      自检只查本地配置/证书，不连 broker（现场无云端网络时用）
#   --no-systemd   不安装/启用 systemd 服务
#   --no-start     只安装并 enable systemd 服务，**不立即启动**（台架/无底盘时用；
#                  不加本项则启动 hunter-can → ota/remote-agent → hunter-edge 整栈）
#   --force-config 用仓内参数文件/接入包覆盖 /etc/hunter/ 下已有内容（默认保留现场修改，
#                  含已人工写入口令的 kafka.properties——重跑脚本不会把口令抹回占位符）
#   --force-key    允许 client-key.pem 权限非 0600 时继续（默认直接失败）
set -euo pipefail

BUNDLE_DIR=""
WS_DIR=""                             # 未指定时按运行用户的家目录推导（见下方“前置检查”）
RUN_USER="${SUDO_USER:-$(id -un)}"
SKIP_DEPS=0 SKIP_BUILD=0 OFFLINE=0 NO_SYSTEMD=0 FORCE_CONFIG=0 FORCE_KEY=0 NO_START=0

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
    --no-start)     NO_START=1; shift ;;
    --force-config) FORCE_CONFIG=1; shift ;;
    --force-key)    FORCE_KEY=1; shift ;;
    -h|--help)      sed -n '/^set -euo pipefail/q; p' "$0"; exit 0 ;;   # 打印文头注释（自动随注释行数变化，不需手工维护行号）
    *) die "未知参数: $1（--help 看用法）" ;;
  esac
done

[ "$(id -u)" -eq 0 ] || die "需要 root 权限：sudo bash $0 --bundle <接入包目录>"

# ---- ⓪ 前置校参 ----
# 运行用户必须真实存在：编译产物属主、/etc/hunter 的组授权、systemd Agent 运行身份都挂在它身上
# （现场常见误传：车上登录的是 agilex 却写 --user hunter，后续 chown root:hunter 必失败）
id "$RUN_USER" >/dev/null 2>&1 || die "运行用户不存在：$RUN_USER（车上登录用户可用 id -un 查；一般直接省略 --user，脚本会取 sudo 的调用者 $SUDO_USER）"
USER_HOME="$(getent passwd "$RUN_USER" | cut -d: -f6)"
[ -n "$USER_HOME" ] || die "查不到用户 $RUN_USER 的家目录（/etc/passwd 条目异常）"
# 用户传 ~/xxx 写法时（被引号包住不会预处理展开）手动映射到运行用户家目录
expand_tilde() { case "$1" in "~"|"~/"*) echo "${USER_HOME}${1#\~}" ;; *) echo "$1" ;; esac; }
BUNDLE_DIR="$(expand_tilde "$BUNDLE_DIR")"
WS_DIR="$(expand_tilde "$WS_DIR")"
[ -n "$WS_DIR" ] || WS_DIR="${USER_HOME}/HunterEdge"   # 不用 $HOME：sudo 下 $HOME 可能是 /root

KAFKA_DIR="/etc/hunter/kafka"
ETC_DIR="/etc/hunter"
PROPS="$KAFKA_DIR/kafka.properties"
RUN_GROUP="$(id -gn "$RUN_USER" 2>/dev/null || echo "$RUN_USER")"   # 用户主组（Ubuntu 默认为同名私有组）
getent group "$RUN_GROUP" >/dev/null || die "运行用户的主组不存在：$RUN_GROUP（用户属组异常，先修 /etc/passwd 与 /etc/group）"
REQUIRED_FILES=(kafka.properties ca-cert.pem client-cert.pem client-key.pem)

# 【“修完重跑”不该再要求原始接入包（实机反馈）】改完口令/参数只想重跑 ③④⑤⑥ 时，旧写法一进来
# 就 die “必须指定 --bundle”，现场只能满车找原始包。/etc/hunter/kafka 四件套齐即视为可复用。
REUSE_BUNDLE=0
if [ -z "$BUNDLE_DIR" ]; then
  HAVE_ALL=1
  for f in "${REQUIRED_FILES[@]}"; do
    [ -f "$KAFKA_DIR/$f" ] || HAVE_ALL=0
  done
  if [ "$HAVE_ALL" -eq 1 ]; then
    REUSE_BUNDLE=1
    warn "未指定 --bundle，复用 $KAFKA_DIR 已部署的接入包（步骤② 跳过，不动现场凭据）"
    warn "  要重装原始包仍加 --bundle <目录>；要用包内容覆盖现场配置再加 --force-config"
  else
    die "必须指定 --bundle <接入包目录>（先拷到车上：scp -r <开发机>:HUNTER-001-bundle $USER_HOME/）
     ↳ 未传 --bundle 且 $KAFKA_DIR 下四件套不全（不能复用现场凭据）：ls -l $KAFKA_DIR"
  fi
fi
# 【复用现场凭据时**不做任何包目录校验**】——REUSE_BUNDLE=1 时 BUNDLE_DIR 为空是**正常**的。
# 旧写法紧接着就 `[ ! -d "$BUNDLE_DIR" ]` → `-d ""` 恒假 → 必然 die “接入包目录不存在:”，
# 于是“改完口令/参数后重跑（不带 --bundle）”这条**文档承诺的路径一进来就挂**，
# 连带步骤③④⑤⑥（参数落盘 / 编译 / 自检 / systemd）全都没跑
# （HUNTER-001 实机复现：systemctl 四个服务 inactive、hunter-kafka-check 命令不存在）。
if [ "$REUSE_BUNDLE" -eq 0 ]; then
  if [ ! -f "$BUNDLE_DIR/kafka.properties" ] && [ -d "$BUNDLE_DIR" ]; then
    # 只传了外层包目录时自动下钻一层（平台下发的目录形如 HUNTER-001-bundle/HUNTER-001）
    CANDIDATES=()
    while IFS= read -r d; do CANDIDATES+=("$d"); done \
      < <(find "$BUNDLE_DIR" -mindepth 1 -maxdepth 2 -name kafka.properties -type f 2>/dev/null | sort)
    if [ "${#CANDIDATES[@]}" -eq 1 ]; then
      warn "--bundle 指向的目录里没有 kafka.properties，自动下钻：$BUNDLE_DIR → $(dirname "${CANDIDATES[0]}")"
      BUNDLE_DIR="$(dirname "${CANDIDATES[0]}")"
    elif [ "${#CANDIDATES[@]}" -gt 1 ]; then
      die "$BUNDLE_DIR 下找到多个 kafka.properties，请明确指定其中一个目录：
$(printf '     %s\n' "${CANDIDATES[@]}")"
    fi
  fi
  if [ ! -d "$BUNDLE_DIR" ]; then
    die "接入包目录不存在: $BUNDLE_DIR
     ↳ 接入包不在仓库内，需先从开发机拷到车上（scp / U 盘），例：
         scp -r <开发机用户>@<开发机IP>:~/HUNTER-001-bundle $USER_HOME/
     ↳ 已在车上但不知道在哪：sudo find / -name kafka.properties 2>/dev/null
     ↳ 拷过来先自检目录内容：ls -l $BUNDLE_DIR   （应见 kafka.properties + 三个 .pem）"
  fi
  BUNDLE_DIR="$(cd "$BUNDLE_DIR" && pwd)"
  for f in "${REQUIRED_FILES[@]}"; do
    [ -f "$BUNDLE_DIR/$f" ] || die "接入包缺少 $f（应在 $BUNDLE_DIR 下）；该目录实际内容如下：
$(ls -l "$BUNDLE_DIR" 2>&1 | sed 's/^/     /')"
  done
  [ -f "$BUNDLE_DIR/kafka-client.p12" ] || warn "包内无 kafka-client.p12（用 PEM 证书链即可，不影响）"
else
  log "接入包目录校验：跳过（复用 $KAFKA_DIR，未使用原始包）"
fi

AGENT_PKGS=(command_agent data_agent ota_agent remote_agent)
BUILD_PKGS=(hunter_msgs hunter_kafka "${AGENT_PKGS[@]}" hunter_bringup)   # 含 hunter_bringup：launch 里的 use_command_agent 必须装到 install/ 才生效
[ -d "$WS_DIR" ] || die "工作空间目录不存在: $WS_DIR（--ws 应指向车上的 HunterEdge 根；默认 $USER_HOME/HunterEdge）"
WS_DIR="$(cd "$WS_DIR" && pwd)"      # 相对路径归一，避免后续 install/ 拼错
SRC_ROOT="$WS_DIR/src"
[ -d "$SRC_ROOT/hunter_agents" ] || die "工作空间不对，未找到 $SRC_ROOT/hunter_agents: $WS_DIR"
[ -d "$SRC_ROOT/hunter_common/hunter_kafka" ] || die "缺少公共包 src/hunter_common/hunter_kafka（本版本新增，先 git pull 或检查仓同步状态）"

if [ "$REUSE_BUNDLE" -eq 1 ]; then
  log "接入包: 复用 $KAFKA_DIR（本次未使用原始包）"
else
  log "接入包: $BUNDLE_DIR"
fi
log "工作空间: $WS_DIR   运行用户: $RUN_USER"

# ---- ① 依赖 ----
# 【包名坑（实测）】SCRAM-SHA-512 机制在 Ubuntu 上**不叫** cyrus-sasl-scram（那是
# RHEL/openSUSE 的名字，在这装直接 Unable to locate package）。
# 【属主包已现场钉死】HUNTER-001 实机（Jetson aarch64 jammy）：
#   dpkg -S /usr/lib/aarch64-linux-gnu/sasl2/libscram.so
#   → libsasl2-modules:arm64  （不是先前从官方 amd64 清单推的 libsasl2-modules-gssapi-mit，
#     Launchpad #1988730 那个归类在 arm64 上不成立）。
# 但判据仍一律用**插件文件**（跨架构/版本都不失真），包名只当处置提示；
# 缺件时两个候选一起试装无害（gssapi-mit 额外拉进 Kerberos 依赖，不影响机制）。
# 【存在性判定要逐个候选测】`ls 路径A 路径B` 只要任一路径不存在就返回非 0，
# 而 /usr/lib/sasl2 在 Ubuntu 上通常不存在 → 插件在位也会被误判为缺（假阴性，现场已误报过）。
# 另：Ubuntu 里名为 `scram` 的包是 Probabilistic Risk Analysis Tool，与 SASL 无关，千万别装。
scram_plugin() {
  local p
  for p in /usr/lib/*/sasl2/libscram.so /usr/lib/sasl2/libscram.so /usr/lib64/sasl2/libscram.so; do
    [ -e "$p" ] && { echo "$p"; return 0; }
  done
  return 1
}
report_scram_plugin() {   # 不论是否 --skip-deps 都报一句，让现场自己看到插件到底归谁
  local so owner
  if so="$(scram_plugin)"; then
    owner="$(dpkg -S "$so" 2>/dev/null | cut -d: -f1 || true)"
    log "  SCRAM 插件在位：$so（属主包：${owner:-未被任何 dpkg 包记录，多为镜像预置或手工摆放}）"
  else
    warn "  缺 SCRAM 插件（libscram.so）：SASL_SSL + SCRAM-SHA-512 必报 No worthy mechs found。处置：
       sudo apt install -y libsasl2-modules                              # aarch64 jammy 实测属主包
       ls /usr/lib/*/sasl2/libscram.so                                     # 有输出即机制就绪（无需重编，重启 Agent 即可）
       dpkg -S /usr/lib/*/sasl2/libscram.so                                # 拿这台机子的答复为准
     ↳ 上面两条都说明缺而未补上时再试 libsasl2-modules-gssapi-mit（另一候选，个别版本/架构把它归这里）；dpkg -L 与 dpkg -S 可能不一致，以 dpkg -S 查到的属主为准"
  fi
}
if [ "$SKIP_DEPS" -eq 0 ]; then
  log "① 依赖检查与安装（apt + pip）"
  export DEBIAN_FRONTEND=noninteractive
  need_pkgs=()
  dpkg -s librdkafka++1  >/dev/null 2>&1 || need_pkgs+=(librdkafka++1)     # data_agent C++ 运行时
  dpkg -s librdkafka-dev >/dev/null 2>&1 || need_pkgs+=(librdkafka-dev)     # ④ 编译期头文件
  dpkg -s libsqlite3-dev >/dev/null 2>&1 || need_pkgs+=(libsqlite3-dev)     # 断网缓存
  dpkg -s libsasl2-modules >/dev/null 2>&1 || need_pkgs+=(libsasl2-modules)
  dpkg -s ca-certificates >/dev/null 2>&1 || need_pkgs+=(ca-certificates)
  if ! scram_plugin >/dev/null; then
    # 插件不在位：两个候选包一起试（不同架构/版本分属不同包，只赌一个会“已最新但仍缺”）
    need_pkgs+=(libsasl2-modules libsasl2-modules-gssapi-mit)
  fi
  if [ "${#need_pkgs[@]}" -gt 0 ]; then
    log "  待装：${need_pkgs[*]}"
    # update 失败不阻断：包已在本地而源不可达（4G/内网/仓库过期）时照样能装；
    # 旧写法在 set -e 下直接静默退出，现场只看到一行 W: 就停了，像“脚本挂了”
    apt-get update -qq || warn "  apt-get update 未全成功（源不可达/仓库过期），继续尝试安装"
    apt-get install -y "${need_pkgs[@]}" || die "  依赖安装失败：${need_pkgs[*]}
     ↳ 上面 apt 的 E: 行才是真原因：
         Unable to locate package → 源里没有该包或未启用 universe：
           sudo add-apt-repository universe && sudo apt-get update
         404 / Failed to fetch → 源不可达或索引过期，先修源（或接本地镜像）
     ↳ 完全离线：在有网机器上 apt download 同一型号 deb 后 dpkg -i，或用 --skip-deps 只跑后续步骤
     ↳ 已手工装齐：重跑本脚本加 --skip-deps（本步会按文件/插件存在性自动判定，不需猜）"
  else
    log "  apt 依赖已齐（按本地文件判定，无需联网）"
  fi
  # 【判据要用 API，不只看 import 成不成功（实机反馈）】pip 装成功后仍可能因**版本过旧**而
  # 没有 AdminClient（自检 6/7 层与 Topic 清单核对都靠它），所以直接试取那个符号。
  if python3 -c "from confluent_kafka.admin import AdminClient" 2>/dev/null; then
    log "  confluent-kafka 可用：$(python3 -c 'import confluent_kafka as c; print(getattr(c, "__version__", None) or c.version()[0])' 2>/dev/null || echo 未知版本)"
  else
    warn "  confluent-kafka 缺失或过旧（取不到 admin.AdminClient，自检第 6/7 层与三个 Python Agent 都依赖它）：尝试安装/升级"
    pip3 install -U --quiet confluent-kafka \
      || die "pip3 安装/升级 confluent-kafka 失败（离线环境请预先准备 wheel；缺它只影响 ota/remote/command 三个 Python Agent 与自检 6/7 层，data_agent 仍可上报）
     ↳ 先看现场装的是什么：python3 -c 'import confluent_kafka as c; print(getattr(c, "__version__", "?"), c.__file__)'
       版本过旧或路径不在 /usr/local/lib/python3.*/dist-packages → 先卸再装：
         sudo pip3 uninstall -y confluent-kafka && sudo pip3 install -U confluent-kafka
       报 librdkafka 相关错误时：sudo apt install -y librdkafka-dev 后重试"
  fi
  # remote_agent 的 WebSocket 备用通道（主通道是 Kafka 的 hunter.<vid>.remote_control）：
  # 缺包只少一条备用路径、不影响指令链；缺失时 remote_agent 日志只提示一次补装命令（不再每 5s 刷屏）
  if python3 -c "import websocket" 2>/dev/null; then
    log "  websocket-client 可用（remote_agent 备用 WS 通道可启用）"
  else
    if pip3 install -U --quiet websocket-client; then
      log "  已安装 websocket-client（remote_agent 备用 WS 通道可用）"
    else
      warn "  websocket-client 安装失败（离线？）：remote_agent 将只走 Kafka 主通道，WS 备用通道关闭"
    fi
  fi

else
  log "① 跳过依赖安装（--skip-deps）"
fi
report_scram_plugin

# ---- ② 接入包落盘 ----
# 未传 --bundle 且现场四件套齐时整段跳过（段内语句按原缩进留在 else 分支里，不重排以减小 diff）
if [ "$REUSE_BUNDLE" -eq 1 ]; then
  log "② 跳过接入包落盘：复用 $KAFKA_DIR 下的现场凭据（未做任何改动）"
  if grep -q '<SCRAM_PASSWORD>' "$PROPS" 2>/dev/null; then
    warn "  但现场 $PROPS 仍是占位口令 <SCRAM_PASSWORD>：请人工写入真实 SCRAM 口令后重跑"
  fi
else
log "② 安装接入包到 $KAFKA_DIR"
install -d -m 0750 -o root -g "$RUN_GROUP" "$KAFKA_DIR"
# properties 默认**不覆盖现场**：运营已手改写入 SCRAM 口令、而包内仍是占位符时，
# 重跑脚本（如 --skip-build）不得把口令抹回 <SCRAM_PASSWORD>；确需覆盖用 --force-config。
# 只看包内容是否含占位串（grep -q，不输出任何行），避免口令进终端/日志。
PRESERVE_PROPS=0
if [ -f "$KAFKA_DIR/kafka.properties" ] && [ "$FORCE_CONFIG" -eq 0 ] \
   && grep -q '<SCRAM_PASSWORD>' "$BUNDLE_DIR/kafka.properties" \
   && ! grep -q '<SCRAM_PASSWORD>' "$KAFKA_DIR/kafka.properties"; then
  PRESERVE_PROPS=1
  warn "  保留现场 $KAFKA_DIR/kafka.properties（已含接入凭据，包内仍是占位符）；要强制用包内容覆盖请加 --force-config"
fi
cp -f "$BUNDLE_DIR"/ca-cert.pem "$BUNDLE_DIR"/client-cert.pem "$BUNDLE_DIR"/client-key.pem "$KAFKA_DIR/"
if [ "$PRESERVE_PROPS" -eq 0 ]; then
  cp -f "$BUNDLE_DIR/kafka.properties" "$KAFKA_DIR/"
fi
if [ -f "$BUNDLE_DIR/kafka-client.p12" ]; then
  # p12 仅作备用（PEM 链已可用：librdkafka 无需 keystore 即可建 mTLS）
  cp -f "$BUNDLE_DIR/kafka-client.p12" "$KAFKA_DIR/kafka-client.p12"
fi
chown root:"$RUN_GROUP" "$KAFKA_DIR"/*
chmod 0640 "$KAFKA_DIR"/kafka.properties "$KAFKA_DIR"/ca-cert.pem "$KAFKA_DIR"/client-cert.pem
# 私钥与 p12 含私钥材料，它们的属主/权限由下面 ②.1 统一收敛（0600 只授予属主，
# 而属主必须是运行 Agent 的用户）：放在 ②.1 而不在这里，是为了让省 --bundle 的
# 重跑也能修正现场已落坏的属主（旧版把私钥装成 root:组 0600，Agent 打不开）

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
fi

# ---- ②.1 凭据属主/权限收敛（**不依赖是否重新落盘**，省 --bundle 重跑也会修）----
if [ -f "$KAFKA_DIR/client-key.pem" ]; then
  chown "$RUN_USER:$RUN_GROUP" "$KAFKA_DIR/client-key.pem"
  chmod 0600 "$KAFKA_DIR/client-key.pem"
fi
if [ -f "$KAFKA_DIR/kafka-client.p12" ]; then
  chown "$RUN_USER:$RUN_GROUP" "$KAFKA_DIR/kafka-client.p12"
  chmod 0600 "$KAFKA_DIR/kafka-client.p12"
fi

# 以运行用户身份真的 open 一次：只看 mode 会漏“属主是 root 的 0600”这类读不到的情况；
# 只取 1 字节且输出丢弃，凭据内容不落终端/日志
if command -v sudo >/dev/null 2>&1; then
  for f in kafka.properties ca-cert.pem client-cert.pem client-key.pem kafka-client.p12; do
    [ -f "$KAFKA_DIR/$f" ] || continue
    if ! sudo -u "$RUN_USER" head -c 1 "$KAFKA_DIR/$f" >/dev/null 2>&1; then
      want_mode=640
      case "$f" in
        client-key.pem|kafka-client.p12) want_mode=600 ;;
      esac
      die "$RUN_USER 读不到 $KAFKA_DIR/$f（现为 $(stat -c '%U:%G %a' "$KAFKA_DIR/$f")）
 ↳ 文件权限只授予属主/属组，而目录又是 root 拥有时就会卡在这；修正：
     sudo chown $RUN_USER:$RUN_GROUP $KAFKA_DIR/$f && sudo chmod $want_mode $KAFKA_DIR/$f"
    fi
  done
  log "凭据可读性：$RUN_USER 可打开 $KAFKA_DIR 下全部凭据"
else
  warn "无 sudo 命令，跳过“运行用户能否真打开凭据”实测；请手工确认 "\
       "$RUN_USER 可读 $KAFKA_DIR/client-key.pem（否则 TLS 阶段报 Permission denied）"
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

# DDS 实现继承（可选但重要）：systemd 不会读登录 shell 的 ~/.bashrc，
# 若现场在 .bashrc 里设了 RMW_IMPLEMENTATION（本车典型为 rmw_cyclonedds_cpp），
# 开机自启的节点就会落到另一套 DDS → 登录终端里的 ros2 CLI / rviz2 “看不见”它们。
# 这里以运行用户身份探测并写入 agent_env.sh；探测不到就留空＝交给 ROS 默认值，
# 绝不在单元里硬编码某个实现（否则会把现场悄悄换成另一套 DDS）。
#
# ⚠ 探测细节：Ubuntu 默认 ~/.bashrc 开头有“非交互直接 return”的卫兵，
#   所以先用 `bash -lic`（交互）试，失败再退回 `bash -lc`（登录）；
#   并且用 __RMW__ 标记行 + 白名单正则取值，避免 .bashrc 里的其它输出混进变量。
RMW_DETECTED=""
if command -v sudo >/dev/null 2>&1 && [ "$RUN_USER" != "root" ]; then
  extract_rmw() { sed -n 's/^__RMW__\(rmw_[a-z0-9_]*\)$/\1/p' | tail -n1; }
  RMW_DETECTED="$(sudo -u "$RUN_USER" bash -lic 'printf "__RMW__%s\n" "${RMW_IMPLEMENTATION:-}"' 2>/dev/null | extract_rmw || true)"
  if [ -z "$RMW_DETECTED" ]; then
    RMW_DETECTED="$(sudo -u "$RUN_USER" bash -lc 'printf "__RMW__%s\n" "${RMW_IMPLEMENTATION:-}"' 2>/dev/null | extract_rmw || true)"
  fi
fi
if [ -n "$RMW_DETECTED" ]; then
  log "  检测到现场 DDS 实现：$RMW_DETECTED（写入 agent_env.sh，供 systemd 下的整栈继承）"
else
  warn "  未检测到 RMW_IMPLEMENTATION：整栈将用 ROS 默认 DDS。若现场确实在 ~/.bashrc 里设过，请手工把同一值写进 $ETC_DIR/agent_env.sh 的 HUNTER_RMW_IMPLEMENTATION（否则 systemd 下的节点与登录终端的 ros2 CLI/rviz2 会分属两套 DDS）"
fi

# systemd 单元 source 此文件取工作空间路径（单元里写死路径会挡住换机/换目录）
#
# ⚠ 本文件是 systemd EnvironmentFile：**不支持行内注释**（`#` 之后的整段都会被算进
#   变量值），所以注释必须独占一行写在这种位置，不能跟在值后面。
cat > "$ETC_DIR/agent_env.sh" <<EOF
# 由 hunter_core_setup.sh 生成于 $(date '+%F %T')；改工作空间位置后重跑脚本即可
# 修改任一开关后：sudo systemctl restart hunter-edge
HUNTER_WS_PREFIX=$WS_DIR/install
HUNTER_ROS_SETUP=/opt/ros/humble/setup.bash
HUNTER_OTA_CONFIG=$ETC_DIR/ota_agent_params.yaml
HUNTER_REMOTE_CONFIG=$ETC_DIR/remote_agent_params.yaml
# ── 开机自启链路（需求①②，见 hunter-can.service / hunter-edge.service）──
# 源码树位置（脚本/systemd 单元的兜底定位）
HUNTER_SRC_DIR=$WS_DIR
HUNTER_CAN_UP_SCRIPT=$SRC_ROOT/hunter_bringup/scripts/hunter_can_up.sh
HUNTER_EDGE_UP_SCRIPT=$SRC_ROOT/hunter_bringup/scripts/hunter_edge_up.sh
# 上电即进入自动驾驶准备状态；设为 false 则只跑链路（不含 Nav2/auto_mission，联调用）
HUNTER_EDGE_AUTONOMOUS_NAV=true
# nav = 导航巡航；mapping = 建图模式
HUNTER_EDGE_NAV_MODE=nav
HUNTER_EDGE_MAP_YAML=$WS_DIR/maps/hunter_map.yaml
HUNTER_EDGE_MAP_FILE=$WS_DIR/maps/hunter_map.pcd
# DDS 实现：留空 = 用 ROS 默认；须与现场 ~/.bashrc 的 RMW_IMPLEMENTATION 保持一致
HUNTER_RMW_IMPLEMENTATION=$RMW_DETECTED
EOF
chmod 0644 "$ETC_DIR/agent_env.sh"
log "  写入 $ETC_DIR/agent_env.sh（HUNTER_WS_PREFIX=$WS_DIR/install）"

# systemd 单元的 Documentation= 指向此文件（现场只需这一个说明就知道改哪里）
cat > "$ETC_DIR/README_agent_env" <<EOF
Agent 运行环境说明（由 hunter_core_setup.sh 生成于 $(date '+%F %T')）

  $ETC_DIR/agent_env.sh              工作空间/ROS 路径、参数文件位置、开机自启开关（systemd 单元 source 它）
  $ETC_DIR/<agent>_params.yaml        四个 Agent 的运行期参数（不含凭据）
  $KAFKA_DIR/                         HunterCore 接入包（properties + 证书，私钥 0600）

上电自启链路（需求①→④，systemd 单元按依赖顺序自动拉起）：
  hunter-can.service   上电即启用底盘 CAN（can2 @500k，抓帧验证）；不是 root 手工敲 ip link
  hunter-edge.service   CAN 就绪后启动 HunterEdge 整栈（Requires=hunter-can）：
                        · data_agent     → telemetry 10Hz / health 1Hz / event 上报运营端
                        · command_agent  → 消费平台指令，进入相应模式并回执 command_result
                        · 定位/感知/Nav2/auto_mission → 自动驾驶准备状态（待命）
  ota-agent.service    OTA 通知/状态（hunter.<vid>.ota_notify / ota_status）
  remote-agent.service 远程操控（hunter.<vid>.remote_control）

常见动作：
  换工作空间目录 / 换车 / 接入包重发  →  sudo bash <仓>/src/hunter_bringup/scripts/hunter_core_setup.sh --bundle <目录>
  改业务参数                        →  vi $ETC_DIR/<agent>_params.yaml && sudo systemctl restart ota-agent remote-agent
  改自启开关（如建图模式/关闭自主导航）→  vi $ETC_DIR/agent_env.sh && sudo systemctl restart hunter-edge
  只重启整栈（不动 Agent 服务）        →  sudo systemctl restart hunter-edge
  看开机时序                              →  systemctl status hunter-can hunter-edge
                                          journalctl -u hunter-can -u hunter-edge -b --no-pager
  接入不通                          →  hunter-kafka-check（看退出码：10 配置 20 证书 30 认证 40 网络 50 Topic 60 投递）
  底盘不通                          →  sudo bash <仓>/src/hunter_bringup/scripts/hunter_can_up.sh（看退出码 2=无接口 3=无底盘帧）
  严禁                              →  把凭据写进任何 YAML、拷进仓库/镜像层、或在日志里打印口令
EOF
chmod 0644 "$ETC_DIR/README_agent_env"

# ---- ④ 编译 ----
if [ "$SKIP_BUILD" -eq 0 ]; then
  log "④ colcon 编译（hunter_kafka + 四个 Agent + hunter_bringup）"
  [ -f /opt/ros/humble/setup.bash ] || die "未找到 /opt/ros/humble/setup.bash（先装 ROS2 Humble）"
  # 编译必须用普通用户身份：root 编译会在 build/install 留下 root 属主文件，
  # 下次用户自己 colcon build 就报 Permission denied（现场高频坑）
  BUILD_CMD="source /opt/ros/humble/setup.bash; cd '$WS_DIR'; colcon build --symlink-install --packages-select ${BUILD_PKGS[*]}"
  if [ "$RUN_USER" = "root" ]; then
    bash -c "$BUILD_CMD" || die "colcon 编译失败（先修编译错误，再谈接入联调）"
  else
    sudo -u "$RUN_USER" bash -c "$BUILD_CMD" \
      || die "colcon 编译失败（先修编译错误，再谈接入联调）"
  fi
else
  log "④ 跳过编译（--skip-build）"
fi

# ---- ④.1 可执行位自愈（脚本与三个 Agent 的入口）----
# git 在 Linux 上按索引 mode 恢复 +x，但 Windows 侧编辑/传输可能丢位；
# 这里统一兜一次，避免现场 `./hunter_status.sh: Permission denied`，
# 或 `ros2 launch` / systemd 找不到 lib/<pkg>/<pkg>_node。
chmod 0755 "$SRC_ROOT"/hunter_bringup/scripts/*.sh 2>/dev/null || true
for _pkg in command_agent ota_agent remote_agent; do
  _node="$SRC_ROOT/hunter_agents/$_pkg/scripts/${_pkg}_node"
  [ -f "$_node" ] && chmod 0755 "$_node"
done
log "④.1 已收敛可执行位（hunter_bringup/scripts/*.sh 与三个 Agent 的 *_node 入口）"

# ---- ⑤ 自检 ----
# 能离线判定的失败原因（口令占位、证书 CN 与 vehicle_id 不符、时间漂移、
# 私钥权限、broker 不可达、Topic 缺失）一次跑完，不依赖 ROS 环境
CHECK_BIN=""
for _cand in "$WS_DIR/install/hunter_kafka/bin/hunter-kafka-check" \
             "$WS_DIR/install/hunter_kafka/lib/hunter_kafka/hunter-kafka-check"; do
  [ -x "$_cand" ] && { CHECK_BIN="$_cand"; break; }
done
# ament_python 的 console_script 落在 bin/ 还是 lib/<pkg>/ 取决于安装布局，别赌路径：
# 激活工作空间后用 command -v 兜底
if [ -z "$CHECK_BIN" ] && [ -f "$WS_DIR/install/setup.bash" ]; then
  CHECK_BIN="$(bash -c "source '$WS_DIR/install/setup.bash' >/dev/null 2>&1; command -v hunter-kafka-check" || true)"
fi
if [ -z "$CHECK_BIN" ]; then
  CHECK_BIN="$(find "$WS_DIR/install" "$USER_HOME/.local/bin" -maxdepth 4 -type f -name hunter-kafka-check -perm -u+x 2>/dev/null | sort | head -n1 || true)"
fi
CHECK_ARGS=(--properties "$PROPS" --bundle-dir "$KAFKA_DIR")
if [ "$OFFLINE" -eq 1 ]; then
  CHECK_ARGS+=(--offline)
fi

# 【为什么优先源码方式（V0.1.06 实机教训）】`--symlink-install` 对 ament_python 走
# `setup.py develop` 语义：发行版元数据（`<pkg>.egg-info`）留在**源码目录**，靠
# `install/<pkg>/lib/python3.x/site-packages/easy-install.pth` 把源码目录加进 sys.path；
# 而 **`.pth` 只在 site.py 处理的 site-packages 目录里生效，对 PYTHONPATH 条目无效**。
# 因此 setuptools 生成的入口包装（含 `install/hunter_kafka/lib/hunter_kafka/hunter-kafka-check`）
# 执行 `load_entry_point(...)` 时必抛 `PackageNotFoundError: No package metadata was found for hunter-kafka`
# （实机 Traceback 已确认；同一根因也让 ota-agent/remote-agent 陷入 activating 重启循环、
#   command_agent 起后即退 —— 三个 Python Agent 已改走 `lib/<pkg>/<pkg>_node` 普通脚本入口）。
# 自检同理：**源码方式（`python3 -m hunter_kafka.diagnose`）永远可用，作为首选**；
# 入口包装仅在源码包缺失时兜底。
HK_SRC="$SRC_ROOT/hunter_common/hunter_kafka"
HK_SITE="$(ls -d "$WS_DIR"/install/hunter_kafka/lib/python3.*/site-packages 2>/dev/null | head -n1 || true)"
HK_PY="$HK_SRC${HK_SITE:+:$HK_SITE}${PYTHONPATH:+:$PYTHONPATH}"
run_check_source() {   # 源码方式跑自检：不依赖包元数据，永远可用的路径
  PYTHONPATH="$HK_PY" python3 -m hunter_kafka.diagnose "${CHECK_ARGS[@]}" || SELFTEST=$?
}
# 命令包装：给运行用户装 ~/.local/bin（新 shell 直接可用），
# 同时装 /usr/local/bin（root，**无需重登**就有命令：现场反馈“跑完脚本 hunter-kafka-check 仍 127”）
install_check_wrapper() {
  local marker="由 hunter_core_setup.sh 生成" w body
  body="$(cat <<EOF
#!/usr/bin/env bash
# $marker —— 直接走模块方式，不依赖包元数据，也不管入口被装到哪个目录
# 工作空间：$WS_DIR
export PYTHONPATH="$HK_SRC\${PYTHONPATH:+:\$PYTHONPATH}"
exec python3 -m hunter_kafka.diagnose "\$@"
EOF
)"
  # ① 运行用户家目录（推荐；随用户走）
  w="$USER_HOME/.local/bin/hunter-kafka-check"
  if [ ! -e "$w" ] || grep -qF "$marker" "$w" 2>/dev/null; then
    if install -d -m 0755 -o "$RUN_USER" -g "$RUN_GROUP" "$USER_HOME/.local/bin"; then
      printf '%s\n' "$body" > "$w"
      chmod 0755 "$w"; chown "$RUN_USER:$RUN_GROUP" "$w"
      log "  已安装命令包装：$w"
    else
      warn "  建不了 $USER_HOME/.local/bin，跳过家目录包装"
    fi
  else
    warn "  $w 已存在且不是本脚本生成，不覆盖（自己改）"
  fi
  # ② /usr/local/bin（在默认 PATH 上，登录与否都能敲）
  w="/usr/local/bin/hunter-kafka-check"
  if [ ! -e "$w" ] || grep -qF "$marker" "$w" 2>/dev/null; then
    printf '%s\n' "$body" > "$w" && chmod 0755 "$w" \
      && log "  已安装系统级命令包装：$w（当前 shell 直接可用，无需重登）" \
      || warn "  写不了 $w（跳过；用家目录包装或源码方式）"
  else
    warn "  $w 已存在且不是本脚本生成，不覆盖（自己改）"
  fi
}
install_check_wrapper

SELFTEST=0
if [ -f "$HK_SRC/hunter_kafka/diagnose.py" ]; then
  log "⑤ 自检：源码方式运行 hunter-kafka-check（配置/证书/机制/网络/认证/Topic/投递）"
  # confluent_kafka 在 client.py 里是**惰性导入**，缺它只影响第 6/7 层（SASL 握手与投递），
  # 前 5 层结论照样拿得到
  python3 -c "from confluent_kafka.admin import AdminClient" 2>/dev/null \
    || warn "  confluent-kafka 缺失或过旧（无 admin.AdminClient）：第 6/7 层会挂（sudo pip3 install -U confluent-kafka 补齐），前 5 层结论仍有效"
  run_check_source
elif [ -n "$CHECK_BIN" ]; then
  log "⑤ 自检：$CHECK_BIN（源码包缺失，改走安装的入口；若抛 PackageNotFoundError 属已知元数据问题）"
  env PYTHONPATH="$HK_PY" "$CHECK_BIN" "${CHECK_ARGS[@]}" || SELFTEST=$?
else
  warn "⑤ 既没扫到自检入口、也缺源码包 src/hunter_common/hunter_kafka，跳过自检"
fi
if [ "$SELFTEST" -eq 1 ]; then
  warn "⑤ 自检未真正执行（退出码 1 不是链路结论，是 Python 报错）：手工复现看 Traceback："
  warn "    PYTHONPATH=$HK_SRC python3 -m hunter_kafka.diagnose ${CHECK_ARGS[*]}"
fi
if [ "$SELFTEST" -ne 0 ]; then
  warn "自检未全通过（退出码 $SELFTEST）：按上面每一条的处置提示修，修完重跑 hunter-kafka-check"
fi

# ---- ⑥ systemd ----
# 开机时序（需求①→②）：hunter-can（底盘 CAN）→ hunter-edge（整栈/自动驾驶准备状态）
#   → ota-agent / remote-agent（平台侧交互服务，无 CAN 依赖）
# 单元清单与来源：
#   hunter_bringup/systemd/{hunter-can,hunter-edge}.service   —— 上电自启链路
#   hunter_agents/{ota_agent,remote_agent}/scripts/*.service  —— Agent 常驻服务
if [ "$NO_SYSTEMD" -eq 0 ]; then
  log "⑥ 安装 systemd 服务（hunter-can / hunter-edge / ota-agent / remote-agent）"
  unit_src_of() {
    case "$1" in
      hunter-can|hunter-edge) echo "$SRC_ROOT/hunter_bringup/systemd/$1.service" ;;
      # ota-agent → hunter_agents/ota_agent/scripts/ota-agent.service
      *)                      echo "$SRC_ROOT/hunter_agents/${1/-agent/_agent}/scripts/$1.service" ;;
    esac
  }
  for unit in hunter-can hunter-edge ota-agent remote-agent; do
    src_unit="$(unit_src_of "$unit")"
    if [ ! -f "$src_unit" ]; then
      warn "  未找到 $src_unit，跳过 $unit"
      continue
    fi
    install -m 0644 "$src_unit" "/etc/systemd/system/$unit.service"
    log "  安装 $unit.service"
  done
  systemctl daemon-reload
  # 顺序 enable：systemd 会按单元内的 Requires/After 再排一次，这里只是让现场
  # `systemctl list-unit-files` 一眼看出依赖链
  systemctl enable hunter-can.service
  systemctl enable hunter-edge.service
  systemctl enable ota-agent.service remote-agent.service
  if [ "$OFFLINE" -eq 0 ] && [ "$SELFTEST" -eq 0 ] && [ "$NO_START" -eq 0 ]; then
    # 先单一 CAN 服务（不是整栈）：即便后续整栈起不来，也先保证底盘通信与诊断路径可用
    systemctl restart hunter-can.service
    systemctl restart ota-agent.service remote-agent.service
    systemctl restart hunter-edge.service
    log "  已启动并设为开机自启"
    log "    systemctl status hunter-can hunter-edge ota-agent remote-agent"
    log "    journalctl -u hunter-can -u hunter-edge -b -f"
  elif [ "$NO_START" -eq 1 ]; then
    warn "  已 enable 但按 --no-start **不**立即启动（台架/无底盘现场用）"
    warn "  需要时手工：sudo systemctl start hunter-can && sudo systemctl start hunter-edge"
  else
    warn "  自检未通过，已 enable 但**不**自动启动（避免坏配置反复重启刷日志）"
    warn "  修好后：sudo systemctl start hunter-can && sudo systemctl start hunter-edge"
  fi
else
  log "⑥ 跳过 systemd（--no-systemd）"
fi

echo
if grep -q '<SCRAM_PASSWORD>' "$PROPS" 2>/dev/null; then
  warn "接入尚未生效：$PROPS 的口令仍是占位符 → sudo vi $PROPS 写入 SCRAM 口令后重跑本脚本"
  warn "（重跑不会把你已写入的口令抹回占位符；编译/依赖都齐时可加 --skip-deps --skip-build 只跑⑤⑥）"
fi
log "完成。后续步骤："
log "  1) 开机自启已装好：hunter-can（CAN）→ hunter-edge（整栈/自动驾驶准备状态）"
log "     现场核对：systemctl status hunter-can hunter-edge；journalctl -u hunter-edge -b -f"
log "  2) 手工启动整栈（等价于开机自启那条路径）："
log "     bash $SRC_ROOT/hunter_bringup/scripts/hunter_edge_up.sh"
log "  3) 现场改参：vi $ETC_DIR/<agent>_params.yaml，再按 params_file 指向它启动；"
log "     整栈开关（自动驾驶/建图/地图路径）在 $ETC_DIR/agent_env.sh，改后 sudo systemctl restart hunter-edge"
log "  4) 联调判据：平台侧该车辆 last_online_time 刷新；车端 hunter-kafka-check 退出码 0"
log "     （自检命令入口：新开一个 shell 后直接 hunter-kafka-check；未重登则用"
log "       $USER_HOME/.local/bin/hunter-kafka-check 或 PYTHONPATH=$HK_SRC python3 -m hunter_kafka.diagnose）"
log "  5) 严禁：把 $KAFKA_DIR 下任何文件拷进工作空间/版本库，或在日志中打印口令"
exit "$SELFTEST"
