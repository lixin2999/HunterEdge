#!/usr/bin/env bash
# hunter_can_up.sh — 车辆上电后自动启用底盘 CAN 通信（需求①；文档 §11.3 / §20.3）
#
# 等价于现场手工三条命令（幂等，可反复执行）：
#   sudo ip link set can2 down            # 先下（改波特率必须处于 down）
#   sudo ip link set can2 type can bitrate 500000
#   sudo ip link set can2 up
#   candump can2 -n 10                    # 抓 10 帧确认底盘真的在发报文
#
# 为什么单独做成脚本而不是写进 systemd 一行命令：
#   ① **上电时序**：USB-CAN 适配器（gs_usb）枚举晚于内核启动，开机瞬间
#      `ip link set can2 ...` 必报 "Cannot find device can2"——必须等接口出现；
#   ② **波特率互斥**：接口处于 up 时改 bitrate 报 "Device or resource busy"，
#      所以必须先 down；
#   ③ **可判定性**：只有 candump 收到底盘反馈帧（0x211/0x221，附录 A）才说明
#      “底盘通信真的起来了”，仅看 `ip link` 的 UP 标志会把“接线错/波特率错/
#      底盘未上电”误判为成功（此时 CAN 写入静默丢弃，车辆看着像在跑却不动）。
#
# 用法:
#   sudo bash hunter_can_up.sh                # 上电自启用（要求抓到底盘帧）
#   sudo bash hunter_can_up.sh --no-strict    # 台架调试：只要求接口 up
#   sudo bash hunter_can_up.sh --quick        # 只做幂等复查（不等待枚举，短抓帧）
#   CAN_IF=can0 sudo bash hunter_can_up.sh    # 换接口
#
# 退出码（供 systemd 判定；hunter-edge.service 依赖本脚本返回 0）:
#   0  接口 up（严格模式下同时已抓到底盘反馈帧）
#   1  参数/环境错误（缺 ip 命令、非 root 且无 sudo）
#   2  等待超时仍未出现该 CAN 接口（适配器未插/驱动未加载/接口名不符）
#   3  接口已 up 但未抓到底盘反馈帧（接线/波特率/底盘未上电/终端电阻）
#
# 环境变量:
#   CAN_IF            接口名（默认 can2；can0 为 Jetson 板载 mttcan、未接线束）
#   CAN_BITRATE       波特率（默认 500000，HUNTER SE CAN 2.0B 契约值）
#   CAN_WAIT_SECONDS  等待接口出现的秒数（默认 20；--quick 下强制 3）
#   CAN_FRAME_TIMEOUT candump 抓帧窗口秒数（默认 5）
#   CAN_EXPECT_IDS    期望看到的底盘反馈帧 ID 正则（默认 211|221）
#   CAN_REQUIRE_FRAMES 0 = 抓不到帧也只告警不失败（默认 1）
set -uo pipefail

CAN_IF="${CAN_IF:-can2}"
BITRATE="${CAN_BITRATE:-500000}"
WAIT_SECONDS="${CAN_WAIT_SECONDS:-20}"
FRAME_TIMEOUT="${CAN_FRAME_TIMEOUT:-5}"
EXPECT_IDS="${CAN_EXPECT_IDS:-211|221}"
REQUIRE_FRAMES="${CAN_REQUIRE_FRAMES:-1}"

STRICT=1
QUICK=0
while [ $# -gt 0 ]; do
  case "$1" in
    --no-strict) REQUIRE_FRAMES=0; STRICT=0; shift ;;
    --quick)     QUICK=1; WAIT_SECONDS=3; FRAME_TIMEOUT=2; shift ;;
    -h|--help)   sed -n '/^set -uo pipefail/q; p' "$0"; exit 0 ;;
    *) echo "[can] 未知参数: $1（--help 看用法）" >&2; exit 1 ;;
  esac
done

log()  { echo "[can] $*"; }
warn() { echo "[can] ! $*" >&2; }
die()  { echo "[can] × $*" >&2; exit "${2:-1}"; }

# ---- ⓪ 运行身份与工具检查 ----
SUDO=""
if [ "$(id -u)" -ne 0 ]; then
  command -v sudo >/dev/null 2>&1 || die "非 root 且无 sudo：请用 sudo bash $0 执行"
  SUDO="sudo"
fi
command -v ip >/dev/null 2>&1 || die "缺少 iproute2（ip 命令）：sudo apt install -y iproute2"

# ---- ① 确保接口存在（上电时 USB-CAN 枚举晚于内核启动，必须等待）----
if ! ip link show "$CAN_IF" >/dev/null 2>&1; then
  log "接口 $CAN_IF 尚未出现，尝试加载 gs_usb（USB-CAN 适配器；板载 mttcan 无需此步）"
  $SUDO modprobe gs_usb 2>/dev/null || modprobe gs_usb 2>/dev/null || true
  deadline=$(( $(date +%s) + WAIT_SECONDS ))
  while [ "$(date +%s)" -lt "$deadline" ]; do
    if ip link show "$CAN_IF" >/dev/null 2>&1; then break; fi
    sleep 1
  done
fi
if ! ip link show "$CAN_IF" >/dev/null 2>&1; then
  # 板上可能叫别的名字（can0/can1 或 slcan0）：把现场实际可用接口打出来，省得现场猜
  warn "等待 ${WAIT_SECONDS}s 仍未出现接口 $CAN_IF"
  warn "  当前系统可见的 CAN 类接口：$(ls /sys/class/net 2>/dev/null | tr '\n' ' ')"
  warn "  自查：适配器是否插好 / dmesg | tail -30 看 gs_usb 是否绑定 / lsusb 是否识别"
  exit 2
fi

# ---- ② 配置波特率并启用（改 bitrate 必须先在 down 状态）----
$SUDO ip link set "$CAN_IF" down 2>/dev/null || true
if ! $SUDO ip link set "$CAN_IF" type can bitrate "$BITRATE" 2>/dev/null; then
  # 已配置过相同参数且处于 up 时会失败，这个不是错误；用当前实际值复核
  cur="$($SUDO ip -details link show "$CAN_IF" 2>/dev/null | grep -o 'bitrate [0-9]*' | head -1)"
  if [ "$cur" = "bitrate $BITRATE" ]; then
    log "接口已按 ${BITRATE}bps 配置（无需重复设置）"
  else
    warn "设置 bitrate=$BITRATE 失败（当前：${cur:-未知}）；继续尝试启用接口"
  fi
fi
$SUDO ip link set "$CAN_IF" up || die "启用 $CAN_IF 失败（查：ip -details link show $CAN_IF）" 1
$SUDO ip link set "$CAN_IF" txqueuelen 1000 2>/dev/null || true

OPERSTATE="$(cat "/sys/class/net/$CAN_IF/operstate" 2>/dev/null || echo unknown)"
if [ "$OPERSTATE" != "up" ]; then
  warn "接口 $CAN_IF 状态为 $OPERSTATE（预期 up）"
fi
# 状态回显：bitrate 与 can state 在 `ip -details` 里**分属两行**，必须分开抓
# （旧写法用单条正则 'bitrate N can state X' 跨行匹配，永远抓不到 → 现场看到一行空白）
BITRATE_NOW="$( $SUDO ip -details link show "$CAN_IF" 2>/dev/null | grep -o 'bitrate [0-9]*' | head -1 )"
BUS_STATE="$( $SUDO ip -details link show "$CAN_IF" 2>/dev/null | grep -o 'can state [A-Z-]*' | head -1 )"
log "$CAN_IF 已启用：operstate=${OPERSTATE} ${BITRATE_NOW:-bitrate 未知} ${BUS_STATE:-can state 未知}"

# ---- ③ 抓帧验证：只有收到底盘反馈帧才说明底盘通信真的建立 ----
if ! command -v candump >/dev/null 2>&1; then
  if [ "$REQUIRE_FRAMES" -eq 1 ]; then
    warn "candump 不可用（缺 can-utils）：无法验证底盘帧，按 --no-strict 语义继续"
    warn "  处置：sudo apt install -y can-utils"
  fi
  exit 0
fi

if [ "$QUICK" -eq 1 ]; then
  log "跳过抓帧（--quick）"
  exit 0
fi

log "抓取 ${CAN_IF} 报文（最多 ${FRAME_TIMEOUT}s，期望 ID：$EXPECT_IDS）"
DUMP="$(timeout "$FRAME_TIMEOUT" candump "$CAN_IF" 2>/dev/null || true)"
HITS="$(printf '%s\n' "$DUMP" | grep -E "$EXPECT_IDS" | head -5 || true)"

if [ -n "$HITS" ]; then
  log "✓ 已收到底盘反馈帧（示例）："
  printf '%s\n' "$HITS" | sed 's/^/    /'
  exit 0
fi

# 抓到了报文但都不是底盘反馈 ID：也要说清楚（可能是别的设备在同一条总线）
if [ -n "$DUMP" ]; then
  warn "接口上有报文但未见底盘反馈帧 ID（$EXPECT_IDS）："
  printf '%s\n' "$DUMP" | head -5 | sed 's/^/    /'
else
  warn "接口上没有任何报文（底盘未上电 / 接线 / 终端电阻 / 波特率不符）"
fi
if [ "$REQUIRE_FRAMES" -eq 1 ]; then
  warn "严格模式：底盘通信未建立 → 退出码 3（台架调试可加 --no-strict 只要求接口 up）"
  exit 3
fi
warn "非严格模式：仅接口 up，继续（底盘通信未证实）"
exit 0
