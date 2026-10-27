#!/usr/bin/env bash
# hunter_clock_gate.sh — 上电时钟闸门（V0.1.34；供 hunter-edge.service 的 ExecStartPre 调用）
#
# 为什么需要（2026-10-27 实机事故，release.md V0.1.33 §8）：
#   本机 RTC 掉电不保持时间。10-26 17:13 断电 → 10-27 08:46 上电，内核时钟退回
#   “Jan 01 08:00”，systemd-timesyncd 用 /var/lib/systemd/timesync/clock 把时钟回填成
#   10-26 17:13:55，而 hunter-edge.service 只依赖 hunter-can + network-online，开机约 19s
#   就把整栈拉起。约 17s 后 NTP 首同步（1.pool.ntp.org）把时钟**向前拨了 55970s（15h32m）**：
#     · 跳变前产生的点云 T 时间戳 → 全落进“过去”，TF 缓存里只有最近 10s
#     · MessageFilter 因此整帧丢弃 → costmap 报 observation buffer has not been updated
#       for 55970.22 seconds → 代价地图全空 → 导航心盲（现场观感：“雷达闪一下即停”）
#     · FAST-LIO2 配准失效（No Effective Points!）→ 里程计发散到 845km → 5'40" 后 SIGSEGV
#     · NDT 在同一个 0.5s 内从 fit=3.292 炸到 9.4e18
#   软件侧唯一的正解：**等时钟同步完成，再起栈**。
#
# 设计取舍：
#   • 有界等待（默认上限 60s）：NTP 服务器不可达时**绝不**卡住启动——车不能因为对不上时
#     就永远进不了待命态。超时只 WARN，然后照常启动。
#   • 因此**不**采用 `After=time-sync.target` + `systemd-time-wait-sync.service` 的硬等写法：
#     那套在 NTP 不可达时会无限阻塞（等价于车永远不启动）。
#   • 同步完成后再报「闸门前 → 闸门后」的修正量：若 >1s 说明本机时钟又不准，日志里一眼可见，
#     不必等故障复现再猜。
#
# 用法：
#   bash hunter_clock_gate.sh            # 等待（上限 HUNTER_CLOCK_GATE_TIMEOUT，默认 60s）
#   bash hunter_clock_gate.sh --check    # 只体检不等待（现场排查）
#
# 环境变量（可写入 /etc/hunter/agent_env.sh）：
#   HUNTER_CLOCK_GATE_TIMEOUT  等待上限秒数（默认 60；0 = 不等待）
#   HUNTER_CLOCK_GATE_SYSTOHC  1 = 同步后把系统时间回写 RTC（默认 0；内核 11 分钟模式也会写）
#
# 附带的 RTC 硬件体检（V0.1.36）：每次运行都查一遍内核 `setting system clock to …` 那行。
#   读到 1970 年就说明 RTC 后备电源根本没供电——实测本机 AGX Orin 是
#   `nvvrs-pseq-rtc: setting system clock to 1970-01-01T00:00:36 UTC (36)`，即每次上电
#   计数器都从 0 起（36 秒是 NVVRS 上电到内核读取的时长）。
#   之所以必须放在“已同步 → 直接 return”之前：systemd-timesyncd 会用
#   /var/lib/systemd/timesync/clock 把系统时钟回填成上次的值，回填之后 wall clock 看着
#   完全正常，`timedatectl` 也报 synchronized=yes —— 只有内核那一行还留着真相。
#
# 退出码「恒为 0」：时钟问题只告警、不阻断整栈启动。判据看日志。
set -uo pipefail

TIMEOUT="${HUNTER_CLOCK_GATE_TIMEOUT:-60}"
SYSTOHC="${HUNTER_CLOCK_GATE_SYSTOHC:-0}"
WAIT=1

while [ $# -gt 0 ]; do
  case "$1" in
    --check|--no-wait) WAIT=0; shift ;;
    -h|--help)         sed -n '/^set -uo pipefail/q; p' "$0"; exit 0 ;;
    *)                 echo "[clock] 未知参数: $1（--help 看用法）" >&2; exit 0 ;;
  esac
done

log()  { echo "[clock] $*"; }
warn() { echo "[clock] ! $*" >&2; }

# TIMEOUT 非数字时退回默认值，避免算术比较报错
case "$TIMEOUT" in
  ''|*[!0-9]*) warn "HUNTER_CLOCK_GATE_TIMEOUT='$TIMEOUT' 非整数，按默认 60s"; TIMEOUT=60 ;;
esac

if ! command -v timedatectl >/dev/null 2>&1; then
  warn "无 timedatectl（非 systemd 托管环境？）：跳过时钟闸门"
  exit 0
fi

read_sync() { timedatectl show -p NTPSynchronized --value 2>/dev/null || echo unknown; }

# ---- RTC 硬件体检（V0.1.36）：无条件执行，与“本次是否已同步”无关 ----
# 判据① 内核启动时从 /dev/rtc0 读到的时间；判据② hwclock -r 能否读到设备。
rtc_hw_check() {
  local line=""
  if command -v journalctl >/dev/null 2>&1; then
    line="$(journalctl -k -b --no-pager 2>/dev/null \
      | sed -n 's/.*setting system clock to \([0-9-]*T[0-9:]*\) UTC.*/\1/p' | head -1)"
  elif command -v dmesg >/dev/null 2>&1; then
    line="$(dmesg 2>/dev/null \
      | sed -n 's/.*setting system clock to \([0-9-]*T[0-9:]*\) UTC.*/\1/p' | head -1)"
  fi
  case "$line" in
    ''|2[0-9][0-9][0-9]-*) ;;   # 读不到（无权限/无该行）或 2000 年以后：不吭声
    *)
      warn "本次上电内核从 RTC 只读到 $line ⇒ RTC 后备电源失效（断电期间计数器从 0 重来）"
      warn "  ⇒ 属硬件问题：RTC 电池/超级电容未装、装反或电压不足（唯一根治项，软件补不了）"
      warn "  ⇒ 软件侧只能靠本闸门等 NTP 首次校正；校正前整栈发出的时间戳都是错的"
      ;;
  esac
  if command -v hwclock >/dev/null 2>&1 && ! hwclock -r >/dev/null 2>&1; then
    warn "RTC 不可读取（/dev/rtc0 异常）：属硬件问题（RTC 电池/超级电容/未装）"
  fi
}
rtc_hw_check

t0="$(date +%s)"
before="$(date '+%Y-%m-%d %H:%M:%S')"
status="$(read_sync)"

if [ "$status" = "yes" ]; then
  log "时钟已同步（NTPSynchronized=yes，$before）：无需等待"
  exit 0
fi

log "时钟未同步（NTPSynchronized=$status，当前 $before）"

if [ "$WAIT" -eq 0 ]; then
  warn "按参数跳过等待（--check）：若此后时钟被拨动，整栈时间轴会断裂（release.md V0.1.33 §8）"
  exit 0
fi
if [ "$TIMEOUT" -eq 0 ]; then
  warn "HUNTER_CLOCK_GATE_TIMEOUT=0：不等待，照常启动（风险同上）"
  exit 0
fi

log "等待 NTP 首同步，上限 ${TIMEOUT}s（超时只告警、不阻断启动）"
elapsed=0
while [ "$elapsed" -lt "$TIMEOUT" ]; do
  sleep 1
  elapsed=$(( elapsed + 1 ))
  status="$(read_sync)"
  [ "$status" = "yes" ] && break
done

t1="$(date +%s)"
after="$(date '+%Y-%m-%d %H:%M:%S')"
# 修正量 = 两读数的钟面差 − 真实流逝（sleep 计数），跳变时为数千秒级
delta=$(( t1 - t0 - elapsed ))

if [ "$status" = "yes" ]; then
  if [ "$delta" -ge 2 ] || [ "$delta" -le -2 ]; then
    warn "时钟已同步：$before → $after，等待 ${elapsed}s 期间被修正 ${delta}s"
    warn "  ⇒ 本机 RTC 掉电不保持时间（查 RTC 电池 / sudo hwclock -r）。本次已避免“整栈带着错误时钟启动”"
  else
    log "时钟已同步（等待 ${elapsed}s，无跳变）：$after"
  fi
  if [ "$SYSTOHC" = "1" ] && command -v hwclock >/dev/null 2>&1; then
    if hwclock --systohc 2>/dev/null; then
      log "已回写 RTC（hwclock --systohc）"
    else
      warn "回写 RTC 失败（需要 root/CAP_SYS_TIME）"
    fi
  fi
else
  warn "等待 ${TIMEOUT}s 仍未同步（$after）：按设计不阻断启动"
  warn "  风险：若稍后 NTP 才同步且跳变量较大，会出现整栈时间轴断裂（MessageFilter 整帧丢弃 → 代价地图空 → FAST-LIO2 发散）"
  warn "  现场处置：timedatectl timesync-status 查 NTP 可达性；已断裂则 sudo systemctl restart hunter-edge"
fi

exit 0
