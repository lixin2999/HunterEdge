#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""SIL 会话状态机：sim_id 绑定、传感器断流看门狗、控制回传序号（V0.1.38）。

与 :mod:`sil_bridge_agent.codec` 一样**不依赖 rclpy / confluent_kafka**，
纯逻辑可单测。ROS 节点只负责把这里的判定结果落到发布/生产动作上。

────────────────────────────────────────────────────────────────────────
三条安全语义（与 HunterSim B2 / 车端 safety_guard 对齐）
────────────────────────────────────────────────────────────────────────
1. **会话绑定**：正则订阅 ``^carla\..+\.edge_sensor$`` 会收到所有仿真实例的帧，
   必须按 ``session`` boot 帧绑定唯一 sim_id，之后只处理该实例的数据；
   ``sim_id_filter`` 非空时进一步限定（多车/多实例共用 broker 的隔离手段）。
2. **传感器断流停发**：上行断流 > ``watchdog_ms``（默认 500ms）时**停发 ROS 帧**，
   让车端 ``safety_guard`` 的 ``scan_timeout``/``cmd_timeout`` 超时停车语义自然生效
   （而不是由桥接层伪造一帧零速点云——伪造数据会让感知以为「前方空旷」）。
   控制回传**不停**：继续转发全栈产出的 /cmd_vel（此时 safety_guard 已输出零速），
   仿真端因此拿到新鲜控制帧，由它自己的看门狗决定刹停，两侧语义不互相掩盖。
3. **stop 补零速**：收到会话 stop 帧后停发，并**补恰好一帧零速**控制，
   确保仿真自车不会沿用最后一帧指令滑行（与 remote_agent 的交还驾驶权收尾同构）。
"""
from __future__ import annotations

import threading
import time
from typing import Any, Dict, Optional

from .codec import (
    build_control_frame,
    envelope_meta,
    session_phase,
    session_sim_id,
    sim_id_from_topic,
)

#: 默认看门狗（毫秒）：与仿真端 ``EdgeLinkSettings.watchdog_ms`` 和车端
#: ``safety_guard`` 的 ``scan_timeout``/``cmd_timeout``（0.5s）同口径。
DEFAULT_WATCHDOG_MS = 500

#: 断流告警限频间隔（秒）：避免 500ms 一次的看门狗判定刷屏日志。
STALE_LOG_INTERVAL_S = 5.0


class SessionState:
    """单会话状态机（线程安全：Kafka 消费线程写、ROS 定时器线程读）。

    Args:
        watchdog_ms: 传感器断流阈值（毫秒），<=0 表示关闭看门狗（仅联调用）。
        sim_id_filter: 只接受该 sim_id 的会话（空串=接受首个 boot 帧）。
        stale_log_interval_s: 断流告警限频间隔。
    """

    def __init__(
        self,
        *,
        watchdog_ms: int = DEFAULT_WATCHDOG_MS,
        sim_id_filter: str = "",
        stale_log_interval_s: float = STALE_LOG_INTERVAL_S,
    ) -> None:
        self._lock = threading.RLock()
        self._watchdog_s = max(0.0, float(watchdog_ms) / 1000.0)
        self._sim_id_filter = str(sim_id_filter or "")
        self._stale_log_interval_s = max(0.0, float(stale_log_interval_s))

        self._sim_id = ""
        self._active = False
        self._scene_id = ""
        self._map_id = ""
        self._last_sensor_ts: Optional[float] = None
        self._stop_zero_pending = False
        self._seq = 0

        # 诊断计数（health/event 上报与本地日志用）
        self._sensor_frames: Dict[str, int] = {}
        self._control_frames = 0
        self._rejected_frames = 0
        self._stale_episodes = 0
        self._stale = False
        self._last_stale_log_ts = 0.0
        self._boot_ts: Optional[float] = None
        self._stop_ts: Optional[float] = None

    # ── 只读视图 ────────────────────────────────────────────────────────────

    @property
    def sim_id(self) -> str:
        """当前绑定的仿真实例 ID（未绑定为空串）。"""
        with self._lock:
            return self._sim_id

    @property
    def active(self) -> bool:
        """会话是否存续（boot 置真、stop 置假）。"""
        with self._lock:
            return self._active

    @property
    def sensor_topic(self) -> str:
        """绑定实例的上行 Topic（未绑定为空串）。"""
        with self._lock:
            return f"carla.{self._sim_id}.edge_sensor" if self._sim_id else ""

    @property
    def control_topic(self) -> str:
        """绑定实例的下行 Topic（未绑定为空串）。"""
        with self._lock:
            return f"carla.{self._sim_id}.edge_control" if self._sim_id else ""

    # ── 会话信令 ────────────────────────────────────────────────────────────

    def handle_session(self, envelope: Dict[str, Any], now: Optional[float] = None) -> str:
        """处理 session 帧，返回相位（``boot``/``stop``/``""``=非会话或非法）。

        boot 幂等：同一 sim_id 重复 boot 不重置看门狗与计数（仿真端重连会重发）；
        不同 sim_id 的 boot 视为**会话切换**，重置全部状态（避免串实例数据）。
        """
        phase = session_phase(envelope)
        if phase is None:
            return ""
        now = time.time() if now is None else now
        meta = envelope_meta(envelope)
        incoming = session_sim_id(envelope) or sim_id_from_topic(envelope.get("_topic", ""))
        with self._lock:
            if self._sim_id_filter and incoming and incoming != self._sim_id_filter:
                self._rejected_frames += 1
                return ""
            if phase == "boot":
                if self._active and incoming and incoming != self._sim_id:
                    self._reset_locked()
                if incoming:
                    self._sim_id = incoming
                self._active = True
                self._stop_zero_pending = False
                self._scene_id = str(meta.get("scene_id") or "")
                self._map_id = str(meta.get("map_id") or "")
                self._boot_ts = now
                self._stop_ts = None
                # boot 即视为链路新鲜：仿真端随后立刻馈送传感器，
                # 若这里不置位，首个 500ms 窗口内会被误判断流而停发。
                self._last_sensor_ts = now
                self._stale = False
                return "boot"
            # stop：只对已绑定实例生效（未 boot 就收到 stop 属噪声）
            if not self._active:
                return ""
            if incoming and self._sim_id and incoming != self._sim_id:
                self._rejected_frames += 1
                return ""
            self._active = False
            self._stop_zero_pending = True
            self._stop_ts = now
            return "stop"

    def _reset_locked(self) -> None:
        """会话切换时清空状态（调用方持锁）。"""
        self._sim_id = ""
        self._active = False
        self._scene_id = ""
        self._map_id = ""
        self._last_sensor_ts = None
        self._stop_zero_pending = False
        self._sensor_frames.clear()
        self._control_frames = 0
        self._stale_episodes = 0
        self._stale = False
        self._boot_ts = None
        self._stop_ts = None

    # ── 帧准入 ──────────────────────────────────────────────────────────────

    def accepts(self, topic: str) -> bool:
        """该 Topic 的帧是否属于当前会话（正则订阅下的实例隔离）。"""
        with self._lock:
            if not self._active or not self._sim_id:
                return False
            incoming = sim_id_from_topic(topic)
            return bool(incoming) and incoming == self._sim_id

    def note_sensor(self, stream: str, now: Optional[float] = None) -> None:
        """记录一帧已接受的传感器（刷新看门狗时间戳 + 分流计数）。"""
        now = time.time() if now is None else now
        with self._lock:
            self._last_sensor_ts = now
            self._sensor_frames[stream] = self._sensor_frames.get(stream, 0) + 1
            if self._stale:
                self._stale = False

    def reject(self) -> None:
        """记录一帧被拒（非本会话/未绑定），供诊断定位串实例问题。"""
        with self._lock:
            self._rejected_frames += 1

    # ── 看门狗 ──────────────────────────────────────────────────────────────

    def sensor_fresh(self, now: Optional[float] = None) -> bool:
        """传感器链路是否新鲜（决定能否发布 ROS 帧）。

        未 boot / 已 stop / 从未收到传感器 / 断流超阈值 → False。
        ``watchdog_ms<=0`` 时只要求会话活跃且收到过帧（关闭超时判定，仅联调）。
        """
        now = time.time() if now is None else now
        with self._lock:
            if not self._active or self._last_sensor_ts is None:
                return False
            if self._watchdog_s <= 0.0:
                return True
            return (now - self._last_sensor_ts) <= self._watchdog_s

    def poll_stale(self, now: Optional[float] = None) -> bool:
        """看门狗判定并返回「是否应打印一次断流告警」（限频，避免刷屏）。

        与 :meth:`sensor_fresh` 分开：前者是纯判定（热路径每帧调用），
        本方法额外维护 episode 计数与日志节流（低频定时器调用）。
        """
        now = time.time() if now is None else now
        with self._lock:
            fresh = self.sensor_fresh(now)
            if fresh:
                self._stale = False
                return False
            if not self._active:
                return False
            if not self._stale:
                self._stale = True
                self._stale_episodes += 1
                self._last_stale_log_ts = now
                return True
            if now - self._last_stale_log_ts >= self._stale_log_interval_s:
                self._last_stale_log_ts = now
                return True
            return False

    # ── 控制回传 ────────────────────────────────────────────────────────────

    def pop_control(
        self,
        linear_x: float,
        angular_z: float,
        now: Optional[float] = None,
    ) -> Optional[Dict[str, Any]]:
        """构造一帧下行控制；不该发时返回 None。

        * 会话活跃：按入参构造，seq 严格递增（仿真端按 seq 去重）。
        * stop 待收尾：**强制零速**并消耗掉该待办（恰好一帧），之后返回 None。
        * 未 boot / 已收尾：None。
        """
        now = time.time() if now is None else now
        with self._lock:
            if self._stop_zero_pending:
                self._stop_zero_pending = False
                self._seq += 1
                self._control_frames += 1
                return build_control_frame(0.0, 0.0, self._seq, int(now * 1e9))
            if not self._active or not self._sim_id:
                return None
            self._seq += 1
            self._control_frames += 1
            return build_control_frame(linear_x, angular_z, self._seq, int(now * 1e9))

    def control_pending(self) -> bool:
        """是否还欠一帧 stop 零速收尾（节点退出前据此补发）。"""
        with self._lock:
            return self._stop_zero_pending

    def close(self, now: Optional[float] = None) -> bool:
        """进程退出收尾：等价于收到 stop 帧（活跃则置停并欠一帧零速）。

        覆盖「没收到 stop 帧就退出」的路径（SIGINT / systemd 重启 / 崩溃恢复）：
        不补零速时仿真自车会沿用末帧指令滑行到看门狗超时
        （500ms × 3.5m/s ≈ 1.75m）。与 stop 帧共用同一待办，故**幂等**：
        第二次调用不再补帧；未 boot 时返回 False（不得凭空产生控制帧）。
        """
        now = time.time() if now is None else now
        with self._lock:
            if not self._active:
                return False
            self._active = False
            self._stop_zero_pending = True
            self._stop_ts = now
            return True

    # ── 诊断 ────────────────────────────────────────────────────────────────

    def stats(self, now: Optional[float] = None) -> Dict[str, Any]:
        """链路诊断快照（脱敏；供 health 上报与本地日志）。"""
        now = time.time() if now is None else now
        with self._lock:
            age_ms = (
                round((now - self._last_sensor_ts) * 1000.0, 1)
                if self._last_sensor_ts is not None
                else None
            )
            return {
                "active": self._active,
                "sim_id": self._sim_id,
                "scene_id": self._scene_id,
                "map_id": self._map_id,
                "sensor_topic": self.sensor_topic,
                "control_topic": self.control_topic,
                "watchdog_ms": int(self._watchdog_s * 1000),
                "sensor_age_ms": age_ms,
                "sensor_fresh": self.sensor_fresh(now),
                "stale": self._stale,
                "stale_episodes": self._stale_episodes,
                "sensor_frames": dict(self._sensor_frames),
                "control_frames": self._control_frames,
                "rejected_frames": self._rejected_frames,
                "stop_zero_pending": self._stop_zero_pending,
                "uptime_s": round(now - self._boot_ts, 3) if self._boot_ts else None,
            }


class StreamRateLimiter:
    """各流 ROS 发布限频（保护 Jetson CPU：上行 10Hz 不代表下游必须 10Hz）。

    ``fps<=0`` 表示不限频。与仿真端 ``EdgeLinkRuntime`` 的限频相互独立：
    仿真端限的是**上行带宽**，这里限的是**车端发布/感知负载**。
    """

    def __init__(self, fps_by_stream: Optional[Dict[str, float]] = None) -> None:
        self._fps = {str(k): float(v) for k, v in (fps_by_stream or {}).items()}
        self._last: Dict[str, float] = {}
        self._dropped: Dict[str, int] = {}
        self._lock = threading.Lock()

    def allow(self, stream: str, now: Optional[float] = None) -> bool:
        """是否放行该帧（放行则刷新时间戳；限频丢弃计入诊断）。"""
        now = time.time() if now is None else now
        fps = self._fps.get(stream, 0.0)
        with self._lock:
            if fps <= 0.0:
                self._last[stream] = now
                return True
            last = self._last.get(stream)
            interval = 1.0 / fps
            # ⚠ 必须留时间戳量级容差：生产用 time.time()（≈1.7e9），float64 在该
            #   量级的表示误差可达 ~4e-7s。不留容差时**恰好按标称周期到达**的帧会
            #   被算成「早到」而丢弃——上行 10Hz 配限频 10Hz 会退化成约 5Hz，
            #   表现为感知输入莫名减半且无任何报错。
            tol = max(1e-9, abs(now) * 1e-15)
            if last is not None and (now - last) < interval - tol:
                self._dropped[stream] = self._dropped.get(stream, 0) + 1
                return False
            self._last[stream] = now
            return True

    @property
    def dropped(self) -> Dict[str, int]:
        """各流限频丢弃计数。"""
        with self._lock:
            return dict(self._dropped)


__all__ = [
    "DEFAULT_WATCHDOG_MS", "STALE_LOG_INTERVAL_S",
    "SessionState", "StreamRateLimiter",
]
