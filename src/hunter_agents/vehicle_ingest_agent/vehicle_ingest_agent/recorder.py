# -*- coding: utf-8 -*-
"""分段录制监管（需求 2 核心；设计 §5.2 seal 判定）。

录制进程持续写盘，本监管器把它切成**相互独立、可单独上传/删除**的片段：
  - 每段以 ``ros2 bag record`` 起一个自包含 rosbag2 目录（独立 metadata.yaml），
    bag_id = ``bag_{yyyyMMddHHmmss}_{vehicle_id}_{seq4}``（设计 §4，全局唯一）；
  - 巡检（``recording.poll_interval_s``）当前段的**体积**（各 .db3 之和）与**时长**，
    任一达到 ``max_segment_bytes``(500MB) / ``max_segment_duration_s``(10min) → 封段；
  - 封段 = 对 record 进程发 SIGINT（rosbag2 收尾写 metadata.yaml），等待
    ``seal_grace_s`` 确认 sealed（§5.2），登记到 SQLite ``segments``/``tasks``
    并回调下游（verify → manifest → upload）；随即另起下一段 → 形成环形缓存供给端。

同时给 rosbag2 传 ``--max-bag-size/--max-bag-duration`` 作为进程内硬兜底；
但真正的"独立片段可删"语义由监管器的封段/另起保证（rosbag2 同目录分片不可中间删除）。

``record_transport=none`` 时不起录制进程（由外部 ROS 录制组件产出），监管器仅负责
把已存在的 sealed bag 目录纳入环形缓存治理（发现即登记）。
"""
from __future__ import annotations

import asyncio
import os
import signal
import time
from datetime import datetime
from typing import Any, Awaitable, Callable, Dict, Optional

from .logging_util import get_logger

logger = get_logger("recorder")

DbSealed = Callable[[Dict[str, Any]], Awaitable[None]]
Alerter = Callable[[str, str, Dict[str, Any]], None]


def _dir_size_bytes(path: str) -> int:
    total = 0
    for root, _dirs, files in os.walk(path):
        for f in files:
            fp = os.path.join(root, f)
            try:
                total += os.path.getsize(fp)
            except OSError:
                pass
    return total


def _find_db_files(path: str) -> int:
    n = 0
    for _root, _dirs, files in os.walk(path):
        for f in files:
            if f.endswith((".db3", ".bag")):
                n += 1
    return n


class SegmentRecorder:
    def __init__(self, cfg, store, on_sealed: DbSealed, alert: Optional[Alerter] = None):
        self.cfg = cfg
        self.store = store
        self.on_sealed = on_sealed
        self.alert = alert or (lambda *a, **k: None)
        self.vehicle_id = cfg.vehicle_id

        self.enabled = bool(cfg.get("recording.enabled", True))
        self.transport = cfg.get("recording.record_transport", "ros2")
        self.topics = list(cfg.get("recording.topics", []) or [])
        self.max_bytes = int(cfg.get("recording.max_segment_bytes"))
        self.max_dur = float(cfg.get("recording.max_segment_duration_s"))
        self.poll_s = float(cfg.get("recording.poll_interval_s"))
        self.seal_grace = float(cfg.get("recording.seal_grace_s"))
        self.min_dur = float(cfg.get("recording.min_sealed_duration_s"))

        self.bags_root = cfg.bags_dir
        self._proc: Optional[asyncio.subprocess.Process] = None
        self._cur: Optional[Dict[str, Any]] = None      # 当前在录片段
        self._running = False
        self._accept_new = True                          # pause/resume 门控（§6.8.5）
        self._task: Optional[asyncio.Task] = None
        self._protected_pending = False                 # 关键事件：当前段封段时标 protected

    # ── bag_id 生成（§4 命名；seq 持久化跨重启唯一）──
    def _next_bag_id(self, when: float) -> str:
        dt = datetime.utcfromtimestamp(when)
        seq = int(self.store.meta_get("recorder.seq", "0")) + 1
        self.store.meta_set("recorder.seq", str(seq))
        return "bag_%s_%s_%04d" % (dt.strftime("%Y%m%d%H%M%S"), self.vehicle_id, seq)

    def _segment_dir(self, bag_id: str, when: float) -> str:
        date = datetime.utcfromtimestamp(when).strftime("%Y-%m-%d")
        return os.path.join(self.bags_root, self.vehicle_id, date, bag_id)

    # ── 生命周期 ──
    async def start(self) -> None:
        os.makedirs(self.bags_root, exist_ok=True)
        self._running = True
        if self.transport == "ros2":
            await self._start_segment()
        self._task = asyncio.create_task(self._poll_loop(), name="recorder-poll")
        logger.info("分段录制监管启动 transport=%s max_bytes=%d max_dur=%.0fs topics=%s",
                    self.transport, self.max_bytes, self.max_dur, self.topics,
                    extra={"component": "recorder"})

    async def stop(self) -> None:
        self._running = False
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
        if self._cur is not None:
            await self._seal_current(drain=True)   # drain：SIGINT 收尾（systemd KillSignal=SIGINT）
        logger.info("分段录制监管已停止", extra={"component": "recorder"})

    # ── 起/封段 ──
    async def _start_segment(self) -> None:
        when = time.time()
        bag_id = self._next_bag_id(when)
        seg_dir = self._segment_dir(bag_id, when)
        os.makedirs(seg_dir, exist_ok=True)
        # 进程内硬兜底：即使监管器 tick 延迟，rosbag2 也会自切；但监管器仍负责"独立可删段"
        cmd = ["ros2", "bag", "record", "-o", seg_dir,
               "--max-bag-size", str(self.max_bytes),
               "--max-bag-duration", str(int(self.max_dur))]
        cmd += self.topics
        try:
            proc = await asyncio.create_subprocess_exec(
                *cmd, stdout=asyncio.subprocess.DEVNULL,
                stderr=asyncio.subprocess.DEVNULL,
                start_new_session=True)   # 独立进程组，便于整组 SIGINT
        except (OSError, FileNotFoundError) as exc:
            self.alert("E_RECORDER_SPAWN", "critical", {"error": str(exc), "cmd": " ".join(cmd)})
            logger.error("ros2 bag record 启动失败：%s", exc, extra={"component": "recorder"})
            self._cur = None
            return
        self._proc = proc
        self._cur = {
            "bag_id": bag_id, "path": seg_dir, "start_wall": when,
            "start_ns": int(when * 1e9), "proc": proc,
        }
        self._protected_pending = False
        logger.info("开始录制片段 %s → %s", bag_id, seg_dir,
                    extra={"bag_id": bag_id, "component": "recorder"})

    async def _seal_current(self, drain: bool = False) -> None:
        cur = self._cur
        proc = self._proc
        if cur is None:
            return
        self._cur = None
        self._proc = None
        seg_dir = cur["path"]
        # 1) SIGINT → rosbag2 flush + 写 metadata.yaml（§5.2 sealed 判定）
        await self._terminate(proc, drain=drain)
        sealed_ok = await self._await_metadata(seg_dir)
        end_wall = time.time()
        duration = end_wall - cur["start_wall"]
        size = _dir_size_bytes(seg_dir)
        if not sealed_ok:
            self.alert("E_BAG_CORRUPT", "error",
                       {"bag_id": cur["bag_id"], "reason": "metadata.yaml 未生成"})
            logger.warning("片段未 seal 成功（缺 metadata），保留现场不登记",
                           extra={"bag_id": cur["bag_id"], "component": "recorder"})
            return
        if duration < self.min_dur or _find_db_files(seg_dir) == 0:
            logger.warning("空/过短片段丢弃 duration=%.1fs size=%d", duration, size,
                           extra={"bag_id": cur["bag_id"], "component": "recorder"})
            return
        # 2) 登记 segments + tasks（discovered），交下游
        self.store.register_segment(
            bag_id=cur["bag_id"], path=seg_dir, size_bytes=size,
            start_ns=cur["start_ns"], end_ns=int(end_wall * 1e9),
            duration_s=duration, state="discovered", priority=4,
            protected=self._protected_pending)
        self.store.create_task(cur["bag_id"], state="discovered")
        seg = dict(self.store.get_segment(cur["bag_id"]))
        logger.info("片段已封存 bag_id=%s size=%d duration=%.1fs protected=%s",
                    cur["bag_id"], size, duration, seg.get("protected"),
                    extra={"bag_id": cur["bag_id"], "component": "recorder",
                           "task_state": "discovered"})
        await self.on_sealed(seg)

    async def _terminate(self, proc: asyncio.subprocess.Process, drain: bool) -> None:
        if proc.returncode is not None:
            return
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)  # 整组 SIGINT（start_new_session）
        except (ProcessLookupError, PermissionError, OSError):
            try:
                proc.send_signal(signal.SIGINT)
            except ProcessLookupError:
                return
        try:
            await asyncio.wait_for(proc.wait(), timeout=self.seal_grace + (5.0 if drain else 0))
        except asyncio.TimeoutError:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError, OSError):
                proc.kill()

    async def _await_metadata(self, seg_dir: str) -> bool:
        """轮询 metadata.yaml 出现（rosbag2 收尾产物）= sealed（§5.2）。"""
        meta = os.path.join(seg_dir, "metadata.yaml")
        deadline = time.time() + self.seal_grace
        while time.time() < deadline:
            if os.path.isfile(meta) and os.path.getsize(meta) > 0:
                return True
            await asyncio.sleep(0.2)
        return os.path.isfile(meta) and os.path.getsize(meta) > 0

    # ── 巡检：体积/时长达到阈值即切分 ──
    async def _poll_loop(self) -> None:
        while self._running:
            try:
                await asyncio.sleep(self.poll_s)
                if self.transport != "ros2":
                    continue
                if self._cur is None:
                    if self._running and self.enabled and self._accept_new:
                        await self._start_segment()
                    continue
                elapsed = time.time() - self._cur["start_wall"]
                size = _dir_size_bytes(self._cur["path"])
                if size >= self.max_bytes or elapsed >= self.max_dur:
                    reason = "size" if size >= self.max_bytes else "duration"
                    logger.info("触发切分（%s）size=%d elapsed=%.1fs", reason, size, elapsed,
                                extra={"bag_id": self._cur["bag_id"], "component": "recorder"})
                    await self._seal_current()
                    if self._running and self.enabled and self._accept_new:
                        await self._start_segment()
            except asyncio.CancelledError:
                break
            except Exception as exc:  # noqa: BLE001  监管循环不得因单次异常退出
                logger.exception("录制巡检异常：%s", exc, extra={"component": "recorder"})
                await asyncio.sleep(1.0)

    # ── 关键事件：把当前/指定片段标记 protected（需求 2：不自动删除）──
    def mark_current_protected(self, reason: str) -> None:
        self._protected_pending = True
        if self._cur:
            self.store.update_segment(self._cur["bag_id"], protected=True)
        logger.info("已标记保护片段（关键事件）：%s reason=%s",
                    self._cur["bag_id"] if self._cur else "<下一段>", reason,
                    extra={"bag_id": self._cur["bag_id"] if self._cur else None,
                           "component": "recorder"})

    def protect(self, bag_id: str) -> None:
        if self._cur and self._cur["bag_id"] == bag_id:
            self._protected_pending = True
        self.store.update_segment(bag_id, protected=True)

    # ── 控制钩子（供命令 / 优雅退出）──
    async def force_seal(self) -> None:
        """远程 seal（§5.2 兜底：录制进程异常退出时强制封存当前段）。"""
        if self._cur is not None:
            await self._seal_current()
            if self._running and self.enabled and self._accept_new:
                await self._start_segment()

    def pause(self) -> None:
        self._accept_new = False
        logger.info("录制已暂停（不再另起新片段）", extra={"component": "recorder"})

    def resume(self) -> None:
        self._accept_new = True
        logger.info("录制已恢复", extra={"component": "recorder"})
