# -*- coding: utf-8 -*-
"""心跳与告警（设计 §6.11、§8）。

每 ``telemetry.heartbeat_interval_s``（默认 30s）上报 ``vehicle.agent.heartbeat``：
CPU 负载 / RSS / 磁盘水位 / 队列深度（outbox pending）/ 在盘片段数与字节 /
任务计数（各状态）/ config_rev / 上传在途 / kafka 可用性。
顺带做 outbox ``done`` 行的 TTL 清理（§6.8.1：done 24h 后清理）。

指标采集仅用 stdlib（resource/os/shutil），无 psutil 依赖也能跑（§9 资源可控）。
"""
from __future__ import annotations

import asyncio
import os
import time
from typing import Any, Callable, Dict, Optional

from .logging_util import get_logger

logger = get_logger("heartbeat")


def _rss_mb() -> float:
    try:
        import resource
        peak = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss  # Linux: KB
        return peak / 1024.0
    except Exception:  # noqa: BLE001
        return 0.0


class Heartbeat:
    def __init__(self, cfg, store, machine, spool,
                 extra_metrics: Optional[Callable[[], Dict[str, Any]]] = None):
        self.cfg = cfg
        self.store = store
        self.machine = machine
        self.spool = spool
        self.interval = float(cfg.get("telemetry.heartbeat_interval_s", 30))
        self.extra = extra_metrics or (lambda: {})
        self.running = False

    async def run(self) -> None:
        self.running = True
        done_ttl = float(self.cfg.get("kafka.outbox.done_ttl_h", 24)) * 3600
        logger.info("心跳任务启动（%.0fs 周期）", self.interval, extra={"component": "heartbeat"})
        while self.running:
            try:
                metrics = self._collect()
                self.machine.emit_heartbeat(metrics)
                # done outbox TTL 清理（§6.8.1）
                self.store.purge_done_before(time.time() - done_ttl)
                # pending 积压告警（不清理，§6.8.1）
                if metrics.get("outbox_pending", 0) > int(
                        self.cfg.get("kafka.outbox.max_pending", 500000)):
                    self.machine.emit_alert("E_OUTBOX_BACKLOG", "warning",
                                            {"pending": metrics["outbox_pending"]})
            except asyncio.CancelledError:
                break
            except Exception as exc:  # noqa: BLE001
                logger.exception("心跳采集异常：%s", exc, extra={"component": "heartbeat"})
            try:
                await asyncio.sleep(self.interval)
            except asyncio.CancelledError:
                break

    def _collect(self) -> Dict[str, Any]:
        ds = self.spool.disk_status()
        pending = self.store.count_pending()
        seg_count = self.store.count_segments()
        seg_bytes = self.store.total_segment_bytes()
        state_counts: Dict[str, int] = {}
        for t in self.store.list_nonterminal_tasks():
            state_counts[t["state"]] = state_counts.get(t["state"], 0) + 1
        try:
            load1, load5, load15 = os.getloadavg()
        except OSError:
            load1 = load5 = load15 = 0.0
        metrics: Dict[str, Any] = {
            "load": {"1m": round(load1, 2), "5m": round(load5, 2), "15m": round(load15, 2)},
            "rss_mb": round(_rss_mb(), 1),
            "disk": ds,
            "outbox_pending": pending,
            "segments_on_disk": seg_count,
            "segment_bytes": seg_bytes,
            "task_states": state_counts,
            "config_rev": self.cfg.config_rev,
            "agent_version": self.cfg.agent_version,
            "ts": int(time.time()),
        }
        metrics.update(self.extra())
        return metrics

    def stop(self) -> None:
        self.running = False
