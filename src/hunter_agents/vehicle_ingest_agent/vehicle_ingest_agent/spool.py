# -*- coding: utf-8 -*-
"""环形本地缓存与磁盘治理（需求 2 核心；设计 §6.6）。

需求 2 的释放规则在此落地为可组合的策略：

1. **上传成功立即删除**：片段全部对象上传成功后，按 ``upload.keep_raw`` 策略处理：
   - ``delete_on_uploaded``（默认，对应需求 2"上传成功的片段立即删除"）→ 立即删本地；
   - ``after_confirmed`` → 等服务端 ``cmd.cleanup`` 确认再删；
   - ``keep`` → 保留至 TTL GC。
   **protected（关键事件）片段一律不自动删除**（需求 2）。
2. **只保留最近 N 个**：``recording.retention_max_segments`` 为环形窗口软上限；
   上传跟得上时稳态在盘数 ≪ N；离线积压可暂时超 N（不硬删未上传，见 3）。
3. **磁盘水位淘汰**：使用率 ≥ high_watermark → 先删"已上传但仍保留"的对象，
   再删**最旧未上传非保护**片段，直到 ≤ low_watermark；若无可删且仍 ≥ critical →
   上报 ``agent.disk.critical``，宁可让运营介入也**绝不删除未到 uploaded 的保护数据**。
4. **TTL GC**（§6.6）：done 片段中间产物 ``done_ttl_h``、原始片段 ``raw_ttl_d`` 兜底清理。
"""
from __future__ import annotations

import os
import shutil
import time
from datetime import datetime, timezone
from typing import Any, Callable, Dict, List, Optional

from .logging_util import get_logger

logger = get_logger("spool")

Alerter = Callable[[str, str, Dict[str, Any]], None]


class SpoolManager:
    def __init__(self, cfg, store, alert: Optional[Alerter] = None):
        self.cfg = cfg
        self.store = store
        self.alert = alert or (lambda *a, **k: None)
        self.root = cfg.root
        self.high = float(cfg.get("disk.high_watermark"))
        self.low = float(cfg.get("disk.low_watermark"))
        self.critical = float(cfg.get("disk.critical_watermark"))
        self.spool_max = int(cfg.get("disk.spool_max_bytes"))
        self.evict_unuploaded = bool(cfg.get("disk.evict_unuploaded_on_pressure"))
        self.keep_raw = cfg.get("upload.keep_raw", "delete_on_uploaded")
        self.done_ttl_s = float(cfg.get("disk.gc.done_ttl_h", 48)) * 3600
        self.raw_ttl_s = float(cfg.get("disk.gc.raw_ttl_d", 7)) * 86400
        self.retention = int(cfg.get("recording.retention_max_segments"))

    # ── 磁盘水位 ──
    def disk_status(self) -> Dict[str, Any]:
        try:
            total, used, free = shutil.disk_usage(self.root)
            ratio = used / total if total else 0.0
        except OSError as exc:
            logger.warning("磁盘用量读取失败 %s：%s", self.root, exc)
            return {"available": False, "error": str(exc)}
        return {"available": True, "total_bytes": total, "used_bytes": used,
                "free_bytes": free, "ratio": ratio}

    def _now_min(self) -> int:
        return datetime.now(timezone.utc).hour * 60 + datetime.now(timezone.utc).minute

    # ── 上传成功后的释放策略（需求 2：立即删除）──
    def apply_after_upload(self, bag_id: str) -> bool:
        """按 keep_raw 策略决定是否/何时删除本地片段。返回是否已删除。

        protected 片段：不自动删除（保留供检索），仅打标记，交由 GC/人工清理。
        """
        seg = self.store.get_segment(bag_id)
        if not seg:
            return False
        self.store.update_segment(bag_id, uploaded=True)
        if seg.get("protected"):
            logger.info("关键事件片段已上传但按策略保留（protected）bag_id=%s", bag_id,
                        extra={"bag_id": bag_id, "component": "spool"})
            return False
        if self.keep_raw == "delete_on_uploaded":
            return self._delete_local(seg, reason="uploaded")
        # after_confirmed / keep：保留，等确认或 TTL GC（enforce 内处理）
        return False

    # ── 综合治理（周期性调用）──
    def enforce(self) -> Dict[str, Any]:
        """执行环形 + 水位 + TTL 治理。返回处置摘要（供心跳指标）。

        删除顺序严格遵循"仅删可安全删除的对象"（保护未上传数据）。
        """
        summary = {"deleted_uploaded": [], "evicted_unuploaded": [],
                   "ttl_gc": [], "critical": False, "disk_ratio": None}
        # 1) keep_raw=keep/after_confirmed 时，先删已上传的（非保护）以腾空间
        ds = self.disk_status()
        ratio = ds.get("ratio")
        summary["disk_ratio"] = ratio
        keep_mode = self.keep_raw in ("keep", "after_confirmed")

        # 2) 水位压力淘汰（需求 2：磁盘低于阈值时删除最旧未上传片段）
        if ratio is not None and ratio >= self.high:
            summary["evicted_unuploaded"] = self._evict_until(low=self.low,
                                                              include_unuploaded=self.evict_unuploaded)
            ds = self.disk_status()
            ratio = ds.get("ratio")
        elif keep_mode and self.store.count_segments() > self.retention:
            # 无水位压力但 keep 模式积压超环形窗口 → 删已上传非保护（维持稳态）
            summary["deleted_uploaded"] = self._delete_uploaded_beyond_window()

        # 3) TTL GC
        summary["ttl_gc"] = self._ttl_gc()

        # 4) 仍处临界且无可清理 → 告警（绝不删未上传保护数据）
        ds = self.disk_status()
        ratio = ds.get("ratio")
        summary["disk_ratio"] = ratio
        if ratio is not None and ratio >= self.critical and not self._has_deletable():
            summary["critical"] = True
            self.alert("E_DISK_CRITICAL", "critical",
                       {"disk_ratio": round(ratio, 3), "spool_max_bytes": self.spool_max})
            logger.error("磁盘临界且无可清理对象，保护未上传数据（需运营介入）ratio=%.3f",
                         ratio, extra={"component": "spool"})
        return summary

    def _delete_uploaded_beyond_window(self) -> List[str]:
        deleted = []
        segs = [s for s in self.store.list_segments()
                if s.get("uploaded") and not s.get("protected")]
        # 保留最近 retention 个（按 created_at 升序，删最旧）
        over = max(0, self.store.count_segments() - self.retention)
        for seg in segs[:over]:
            if self._delete_local(seg, reason="ring-window"):
                deleted.append(seg["bag_id"])
        return deleted

    def _evict_until(self, low: float, include_unuploaded: bool) -> List[str]:
        """水位淘汰：先已上传，再最旧未上传（非保护），直到 ≤ low 或无可删。"""
        evicted: List[str] = []
        # a) 已上传非保护（keep 模式残留）
        for seg in self.store.list_segments():
            if self._ratio() <= low:
                return evicted
            if seg.get("uploaded") and not seg.get("protected"):
                if self._delete_local(seg, reason="high-watermark(uploaded)"):
                    evicted.append(seg["bag_id"])
        # b) 最旧未上传非保护（需求 2 明确允许）
        if include_unuploaded:
            for seg in self.store.list_segments():     # 已按 created_at 升序
                if self._ratio() <= low:
                    return evicted
                if not seg.get("uploaded") and not seg.get("protected"):
                    self.alert("E_DISK_CRITICAL", "warning",
                               {"evicting_unuploaded": seg["bag_id"]})
                    if self._delete_local(seg, reason="high-watermark(unuploaded)"):
                        evicted.append(seg["bag_id"])
        return evicted

    def _ttl_gc(self) -> List[str]:
        """原始片段 TTL 兜底（design §6.6：raw_ttl_d 天后本地 GC，且触发事件）。"""
        removed: List[str] = []
        now = time.time()
        for seg in self.store.list_segments():
            if seg.get("protected"):
                continue
            age = now - seg["created_at"]
            if seg.get("uploaded") and self.keep_raw == "keep" and age > self.done_ttl_s:
                if self._delete_local(seg, reason="ttl-done"):
                    removed.append(seg["bag_id"])
            elif not seg.get("uploaded") and age > self.raw_ttl_s:
                # 未上传但超 raw_ttl：保守——不删未上传（保护数据），仅告警
                self.alert("E_DISK_CRITICAL", "warning",
                           {"unuploaded_aged": seg["bag_id"], "age_days": round(age / 86400, 1)})
        return removed

    def _has_deletable(self) -> bool:
        return any((not s.get("protected")) for s in self.store.list_segments())

    def _ratio(self) -> float:
        ds = self.disk_status()
        return ds.get("ratio", 0.0) if ds.get("available") else 0.0

    # ── 底层删除 ──
    def _delete_local(self, seg: Dict[str, Any], reason: str) -> bool:
        bag_id = seg["bag_id"]
        path = seg["path"]
        ok = True
        # 片段目录（含子目录）删除；仅删除 bags 根下的片段，防误删
        if _is_under(path, self.cfg.bags_dir):
            try:
                shutil.rmtree(path, ignore_errors=False)
            except OSError as exc:
                logger.error("删除片段目录失败 %s：%s", path, exc,
                             extra={"bag_id": bag_id, "component": "spool"})
                ok = False
        else:
            logger.error("拒绝删除 bags 根之外的路径（安全护栏）：%s", path,
                         extra={"bag_id": bag_id, "component": "spool"})
            ok = False
        if ok:
            self.store.delete_segment(bag_id)
            logger.info("本地片段已释放（%s）bag_id=%s", reason, bag_id,
                        extra={"bag_id": bag_id, "component": "spool"})
        return ok

    # ── 服务端确认清理（after_confirmed 策略，§6.8.5 cmd.cleanup）──
    def cleanup_confirmed(self, bag_ids: List[str]) -> List[str]:
        cleaned = []
        for bag_id in bag_ids:
            seg = self.store.get_segment(bag_id)
            if seg and seg.get("uploaded"):
                if self._delete_local(seg, reason="server-confirmed"):
                    cleaned.append(bag_id)
        return cleaned


def _is_under(path: str, root: str) -> bool:
    try:
        return os.path.commonpath([os.path.abspath(path), os.path.abspath(root)]) == \
            os.path.abspath(root)
    except ValueError:
        return False
