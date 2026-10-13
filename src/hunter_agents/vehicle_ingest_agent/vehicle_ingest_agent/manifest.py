# -*- coding: utf-8 -*-
"""manifest.json 生成（设计 §6.2、§7.1 schema v1）。

Verifier 通过后，据校验事实（topics/message_count/duration/hash）组装 manifest，
写入 ``spool/{bag_id}/manifest.json`` 并落盘对象存储路径（§7.2）。

降级模式（parse.enabled=false，本轮默认）：``parse_status="skipped"``，
不产图像/点云/表格，仅上传原始片段 + manifest，服务端集中解析。
"""
from __future__ import annotations

import json
import os
from typing import Any, Dict

from .logging_util import get_logger

logger = get_logger("manifest")

PARSE_VERSION = "vha-parser-v1.0"


def build_manifest(bag_id: str, cfg, facts: Dict[str, Any], *, protected: bool) -> Dict[str, Any]:
    """组装 manifest（§7.1）。``facts`` 来自 verifier.verify_segment。"""
    parse_enabled = cfg.parse_enabled
    topics = []
    for t in facts.get("topics", []):
        entry = {
            "topic": t["topic"], "type": t.get("type"),
            "message_count": int(t.get("message_count", 0)),
            "start_time": t.get("start_time", 0), "end_time": t.get("end_time", 0),
        }
        if not parse_enabled:
            entry["parse"] = {"parser": "RawKeep", "status": "skipped_server_parse"}
        topics.append(entry)

    warnings = facts.get("warnings", [])
    clock_status = "ok"
    max_skew = 0.0
    if "W_CLOCK_SKEW" in warnings:
        clock_status = "skewed"

    manifest: Dict[str, Any] = {
        "schema_version": 1,
        "bag_id": bag_id,
        "vehicle_id": cfg.vehicle_id,
        "ros_version": "ROS2",
        "source_path": facts.get("path", ""),
        "start_time_ns": int(facts.get("start_ns", 0)),
        "end_time_ns": int(facts.get("end_ns", 0)),
        "duration_s": float(facts.get("duration_s", 0.0)),
        "file_size_bytes": int(facts.get("size_bytes", 0)),
        "file_sha256": facts.get("sha256", ""),
        "message_count": int(facts.get("message_count", 0)),
        "coordinate_frames": [],                     # 车端降级不解析 TF，由服务端补
        "clock_health": {"max_skew_ms": max_skew, "status": clock_status},
        "topics": topics,
        "parse_status": "completed" if parse_enabled else "skipped",
        "parse_stats": {} if parse_enabled else {"mode": "degraded_raw_upload"},
        "protected": bool(protected),
        "parse_version": PARSE_VERSION,
        "config_rev": cfg.config_rev,
        "agent_version": cfg.agent_version,
    }
    return manifest


def write_manifest(spool_bag_dir: str, manifest: Dict[str, Any]) -> str:
    """写 ``spool/{bag_id}/manifest.json``，返回其绝对路径。"""
    os.makedirs(spool_bag_dir, exist_ok=True)
    path = os.path.join(spool_bag_dir, "manifest.json")
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(manifest, fh, ensure_ascii=False, indent=2)
    logger.info("manifest 已生成 %s (topics=%d sha256=%s)",
                path, len(manifest.get("topics", [])),
                (manifest.get("file_sha256") or "")[:12],
                extra={"bag_id": manifest.get("bag_id"), "component": "manifest"})
    return path
