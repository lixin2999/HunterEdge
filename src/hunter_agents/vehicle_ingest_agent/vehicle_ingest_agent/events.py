# -*- coding: utf-8 -*-
"""Kafka 上行事件封装（设计 §6.8.3 envelope、§7.4 payload 契约）。

所有上行 topic 统一 envelope：
  {schema_version, event_id, event_type, occurred_at, vehicle_id, agent_version,
   bag_id, seq, payload[, trace_id]}
- ``event_id``：服务端唯一约束去重键（uuid，时间有序前缀便于排障）。
- ``seq``：本 bag 内单调递增，由 tasks.seq 派生；task.status 投影仅接受更大 seq（§6.8.4）。
"""
from __future__ import annotations

import time
import uuid
from datetime import datetime, timezone
from typing import Any, Dict, Optional

SCHEMA_VERSION = 1


def _event_id() -> str:
    """时间有序 id：<unix_ms_hex>-<uuid4>，服务端按 event_id 唯一去重。"""
    return "%x-%s" % (int(time.time() * 1000), uuid.uuid4().hex)


def _iso_utc(ts: Optional[float] = None) -> str:
    dt = datetime.fromtimestamp(ts if ts is not None else time.time(), tz=timezone.utc)
    return dt.strftime("%Y-%m-%dT%H:%M:%S.") + "%03dZ" % (dt.microsecond // 1000)


def make_envelope(event_type: str, vehicle_id: str, agent_version: str,
                  payload: Dict[str, Any], bag_id: Optional[str] = None,
                  seq: int = 0, trace_id: Optional[str] = None) -> Dict[str, Any]:
    env: Dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "event_id": _event_id(),
        "event_type": event_type,
        "occurred_at": _iso_utc(),
        "vehicle_id": vehicle_id,
        "agent_version": agent_version,
        "bag_id": bag_id,
        "seq": seq,
        "payload": payload,
    }
    if trace_id:
        env["trace_id"] = trace_id
    return env


# ── 事件构造器（对照 §7.4 契约要点）────────────────────────────────
def task_status_payload(from_state: str, to_state: str,
                        reason_code: Optional[str] = None, attempt: int = 0) -> Dict[str, Any]:
    return {"from": from_state, "to": to_state,
            "reason_code": reason_code, "attempt": attempt}


def manifest_payload(manifest: Dict[str, Any]) -> Dict[str, Any]:
    """manifest 全文（<1MB，超限由调用方改传对象存储指针，§6.8.3）。"""
    return {"manifest": manifest}


def parse_progress_payload(phase: str, percent: int,
                           topics_done: Optional[list] = None) -> Dict[str, Any]:
    return {"phase": phase, "percent": percent, "topics_done": topics_done or []}


def artifact_ready_payload(artifact_class: str, local_rel_path: str,
                           count: int, size_bytes: int) -> Dict[str, Any]:
    return {"class": artifact_class, "local_rel_path": local_rel_path,
            "count": count, "bytes": size_bytes}


def artifact_uploaded_payload(artifact_class: str, object_key: str, size_bytes: int,
                              sha256: str, etag: str, upload_duration_ms: int,
                              endpoint: str, rows: Optional[int] = None) -> Dict[str, Any]:
    p: Dict[str, Any] = {
        "artifact_class": artifact_class, "object_key": object_key,
        "size_bytes": size_bytes, "sha256": sha256, "etag": etag,
        "upload_duration_ms": upload_duration_ms, "endpoint": endpoint,
    }
    if rows is not None:
        p["rows"] = rows
    return p


def alert_payload(code: str, severity: str, detail: Dict[str, Any]) -> Dict[str, Any]:
    return {"code": code, "severity": severity, "detail": detail}


def heartbeat_payload(metrics: Dict[str, Any]) -> Dict[str, Any]:
    return metrics
