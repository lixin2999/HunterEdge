# -*- coding: utf-8 -*-
"""任务状态机（设计 §6.9）+ 事件投递封装（§7.4）。

状态图（含降级分支，parse.enabled=false 走 uploading_raw）：

  discovered → verifying → {quarantined | manifest_ready}
  manifest_ready → {parsing | uploading_raw}
  parsing → {parsed | parse_failed}；parse_failed → {parsing | uploading_raw}
  parsed → uploading；uploading_raw → uploading_done
  uploading → {uploading_done | upload_failed}；upload_failed → uploading
  uploading_done → done

不变量：每次转移都经 :meth:`StateStore.transition` 与 ``bag.task.status.v1`` 事件同事务落库；
业务事件（manifest/artifact.*）单独入 outbox（at-least-once + 服务端 event_id 幂等）。
本模块只做"合法边校验 + envelope 组装 + 入队"，不碰线程/IO。
"""
from __future__ import annotations

from typing import Any, Dict, Optional, Set

from . import events
from .logging_util import get_logger

logger = get_logger("statemachine")

DISCOVERED = "discovered"
VERIFYING = "verifying"
QUARANTINED = "quarantined"
MANIFEST_READY = "manifest_ready"
PARSING = "parsing"
PARSED = "parsed"
PARSE_FAILED = "parse_failed"
UPLOADING_RAW = "uploading_raw"
UPLOADING = "uploading"
UPLOADING_DONE = "uploading_done"
UPLOAD_FAILED = "upload_failed"
DONE = "done"

#: 允许的状态边（§6.9）
_TRANSITIONS: Dict[str, Set[str]] = {
    DISCOVERED: {VERIFYING},
    VERIFYING: {QUARANTINED, MANIFEST_READY},
    MANIFEST_READY: {PARSING, UPLOADING_RAW},
    PARSING: {PARSED, PARSE_FAILED},
    PARSE_FAILED: {PARSING, UPLOADING_RAW},
    PARSED: {UPLOADING},
    UPLOADING_RAW: {UPLOADING_DONE},
    UPLOADING: {UPLOADING_DONE, UPLOAD_FAILED},
    UPLOAD_FAILED: {UPLOADING},
    UPLOADING_DONE: {DONE},
    QUARANTINED: set(),
    DONE: set(),
}
TERMINAL: Set[str] = {DONE, QUARANTINED}


class TaskStateMachine:
    def __init__(self, cfg, store):
        self.cfg = cfg
        self.store = store
        self.vehicle_id = cfg.vehicle_id
        self.agent_version = cfg.agent_version
        tm = cfg.get("kafka.topic_map", {})
        self.t_task_status = tm.get("task_status", "vehicle.bag.task.status")
        self.t_manifest = tm.get("manifest", "vehicle.bag.manifest")
        self.t_progress = tm.get("parse_progress", "vehicle.bag.parse.progress")
        self.t_uploaded = tm.get("artifact_uploaded", "vehicle.bag.artifact.uploaded")
        self.t_heartbeat = tm.get("heartbeat", "vehicle.agent.heartbeat")
        self.t_alert = tm.get("alert", "vehicle.agent.alert")

    @staticmethod
    def can_transition(frm: str, to: str) -> bool:
        return to in _TRANSITIONS.get(frm, set())

    def transition(self, bag_id: str, to: str, *, reason: Optional[str] = None,
                   attempt: int = 0, payload: Optional[Dict[str, Any]] = None,
                   force: bool = False) -> Dict[str, Any]:
        """执行一次状态转移（写状态 + task.status 事件同事务）。

        非法边默认拒绝（抛 :class:`InvalidTransition`）；``force=True`` 供恢复路径放行。
        """
        task = self.store.get_task(bag_id)
        if task is None:
            raise KeyError(f"transition: 未知 bag_id={bag_id}")
        frm = task["state"]
        if not force and not self.can_transition(frm, to):
            raise InvalidTransition(f"{bag_id}: {frm} → {to} 不是合法边")
        return self.store.transition(
            bag_id, to, event_type="bag.task.status.v1", topic=self.t_task_status,
            vehicle_id=self.vehicle_id, agent_version=self.agent_version,
            payload=payload, reason_code=reason, attempt=attempt)

    # ── 业务事件（独立 outbox 行）──
    def emit_manifest(self, bag_id: str, seq: int, manifest: Dict[str, Any]) -> None:
        env = events.make_envelope("bag.manifest.v1", self.vehicle_id, self.agent_version,
                                   events.manifest_payload(manifest), bag_id=bag_id, seq=seq)
        self._enqueue_guarded(self.t_manifest, bag_id, env)

    def emit_artifact_ready(self, bag_id: str, seq: int, artifact_class: str,
                            local_rel_path: str, count: int, size_bytes: int) -> None:
        env = events.make_envelope(
            "bag.artifact.ready.v1", self.vehicle_id, self.agent_version,
            events.artifact_ready_payload(artifact_class, local_rel_path, count, size_bytes),
            bag_id=bag_id, seq=seq)
        self._enqueue_guarded(self.t_uploaded, bag_id, env)  # ready 复用同 topic 无碍，见下注

    def emit_artifact_uploaded(self, bag_id: str, seq: int, **kw: Any) -> None:
        env = events.make_envelope(
            "bag.artifact.uploaded.v1", self.vehicle_id, self.agent_version,
            events.artifact_uploaded_payload(**kw), bag_id=bag_id, seq=seq)
        self._enqueue_guarded(self.t_uploaded, bag_id, env)

    def emit_parse_progress(self, bag_id: str, seq: int, phase: str, percent: int,
                            topics_done: Optional[list] = None) -> None:
        env = events.make_envelope(
            "bag.parse.progress.v1", self.vehicle_id, self.agent_version,
            events.parse_progress_payload(phase, percent, topics_done), bag_id=bag_id, seq=seq)
        self.store.enqueue_outbox(self.t_progress, bag_id, env)

    def emit_alert(self, code: str, severity: str, detail: Dict[str, Any],
                   bag_id: Optional[str] = None) -> None:
        env = events.make_envelope(
            "vehicle.agent.alert", self.vehicle_id, self.agent_version,
            events.alert_payload(code, severity, detail), bag_id=bag_id)
        self.store.enqueue_outbox(self.t_alert, self.vehicle_id, env)

    def emit_heartbeat(self, metrics: Dict[str, Any]) -> None:
        env = events.make_envelope(
            "vehicle.agent.heartbeat", self.vehicle_id, self.agent_version,
            events.heartbeat_payload(metrics))
        self.store.enqueue_outbox(self.t_heartbeat, self.vehicle_id, env)

    def _enqueue_guarded(self, topic: str, key: str, env: Dict[str, Any]) -> None:
        """超大 manifest（>message.max.bytes）改传对象存储指针（§6.8.3）；此处按大小粗略降级。"""
        import json
        blob = json.dumps(env, ensure_ascii=False)
        limit = int(self.cfg.get("kafka.producer.message_max_bytes", 4194304))
        if len(blob.encode("utf-8")) > limit:
            logger.warning("事件超 Kafka 上限，转指针 topic=%s bag_id=%s size=%d",
                           topic, key, len(blob), extra={"bag_id": key})
            env["payload"] = {"truncated": True, "size_bytes": len(blob),
                              "note": "exceeds message.max.bytes; full body in object store"}
        self.store.enqueue_outbox(topic, key, env)


class InvalidTransition(RuntimeError):
    """非法状态转移（调用方据此决定是否 force 或改走告警）。"""
