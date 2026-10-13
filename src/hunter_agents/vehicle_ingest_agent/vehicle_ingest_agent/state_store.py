# -*- coding: utf-8 -*-
"""SQLite 持久层（设计 §6.8.1 outbox、§6.9 状态机、§6.6 环形片段治理）。

单文件 ``agent.db``，WAL 模式，事务安全，进程重启后非终态任务可恢复。
所有写走同一 ``threading.Lock``（asyncio 事件循环 + librdkafka 回调线程可能并发），
避免并发写事务互踩。

表：
  tasks(bag_id PK, state, seq, reason_code, attempts, ...)   —— 任务状态机（§6.9）
  segments(id PK, bag_id UNIQUE, path, size, sha256, uploaded, protected, ...)
        —— 环形本地缓存片段登记（需求 2 治理依据）
  outbox(id PK, topic, pkey, payload, status, ...)            —— at-least-once 起点（§6.8.1）
  cmd_dedup(cmd_id PK, ...)                                   —— 指令幂等（§6.8.5）
  meta(key PK, value)                                         —— config_rev / 状态游标

关键不变量：**状态转移与对应 outbox 事件同事务提交**（:meth:`transition`），
从而"任务状态变了，事件必然最终送达"。
"""
from __future__ import annotations

import json
import os
import sqlite3
import threading
import time
from typing import Any, Dict, List, Optional

from .logging_util import get_logger
from . import events

logger = get_logger("state")

_SCHEMA = """
CREATE TABLE IF NOT EXISTS tasks(
    bag_id      TEXT PRIMARY KEY,
    state       TEXT NOT NULL,
    seq         INTEGER NOT NULL DEFAULT 0,
    reason_code TEXT,
    attempts    INTEGER NOT NULL DEFAULT 0,
    created_at  REAL NOT NULL,
    updated_at  REAL NOT NULL
);
CREATE TABLE IF NOT EXISTS segments(
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    bag_id      TEXT NOT NULL UNIQUE,
    path        TEXT NOT NULL,
    size_bytes  INTEGER NOT NULL DEFAULT 0,
    sha256      TEXT,
    start_ns    INTEGER NOT NULL DEFAULT 0,
    end_ns      INTEGER NOT NULL DEFAULT 0,
    duration_s  REAL NOT NULL DEFAULT 0,
    state       TEXT NOT NULL,
    uploaded    INTEGER NOT NULL DEFAULT 0,
    protected   INTEGER NOT NULL DEFAULT 0,
    priority    INTEGER NOT NULL DEFAULT 4,
    created_at  REAL NOT NULL,
    updated_at  REAL NOT NULL
);
CREATE INDEX IF NOT EXISTS idx_seg_created ON segments(created_at);
CREATE INDEX IF NOT EXISTS idx_seg_uploaded ON segments(uploaded, protected);
CREATE TABLE IF NOT EXISTS outbox(
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    topic       TEXT NOT NULL,
    pkey        TEXT NOT NULL,
    payload     BLOB NOT NULL,
    status      TEXT NOT NULL DEFAULT 'pending',   -- pending | done
    created_at  REAL NOT NULL,
    sent_at     REAL,
    attempts    INTEGER NOT NULL DEFAULT 0
);
CREATE INDEX IF NOT EXISTS idx_outbox_status ON outbox(status, id);
CREATE TABLE IF NOT EXISTS cmd_dedup(
    cmd_id      TEXT PRIMARY KEY,
    received_at REAL NOT NULL,
    result      TEXT
);
CREATE TABLE IF NOT EXISTS meta(
    key   TEXT PRIMARY KEY,
    value TEXT
);
"""

TERMINAL_STATES = ("done", "quarantined")


class StateStore:
    def __init__(self, db_path: str):
        self.db_path = db_path
        os.makedirs(os.path.dirname(db_path), exist_ok=True)
        self._lock = threading.Lock()
        self._conn = sqlite3.connect(db_path, check_same_thread=False, timeout=30.0)
        self._conn.row_factory = sqlite3.Row
        with self._conn:
            self._conn.execute("PRAGMA journal_mode=WAL;")
            self._conn.execute("PRAGMA synchronous=NORMAL;")
            self._conn.executescript(_SCHEMA)

    def close(self) -> None:
        with self._lock:
            try:
                self._conn.commit()
            finally:
                self._conn.close()

    # ── meta ──
    def meta_get(self, key: str, default: Optional[str] = None) -> Optional[str]:
        with self._lock:
            row = self._conn.execute("SELECT value FROM meta WHERE key=?", (key,)).fetchone()
            return row["value"] if row else default

    def meta_set(self, key: str, value: str) -> None:
        with self._lock, self._conn:
            self._conn.execute(
                "INSERT INTO meta(key,value) VALUES(?,?) "
                "ON CONFLICT(key) DO UPDATE SET value=excluded.value", (key, value))

    # ── tasks（状态机）──
    def create_task(self, bag_id: str, state: str = "discovered") -> Dict[str, Any]:
        now = time.time()
        with self._lock, self._conn:
            cur = self._conn.execute(
                "INSERT OR IGNORE INTO tasks(bag_id,state,seq,created_at,updated_at)"
                " VALUES(?,?,0,?,?)", (bag_id, state, now, now))
            if cur.rowcount == 0:
                logger.debug("task 已存在，忽略重复创建", extra={"bag_id": bag_id})
        return self.get_task(bag_id)  # type: ignore[return-value]

    def get_task(self, bag_id: str) -> Optional[Dict[str, Any]]:
        with self._lock:
            row = self._conn.execute("SELECT * FROM tasks WHERE bag_id=?", (bag_id,)).fetchone()
            return dict(row) if row else None

    def task_exists(self, bag_id: str) -> bool:
        with self._lock:
            return self._conn.execute(
                "SELECT 1 FROM tasks WHERE bag_id=?", (bag_id,)).fetchone() is not None

    def list_nonterminal_tasks(self) -> List[Dict[str, Any]]:
        """重启恢复：所有未到终态的任务（§6.9）。"""
        with self._lock:
            q = "SELECT * FROM tasks WHERE state NOT IN (%s)" % \
                ",".join("?" * len(TERMINAL_STATES))
            rows = self._conn.execute(q, TERMINAL_STATES).fetchall()
            return [dict(r) for r in rows]

    # ── outbox ──
    def enqueue_outbox(self, topic: str, pkey: str, payload: Dict[str, Any]) -> int:
        """单条入队（不与状态绑定）；返回 outbox id（envelope seq 由此派生）。"""
        return self._enqueue_in(None, topic, pkey, payload)

    def _enqueue_in(self, conn: Optional[sqlite3.Connection], topic: str, pkey: str,
                    payload: Dict[str, Any]) -> int:
        blob = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        own = conn is None
        con = conn or self._conn
        if own:
            with self._lock, con:
                cur = con.execute(
                    "INSERT INTO outbox(topic,pkey,payload,status,created_at)"
                    " VALUES(?,?,?,'pending',?)", (topic, pkey, blob, time.time()))
                return int(cur.lastrowid)
        cur = con.execute(
            "INSERT INTO outbox(topic,pkey,payload,status,created_at)"
            " VALUES(?,?,?,'pending',?)", (topic, pkey, blob, time.time()))
        return int(cur.lastrowid)

    def fetch_pending(self, limit: int) -> List[Dict[str, Any]]:
        """BEGIN IMMEDIATE 取出待发送（§6.8.1：取行即占用，避免并发重复投递）。"""
        with self._lock:
            with self._conn:
                self._conn.execute("BEGIN IMMEDIATE")
                rows = self._conn.execute(
                    "SELECT id,topic,pkey,payload,attempts FROM outbox "
                    "WHERE status='pending' ORDER BY id LIMIT ?", (limit,)).fetchall()
                return [dict(r) for r in rows]

    def mark_done(self, outbox_id: int) -> None:
        with self._lock, self._conn:
            self._conn.execute(
                "UPDATE outbox SET status='done', sent_at=? WHERE id=?",
                (time.time(), outbox_id))

    def bump_attempts(self, outbox_id: int) -> None:
        with self._lock, self._conn:
            self._conn.execute(
                "UPDATE outbox SET attempts=attempts+1 WHERE id=?", (outbox_id,))

    def count_pending(self) -> int:
        with self._lock:
            return int(self._conn.execute(
                "SELECT COUNT(*) FROM outbox WHERE status='pending'").fetchone()[0])

    def purge_done_before(self, ts: float) -> int:
        with self._lock, self._conn:
            cur = self._conn.execute(
                "DELETE FROM outbox WHERE status='done' AND sent_at < ?", (ts,))
            return cur.rowcount

    # ── 状态转移 + 事件同事务（§6.9 关键不变量）──
    def transition(self, bag_id: str, to_state: str, *, event_type: str,
                   topic: str, vehicle_id: str, agent_version: str,
                   payload: Optional[Dict[str, Any]] = None,
                   reason_code: Optional[str] = None,
                   attempt: int = 0) -> Dict[str, Any]:
        """原子地：更新 tasks.state、seq+1、写 outbox(task.status) envelope。

        task.status 事件在此以完整 envelope 落库（含服务端去重用的 event_id 与本 bag
        单调 seq），从而"状态变了 → 事件必然最终送达"（§6.8.1、§6.8.4）。
        返回 {seq, outbox_id, from_state, to_state}。
        """
        now = time.time()
        with self._lock, self._conn:
            self._conn.execute("BEGIN IMMEDIATE")
            row = self._conn.execute(
                "SELECT state, seq FROM tasks WHERE bag_id=?", (bag_id,)).fetchone()
            if row is None:
                raise KeyError(f"transition: 未知 bag_id={bag_id}")
            from_state = row["state"]
            seq = int(row["seq"]) + 1
            self._conn.execute(
                "UPDATE tasks SET state=?, seq=?, reason_code=?, attempts=?, updated_at=?"
                " WHERE bag_id=?", (to_state, seq, reason_code, attempt, now, bag_id))
            status_payload = dict(payload or {})
            status_payload.setdefault("from", from_state)
            status_payload.setdefault("to", to_state)
            status_payload.setdefault("reason_code", reason_code)
            status_payload.setdefault("attempt", attempt)
            envelope = events.make_envelope(
                event_type, vehicle_id, agent_version, status_payload,
                bag_id=bag_id, seq=seq)
            outbox_id = self._enqueue_in(self._conn, topic, bag_id, envelope)
            self._conn.execute("COMMIT")
            logger.info("状态转移 %s → %s", from_state, to_state,
                        extra={"bag_id": bag_id, "task_state": to_state,
                               "reason_code": reason_code, "seq": seq})
            return {"seq": seq, "outbox_id": outbox_id,
                    "from_state": from_state, "to_state": to_state}

    def set_seq_only(self, bag_id: str, to_state: str, reason_code: Optional[str] = None,
                     attempt: int = 0) -> int:
        """不写 outbox，仅更新状态（用于纯恢复/去重路径）。返回新 seq。"""
        now = time.time()
        with self._lock, self._conn:
            row = self._conn.execute("SELECT seq FROM tasks WHERE bag_id=?",
                                     (bag_id,)).fetchone()
            seq = int(row["seq"]) + 1 if row else 1
            self._conn.execute(
                "UPDATE tasks SET state=?, seq=?, reason_code=?, attempts=?, updated_at=?"
                " WHERE bag_id=?", (to_state, seq, reason_code, attempt, now, bag_id))
            return seq

    # ── segments（环形缓存治理核心，需求 2）──
    def register_segment(self, bag_id: str, path: str, size_bytes: int,
                         start_ns: int, end_ns: int, duration_s: float,
                         state: str = "discovered", priority: int = 4,
                         protected: bool = False) -> Dict[str, Any]:
        now = time.time()
        with self._lock, self._conn:
            self._conn.execute(
                "INSERT OR IGNORE INTO segments(bag_id,path,size_bytes,start_ns,end_ns,"
                "duration_s,state,protected,priority,created_at,updated_at)"
                " VALUES(?,?,?,?,?,?,?,?,?,?,?)",
                (bag_id, path, size_bytes, start_ns, end_ns, duration_s, state,
                 1 if protected else 0, priority, now, now))
        return self.get_segment(bag_id)  # type: ignore[return-value]

    def get_segment(self, bag_id: str) -> Optional[Dict[str, Any]]:
        with self._lock:
            row = self._conn.execute("SELECT * FROM segments WHERE bag_id=?",
                                     (bag_id,)).fetchone()
            return dict(row) if row else None

    def update_segment(self, bag_id: str, **fields: Any) -> None:
        if not fields:
            return
        if "protected" in fields:
            fields["protected"] = 1 if fields["protected"] else 0
        if "uploaded" in fields:
            fields["uploaded"] = 1 if fields["uploaded"] else 0
        cols = ",".join(f"{k}=?" for k in fields)
        vals = list(fields.values()) + [time.time(), bag_id]
        with self._lock, self._conn:
            self._conn.execute(
                f"UPDATE segments SET {cols}, updated_at=? WHERE bag_id=?", vals)

    def list_segments(self) -> List[Dict[str, Any]]:
        """按创建时间升序（最旧在前）——环形淘汰/保留判定依据。"""
        with self._lock:
            rows = self._conn.execute(
                "SELECT * FROM segments ORDER BY created_at ASC, id ASC").fetchall()
            return [dict(r) for r in rows]

    def count_segments(self) -> int:
        with self._lock:
            return int(self._conn.execute("SELECT COUNT(*) FROM segments").fetchone()[0])

    def delete_segment(self, bag_id: str) -> None:
        with self._lock, self._conn:
            self._conn.execute("DELETE FROM segments WHERE bag_id=?", (bag_id,))

    def total_segment_bytes(self) -> int:
        with self._lock:
            row = self._conn.execute(
                "SELECT COALESCE(SUM(size_bytes),0) AS s FROM segments").fetchone()
            return int(row["s"])

    # ── cmd dedup（§6.8.5）──
    def cmd_seen(self, cmd_id: str) -> Optional[Dict[str, Any]]:
        with self._lock:
            row = self._conn.execute("SELECT * FROM cmd_dedup WHERE cmd_id=?",
                                     (cmd_id,)).fetchone()
            return dict(row) if row else None

    def record_cmd(self, cmd_id: str, result: str) -> None:
        with self._lock, self._conn:
            self._conn.execute(
                "INSERT OR IGNORE INTO cmd_dedup(cmd_id,received_at,result) VALUES(?,?,?)",
                (cmd_id, time.time(), result))
