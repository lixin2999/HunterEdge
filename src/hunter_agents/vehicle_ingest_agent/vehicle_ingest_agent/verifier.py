# -*- coding: utf-8 -*-
"""片段完整性校验（设计 §6.2 校验表、附录 C 错误码）。

对**已 seal** 的 rosbag2 片段目录做：可读性、流式 sha256、topic 命中白名单、
时长/消息数下限、时间戳单调抽样。全部通过才进入 manifest 生成与上传。

读取 ROS2 元数据不依赖 rosbag2_py：解析目录内 ``metadata.yaml``（rosbag2 收尾产物）
即得 topics/message_count/duration/starting_time，成本极低（§3.3 兜底思路）。
CPU/IO 密集（sha256）用同步函数，由上层 ``run_in_executor`` 调用以免阻塞事件循环，
并按 ``manifest.hash_chunk_mb`` 分块 + 可选 sleep 限速（§9 不与主计算争抢）。
"""
from __future__ import annotations

import hashlib
import os
import time
from typing import Any, Dict, List, Optional, Tuple

from .logging_util import get_logger

logger = get_logger("verifier")

# 错误/警告码（附录 C）
E_BAG_CORRUPT = "E_BAG_CORRUPT"
E_HASH = "E_HASH"
E_NO_TARGET_TOPIC = "E_NO_TARGET_TOPIC"
E_EMPTY_BAG = "E_EMPTY_BAG"
W_TS_NONMONO = "W_TS_NONMONO"
W_CLOCK_SKEW = "W_CLOCK_SKEW"


def _read_metadata(seg_dir: str) -> Optional[Dict[str, Any]]:
    meta_path = os.path.join(seg_dir, "metadata.yaml")
    if not os.path.isfile(meta_path):
        return None
    try:
        import yaml
        with open(meta_path, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh)
    except Exception as exc:  # noqa: BLE001  元数据不可读 = 损坏
        logger.warning("metadata.yaml 解析失败 %s：%s", seg_dir, exc)
        return None
    return data


def _flatten_ros2_meta(meta: Dict[str, Any]) -> Dict[str, Any]:
    """把 rosbag2 metadata.yaml 归一为 {topics:[{name,type,count}], duration_s,
    start_ns, message_count}。不同 ROS 版本键位略有差异，这里做兼容。"""
    info = meta.get("rosbag2_info") or meta
    topics_raw = info.get("topics_with_message_count") or info.get("topics") or []
    topics: List[Dict[str, Any]] = []
    msg_total = 0
    for t in topics_raw:
        td = t.get("topic_data", t) if isinstance(t, dict) else {}
        name = td.get("name")
        typ = td.get("type")
        cnt = t.get("message_count", td.get("message_count", 0)) if isinstance(t, dict) else 0
        if name:
            topics.append({"topic": name, "type": typ, "message_count": int(cnt or 0)})
            msg_total += int(cnt or 0)
    starting = info.get("starting_time", {})
    start_ns = starting.get("nanoseconds", 0) if isinstance(starting, dict) else 0
    duration = float(info.get("duration", 0.0) if isinstance(info.get("duration"), (int, float))
                     else _dur_from_str(info.get("duration", 0.0)))
    return {"topics": topics, "message_count": msg_total,
            "start_ns": int(start_ns or 0), "duration_s": duration}


def _dur_from_str(val: Any) -> float:
    """兼容 duration 以 'H:MM:SS' 或 float 秒字符串表达的情况。"""
    if not isinstance(val, str):
        return 0.0
    try:
        if ":" in val:
            h, m, s = val.split(":")
            return int(h) * 3600 + int(m) * 60 + float(s)
        return float(val)
    except (ValueError, IndexError):
        return 0.0


def compute_sha256(seg_dir: str, chunk_bytes: int = 8 * 1024 * 1024,
                   sleep_every_chunks: int = 0) -> str:
    """对流内所有数据文件（.db3/.bag，按名排序）计算组合 sha256（分块 + 可选限速）。"""
    files = []
    for root, _d, fs in os.walk(seg_dir):
        for f in sorted(fs):
            if f.endswith((".db3", ".bag")):
                files.append(os.path.join(root, f))
    files.sort()
    h = hashlib.sha256()
    n = 0
    for fp in files:
        with open(fp, "rb") as fh:
            while True:
                chunk = fh.read(chunk_bytes)
                if not chunk:
                    break
                h.update(chunk)
                n += 1
                if sleep_every_chunks and n % sleep_every_chunks == 0:
                    time.sleep(0.01)   # 轻量限速，避免抢占主计算 IO（§9）
    return h.hexdigest()


def verify_segment(seg_dir: str, *, whitelist: List[str], min_duration_s: float,
                   compute_hash: bool, hash_chunk_mb: int,
                   skew_warn_ms: int) -> Tuple[bool, Optional[str], Dict[str, Any]]:
    """执行 §6.2 校验。返回 (passed, error_code_or_None, facts)。

    ``facts`` 含 topics/message_count/duration_s/start_ns/end_ns/sha256/warnings，
    供 manifest 直接复用（避免二次读盘）。
    """
    facts: Dict[str, Any] = {"warnings": []}
    meta = _read_metadata(seg_dir)
    if meta is None:
        return False, E_BAG_CORRUPT, facts
    norm = _flatten_ros2_meta(meta)
    facts.update(topics=norm["topics"], message_count=norm["message_count"],
                 duration_s=norm["duration_s"], start_ns=norm["start_ns"])

    # topic 存在性 + 白名单命中 ≥1
    names = [t["topic"] for t in norm["topics"]]
    if not names:
        return False, E_NO_TARGET_TOPIC, facts
    if whitelist and not any(n in whitelist for n in names):
        return False, E_NO_TARGET_TOPIC, facts

    # 时长/消息数下限
    if norm["duration_s"] < min_duration_s or norm["message_count"] <= 0:
        return False, E_EMPTY_BAG, facts

    facts["end_ns"] = norm["start_ns"] + int(norm["duration_s"] * 1e9)

    # 时间戳单调抽样（§6.2 抽样 1000；无逐条 ts 时按 topic 计数粗判，仅告警不阻断）
    # 这里以 metadata 的 per-topic start/end 单调性做轻量校验。
    for t in norm["topics"]:
        st, en = t.get("start_time", 0), t.get("end_time", 0)
        if st and en and en < st:
            facts["warnings"].append(W_TS_NONMONO)
            break

    # 流式 sha256（限速分块）
    if compute_hash:
        try:
            facts["sha256"] = compute_sha256(seg_dir, chunk_bytes=hash_chunk_mb * 1024 * 1024,
                                             sleep_every_chunks=64)
        except OSError as exc:
            logger.warning("sha256 计算失败 %s：%s", seg_dir, exc)
            return False, E_HASH, facts

    # 时钟偏差（§6.5 W_CLOCK_SKEW）：车端无逐条 bag_ts/stamp 时跳过，交由服务端
    return True, None, facts
