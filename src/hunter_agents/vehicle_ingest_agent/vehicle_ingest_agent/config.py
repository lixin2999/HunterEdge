# -*- coding: utf-8 -*-
"""三层配置加载与校验（设计 §6.10、附录 A）。

优先级（低→高）：内置 ``DEFAULTS`` < 主配置 YAML（随版本发布）<
``<paths.root>/config.override.yaml``（远程 set_config 下发持久化）。

加载后做**结构性校验**（水位单调、分片体积区间、限速为正、时间窗可解析等），
非法配置：
  - 启动期非法 → 抛 :class:`ConfigError`（systemd 重启，运维可见）；
  - 热更新非法 → 由调用方拒绝并回 ``cmd.ack(failed)``，不覆盖内存中的有效配置。

凭据绝不落本文件：Kafka 走 hunter_kafka（properties），对象存储走 /etc/hunter/oss（见 uploader）。
"""
from __future__ import annotations

import copy
import hashlib
from typing import Any, Dict, List, Optional

try:
    import yaml
    HAS_YAML = True
except ImportError:  # 极窄兜底：无 PyYAML 时只能吃内置默认
    HAS_YAML = False

from .logging_util import get_logger

logger = get_logger("config")

# ── 内置默认（与 config/vehicle_ingest_agent.yaml 同构，保证缺文件也能起）──────────
DEFAULTS: Dict[str, Any] = {
    "vehicle_id": "",
    "agent_version": "1.0.0",
    "paths": {
        "root": "/var/lib/huntercore-agent",
        "bags_dir": "bags",
        "spool_dir": "spool",
        "state_dir": "state",
        "part_state_dir": "part-state",
        "logs_dir": "logs",
    },
    "recording": {
        "enabled": True,
        "topics": ["/lidar_points", "/camera/camera/color/image_raw",
                   "/imu/data", "/chassis/state"],
        "record_transport": "ros2",
        "max_segment_bytes": 500 * 1024 * 1024,
        "max_segment_duration_s": 600,
        "retention_max_segments": 10,
        "seal_grace_s": 3.0,
        "poll_interval_s": 2.0,
        "min_sealed_duration_s": 1.0,
    },
    "disk": {
        "spool_max_bytes": 150 * 1024 ** 3,
        "high_watermark": 0.85,
        "low_watermark": 0.70,
        "critical_watermark": 0.95,
        "evict_unuploaded_on_pressure": True,
        "gc": {"done_ttl_h": 48, "raw_ttl_d": 7},
    },
    "manifest": {
        "compute_sha256": True,
        "hash_chunk_mb": 8,
        "min_duration_s": 1.0,
        "target_topic_whitelist": [],
    },
    "parse": {
        "enabled": False,               # 降级模式：解析/切片/结构化放服务端
        "workers": 2,
        "pin_cores": [4, 5],
        "throttle": {"msg_per_sec": 20000, "nice": 10},
    },
    "upload": {
        "enabled": True,
        "endpoint_type": "s3",
        "endpoint": "",
        "region": "us-east-1",
        "bucket": "bags",
        "bucket_version": "v1",
        "use_tls": True,
        "access_key": "",
        "secret_key": "",
        "part_size_mb": 16,
        "concurrency": {"per_object_parts": 4, "objects": 3},
        "rate_limit_mbps": 160,
        "schedule": [],
        "max_attempts": 12,
        "retry_backoff_base_s": 1,
        "retry_backoff_max_s": 600,
        "keep_raw": "delete_on_uploaded",
    },
    "kafka": {
        "properties_path": "/etc/hunter/kafka/kafka.properties",
        "bundle_dir": "/etc/hunter/kafka",
        "outbox": {"flush_interval_ms": 100, "batch": 200, "max_pending": 500000,
                   "done_ttl_h": 24},
        "topic_map": {
            "manifest": "vehicle.bag.manifest",
            "task_status": "vehicle.bag.task.status",
            "parse_progress": "vehicle.bag.parse.progress",
            "artifact_uploaded": "vehicle.bag.artifact.uploaded",
            "heartbeat": "vehicle.agent.heartbeat",
            "alert": "vehicle.agent.alert",
            "cmd": "vehicle.agent.cmd",
            "cmd_ack": "vehicle.agent.cmd.ack",
        },
        "producer": {"acks_critical": "all", "acks_progress": "1",
                     "enable_idempotence": True, "compression": "lz4",
                     "message_max_bytes": 4194304, "message_timeout_ms": 600000},
        "fallback_brokers": "",
        "fallback_username": "",
        "fallback_password": "",
    },
    "clock": {"skew_warn_ms": 200},
    "telemetry": {
        "heartbeat_interval_s": 30,
        "log_max_bytes": 50 * 1024 * 1024,
        "log_backup_count": 10,
    },
}


class ConfigError(ValueError):
    """配置结构非法（启动期致命；热更新期由上层拒绝）。"""


def _deep_merge(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    """递归合并：``override`` 覆盖 ``base``；同为 dict 则深合并，其余整体替换。"""
    out = copy.deepcopy(base)
    for key, val in (override or {}).items():
        if isinstance(val, dict) and isinstance(out.get(key), dict):
            out[key] = _deep_merge(out[key], val)
        else:
            out[key] = copy.deepcopy(val)
    return out


def _load_yaml(path: str) -> Dict[str, Any]:
    if not path:
        return {}
    import os
    if not os.path.isfile(path):
        return {}
    if not HAS_YAML:
        logger.warning("缺 PyYAML，忽略 YAML 层配置：%s（仅用内置默认）", path)
        return {}
    with open(path, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}
    if not isinstance(data, dict):
        raise ConfigError(f"配置文件根节点必须是映射：{path}")
    return data


def _parse_window(window: str) -> Optional[List[int]]:
    """把 "09:00-18:00" 解析为 [start_min, end_min]；非法返回 None。"""
    try:
        start, end = window.split("-", 1)
        sh, sm = start.strip().split(":", 1)
        eh, em = end.strip().split(":", 1)
        smin = int(sh) * 60 + int(sm)
        emin = int(eh) * 60 + int(em)
        if not (0 <= smin <= 1440 and 0 <= emin <= 1440):
            return None
        return [smin, emin]
    except (ValueError, AttributeError):
        return None


class AgentConfig:
    """配置访问对象：点号路径取值 + 计算派生属性（路径、config_rev、限速窗口）。"""

    def __init__(self, data: Dict[str, Any], source_path: str = ""):
        self._d = data
        self.source_path = source_path
        self._rev = hashlib.sha1(
            _canonical_json(data).encode("utf-8")).hexdigest()

    # ── 取值 ──
    def get(self, dotted: str, default: Any = None) -> Any:
        node: Any = self._d
        for part in dotted.split("."):
            if not isinstance(node, dict) or part not in node:
                return default
            node = node[part]
        return node

    def raw(self) -> Dict[str, Any]:
        return copy.deepcopy(self._d)

    @property
    def config_rev(self) -> str:
        return "sha1:" + self._rev

    # ── 便捷属性 ──
    @property
    def vehicle_id(self) -> str:
        return self.get("vehicle_id", "") or ""

    @property
    def agent_version(self) -> str:
        return self.get("agent_version", "0.0.0")

    @property
    def root(self) -> str:
        return self.get("paths.root")

    def path(self, sub: str) -> str:
        import os
        return os.path.join(self.root, self.get("paths." + sub))

    @property
    def bags_dir(self) -> str:
        return self.path("bags_dir")

    @property
    def spool_dir(self) -> str:
        return self.path("spool_dir")

    @property
    def state_dir(self) -> str:
        return self.path("state_dir")

    @property
    def part_state_dir(self) -> str:
        return self.path("part_state_dir")

    @property
    def logs_dir(self) -> str:
        return self.path("logs_dir")

    @property
    def db_path(self) -> str:
        import os
        return os.path.join(self.state_dir, "agent.db")

    @property
    def parse_enabled(self) -> bool:
        return bool(self.get("parse.enabled", False))

    def current_rate_limit_mbps(self, now_min: int) -> float:
        """按时间窗返回当前限速（无匹配窗口用基础 rate_limit_mbps）。"""
        base = float(self.get("upload.rate_limit_mbps", 160))
        for item in self.get("upload.schedule", []) or []:
            win = _parse_window(item.get("window", ""))
            if not win:
                continue
            s, e = win
            inside = (s <= now_min <= e) if s <= e else (now_min >= s or now_min <= e)
            if inside:
                return float(item.get("rate_mbps", base))
        return base


def _canonical_json(data: Any) -> str:
    import json
    return json.dumps(data, sort_keys=True, ensure_ascii=False)


def validate(cfg: AgentConfig) -> None:
    """结构不变量校验；违反抛 :class:`ConfigError`。"""
    d = cfg.get("disk")
    lo, hi, crit = d["low_watermark"], d["high_watermark"], d["critical_watermark"]
    if not (0 < lo <= hi <= crit < 1.0):
        raise ConfigError(f"磁盘水位需满足 0<low<=high<=critical<1，现为 {lo}/{hi}/{crit}")
    if d["spool_max_bytes"] <= 0:
        raise ConfigError("disk.spool_max_bytes 必须为正")

    r = cfg.get("recording")
    if r["max_segment_bytes"] <= 0:
        raise ConfigError("recording.max_segment_bytes 必须为正（500MB 切分）")
    if r["max_segment_duration_s"] <= 0:
        raise ConfigError("recording.max_segment_duration_s 必须为正（10min 切分）")
    if r["retention_max_segments"] < 1:
        raise ConfigError("recording.retention_max_segments 至少为 1（环形保留 N 个）")
    if r["poll_interval_s"] <= 0:
        raise ConfigError("recording.poll_interval_s 必须为正")
    if r["record_transport"] not in ("ros2", "none"):
        raise ConfigError("recording.record_transport ∈ {ros2, none}")

    u = cfg.get("upload")
    if not (8 <= u["part_size_mb"] <= 32):
        raise ConfigError("upload.part_size_mb 需在 8~32 之间（设计 §6.7）")
    if u["endpoint_type"] not in ("s3", "gateway"):
        raise ConfigError("upload.endpoint_type ∈ {s3, gateway}")
    if u["max_attempts"] < 1:
        raise ConfigError("upload.max_attempts 至少为 1")
    if float(u["rate_limit_mbps"]) <= 0:
        raise ConfigError("upload.rate_limit_mbps 必须为正")
    if u["keep_raw"] not in ("delete_on_uploaded", "keep", "after_confirmed"):
        raise ConfigError("upload.keep_raw ∈ {delete_on_uploaded, keep, after_confirmed}")
    for item in u.get("schedule", []) or []:
        if _parse_window(item.get("window", "")) is None:
            raise ConfigError(f"upload.schedule 时间窗非法：{item.get('window')!r}")

    o = cfg.get("kafka.outbox")
    if o["flush_interval_ms"] < 10:
        raise ConfigError("kafka.outbox.flush_interval_ms 过小")
    if o["batch"] < 1:
        raise ConfigError("kafka.outbox.batch 至少为 1")


def load_config(main_path: str,
                override_path: Optional[str] = None) -> AgentConfig:
    """按三层优先级装配配置并校验。

    ``override_path`` 默认 ``<root>/config.override.yaml``（root 取自主配置，缺则默认）。
    """
    main = _load_yaml(main_path)
    merged = _deep_merge(DEFAULTS, main)
    # override 落在合并后 root 下（与主配置 root 一致）
    root = merged["paths"]["root"]
    import os
    ov_path = override_path or os.path.join(root, "config.override.yaml")
    override = _load_yaml(ov_path)
    if override:
        merged = _deep_merge(merged, override)
        logger.info("已叠加远程配置：%s", ov_path)
    cfg = AgentConfig(merged, source_path=main_path)
    validate(cfg)
    logger.info("配置加载完成 rev=%s parse_enabled=%s root=%s",
                cfg.config_rev, cfg.parse_enabled, cfg.root,
                extra={"component": "config"})
    return cfg
