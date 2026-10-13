# -*- coding: utf-8 -*-
"""结构化日志（设计 §6.11）：JSON lines + 大小轮转（50MB×10），同时透传 stdout（journald）。

约定字段：ts / level / logger / msg / bag_id / task_state / component / err_code。
业务代码通过 :func:`get_logger` 取 logger，用 :meth:`JsonLogger.bind` 附带上下文，
或用 ``extra={"bag_id": ...}`` 直接注入结构化字段（本模块会搬到 JSON 顶层）。
"""
from __future__ import annotations

import json
import logging
import os
import time
from logging.handlers import RotatingFileHandler
from typing import Any, Dict, Optional

_RESERVED = set(
    ("args", "exc_info", "exc_text", "filename", "funcName", "levelname", "levelno",
     "lineno", "module", "msecs", "message", "msg", "name", "pathname", "stack_info",
     "taskName", "created", "relativeCreated", "thread", "threadName", "process",
     "processName"))


class JsonFormatter(logging.Formatter):
    """把 LogRecord 渲染成一行 JSON；带 ``bag_id``/``component``/``err_code`` 等结构化字段。"""

    def format(self, record: logging.LogRecord) -> str:
        payload: Dict[str, Any] = {
            "ts": time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime(record.created))
            + ".%03dZ" % int(record.msecs),
            "level": record.levelname,
            "logger": record.name,
            "msg": record.getMessage(),
        }
        for key, val in record.__dict__.items():
            if key not in _RESERVED and not key.startswith("_"):
                payload[key] = val
        if record.exc_info:
            payload["exc"] = self.formatException(record.exc_info)
        return json.dumps(payload, ensure_ascii=False, default=str)


def _console() -> logging.Handler:
    h = logging.StreamHandler()
    h.setFormatter(JsonFormatter())
    return h


def setup_logging(log_dir: str, max_bytes: int = 52428800, backup_count: int = 10,
                  level: int = logging.INFO) -> None:
    """根 logger 接 JSON lines 文件（轮转）+ stdout；重复调用幂等。"""
    root = logging.getLogger()
    root.setLevel(level)
    for h in list(root.handlers):
        root.removeHandler(h)
    root.addHandler(_console())
    if log_dir:
        try:
            os.makedirs(log_dir, exist_ok=True)
            fh = RotatingFileHandler(
                os.path.join(log_dir, "vehicle_ingest_agent.log"),
                maxBytes=max_bytes, backupCount=backup_count, encoding="utf-8")
            fh.setFormatter(JsonFormatter())
            root.addHandler(fh)
        except OSError as exc:  # 磁盘/权限问题不能阻断启动，退化为仅 stdout
            root.warning("日志文件初始化失败，仅输出到 journald：%s", exc)


def get_logger(name: str) -> logging.Logger:
    return logging.getLogger(name if name.startswith("via.") else "via." + name)


def log_extra(**kwargs: Any) -> Dict[str, Any]:
    """构造 ``extra`` dict（过滤 None，避免把空字段刷进每一行日志）。"""
    return {k: v for k, v in kwargs.items() if v is not None}


class BoundLogger:
    """轻量上下文绑定：``log = get_logger('recorder').bind(component='recorder')``。"""

    def __init__(self, logger: logging.Logger, ctx: Optional[Dict[str, Any]] = None):
        self._log = logger
        self._ctx = dict(ctx or {})

    def bind(self, **kwargs: Any) -> "BoundLogger":
        merged = dict(self._ctx)
        merged.update({k: v for k, v in kwargs.items() if v is not None})
        return BoundLogger(self._log, merged)

    def _emit(self, level: int, msg: str, **extra: Any) -> None:
        merged = dict(self._ctx)
        merged.update({k: v for k, v in extra.items() if v is not None})
        self._log.log(level, msg, extra=merged)

    def debug(self, msg: str, **e: Any) -> None:
        self._emit(logging.DEBUG, msg, **e)

    def info(self, msg: str, **e: Any) -> None:
        self._emit(logging.INFO, msg, **e)

    def warning(self, msg: str, **e: Any) -> None:
        self._emit(logging.WARNING, msg, **e)

    def error(self, msg: str, **e: Any) -> None:
        self._emit(logging.ERROR, msg, **e)

    def exception(self, msg: str, **e: Any) -> None:
        merged = dict(self._ctx)
        merged.update({k: v for k, v in e.items() if v is not None})
        self._log.exception(msg, extra=merged)
