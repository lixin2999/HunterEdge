# -*- coding: utf-8 -*-
"""下行命令消费（设计 §6.8.5、§10.6）。

- 消费 ``vehicle.agent.cmd``，key=vehicle_id（客户端过滤只认自己车）；
- **逐条幂等**：``cmd_id`` 落 SQLite ``cmd_dedup``，重复投递只回 DUPLICATE；
- **验签**（§10.6）：报文带 ``sig``（HMAC-SHA256，密钥仅服务端与设备持有），验不过丢弃 +
  ``E_CMD_SIG_INVALID`` 告警；破坏性指令（cleanup/purge）要求 ``ops_approved`` 标志；
- 执行结果统一回 ``vehicle.agent.cmd.ack``（acks=all）；处理 + 回执证实后才 commit offset。

具体动作由 agent 注入的 ``handlers``（command_type → async callable(params)→(code,data)）落地，
本模块只做"过滤 / 去重 / 验签 / 分发 / 回执 / 提交"的通用护栏。
"""
from __future__ import annotations

import hashlib
import hmac
import json
import os
import time
from typing import Any, Awaitable, Callable, Dict, Optional, Tuple

from . import events
from .logging_util import get_logger

logger = get_logger("cmd")

Handler = Callable[[Dict[str, Any]], Awaitable[Tuple[str, Any]]]

#: 破坏性指令（需 ops_approved 标志位，§10.6）
DESTRUCTIVE = {"cleanup", "purge_outbox", "seal"}


class CommandConsumer:
    def __init__(self, cfg, store, kafka_bus, machine, handlers: Dict[str, Handler]):
        self.cfg = cfg
        self.store = store
        self.bus = kafka_bus
        self.machine = machine
        self.handlers = handlers
        self.running = False
        self._hmac_key = os.environ.get("HUNTER_CMD_HMAC_KEY", "").encode("utf-8")
        tm = cfg.get("kafka.topic_map", {})
        self.cmd_topic = tm.get("cmd", "vehicle.agent.cmd")
        self.ack_topic = tm.get("cmd_ack", "vehicle.agent.cmd.ack")
        self.vehicle_id = cfg.vehicle_id

    async def run(self) -> None:
        """asyncio 任务：轮询 cmd（run_in_executor 避免阻塞事件循环）。"""
        import asyncio
        self.running = True
        loop = asyncio.get_running_loop()
        logger.info("命令消费者启动，订阅 %s（group=cmd-%s）", self.cmd_topic, self.vehicle_id,
                    extra={"component": "cmd"})
        while self.running:
            if not self.bus.available:
                await asyncio.sleep(2.0)
                continue
            try:
                msg = await loop.run_in_executor(None, self.bus.poll_cmd, 0.5)
            except Exception as exc:  # noqa: BLE001
                logger.warning("cmd 轮询异常：%s", exc, extra={"component": "cmd"})
                await asyncio.sleep(1.0)
                continue
            if msg is None:
                continue
            await self._handle(msg)
            self.bus.commit_cmd(msg)

    async def _handle(self, msg: Dict[str, Any]) -> None:
        raw = msg.get("value") or b"{}"
        try:
            cmd = json.loads(raw.decode("utf-8") if isinstance(raw, (bytes, bytearray)) else raw)
        except (UnicodeDecodeError, json.JSONDecodeError):
            logger.warning("cmd 报文非 JSON，丢弃")
            return
        if not isinstance(cmd, dict):
            return
        # key 过滤：只处理本车（§6.8.3 客户端过滤）
        key = (msg.get("key") or b"").decode("utf-8") if isinstance(
            msg.get("key"), (bytes, bytearray)) else msg.get("key")
        if key and key != self.vehicle_id:
            return

        cmd_id = cmd.get("cmd_id") or cmd.get("id") or "noid"
        ctype = cmd.get("command_type") or cmd.get("type") or cmd.get("cmd")
        params = cmd.get("params") or cmd.get("payload") or {}

        seen = self.store.cmd_seen(cmd_id)
        if seen:
            logger.debug("cmd 重复投递 cmd_id=%s", cmd_id, extra={"component": "cmd"})
            self._ack(cmd_id, ctype, "DUPLICATE", True, seen.get("result"))
            return

        # 验签（§10.6）
        if not self._verify_sig(cmd):
            self.machine.emit_alert("E_CMD_SIG_INVALID", "critical",
                                    {"cmd_id": cmd_id, "command_type": ctype})
            self.store.record_cmd(cmd_id, "REJECTED_SIG")
            self._ack(cmd_id, ctype, "REJECTED", False, "signature invalid")
            return

        # 破坏性指令 ops 门控
        if ctype in DESTRUCTIVE and not (cmd.get("ops_approved") or params.get("ops_approved")):
            self.store.record_cmd(cmd_id, "REJECTED_OPS")
            self._ack(cmd_id, ctype, "REJECTED", False, "destructive cmd needs ops_approved")
            return

        handler = self.handlers.get(ctype)
        if handler is None:
            self.store.record_cmd(cmd_id, "UNKNOWN")
            self._ack(cmd_id, ctype, "REJECTED", False, f"unknown command_type {ctype!r}")
            return
        try:
            code, data = await handler(params)
        except Exception as exc:  # noqa: BLE001
            logger.exception("cmd 执行异常 cmd_id=%s type=%s：%s", cmd_id, ctype, exc,
                             extra={"component": "cmd"})
            code, data = "FAILED", str(exc)
        self.store.record_cmd(cmd_id, code)
        self._ack(cmd_id, ctype, code, code in ("SUCCEEDED", "ACCEPTED"), data)

    def _verify_sig(self, cmd: Dict[str, Any]) -> bool:
        """无密钥（开发环境）放行；有密钥则强制校验（生产缺 sig 视为非法）。"""
        if not self._hmac_key:
            return True
        sig = cmd.get("sig")
        if not sig:
            return False
        body = {k: v for k, v in cmd.items() if k != "sig"}
        canon = json.dumps(body, sort_keys=True, ensure_ascii=False).encode("utf-8")
        expect = hmac.new(self._hmac_key, canon, hashlib.sha256).hexdigest()
        return hmac.compare_digest(expect, str(sig))

    def _ack(self, cmd_id: str, ctype: Optional[str], code: str, ok: bool,
             data: Any) -> None:
        payload = {"cmd_id": cmd_id, "command_type": ctype, "result_code": code,
                   "success": ok, "message": code, "data": data, "ts": int(time.time())}
        env = events.make_envelope("vehicle.agent.cmd.ack.v1", self.vehicle_id,
                                   self.cfg.agent_version, payload)
        self.store.enqueue_outbox(self.ack_topic, cmd_id, env)

    def stop(self) -> None:
        self.running = False
