# -*- coding: utf-8 -*-
"""vehicle_ingest_agent 主编排（设计 §3.2、§5 端到端流程；需求 2 环形释放闭环）。

单进程 asyncio：
  recorder  ─封段─▶ pipeline queue ─▶ _process（verify→manifest→route→upload）
  uploader  ─对象完成/失败─▶ 记账 ─▶ 片段全完成 ─▶ 状态机 uploading_done→done
                                          └─▶ spool.apply_after_upload（上传成功即删，protected 保留）
  kafka_bus flush loop（outbox at-least-once）+ cmd consumer + heartbeat + spool 周期治理。

降级模式（默认）：不解析，manifest_ready → uploading_raw（原始片段 + manifest 上传），
解析/切片/结构化交服务端。任何单点异常都被隔离在对应 task，绝不拖垮主循环（§8）。
"""
from __future__ import annotations

import argparse
import asyncio
import os
import signal
import time
from typing import Any, Dict, Optional

from . import verifier
from .cmd_consumer import CommandConsumer
from .config import load_config
from .heartbeat import Heartbeat
from .logging_util import get_logger, setup_logging
from .manifest import build_manifest, write_manifest
from .parse_scheduler import ParseScheduler, should_parse
from .recorder import SegmentRecorder
from .spool import SpoolManager
from .state_store import StateStore
from .status_machine import TaskStateMachine
from .uploader import UploadEngine, UploadUnavailable

logger = get_logger("agent")


class VehicleIngestAgent:
    def __init__(self, cfg):
        self.cfg = cfg
        self.store: Optional[StateStore] = None
        self.machine: Optional[TaskStateMachine] = None
        self.spool: Optional[SpoolManager] = None
        self.bus = None
        self.engine: Optional[UploadEngine] = None
        self.recorder: Optional[SegmentRecorder] = None
        self.parser: Optional[ParseScheduler] = None
        self.cmd: Optional[CommandConsumer] = None
        self.heartbeat: Optional[Heartbeat] = None
        self._pipeline: "asyncio.Queue[str]" = asyncio.Queue()
        self._expected: Dict[str, int] = {}
        self._done: Dict[str, int] = {}
        self._failed: Dict[str, int] = {}
        self._manifests: Dict[str, str] = {}
        self._stop = asyncio.Event()
        self._tasks: list = []

    # ── 告警枢纽 ──
    def _alert(self, code: str, severity: str, detail: Dict[str, Any]) -> None:
        logger.warning("告警 %s[%s] %s", code, severity, detail,
                       extra={"err_code": code, "component": "agent", **detail})
        if self.machine:
            self.machine.emit_alert(code, severity, detail)

    # ── 装配 ──
    async def setup(self) -> None:
        for d in (self.cfg.bags_dir, self.cfg.spool_dir, self.cfg.state_dir,
                  self.cfg.part_state_dir, self.cfg.logs_dir):
            os.makedirs(d, exist_ok=True)

        self.store = StateStore(self.cfg.db_path)
        self.machine = TaskStateMachine(self.cfg, self.store)
        self.spool = SpoolManager(self.cfg, self.store, alert=self._alert)

        from .kafka_bus import KafkaBus
        self.bus = KafkaBus(self.cfg, self.store)
        self.bus.vehicle_id = self.bus.vehicle_id or self.cfg.vehicle_id
        self.bus.start()
        if self.bus.vehicle_id and self.bus.vehicle_id != self.cfg.vehicle_id:
            self.cfg._d["vehicle_id"] = self.bus.vehicle_id   # 接入包 SASL 用户名为准（§5.6）

        self.parser = ParseScheduler(self.cfg, self.store, self.machine)
        self.engine = UploadEngine(self.cfg, self.store, self.cfg.get("upload.bucket_version", "v1"))
        try:
            await self.engine.start(self._on_object_done, self._on_object_fail)
        except UploadUnavailable as exc:
            self._alert("E_UPLOAD_UNAVAILABLE", "critical", {"error": str(exc)})
            logger.error("上传通道不可用，片段将滞留本地并周期性重试：%s", exc)

        self.recorder = SegmentRecorder(self.cfg, self.store, self._on_sealed, self._alert)
        if self.recorder.enabled:
            await self.recorder.start()

        handlers = {
            "seal": self._cmd_seal, "reparse": self._cmd_reparse,
            "reupload": self._cmd_reupload, "set_config": self._cmd_set_config,
            "cleanup": self._cmd_cleanup, "pause": self._cmd_pause,
            "resume": self._cmd_resume, "purge_outbox": self._cmd_purge,
        }
        self.cmd = CommandConsumer(self.cfg, self.store, self.bus, self.machine, handlers)
        self.heartbeat = Heartbeat(self.cfg, self.store, self.machine, self.spool,
                                   extra_metrics=self._extra_metrics)

        await self._recover()
        logger.info("Agent 装配完成 vehicle_id=%s parse=%s upload=%s kafka=%s",
                    self.cfg.vehicle_id, should_parse(self.cfg),
                    self.cfg.get("upload.enabled"), self.bus.available,
                    extra={"component": "agent"})

    def _extra_metrics(self) -> Dict[str, Any]:
        return {
            "kafka_available": bool(self.bus and self.bus.available),
            "upload_inflight": sum(
                max(0, self._expected.get(b, 0) - self._done.get(b, 0) - self._failed.get(b, 0))
                for b in self._expected),
            "pipeline_backlog": self._pipeline.qsize(),
        }

    # ── 恢复（§6.9：非终态任务续作）──
    async def _recover(self) -> None:
        tasks = self.store.list_nonterminal_tasks()
        if not tasks:
            return
        logger.info("启动恢复：%d 个非终态任务", len(tasks), extra={"component": "agent"})
        for t in tasks:
            bag_id = t["bag_id"]
            state = t["state"]
            seg = self.store.get_segment(bag_id)
            if state in ("uploading_raw", "uploading", "upload_failed") and seg and self.engine:
                count = self.engine.enqueue_segment(seg, self._manifests.get(bag_id))
                self._expected[bag_id] = count
                self._done[bag_id] = self._failed[bag_id] = 0
            else:
                await self._pipeline.put(bag_id)   # 早阶段（或上传不可用）重新走流水线

    # ── 录制封段回调：入流水线 ──
    async def _on_sealed(self, seg: Dict[str, Any]) -> None:
        await self._pipeline.put(seg["bag_id"])

    # ── 流水线 worker ──
    async def _pipeline_worker(self) -> None:
        loop = asyncio.get_running_loop()
        while not self._stop.is_set():
            try:
                bag_id = await asyncio.wait_for(self._pipeline.get(), timeout=1.0)
            except asyncio.TimeoutError:
                continue
            except asyncio.CancelledError:
                break
            try:
                await self._process(loop, bag_id)
            except Exception as exc:  # noqa: BLE001  单任务失败隔离
                logger.exception("流水线处理异常 bag_id=%s：%s", bag_id, exc,
                                 extra={"bag_id": bag_id, "component": "agent"})
            finally:
                self._pipeline.task_done()

    async def _process(self, loop, bag_id: str) -> None:
        seg = self.store.get_segment(bag_id)
        if not seg:
            return
        # verifying（§6.2）
        self.machine.transition(bag_id, "verifying")
        passed, err_code, facts = await self._verify(seg)
        if not passed:
            self.machine.transition(bag_id, "quarantined", reason=err_code)
            self._alert(err_code or "E_VERIFY", "error", {"bag_id": bag_id})
            return
        # manifest（§6.2/§7.1）
        facts["path"] = seg["path"]
        facts["size_bytes"] = seg["size_bytes"]
        m = build_manifest(bag_id, self.cfg, facts, protected=bool(seg.get("protected")))
        spool_bag = os.path.join(self.cfg.spool_dir, bag_id)
        manifest_path = write_manifest(spool_bag, m)
        self._manifests[bag_id] = manifest_path
        self.store.update_segment(bag_id, sha256=facts.get("sha256", ""),
                                  size_bytes=facts.get("size_bytes", seg["size_bytes"]))
        res = self.machine.transition(bag_id, "manifest_ready")
        self.machine.emit_manifest(bag_id, res["seq"], m)

        # 路由：解析启用→parsing（本默认未实现解析器，随即降级兜底）；否则原样上传
        if should_parse(self.cfg):
            self.machine.transition(bag_id, "parsing")
            out = self.parser.process(bag_id, m)
            if out.get("parsed"):
                self.machine.transition(bag_id, "parsed")
                self.machine.transition(bag_id, "uploading")
                self._begin_upload(bag_id, manifest_path)
                return
            self.machine.transition(bag_id, "parse_failed", reason="E_PARSER_NA")
        self.machine.transition(bag_id, "uploading_raw")
        self._begin_upload(bag_id, manifest_path)

    async def _verify(self, seg: Dict[str, Any]):
        loop = asyncio.get_running_loop()
        import functools
        fn = functools.partial(
            verifier.verify_segment, seg["path"],
            whitelist=self.cfg.get("manifest.target_topic_whitelist", []) or [],
            min_duration_s=float(self.cfg.get("manifest.min_duration_s", 1.0)),
            compute_hash=bool(self.cfg.get("manifest.compute_sha256", True)),
            hash_chunk_mb=int(self.cfg.get("manifest.hash_chunk_mb", 8)),
            skew_warn_ms=int(self.cfg.get("clock.skew_warn_ms", 200)))
        return await loop.run_in_executor(None, fn)

    def _begin_upload(self, bag_id: str, manifest_path: str) -> None:
        if not self.cfg.get("upload.enabled", True) or self.engine is None:
            logger.warning("上传已禁用/暂停，片段滞留本地不删 bag_id=%s", bag_id,
                           extra={"bag_id": bag_id, "component": "agent"})
            return
        seg = self.store.get_segment(bag_id)
        count = self.engine.enqueue_segment(seg, manifest_path)
        self._expected[bag_id] = count
        self._done[bag_id] = 0
        self._failed[bag_id] = 0
        if count == 0:
            # 无可上传对象（异常）→ 直接完成避免悬挂
            self._finalize_uploaded(bag_id)

    # ── 上传对象完成 / 失败（引擎回调，运行于事件循环）──
    def _on_object_done(self, bag_id: str, job, etag: str, duration_ms: int) -> None:
        if self.machine:
            task = self.store.get_task(bag_id)
            seq = task["seq"] if task else 0
            self.machine.emit_artifact_uploaded(
                bag_id, seq, artifact_class=job.artifact_class, object_key=job.object_key,
                size_bytes=job.size, sha256=job.sha256, etag=etag,
                upload_duration_ms=duration_ms,
                endpoint=self.cfg.get("upload.endpoint", ""))
        self._done[bag_id] = self._done.get(bag_id, 0) + 1
        self._maybe_finalize(bag_id)

    def _on_object_fail(self, bag_id: str, exc: Exception) -> None:
        self._failed[bag_id] = self._failed.get(bag_id, 0) + 1
        self._maybe_finalize(bag_id)

    def _maybe_finalize(self, bag_id: str) -> None:
        expected = self._expected.get(bag_id)
        if expected is None:
            return
        if self._done.get(bag_id, 0) + self._failed.get(bag_id, 0) >= expected:
            if self._failed.get(bag_id, 0) > 0:
                self._finalize_failed(bag_id)
            else:
                self._finalize_uploaded(bag_id)

    def _finalize_uploaded(self, bag_id: str) -> None:
        task = self.store.get_task(bag_id)
        if not task or task["state"] == "done":
            return
        state = task["state"]
        if state in ("uploading_raw", "uploading"):
            self.machine.transition(bag_id, "uploading_done")
        else:
            self.machine.transition(bag_id, "uploading_done", force=True)
        self.machine.transition(bag_id, "done")
        # 需求 2：上传成功后按策略释放本地（protected 不删）
        try:
            self.spool.apply_after_upload(bag_id)
        except Exception as exc:  # noqa: BLE001
            logger.exception("上传成功后释放失败 bag_id=%s：%s", bag_id, exc,
                             extra={"bag_id": bag_id})
        self._expected.pop(bag_id, None)
        self._done.pop(bag_id, None)
        self._failed.pop(bag_id, None)
        self._manifests.pop(bag_id, None)

    def _finalize_failed(self, bag_id: str) -> None:
        task = self.store.get_task(bag_id)
        if not task:
            return
        try:
            self.machine.transition(bag_id, "upload_failed", reason="E_UPLOAD_PART_FAILED",
                                    force=True)
        except Exception as exc:  # noqa: BLE001
            logger.warning("置 upload_failed 转移异常 bag_id=%s：%s", bag_id, exc)
        self._alert("E_UPLOAD_PART_FAILED", "error", {"bag_id": bag_id})
        # 保留现场：不删本地（§6.7/§8），等 reupload / 自动续传 / 水位淘汰
        self._expected.pop(bag_id, None)
        self._done.pop(bag_id, None)
        self._failed.pop(bag_id, None)

    # ── outbox flush 循环（§6.8.1）──
    async def _flush_loop(self) -> None:
        interval = float(self.cfg.get("kafka.outbox.flush_interval_ms", 100)) / 1000.0
        while not self._stop.is_set():
            try:
                if self.bus and self.bus.available:
                    self.bus.flush_once()
            except Exception as exc:  # noqa: BLE001
                logger.warning("outbox flush 异常：%s", exc, extra={"component": "agent"})
            try:
                await asyncio.wait_for(self._stop.wait(), timeout=interval)
            except asyncio.TimeoutError:
                pass

    # ── spool 周期治理（环形/水位/TTL）──
    async def _spool_loop(self) -> None:
        period = max(5.0, float(self.cfg.get("recording.poll_interval_s", 2)) * 5)
        while not self._stop.is_set():
            try:
                self.spool.enforce()
            except Exception as exc:  # noqa: BLE001
                logger.exception("spool 治理异常：%s", exc, extra={"component": "agent"})
            try:
                await asyncio.wait_for(self._stop.wait(), timeout=period)
            except asyncio.TimeoutError:
                pass

    # ── 命令 handlers（§6.8.5）──
    async def _cmd_seal(self, params: Dict[str, Any]):
        await self.recorder.force_seal()
        return "SUCCEEDED", {"sealed": True}

    async def _cmd_reparse(self, params: Dict[str, Any]):
        bag_id = params.get("bag_id")
        if not bag_id or not self.store.get_task(bag_id):
            return "REJECTED", "unknown bag_id"
        self.machine.transition(bag_id, "manifest_ready", force=True)
        await self._pipeline.put(bag_id)
        return "ACCEPTED", {"bag_id": bag_id}

    async def _cmd_reupload(self, params: Dict[str, Any]):
        bag_id = params.get("bag_id")
        seg = self.store.get_segment(bag_id) if bag_id else None
        if not seg:
            return "REJECTED", "unknown/unpresent bag_id"
        count = self.engine.enqueue_segment(seg, self._manifests.get(bag_id))
        self._expected[bag_id] = count
        self._done[bag_id] = self._failed[bag_id] = 0
        return "ACCEPTED", {"objects": count}

    async def _cmd_set_config(self, params: Dict[str, Any]):
        """写 config.override.yaml（远程下发持久化），校验；非法拒绝（§6.10）。"""
        from .config import ConfigError, load_config as _lc, validate
        patch = params.get("patch") or params
        try:
            import yaml
            ov_path = os.path.join(self.cfg.root, "config.override.yaml")
            cur = {}
            if os.path.isfile(ov_path):
                cur = yaml.safe_load(open(ov_path, encoding="utf-8")) or {}
            _deep = _merge(cur, patch)
            with open(ov_path, "w", encoding="utf-8") as fh:
                yaml.safe_dump(_deep, fh, allow_unicode=True)
            new_cfg = _lc(self.cfg.source_path)     # 会叠加重写的 override 并校验
            validate(new_cfg)
        except (ConfigError, Exception) as exc:  # noqa: BLE001
            self._alert("E_CONFIG_INVALID", "warning", {"error": str(exc)})
            return "FAILED", f"invalid config: {exc}"
        self.cfg = new_cfg
        return "SUCCEEDED", {"config_rev": new_cfg.config_rev,
                             "note": "解析/限速/水位下一任务生效；watch.roots/kafka 需重启组件"}

    async def _cmd_cleanup(self, params: Dict[str, Any]):
        ids = params.get("bag_ids") or ([params["bag_id"]] if params.get("bag_id") else [])
        return "SUCCEEDED", {"cleaned": self.spool.cleanup_confirmed(ids)}

    async def _cmd_pause(self, params: Dict[str, Any]):
        comp = params.get("component", "recorder")
        if comp == "recorder":
            self.recorder.pause()
        elif comp == "uploader" and self.engine:
            await self.engine.stop()
            self.engine = None
        return "SUCCEEDED", {"paused": comp}

    async def _cmd_resume(self, params: Dict[str, Any]):
        comp = params.get("component", "recorder")
        if comp == "recorder":
            self.recorder.resume()
        elif comp == "uploader" and self.engine is None:
            self.engine = UploadEngine(self.cfg, self.store,
                                       self.cfg.get("upload.bucket_version", "v1"))
            await self.engine.start(self._on_object_done, self._on_object_fail)
        return "SUCCEEDED", {"resumed": comp}

    async def _cmd_purge(self, params: Dict[str, Any]):
        before = float(params.get("before", time.time()))
        removed = self.store.purge_done_before(before)
        return "SUCCEEDED", {"purged": removed}

    # ── 运行 ──
    async def run(self) -> None:
        await self.setup()
        loop = asyncio.get_running_loop()
        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                loop.add_signal_handler(sig, self._stop.set)
            except (NotImplementedError, ValueError):
                pass   # Windows/非主线程兜底：靠 KeyboardInterrupt

        self._tasks = [
            asyncio.create_task(self._pipeline_worker(), name="pipeline"),
            asyncio.create_task(self._flush_loop(), name="flush"),
            asyncio.create_task(self._spool_loop(), name="spool"),
            asyncio.create_task(self.cmd.run(), name="cmd"),
            asyncio.create_task(self.heartbeat.run(), name="heartbeat"),
        ]
        logger.info("Agent 运行中（Ctrl-C / SIGINT 触发 drain 退出）", extra={"component": "agent"})
        await self._stop.wait()
        await self._drain()

    async def _drain(self) -> None:
        logger.info("开始 drain：停录制/上传/消费，flush outbox", extra={"component": "agent"})
        if self.recorder:
            await self.recorder.stop()          # seal 当前段（SIGINT 收尾 metadata）
        if self.cmd:
            self.cmd.stop()
        if self.heartbeat:
            self.heartbeat.stop()
        for t in self._tasks:
            t.cancel()
        for t in self._tasks:
            try:
                await t
            except (asyncio.CancelledError, Exception):  # noqa: BLE001
                pass
        if self.engine:
            await self.engine.stop()
        # 退出前尽力把 outbox 打空（弱网可滞留，恢复后补齐）
        if self.bus and self.bus.available:
            for _ in range(20):
                if self.store.count_pending() == 0:
                    break
                self.bus.flush_once()
                await asyncio.sleep(0.1)
            self.bus.shutdown()
        if self.store:
            self.store.close()
        logger.info("Agent 已停止", extra={"component": "agent"})


def _merge(base: Dict[str, Any], over: Dict[str, Any]) -> Dict[str, Any]:
    out = dict(base)
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _merge(out[k], v)
        else:
            out[k] = v
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description="HunterCore vehicle rosbag ingest agent")
    parser.add_argument("--config",
                        default=os.environ.get("HUNTER_VIA_CONFIG",
                                               "/etc/hunter/vehicle_ingest_agent.yaml"),
                        help="主配置 YAML（不含任何 Kafka/OSS 凭据）")
    args = parser.parse_args()

    cfg = load_config(args.config)
    setup_logging(cfg.logs_dir,
                  max_bytes=int(cfg.get("telemetry.log_max_bytes", 52428800)),
                  backup_count=int(cfg.get("telemetry.log_backup_count", 10)))
    logger.info("vehicle_ingest_agent 启动 config=%s", args.config,
                extra={"component": "agent"})

    agent = VehicleIngestAgent(cfg)
    try:
        asyncio.run(agent.run())
    except KeyboardInterrupt:
        pass   # systemd KillSignal=SIGINT：asyncio.run 会取消任务，_drain 已尽力收尾


if __name__ == "__main__":
    main()
