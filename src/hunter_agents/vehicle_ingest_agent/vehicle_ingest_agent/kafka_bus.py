# -*- coding: utf-8 -*-
"""Kafka 总线：落盘 Outbox 生产者 + 下行命令消费者（设计 §6.8）。

复用 hunter_kafka 的凭据装配（单一可信源 /etc/hunter/kafka/kafka.properties），
但 topic 采用本 Agent 的 ``vehicle.*`` 命名（config.kafka.topic_map），故直接用
:func:`hunter_kafka.config.load_conf` 造 confluent 客户端，而非受契约枚举约束的
``make_producer/make_consumer``。

可靠性要点（对齐仓内既有踩坑记忆）：
  - 投递证实只走**逐条 on_delivery 闭包**（本版本 librdkafka 的 ``dr_cb`` 会报
    _INVALID_ARG，见 hunter_kafka.client 说明），闭包捕获 outbox_id 精确落库；
  - produce() 抛 BufferError（本地队列满）→ 保持 pending、下轮重试，不丢事件；
  - Kafka/confluent 不可用时**优雅降级**：outbox 继续本地累积，Agent 照常落地/上传，
    链路恢复由 flusher 自动补齐（§8 断网处置）。
"""
from __future__ import annotations

import asyncio
import json
from typing import Any, Dict, Optional, Set

from .logging_util import get_logger

logger = get_logger("kafka")

try:
    from hunter_kafka.config import (DEFAULT_BUNDLE_DIR, DEFAULT_PROPERTIES_PATH,
                                     KafkaConfigError, load_conf, resolve_vehicle_id,
                                     strip_meta)
    HAS_HUNTER_KAFKA = True
except ImportError:
    HAS_HUNTER_KAFKA = False
    DEFAULT_BUNDLE_DIR = "/etc/hunter/kafka"
    DEFAULT_PROPERTIES_PATH = "/etc/hunter/kafka/kafka.properties"
    KafkaConfigError = RuntimeError

    def load_conf(**_kw):  # type: ignore[misc]
        raise KafkaConfigError("hunter_kafka 不可用")

    def resolve_vehicle_id(_d, default=""):  # type: ignore[misc]
        return default

    def strip_meta(conf):  # type: ignore[misc]
        return {k: v for k, v in conf.items() if not str(k).startswith("__")}


#: 需要 acks=all 的 topic（manifest/task.status/alert/artifact.uploaded/cmd.ack）
def _critical_topics(topic_map: Dict[str, str]) -> Set[str]:
    keys = ("manifest", "task_status", "alert", "artifact_uploaded", "cmd_ack")
    return {topic_map[k] for k in keys if k in topic_map}


class KafkaBus:
    """持有两个 Producer（critical/progress）与一个 cmd Consumer，并驱动 outbox flush。"""

    def __init__(self, cfg, store):
        self.cfg = cfg
        self.store = store
        self.vehicle_id = cfg.vehicle_id
        self.topic_map: Dict[str, str] = dict(cfg.get("kafka.topic_map", {}))
        self._critical: Set[str] = _critical_topics(self.topic_map)
        self._prod_critical = None
        self._prod_progress = None
        self._consumer = None
        self.available = False
        self._last_error: Optional[str] = None

    # ── 生命周期 ──
    def start(self) -> None:
        if not HAS_HUNTER_KAFKA:
            self._last_error = "hunter_kafka 未安装/未 source，事件仅本地累积于 outbox"
            logger.error(self._last_error, extra={"component": "kafka"})
            return
        props_path = self.cfg.get("kafka.properties_path", DEFAULT_PROPERTIES_PATH)
        bundle_dir = self.cfg.get("kafka.bundle_dir", DEFAULT_BUNDLE_DIR)
        fb = self._fallback()
        try:
            self._prod_critical = self._make_producer("all", "ingest-critical",
                                                      props_path, bundle_dir, fb)
            self._prod_progress = self._make_producer("1", "ingest-progress",
                                                      props_path, bundle_dir, fb)
            self._consumer = self._make_consumer(props_path, bundle_dir, fb)
            # vehicle_id 以接入包 SASL 用户名为准（== 证书 CN）
            resolved = self._resolve_vid(props_path, bundle_dir, fb)
            if resolved and resolved != self.vehicle_id:
                logger.error("vehicle_id 冲突：配置=%s 接入包=%s，以接入包为准",
                             self.vehicle_id, resolved, extra={"component": "kafka"})
                self.vehicle_id = resolved
            self.available = True
            logger.info("Kafka 总线就绪：critical=%s progress=%s cmd=%s",
                        self._producer_ready(self._prod_critical),
                        self._producer_ready(self._prod_progress),
                        bool(self._consumer), extra={"component": "kafka",
                                                     "vehicle_id": self.vehicle_id})
        except KafkaConfigError as exc:
            self._last_error = str(exc)
            logger.error("Kafka 接入失败（降级为本地 outbox 累积）：%s", exc,
                         extra={"component": "kafka"})
        except Exception as exc:  # noqa: BLE001  构造期任何异常都不得拖垮 Agent 启动
            self._last_error = repr(exc)
            logger.exception("Kafka 客户端构造异常（降级）：%s", exc,
                             extra={"component": "kafka"})

    def _resolve_vid(self, props_path, bundle_dir, fb) -> str:
        try:
            conf = load_conf(properties_path=props_path, bundle_dir=bundle_dir,
                              role="producer", acks="1", fallback=fb)
            return resolve_vehicle_id(conf, self.cfg.vehicle_id)
        except Exception:  # noqa: BLE001
            return self.cfg.vehicle_id

    @staticmethod
    def _producer_ready(p) -> bool:
        return p is not None

    def _fallback(self) -> Optional[Dict[str, str]]:
        """无接入包的开发机兜底（生产留空）。"""
        brokers = self.cfg.get("kafka.fallback_brokers", "")
        if not brokers:
            return None
        return {
            "bootstrap.servers": brokers,
            "security.protocol": "plaintext",
            "client.id": f"vha-{self.cfg.vehicle_id}",
        }

    def _make_producer(self, acks: str, suffix: str, props_path, bundle_dir, fb):
        from confluent_kafka import Producer
        pconf = self.cfg.get("kafka.producer", {})
        # librdkafka message.timeout.ms 合法区间 0..2147483647（-1 会被拒 → 构造异常）。
        # 消息淘汰仍由 outbox 掌控（§6.8.1）：取一个足够长的有限值，超时失败后 on_delivery
        # 触发、outbox 行不置 done、后续 flush 重试（at-least-once）。
        mto = int(pconf.get("message_timeout_ms", 600000))
        mto = max(0, min(mto, 2147483647))
        extra = {
            "acks": acks,
            "enable.idempotence": str(pconf.get("enable_idempotence", True)).lower(),
            "compression.type": pconf.get("compression", "lz4"),
            "linger.ms": "50",
            "batch.size": "262144",
            "max.in.flight.requests.per.connection": "5",
            "message.timeout.ms": str(mto),
            "queue.buffering.max.ms": str(pconf.get("queue_buffering_ms", 100)),
            "message.max.bytes": str(pconf.get("message_max_bytes", 4194304)),
        }
        conf = load_conf(properties_path=props_path, bundle_dir=bundle_dir,
                         client_id=f"vha-{self.cfg.vehicle_id}-{suffix}",
                         role="producer", acks=acks, extra=extra, fallback=fb)
        conf = strip_meta(conf)
        conf.pop("dr_cb", None)
        conf.pop("dr_msg_cb", None)
        return Producer(conf)

    def _make_consumer(self, props_path, bundle_dir, fb):
        from confluent_kafka import Consumer
        cmd_topic = self.topic_map.get("cmd")
        if not cmd_topic:
            return None
        cconf = load_conf(
            properties_path=props_path, bundle_dir=bundle_dir,
            client_id=f"vha-{self.cfg.vehicle_id}-cmd", role="consumer",
            extra={"group.id": f"cmd-{self.cfg.vehicle_id}",
                   "enable.auto.commit": "false",
                   "auto.offset.reset": "earliest"},
            fallback=fb)
        cconf = strip_meta(cconf)
        cconf["enable.auto.commit"] = "false"
        consumer = Consumer(cconf)
        consumer.subscribe([cmd_topic])
        return consumer

    # ── outbox flush（在事件循环里按 tick 调用）──
    def flush_once(self) -> int:
        """取一批 pending → produce（逐条 on_delivery 落库），返回本次投递条数。

        非阻塞：produce 后调用 ``poll(0)`` 驱动已完成回调（§附录 B 语义）。
        """
        if not self.available:
            return 0
        batch = int(self.cfg.get("kafka.outbox.batch", 200))
        rows = self.store.fetch_pending(batch)
        n = 0
        for r in rows:
            topic = r["topic"]
            prod = self._prod_critical if topic in self._critical else self._prod_progress
            if prod is None:
                break
            cb = self._make_cb(int(r["id"]))
            try:
                prod.produce(topic, key=str(r["pkey"]).encode("utf-8"),
                             value=r["payload"], on_delivery=cb)
                n += 1
            except BufferError:
                # 本地队列满：保持 pending，poll 排空后下轮重试（不丢）
                prod.poll(0.01)
            except Exception as exc:  # noqa: BLE001  序列化/超长等：计次重试，滞留 outbox
                logger.warning("produce 失败 topic=%s：%s", topic, exc,
                               extra={"component": "kafka"})
                self.store.bump_attempts(int(r["id"]))
        # 驱动回调（投递证实）
        for prod in (self._prod_critical, self._prod_progress):
            if prod is not None:
                prod.poll(0.0)
        return n

    def _make_cb(self, outbox_id: int):
        def _cb(err, msg):  # 逐条 on_delivery：捕获 outbox_id，精确落库
            if err is None:
                self.store.mark_done(outbox_id)
            else:
                logger.warning("投递回执失败 outbox=%s：%s", outbox_id, err,
                               extra={"component": "kafka"})
                self.store.bump_attempts(outbox_id)
        return _cb

    def stats(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {"available": self.available}
        if self._prod_critical is not None:
            try:
                out["rdkafka"] = json.loads(self._prod_critical.stats())
            except Exception:  # noqa: BLE001
                pass
        if self._last_error:
            out["last_error"] = self._last_error
        return out

    # ── 下行命令（供 agent 用 run_in_executor 非阻塞轮询）──
    def poll_cmd(self, timeout_s: float):
        if self._consumer is None:
            return None
        msg = self._consumer.poll(timeout=timeout_s)
        if msg is None:
            return None
        if msg.error():
            logger.warning("cmd 消费错误：%s", msg.error(), extra={"component": "kafka"})
            return None
        return {"topic": msg.topic(), "key": msg.key(), "value": msg.value(),
                "offset": msg.offset(), "partition": msg.partition()}

    def commit_cmd(self, cmd_msg: Dict[str, Any]) -> None:
        """处理成功后手动提交 offset（§6.8.5：执行 + 回执证实后才 commit）。"""
        if self._consumer is None:
            return
        from confluent_kafka import TopicPartition
        tp = TopicPartition(cmd_msg["topic"], cmd_msg["partition"],
                            cmd_msg["offset"] + 1)
        try:
            self._consumer.commit(offsets=[tp], asynchronous=False)
        except Exception as exc:  # noqa: BLE001
            logger.warning("cmd offset 提交失败：%s", exc, extra={"component": "kafka"})

    def shutdown(self) -> None:
        for prod in (self._prod_critical, self._prod_progress):
            if prod is not None:
                try:
                    prod.flush(3.0)
                except Exception:  # noqa: BLE001
                    pass
        if self._consumer is not None:
            try:
                self._consumer.close()
            except Exception:  # noqa: BLE001
                pass
