#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Kafka 客户端工厂与 Topic 命名（严格对齐 contracts/kafka/topics.yaml）。

Topic 命名规则（车端 8 个/车）：``hunter.<vehicle_id>.<type>``
  车→云：telemetry / event / health / command_result / ota_status
  云→车：command / remote_control / ota_notify

acks 策略（接入包 kafka.properties 明确要求）：
  telemetry、health  → ``acks=1``（高频、允许极小概率丢一帧）
  event、command_result、ota_notify/ota_status → ``acks=all``（业务链不能丢）
故同一进程内按需建两个 producer，而不是全局共用一个。
"""
from __future__ import annotations

from typing import Callable, Dict, Optional

from .config import load_conf, strip_meta

TOPIC_TYPES = (
    "telemetry", "event", "health", "command", "command_result",
    "ota_notify", "ota_status", "remote_control",
)

#: 需要 acks=all 的 Topic（事件/指令回执/OTA 状态属业务链，不可丢）
CRITICAL_TYPES = ("event", "command_result", "ota_status")


class TopicNameError(ValueError):
    """非法 Topic 类型（防止把 remote_status 这类契约外 Topic 发出去）"""


def topic_name(vehicle_id: str, kind: str) -> str:
    """返回 ``hunter.<vehicle_id>.<kind>``；kind 必须在契约清单内。"""
    if kind not in TOPIC_TYPES:
        raise TopicNameError(
            f"Topic 类型 {kind!r} 不在契约清单内（可用：{', '.join(TOPIC_TYPES)}）")
    return f"hunter.{vehicle_id}.{kind}"


class _ProducerWithDeliveryReport:
    """给 `Producer` 补上「逐条投递证实回调」（`on_delivery`）。

    ⚠ 为什么不用 librdkafka/Python 绑定的全局 `dr_cb`（HUNTER-001 实机 + 本地同版本 2.16.0 双证）：
      - `Producer(conf, dr_cb=cb)` → `KafkaException: _INVALID_ARG "Property "dr_cb" must be
        set through dedicated .._set_..() function"`（该版本**不把 dr_cb 当回调 kwarg**，
        而是并进 conf 交给 `conf_set`，于是被 librdkafka 拒绝）；
      - `Producer({**conf, "dr_cb": cb})` → 同样 `_INVALID_ARG`；
      - `p.dr_cb = cb` → `AttributeError`（该版本没有这个属性，只有 error_cb/stats_cb/
        throttle_cb/logger 几个 kwarg 回调）。
      该版本**唯一可用**的投递证实入口是 `produce(..., on_delivery=cb)`（官方文档明确：
      回调在 `poll()`/`flush()` 时触发）——本包装类即在 `produce()` 上自动挂上它，
      对调用方保持「构造时传 dr_callback，之后每条消息都会回调」的原语义。
    """

    def __init__(self, producer, dr_callback):
        self._producer = producer
        self._dr_callback = dr_callback

    def produce(self, topic, value=None, key=None, *args, **kwargs):
        # 调用方若已显式传 on_delivery（如 command_agent），尊重其选择
        kwargs.setdefault("on_delivery", self._dr_callback)
        return self._producer.produce(topic, value, key, *args, **kwargs)

    def __getattr__(self, name):        # 其余方法/属性透传（poll/flush/produce 计数等）
        return getattr(self._producer, name)


def make_producer(
    vehicle_id: str,
    kind: str,
    *,
    properties_path: Optional[str] = None,
    bundle_dir: Optional[str] = None,
    dr_callback: Optional[Callable] = None,
    extra: Optional[Dict[str, str]] = None,
    fallback: Optional[Dict[str, str]] = None,
):
    """按 Topic 语义创建 Producer（自动选 acks、client.id、linger/batch/compression）。

    ``dr_callback``：投递证实回调（err, msg）——见 :class:`_ProducerWithDeliveryReport`
    的说明（本版本必须走 `on_delivery`，不能用全局 `dr_cb`）。
    ``fallback``：接入包尚未部署时的内联配置（见 :func:`hunter_kafka.config.load_conf`）。
    """
    from confluent_kafka import Producer  # 延迟导入：降级路径不依赖该包

    acks = "all" if kind in CRITICAL_TYPES else "1"
    conf = load_conf(
        properties_path=properties_path, bundle_dir=bundle_dir,
        client_id=f"{vehicle_id}-{kind}", role="producer", acks=acks,
        extra=extra, fallback=fallback)
    plain = strip_meta(conf)
    # 防御：历史上曾把回调塞进 conf（非法键）→ 这里确保不会带进去
    plain.pop("dr_cb", None)
    plain.pop("dr_msg_cb", None)
    producer = Producer(plain)
    if dr_callback is not None:
        return _ProducerWithDeliveryReport(producer, dr_callback)
    return producer


def make_consumer(
    vehicle_id: str,
    kinds,
    *,
    group_suffix: str,
    properties_path: Optional[str] = None,
    bundle_dir: Optional[str] = None,
    auto_offset_reset: str = "earliest",
    extra: Optional[Dict[str, str]] = None,
    fallback: Optional[Dict[str, str]] = None,
):
    """创建订阅 ``hunter.<vehicle_id>.<kind>`` 的 Consumer。

    车端消费平台指令必须**手动提交**（``enable.auto.commit=false``）：
    指令处理+回执成功后才 commit，否则进程在处理中途被 kill 会丢指令。
    """
    from confluent_kafka import Consumer

    if isinstance(kinds, str):
        kinds = [kinds]
    topics = [topic_name(vehicle_id, k) for k in kinds]

    conf = load_conf(
        properties_path=properties_path, bundle_dir=bundle_dir,
        client_id=f"{vehicle_id}-{group_suffix}", role="consumer",
        extra={
            "group.id": f"{group_suffix}-{vehicle_id}",
            "enable.auto.commit": "false",
            "auto.offset.reset": auto_offset_reset,
            "isolation.level": "read_committed",
            "session.timeout.ms": "15000",
            **(extra or {}),
        }, fallback=fallback)
    # 契约要求手动提交，禁止被 properties 的基准值反过来覆盖成自动提交
    conf["enable.auto.commit"] = "false"
    consumer = Consumer(strip_meta(conf))
    consumer.subscribe(topics)
    return consumer


def is_kafka_available() -> bool:
    """confluent_kafka 是否可用（不可用时各 Agent 降级为仅本地日志，不阻塞业务）。"""
    try:
        import confluent_kafka  # noqa: F401
        return True
    except ImportError:
        return False
