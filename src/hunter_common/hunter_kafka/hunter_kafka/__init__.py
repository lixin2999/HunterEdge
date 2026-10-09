# -*- coding: utf-8 -*-
"""hunter_kafka — HunterCore 车端 Kafka 接入公共库。

对外稳定接口：
  :func:`hunter_kafka.config.load_properties`  读取接入包 kafka.properties
  :func:`hunter_kafka.config.load_conf`        装配 librdkafka 配置（SASL_SSL + mTLS）
  :func:`hunter_kafka.client.topic_name`       契约 Topic 命名（hunter.<vehicle_id>.<type>）
  :func:`hunter_kafka.client.make_producer`    按 Topic 语义创建 Producer（acks 分档）
  :func:`hunter_kafka.client.make_consumer`    创建手动提交的 Consumer（平台下行指令）
  :mod:`hunter_kafka.diagnose`                 接入自检 CLI（``hunter-kafka-check``）

设计文档：《自动驾驶车辆系统详细设计文档》第 12/13/14 章；
契约来源：HunterCore 接入包（HUNTER-001 bundle）README + kafka.properties。
"""
from .client import (CRITICAL_TYPES, TOPIC_TYPES, TopicNameError, is_kafka_available,
                     make_consumer, make_producer, topic_name)
from .config import (DEFAULT_BUNDLE_DIR, DEFAULT_PROPERTIES_PATH, KafkaConfigError,
                     build_client_conf, load_conf, load_properties, redact,
                     resolve_vehicle_id, strip_meta)

__all__ = [
    "CRITICAL_TYPES", "TOPIC_TYPES", "TopicNameError", "is_kafka_available",
    "make_consumer", "make_producer", "topic_name",
    "DEFAULT_BUNDLE_DIR", "DEFAULT_PROPERTIES_PATH", "KafkaConfigError",
    "build_client_conf", "load_conf", "load_properties", "redact",
    "resolve_vehicle_id", "strip_meta",
]

__version__ = "1.0.0"
