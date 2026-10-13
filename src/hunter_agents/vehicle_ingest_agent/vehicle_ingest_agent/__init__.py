# -*- coding: utf-8 -*-
"""vehicle_ingest_agent — 车端 rosbag 落地 / 环形缓存 / 上传 Agent。

严格实现《车端 rosbag 解析与上传 Agent 详细设计文档》，并把本轮需求 2
（大 bag 分段录制 → 环形本地缓存 → 自动上传释放 → 磁盘水位淘汰 → 关键事件保护）
落在 recorder.py + spool.py + status_machine.py 三处。

默认运行在**降级模式**（parse.enabled=false）：车端只负责把 rosbag 安全落地、
生成 manifest 元数据、按环形缓存策略上传原始片段并在上传成功后释放；
解析 / 切片 / 结构化交由服务端集中完成。

模块划分（对照设计文档章节）：
  config.py          三层配置加载与校验（§6.10、附录 A）
  state_store.py     SQLite：任务状态机 + 片段登记表 + outbox + 指令去重（§6.8/§6.9）
  events.py          Kafka envelope 与事件构造（§6.8.3、§7.4）
  kafka_bus.py       复用 hunter_kafka 的 producer + outbox flusher + cmd consumer（§6.8）
  recorder.py        分段录制监管（ros2 bag record 500MB/10min 切分，需求 2）
  verifier.py        片段完整性校验（§6.2）
  manifest.py        manifest.json 生成（§6.2、§7.1）
  uploader.py        上传引擎（S3/MinIO/网关，分片/限速/断点/优先级，§6.7）
  spool.py           环形缓存与磁盘治理（§6.6 + 需求 2）
  status_machine.py  任务状态机（§6.9）
  parse_scheduler.py 解析调度（§6.3，默认降级=原样上传）
  cmd_consumer.py    下行指令（§6.8.5）
  heartbeat.py       心跳 / 告警指标（§6.11）
  logging_util.py    JSON lines 结构化日志（§6.11）
  agent.py           asyncio 主编排（§3.2）
"""

__version__ = "1.0.0"
