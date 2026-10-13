# -*- coding: utf-8 -*-
"""解析调度（设计 §6.3）。

本轮按需求把**解析 / 切片 / 结构化放到服务端**，因此 Agent 默认运行在降级模式
（``parse.enabled=false``）：manifest_ready 直接进 ``uploading_raw``（上传原始片段 + manifest），
:func:`should_parse` 返回 False，服务端拿到原始 bag 后集中解析。

当现场需要"车端近线解析"（设计 §1.2）时，把 ``parse.enabled`` 置 true 即启用下方路由；
解析器（ImageParser/Pc2SliceParser/TableParser/TfParser）在 ``parsers/`` 内以进程池
（绑核 + nice，§3.2/§9）实现，产物落 ``spool/{bag_id}/`` 并逐类上报 ``bag.artifact.ready.v1``。

本模块只负责"是否解析 + topic→解析器路由 + 并发/节流编排"，不含具体编解码。
"""
from __future__ import annotations

from typing import Any, Dict, List

from .logging_util import get_logger

logger = get_logger("parse")

#: topic 类型 → 解析器（设计 §6.3 路由表；config.parse.topics 可覆盖）
DEFAULT_ROUTING: Dict[str, str] = {
    "sensor_msgs/Image": "image",
    "sensor_msgs/CompressedImage": "image",
    "sensor_msgs/PointCloud2": "pc2",
    "sensor_msgs/Imu": "table",
    "nav_msgs/Odometry": "table",
    "sensor_msgs/NavSatFix": "table",
    "tf2_msgs/TFMessage": "tf",
    "diagnostic_msgs/DiagnosticArray": "table",
}


def should_parse(cfg) -> bool:
    """降级模式判定：false → 走 uploading_raw（原样上传，服务端解析）。"""
    return bool(cfg.parse_enabled)


def route_topics(manifest: Dict[str, Any], cfg) -> List[Dict[str, str]]:
    """把 manifest.topics 按类型路由到解析器名，供进程池分发（启用解析时）。"""
    override = cfg.get("parse.topics", {}) or {}
    routed: List[Dict[str, str]] = []
    for t in manifest.get("topics", []):
        topic, typ = t.get("topic"), t.get("type")
        parser = (override.get(topic, {}) or {}).get("parser") if isinstance(
            override.get(topic), dict) else None
        parser = parser or DEFAULT_ROUTING.get(typ, "raw_keep")
        routed.append({"topic": topic, "type": typ, "parser": parser})
    return routed


class ParseScheduler:
    """解析编排器。降级模式下不建进程池，:meth:`process` 直接返回未解析。"""

    def __init__(self, cfg, store, machine):
        self.cfg = cfg
        self.store = store
        self.machine = machine
        self.enabled = should_parse(cfg)
        self._pool = None
        if self.enabled:
            # 预留：ProcessPoolExecutor(workers) + 绑核/nice（§6.3/§9）。
            # 本轮默认降级，不在此拉起进程池以免无谓占资源。
            logger.info("解析模式已启用（需实现 parsers/ 后生效）",
                        extra={"component": "parse"})
        else:
            logger.info("解析已按需求关闭：manifest_ready → uploading_raw（服务端解析）",
                        extra={"component": "parse"})

    def process(self, bag_id: str, manifest: Dict[str, Any]) -> Dict[str, Any]:
        """降级模式直接返回"不解析"；启用模式路由到进程池（此处留接口）。"""
        if not self.enabled:
            return {"parsed": False, "mode": "degraded_raw_upload", "topics": []}
        routed = route_topics(manifest, self.cfg)
        # TODO(M3): 提交进程池执行 image/pc2/table/tf 解析并产 artifact.ready 事件
        logger.warning("parse.enabled=true 但解析器尚未落地，按降级上传处理 bag_id=%s",
                       bag_id, extra={"bag_id": bag_id, "component": "parse"})
        return {"parsed": False, "mode": "parser_not_implemented", "topics": routed}
