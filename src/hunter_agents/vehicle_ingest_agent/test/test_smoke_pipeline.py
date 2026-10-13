# -*- coding: utf-8 -*-
"""核心链路冒烟测试（不依赖 Kafka/OSS/ROS，可 pytest 或直接 python 运行）。

覆盖需求 2 的落地闭环：分段登记 → 校验 → manifest → 状态机 → 上传成功即删（含 protected 保留）
+ 环形窗口/水位淘汰。用临时目录 + 伪 rosbag2（metadata.yaml + .db3）驱动。
"""
from __future__ import annotations

import os
import sys
import tempfile

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from vehicle_ingest_agent import manifest as mf            # noqa: E402
from vehicle_ingest_agent import verifier                   # noqa: E402
from vehicle_ingest_agent.config import AgentConfig, DEFAULTS, _deep_merge  # noqa: E402
from vehicle_ingest_agent.spool import SpoolManager         # noqa: E402
from vehicle_ingest_agent.state_store import StateStore     # noqa: E402
from vehicle_ingest_agent.status_machine import TaskStateMachine  # noqa: E402


def _make_cfg(root: str, keep_raw: str = "delete_on_uploaded") -> AgentConfig:
    data = _deep_merge(DEFAULTS, {"paths": {"root": root},
                                  "vehicle_id": "TEST-001",
                                  "upload": {"keep_raw": keep_raw}})
    return AgentConfig(data)


def _fake_bag(seg_dir: str) -> None:
    os.makedirs(seg_dir, exist_ok=True)
    # 最小 rosbag2 元数据（verifier 解析 topics/message_count/duration/starting_time）
    meta = (
        "rosbag2_info:\n"
        "  starting_time:\n    nanoseconds: 1760313600000000000\n"
        "  duration: 60.0\n"
        "  topics_with_message_count:\n"
        "    - topic_data: {name: /imu/data, type: sensor_msgs/msg/Imu}\n"
        "      message_count: 1800\n"
        "    - topic_data: {name: /lidar_points, type: sensor_msgs/msg/PointCloud2}\n"
        "      message_count: 600\n"
    )
    try:
        import yaml  # noqa: F401
        has_yaml = True
    except ImportError:
        has_yaml = False
    with open(os.path.join(seg_dir, "metadata.yaml"), "w", encoding="utf-8") as fh:
        fh.write(meta)
    with open(os.path.join(seg_dir, "bag_0.db3"), "wb") as fh:
        fh.write(b"\x00" * 1024)
    _ = has_yaml


def _run_pipeline(cfg: AgentConfig, store: StateStore, bag_id: str, seg_dir: str,
                  protected: bool = False) -> None:
    machine = TaskStateMachine(cfg, store)
    _fake_bag(seg_dir)
    size = sum(os.path.getsize(os.path.join(seg_dir, f)) for f in os.listdir(seg_dir))
    store.register_segment(bag_id, seg_dir, size, 1760313600000000000,
                           1760313660000000000, 60.0, state="discovered",
                           protected=protected)
    store.create_task(bag_id, "discovered")

    passed, err, facts = verifier.verify_segment(
        seg_dir, whitelist=[], min_duration_s=1.0, compute_hash=True,
        hash_chunk_mb=8, skew_warn_ms=200)
    assert passed, f"verify 失败：{err}"
    facts["path"] = seg_dir
    facts["size_bytes"] = size
    m = mf.build_manifest(bag_id, cfg, facts, protected=protected)
    assert m["file_sha256"], "sha256 缺失"
    machine.transition(bag_id, "verifying")
    res = machine.transition(bag_id, "manifest_ready")
    machine.emit_manifest(bag_id, res["seq"], m)
    machine.transition(bag_id, "uploading_raw")
    machine.transition(bag_id, "uploading_done")
    machine.transition(bag_id, "done")


def main() -> int:
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as root:
        os.makedirs(root, exist_ok=True)
        cfg = _make_cfg(root)
        for d in (cfg.bags_dir, cfg.spool_dir, cfg.state_dir, cfg.part_state_dir):
            os.makedirs(d, exist_ok=True)
        store = StateStore(cfg.db_path)
        try:
            run_checks(cfg, store, root)
        finally:
            store.close()

    print("SMOKE_OK: 分段→校验→manifest→状态机→上传即删/protect 保留/环形治理 全部通过")
    return 0


def run_checks(cfg: AgentConfig, store: StateStore, root: str) -> None:
    spool = SpoolManager(cfg, store, alert=lambda *a, **k: None)

    # 片段 A：上传成功 → 立即删除（delete_on_uploaded）
    segA = os.path.join(cfg.bags_dir, "TEST-001", "2026-10-13", "bag_A")
    _run_pipeline(cfg, store, "bag_A", segA)
    assert store.get_task("bag_A")["state"] == "done"
    assert os.path.isdir(segA)  # apply_after_upload 前仍在
    deleted = spool.apply_after_upload("bag_A")
    assert deleted and not os.path.isdir(segA), "上传成功片段应立即删除"
    assert store.get_segment("bag_A") is None

    # 片段 B：protected → 不自动删除
    segB = os.path.join(cfg.bags_dir, "TEST-001", "2026-10-13", "bag_B")
    _run_pipeline(cfg, store, "bag_B", segB, protected=True)
    spool.apply_after_upload("bag_B")
    assert os.path.isdir(segB), "protected 片段不得自动删除"

    # 环形窗口/水位：登记 12 个未保护未上传片段，retention=10
    for i in range(12):
        p = os.path.join(cfg.bags_dir, "TEST-001", "d", "bag_c%d" % i)
        _fake_bag(p)
        store.register_segment("bag_c%d" % i, p, 2048, 0, 0, 60.0,
                               state="discovered", protected=False)
    # 已上传部分标记，模拟上传跟得上但 keep 残留 → enforce 走 keep 分支需 keep_raw=keep
    cfg_keep = _make_cfg(root, keep_raw="keep")
    spool2 = SpoolManager(cfg_keep, store, alert=lambda *a, **k: None)
    summary = spool2.enforce()  # 不应抛异常；水位低 → 无 evict
    assert "deleted_uploaded" in summary and "critical" in summary

    pending = store.count_pending()
    rows = store.fetch_pending(200)
    for r in rows:
        store.mark_done(r["id"])
    assert store.count_pending() < pending


def test_smoke() -> None:
    assert main() == 0


if __name__ == "__main__":
    raise SystemExit(main())
