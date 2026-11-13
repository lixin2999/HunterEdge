# -*- coding: utf-8 -*-
"""session 单测：会话绑定、实例隔离、断流看门狗、stop 补零速、限频。

全部用**注入时钟**（now 参数）驱动，不 sleep，保证 CI 稳定且可精确验证
500ms 边界；断流/收尾语义是真 SIL 的安全底线，逐条独立断言。
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from sil_bridge_agent.session import (                        # noqa: E402
    DEFAULT_WATCHDOG_MS,
    SessionState,
    StreamRateLimiter,
)

SIM = "SIM-001"
OTHER = "SIM-002"
SENSOR_TOPIC = "carla.%s.edge_sensor" % SIM
OTHER_TOPIC = "carla.%s.edge_sensor" % OTHER


def _session(phase: str, sim_id: str = SIM, topic: str = "", **meta) -> dict:
    payload = {"phase": phase, "sim_id": sim_id}
    payload.update(meta)
    env = {"stream": "session", "meta": payload, "payload_b64": ""}
    if topic:
        env["_topic"] = topic
    return env


# ---------------------------------------------------------------------------
# 会话绑定
# ---------------------------------------------------------------------------
def test_initial_state_is_unbound() -> None:
    s = SessionState()
    assert s.sim_id == "" and s.active is False
    assert s.sensor_topic == "" and s.control_topic == ""
    # 未 boot 时不得发布任何 ROS 帧、不得回传控制
    assert s.sensor_fresh(now=100.0) is False
    assert s.accepts(SENSOR_TOPIC) is False
    assert s.pop_control(1.0, 0.0, now=100.0) is None


def test_boot_binds_sim_id_and_topics() -> None:
    s = SessionState()
    assert s.handle_session(
        _session("boot", scene_id="SCN-2056", map_id="Town03"), now=100.0
    ) == "boot"
    assert s.active is True and s.sim_id == SIM
    assert s.sensor_topic == SENSOR_TOPIC
    assert s.control_topic == "carla.%s.edge_control" % SIM
    st = s.stats(now=100.0)
    assert st["scene_id"] == "SCN-2056" and st["map_id"] == "Town03"
    assert st["uptime_s"] == 0.0


def test_boot_marks_link_fresh_immediately() -> None:
    """boot 即置新鲜：否则首个 500ms 窗口会被误判断流而停发传感器。"""
    s = SessionState(watchdog_ms=500)
    s.handle_session(_session("boot"), now=100.0)
    assert s.sensor_fresh(now=100.0) is True
    assert s.sensor_fresh(now=100.499) is True
    assert s.sensor_fresh(now=100.501) is False


def test_boot_idempotent_keeps_counters() -> None:
    """仿真端重连会重发 boot：同 sim_id 不得重置计数与序号。"""
    s = SessionState()
    s.handle_session(_session("boot"), now=100.0)
    s.note_sensor("lidar", now=100.1)
    s.note_sensor("imu", now=100.2)
    s.pop_control(1.0, 0.0, now=100.3)
    s.reject()
    before = s.stats(now=100.4)
    assert s.handle_session(_session("boot"), now=100.5) == "boot"
    after = s.stats(now=100.5)
    assert after["sensor_frames"] == before["sensor_frames"]
    assert after["control_frames"] == before["control_frames"]
    assert after["rejected_frames"] == before["rejected_frames"]
    # seq 必须继续递增（重置会让仿真端按 seq<=last 丢弃全部后续控制帧）
    nxt = s.pop_control(1.0, 0.0, now=100.6)
    assert nxt is not None and nxt["seq"] == before["control_frames"] + 1


def test_boot_different_sim_id_switches_session() -> None:
    s = SessionState()
    s.handle_session(_session("boot"), now=100.0)
    s.note_sensor("lidar", now=100.1)
    s.pop_control(1.0, 0.0, now=100.2)
    assert s.handle_session(_session("boot", sim_id=OTHER), now=101.0) == "boot"
    assert s.sim_id == OTHER
    st = s.stats(now=101.0)
    assert st["sensor_frames"] == {} and st["control_frames"] == 0
    assert st["stale_episodes"] == 0
    assert s.accepts(OTHER_TOPIC) is True
    assert s.accepts(SENSOR_TOPIC) is False


def test_boot_sim_id_fallback_from_topic() -> None:
    """meta 缺 sim_id 时从 Topic 反解（正则订阅下 Topic 一定带实例名）。"""
    s = SessionState()
    env = {"stream": "session", "meta": {"phase": "boot"}, "_topic": OTHER_TOPIC}
    assert s.handle_session(env, now=100.0) == "boot"
    assert s.sim_id == OTHER


def test_sim_id_filter_rejects_other_instances() -> None:
    s = SessionState(sim_id_filter=SIM)
    assert s.handle_session(_session("boot", sim_id=OTHER), now=100.0) == ""
    assert s.active is False and s.sim_id == ""
    assert s.stats(now=100.0)["rejected_frames"] == 1
    assert s.handle_session(_session("boot", sim_id=SIM), now=101.0) == "boot"
    assert s.sim_id == SIM


def test_invalid_session_frames_are_ignored() -> None:
    s = SessionState()
    assert s.handle_session({"stream": "lidar", "meta": {"phase": "boot"}}, now=1.0) == ""
    assert s.handle_session(_session("pause"), now=1.0) == ""
    assert s.handle_session({"stream": "session", "meta": {}}, now=1.0) == ""
    assert s.active is False


# ---------------------------------------------------------------------------
# stop 与零速收尾
# ---------------------------------------------------------------------------
def test_stop_without_boot_is_noise() -> None:
    s = SessionState()
    assert s.handle_session(_session("stop"), now=100.0) == ""
    assert s.control_pending() is False
    assert s.pop_control(1.0, 0.0, now=100.0) is None


def test_stop_deactivates_and_queues_exactly_one_zero() -> None:
    s = SessionState()
    s.handle_session(_session("boot"), now=100.0)
    s.pop_control(2.0, 0.5, now=100.1)
    assert s.handle_session(_session("stop"), now=101.0) == "stop"
    assert s.active is False
    assert s.control_pending() is True
    # 停发传感器帧
    assert s.sensor_fresh(now=101.0) is False
    assert s.accepts(SENSOR_TOPIC) is False
    # 补**恰好一帧**零速：入参的非零指令必须被强制归零
    zero = s.pop_control(2.0, 0.5, now=101.0)
    assert zero is not None
    assert zero["linear_x"] == 0.0 and zero["angular_z"] == 0.0
    assert s.control_pending() is False
    assert s.pop_control(2.0, 0.5, now=101.1) is None
    assert s.pop_control(0.0, 0.0, now=101.2) is None


def test_stop_zero_seq_continues_monotonic() -> None:
    """收尾零速帧的 seq 必须大于此前所有帧，否则被仿真端去重丢弃 → 自车滑行。"""
    s = SessionState()
    s.handle_session(_session("boot"), now=100.0)
    seqs = [s.pop_control(1.0, 0.0, now=100.0 + i)["seq"] for i in range(3)]
    s.handle_session(_session("stop"), now=101.0)
    zero = s.pop_control(1.0, 0.0, now=101.0)
    assert zero is not None
    assert zero["seq"] == max(seqs) + 1
    assert zero["wall_ts_ns"] == int(101.0 * 1e9)


def test_stop_of_other_instance_rejected() -> None:
    s = SessionState()
    s.handle_session(_session("boot"), now=100.0)
    assert s.handle_session(_session("stop", sim_id=OTHER), now=101.0) == ""
    assert s.active is True                      # 别的实例 stop 不得终止本会话
    assert s.control_pending() is False
    assert s.stats(now=101.0)["rejected_frames"] == 1


def test_reboot_clears_pending_zero() -> None:
    s = SessionState()
    s.handle_session(_session("boot"), now=100.0)
    s.handle_session(_session("stop"), now=101.0)
    assert s.control_pending() is True
    s.handle_session(_session("boot"), now=102.0)
    assert s.control_pending() is False
    frame = s.pop_control(1.5, 0.2, now=102.1)
    assert frame is not None and frame["linear_x"] == 1.5


def test_close_ends_active_session_and_queues_zero() -> None:
    """进程退出（没收到 stop 帧）也必须补零速，防自车沿用末帧滑行。"""
    s = SessionState()
    s.handle_session(_session("boot"), now=100.0)
    s.pop_control(2.0, 0.5, now=100.1)
    assert s.close(now=101.0) is True
    assert s.active is False and s.control_pending() is True
    assert s.sensor_fresh(now=101.0) is False
    zero = s.pop_control(2.0, 0.5, now=101.0)
    assert zero is not None
    assert zero["linear_x"] == 0.0 and zero["angular_z"] == 0.0
    assert zero["seq"] == 2                      # 序号接在末帧之后，不被仿真端去重
    assert s.pop_control(2.0, 0.5, now=101.1) is None


def test_close_is_idempotent_and_noop_when_unbound() -> None:
    s = SessionState()
    assert s.close(now=100.0) is False
    assert s.pop_control(1.0, 0.0, now=100.0) is None   # 未绑定不得凭空产帧
    s.handle_session(_session("boot"), now=100.0)
    assert s.close(now=101.0) is True
    assert s.close(now=102.0) is False                  # 幂等：不再欠帧
    assert s.pop_control(0.0, 0.0, now=102.0) is not None
    assert s.control_pending() is False
    assert s.pop_control(0.0, 0.0, now=102.1) is None


def test_close_after_stop_does_not_double_zero() -> None:
    """stop 帧已收尾后再退出，不得补第二帧零速。"""
    s = SessionState()
    s.handle_session(_session("boot"), now=100.0)
    s.handle_session(_session("stop"), now=101.0)
    assert s.close(now=102.0) is False
    zero = s.pop_control(1.0, 0.0, now=102.0)
    assert zero is not None and zero["linear_x"] == 0.0
    assert s.pop_control(1.0, 0.0, now=102.1) is None   # 恰好一帧


# ---------------------------------------------------------------------------
# 实例隔离
# ---------------------------------------------------------------------------
def test_accepts_isolates_instances() -> None:
    s = SessionState()
    s.handle_session(_session("boot"), now=100.0)
    assert s.accepts(SENSOR_TOPIC) is True
    assert s.accepts(OTHER_TOPIC) is False
    assert s.accepts("hunter.HUNTER-001.telemetry") is False
    assert s.accepts("") is False
    s.reject()
    s.reject()
    assert s.stats(now=100.0)["rejected_frames"] == 2


def test_note_sensor_counts_per_stream() -> None:
    s = SessionState()
    s.handle_session(_session("boot"), now=100.0)
    for _ in range(3):
        s.note_sensor("lidar", now=100.1)
    s.note_sensor("imu", now=100.2)
    st = s.stats(now=100.2)
    assert st["sensor_frames"] == {"lidar": 3, "imu": 1}
    assert st["sensor_age_ms"] == 0.0


# ---------------------------------------------------------------------------
# 看门狗
# ---------------------------------------------------------------------------
def test_watchdog_boundary() -> None:
    s = SessionState(watchdog_ms=DEFAULT_WATCHDOG_MS)
    s.handle_session(_session("boot"), now=100.0)
    s.note_sensor("rgb", now=100.0)
    assert s.sensor_fresh(now=100.5) is True      # 恰好 500ms 仍新鲜
    assert s.sensor_fresh(now=100.5001) is False
    # 恢复后必须重新新鲜
    s.note_sensor("rgb", now=101.0)
    assert s.sensor_fresh(now=101.2) is True


def test_watchdog_disabled_only_for_debug() -> None:
    s = SessionState(watchdog_ms=0)
    s.handle_session(_session("boot"), now=100.0)
    s.note_sensor("rgb", now=100.0)
    assert s.sensor_fresh(now=10000.0) is True
    # 但仍要求会话活跃（stop 后不得继续发布）
    s.handle_session(_session("stop"), now=10001.0)
    assert s.sensor_fresh(now=10001.0) is False


def test_watchdog_negative_treated_as_disabled() -> None:
    s = SessionState(watchdog_ms=-100)
    s.handle_session(_session("boot"), now=100.0)
    assert s.sensor_fresh(now=9999.0) is True
    assert s.stats(now=100.0)["watchdog_ms"] == 0


def test_poll_stale_reports_once_per_episode() -> None:
    s = SessionState(watchdog_ms=500, stale_log_interval_s=5.0)
    s.handle_session(_session("boot"), now=100.0)
    s.note_sensor("lidar", now=100.0)
    assert s.poll_stale(now=100.2) is False
    assert s.poll_stale(now=100.6) is True        # 首次断流告警
    assert s.poll_stale(now=100.7) is False       # 限频，不刷屏
    assert s.poll_stale(now=101.0) is False
    st = s.stats(now=101.0)
    assert st["stale"] is True and st["stale_episodes"] == 1
    # 超过限频间隔再告警一次，但仍是同一次断流事件
    assert s.poll_stale(now=105.7) is True
    assert s.stats(now=105.7)["stale_episodes"] == 1


def test_poll_stale_recovery_starts_new_episode() -> None:
    s = SessionState(watchdog_ms=500)
    s.handle_session(_session("boot"), now=100.0)
    assert s.poll_stale(now=100.6) is True
    s.note_sensor("lidar", now=101.0)
    assert s.poll_stale(now=101.1) is False
    assert s.stats(now=101.1)["stale"] is False
    assert s.poll_stale(now=101.7) is True        # 第二次断流
    assert s.stats(now=101.7)["stale_episodes"] == 2


def test_poll_stale_silent_when_session_inactive() -> None:
    """未 boot / 已 stop 不算断流事件（否则每次收车都误报 communication_loss）。"""
    s = SessionState()
    assert s.poll_stale(now=100.0) is False
    s.handle_session(_session("boot"), now=100.0)
    s.handle_session(_session("stop"), now=101.0)
    assert s.poll_stale(now=102.0) is False
    assert s.stats(now=102.0)["stale_episodes"] == 0


def test_control_keeps_flowing_while_sensor_stale() -> None:
    """传感器断流时控制回传不停。

    此时 safety_guard 已输出零速，继续转发才能让仿真端拿到新鲜控制帧，
    两侧看门狗语义不互相掩盖。
    """
    s = SessionState(watchdog_ms=500)
    s.handle_session(_session("boot"), now=100.0)
    s.note_sensor("lidar", now=100.0)
    assert s.sensor_fresh(now=101.0) is False
    frame = s.pop_control(0.0, 0.0, now=101.0)
    assert frame is not None and frame["seq"] == 1


# ---------------------------------------------------------------------------
# 诊断快照
# ---------------------------------------------------------------------------
def test_stats_shape_and_no_credentials() -> None:
    s = SessionState(watchdog_ms=500)
    st = s.stats(now=100.0)
    expected = {
        "active", "sim_id", "scene_id", "map_id", "sensor_topic", "control_topic",
        "watchdog_ms", "sensor_age_ms", "sensor_fresh", "stale", "stale_episodes",
        "sensor_frames", "control_frames", "rejected_frames", "stop_zero_pending",
        "uptime_s",
    }
    assert set(st) == expected
    assert st["sensor_age_ms"] is None and st["uptime_s"] is None
    s.handle_session(_session("boot"), now=100.0)
    s.note_sensor("lidar", now=100.25)
    st2 = s.stats(now=100.5)
    assert st2["sensor_age_ms"] == 250.0
    assert st2["uptime_s"] == 0.5
    assert st2["watchdog_ms"] == 500
    # 快照不得含任何凭据字段
    assert not any("password" in k or "sasl" in k or "ssl" in k for k in st2)


def test_stats_sensor_frames_is_copy() -> None:
    """返回的计数 dict 必须是副本，外部改动不得污染内部状态。"""
    s = SessionState()
    s.handle_session(_session("boot"), now=100.0)
    s.note_sensor("lidar", now=100.1)
    snap = s.stats(now=100.1)["sensor_frames"]
    snap["lidar"] = 999
    assert s.stats(now=100.1)["sensor_frames"]["lidar"] == 1


# ---------------------------------------------------------------------------
# 限频
# ---------------------------------------------------------------------------
def test_rate_limiter_first_frame_always_allowed() -> None:
    lim = StreamRateLimiter({"lidar": 10.0})
    assert lim.allow("lidar", now=100.0) is True
    assert lim.dropped == {}


def test_rate_limiter_enforces_interval() -> None:
    lim = StreamRateLimiter({"lidar": 10.0})       # 100ms 间隔
    assert lim.allow("lidar", now=100.0) is True
    assert lim.allow("lidar", now=100.05) is False
    assert lim.allow("lidar", now=100.099) is False
    assert lim.allow("lidar", now=100.1) is True
    assert lim.dropped == {"lidar": 2}


def test_rate_limiter_nominal_cadence_at_epoch_scale() -> None:
    """回归：生产用 time.time()（≈1.7e9），恰好按标称周期到达的帧不得被误丢。

    float64 在 epoch 量级的表示误差可达 ~4e-7s；若限频判定不留量级容差，
    上行 10Hz 配限频 10Hz 会退化成约 5Hz（感知输入莫名减半且无报错）。
    """
    lim = StreamRateLimiter({"lidar": 10.0, "imu": 50.0})
    epoch = 1760313600.0
    for i in range(20):
        assert lim.allow("lidar", now=epoch + i * 0.1) is True, "lidar 第 %d 帧被误丢" % i
        assert lim.allow("imu", now=epoch + i * 0.02) is True, "imu 第 %d 帧被误丢" % i
    assert lim.dropped == {}
    # 真正早到的帧仍须被限频（容差不得大到放行半周期内的帧）
    # 末帧是 i=19 → epoch+1.9；再隔 0.05s（半个周期）必须被丢
    assert lim.allow("lidar", now=epoch + 19 * 0.1 + 0.05) is False
    assert lim.dropped == {"lidar": 1}


def test_rate_limiter_streams_are_independent() -> None:
    lim = StreamRateLimiter({"lidar": 10.0, "imu": 50.0})
    assert lim.allow("lidar", now=100.0) is True
    assert lim.allow("imu", now=100.0) is True
    assert lim.allow("imu", now=100.02) is True     # 50Hz → 20ms
    assert lim.allow("imu", now=100.03) is False
    assert lim.allow("lidar", now=100.03) is False


def test_rate_limiter_zero_or_unknown_means_unlimited() -> None:
    lim = StreamRateLimiter({"rgb": 0.0, "depth": -5.0})
    for i in range(5):
        assert lim.allow("rgb", now=100.0 + i * 0.001) is True
        assert lim.allow("depth", now=100.0 + i * 0.001) is True
        assert lim.allow("ego_state", now=100.0) is True   # 未配置流不限频
    assert lim.dropped == {}


def test_rate_limiter_dropped_is_copy() -> None:
    lim = StreamRateLimiter({"lidar": 10.0})
    lim.allow("lidar", now=100.0)
    lim.allow("lidar", now=100.01)
    snap = lim.dropped
    snap["lidar"] = 999
    assert lim.dropped == {"lidar": 1}


def main() -> int:
    tests = [(n, f) for n, f in sorted(globals().items())
             if n.startswith("test_") and callable(f)]
    for name, fn in tests:
        fn()
        print("PASS %s" % name)
    print("SESSION_OK: %d tests" % len(tests))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
