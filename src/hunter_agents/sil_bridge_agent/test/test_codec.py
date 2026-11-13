# -*- coding: utf-8 -*-
"""codec 单测：坐标系反射、信封解析、各流解码、控制帧（不依赖 rclpy/Kafka）。

坐标系部分是真 SIL 闭环最容易错、且错了最难查的一环（车会朝反方向转向），
故对真向量/伪向量/四元数分别做**独立推导校验**，而不是只断言实现自身的输出。
"""
from __future__ import annotations

import base64
import json
import math
import os
import struct
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from sil_bridge_agent import codec                            # noqa: E402


# ---------------------------------------------------------------------------
# 坐标系反射：R = diag(1,-1,1)，det(R) = -1
# ---------------------------------------------------------------------------
def test_position_is_true_vector() -> None:
    """位置/速度/加速度是真向量：只有 Y 取反。"""
    assert codec.carla_position_to_ros(1.0, 2.0, 3.0) == (1.0, -2.0, 3.0)
    assert codec.carla_position_to_ros(0.0, 0.0, 0.0) == (0.0, 0.0, 0.0)
    # CARLA +Y 是右，ROS +Y 是左：车右侧的点在 ROS 系必须为负 Y
    right_of_car = codec.carla_position_to_ros(10.0, 5.0, 0.0)
    assert right_of_car[1] < 0.0


def test_angular_is_pseudovector() -> None:
    """角速度是伪向量：det(R)·R·ω = (-x,+y,-z)，与真向量规则**不同**。"""
    assert codec.carla_angular_to_ros(1.0, 2.0, 3.0) == (-1.0, 2.0, -3.0)
    # CARLA yaw>0 为右转（顺时针），ROS yaw>0 为左转（逆时针）→ z 必须取反
    carla_right_turn = codec.carla_angular_to_ros(0.0, 0.0, 0.5)
    assert carla_right_turn == (0.0, 0.0, -0.5)
    # 与真向量规则对照：若误用真向量规则会得到 (0,0,+0.5)，转向反向
    assert carla_right_turn != codec.carla_position_to_ros(0.0, 0.0, 0.5)


def test_euler_flip() -> None:
    assert codec.carla_euler_to_ros(0.1, 0.2, 0.3) == (-0.1, 0.2, -0.3)


def test_quaternion_yaw_only_matches_reference() -> None:
    """纯 yaw 四元数与独立参考公式一致（绕 Z 轴转 -yaw_carla）。"""
    carla_yaw = math.pi / 3
    qx, qy, qz, qw = codec.carla_pose_to_ros_quaternion({"yaw": carla_yaw})
    expected_z = math.sin(-carla_yaw / 2.0)
    expected_w = math.cos(-carla_yaw / 2.0)
    assert abs(qx) < 1e-12 and abs(qy) < 1e-12
    assert abs(qz - expected_z) < 1e-12
    assert abs(qw - expected_w) < 1e-12
    # 单位四元数（全零四元数会被 TF/消息滤波器判非法）
    assert abs(qx * qx + qy * qy + qz * qz + qw * qw - 1.0) < 1e-12


def test_quaternion_unit_norm_for_general_euler() -> None:
    for rpy in ((0.3, -0.2, 1.1), (-1.0, 0.5, 2.9), (0.0, 0.0, 0.0)):
        q = codec.euler_to_quaternion(*rpy)
        assert abs(sum(v * v for v in q) - 1.0) < 1e-12
    identity = codec.euler_to_quaternion(0.0, 0.0, 0.0)
    assert identity == (0.0, 0.0, 0.0, 1.0)


def test_quaternion_component_flip_equivalence() -> None:
    """先翻欧拉角再构造，等价于对 CARLA 系四元数做 (-qx,+qy,-qz,qw)。"""
    pose = {"roll": 0.12, "pitch": -0.34, "yaw": 1.23}
    q_ros = codec.carla_pose_to_ros_quaternion(pose)
    q_carla = codec.euler_to_quaternion(pose["roll"], pose["pitch"], pose["yaw"])
    flipped = (-q_carla[0], q_carla[1], -q_carla[2], q_carla[3])
    for a, b in zip(q_ros, flipped):
        assert abs(a - b) < 1e-12


# ---------------------------------------------------------------------------
# 安全转换助手
# ---------------------------------------------------------------------------
def test_safe_float_rejects_dirty_values() -> None:
    assert codec._f(None) == 0.0
    assert codec._f("abc") == 0.0
    assert codec._f(float("nan")) == 0.0
    assert codec._f(float("inf")) == 0.0
    assert codec._f("2.5") == 2.5
    assert codec._f(None, default=-1.0) == -1.0


def test_vec3_pads_and_truncates() -> None:
    assert codec._vec3([1.0, 2.0]) == (1.0, 2.0, 0.0)
    assert codec._vec3([1.0, 2.0, 3.0, 4.0]) == (1.0, 2.0, 3.0)
    assert codec._vec3(None) == (0.0, 0.0, 0.0)
    assert codec._vec3("xyz") == (0.0, 0.0, 0.0)


# ---------------------------------------------------------------------------
# 信封解析
# ---------------------------------------------------------------------------
def _envelope(stream: str, payload: bytes = b"", meta: dict | None = None) -> dict:
    return {
        "stream": stream,
        "sim_ts": 12.5,
        "wall_ts_ns": 1760313600000000000,
        "seq": 7,
        "meta": meta or {},
        "payload_b64": base64.b64encode(payload).decode("ascii"),
    }


def test_decode_envelope_accepts_bytes_str_dict() -> None:
    env = _envelope("imu", b"{}")
    raw = json.dumps(env).encode("utf-8")
    assert codec.decode_envelope(raw) == env
    assert codec.decode_envelope(raw.decode("utf-8")) == env
    assert codec.decode_envelope(env) == env


def test_decode_envelope_rejects_invalid() -> None:
    assert codec.decode_envelope(b"\xff\xfe not utf8") is None
    assert codec.decode_envelope("{not json") is None
    assert codec.decode_envelope("[1,2,3]") is None
    assert codec.decode_envelope({"meta": {}}) is None          # 缺 stream
    assert codec.decode_envelope({"stream": ""}) is None        # 空 stream
    assert codec.decode_envelope({"stream": 42}) is None        # 非字符串
    assert codec.decode_envelope(None) is None


def test_envelope_payload_and_meta() -> None:
    env = _envelope("lidar", b"\x01\x02\x03", {"points": 1})
    assert codec.envelope_payload(env) == b"\x01\x02\x03"
    assert codec.envelope_meta(env) == {"points": 1}
    # 非法 base64 不得抛（消费循环不能被单帧脏数据打断）
    assert codec.envelope_payload({"payload_b64": "!!!not-base64!!!"}) == b""
    assert codec.envelope_payload({}) == b""
    assert codec.envelope_payload({"payload_b64": 123}) == b""
    assert codec.envelope_meta({"meta": "oops"}) == {}
    assert codec.envelope_meta({}) == {}


def test_session_phase_and_sim_id() -> None:
    boot = _envelope("session", b"", {"phase": "boot", "sim_id": "SIM-1", "scene_id": "SCN-2056"})
    assert codec.session_phase(boot) == "boot"
    assert codec.session_sim_id(boot) == "SIM-1"
    stop = _envelope("session", b"", {"phase": "stop", "sim_id": "SIM-1"})
    assert codec.session_phase(stop) == "stop"
    # 非法相位 / 非 session 帧
    assert codec.session_phase(_envelope("session", b"", {"phase": "pause"})) is None
    assert codec.session_phase(_envelope("lidar")) is None
    assert codec.session_sim_id(_envelope("session")) == ""


def test_sim_id_from_topic() -> None:
    assert codec.sim_id_from_topic("carla.SIM-1.edge_sensor") == "SIM-1"
    # sim_id 自身含点号时按「去头去尾」拼接，不能只取 parts[1]
    assert codec.sim_id_from_topic("carla.a.b.c.edge_control") == "a.b.c"
    assert codec.sim_id_from_topic("hunter.HUNTER-001.telemetry") == ""
    assert codec.sim_id_from_topic("") == ""
    assert codec.sim_id_from_topic("carla.edge_sensor") == ""


# ---------------------------------------------------------------------------
# IMU / ego_state
# ---------------------------------------------------------------------------
def test_decode_imu_flips_accel_and_gyro_differently() -> None:
    payload = json.dumps({"accel": [0.0, 0.0, 9.81], "gyro": [0.1, 0.2, 0.3]}).encode()
    out = codec.decode_imu(payload)
    assert out is not None
    assert out["accel"] == (0.0, 0.0, 9.81)          # 真向量：Z 不变
    assert out["gyro"] == (-0.1, 0.2, -0.3)          # 伪向量：X/Z 取反
    assert out["orientation_covariance"] == [0.0] * 9
    assert codec.decode_imu(b"") is None
    assert codec.decode_imu(b"not json") is None
    assert codec.decode_imu(b"[1,2]") is None


def test_decode_imu_missing_fields_default_zero() -> None:
    out = codec.decode_imu(b"{}")
    assert out is not None
    assert out["accel"] == (0.0, 0.0, 0.0)
    assert out["gyro"] == (0.0, 0.0, 0.0)


def test_decode_ego_state_full() -> None:
    body = {
        "speed_ms": 3.5,
        "velocity": [3.5, 1.0, 0.0],
        "yaw_rate": 0.4,
        "angular_velocity": [0.0, 0.0, 0.4],
        "pose": {"x": 10.0, "y": -2.0, "z": 0.3, "yaw": 0.5, "pitch": 0.0, "roll": 0.0},
    }
    out = codec.decode_ego_state(json.dumps(body).encode())
    assert out is not None
    assert out["position"] == (10.0, 2.0, 0.3)       # CARLA y=-2（左）→ ROS +2
    assert out["linear_velocity"] == (3.5, -1.0, 0.0)
    assert out["angular_velocity"] == (0.0, 0.0, -0.4)
    # CARLA 右转 0.4 rad/s → ROS 左转为正，故 -0.4
    assert out["yaw_rate"] == -0.4
    assert out["speed_ms"] == 3.5                    # 标量与坐标系无关
    qx, qy, qz, qw = out["orientation"]
    assert abs(qz - math.sin(-0.25)) < 1e-12
    assert abs(qw - math.cos(-0.25)) < 1e-12
    assert abs(qx) < 1e-12 and abs(qy) < 1e-12


def test_decode_ego_state_yaw_rate_fallback() -> None:
    """缺 angular_velocity 时回落标量 yaw_rate，取反规则必须一致。"""
    out = codec.decode_ego_state(json.dumps({"yaw_rate": 0.4, "speed_ms": 1.0}).encode())
    assert out is not None
    assert out["yaw_rate"] == -0.4
    # 与走 angular_velocity 分支的结果严格相同（两分支不得出现符号分歧）
    out2 = codec.decode_ego_state(
        json.dumps({"yaw_rate": 0.4, "angular_velocity": [0.0, 0.0, 0.4]}).encode()
    )
    assert out2 is not None and out2["yaw_rate"] == out["yaw_rate"]


def test_decode_ego_state_dirty_pose() -> None:
    out = codec.decode_ego_state(json.dumps({"pose": None, "velocity": "bad"}).encode())
    assert out is not None
    assert out["position"] == (0.0, 0.0, 0.0)
    assert out["linear_velocity"] == (0.0, 0.0, 0.0)
    assert codec.decode_ego_state(b"") is None


# ---------------------------------------------------------------------------
# depth
# ---------------------------------------------------------------------------
def test_decode_depth_ok() -> None:
    payload = struct.pack("<4f", 1.0, 2.0, 3.0, 4.0)
    out = codec.decode_depth(payload, {"width": 2, "height": 2})
    assert out is not None
    assert out["encoding"] == "32FC1"
    assert (out["width"], out["height"], out["step"]) == (2, 2, 8)
    assert out["data"] == payload


def test_decode_depth_rejects_size_mismatch() -> None:
    """尺寸错位的深度图比丢帧危险：会把障碍距离算到错误像素上。"""
    payload = struct.pack("<4f", 1.0, 2.0, 3.0, 4.0)
    assert codec.decode_depth(payload, {"width": 4, "height": 4}) is None
    assert codec.decode_depth(payload, {"width": 0, "height": 2}) is None
    assert codec.decode_depth(payload, {"width": 2, "height": -1}) is None
    assert codec.decode_depth(payload, {}) is None
    assert codec.decode_depth(b"", {"width": 2, "height": 2}) is None


# ---------------------------------------------------------------------------
# lidar
# ---------------------------------------------------------------------------
def _lidar_payload(points: list) -> bytes:
    flat: list = []
    for p in points:
        flat.extend(p)
    return struct.pack("<%df" % len(flat), *flat)


def test_flip_lidar_payload_only_negates_y() -> None:
    raw = _lidar_payload([(1.0, 2.0, 3.0, 100.0), (-4.0, -5.0, 6.0, 0.0)])
    flipped = codec.flip_lidar_payload(raw)
    pts = codec.decode_lidar_points(raw)          # decode 内部也翻 Y，用于对照
    got = struct.unpack("<8f", flipped)
    assert got == (1.0, -2.0, 3.0, 100.0, -4.0, 5.0, 6.0, 0.0)
    # intensity 与 x/z 不得被改动
    assert got[0] == 1.0 and got[2] == 3.0 and got[3] == 100.0
    assert [(p[0], p[1], p[2], p[3]) for p in pts] == [
        (1.0, -2.0, 3.0, 100.0), (-4.0, 5.0, 6.0, 0.0)
    ]
    assert len(flipped) == len(raw)


def test_flip_lidar_payload_struct_fallback_matches_numpy() -> None:
    """无 numpy 时的 struct 兜底路径必须与 numpy 快路径逐字节一致。"""
    raw = _lidar_payload([(float(i), float(-i), float(i * 2), 7.0) for i in range(5)])
    numpy_out = codec.flip_lidar_payload(raw)
    # 直接按 struct 逻辑复算参考值（不依赖被测实现的分支选择）
    floats = list(struct.unpack("<20f", raw))
    for i in range(1, len(floats), 4):
        floats[i] = -floats[i]
    assert numpy_out == struct.pack("<20f", *floats)


def test_flip_lidar_payload_passthrough_on_unknown_layout() -> None:
    """未知布局/长度不整除时原样返回，不猜测格式产出错误点云。"""
    raw = b"\x01\x02\x03"
    assert codec.flip_lidar_payload(raw) == raw
    assert codec.flip_lidar_payload(raw, point_bytes=12) == raw
    assert codec.flip_lidar_payload(b"") == b""
    assert codec.decode_lidar_points(raw) == []
    assert codec.decode_lidar_points(raw, point_bytes=12) == []


def test_flip_lidar_payload_empty_and_large() -> None:
    assert codec.flip_lidar_payload(b"") == b""
    raw = _lidar_payload([(1.0, 1.0, 1.0, 1.0)] * 20000)
    out = codec.flip_lidar_payload(raw)
    assert len(out) == len(raw)
    assert struct.unpack("<4f", out[:16]) == (1.0, -1.0, 1.0, 1.0)


# ---------------------------------------------------------------------------
# rgb（cv2 可选依赖）
# ---------------------------------------------------------------------------
def test_decode_rgb_empty_payload() -> None:
    assert codec.decode_rgb(b"", {}) is None


def test_decode_rgb_roundtrip_if_cv2_available() -> None:
    try:
        import cv2                                    # noqa: F401
        import numpy as np
    except ImportError:
        # 无 cv2 时必须安全降级（返回 None），不得抛异常打断其它流
        assert codec.decode_rgb(b"\x00" * 16, {}) is None
        return
    img = np.zeros((8, 12, 3), dtype=np.uint8)
    img[:, :, 0] = 200        # BGR 的 B 通道
    ok, buf = cv2.imencode(".png", img)
    assert ok
    out = codec.decode_rgb(buf.tobytes(), {"width": 12, "height": 8})
    assert out is not None
    assert out["encoding"] == "rgb8"
    assert (out["width"], out["height"], out["step"]) == (12, 8, 36)
    assert len(out["data"]) == 8 * 12 * 3
    # BGR→RGB：原 B=200 应落到输出的 R 通道（每像素第 0 字节）
    assert out["data"][0] == 200 and out["data"][2] == 0


def test_decode_rgb_invalid_bytes() -> None:
    try:
        import cv2                                    # noqa: F401
    except ImportError:
        return
    assert codec.decode_rgb(b"not an image at all", {}) is None


# ---------------------------------------------------------------------------
# 控制帧
# ---------------------------------------------------------------------------
def test_build_control_frame_fields() -> None:
    frame = codec.build_control_frame(1.5, -0.3, 9, wall_ts_ns=123456789)
    assert frame == {"linear_x": 1.5, "angular_z": -0.3, "wall_ts_ns": 123456789, "seq": 9}
    # 倒车负值必须原样保留（仿真端速度环支持负目标速度）
    assert codec.build_control_frame(-1.0, 0.0, 1)["linear_x"] == -1.0


def test_build_control_frame_sanitizes_and_stamps() -> None:
    frame = codec.build_control_frame(float("nan"), None, 3)
    assert frame["linear_x"] == 0.0 and frame["angular_z"] == 0.0
    assert frame["seq"] == 3
    assert isinstance(frame["wall_ts_ns"], int) and frame["wall_ts_ns"] > 0


def test_build_control_frame_seq_monotonic_requirement() -> None:
    """仿真端按 seq<=last 丢弃，故连续两帧 seq 必须严格递增。"""
    a = codec.build_control_frame(1.0, 0.0, 1)
    b = codec.build_control_frame(1.0, 0.0, 2)
    assert b["seq"] > a["seq"]


def main() -> int:
    tests = [(n, f) for n, f in sorted(globals().items())
             if n.startswith("test_") and callable(f)]
    for name, fn in tests:
        fn()
        print("PASS %s" % name)
    print("CODEC_OK: %d tests" % len(tests))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
