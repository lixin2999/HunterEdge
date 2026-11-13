#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CARLA ↔ ROS 编解码与坐标系转换（V0.1.38 真 SIL 桥接）。

本模块**不依赖 rclpy / ROS 消息类型**，只产出纯 Python 数据结构与字节，
由 :mod:`sil_bridge_agent.sil_bridge_agent` 装配成 ROS 消息。这样坐标转换、
信封解析、payload 解码这些最容易出错的部分可以在无 ROS 环境（Windows CI）
直接单测。

────────────────────────────────────────────────────────────────────────
坐标系转换（**HunterSim B2 符号约定的车端对偶，错一处整个闭环就反向**）
────────────────────────────────────────────────────────────────────────
CARLA/Unreal 是**左手系**：X 前、**Y 右**、Z 上；
ROS REP-103 是**右手系**：X 前、**Y 左**、Z 上。
二者相差一个反射 ``R = diag(1, -1, 1)``，``det(R) = -1``。由此：

* **位置 / 真向量**（速度、加速度）：``v_ros = R · v_carla = (x, -y, z)``
* **角速度 / 伪向量**（陀螺、横摆）：反射同时翻转轴与旋向，
  ``ω_ros = det(R) · R · ω_carla = (-x, +y, -z)``
* **姿态四元数**：``q_ros = (-qx, +qy, -qz, qw)``
* **欧拉角**（小角度下等于旋转向量分量）：``(-roll, +pitch, -yaw)``

仿真端 ``EdgeLinkRuntime`` 上行的是 CARLA 原始量（``yaw_rate`` 已由 deg/s
转 rad/s，但**仍在 CARLA 系**），所以取反必须在这里做。

HunterSim B2 的 ``_edge_twist_to_control`` 按 ROS 口径实现
``steer = -atan(L·ω / max(|v|, 0.5)) / MAX_STEER``（ROS ω>0 为左转，
CARLA steer>0 朝 +Y 即右）。**只有本模块把位姿/横摆翻到 ROS 系，
车端全栈算出的 /cmd_vel 回传后符号才自洽**；否则车会朝反方向转向。

────────────────────────────────────────────────────────────────────────
上行线格式（与 HunterSim ``cloud/edge_link.py`` 严格对齐）
────────────────────────────────────────────────────────────────────────
Topic ``carla.{sim_id}.edge_sensor``，JSON 信封::

    {stream, sim_ts, wall_ts_ns, seq, meta, payload_b64}

各流 payload：
* ``rgb``       JPEG/PNG 字节（meta: content_type/format/width/height/frame_id）
* ``depth``     32FC1 float32 米制（meta: encoding/width/height/frame_id）
* ``lidar``     xyzI float32×4，point_bytes=16（meta: points/point_bytes/encoding）
* ``imu``       JSON ``{accel:[x,y,z], gyro:[x,y,z]}``（CARLA 系）
* ``ego_state`` JSON ``{speed_ms, velocity[3], yaw_rate, angular_velocity[3], pose{...}}``
* ``session``   空 payload，meta ``{phase: boot|stop, sim_id, scene_id, map_id, ...}``

下行 Topic ``carla.{sim_id}.edge_control``：``{linear_x, angular_z, wall_ts_ns, seq}``。
"""
from __future__ import annotations

import base64
import binascii
import json
import math
import struct
import time
from typing import Any, Dict, List, Optional, Tuple

# ─── 上行流名（与仿真端 EdgeLinkRuntime 一致）───────────────────────────────
STREAM_RGB = "rgb"
STREAM_DEPTH = "depth"
STREAM_LIDAR = "lidar"
STREAM_IMU = "imu"
STREAM_EGO_STATE = "ego_state"
STREAM_SESSION = "session"

#: lidar 单点字节数（xyzI = 4 × float32）；仿真端 ``_LIDAR_POINT_BYTES`` 同值。
LIDAR_POINT_BYTES = 16

#: 会话相位受控词表（仿真端 ``_send_session`` 只发这两个）。
SESSION_PHASES = ("boot", "stop")


# ---------------------------------------------------------------------------
# 坐标系转换
# ---------------------------------------------------------------------------
def carla_position_to_ros(x: float, y: float, z: float) -> Tuple[float, float, float]:
    """位置 / 真向量：CARLA 左手系 → ROS 右手系（Y 取反）。"""
    return (float(x), -float(y), float(z))


def carla_angular_to_ros(x: float, y: float, z: float) -> Tuple[float, float, float]:
    """角速度 / 伪向量：反射同时翻转轴与旋向 → ``(-x, +y, -z)``。"""
    return (-float(x), float(y), -float(z))


def carla_euler_to_ros(
    roll: float, pitch: float, yaw: float
) -> Tuple[float, float, float]:
    """欧拉角 CARLA → ROS（小角度下等于旋转向量分量，故同伪向量规则）。

    地面车辆 roll/pitch ≈ 0，主导量是 yaw；``yaw_ros = -yaw_carla`` 是
    2D 导航（Nav2 / auto_mission）唯一真正消费的项，必须精确。
    """
    return (-float(roll), float(pitch), -float(yaw))


def euler_to_quaternion(
    roll: float, pitch: float, yaw: float
) -> Tuple[float, float, float, float]:
    """欧拉角 → 四元数 ``(x, y, z, w)``（ROS REP-103 惯例，内旋 ZYX）。"""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def carla_pose_to_ros_quaternion(pose: Dict[str, Any]) -> Tuple[float, float, float, float]:
    """CARLA 位姿欧拉角 → ROS 四元数（先翻系再构造，避免直接翻四元数分量的歧义）。"""
    roll, pitch, yaw = carla_euler_to_ros(
        _f(pose.get("roll")), _f(pose.get("pitch")), _f(pose.get("yaw"))
    )
    return euler_to_quaternion(roll, pitch, yaw)


def _f(value: Any, default: float = 0.0) -> float:
    """安全转 float：None / 非法 / 非有限一律回落 default（单帧脏数据不得炸链路）。"""
    try:
        out = float(value)
    except (TypeError, ValueError):
        return default
    return out if math.isfinite(out) else default


def _vec3(value: Any) -> Tuple[float, float, float]:
    """把任意三元序列安全转成 float 三元组（长度不足补 0，超长截断）。"""
    if not isinstance(value, (list, tuple)):
        return (0.0, 0.0, 0.0)
    items = [_f(v) for v in value[:3]]
    while len(items) < 3:
        items.append(0.0)
    return (items[0], items[1], items[2])


# ---------------------------------------------------------------------------
# 信封解析
# ---------------------------------------------------------------------------
def decode_envelope(raw: Any) -> Optional[Dict[str, Any]]:
    """解析一条上行 JSON 信封；非法/非对象返回 None（不抛，消费循环不得中断）。"""
    if isinstance(raw, (bytes, bytearray)):
        try:
            raw = raw.decode("utf-8")
        except UnicodeDecodeError:
            return None
    if isinstance(raw, str):
        try:
            raw = json.loads(raw)
        except (json.JSONDecodeError, ValueError):
            return None
    if not isinstance(raw, dict):
        return None
    stream = raw.get("stream")
    if not isinstance(stream, str) or not stream:
        return None
    return raw


def envelope_payload(envelope: Dict[str, Any]) -> bytes:
    """取 base64 payload；缺失/非法返回空 bytes（session 帧本就是空串）。"""
    encoded = envelope.get("payload_b64")
    if not isinstance(encoded, str) or not encoded:
        return b""
    try:
        return base64.b64decode(encoded, validate=True)
    except (binascii.Error, ValueError):
        return b""


def envelope_meta(envelope: Dict[str, Any]) -> Dict[str, Any]:
    """取 meta（非 dict 时返回空 dict，调用方无需再判类型）。"""
    meta = envelope.get("meta")
    return meta if isinstance(meta, dict) else {}


def session_phase(envelope: Dict[str, Any]) -> Optional[str]:
    """会话帧相位；非 session 帧或非法相位返回 None。"""
    if envelope.get("stream") != STREAM_SESSION:
        return None
    phase = envelope_meta(envelope).get("phase")
    return phase if phase in SESSION_PHASES else None


def session_sim_id(envelope: Dict[str, Any]) -> str:
    """会话帧携带的 sim_id（缺失返回空串）。"""
    value = envelope_meta(envelope).get("sim_id")
    return str(value) if value else ""


def sim_id_from_topic(topic: str) -> str:
    """从 ``carla.{sim_id}.edge_sensor`` 反解 sim_id（正则订阅下定位来源实例）。"""
    if not topic:
        return ""
    parts = topic.split(".")
    if len(parts) >= 3 and parts[0] == "carla":
        return ".".join(parts[1:-1])
    return ""


# ---------------------------------------------------------------------------
# 各流 payload 解码（输出已在 ROS 系）
# ---------------------------------------------------------------------------
def decode_imu(payload: bytes) -> Optional[Dict[str, Any]]:
    """IMU JSON → ROS 系加速度/角速度。

    加速度是真向量 ``(x,-y,z)``；陀螺是伪向量 ``(-x,+y,-z)``。
    """
    body = _decode_json(payload)
    if body is None:
        return None
    accel = carla_position_to_ros(*_vec3(body.get("accel")))
    gyro = carla_angular_to_ros(*_vec3(body.get("gyro")))
    return {
        "accel": accel,
        "gyro": gyro,
        # 仿真无姿态解算，朝向由 ego_state 提供；协方差置 0 表示「未知」，
        # 让下游按自身策略处理，不伪造精度。
        "orientation_covariance": [0.0] * 9,
    }


def decode_ego_state(payload: bytes) -> Optional[Dict[str, Any]]:
    """自车真值 → ROS 系（SIL 用仿真真值定位替代 fast_lio2 / robot_localization）。

    输出字段：
    * ``position`` / ``orientation``：ROS 系位姿（odom 用）
    * ``linear_velocity``：ROS 系速度（m/s）
    * ``angular_velocity``：ROS 系角速度（rad/s）
    * ``speed_ms``：速度模长（标量，与坐标系无关，底盘反馈用）
    * ``yaw_rate``：ROS 系横摆角速度（rad/s，左转为正）
    """
    body = _decode_json(payload)
    if body is None:
        return None
    pose = body.get("pose")
    pose = pose if isinstance(pose, dict) else {}
    position = carla_position_to_ros(
        _f(pose.get("x")), _f(pose.get("y")), _f(pose.get("z"))
    )
    velocity = carla_position_to_ros(*_vec3(body.get("velocity")))
    angular = carla_angular_to_ros(*_vec3(body.get("angular_velocity")))
    # yaw_rate 是伪向量的 z 分量：优先用已转好的 angular[2]，
    # 缺失时回落标量字段并取反（仿真端两者同源，取反规则一致）。
    if body.get("angular_velocity") is not None:
        yaw_rate = angular[2]
    else:
        yaw_rate = -_f(body.get("yaw_rate"))
    return {
        "position": position,
        "orientation": carla_pose_to_ros_quaternion(pose),
        "linear_velocity": velocity,
        "angular_velocity": angular,
        "speed_ms": _f(body.get("speed_ms")),
        "yaw_rate": yaw_rate,
    }


def decode_depth(payload: bytes, meta: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    """32FC1 米制深度 → Image 描述（深度是标量距离，无需坐标翻转）。

    校验字节数与 width×height×4 一致，不一致返回 None——尺寸错位的深度图
    会让感知把障碍距离算到错误像素上，比丢帧危险得多。
    """
    width = int(_f(meta.get("width")))
    height = int(_f(meta.get("height")))
    if width <= 0 or height <= 0:
        return None
    if len(payload) != width * height * 4:
        return None
    return {
        "encoding": "32FC1",
        "width": width,
        "height": height,
        "step": width * 4,
        "data": payload,
    }


def flip_lidar_payload(payload: bytes, point_bytes: int = LIDAR_POINT_BYTES) -> bytes:
    """把 xyzI 点云的 Y 分量取反（CARLA 左手系 → ROS 右手系）。

    只处理已知的 16 字节布局；其它布局原样返回（不猜测未知格式，
    宁可让下游按 CARLA 系报错也不静默产出错误点云）。
    """
    if point_bytes != LIDAR_POINT_BYTES or not payload:
        return payload
    if len(payload) % LIDAR_POINT_BYTES:
        return payload
    try:
        import numpy as np  # noqa: PLC0415  Jetson 上点云可达 10 万点，走 numpy
    except ImportError:
        floats = list(struct.unpack("<%df" % (len(payload) // 4), payload))
        for i in range(1, len(floats), 4):
            floats[i] = -floats[i]
        return struct.pack("<%df" % len(floats), *floats)
    points = np.frombuffer(payload, dtype="<f4").reshape(-1, 4).copy()
    points[:, 1] = -points[:, 1]
    return points.astype("<f4").tobytes()


def decode_lidar_points(
    payload: bytes, point_bytes: int = LIDAR_POINT_BYTES
) -> List[Tuple[float, float, float, float]]:
    """点云 → ROS 系 ``(x, y, z, intensity)`` 列表（诊断/单测用，非发布热路径）。"""
    if point_bytes != LIDAR_POINT_BYTES or len(payload) % LIDAR_POINT_BYTES:
        return []
    count = len(payload) // LIDAR_POINT_BYTES
    flat = struct.unpack("<%df" % (count * 4), payload)
    return [
        (flat[i], -flat[i + 1], flat[i + 2], flat[i + 3])
        for i in range(0, count * 4, 4)
    ]


def decode_rgb(payload: bytes, meta: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    """JPEG/PNG → rgb8 Image 描述。

    仿真端上行的是**已编码产物**（``encode_rgb_frame``，默认 JPEG，宽度沿用
    ``ADAPTER_STREAM_MAX_WIDTH``），不是裸 BGRA。这里解码后转 RGB8，
    与 HunterEdge ``vision_perception`` 期望的 ``/camera/color/image_raw``
    编码一致。cv2 不可用时返回 None（降级为不发图像，其它流照常）。
    """
    if not payload:
        return None
    try:
        import cv2  # noqa: PLC0415  可选依赖惰性导入
        import numpy as np  # noqa: PLC0415
    except ImportError:
        return None
    buf = np.frombuffer(payload, dtype=np.uint8)
    bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    if bgr is None:
        return None
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    height, width = rgb.shape[:2]
    return {
        "encoding": "rgb8",
        "width": int(width),
        "height": int(height),
        "step": int(width) * 3,
        "data": rgb.tobytes(),
    }


def build_control_frame(
    linear_x: float, angular_z: float, seq: int, wall_ts_ns: Optional[int] = None
) -> Dict[str, Any]:
    """构造下行控制帧（Twist 语义，与仿真端 ``_handle_control`` 对齐）。

    仿真端按 ``seq`` 单调去重（``seq <= last`` 直接丢弃），故 seq 必须严格递增；
    ``wall_ts_ns`` 供仿真端算控制延迟（B4 面板链路卡片消费）。
    """
    return {
        "linear_x": _f(linear_x),
        "angular_z": _f(angular_z),
        "wall_ts_ns": int(wall_ts_ns if wall_ts_ns is not None else time.time() * 1e9),
        "seq": int(seq),
    }


def _decode_json(payload: bytes) -> Optional[Dict[str, Any]]:
    """payload → JSON 对象；非法返回 None。"""
    if not payload:
        return None
    try:
        body = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError, ValueError):
        return None
    return body if isinstance(body, dict) else None


__all__ = [
    "STREAM_RGB", "STREAM_DEPTH", "STREAM_LIDAR", "STREAM_IMU",
    "STREAM_EGO_STATE", "STREAM_SESSION", "LIDAR_POINT_BYTES", "SESSION_PHASES",
    "carla_position_to_ros", "carla_angular_to_ros", "carla_euler_to_ros",
    "euler_to_quaternion", "carla_pose_to_ros_quaternion",
    "decode_envelope", "envelope_payload", "envelope_meta",
    "session_phase", "session_sim_id", "sim_id_from_topic",
    "decode_imu", "decode_ego_state", "decode_depth",
    "flip_lidar_payload", "decode_lidar_points", "decode_rgb",
    "build_control_frame",
]
