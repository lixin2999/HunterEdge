# -*- coding: utf-8 -*-
"""节点层单测：用假 rclpy / ROS 消息 / confluent_kafka / hunter_kafka 驱动 SilBridgeAgent。

覆盖真 SIL 闭环里「错了不报错、只是静默不工作」的高危约定：
* 话题名与 frame_id 必须等于 sensor_params.yaml（相机双层 namespace、/Odometry）
* 传感器 QoS 必须 best_effort（reliable 会因 QoS 不兼容静默收不到）
* ego_state 必须同时发 /localization/odom 与 /Odometry，并接管两条 TF
* 控制回传不限幅不平滑、seq 严格递增、stop 补恰好一帧零速
* 事件只上报受控词表内类型（自造类型/错等级会被平台判非法进 DLQ）

本文件不依赖任何真实 ROS/Kafka，可在 Windows CI 直接 pytest 或 python 运行。
"""
from __future__ import annotations

import base64
import json
import os
import struct
import sys
import tempfile
import time
import types

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

SIM = "SIM-001"
OTHER = "SIM-002"
TOPIC = "carla.%s.edge_sensor" % SIM
OTHER_TOPIC = "carla.%s.edge_sensor" % OTHER


# ---------------------------------------------------------------------------
# 假 ROS / Kafka 依赖
# ---------------------------------------------------------------------------
# 假消息必须复刻 ROS 的**默认零值语义**：未赋值字段是 0.0 而不是空对象，
# 否则「朝向回落单位四元数」这类安全断言无法成立。
class _Vec3:
    def __init__(self):
        self.x = self.y = self.z = 0.0


class _Quat:
    def __init__(self):
        self.x = self.y = self.z = 0.0
        self.w = 0.0


class _Header:
    def __init__(self):
        self.stamp = None
        self.frame_id = ""


class _Pose:
    def __init__(self):
        self.position = _Vec3()
        self.orientation = _Quat()


class _PoseWithCov:
    def __init__(self):
        self.pose = _Pose()
        self.covariance = [0.0] * 36


class _Twist:
    def __init__(self):
        self.linear = _Vec3()
        self.angular = _Vec3()


class _TwistWithCov:
    def __init__(self):
        self.twist = _Twist()
        self.covariance = [0.0] * 36


class _Transform:
    def __init__(self):
        self.translation = _Vec3()
        self.rotation = _Quat()


class FakeImage:
    def __init__(self):
        self.header = _Header()
        self.height = self.width = self.step = 0
        self.encoding = ""
        self.is_bigendian = 0
        self.data = b""


class FakeImu:
    def __init__(self):
        self.header = _Header()
        self.orientation = _Quat()
        self.orientation_covariance = [0.0] * 9
        self.angular_velocity = _Vec3()
        self.angular_velocity_covariance = [0.0] * 9
        self.linear_acceleration = _Vec3()
        self.linear_acceleration_covariance = [0.0] * 9


class FakePointCloud2:
    def __init__(self):
        self.header = _Header()
        self.height = self.width = 0
        self.fields = []
        self.is_bigendian = False
        self.point_step = self.row_step = 0
        self.data = b""
        self.is_dense = False


class FakePointField:
    FLOAT32 = 7

    def __init__(self, name="", offset=0, datatype=0, count=0):
        self.name, self.offset, self.datatype, self.count = name, offset, datatype, count


class FakeTwist:
    def __init__(self):
        self.linear = _Vec3()
        self.angular = _Vec3()


class FakeTransformStamped:
    def __init__(self):
        self.header = _Header()
        self.child_frame_id = ""
        self.transform = _Transform()


class FakeOdometry:
    def __init__(self):
        self.header = _Header()
        self.child_frame_id = ""
        self.pose = _PoseWithCov()
        self.twist = _TwistWithCov()


class FakeChassisState:
    def __init__(self):
        self.header = _Header()
        self.velocity = 0.0
        self.steering_angle = 0.0
        self.battery_voltage = 0.0
        self.battery_soc = 0.0
        self.battery_current = 0.0
        self.battery_temperature = 0.0
        self.control_mode = ""
        self.vehicle_state = ""
        self.fault_code = 0
        self.motor_temperature = []


class FakePublisher:
    def __init__(self, msg_type, topic, qos):
        self.msg_type, self.topic, self.qos = msg_type, topic, qos
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


class FakeNode:
    def __init__(self, name):
        self.name = name
        self.pubs = []
        self.subs = []
        self.destroyed = False

    def create_publisher(self, msg_type, topic, qos):
        pub = FakePublisher(msg_type, topic, qos)
        self.pubs.append(pub)
        return pub

    def create_subscription(self, msg_type, topic, callback, qos):
        self.subs.append((msg_type, topic, callback, qos))
        return self.subs[-1]

    def get_clock(self):
        return FakeClock()

    def destroy_node(self):
        self.destroyed = True


class _Stamp:
    sec = 1760313600
    nanosec = 500000000


CLOCK_STAMP = _Stamp()


class FakeClock:
    def now(self):
        return self

    def to_msg(self):
        return CLOCK_STAMP


class FakeQoS:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


class FakeReliability:
    BEST_EFFORT = "BEST_EFFORT"
    RELIABLE = "RELIABLE"


class FakeHistory:
    KEEP_LAST = "KEEP_LAST"
    KEEP_ALL = "KEEP_ALL"


class FakeTF:
    def __init__(self, node):
        self.node = node
        self.sent = []

    def sendTransform(self, transforms):
        self.sent.append(list(transforms))


class FakeProducer:
    def __init__(self, conf=None):
        self.conf = conf or {}
        self.produced = []
        self.flushed = 0
        self.raise_on_produce = None

    def produce(self, topic, value=None, key=None, on_delivery=None, **kwargs):
        if self.raise_on_produce is not None:
            raise self.raise_on_produce
        self.produced.append({"topic": topic, "value": value, "key": key,
                              "on_delivery": on_delivery})

    def poll(self, timeout):
        return 0

    def flush(self, timeout=None):
        self.flushed += 1
        return 0


class FakeConsumer:
    def __init__(self, conf=None):
        self.conf = conf or {}
        self.subscribed = []
        self.closed = False

    def subscribe(self, topics):
        self.subscribed = list(topics)

    def poll(self, timeout):
        return None

    def close(self):
        self.closed = True


class _Fakes:
    """收集假依赖的调用痕迹，供断言。"""

    def __init__(self):
        self.load_conf_calls = []
        self.make_producer_calls = []
        self.rclpy_init = 0
        self.rclpy_shutdown = 0


FAKES = _Fakes()


def _fake_load_conf(*, properties_path=None, bundle_dir=None, client_id=None,
                    role="producer", acks=None, extra=None, fallback=None):
    FAKES.load_conf_calls.append({"role": role, "acks": acks, "client_id": client_id,
                                  "extra": dict(extra or {})})
    conf = {"bootstrap.servers": "fake:9092", "__properties_path__": properties_path}
    conf.update(extra or {})
    return conf


def _fake_strip_meta(conf):
    return {k: v for k, v in conf.items() if not k.startswith("__")}


def _fake_make_producer(vehicle_id, kind, **kwargs):
    FAKES.make_producer_calls.append((vehicle_id, kind))
    return FakeProducer({"kind": kind})


def _fake_topic_name(vehicle_id, kind):
    return "hunter.%s.%s" % (vehicle_id, kind)


def _install_fakes() -> None:
    """在导入被测模块前注入假依赖（HAS_* 标志在导入期确定）。"""
    def mod(name, **attrs):
        m = types.ModuleType(name)
        for k, v in attrs.items():
            setattr(m, k, v)
        sys.modules[name] = m
        return m

    def pkg(name):
        if name not in sys.modules:
            m = types.ModuleType(name)
            m.__path__ = []
            sys.modules[name] = m
        return sys.modules[name]

    pkg("rclpy")
    mod("rclpy", init=lambda args=None: FAKES.__setattr__(
        "rclpy_init", FAKES.rclpy_init + 1),
        ok=lambda: True,
        shutdown=lambda: FAKES.__setattr__("rclpy_shutdown", FAKES.rclpy_shutdown + 1))
    mod("rclpy.node", Node=FakeNode)
    mod("rclpy.qos", QoSProfile=FakeQoS, ReliabilityPolicy=FakeReliability,
        HistoryPolicy=FakeHistory)

    class _Executor:
        def __init__(self, num_threads=1):
            self.num_threads = num_threads

        def add_node(self, node):
            pass

        def spin(self):
            pass                      # 立即返回，测试不起真实 spin

    mod("rclpy.executors", MultiThreadedExecutor=_Executor)

    pkg("sensor_msgs")
    mod("sensor_msgs.msg", Image=FakeImage, Imu=FakeImu, PointCloud2=FakePointCloud2,
        PointField=FakePointField)
    pkg("geometry_msgs")
    mod("geometry_msgs.msg", Twist=FakeTwist, TransformStamped=FakeTransformStamped)
    pkg("nav_msgs")
    mod("nav_msgs.msg", Odometry=FakeOdometry)
    pkg("hunter_msgs")
    mod("hunter_msgs.msg", ChassisState=FakeChassisState)
    pkg("std_msgs")
    mod("std_msgs.msg", Header=_Header)
    mod("tf2_ros", TransformBroadcaster=FakeTF)
    mod("confluent_kafka", Consumer=FakeConsumer, Producer=FakeProducer)

    pkg("hunter_kafka")
    mod("hunter_kafka.client", make_producer=_fake_make_producer,
        topic_name=_fake_topic_name)

    class KafkaConfigError(RuntimeError):
        pass

    mod("hunter_kafka.config", DEFAULT_BUNDLE_DIR="/etc/hunter/kafka",
        DEFAULT_PROPERTIES_PATH="/etc/hunter/kafka/kafka.properties",
        KafkaConfigError=KafkaConfigError, load_conf=_fake_load_conf,
        load_properties=lambda path: {}, resolve_vehicle_id=lambda props: "",
        strip_meta=_fake_strip_meta)


_install_fakes()

from sil_bridge_agent import sil_bridge_agent as mod          # noqa: E402


# ---------------------------------------------------------------------------
# 测试脚手架
# ---------------------------------------------------------------------------
def _agent(**overrides):
    cfg = dict(mod.DEFAULT_CONFIG)
    # 默认关闭限频，避免限频干扰路由断言（限频本身在 test_session 覆盖）
    cfg.update({k: 0.0 for k in
                ("lidar_fps", "rgb_fps", "depth_fps", "imu_fps", "ego_state_fps")})
    cfg.update(overrides)
    agent = mod.SilBridgeAgent(cfg)
    assert agent.init_ros() is True
    agent.control_producer = FakeProducer()
    return agent


def _env(stream, payload=b"", meta=None, sim_ts=12.5):
    return json.dumps({
        "stream": stream, "sim_ts": sim_ts, "wall_ts_ns": 1760313600000000000,
        "seq": 1, "meta": meta or {},
        "payload_b64": base64.b64encode(payload).decode("ascii"),
    }).encode("utf-8")


def _boot(agent, sim_id=SIM, topic=TOPIC):
    agent.on_message(_env("session", b"", {"phase": "boot", "sim_id": sim_id,
                                           "scene_id": "SCN-2056", "map_id": "Town03"}), topic)


def _feed(agent, stream, payload=b"", meta=None, topic=TOPIC):
    agent.on_message(_env(stream, payload, meta), topic)


def _lidar_payload(points):
    flat = []
    for p in points:
        flat.extend(p)
    return struct.pack("<%df" % len(flat), *flat)


def _ego_payload(**over):
    body = {"speed_ms": 3.5, "velocity": [3.5, 1.0, 0.0], "yaw_rate": 0.4,
            "angular_velocity": [0.0, 0.0, 0.4],
            "pose": {"x": 10.0, "y": -2.0, "z": 0.3, "yaw": 0.5, "pitch": 0.0, "roll": 0.0}}
    body.update(over)
    return json.dumps(body).encode()


def _twist(linear_x, angular_z):
    msg = FakeTwist()
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    return msg


def _frames(producer):
    return [json.loads(p["value"].decode("utf-8")) for p in producer.produced]


# ---------------------------------------------------------------------------
# 配置与话题契约
# ---------------------------------------------------------------------------
def test_default_topics_match_sensor_params() -> None:
    """话题名/frame_id 必须等于 hunter_bringup/config/sensor_params.yaml。"""
    cfg = mod.DEFAULT_CONFIG
    assert cfg["lidar_topic"] == "/lidar_points" and cfg["lidar_frame"] == "rslidar"
    # RealSense 双层 namespace：写成单层会静默无消费者
    assert cfg["rgb_topic"] == "/camera/camera/color/image_raw"
    assert cfg["depth_topic"] == "/camera/camera/depth/image_rect_raw"
    assert cfg["camera_frame"] == "camera_color_optical_frame"
    assert cfg["imu_topic"] == "/imu/data" and cfg["imu_frame"] == "imu"
    assert cfg["chassis_topic"] == "/chassis/state"
    assert cfg["odom_topic"] == "/localization/odom"
    # auto_mission mapping cruise 的巡航反馈源（lio_odom_sub_）
    assert cfg["lio_odom_topic"] == "/Odometry"
    assert cfg["cmd_vel_topic"] == "/cmd_vel"
    assert (cfg["base_frame"], cfg["odom_frame"], cfg["map_frame"]) == \
        ("base_link", "odom", "map")


def test_default_config_carries_no_credentials() -> None:
    cfg = mod.DEFAULT_CONFIG
    assert not any(("password" in k or "sasl" in k or "secret" in k) for k in cfg)
    assert cfg["kafka_properties"] == "/etc/hunter/kafka/kafka.properties"
    assert cfg["watchdog_ms"] == 500
    # data_agent 独占 health 1Hz，默认必须关闭避免双发布者
    assert cfg["enable_health_report"] is False
    # battery_soc 置 0 会引发 data_agent 事件风暴
    assert cfg["battery_soc"] > 0.0 and cfg["battery_voltage"] > 0.0


def test_sensor_topic_regex_is_librdkafka_regex() -> None:
    assert mod.SENSOR_TOPIC_REGEX.startswith("^")
    assert mod.SENSOR_TOPIC_REGEX == r"^carla\..+\.edge_sensor$"


def test_load_config_nested_yaml_override() -> None:
    with tempfile.TemporaryDirectory() as root:
        path = os.path.join(root, "p.yaml")
        with open(path, "w", encoding="utf-8") as fh:
            fh.write("sil_bridge_agent:\n  ros__parameters:\n"
                     "    watchdog_ms: 800\n    sim_id_filter: \"SIM-9\"\n")
        cfg = mod.load_config(path)
    assert cfg["watchdog_ms"] == 800 and cfg["sim_id_filter"] == "SIM-9"
    assert cfg["lidar_topic"] == "/lidar_points"      # 未覆盖项保留默认


def test_load_config_flat_yaml_and_missing_file() -> None:
    with tempfile.TemporaryDirectory() as root:
        path = os.path.join(root, "flat.yaml")
        with open(path, "w", encoding="utf-8") as fh:
            fh.write("watchdog_ms: 250\n")
        assert mod.load_config(path)["watchdog_ms"] == 250
    fallback = mod.load_config(os.path.join(root, "absent.yaml"))
    assert fallback["watchdog_ms"] == mod.DEFAULT_CONFIG["watchdog_ms"]
    assert mod.load_config(None)["rgb_topic"] == mod.DEFAULT_CONFIG["rgb_topic"]


# ---------------------------------------------------------------------------
# ROS 初始化
# ---------------------------------------------------------------------------
def test_init_ros_qos_and_subscription() -> None:
    agent = _agent()
    by_topic = {p.topic: p for p in agent.ros_node.pubs}
    for topic in ("/lidar_points", "/camera/camera/color/image_raw",
                  "/camera/camera/depth/image_rect_raw", "/imu/data", "/Odometry"):
        assert by_topic[topic].qos.reliability == FakeReliability.BEST_EFFORT
        assert by_topic[topic].qos.history == FakeHistory.KEEP_LAST
    for topic in ("/chassis/state", "/localization/odom"):
        assert by_topic[topic].qos.reliability == FakeReliability.RELIABLE
    assert len(agent.ros_node.subs) == 1
    msg_type, topic, callback, _ = agent.ros_node.subs[0]
    assert topic == "/cmd_vel" and callback == agent.on_cmd_vel
    assert msg_type is FakeTwist
    assert isinstance(agent.tf_broadcaster, FakeTF)


def test_init_ros_respects_topic_overrides() -> None:
    agent = _agent(lidar_topic="/custom_points", publish_tf=False)
    assert "/custom_points" in {p.topic for p in agent.ros_node.pubs}
    assert agent.tf_broadcaster is None


# ---------------------------------------------------------------------------
# 会话与准入
# ---------------------------------------------------------------------------
def test_boot_binds_session() -> None:
    agent = _agent()
    _boot(agent)
    assert agent.session.active is True and agent.session.sim_id == SIM
    assert agent.session.control_topic == "carla.%s.edge_control" % SIM


def test_frames_before_boot_are_rejected() -> None:
    agent = _agent()
    _feed(agent, "lidar", _lidar_payload([(1.0, 2.0, 3.0, 4.0)]), {"point_bytes": 16})
    assert agent.publishers["lidar"].published == []
    assert agent.session.stats()["rejected_frames"] == 1


def test_other_instance_frames_are_rejected() -> None:
    agent = _agent()
    _boot(agent)
    _feed(agent, "lidar", _lidar_payload([(1.0, 2.0, 3.0, 4.0)]),
          {"point_bytes": 16}, topic=OTHER_TOPIC)
    assert agent.publishers["lidar"].published == []
    assert agent.session.stats()["rejected_frames"] == 1


def test_invalid_envelope_rejected_without_publish() -> None:
    agent = _agent()
    _boot(agent)
    agent.on_message(b"{not json", TOPIC)
    agent.on_message(json.dumps({"meta": {}}).encode(), TOPIC)
    assert agent.publishers["lidar"].published == []
    assert agent.session.stats()["rejected_frames"] == 2


def test_empty_payload_does_not_refresh_watchdog() -> None:
    agent = _agent()
    _boot(agent)
    before = agent.session.stats()["sensor_frames"]
    _feed(agent, "lidar", b"", {"point_bytes": 16})
    assert agent.session.stats()["sensor_frames"] == before


# ---------------------------------------------------------------------------
# 各流发布
# ---------------------------------------------------------------------------
def test_lidar_publish_flips_y_and_layout() -> None:
    agent = _agent()
    _boot(agent)
    raw = _lidar_payload([(1.0, 2.0, 3.0, 100.0), (-4.0, -5.0, 6.0, 0.0)])
    _feed(agent, "lidar", raw, {"point_bytes": 16, "points": 2})
    msgs = agent.publishers["lidar"].published
    assert len(msgs) == 1
    msg = msgs[0]
    assert msg.header.frame_id == "rslidar"
    assert (msg.height, msg.width) == (1, 2)
    assert (msg.point_step, msg.row_step) == (16, 32)
    assert msg.is_bigendian is False and msg.is_dense is True
    assert [(f.name, f.offset, f.datatype) for f in msg.fields] == [
        ("x", 0, 7), ("y", 4, 7), ("z", 8, 7), ("intensity", 12, 7)]
    assert struct.unpack("<8f", bytes(msg.data)) == \
        (1.0, -2.0, 3.0, 100.0, -4.0, 5.0, 6.0, 0.0)


def test_lidar_unknown_layout_not_published() -> None:
    agent = _agent()
    _boot(agent)
    _feed(agent, "lidar", b"\x01\x02\x03", {"point_bytes": 16})
    assert agent.publishers["lidar"].published == []


def test_depth_ok_and_size_mismatch() -> None:
    agent = _agent()
    _boot(agent)
    payload = struct.pack("<4f", 1.0, 2.0, 3.0, 4.0)
    _feed(agent, "depth", payload, {"width": 2, "height": 2})
    msgs = agent.publishers["depth"].published
    assert len(msgs) == 1
    assert msgs[0].encoding == "32FC1"
    assert (msgs[0].width, msgs[0].height, msgs[0].step) == (2, 2, 8)
    assert msgs[0].header.frame_id == "camera_color_optical_frame"
    # 尺寸错位必须丢帧（错位深度比丢帧危险）
    _feed(agent, "depth", payload, {"width": 4, "height": 4})
    assert len(agent.publishers["depth"].published) == 1


def test_imu_orientation_fallback_is_identity() -> None:
    """无 ego 缓存时朝向必须是单位四元数（全零四元数非法）。"""
    agent = _agent()
    _boot(agent)
    _feed(agent, "imu", json.dumps({"accel": [0.0, 0.0, 9.81],
                                    "gyro": [0.1, 0.2, 0.3]}).encode())
    msg = agent.publishers["imu"].published[0]
    assert msg.header.frame_id == "imu"
    assert (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w) == \
        (0.0, 0.0, 0.0, 1.0)
    assert (msg.linear_acceleration.x, msg.linear_acceleration.y,
            msg.linear_acceleration.z) == (0.0, 0.0, 9.81)
    assert (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z) == \
        (-0.1, 0.2, -0.3)


def test_imu_uses_cached_ego_orientation() -> None:
    agent = _agent()
    _boot(agent)
    _feed(agent, "ego_state", _ego_payload())
    _feed(agent, "imu", json.dumps({"accel": [0, 0, 9.81], "gyro": [0, 0, 0]}).encode())
    msg = agent.publishers["imu"].published[0]
    ego = agent._last_ego
    assert (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w) == \
        ego["orientation"]


def test_ego_state_publishes_chassis_and_dual_odom() -> None:
    agent = _agent()
    _boot(agent)
    _feed(agent, "ego_state", _ego_payload())
    assert len(agent.publishers["chassis"].published) == 1
    odom_msgs = agent.publishers["odom"].published
    lio_msgs = agent.publishers["lio_odom"].published
    assert len(odom_msgs) == 1 and len(lio_msgs) == 1
    # 同一份真值对象：航点反馈与定位严格同源，无 EKF 折算偏差
    assert odom_msgs[0] is lio_msgs[0]
    odom = odom_msgs[0]
    assert odom.header.frame_id == "odom" and odom.child_frame_id == "base_link"
    assert (odom.pose.pose.position.x, odom.pose.pose.position.y,
            odom.pose.pose.position.z) == (10.0, 2.0, 0.3)
    assert (odom.twist.twist.linear.x, odom.twist.twist.linear.y) == (3.5, -1.0)
    assert odom.twist.twist.angular.z == -0.4        # CARLA 右转 → ROS 负


def test_ego_state_tf_takes_over_ekf_and_global_localization() -> None:
    agent = _agent()
    _boot(agent)
    _feed(agent, "ego_state", _ego_payload())
    assert len(agent.tf_broadcaster.sent) == 1
    tfs = agent.tf_broadcaster.sent[0]
    pairs = [(t.header.frame_id, t.child_frame_id) for t in tfs]
    assert ("odom", "base_link") in pairs
    assert ("map", "odom") in pairs
    odom_tf = [t for t in tfs if t.child_frame_id == "base_link"][0]
    assert (odom_tf.transform.translation.x, odom_tf.transform.translation.y) == (10.0, 2.0)
    map_tf = [t for t in tfs if t.child_frame_id == "odom"][0]
    assert map_tf.transform.rotation.w == 1.0        # 恒等


def test_map_tf_can_be_disabled() -> None:
    agent = _agent(publish_map_tf=False)
    _boot(agent)
    _feed(agent, "ego_state", _ego_payload())
    pairs = [(t.header.frame_id, t.child_frame_id) for t in agent.tf_broadcaster.sent[0]]
    assert pairs == [("odom", "base_link")]


def test_chassis_steering_is_ackermann_inverse_of_last_cmd() -> None:
    """δ = atan(L·ω / max(|v|,0.5))，与 auto_mission 的 ω = v·tan(δ)/L 互逆。"""
    import math
    agent = _agent(wheelbase=0.46)
    _boot(agent)
    agent.on_cmd_vel(_twist(2.0, 0.8))
    _feed(agent, "ego_state", _ego_payload())
    msg = agent.publishers["chassis"].published[0]
    assert abs(msg.steering_angle - math.atan(0.46 * 0.8 / 2.0)) < 1e-12
    assert msg.velocity == 3.5                       # 真值速度，非指令速度
    assert msg.control_mode == "AUTO" and msg.vehicle_state == "NORMAL"
    assert msg.fault_code == 0
    assert msg.battery_soc == 100.0 and msg.battery_voltage == 48.0


def test_chassis_steering_low_speed_floor() -> None:
    import math
    agent = _agent(wheelbase=0.46)
    _boot(agent)
    agent.on_cmd_vel(_twist(0.0, 0.8))               # 零速时不得除零
    _feed(agent, "ego_state", _ego_payload())
    msg = agent.publishers["chassis"].published[0]
    assert abs(msg.steering_angle - math.atan(0.46 * 0.8 / 0.5)) < 1e-12


def test_stamp_uses_wall_clock_not_sim_ts() -> None:
    """CARLA 仿真秒不是 epoch，当 stamp 会被 TF/消息滤波器判远古全丢。"""
    agent = _agent()
    _boot(agent)
    _feed(agent, "ego_state", _ego_payload(), )
    assert agent.publishers["odom"].published[0].header.stamp is CLOCK_STAMP
    assert agent.publishers["chassis"].published[0].header.stamp is CLOCK_STAMP


def test_publish_error_isolated_per_stream() -> None:
    agent = _agent()
    _boot(agent)

    def boom(*a, **k):
        raise RuntimeError("publish failed")

    agent._publish_lidar = boom
    _feed(agent, "lidar", _lidar_payload([(1.0, 2.0, 3.0, 4.0)]), {"point_bytes": 16})
    assert agent._publish_errors == 1
    # 同一批其它流必须照常发布
    _feed(agent, "ego_state", _ego_payload())
    assert len(agent.publishers["odom"].published) == 1


def test_rate_limit_drops_excess_frames() -> None:
    agent = _agent(lidar_fps=10.0)
    _boot(agent)
    raw = _lidar_payload([(1.0, 2.0, 3.0, 4.0)])
    for _ in range(5):
        _feed(agent, "lidar", raw, {"point_bytes": 16})
    assert len(agent.publishers["lidar"].published) == 1
    assert agent.limiter.dropped.get("lidar") == 4


# ---------------------------------------------------------------------------
# 控制回传
# ---------------------------------------------------------------------------
def test_cmd_vel_produces_control_frame_without_clamping() -> None:
    agent = _agent()
    _boot(agent)
    agent.on_cmd_vel(_twist(9.0, -3.0))              # 超常值也不得被桥接层加工
    frames = _frames(agent.control_producer)
    assert len(frames) == 1
    assert frames[0]["linear_x"] == 9.0 and frames[0]["angular_z"] == -3.0
    assert frames[0]["seq"] == 1
    assert frames[0]["wall_ts_ns"] > 0
    produced = agent.control_producer.produced[0]
    assert produced["topic"] == "carla.%s.edge_control" % SIM
    assert produced["key"] == SIM.encode("utf-8")
    # confluent_kafka 2.16 唯一可用投递证实入口
    assert produced["on_delivery"] == agent._on_control_delivery


def test_control_seq_strictly_increasing() -> None:
    agent = _agent()
    _boot(agent)
    for i in range(4):
        agent.on_cmd_vel(_twist(1.0 + i, 0.1))
    seqs = [f["seq"] for f in _frames(agent.control_producer)]
    assert seqs == [1, 2, 3, 4]


def test_reverse_command_preserved() -> None:
    agent = _agent()
    _boot(agent)
    agent.on_cmd_vel(_twist(-1.5, 0.0))
    assert _frames(agent.control_producer)[0]["linear_x"] == -1.5


def test_cmd_vel_before_boot_not_produced() -> None:
    agent = _agent()
    agent.on_cmd_vel(_twist(1.0, 0.0))
    assert agent.control_producer.produced == []


def test_stop_flushes_exactly_one_zero_frame() -> None:
    agent = _agent()
    _boot(agent)
    agent.on_cmd_vel(_twist(2.0, 0.5))
    agent.on_message(_env("session", b"", {"phase": "stop", "sim_id": SIM}), TOPIC)
    frames = _frames(agent.control_producer)
    assert len(frames) == 2
    assert frames[-1]["linear_x"] == 0.0 and frames[-1]["angular_z"] == 0.0
    assert frames[-1]["seq"] == 2
    # 收尾后不得继续回传
    agent.on_cmd_vel(_twist(2.0, 0.5))
    assert len(agent.control_producer.produced) == 2
    assert agent.session.active is False


def test_stop_without_preceding_cmd_still_sends_zero() -> None:
    agent = _agent()
    _boot(agent)
    agent.on_message(_env("session", b"", {"phase": "stop", "sim_id": SIM}), TOPIC)
    frames = _frames(agent.control_producer)
    assert len(frames) == 1 and frames[0]["linear_x"] == 0.0


def test_produce_failure_counted_not_raised() -> None:
    agent = _agent()
    _boot(agent)
    agent.control_producer.raise_on_produce = RuntimeError("broker down")
    agent.on_cmd_vel(_twist(1.0, 0.0))               # 不得冒泡打断 ROS 回调
    assert agent._send_errors == 1


def test_delivery_error_counted() -> None:
    agent = _agent()
    agent._on_control_delivery("broker timeout", None)
    assert agent._send_errors == 1
    agent._on_control_delivery(None, None)
    assert agent._send_errors == 1


# ---------------------------------------------------------------------------
# Kafka 接入与事件契约
# ---------------------------------------------------------------------------
def test_init_kafka_roles_regex_and_channels() -> None:
    FAKES.load_conf_calls.clear()
    FAKES.make_producer_calls.clear()
    agent = _agent()
    assert agent.init_kafka() is True
    roles = [c["role"] for c in FAKES.load_conf_calls]
    assert roles == ["consumer", "producer"]         # 消费者不写生产者键（防 CONFWARN）
    sensor_conf = FAKES.load_conf_calls[0]
    assert sensor_conf["extra"]["auto.offset.reset"] == "latest"
    assert sensor_conf["extra"]["enable.auto.commit"] == "true"
    assert FAKES.load_conf_calls[1]["acks"] == "1"
    assert "__properties_path__" not in agent.sensor_consumer.conf
    assert agent.sensor_consumer.subscribed == [mod.SENSOR_TOPIC_REGEX]
    # 事件走契约通道；health 默认关闭（data_agent 独占）
    assert FAKES.make_producer_calls == [(agent.vehicle_id, "event")]
    assert agent.health_producer is None


def test_report_event_rejects_unlisted_types() -> None:
    agent = _agent()
    agent.event_producer = FakeProducer()
    agent.report_event("sil_session_start", {"sim_id": SIM})
    agent.report_event("remote_ws_connected")
    assert agent.event_producer.produced == []       # 自造类型不得进平台（DLQ）


def test_report_event_level_and_suppression() -> None:
    agent = _agent()
    agent.event_producer = FakeProducer()
    agent.report_event("communication_loss", {"sim_id": SIM})
    assert len(agent.event_producer.produced) == 1
    produced = agent.event_producer.produced[0]
    assert produced["topic"] == "hunter.%s.event" % agent.vehicle_id
    payload = json.loads(produced["value"].decode("utf-8"))
    assert payload["event_type"] == "communication_loss"
    assert payload["event_level"] == "critical"      # 契约等级，写错即非法事件
    assert payload["vehicle_id"] == agent.vehicle_id
    assert payload["data"]["sim_id"] == SIM
    # 抑制期内不得重复上报（防事件风暴）
    agent.report_event("communication_loss", {"sim_id": SIM})
    assert len(agent.event_producer.produced) == 1
    agent._last_event_ts["communication_loss"] = time.time() - mod.EVENT_MIN_INTERVAL_S - 1
    agent.report_event("communication_loss", {"sim_id": SIM})
    assert len(agent.event_producer.produced) == 2


def test_report_health_disabled_by_default() -> None:
    agent = _agent()
    agent.report_health(now=100.0)
    assert agent.health_producer is None


def test_report_health_payload_when_enabled() -> None:
    agent = _agent(enable_health_report=True)
    agent.health_producer = FakeProducer()
    _boot(agent)
    _feed(agent, "ego_state", _ego_payload())
    agent.report_health(now=time.time())
    produced = agent.health_producer.produced[0]
    assert produced["topic"] == "hunter.%s.health" % agent.vehicle_id
    payload = json.loads(produced["value"].decode("utf-8"))
    assert payload["status"] == "auto_driving"
    link = payload["system"]["sil_link"]
    assert link["active"] is True and link["sim_id"] == SIM
    assert link["sensor_frames"]["ego_state"] == 1


# ---------------------------------------------------------------------------
# boot 按地图装载航点（V0.2.0：map_id → sil_waypoints_<map>.yaml → 活动文件）
# ---------------------------------------------------------------------------
def _boot_map(agent, map_id):
    agent.on_message(_env("session", b"", {"phase": "boot", "sim_id": SIM,
                                           "scene_id": "SCN-2056",
                                           "map_id": map_id}), TOPIC)


def _wp_agent(tmpdir, map_dir="", active=""):
    target = os.path.join(tmpdir, active) if active else ""
    return _agent(waypoints_map_dir=map_dir, waypoints_active_file=target), target


def _write(path, text):
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(text)


def _read(path):
    with open(path, encoding="utf-8") as fh:
        return fh.read()


def test_boot_waypoints_auto_load_disabled_by_default() -> None:
    """dir/active 置空 = 关闭机制，不得碰任何文件（V0.1.39 行为）。"""
    with tempfile.TemporaryDirectory() as td:
        target = os.path.join(td, "active.yaml")
        agent = _agent(waypoints_map_dir="", waypoints_active_file=target)
        _boot_map(agent, "Town03")
        assert not os.path.exists(target)
        assert agent.session.active is True            # 关闭机制不影响会话本身


def test_boot_copies_matching_map_waypoints() -> None:
    with tempfile.TemporaryDirectory() as td:
        wp_dir = os.path.join(td, "sil_waypoints.d")
        os.makedirs(wp_dir)
        _write(os.path.join(wp_dir, "sil_waypoints_Town03.yaml"), "T3\n")
        agent, target = _wp_agent(td, map_dir=wp_dir, active="active.yaml")
        _write(target, "old\n")
        _boot(agent)                                   # 预置 boot 帧 map_id=Town03
        assert _read(target) == "T3\n"


def test_boot_missing_map_file_keeps_active_file() -> None:
    """该地图没生成过航点：只 WARN，沿用现有活动文件，链路不受影响。"""
    with tempfile.TemporaryDirectory() as td:
        wp_dir = os.path.join(td, "sil_waypoints.d")
        os.makedirs(wp_dir)
        _write(os.path.join(wp_dir, "sil_waypoints_Town05.yaml"), "T5\n")
        agent, target = _wp_agent(td, map_dir=wp_dir, active="active.yaml")
        _write(target, "old\n")
        _boot_map(agent, "Town03")
        assert _read(target) == "old\n"
        assert agent.session.active is True


def test_boot_without_map_id_keeps_active_file() -> None:
    with tempfile.TemporaryDirectory() as td:
        wp_dir = os.path.join(td, "sil_waypoints.d")
        os.makedirs(wp_dir)
        _write(os.path.join(wp_dir, "sil_waypoints_Town03.yaml"), "T3\n")
        agent, target = _wp_agent(td, map_dir=wp_dir, active="active.yaml")
        _write(target, "old\n")
        _boot_map(agent, "")
        assert _read(target) == "old\n"


def test_boot_waypoints_copy_failure_does_not_break_link() -> None:
    """文件级异常（目标不可写等）不得炸会话：后续数据流照常处理。"""
    with tempfile.TemporaryDirectory() as td:
        wp_dir = os.path.join(td, "sil_waypoints.d")
        os.makedirs(wp_dir)
        _write(os.path.join(wp_dir, "sil_waypoints_Town03.yaml"), "T3\n")
        blocker = os.path.join(td, "blocked")          # 目标是目录 → copyfile 必炸
        os.makedirs(blocker)
        agent = _agent(waypoints_map_dir=wp_dir,
                       waypoints_active_file=blocker)
        _boot(agent)
        assert agent.session.active is True


# ---------------------------------------------------------------------------
# 生命周期
# ---------------------------------------------------------------------------
def test_shutdown_flushes_zero_and_closes_idempotently() -> None:
    agent = _agent()
    _boot(agent)
    agent.on_cmd_vel(_twist(2.0, 0.0))
    agent.sensor_consumer = FakeConsumer()
    agent.event_producer = FakeProducer()
    agent.shutdown()
    frames = _frames(agent.control_producer)
    assert frames[-1]["linear_x"] == 0.0             # 退出前补零速，防自车滑行
    assert agent.control_producer.flushed == 1
    assert agent.event_producer.flushed == 1
    assert agent.sensor_consumer is None
    assert agent.ros_node.destroyed is True
    assert agent.running is False
    agent.shutdown()                                 # 幂等：不得重复补帧
    assert len(agent.control_producer.produced) == len(frames)


def test_shutdown_without_session_is_safe() -> None:
    agent = _agent()
    agent.shutdown()
    assert agent.control_producer.produced == []
    assert agent.ros_node.destroyed is True


def main() -> int:
    tests = [(n, f) for n, f in sorted(globals().items())
             if n.startswith("test_") and callable(f)]
    for name, fn in tests:
        fn()
        print("PASS %s" % name)
    print("NODE_OK: %d tests" % len(tests))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
