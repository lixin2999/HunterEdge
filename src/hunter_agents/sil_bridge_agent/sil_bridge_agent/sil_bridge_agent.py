#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""SIL Bridge Agent — HunterSim ↔ HunterEdge 真 SIL 桥接（V0.1.38）。

把 CARLA 仿真传感器经 Kafka 注入 HunterEdge **真实输入话题**，再把全栈自动驾驶
产出的 ``/cmd_vel`` 回传仿真，形成真 SIL 闭环（取代 HunterSim 侧的脚本自车）：

    CARLA 传感器 → EdgeLinkRuntime → Kafka carla.{sim_id}.edge_sensor
      → 本 Agent → /lidar_points /camera/... /imu/data /chassis/state /localization/odom
      → HunterEdge 全栈（perception / decision_making / auto_mission mapping cruise）
      → /cmd_vel → 本 Agent → Kafka carla.{sim_id}.edge_control
      → EdgeLinkRuntime → control_driver hook → 仿真自车

────────────────────────────────────────────────────────────────────────
设计要点
────────────────────────────────────────────────────────────────────────
1. **替代驱动而非旁路**：SIL 模式下不启动 rslidar_sdk / realsense2_camera /
   ch10x_driver / hunter_base(CAN) / fast_lio2 / EKF，本 Agent 独占这些节点的
   输出话题。话题名与 frame_id 全部取自 ``hunter_bringup/config/sensor_params.yaml``
   （``rslidar`` / ``camera_color_optical_frame`` / ``imu``），**不新增任何 ROS 接口**。
2. **真值定位替代 LIO**：``ego_state`` → ``/localization/odom`` + ``/Odometry``
   （后者是 auto_mission **mapping cruise 实际消费的巡航反馈源**，见
   ``auto_mission_node.cpp`` 的 ``lio_odom_sub_``），并接管 ``odom→base_link`` TF
   （原由 EKF ``publish_tf=true`` 发布）。
3. **坐标系翻转**：CARLA 左手系（Y 右）→ ROS 右手系（Y 左），位置/真向量 Y 取反、
   角速度/伪向量 ``(-x,+y,-z)``。详见 :mod:`sil_bridge_agent.codec`——这是
   HunterSim B2 ``steer = -atan(L·ω/v)/MAX`` 符号约定成立的前提。
4. **断流停发**：上行断流 > ``watchdog_ms``（默认 500ms）时**停发 ROS 帧**，让车端
   ``safety_guard`` 的 ``scan_timeout``/``cmd_timeout`` 超时停车语义自然生效；
   **不伪造零速点云**（伪造会让感知以为前方空旷）。控制回传不停，继续转发全栈
   产出（此时已是零速），仿真端由它自己的看门狗决定刹停。
5. **会话制**：正则订阅 ``^carla\..+\.edge_sensor$``，按 ``session`` boot 帧绑定
   唯一 sim_id；stop 帧后停发并**补恰好一帧零速**，避免仿真自车沿用末帧滑行。
6. **契约纪律**：``carla.*`` 是 SIL 专用通道，**不走 ``topic_name()`` 8 Topic 契约**
   （直接用 confluent_kafka + ``load_conf``，契约文件不受影响）；链路事件仍经现有
   ``hunter.<vid>.event`` 上报，且**只上报受控词表内类型**（``communication_loss``
   = critical），会话起止无契约类型 → 仅本地日志（与 remote_agent 同纪律，
   自造类型会被平台判非法进 DLQ）。

配置来源优先级：``--config`` YAML > ``/etc/hunter/sil_bridge_params.yaml`` > 内置默认值。
Kafka 凭据（Broker/SCRAM/mTLS 证书）**不写在 YAML**，统一来自
``/etc/hunter/kafka/kafka.properties``（HunterCore 接入包）。

⚠ 运行形态与 remote_agent 一致：**独立进程内自建 ROS 节点**（systemd 与 launch 均可），
不是 ``rclpy`` 声明式参数的节点，故 YAML 用 ``--config`` 读取而非 ``ros__parameters``。
"""
from __future__ import annotations

import argparse
import json
import logging
import math
import os
import shutil
import sys
import threading
import time
from typing import Any, Dict, Optional, cast

from .codec import (
    STREAM_DEPTH,
    STREAM_EGO_STATE,
    STREAM_IMU,
    STREAM_LIDAR,
    STREAM_RGB,
    STREAM_SESSION,
    LIDAR_POINT_BYTES,
    decode_depth,
    decode_ego_state,
    decode_envelope,
    decode_imu,
    decode_rgb,
    envelope_meta,
    envelope_payload,
    flip_lidar_payload,
)
from .session import SessionState, StreamRateLimiter

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    HAS_ROS = True
except ImportError:
    HAS_ROS = False

try:
    from sensor_msgs.msg import Image, Imu, PointCloud2, PointField
    from geometry_msgs.msg import Twist, TransformStamped
    from nav_msgs.msg import Odometry
    from hunter_msgs.msg import ChassisState
    HAS_MSGS = True
except ImportError:
    HAS_MSGS = False

try:
    from tf2_ros import TransformBroadcaster
    HAS_TF2 = True
except ImportError:
    HAS_TF2 = False

try:
    import yaml
    HAS_YAML = True
except ImportError:
    HAS_YAML = False

try:
    from confluent_kafka import Consumer, Producer
    HAS_KAFKA = True
except ImportError:
    HAS_KAFKA = False

try:
    from hunter_kafka.client import make_producer
    from hunter_kafka.config import (
        DEFAULT_BUNDLE_DIR,
        DEFAULT_PROPERTIES_PATH,
        KafkaConfigError,
        load_conf,
        load_properties,
        resolve_vehicle_id,
        strip_meta,
    )
    HAS_HUNTER_KAFKA = True
except ImportError:
    HAS_HUNTER_KAFKA = False
    DEFAULT_BUNDLE_DIR = "/etc/hunter/kafka"
    DEFAULT_PROPERTIES_PATH = "/etc/hunter/kafka/kafka.properties"

    class KafkaConfigError(RuntimeError):  # type: ignore[no-redef]
        """hunter_kafka 不可用时的占位异常（保持 except 分支可写）。"""

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("sil_bridge_agent")

#: 事件契约受控词表（``contracts/database/enums.md`` §3）。
#: **只上报本表内类型**：平台 data-collector 按「类型↔等级」一致性校验，
#: 等级写错或自造类型 = 非法事件（DLQ）。SIL 会话起止无契约类型 → 仅本地日志。
EVENT_LEVEL_BY_TYPE = {
    "communication_loss": "critical",   # SIL 传感器注入断流（链路不可用）
}

#: 上行正则订阅（librdkafka：以 ``^`` 开头即按正则匹配 Topic）。
SENSOR_TOPIC_REGEX = r"^carla\..+\.edge_sensor$"

#: 断流事件抑制期（秒）：与 data_agent ``event_min_interval`` 同口径，防事件风暴。
EVENT_MIN_INTERVAL_S = 10.0


DEFAULT_CONFIG: Dict[str, Any] = {
    "vehicle_id": "HUNTER-001",
    # Kafka 接入只保留路径（凭据与 mTLS 证书在 /etc/hunter/kafka/kafka.properties）
    "kafka_properties": DEFAULT_PROPERTIES_PATH,
    "kafka_bundle_dir": DEFAULT_BUNDLE_DIR,
    # ── 会话与看门狗 ──
    "sim_id_filter": "",              # 非空=只接受该 sim_id 的会话（多实例隔离）
    # ── 按地图自动装载航点（V0.2.0，配合 tools/gen_sil_waypoints.py）──
    # boot 帧 map_id → 把 dir 下 sil_waypoints_<map_id>.yaml 覆盖到 active 文件，
    # auto_mission 在 start_mapping_cruise 时热重载。**换地图后必须重新发起巡航**。
    # 内置默认**关闭**（裸默认不得碰 /etc 真实文件）；启用由参数 YAML 提供，
    # share/config 与 /etc/hunter 两份标准部署形态都已配好。
    "waypoints_map_dir": "",
    "waypoints_active_file": "",
    "watchdog_ms": 500,               # 传感器断流阈值，与仿真端/safety_guard 同口径
    "sensor_topic_regex": SENSOR_TOPIC_REGEX,
    "consumer_group": "hunter-sil-bridge",
    # ── ROS 话题映射（默认值取自 sensor_params.yaml，**不新增接口**）──
    "lidar_topic": "/lidar_points",
    "lidar_frame": "rslidar",
    "rgb_topic": "/camera/camera/color/image_raw",
    "depth_topic": "/camera/camera/depth/image_rect_raw",
    "camera_frame": "camera_color_optical_frame",
    "imu_topic": "/imu/data",
    "imu_frame": "imu",
    "chassis_topic": "/chassis/state",
    "odom_topic": "/localization/odom",
    # auto_mission mapping cruise 的巡航反馈源（原 FAST-LIO2 输出，SIL 由本 Agent 替代）
    "lio_odom_topic": "/Odometry",
    "cmd_vel_topic": "/cmd_vel",
    "base_frame": "base_link",
    "odom_frame": "odom",
    "map_frame": "map",
    # ── TF 接管（原 odom→base_link 由 EKF publish_tf=true 发布）──
    "publish_tf": True,
    "publish_map_tf": True,           # map→odom 恒等（SIL 真值即世界系，无全局重定位）
    # ── 各流 ROS 发布限频（保护 Jetson CPU；<=0 不限频）──
    "lidar_fps": 10.0,
    "rgb_fps": 10.0,
    "depth_fps": 10.0,
    "imu_fps": 50.0,
    "ego_state_fps": 20.0,
    # ── 底盘反馈占位（SIL 无电池/电机模型）──
    # ⚠ 置 0 会让 data_agent 持续误报 battery_low/battery_critical 事件风暴，
    #   故给满电标称值；这是**仿真占位**而非伪造实车测量，实车模式不启用本 Agent。
    "battery_voltage": 48.0,
    "battery_soc": 100.0,
    "battery_current": 0.0,
    "battery_temperature": 25.0,
    "wheelbase": 0.46,                # HUNTER-SE 轴距（与 safety_guard/auto_mission 一致）
    # ── 上报 ──
    "enable_event_report": True,
    "enable_health_report": False,    # data_agent 独占 health 1Hz，避免双发布者
    "health_interval_s": 1.0,
    "stats_log_interval_s": 10.0,
}


class SilBridgeAgent:
    """SIL 桥接 Agent（Kafka 收发线程 + ROS 节点）。"""

    def __init__(self, config: Dict[str, Any]) -> None:
        self.config = config
        self.running = False
        self.vehicle_id = str(config.get("vehicle_id") or "HUNTER-001")

        self.session = SessionState(
            watchdog_ms=int(config.get("watchdog_ms", 500)),
            sim_id_filter=str(config.get("sim_id_filter") or ""),
        )
        self.limiter = StreamRateLimiter({
            STREAM_LIDAR: float(config.get("lidar_fps", 10.0)),
            STREAM_RGB: float(config.get("rgb_fps", 10.0)),
            STREAM_DEPTH: float(config.get("depth_fps", 10.0)),
            STREAM_IMU: float(config.get("imu_fps", 50.0)),
            STREAM_EGO_STATE: float(config.get("ego_state_fps", 20.0)),
        })

        self.ros_node: Any = None
        self.spin_thread: Optional[threading.Thread] = None
        self.publishers: Dict[str, Any] = {}
        self.tf_broadcaster: Any = None
        self.sensor_consumer: Any = None
        self.control_producer: Any = None
        self.event_producer: Any = None
        self.health_producer: Any = None

        self._lock = threading.RLock()
        self._last_ego: Optional[Dict[str, Any]] = None
        self._last_cmd = (0.0, 0.0)
        self._last_event_ts: Dict[str, float] = {}
        self._send_errors = 0
        self._publish_errors = 0
        self._stale_logged = False

    # ------------------------------------------------------------------
    # ROS2
    # ------------------------------------------------------------------
    def init_ros(self) -> bool:
        """建节点、发布器、TF 广播器与 /cmd_vel 订阅（缺依赖时降级并明确报错）。"""
        if not HAS_ROS or not HAS_MSGS:
            logger.error(
                "rclpy/ROS 消息不可用（HAS_ROS=%s HAS_MSGS=%s）——SIL 桥接无法注入话题。"
                "处置：source /opt/ros/humble/setup.bash 与工作空间 install/setup.bash，"
                "并确认 hunter_msgs 已编译", HAS_ROS, HAS_MSGS)
            return False
        rclpy.init(args=[])
        self.ros_node = Node("sil_bridge_agent")
        # 传感器用 SensorDataQoS 语义（best_effort + keep_last），与真实驱动一致：
        # 下游 lidar_perception / pointcloud_to_laserscan 按此 QoS 订阅，
        # 用 reliable 会因 QoS 不兼容而**静默收不到**（ROS2 常见坑）。
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        cfg = self.config
        self.publishers[STREAM_LIDAR] = self.ros_node.create_publisher(
            PointCloud2, cfg["lidar_topic"], sensor_qos)
        self.publishers[STREAM_RGB] = self.ros_node.create_publisher(
            Image, cfg["rgb_topic"], sensor_qos)
        self.publishers[STREAM_DEPTH] = self.ros_node.create_publisher(
            Image, cfg["depth_topic"], sensor_qos)
        self.publishers[STREAM_IMU] = self.ros_node.create_publisher(
            Imu, cfg["imu_topic"], sensor_qos)
        self.publishers["chassis"] = self.ros_node.create_publisher(
            ChassisState, cfg["chassis_topic"], state_qos)
        self.publishers["odom"] = self.ros_node.create_publisher(
            Odometry, cfg["odom_topic"], state_qos)
        self.publishers["lio_odom"] = self.ros_node.create_publisher(
            Odometry, cfg["lio_odom_topic"], sensor_qos)

        if HAS_TF2 and cfg.get("publish_tf", True):
            self.tf_broadcaster = TransformBroadcaster(self.ros_node)
        elif cfg.get("publish_tf", True):
            logger.warning("tf2_ros 不可用，odom→base_link TF 无法发布；"
                           "Nav2/costmap 会因缺 TF 丢帧（处置：确认 tf2_ros 已安装）")

        # /cmd_vel 由 safety_guard（nav 模式）或 auto_mission cruise（mapping 模式）发布，
        # 本 Agent 是**唯一订阅者→Kafka 回传**，不参与控制决策（真 SIL 的被测系统是全栈）。
        self.ros_node.create_subscription(
            Twist, cfg["cmd_vel_topic"], self.on_cmd_vel, 10)

        self.spin_thread = threading.Thread(target=self.ros_spin, daemon=True)
        self.spin_thread.start()
        logger.info(
            "ROS2 桥接就绪：lidar=%s rgb=%s depth=%s imu=%s chassis=%s odom=%s(+%s) cmd_vel=%s",
            cfg["lidar_topic"], cfg["rgb_topic"], cfg["depth_topic"], cfg["imu_topic"],
            cfg["chassis_topic"], cfg["odom_topic"], cfg["lio_odom_topic"], cfg["cmd_vel_topic"])
        return True

    def ros_spin(self) -> None:
        """MultiThreadedExecutor：避免点云/图像回调阻塞 /cmd_vel 订阅（remote_agent 同教训）。"""
        try:
            from rclpy.executors import MultiThreadedExecutor
            executor = MultiThreadedExecutor(num_threads=4)
            executor.add_node(self.ros_node)
            executor.spin()
        except Exception as exc:  # noqa: BLE001
            logger.error("ROS2 spin 退出：%s", exc)

    def _stamp(self) -> Any:
        """ROS 时间戳。

        ⚠ 用**墙钟**而非 sim_ts：CARLA 仿真经过秒不是 Unix epoch，直接当 stamp
        会让下游 TF/消息滤波器判为「远古消息」全量丢弃（HunterSim A1 同根因）。
        仿真时刻另经 ``wall_ts_ns``/``sim_ts`` 在链路诊断里透出。
        """
        return self.ros_node.get_clock().now().to_msg()

    # ------------------------------------------------------------------
    # Kafka
    # ------------------------------------------------------------------
    def init_kafka(self) -> bool:
        """建传感器消费者（正则订阅）+ 控制生产者 + 事件生产者。

        ``carla.*`` 直接用 confluent_kafka（**不走 topic_name() 契约**）；
        ``hunter.<vid>.event`` 走 make_producer（现有契约，acks=all）。
        """
        if not HAS_KAFKA or not HAS_HUNTER_KAFKA:
            logger.error(
                "confluent_kafka/hunter_kafka 不可用（HAS_KAFKA=%s HAS_HUNTER_KAFKA=%s）"
                "——SIL 链路无法建立。处置：pip3 install confluent-kafka 并 source 工作空间",
                HAS_KAFKA, HAS_HUNTER_KAFKA)
            return False
        cfg = self.config
        props_path = cfg.get("kafka_properties", DEFAULT_PROPERTIES_PATH)
        bundle_dir = cfg.get("kafka_bundle_dir", DEFAULT_BUNDLE_DIR)
        try:
            # 传感器消费：latest 偏移（不回放历史会话）+ 自动提交
            # （传感器是易逝数据，无业务续点需求；与指令消费的手动提交语义不同）
            sensor_conf = load_conf(
                properties_path=props_path, bundle_dir=bundle_dir,
                client_id=f"{self.vehicle_id}-sil-sensor", role="consumer",
                extra={
                    "group.id": f"{cfg['consumer_group']}-{self.vehicle_id}",
                    "enable.auto.commit": "true",
                    "auto.offset.reset": "latest",
                    "session.timeout.ms": "15000",
                    "max.poll.interval.ms": "30000",
                })
            self.sensor_consumer = Consumer(strip_meta(sensor_conf))
            self.sensor_consumer.subscribe([cfg["sensor_topic_regex"]])

            # 控制回传：acks=1（高频低延迟，与仿真端 edge_sensor 同档）
            control_conf = load_conf(
                properties_path=props_path, bundle_dir=bundle_dir,
                client_id=f"{self.vehicle_id}-sil-control", role="producer", acks="1",
                extra={"linger.ms": "5", "batch.size": "16384"})
            self.control_producer = Producer(strip_meta(control_conf))
        except KafkaConfigError as exc:
            logger.error("SIL Kafka 接入失败（启动即失败，不静默降级）：%s", exc)
            return False
        except Exception as exc:  # noqa: BLE001
            logger.error("SIL Kafka 客户端构建失败：%s: %s", type(exc).__name__, exc)
            return False

        if cfg.get("enable_event_report", True):
            try:
                self.event_producer = make_producer(
                    self.vehicle_id, "event",
                    properties_path=props_path, bundle_dir=bundle_dir)
            except Exception as exc:  # noqa: BLE001  事件通道失败不阻断 SIL 主链路
                logger.warning("事件 producer 构建失败（链路事件仅本地留痕）：%s", exc)
        if cfg.get("enable_health_report", False):
            try:
                self.health_producer = make_producer(
                    self.vehicle_id, "health",
                    properties_path=props_path, bundle_dir=bundle_dir)
            except Exception as exc:  # noqa: BLE001
                logger.warning("health producer 构建失败：%s", exc)

        logger.info("SIL Kafka 就绪：订阅 %s（正则），控制回传 carla.{sim_id}.edge_control",
                    cfg["sensor_topic_regex"])
        threading.Thread(target=self.kafka_loop, name="sil-sensor", daemon=True).start()
        threading.Thread(target=self.producer_loop, name="sil-producer", daemon=True).start()
        threading.Thread(target=self.watchdog_loop, name="sil-watchdog", daemon=True).start()
        return True

    def kafka_loop(self) -> None:
        """传感器消费主循环（独立线程）。"""
        while self.running:
            consumer = self.sensor_consumer
            if consumer is None:
                time.sleep(0.2)
                continue
            try:
                msg = consumer.poll(0.2)
            except Exception as exc:  # noqa: BLE001  轮询失败退避重试，不退出循环
                logger.error("SIL 传感器轮询异常：%s", exc)
                time.sleep(1.0)
                continue
            if msg is None:
                continue
            if msg.error():
                logger.error("SIL 传感器消费错误：%s", msg.error())
                continue
            try:
                self.on_message(msg.value(), msg.topic())
            except Exception as exc:  # noqa: BLE001  单帧异常不得中断消费
                logger.warning("SIL 帧处理异常（topic=%s）：%s", msg.topic(), exc)

    def producer_loop(self) -> None:
        """生产者投递证实轮询（confluent_kafka 回调只在 poll/flush 时触发）。"""
        while self.running:
            for producer in (self.control_producer, self.event_producer, self.health_producer):
                if producer is None:
                    continue
                try:
                    producer.poll(0.0)
                except Exception as exc:  # noqa: BLE001
                    logger.debug("producer poll 异常：%s", exc)
            time.sleep(0.05)

    def watchdog_loop(self) -> None:
        """低频看门狗 + 诊断日志 + 事件上报（与热路径判定分离，避免刷屏）。"""
        cfg = self.config
        stats_interval = float(cfg.get("stats_log_interval_s", 10.0))
        health_interval = float(cfg.get("health_interval_s", 1.0))
        last_stats = 0.0
        last_health = 0.0
        while self.running:
            now = time.time()
            if self.session.poll_stale(now):
                stats = self.session.stats(now)
                logger.warning(
                    "SIL 传感器注入断流（age=%sms > %dms）：停发 ROS 帧，"
                    "由车端 safety_guard 超时停车语义接管；sim_id=%s",
                    stats["sensor_age_ms"], stats["watchdog_ms"], stats["sim_id"])
                self.report_event("communication_loss", {
                    "link": "sil_edge_sensor",
                    "sim_id": stats["sim_id"],
                    "sensor_age_ms": stats["sensor_age_ms"],
                    "watchdog_ms": stats["watchdog_ms"],
                })
            if now - last_stats >= stats_interval:
                last_stats = now
                stats = self.session.stats(now)
                if stats["active"]:
                    logger.info(
                        "SIL 链路：sim_id=%s scene=%s fresh=%s 传感器=%s 控制=%d 限频丢弃=%s",
                        stats["sim_id"], stats["scene_id"], stats["sensor_fresh"],
                        stats["sensor_frames"], stats["control_frames"], self.limiter.dropped)
            if self.health_producer is not None and now - last_health >= health_interval:
                last_health = now
                self.report_health(now)
            time.sleep(0.1)

    # ------------------------------------------------------------------
    # 帧处理
    # ------------------------------------------------------------------
    def _apply_map_waypoints(self, map_id: str) -> None:
        """boot 时按仿真场景 CARLA 地图装载航点文件（V0.2.0）。

        文件命名契约与 tools/gen_sil_waypoints.py 产物一致：
        ``<dir>/sil_waypoints_<map_id>.yaml``。拷贝只发生在 boot 帧，
        auto_mission 只在 start_mapping_cruise 热重载——因此**换地图后需
        重新发起巡航**才装载新航点。文件级操作不得影响链路：全程 try 保护，
        失败仅 WARN；未配置 dir/active 时关闭本机制（内置裸默认即关闭）。
        """
        wp_dir = str(self.config.get("waypoints_map_dir") or "")
        target = str(self.config.get("waypoints_active_file") or "")
        if not wp_dir or not target:
            return
        if not map_id:
            logger.warning("boot 帧无 map_id，沿用现航点文件：%s", target)
            return
        source = os.path.join(wp_dir, "sil_waypoints_%s.yaml" % map_id)
        if not os.path.isfile(source):
            logger.warning(
                "地图 %s 无航点文件：%s（用 gen_sil_waypoints.py 生成并部署到 %s；"
                "沿用现有活动文件 %s）", map_id, source, wp_dir, target)
            return
        try:
            os.makedirs(os.path.dirname(target), exist_ok=True)
            shutil.copyfile(source, target)
            logger.info("地图 %s 航点已装载：%s → %s（下次 start_mapping_cruise 热重载生效）",
                        map_id, source, target)
        except OSError as exc:
            logger.warning("航点文件装载失败（不影响链路）：%s", exc)

    def on_message(self, raw: Any, topic: str) -> None:
        """处理一条上行信封（会话帧先于准入判定，迟到帧也必须按序执行）。"""
        envelope = decode_envelope(raw)
        if envelope is None:
            self.session.reject()
            return
        # decode_envelope 已保证 stream 为非空 str（非法信封在那里即返回 None
        # 并被 reject），此处 cast 只向类型检查器表达该不变量，不新增运行时判定。
        stream = cast(str, envelope.get("stream"))
        if stream == STREAM_SESSION:
            envelope["_topic"] = topic
            phase = self.session.handle_session(envelope)
            if phase == "boot":
                stats = self.session.stats()
                logger.info("SIL 会话开始：sim_id=%s scene=%s map=%s watchdog=%dms",
                            stats["sim_id"], stats["scene_id"], stats["map_id"],
                            stats["watchdog_ms"])
                self._last_ego = None
                self._apply_map_waypoints(stats["map_id"])
            elif phase == "stop":
                logger.info("SIL 会话结束：sim_id=%s（补一帧零速后停发）", self.session.sim_id)
                self._flush_stop_zero()
            return
        if not self.session.accepts(topic):
            self.session.reject()
            return
        payload = envelope_payload(envelope)
        meta = envelope_meta(envelope)
        if not payload:
            return
        now = time.time()
        if not self.limiter.allow(stream, now):
            return
        # 刷新链路时间戳（看门狗基准）。发布是**帧驱动**的：上行断流时本方法
        # 根本不被调用，ROS 帧自然停发 → 车端 safety_guard 的 scan_timeout/
        # cmd_timeout 超时停车语义生效，无需（也不得）伪造零速点云填充。
        # 断流的检测/告警/事件上报由 watchdog_loop 低频负责。
        self.session.note_sensor(stream, now)
        try:
            if stream == STREAM_LIDAR:
                self._publish_lidar(payload, meta)
            elif stream == STREAM_RGB:
                self._publish_rgb(payload, meta)
            elif stream == STREAM_DEPTH:
                self._publish_depth(payload, meta)
            elif stream == STREAM_IMU:
                self._publish_imu(payload, meta)
            elif stream == STREAM_EGO_STATE:
                self._publish_ego_state(payload, meta)
        except Exception as exc:  # noqa: BLE001  单流发布失败不阻断其它流
            self._publish_errors += 1
            logger.warning("SIL 发布失败（stream=%s）：%s", stream, exc)

    def _header(self, frame_id: str) -> Any:
        from std_msgs.msg import Header
        header = Header()
        header.stamp = self._stamp()
        header.frame_id = frame_id
        return header

    def _publish_lidar(self, payload: bytes, meta: Dict[str, Any]) -> None:
        """xyzI float32 → PointCloud2（Y 取反到 ROS 系）。"""
        point_bytes = int(meta.get("point_bytes") or LIDAR_POINT_BYTES)
        data = flip_lidar_payload(payload, point_bytes)
        if not data or len(data) % LIDAR_POINT_BYTES:
            return
        count = len(data) // LIDAR_POINT_BYTES
        msg = PointCloud2()
        msg.header = self._header(self.config["lidar_frame"])
        msg.height = 1
        msg.width = count
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = LIDAR_POINT_BYTES
        msg.row_step = LIDAR_POINT_BYTES * count
        msg.data = data
        msg.is_dense = True
        self.publishers[STREAM_LIDAR].publish(msg)

    def _publish_rgb(self, payload: bytes, meta: Dict[str, Any]) -> None:
        """JPEG/PNG → rgb8 Image（编码产物解码，非裸 BGRA）。"""
        decoded = decode_rgb(payload, meta)
        if decoded is None:
            return
        self.publishers[STREAM_RGB].publish(self._image_msg(decoded, self.config["camera_frame"]))

    def _publish_depth(self, payload: bytes, meta: Dict[str, Any]) -> None:
        """32FC1 米制深度 → Image（尺寸校验不过则丢帧，不产出错位深度）。"""
        decoded = decode_depth(payload, meta)
        if decoded is None:
            logger.warning("SIL 深度帧尺寸不符（width=%s height=%s bytes=%d），丢弃",
                           meta.get("width"), meta.get("height"), len(payload))
            return
        self.publishers[STREAM_DEPTH].publish(
            self._image_msg(decoded, self.config["camera_frame"]))

    def _image_msg(self, decoded: Dict[str, Any], frame_id: str) -> Any:
        msg = Image()
        msg.header = self._header(frame_id)
        msg.height = int(decoded["height"])
        msg.width = int(decoded["width"])
        msg.encoding = str(decoded["encoding"])
        msg.is_bigendian = 0
        msg.step = int(decoded["step"])
        msg.data = decoded["data"]
        return msg

    def _publish_imu(self, payload: bytes, meta: Dict[str, Any]) -> None:
        """IMU JSON → Imu（加速度真向量 / 陀螺伪向量分别翻系）。"""
        decoded = decode_imu(payload)
        if decoded is None:
            return
        msg = Imu()
        msg.header = self._header(self.config["imu_frame"])
        # 姿态由 ego_state 提供（IMU 流无姿态解算）；有缓存则带上，避免下游
        # 拿到全零四元数（非法姿态）而报错。
        ego = self._last_ego
        if ego is not None:
            qx, qy, qz, qw = ego["orientation"]
            msg.orientation.x, msg.orientation.y = qx, qy
            msg.orientation.z, msg.orientation.w = qz, qw
        else:
            msg.orientation.w = 1.0
        msg.orientation_covariance = decoded["orientation_covariance"]
        ax, ay, az = decoded["accel"]
        msg.linear_acceleration.x, msg.linear_acceleration.y = ax, ay
        msg.linear_acceleration.z = az
        gx, gy, gz = decoded["gyro"]
        msg.angular_velocity.x, msg.angular_velocity.y = gx, gy
        msg.angular_velocity.z = gz
        self.publishers[STREAM_IMU].publish(msg)

    def _publish_ego_state(self, payload: bytes, meta: Dict[str, Any]) -> None:
        """自车真值 → ChassisState + Odometry（×2）+ odom→base_link TF。"""
        decoded = decode_ego_state(payload)
        if decoded is None:
            return
        with self._lock:
            self._last_ego = decoded
        stamp = self._stamp()
        self._publish_chassis(decoded, stamp)
        odom = self._odom_msg(decoded, stamp)
        self.publishers["odom"].publish(odom)
        # mapping cruise 的巡航反馈源（原 FAST-LIO2 /Odometry）：同一份真值，
        # 保证 auto_mission 的航点反馈与 Nav2 定位严格同源，无 EKF 折算偏差。
        self.publishers["lio_odom"].publish(odom)
        self._publish_tf(decoded, stamp)

    def _publish_chassis(self, ego: Dict[str, Any], stamp: Any) -> None:
        cfg = self.config
        msg = ChassisState()
        msg.header.stamp = stamp
        msg.header.frame_id = cfg["base_frame"]
        msg.velocity = float(ego["speed_ms"])
        # SIL 无真实底盘转角反馈：按末帧 /cmd_vel 经阿克曼反解等效转角
        # δ = atan(L·ω / v)（与 auto_mission_node.cpp 的 ω = v·tan(δ)/L 互逆）。
        linear_x, angular_z = self._last_cmd
        speed = max(abs(linear_x), 0.5)
        msg.steering_angle = float(math.atan(cfg["wheelbase"] * angular_z / speed))
        msg.battery_voltage = float(cfg["battery_voltage"])
        msg.battery_soc = float(cfg["battery_soc"])
        msg.battery_current = float(cfg["battery_current"])
        msg.battery_temperature = float(cfg["battery_temperature"])
        msg.control_mode = "AUTO"
        msg.vehicle_state = "NORMAL"
        msg.fault_code = 0
        msg.motor_temperature = []
        self.publishers["chassis"].publish(msg)

    def _odom_msg(self, ego: Dict[str, Any], stamp: Any) -> Any:
        cfg = self.config
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = cfg["odom_frame"]
        msg.child_frame_id = cfg["base_frame"]
        px, py, pz = ego["position"]
        msg.pose.pose.position.x, msg.pose.pose.position.y = px, py
        msg.pose.pose.position.z = pz
        qx, qy, qz, qw = ego["orientation"]
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y = qx, qy
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = qz, qw
        vx, vy, vz = ego["linear_velocity"]
        msg.twist.twist.linear.x, msg.twist.twist.linear.y = vx, vy
        msg.twist.twist.linear.z = vz
        wx, wy, wz = ego["angular_velocity"]
        msg.twist.twist.angular.x, msg.twist.twist.angular.y = wx, wy
        msg.twist.twist.angular.z = wz
        # 仿真真值：位姿/速度协方差置 0 表示「精确已知」，让下游 EKF/NDT
        # 按真值处理（SIL 本就是要排除定位误差这一变量）。
        return msg

    def _publish_tf(self, ego: Dict[str, Any], stamp: Any) -> None:
        """接管 odom→base_link（原 EKF 发布）与 map→odom 恒等（原全局定位发布）。"""
        if self.tf_broadcaster is None:
            return
        cfg = self.config
        px, py, pz = ego["position"]
        qx, qy, qz, qw = ego["orientation"]
        transforms = []
        if cfg.get("publish_tf", True):
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = cfg["odom_frame"]
            tf.child_frame_id = cfg["base_frame"]
            tf.transform.translation.x, tf.transform.translation.y = px, py
            tf.transform.translation.z = pz
            tf.transform.rotation.x, tf.transform.rotation.y = qx, qy
            tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
            transforms.append(tf)
        if cfg.get("publish_map_tf", True):
            # SIL 无全局重定位：真值世界系即 map 系，map→odom 恒等。
            # 缺这条 TF 会让 Nav2 的 map→base_link 查询链断裂。
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = cfg["map_frame"]
            tf.child_frame_id = cfg["odom_frame"]
            tf.transform.rotation.w = 1.0
            transforms.append(tf)
        if transforms:
            self.tf_broadcaster.sendTransform(transforms)

    # ------------------------------------------------------------------
    # 控制回传
    # ------------------------------------------------------------------
    def on_cmd_vel(self, msg: Any) -> None:
        """/cmd_vel → Kafka carla.{sim_id}.edge_control（Twist 语义原样透传）。

        **不做限幅/平滑**：真 SIL 的被测对象就是 HunterEdge 全栈，桥接层若再加工
        指令，评估裁决的就不是全栈真实表现（safety_guard 已负责安全闸）。
        """
        linear_x = float(getattr(msg.linear, "x", 0.0))
        angular_z = float(getattr(msg.angular, "z", 0.0))
        with self._lock:
            self._last_cmd = (linear_x, angular_z)
        self._send_control(linear_x, angular_z)

    def _send_control(self, linear_x: float, angular_z: float) -> bool:
        frame = self.session.pop_control(linear_x, angular_z)
        if frame is None:
            return False
        return self._produce_control(frame)

    def _flush_stop_zero(self) -> None:
        """会话 stop 收尾：补恰好一帧零速（pop_control 内部强制零并消耗待办）。"""
        if not self.session.control_pending():
            return
        self._send_control(0.0, 0.0)

    def _produce_control(self, frame: Dict[str, Any]) -> bool:
        producer = self.control_producer
        topic = self.session.control_topic
        if producer is None or not topic:
            return False
        try:
            producer.produce(
                topic,
                value=json.dumps(frame, ensure_ascii=False).encode("utf-8"),
                key=self.session.sim_id.encode("utf-8"),
                on_delivery=self._on_control_delivery)
        except Exception as exc:  # noqa: BLE001  发送失败计数降级，不冒泡阻断 ROS 回调
            self._send_errors += 1
            logger.warning("SIL 控制回传失败（topic=%s）：%s", topic, exc)
            return False
        return True

    def _on_control_delivery(self, err: Any, msg: Any) -> None:
        """投递证实回调（confluent_kafka 2.16 唯一可用入口是 produce(on_delivery=...)）。"""
        if err is not None:
            self._send_errors += 1
            logger.warning("SIL 控制帧投递失败：%s", err)

    # ------------------------------------------------------------------
    # 平台上报（现有契约通道）
    # ------------------------------------------------------------------
    def report_event(self, event_type: str, extra: Optional[Dict[str, Any]] = None) -> None:
        """链路事件 → ``hunter.<vid>.event``（只上报受控词表内类型 + 抑制期防风暴）。"""
        level = EVENT_LEVEL_BY_TYPE.get(event_type)
        if level is None:
            logger.info("SIL 事件（契约无此类型，仅本地留痕）：%s %s", event_type, extra or {})
            return
        now = time.time()
        last = self._last_event_ts.get(event_type, 0.0)
        if now - last < EVENT_MIN_INTERVAL_S:
            return
        self._last_event_ts[event_type] = now
        payload = {
            "vehicle_id": self.vehicle_id,
            "timestamp": int(now),
            "event_type": event_type,
            "event_level": level,
            "description": f"SIL 桥接：{event_type}",
        }
        if extra:
            payload["data"] = extra
        logger.info("SIL 事件上报：%s", json.dumps(payload, ensure_ascii=False))
        if self.event_producer is None:
            return
        try:
            from hunter_kafka.client import topic_name
            self.event_producer.produce(
                topic_name(self.vehicle_id, "event"),
                value=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
                key=self.vehicle_id.encode("utf-8"))
        except Exception as exc:  # noqa: BLE001
            logger.error("SIL 事件投递失败：%s", exc)

    def report_health(self, now: Optional[float] = None) -> None:
        """链路健康 → ``hunter.<vid>.health``（默认关闭：data_agent 独占该通道）。"""
        if self.health_producer is None:
            return
        now = time.time() if now is None else now
        stats = self.session.stats(now)
        payload = {
            "vehicle_id": self.vehicle_id,
            "timestamp": int(now),
            "status": "auto_driving" if stats["sensor_fresh"] else "fault",
            "system": {
                "sil_link": {
                    "active": stats["active"],
                    "sim_id": stats["sim_id"],
                    "sensor_age_ms": stats["sensor_age_ms"],
                    "sensor_frames": stats["sensor_frames"],
                    "control_frames": stats["control_frames"],
                    "stale_episodes": stats["stale_episodes"],
                    "send_errors": self._send_errors,
                    "publish_errors": self._publish_errors,
                },
            },
        }
        try:
            from hunter_kafka.client import topic_name
            self.health_producer.produce(
                topic_name(self.vehicle_id, "health"),
                value=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
                key=self.vehicle_id.encode("utf-8"))
        except Exception as exc:  # noqa: BLE001
            logger.error("SIL health 投递失败：%s", exc)

    def resolve_vehicle_id(self) -> None:
        """vehicle_id 以接入包为准（SASL 用户名 == 证书 CN == vehicle_id）。"""
        if not HAS_HUNTER_KAFKA:
            return
        path = self.config.get("kafka_properties", DEFAULT_PROPERTIES_PATH)
        if not os.path.isfile(path):
            return
        try:
            props = load_properties(path)
        except KafkaConfigError as exc:
            logger.warning("kafka.properties 读取失败：%s", exc)
            return
        props_vid = resolve_vehicle_id(props)
        if props_vid and props_vid != self.vehicle_id:
            logger.warning("vehicle_id 冲突：配置=%s 接入包=%s；以接入包为准",
                           self.vehicle_id, props_vid)
            self.vehicle_id = props_vid

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------
    def run(self) -> None:
        """启动主循环（阻塞；SIGINT 走 finally 收拾干净，与 remote_agent 同口径）。"""
        self.resolve_vehicle_id()
        if not self.init_ros():
            raise SystemExit(1)
        if not self.init_kafka():
            self.shutdown()
            raise SystemExit(1)
        self.running = True
        logger.info("SIL Bridge Agent 启动：vehicle_id=%s watchdog=%dms（等待仿真 session boot 帧）",
                    self.vehicle_id, self.config.get("watchdog_ms", 500))
        try:
            while self.running:
                time.sleep(0.5)
        except KeyboardInterrupt:
            logger.info("收到 SIGINT，正在关闭 SIL 桥接")
        finally:
            self.running = False
            self.shutdown()

    def shutdown(self) -> None:
        """幂等收尾：补零速 → flush → 关消费者 → 销毁 ROS 节点。"""
        self.running = False
        # 进程退出等价于会话结束：即使没收到 stop 帧（SIGINT / systemd 重启）
        # 也补一帧零速，避免仿真自车沿用末帧指令滑行到看门狗超时。
        self.session.close()
        try:
            self._flush_stop_zero()
        except Exception as exc:  # noqa: BLE001
            logger.debug("stop 零速收尾异常：%s", exc)
        for producer in (self.control_producer, self.event_producer, self.health_producer):
            if producer is None:
                continue
            try:
                producer.flush(3.0)
            except Exception as exc:  # noqa: BLE001
                logger.warning("producer flush 异常：%s", exc)
        if self.sensor_consumer is not None:
            try:
                self.sensor_consumer.close()
            except Exception as exc:  # noqa: BLE001
                logger.warning("consumer 关闭异常：%s", exc)
            self.sensor_consumer = None
        try:
            if self.ros_node is not None and HAS_ROS and rclpy.ok():
                self.ros_node.destroy_node()
                rclpy.shutdown()
        except Exception as exc:  # noqa: BLE001
            logger.warning("ROS2 关闭异常：%s", exc)
        logger.info("SIL Bridge Agent 已关闭：%s", json.dumps(self.session.stats(), ensure_ascii=False))


def load_config(path: Optional[str]) -> Dict[str, Any]:
    """YAML 覆盖内置默认值；支持 ``sil_bridge_agent: ros__parameters:`` 或扁平结构。"""
    cfg = dict(DEFAULT_CONFIG)
    cfg["kafka_properties"] = os.environ.get("HUNTER_KAFKA_PROPERTIES", cfg["kafka_properties"])
    if path and os.path.isfile(path) and HAS_YAML:
        with open(path, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh) or {}
        section = data.get("sil_bridge_agent", data)
        cfg.update({k: v for k, v in section.get("ros__parameters", section).items()})
        logger.info("已加载配置：%s", path)
    elif path:
        logger.warning("配置文件不可用（缺文件或缺 PyYAML），使用内置默认值：%s", path)
    return cfg


def strip_ros_args(argv: list) -> list:
    """剔除 ros2 launch 注入的 ROS 参数，只留下本节点 argparse 认识的参数。

    ``launch_ros.actions.Node(name=...)`` 会无条件在命令行末尾追加
    ``--ros-args -r __node:=sil_bridge_agent``；本节点不是 rclpy 声明式参数节点，
    argparse 遇到未知参数会直接 ``exit code 2``，launch 里表现为
    ``sil_bridge_node: error: unrecognized arguments: --ros-args ...``，
    随后整条 SIL 链路无人消费 ``edge_sensor``，面板显示"SIL 等待车端"。
    """
    if not argv:
        return []
    if HAS_ROS:
        try:
            from rclpy.utilities import remove_ros_args
            return list(remove_ros_args(args=argv)[1:])
        except Exception:  # noqa: BLE001 —— 老版本 rclpy 无该函数时走手写兜底
            logger.debug("remove_ros_args 不可用，改用截断兜底")
    # 兜底：launch 只会把 ROS 参数追加在尾部，遇到 `--ros-args` 即截断；
    # 同时丢掉 ROS 段结束后可能出现的分隔符 `--`。
    cleaned = []
    for token in argv:
        if token == "--ros-args":
            break
        if token == "--":
            continue
        cleaned.append(token)
    return cleaned


def main(argv: Optional[list] = None) -> None:
    parser = argparse.ArgumentParser(description="HUNTER SIL Bridge Agent")
    parser.add_argument(
        "--config",
        default=os.environ.get("HUNTER_SIL_CONFIG", "/etc/hunter/sil_bridge_params.yaml"),
        help="YAML 参数文件（不含任何 Kafka 凭据）")
    args = parser.parse_args(strip_ros_args(sys.argv if argv is None else argv))
    agent = SilBridgeAgent(load_config(args.config))
    # systemd KillSignal=SIGINT → Python 默认抛 KeyboardInterrupt → run() 的 finally
    # 执行 shutdown()（补零速 + flush + 关节点）收拾干净再退出。
    # ⚠ 不用 SIGTERM：默认 SIGTERM 会直接终止进程、不走 finally（remote_agent 实机教训）。
    agent.run()


if __name__ == "__main__":
    main()
