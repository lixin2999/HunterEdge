#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Remote Agent — 远程操控车载端（文档第 13 章，systemd 服务，非 ROS 节点）。

功能：
  1. GStreamer AGX 硬件 H264 编码 D435 视频 → WebRTC/SRS 传输；
  2. 遥控指令双通道：WebSocket（低带宽临时联调）+ **Kafka hunter.<vid>.remote_control**
     （HunterCore 接入包契约通道，与平台其它业务同源同证书）；
  3. 指令限幅、500ms 超时自动停车并**交还驾驶权**、断线重连；
  4. 遥控会话事件经 ``hunter.<vid>.event`` 上报（接入包 Topic 契约不含 remote_status，
     旧版自发该 Topic 已废弃）。

⚠ 关键修复（旧版缺陷）：指令超时后不得继续 20Hz 发布 /remote/command。
  decision_making 判定 REMOTE 是否生效只看该话题是否在 remote_timeout(0.5s) 内到过数据
  （src/decision_making/src/decision_making.cpp:122），因此旧版"超时后一直发零速帧"
  会让 REMOTE 模式永不失效、AUTO 无法恢复。现在超时后：先补一帧刹车停车，
  随即停止发布 → 决策层判 remote_stale → 回 AUTO。

配置来源优先级：命令行 --config 指定的 YAML > /etc/hunter/remote_agent_params.yaml >
内置默认值。Kafka 接入参数（Broker/SCRAM/mTLS 证书）不写在 YAML，统一来自
/etc/hunter/kafka/kafka.properties（HunterCore 接入包：目录 0750 / properties 0640 / 私钥 0600）。
"""
import argparse
import json
import logging
import os
import subprocess
import threading
import time

try:
    import websocket  # websocket-client
    HAS_WS = True
except ImportError:
    HAS_WS = False

try:
    import rclpy
    from rclpy.node import Node
    from hunter_msgs.msg import ChassisCommand
    HAS_ROS = True
except ImportError:
    HAS_ROS = False

try:
    import yaml
    HAS_YAML = True
except ImportError:
    HAS_YAML = False

try:
    from hunter_kafka.client import make_consumer, make_producer, topic_name
    from hunter_kafka.config import (DEFAULT_BUNDLE_DIR, DEFAULT_PROPERTIES_PATH,
                                     KafkaConfigError, load_properties, resolve_vehicle_id)
    HAS_HUNTER_KAFKA = True
except ImportError:
    HAS_HUNTER_KAFKA = False
    DEFAULT_BUNDLE_DIR = "/etc/hunter/kafka"
    DEFAULT_PROPERTIES_PATH = "/etc/hunter/kafka/kafka.properties"

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("remote_agent")


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class RemoteAgent:
    def __init__(self, config):
        self.config = config
        self.gst_proc = None
        self.ws = None
        self.running = False
        self.last_cmd_time = 0.0
        self.ros_node = None
        self.cmd_publisher = None
        self.kafka_producer = None
        self.kafka_consumer = None

        self.vehicle_id = config["vehicle_id"]
        self.ws_url = config["ws_url"]           # wss://platform/ws/remote
        self.cmd_topic = config.get("cmd_topic", "/remote/command")

        # 遥控指令缓存（由 WS/Kafka 任一通道写入，20Hz 定时器读出并发布）
        self.latest_velocity = 0.0
        self.latest_steering = 0.0
        self.latest_brake = False
        self.cmd_source = ""                     # "WS" | "KAFKA"（事件上报定位用）
        self.remote_active = False               # 是否处于"正在向底盘下发遥控帧"状态
        self.release_requested = False           # 平台显式交还驾驶权
        self.spin_thread = None

    # ------------------------------------------------------------------
    # ROS2 桥接（发布 /remote/command，文档 13.4）
    # ------------------------------------------------------------------
    def init_ros(self):
        if not HAS_ROS:
            logger.warning("rclpy 不可用，远程指令发布降级")
            return
        rclpy.init(args=[])
        self.ros_node = Node("remote_agent_bridge")
        self.cmd_publisher = self.ros_node.create_publisher(ChassisCommand, self.cmd_topic, 10)
        # 交还驾驶权入口：command_agent 收到平台 REMOTE_RELEASE 指令后，会在本话题发一帧
        # control_mode="AUTO"；遥控帧一律是 "REMOTE"，因此非 REMOTE 帧即为"交还"信号。
        # （不新增话题/字段，符合 .ai-rules 接口契约约束）
        self.ros_node.create_subscription(
            ChassisCommand, self.cmd_topic, self._on_handover_frame, 10)
        self.spin_thread = threading.Thread(target=self.ros_spin, daemon=True)
        self.spin_thread.start()
        # 20Hz 独立定时发布（不由 websocket 事件直接触发，保证限幅与超时判定同点生效）
        self.ros_node.create_timer(0.05, self.timer_callback)
        logger.info("ROS2 桥接就绪，20Hz 定时发布 %s", self.cmd_topic)

    def ros_spin(self):
        try:
            rclpy.spin(self.ros_node)
        except Exception as exc:  # noqa: BLE001
            logger.error("ROS2 spin 退出：%s", exc)

    def _on_handover_frame(self, msg):
        if getattr(msg, "control_mode", "REMOTE") != "REMOTE":
            self.release_requested = True
            self.last_cmd_time = 0.0
            logger.info("收到交还驾驶权帧（control_mode=%s）→ 停止遥控发布", msg.control_mode)

    def timer_callback(self):
        """20Hz：仅在遥控数据新鲜时下发遥控帧；超时立即停车并交还（见模块 docstring）"""
        timeout = float(self.config["cmd_timeout"])
        fresh = (time.time() - self.last_cmd_time) <= timeout if self.last_cmd_time else False

        if fresh and not self.release_requested:
            self.remote_active = True
            self.publish_command(self.latest_velocity, self.latest_steering, self.latest_brake)
            return

        if self.remote_active:
            # 超时/交还的**一次性**收尾：先刹停，再停止发布，让 decision_making 回 AUTO
            self.publish_command(0.0, 0.0, brake=True)
            self.remote_active = False
            reason = "release" if self.release_requested else "cmd_timeout"
            logger.warning("遥控链路结束（%s）：已下发停车帧并停止发布 /remote/command", reason)
            self.report_event("remote_%s" % ("released" if reason == "release" else "timeout"),
                              "warning" if reason == "release" else "critical",
                              {"source": self.cmd_source,
                               "last_cmd_age_ms": int((time.time() - self.last_cmd_time) * 1000)})

    def publish_command(self, velocity, steering, brake=False):
        if not HAS_ROS or not self.cmd_publisher or not self.ros_node:
            return
        msg = ChassisCommand()
        msg.header.stamp = self.ros_node.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.target_velocity = float(velocity)
        msg.target_steering = float(steering)
        msg.control_mode = "REMOTE"
        msg.emergency_stop = bool(brake)
        self.cmd_publisher.publish(msg)

    # ------------------------------------------------------------------
    # Kafka：remote_control 消费（接入包契约）+ event 上报（文档 13.1）
    # ------------------------------------------------------------------
    def init_kafka(self):
        if not HAS_HUNTER_KAFKA:
            logger.warning("hunter_kafka 公共包不可用（未 source 工作空间？），"
                           "remote_control Kafka 通道与事件上报降级")
            return False
        try:
            # 交还驾驶权后不希望历史遥控帧被重新推给车辆：只消费最新（与 TTL 校验双保险）
            self.kafka_consumer = make_consumer(
                self.vehicle_id, "remote_control", group_suffix="remote",
                properties_path=self.config.get("kafka_properties", DEFAULT_PROPERTIES_PATH),
                bundle_dir=self.config.get("kafka_bundle_dir", DEFAULT_BUNDLE_DIR),
                auto_offset_reset="latest")
            self.kafka_producer = make_producer(
                self.vehicle_id, "event",
                properties_path=self.config.get("kafka_properties", DEFAULT_PROPERTIES_PATH),
                bundle_dir=self.config.get("kafka_bundle_dir", DEFAULT_BUNDLE_DIR))
        except KafkaConfigError as exc:
            logger.error("Kafka 接入失败：%s", exc)
            return False
        logger.info("remote_control Kafka 通道就绪：%s",
                    topic_name(self.vehicle_id, "remote_control"))
        threading.Thread(target=self.kafka_loop, daemon=True).start()
        return True

    def kafka_loop(self):
        while self.running:
            try:
                self.kafka_producer.poll(0.0)
                msg = self.kafka_consumer.poll(0.2)
            except Exception as exc:  # noqa: BLE001
                logger.error("Kafka 轮询异常：%s", exc)
                time.sleep(1.0)
                continue
            if msg is None:
                continue
            if msg.error():
                logger.error("Kafka 消费错误：%s", msg.error())
                continue
            try:
                self.on_command(msg.value(), source="KAFKA")
            finally:
                # 逐条手动提交（契约：enable.auto.commit=false）
                try:
                    self.kafka_consumer.commit(offsets=[msg], asynchronous=False)
                except Exception as exc:  # noqa: BLE001
                    logger.error("Kafka offset 提交失败：%s", exc)

    def report_event(self, event_type, level, extra=None):
        """遥控会话事件 → hunter.<vid>.event（acks=all）。
        接入包 Topic 契约没有 remote_status，旧版自发该 Topic 已废弃。"""
        payload = {
            "vehicle_id": self.vehicle_id,
            "timestamp": int(time.time()),
            "type": event_type,
            "level": level,
        }
        payload.update(extra or {})
        logger.info("遥控事件上报：%s", json.dumps(payload, ensure_ascii=False))
        if not self.kafka_producer:
            return
        try:
            self.kafka_producer.produce(
                topic_name(self.vehicle_id, "event"),
                value=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
                key=self.vehicle_id.encode("utf-8"))
            self.kafka_producer.poll(0)
        except Exception as exc:  # noqa: BLE001
            logger.error("事件投递失败：%s", exc)

    # ------------------------------------------------------------------
    # 控制指令处理（文档 13.4；WebSocket 与 Kafka remote_control 共用）
    # ------------------------------------------------------------------
    def on_command(self, data, source="WS"):
        try:
            cmd = json.loads(data) if isinstance(data, (str, bytes, bytearray)) else data
        except (json.JSONDecodeError, UnicodeDecodeError, TypeError):
            logger.warning("指令解析失败：%r", data)
            return
        if not isinstance(cmd, dict):
            logger.warning("指令报文不是 JSON 对象：%r", data)
            return

        # 平台显式交还驾驶权：remote_control 报文带 release=true / control_mode="AUTO"
        if cmd.get("release") is True or cmd.get("control_mode") == "AUTO":
            self.release_requested = True
            logger.info("平台请求交还驾驶权（remote_control）")
            return

        # 时效校验：断网重连/消费组重建时不得执行历史遥控帧
        issued = cmd.get("issued_at") or cmd.get("ts")
        if isinstance(issued, (int, float)) and issued > 0:
            ttl = float(self.config.get("remote_cmd_ttl", 1.0))
            if time.time() - float(issued) > ttl:
                logger.warning("丢弃过期遥控指令（%.2fs 前下发）", time.time() - float(issued))
                return

        # 安全限幅（文档 13.4：远程模式最高 2.0 m/s，转向 ±0.4 rad）
        self.latest_velocity = clamp(
            float(cmd.get("velocity", 0.0)),
            -float(self.config["max_velocity"]), float(self.config["max_velocity"]))
        self.latest_steering = clamp(
            float(cmd.get("steering_angle", 0.0)),
            -float(self.config["max_steering"]), float(self.config["max_steering"]))
        self.latest_brake = bool(cmd.get("brake", False))
        self.last_cmd_time = time.time()
        self.release_requested = False
        self.cmd_source = source

    # ------------------------------------------------------------------
    # WebSocket 循环（SRS 信令 + 控制指令，断线重连）
    # ------------------------------------------------------------------
    def websocket_loop(self):
        retry = 0
        while self.running:
            if not HAS_WS:
                logger.error("websocket-client 不可用")
                time.sleep(5)
                continue
            try:
                self.ws = websocket.WebSocket()
                self.ws.connect(self.ws_url, timeout=10)
                logger.info("WebSocket 已连接：%s", self.ws_url)
                retry = 0
                self.report_event("remote_ws_connected", "info", {"ws_url": self.ws_url})
                while self.running:
                    data = self.ws.recv()
                    if data:
                        self.on_command(data, source="WS")
            except Exception as exc:  # noqa: BLE001
                logger.warning("WebSocket 断开：%s", exc)
                # 断线重连（文档 13.3.2：最多重试 10 次）
                retry = min(retry + 1, 10)
                self.report_event("remote_ws_disconnected", "warning",
                                  {"retry": retry, "error": str(exc)})
                time.sleep(min(2 ** retry, 10))
            finally:
                if self.ws:
                    try:
                        self.ws.close()
                    except Exception:  # noqa: BLE001
                        pass
                    self.ws = None

    # ------------------------------------------------------------------
    # 视频链路（文档 13.2.2：AGX 硬件 H264 编码）
    # ------------------------------------------------------------------
    def build_pipeline(self):
        """构建 GStreamer 编码流水线（1280x720@30fps，3Mbps CBR）"""
        c = self.config
        return (
            f"v4l2src device={c['camera_device']} ! "
            "video/x-raw,width=1280,height=720,framerate=30/1 ! "
            "videoconvert ! nvvidconv ! 'video/x-raw(memory:NVMM),format=NV12' ! "
            f"nvv4l2h264enc bitrate={c['bitrate']} iframeinterval=30 control-rate=1 "
            "profile=4 preset-level=1 ! "
            "h264parse ! rtph264pay config-interval=1 pt=96 ! "
            f"udpsink host={c['srs_host']} port={c['srs_rtp_port']}"
        )

    def start_gstreamer(self):
        if not self.config.get("enable_video", True):
            logger.info("视频链路已按配置关闭（enable_video=false）")
            return
        pipeline = self.build_pipeline()
        logger.info("启动 GStreamer 编码：%s", pipeline)
        try:
            self.gst_proc = subprocess.Popen(
                ["gst-launch-1.0", pipeline],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as exc:  # noqa: BLE001
            logger.error("GStreamer 启动失败：%s", exc)

    def stop_gstreamer(self):
        if self.gst_proc and self.gst_proc.poll() is None:
            self.gst_proc.terminate()

    # ------------------------------------------------------------------
    # 主入口
    # ------------------------------------------------------------------
    def run(self):
        self.running = True
        self.resolve_vehicle_id()
        self.init_ros()
        self.init_kafka()
        self.start_gstreamer()
        logger.info("Remote Agent 启动：vehicle_id=%s，遥控通道=WS(%s)+Kafka(%s)",
                    self.vehicle_id, HAS_WS and "on" or "off",
                    "on" if self.kafka_consumer else "off")
        try:
            # WebSocket 主循环（含断线重连）阻塞在此；Kafka 通道在独立线程
            self.websocket_loop()
        finally:
            self.running = False
            self.stop_gstreamer()
            self.shutdown()

    def shutdown(self):
        try:
            if self.kafka_consumer:
                self.kafka_consumer.close()
        except Exception as exc:  # noqa: BLE001
            logger.warning("Kafka consumer 关闭异常：%s", exc)
        try:
            if self.kafka_producer:
                self.kafka_producer.flush(3.0)
        except Exception as exc:  # noqa: BLE001
            logger.warning("Kafka producer flush 异常：%s", exc)
        try:
            if self.ros_node and HAS_ROS and rclpy.ok():
                self.ros_node.destroy_node()
                rclpy.shutdown()
        except Exception as exc:  # noqa: BLE001
            logger.warning("ROS2 关闭异常：%s", exc)

    def resolve_vehicle_id(self):
        """vehicle_id 以接入包为准（SASL 用户名 == 证书 CN == vehicle_id）"""
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
            logger.error("vehicle_id 冲突：配置=%s 而接入包=%s（等同证书 CN）；以接入包为准",
                         self.vehicle_id, props_vid)
            self.vehicle_id = props_vid


DEFAULT_CONFIG = {
    "vehicle_id": "HUNTER-001",
    "camera_device": "/dev/video0",          # D435 RGB（文档 13.2.1）
    "enable_video": True,
    "bitrate": 3000000,                       # 3Mbps CBR（文档 13.2.2）
    "srs_host": "127.0.0.1",                  # SRS 媒体服务器
    "srs_rtp_port": 8000,
    "ws_url": "wss://platform.example.com/ws/remote",  # 文档 17.2
    "cmd_topic": "/remote/command",           # 文档 13.4
    # Kafka 接入只保留路径（凭据与 mTLS 证书在 /etc/hunter/kafka/kafka.properties）
    "kafka_properties": DEFAULT_PROPERTIES_PATH,
    "kafka_bundle_dir": DEFAULT_BUNDLE_DIR,
    "max_velocity": 2.0,                      # 文档 13.4：远程限速 2.0 m/s
    "max_steering": 0.4,                      # 文档 4.3：转向 ±0.4 rad
    "cmd_timeout": 0.5,                       # 文档 13.4：超时 500ms 停车并交还
    "remote_cmd_ttl": 1.0,                    # remote_control 报文时效（秒）
}


def load_config(path):
    """YAML 覆盖内置默认值；支持 remote_agent: ros__parameters: 结构或扁平结构"""
    cfg = dict(DEFAULT_CONFIG)
    cfg["kafka_properties"] = os.environ.get("HUNTER_KAFKA_PROPERTIES",
                                             cfg["kafka_properties"])
    if path and os.path.isfile(path) and HAS_YAML:
        with open(path, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh) or {}
        section = data.get("remote_agent", data)
        cfg.update({k: v for k, v in section.get("ros__parameters", section).items()})
        logger.info("已加载配置：%s", path)
    elif path:
        logger.warning("配置文件不可用（缺文件或缺 PyYAML），使用内置默认值：%s", path)
    return cfg


def main():
    parser = argparse.ArgumentParser(description="HUNTER Remote Agent")
    parser.add_argument("--config",
                        default=os.environ.get("HUNTER_REMOTE_CONFIG",
                                               "/etc/hunter/remote_agent_params.yaml"),
                        help="YAML 参数文件（不含任何 Kafka 凭据）")
    args = parser.parse_args()
    agent = RemoteAgent(load_config(args.config))
    agent.run()


if __name__ == "__main__":
    main()
