#!/usr/bin/env python3
"""Remote Agent — 远程操控车载端（文档第 13 章，systemd 服务，非 ROS 节点）。

功能：GStreamer AGX 硬件 H264 编码 D435 视频 → WebRTC/SRS 传输；
WebSocket 接收远程控制指令 → 转 ChassisCommand 发布 /remote/command；
安全限速、指令超时自动停车、断线重连、状态 Kafka 上报。
"""
import json
import logging
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
    from confluent_kafka import Producer
    HAS_KAFKA = True
except ImportError:
    HAS_KAFKA = False

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
        self.last_cmd_time = time.time()
        self.ros_node = None
        self.cmd_publisher = None
        self.kafka_producer = None

        self.ws_url = config["ws_url"]           # wss://platform/ws/remote
        self.cmd_topic = config.get("cmd_topic", "/remote/command")

        # 修正：引入 20Hz 定时器独立发布，而不是由 websocket 事件直接触发
        self.latest_velocity = 0.0
        self.latest_steering = 0.0
        self.latest_brake = False
        self.timer = None

    # ------------------------------------------------------------------
    # GStreamer（文档 13.2.2：AGX 硬件 H264 编码）
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
        pipeline = self.build_pipeline()
        logger.info("启动 GStreamer 编码：%s", pipeline)
        try:
            self.gst_proc = subprocess.Popen(
                ["gst-launch-1.0", pipeline],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as e:
            logger.error("GStreamer 启动失败：%s", e)

    # ------------------------------------------------------------------
    # ROS2 桥接（发布 /remote/command，文档 13.4）
    # ------------------------------------------------------------------
    def init_ros(self):
        if not HAS_ROS:
            logger.warning("rclpy 不可用，远程指令发布降级")
            return
        rclpy.init(args=[])
        self.ros_node = Node("remote_agent_bridge")
        self.cmd_publisher = self.ros_node.create_publisher(
            ChassisCommand, self.cmd_topic, 10)

        # 修正：引入 20Hz 定时器独立发布，而不是由 websocket 事件直接触发
        self.timer = self.ros_node.create_timer(0.05, self.timer_callback)  # 20Hz 频率
        logger.info("ROS2 桥接就绪，20Hz 定时发布 %s", self.cmd_topic)

    def timer_callback(self):
        # 修正：将超时检查融入 20Hz 定时器中
        if time.time() - self.last_cmd_time > self.config["cmd_timeout"]:
            self.publish_command(0.0, 0.0, brake=True)
        else:
            self.publish_command(self.latest_velocity, self.latest_steering, self.latest_brake)

    def publish_command(self, velocity, steering, brake=False):
        if not HAS_ROS or not self.cmd_publisher:
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
    # Kafka 状态上报（文档 13.1）
    # ------------------------------------------------------------------
    def init_kafka(self):
        if not HAS_KAFKA:
            logger.warning("confluent_kafka 不可用，状态上报降级")
            return
        self.kafka_producer = Producer(
            {"bootstrap.servers": self.config["kafka_brokers"]})

    def report_status(self, payload):
        if self.kafka_producer and HAS_KAFKA:
            topic = f"hunter.{self.config['vehicle_id']}.remote_status"
            self.kafka_producer.produce(topic, json.dumps(payload).encode("utf-8"))
            self.kafka_producer.poll(0)

    # ------------------------------------------------------------------
    # 控制指令处理（文档 13.4）
    # ------------------------------------------------------------------
    def on_command(self, data):
        try:
            cmd = json.loads(data) if isinstance(data, str) else data
        except (json.JSONDecodeError, TypeError):
            logger.warning("指令解析失败：%r", data)
            return

        # 安全限速（文档 13.4：远程模式最高 2.0 m/s，转向 ±0.4 rad）
        self.latest_velocity = clamp(
            cmd.get("velocity", 0.0),
            -self.config["max_velocity"], self.config["max_velocity"])
        self.latest_steering = clamp(
            cmd.get("steering_angle", 0.0),
            -self.config["max_steering"], self.config["max_steering"])
        self.latest_brake = bool(cmd.get("brake", False))
        self.last_cmd_time = time.time()

    def check_timeout(self):
        """指令超时自动停车（文档 13.4：500ms 未收到 → 停车）"""
        # 修正：超时检查已移至 20Hz 定时器中，此方法保留为占位符
        while self.running:
            time.sleep(0.1)

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
                self.report_status({"event": "connected", "ts": int(time.time())})
                while self.running:
                    data = self.ws.recv()
                    if data:
                        self.on_command(data)
            except Exception as e:
                logger.warning("WebSocket 断开：%s", e)
                # 断线重连（文档 13.3.2：最多重试 10 次）
                retry = min(retry + 1, 10)
                self.report_status({"event": "disconnected", "retry": retry,
                                    "ts": int(time.time())})
                time.sleep(min(2 ** retry, 10))
            finally:
                if self.ws:
                    try:
                        self.ws.close()
                    except Exception:
                        pass
                    self.ws = None

    # ------------------------------------------------------------------
    # 主入口
    # ------------------------------------------------------------------
    def run(self):
        self.running = True
        self.init_ros()
        self.init_kafka()
        self.start_gstreamer()

        # 超时监控线程
        threading.Thread(target=self.check_timeout, daemon=True).start()

        # WebSocket 主循环（含断线重连）
        self.websocket_loop()

        self.running = False


DEFAULT_CONFIG = {
    "vehicle_id": "hunter_001",
    "camera_device": "/dev/video0",          # D435 RGB（文档 13.2.1）
    "bitrate": 3000000,                       # 3Mbps CBR（文档 13.2.2）
    "srs_host": "127.0.0.1",                 # SRS 媒体服务器
    "srs_rtp_port": 8000,
    "ws_url": "wss://platform.example.com/ws/remote",  # 文档 17.2
    "cmd_topic": "/remote/command",           # 文档 13.4
    "kafka_brokers": "platform.example.com:9093",
    "max_velocity": 2.0,                      # 文档 13.4：远程限速 2.0 m/s
    "max_steering": 0.4,                      # 文档 4.3：转向 ±0.4 rad
    "cmd_timeout": 0.5,                       # 文档 13.4：超时 500ms 停车
}


def main():
    agent = RemoteAgent(DEFAULT_CONFIG)
    agent.run()


if __name__ == "__main__":
    main()

