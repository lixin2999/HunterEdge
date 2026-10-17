#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Remote Agent — 远程操控车载端（文档第 13 章，systemd 服务，非 ROS 节点）。

功能（V0.1.15 起对齐 HunterCore V1.21.0《车端远程操控部署指南》）：
  1. 视频**会话制 RTMP 推流**（V1.21.0 架构：车端 RTMP 推流(1935) → SRS → 浏览器 WHEP
     拉流(9090)）：收到 boot 帧（heartbeat=true 且 video.recording=true）起 AGX 硬件
     H.264 编码，推到 ``<srs_rtmp_base_url>/<prefix>_<vehicle_id>_<session_id>``，
     stop 帧立即停推；非操控态不占编码器与带宽；
  2. 遥控指令双通道：WebSocket（备用，默认关闭）+ **Kafka hunter.<vid>.remote_control**
     （HunterCore 接入包契约通道，与平台其它业务同源同证书）；
  3. 指令限幅；**会话内失联刹停保持**（V0.1.18）：会话存续期间控制帧断流 >500ms
     持续下发零速刹停帧（车停在原地、不交还），仅收到 stop 帧/REMOTE_RELEASE 才让出；
  4. **逐帧回投 ``command_result``**（acks=all）：契约规定平台 >500ms 收不到回执即判
     链路异常并结束会话；remote_control.schema.json 无 command_id（pending #16 近似
     关联），故回执 command_id 用新生成 uuid，与平台参考实现（vehicle-simulator）一致；
  5. 遥控会话事件经 ``hunter.<vid>.event`` 上报（接入包 Topic 契约不含 remote_status，
     旧版自发该 Topic 已废弃）。

⚠ 驾驶权交还语义（V0.1.18 修订，取代旧版“超时即停发→回 AUTO”）：
  decision_making 判定 REMOTE 是否生效只看 /remote/command 是否在 remote_timeout 内到过
  数据（V0.1.28 起 0.5s→2.0s，与 safety_guard 遥控执行桥、本包 cmd_timeout 全链路对齐，
  见 src/decision_making/src/decision_making.cpp）。旧版无会话概念，
  任何超时都停发→回 AUTO，导致“遥控中短暂丢帧→车恢复跑自主航点”。现按会话区分：
  • 会话内（boot 已起、未 stop）失联：持续发布刹停帧保持 REMOTE，车停在原地等恢复；
  • 会话结束（stop 帧 / REMOTE_RELEASE 交还帧）：补一帧刹停后停止发布 → 回 AUTO。
  ⚠ 代价：若平台异常崩溃且未发 stop 帧，车会保持刹停直至人工 REMOTE_RELEASE/急停；
     这是“宁可停死不可失控”的有意取舍（与平台会话信令强绑定，需确保 stop 帧必发）。

配置来源优先级：命令行 --config 指定的 YAML > /etc/hunter/remote_agent_params.yaml >
内置默认值。Kafka 接入参数（Broker/SCRAM/mTLS 证书）不写在 YAML，统一来自
/etc/hunter/kafka/kafka.properties（HunterCore 接入包：目录 0750 / properties 0640 / 私钥 0600）。
"""
import argparse
import fcntl
import json
import logging
import os
import queue
import select
import subprocess
import threading
import time
import uuid

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
    from sensor_msgs.msg import Image as RosImage
    HAS_IMAGE_MSG = True
except ImportError:
    HAS_IMAGE_MSG = False

try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

try:
    from cv_bridge import CvBridge
    HAS_CV_BRIDGE = True
except ImportError:
    HAS_CV_BRIDGE = False

try:
    import yaml
    HAS_YAML = True
except ImportError:
    HAS_YAML = False

try:
    from confluent_kafka import TopicPartition  # 手动提交需构造（V0.1.16）
except ImportError:
    TopicPartition = None

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

#: 事件类型 → 契约等级（HunterCore ``contracts/database/enums.md`` §3，**不可更改**）。
#: 平台 data-collector 会按 ``EVENT_LEVEL_BY_TYPE`` 校验「类型↔等级」一致性：
#: 等级写错 = 非法事件（DLQ）。车端只上报本表内的类型，其余（遥控链路噪声）只落本地日志。
EVENT_LEVEL_BY_TYPE = {
    "manual_takeover": "info",       # 人工接管（遥控接管生效）
    "emergency_stop": "critical",    # 遥控急停（如平台下发零速+brake=1 的急停帧）
}



def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class RemoteAgent:
    def __init__(self, config):
        self.config = config
        self.ws = None
        self.running = False
        self.last_cmd_time = 0.0
        self.ros_node = None
        self.cmd_publisher = None
        self.kafka_producer = None
        self.kafka_consumer = None
        self.rc_result_producer = None          # command_result 回执（acks=all，不可丢）

        self.vehicle_id = config["vehicle_id"]
        self.ws_url = config["ws_url"]           # ws://<SERVER_IP>/ws/remote（备用通道）
        self.cmd_topic = config.get("cmd_topic", "/remote/command")

        # 遥控指令缓存（由 WS/Kafka 任一通道写入，20Hz 定时器读出并发布）
        self.latest_velocity = 0.0
        self.latest_steering = 0.0
        self.latest_brake = False
        self.cmd_source = ""                     # "WS" | "KAFKA"（事件上报定位用）
        self.remote_active = False               # 是否处于"正在向底盘下发遥控帧"状态
        self.release_requested = False           # 平台显式交还驾驶权
        self.session_active = False              # 会话是否存续（boot 置真、stop/交还置假）
        self.hold_logged = False                 # 会话内失联刹停日志只打一次
        self.spin_thread = None

        # RTMP 推流会话态（boot 帧起、stop 帧停；_video_lock 保护跨 Kafka/信号处理）
        self.session_id = None
        self.rtmp_proc = None
        self._stream_session = None
        self._video_lock = threading.Lock()
        self._image_sub = None              # ROS image topic subscriber
        self._video_stop_event = threading.Event()
        # V0.1.19 关键修复：独立写线程 + 有界队列，彻底解耦 ROS 执行器与 GStreamer 管道写入
        self._frame_queue = queue.Queue(maxsize=1)  # 只保留最新一帧，满则丢旧留新（见 _on_image_frame）
        self._frame_writer_thread = None
        # 推流分辨率（fdsrc caps 固定值），由 _build_ros_pipeline 写入，供 _convert_frame 对齐尺寸
        self._stream_size = (int(config.get("stream_width", 640)),
                              int(config.get("stream_height", 480)))
        self._cv_bridge = CvBridge() if HAS_CV_BRIDGE else None
        self._convert_logged = False

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
        """V0.1.19 关键修复：用 MultiThreadedExecutor 避免 image 回调阻塞 timer。

        旧实现用 rclpy.spin()（SingleThreadedExecutor），当 RTMP 连接失败时
        _on_image_frame 的 proc.stdin.write() 会阻塞整个执行器线程，
        导致 20Hz timer_callback 饥饿（实测只有 0.067Hz）。
        MultiThreadedExecutor 让 image/timer 回调在不同线程并行。
        """
        try:
            from rclpy.executors import MultiThreadedExecutor
            executor = MultiThreadedExecutor(num_threads=4)
            executor.add_node(self.ros_node)
            executor.spin()
        except Exception as exc:  # noqa: BLE001
            logger.error("ROS2 spin 退出：%s", exc)

    def _on_handover_frame(self, msg):
        if getattr(msg, "control_mode", "REMOTE") != "REMOTE":
            self.release_requested = True
            self.session_active = False
            self.last_cmd_time = 0.0
            logger.info("收到交还驾驶权帧（control_mode=%s）→ 停止遥控发布", msg.control_mode)

    def timer_callback(self):
        """20Hz 发布 /remote/command（V0.1.18：会话内失联刹停保持）
        - 会话内（session_active 且未交还）：有新鲜控制帧→下发该帧；帧断流→持续
          下发刹停帧（保持 /remote/command 新鲜 → decision_making 维持 REMOTE、
          safety_guard 遥控输入零速），车停在原地，直到收到 stop/交还才让出。
        - 非会话（未 boot / 已 stop / 已交还）：若之前在发→补一帧刹停后停止发布，
          decision_making 判超时回 AUTO。
        """
        timeout = float(self.config["cmd_timeout"])
        fresh = (time.time() - self.last_cmd_time) <= timeout if self.last_cmd_time else False

        if self.session_active and not self.release_requested:
            if not self.remote_active:
                # 遥控接管**开始**：契约受控词表里的 manual_takeover（info）
                self.report_event("manual_takeover", extra={"source": self.cmd_source})
                self.remote_active = True
            if fresh:
                self.hold_logged = False
                self.publish_command(self.latest_velocity, self.latest_steering, self.latest_brake)
            else:
                # 会话内帧断流：刹停保持（不下线、不交还），限频提示避免刷屏
                self.publish_command(0.0, 0.0, brake=True)
                if not self.hold_logged:
                    self.hold_logged = True
                    logger.warning("遥控会话内指令断流（>%.0fms）：刹停保持，等待恢复或 stop",
                                   timeout * 1000)
            return

        if self.remote_active:
            # 会话结束/交还的**一次性**收尾：先刹停，再停止发布，让 decision_making 回 AUTO
            self.publish_command(0.0, 0.0, brake=True)
            self.remote_active = False
            self.hold_logged = False
            reason = "release" if self.release_requested else "session_inactive"
            logger.warning("遥控链路结束（%s）：已下发停车帧并停止发布 /remote/command", reason)
            # 契约没有「遥控结束」事件类型 → 只落本地日志（report_event 会过滤掉自造类型）
            self.report_event("remote_%s" % ("released" if self.release_requested else "timeout"),
                              "warning" if self.release_requested else "critical",
                              {"source": self.cmd_source,
                               "last_cmd_age_ms": int((time.time() - self.last_cmd_time) * 1000)
                               if self.last_cmd_time else -1})

    def publish_command(self, velocity, steering, brake=False):
        if not HAS_ROS or not self.cmd_publisher or not self.ros_node:
            # V0.1.19 诊断：一次性报告 ROS 发布不可用的原因
            if not getattr(self, '_pub_warn_logged', False):
                self._pub_warn_logged = True
                logger.error("[遥控发布] ✘ ROS 不可用！HAS_ROS=%s cmd_publisher=%s ros_node=%s"
                             "—— 确认 hunter_msgs 已安装并 source 了工作空间",
                             HAS_ROS, self.cmd_publisher, self.ros_node)
            return
        # V0.1.19 诊断：首次成功发布时记录
        if not getattr(self, '_pub_ok_logged', False):
            self._pub_ok_logged = True
            logger.info("[遥控发布] ✔ 首帧发布成功 v=%.2f steer=%.3f brake=%s", 
                        velocity, steering, brake)
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
            # 操控回执专用 producer（契约 command_result 属 acks=all 关键链，与 event 同档
            # 但分开建，便于高频 20Hz 回执不挤占事件队列）
            self.rc_result_producer = make_producer(
                self.vehicle_id, "command_result",
                properties_path=self.config.get("kafka_properties", DEFAULT_PROPERTIES_PATH),
                bundle_dir=self.config.get("kafka_bundle_dir", DEFAULT_BUNDLE_DIR))
        except KafkaConfigError as exc:
            logger.error("Kafka 接入失败：%s", exc)
            return False
        logger.info("remote_control Kafka 通道就绪：%s（回执：%s）",
                    topic_name(self.vehicle_id, "remote_control"),
                    topic_name(self.vehicle_id, "command_result"))
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
                # 逐条手动提交（契约：enable.auto.commit=false）。
                # ⚠ commit(offsets=...) 只接受 TopicPartition 列表且 offset 需 +1（下一条）；
                # 直传 Message 必报 "expected cimpl.TopicPartition"（V0.1.05~V0.1.15 每帧
                # ERROR、offset 从未落盘；不阻断消费但破坏重启后续点语义）
                try:
                    if TopicPartition is not None:
                        tp = TopicPartition(msg.topic(), msg.partition(), msg.offset() + 1)
                        self.kafka_consumer.commit(offsets=[tp], asynchronous=False)
                    else:
                        self.kafka_consumer.commit(asynchronous=False)
                except Exception as exc:  # noqa: BLE001
                    logger.error("Kafka offset 提交失败：%s", exc)

    def report_event(self, event_type, level=None, extra=None):
        """遥控事件 → ``hunter.<vid>.event``（**契约字段**：``event_type``/``event_level``）。

        ⚠ 契约 ``event.schema.json`` 只认 19 种 ``event_type``（受控词表，
        ``additionalProperties=false``），等级由 ``EVENT_LEVEL_BY_TYPE`` 固定
        （``contracts/database/enums.md`` §3：``manual_takeover`` = **info**）。
        旧版发 ``{type, level}`` 且用了自造类型 ``remote_ws_connected`` /
        ``remote_ws_disconnected`` / ``remote_released`` / ``remote_timeout``
        → 平台判非法（DLQ），而且 WS 断线重连每轮都发一条，属噪声污染。

        因此：**只有受控词表内的类型才上平台**，其余仅本地日志留痕。
        """
        contract_level = EVENT_LEVEL_BY_TYPE.get(event_type)
        if contract_level is None:
            logger.info("遥控事件（契约无此类型，仅本地留痕）：%s %s", event_type, extra or {})
            return
        payload = {
            "vehicle_id": self.vehicle_id,
            "timestamp": int(time.time()),
            "event_type": event_type,
            "event_level": contract_level,
            "description": f"远程操控：{event_type}",
        }
        if extra:
            payload["data"] = extra
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
    
        # ── 帧类别与字段名（HunterCore ``remote_control.schema.json``）──────────
        # 契约：{vehicle_id, timestamp, seq, control:{target_velocity,target_steer,brake,gear,mode},
        #        heartbeat, reason, session_id, video:{recording,...}}
        # G-10/V1.21.0：boot 帧 heartbeat=true + video.recording=true（控制量恒零）；
        # stop 帧 heartbeat=false + video.recording=false，且带受控词表 reason。
        control = cmd.get("control") if isinstance(cmd.get("control"), dict) else {}
        heartbeat = bool(cmd.get("heartbeat", False))
        video = cmd.get("video") if isinstance(cmd.get("video"), dict) else None
        session_id = cmd.get("session_id") or self.session_id
    
        # V0.1.19 视频诊断：记录首帧的 video 字段状态，帮助确认平台是否发了 boot 帧
        if not getattr(self, '_frame_logged', False):
            self._frame_logged = True
            logger.info("[遥控帧] video=%s heartbeat=%s session_id=%s cmd_keys=%s",
                        video, heartbeat, session_id, list(cmd.keys())[:10])

        # 会话边界帧先于 TTL 判定处理：boot/stop 决定推拉流，迟到几帧也必须按序执行，
        # 否则会留下野推流（stop 被丢）或拒绝接管（boot 被丢）的脏状态。
        # ① boot（recording=true）：记 session_id → 起 RTMP 推流（幂等）
        if video is not None and video.get("recording") is True:
            if session_id:
                self.session_id = str(session_id)
            self.session_active = True
            self.release_requested = False
            self.start_stream(self.session_id, video)
        # ② stop（recording=false）或旧版兼容帧（reason 非空且 heartbeat=false）：
        #    停推 + 交还驾驶权（兼容路径：部分平台版本 stop 帧不带 video 段）
        if ((video is not None and video.get("recording") is False)
                or (cmd.get("reason") is not None and not heartbeat)):
            self.stop_stream(reason=cmd.get("reason"))
            self.session_id = None
            self.session_active = False
            self.release_requested = True
            logger.info("平台会话结束（remote_control stop，reason=%s）→ 停推流并交还驾驶权",
                        cmd.get("reason"))
            self.ack_command_result(cmd)
            return
    
        # 时效校验（仅限控制/心跳帧）：断网重连/消费组重建时不得执行历史遥控帧
        # （契约字段 timestamp，兼容旧名 issued_at/ts；秒级 epoch）
        issued = cmd.get("timestamp")
        if not isinstance(issued, (int, float)) or issued <= 0:
            issued = cmd.get("issued_at")
        if not isinstance(issued, (int, float)) or issued <= 0:
            issued = cmd.get("ts")
        if isinstance(issued, (int, float)) and issued > 0:
            ttl = float(self.config.get("remote_cmd_ttl", 1.0))
            if time.time() - float(issued) > ttl:
                logger.warning("丢弃过期遥控指令（%.2fs 前下发）", time.time() - float(issued))
                return
    
        # 安全限幅（文档 13.4：远程模式最高 2.0 m/s，转向 ±0.4 rad）
        # 契约字段 control.target_velocity / control.target_steer
        # V0.1.19 兼容：部分平台版本发 target_steering（带 ing），增加多字段 fallback
        raw_velocity = control.get("target_velocity",
                       control.get("velocity",
                         cmd.get("velocity", 0.0)))
        raw_steering = control.get("target_steer",
                        control.get("target_steering",
                          cmd.get("steering_angle", 0.0)))
        self.latest_velocity = clamp(
            float(raw_velocity), -float(self.config["max_velocity"]), float(self.config["max_velocity"]))
        self.latest_steering = clamp(
            float(raw_steering), -float(self.config["max_steering"]), float(self.config["max_steering"]))
        # V0.1.19 诊断日志：首次收到非零转向或每 100 帧记录一次，帮助确认字段映射
        if self.latest_steering != 0.0 and not getattr(self, '_steer_logged', False):
            self._steer_logged = True
            logger.info("[遥控] 首次收到非零转向 steer=%.3f rad (v=%.2f m/s) control_keys=%s",
                        self.latest_steering, self.latest_velocity, list(control.keys()))
        # 契约 brake 为归一化 0~1（G-10 可选；estop 零速帧固定 1.0）；底盘只有布尔
        # 急停 → 仅 brake>=1.0 视为急停（旧版 bool(0.5)=True 会把比例制动误判成急停）
        brake_raw = control.get("brake", cmd.get("brake", 0.0))
        if isinstance(brake_raw, bool):
            self.latest_brake = brake_raw
        else:
            try:
                self.latest_brake = float(brake_raw) >= 1.0
            except (TypeError, ValueError):
                self.latest_brake = False
        self.last_cmd_time = time.time()
        self.release_requested = False
        self.cmd_source = source
        # 逐帧回执（含 boot/心跳帧：平台据此计 commands_acked 与链路健康）
        self.ack_command_result(cmd)
    
    def ack_command_result(self, cmd):
        """回投 ``hunter.<vid>.command_result``（契约 required：command_id/vehicle_id/
        timestamp/success；additionalProperties=false）。
    
        remote_control 帧不携带 command_id（契约无此字段，pending #16 平台按
        (session_id, seq, 时序窗口) 近似关联计数），故幂等键用**新生成 uuid**；
        >500ms 无回执平台会判链路异常结束会话，因此**每帧都回**，失败只记日志
        不影响控制应用（回执丢失的后续帧会补上链路健康信号）。
        """
        if not self.config.get("ack_command_result", True):
            return
        if self.rc_result_producer is None:
            return
        control = cmd.get("control") if isinstance(cmd.get("control"), dict) else {}
        payload = {
            "command_id": str(uuid.uuid4()),
            "vehicle_id": self.vehicle_id,
            "timestamp": round(time.time(), 3),
            "success": True,
            "result_code": "0",
            "message": "applied v=%s steer=%s seq=%s" % (
                control.get("target_velocity", 0.0),
                control.get("target_steer", 0.0),
                cmd.get("seq"))[:512],
        }
        if cmd.get("session_id"):
            payload["data"] = {"session_id": str(cmd["session_id"])}
        try:
            self.rc_result_producer.produce(
                topic_name(self.vehicle_id, "command_result"),
                value=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
                key=self.vehicle_id.encode("utf-8"))
            self.rc_result_producer.poll(0)
        except Exception as exc:  # noqa: BLE001
            logger.warning("command_result 回执投递失败（不影响控制应用）：%s", exc)

    # ------------------------------------------------------------------
    # WebSocket 循环（SRS 信令 + 控制指令，断线重连）
    # ------------------------------------------------------------------
    def websocket_loop(self):
        retry = 0
        ws_missing_logged = False
        while self.running:
            if not HAS_WS:
                # 主通道是 Kafka（contract 通道）；WS 只是备用，缺包不必每 5s 刷一条 ERROR
                # （实机 remote_agent 日志被刷屏）。只首条提示一次 + 给出补装命令。
                if not ws_missing_logged:
                    logger.error("websocket-client 未安装：WebSocket 备用通道关闭"
                                 "（Kafka remote_control 主通道不受影响）；"
                                 "补装：sudo pip3 install websocket-client")
                    ws_missing_logged = True
                time.sleep(5)
                continue
            ws_missing_logged = False
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
    # 视频链路（V1.21.0：会话制 RTMP 推流，boot 起 / stop 停，AGX 硬件 H.264 编码）
    # ------------------------------------------------------------------
    def _rtmp_url_for(self, session_id):
        """拼接完整推流地址：``<base>/<prefix>_<vehicle_id>_<session_id>``（契约命名）。

        基址由车端静态配置（指南 §1：rtmp://<SERVER_IP>:1935/remote_control），
        流名前缀默认 ``hunter``（= 平台 RC_SRS_STREAM_PREFIX，不可自创）。
        """
        base = (self.config.get("srs_rtmp_base_url") or "").strip().rstrip("/")
        if not base or not session_id:
            return ""
        prefix = self.config.get("srs_stream_prefix", "hunter")
        return f"{base}/{prefix}_{self.vehicle_id}_{session_id}"

    def build_pipeline(self, session_id, video_cfg):
        """构建 GStreamer 硬编→RTMP 流水线（v4l2 模式，专用相机设备可用时使用）。

        V0.1.19 修复：去掉 videoconvert、去掉单/双引号包裹。
        """
        c = self.config
        fps = int(video_cfg.get("fps") or c.get("video_fps", 30))
        fps = max(1, min(fps, 30))
        kbps = int(video_cfg.get("bitrate_kbps") or c.get("video_bitrate_kbps", 3000))
        kbps = max(2048, min(kbps, 4096))
        url = self._rtmp_url_for(session_id)
        return (
            f"v4l2src device={c['camera_device']} ! "
            f"video/x-raw,width=1280,height=720,framerate={fps}/1 ! "
            "videoconvert ! "
            f"nvv4l2h264enc bitrate={kbps * 1000} iframeinterval={fps} ! "
            "h264parse ! flvmux streamable=true ! "
            f"rtmpsink location={url}"
        )

    def _build_ros_pipeline(self, session_id, video_cfg):
        """构建从 ROS image topic 读取帧的 GStreamer 流水线（fdsrc + stdin 管道）。

        V0.1.19：当 RealSense 被 ROS 节点占用时，通过订阅 /camera/color/image_raw
        获取原始 RGB 帧，写入 GStreamer stdin，由 fdsrc 接收并编码后推 RTMP。

        ❗ 分辨率必须以实际相机输出为准（stream_width/height 配置），
        而不以 boot 帧 video.width/height 为准（平台请求的分辨率可能与车端不符）。

        V0.1.22：必须显式设置 fdsrc 的 blocksize=width*height*3（见下），否则
        fdsrc 会按默认 4096 字节分块读 stdin，与一帧实际字节数不匹配，导致
        每一帧都被下游判定为非法 buffer 而丢弃（真实现象：整条链路一直在报
        `invalid buffer size 4096 < 921600`，但画面层看起来只是“绿屏”，容易被
        误判成颜色转换问题）。
        """
        c = self.config
        fps = int(c.get("video_fps", 15))
        fps = max(1, min(fps, 30))
        kbps = int(video_cfg.get("bitrate_kbps") or c.get("video_bitrate_kbps", 3000))
        kbps = max(2048, min(kbps, 4096))
        # 强制用配置的实际相机分辨率，忽略 boot 帧的请求（boot 帧要求 1280x720 但相机只出 640x480）
        width = int(c.get("stream_width", 640))
        height = int(c.get("stream_height", 480))
        # 记录到实例，供 _on_image_frame/_convert_frame 把 ROS 图像严格对齐到这个尺寸
        self._stream_size = (width, height)
        url = self._rtmp_url_for(session_id)
        # V0.1.19：默认用 x264enc 软编（所有 GStreamer 安装均有），
        # nvv4l2h264enc 在部分 JetPack 版本不存在导致 syntax error
        prefer_hw = c.get("prefer_hw_encode", False)
        if prefer_hw:
            enc_stage = (
                f"videoconvert ! video/x-raw,format=I420 ! "
                f"nvv4l2h264enc bitrate={kbps * 1000} iframeinterval={fps}"
            )
        else:
            # WebRTC 强制要求 Constrained Baseline profile：
            #   x264enc 无 profile 属性（GStreamer 1.20），用 cabac=false + bframes=0 等效
            enc_stage = (
                f"videoconvert ! x264enc bitrate={kbps} tune=zerolatency "
                f"speed-preset=ultrafast key-int-max={fps} "
                f"bframes=0 cabac=false"
            )
        # V0.1.22 根因修复：fdsrc 继承 GstBaseSrc，默认 blocksize=4096 字节——
        # 从 stdin 每次只读 4096 字节就当作一个完整 buffer 推给下游，不管下游
        # capsfilter 要求的是 640x480x3=921600 字节/帧，导致每个 buffer 都被
        # videoconvert 判定为非法大小（`invalid buffer size 4096 < 921600`），
        # 整条管道从未成功编码出一帧真实画面，浏览器收不到有效 H.264 数据而
        # 显示解码器默认绿色。这不是唯一问题（见下方 V0.1.23 补充：修好
        # blocksize 后仍观察到写入超时，说明管道进入 PLAYING/RTMP 推流这一环
        # 节确实也可能存在问题，V0.1.21 的“RTMP 卡住”假设当时被本 bug 掩盖，
        # 不是被推翻），但它是必须修的第一层。
        # 修复：显式把 blocksize 设为完整一帧的字节数（RGB24 = width*height*3），
        # fdsrc 会精确读满一帧才输出，与 _frame_writer_loop 的整帧原子写入
        #（V0.1.21）天然对齐——管道里永远不会有半帧，也不会错位。
        frame_bytes = width * height * 3
        # V0.1.23：queue 从 3 提到 15（约 1s @15fps）——blocksize 修正后，fdsrc
        # 会真正把完整帧喂给下游，一旦 RTMP/x264 瞬时比帧率慢（网络抖动、关键帧
        # 峰值），3 个 buffer 的队列会立即填满并反压 fdsrc 停止读 stdin，导致
        # 写线程误判为“卡死”而触发超时重启。适当加大队列能吸收瞬时抖动。
        return (
            f"fdsrc blocksize={frame_bytes} ! "
            f"video/x-raw,format=RGB,width={width},height={height},framerate={fps}/1 ! "
            f"queue max-size-buffers=15 ! "
            f"{enc_stage} ! "
            "h264parse ! flvmux streamable=true ! "
            f"rtmpsink location={url}"
        )

    def start_stream(self, session_id, video_cfg=None):
        """boot 帧→启动 RTMP 推流（按 session_id 幂等；同会话重复 boot 不重启）

        支持两种模式（配置 camera_source）：
          - "ros_topic"（默认）：订阅 /camera/color/image_raw 获取帧，通过 stdin 管道送入 GStreamer
          - "v4l2"：直接 v4l2src 读相机设备（适用于专用USB摄像头，不与其他节点冲突）
        """
        if not self.config.get("enable_video", True):
            logger.info("视频链路已按配置关闭（enable_video=false），boot 帧不起推流")
            return
        if not session_id:
            logger.warning("boot 帧无 session_id，无法定位流名，不起推流")
            return
        url = self._rtmp_url_for(session_id)
        if not url:
            logger.error("srs_rtmp_base_url 未配置，无法推流（例："
                         "rtmp://192.168.31.35:1935/remote_control）")
            return

        mode = self.config.get("camera_source", "ros_topic")

        # v4l2 模式预检设备可用性
        if mode == "v4l2":
            dev = self.config.get("camera_device", "/dev/video0")
            if not os.path.exists(dev):
                logger.error("相机设备不存在：%s，请确认接线并用 `v4l2-ctl --list-devices` 查可用设备", dev)
                return
            try:
                r = subprocess.run(["fuser", dev], capture_output=True, text=True, timeout=3)
                if r.stdout.strip():
                    logger.warning("⚠ 相机 %s 已被进程占用 (PID=%s)，v4l2src 将失败！"
                                   "建议改用 camera_source: ros_topic",
                                   dev, r.stdout.strip())
                    return  # 直接返回，不启动必败的推流
            except Exception:
                pass

        with self._video_lock:
            if self.rtmp_proc and self.rtmp_proc.poll() is None \
                    and self._stream_session == session_id:
                return
            self._shutdown_proc_locked()

            # 构建流水线
            if mode == "ros_topic":
                pipeline = self._build_ros_pipeline(session_id, video_cfg or {})
            else:
                pipeline = self.build_pipeline(session_id, video_cfg or {})

            logger.info("boot 帧到，启动 RTMP 推流（mode=%s）：%s", mode, url)
            logger.info("[Pipeline] %s", pipeline)

            try:
                # V0.1.19 关键修复：GStreamer 1.20.1 on Jetson aarch64
                # 的 gst-launch-1.0 不能接受整个 pipeline 作为单参数（报 syntax error），
                # 必须像命令行一样按空格拆分为多个 argv 元素。
                # 实机证据：
                #   命令行多参数 → ✅ 成功；命令行单引号包裹 → ❌ syntax error
                gst_args = ["gst-launch-1.0", "-e"] + pipeline.split()
                # V0.1.21：现场排查“RTMP 已建立 TCP 连接但管道不进入 PLAYING”类问题时，
                # 默认关键字过滤会掉掉大量 GStreamer 自身的握手/状态日志（logger.debug
                # 不展示），需现场把 `gst_debug` 设为 "2"~"4" 开启详细日志才能定位卡在哪一步。
                gst_debug = str(self.config.get("gst_debug", "") or "").strip()
                gst_env = None
                if gst_debug:
                    gst_env = os.environ.copy()
                    gst_env["GST_DEBUG"] = gst_debug
                if mode == "ros_topic":
                    self.rtmp_proc = subprocess.Popen(
                        gst_args,
                        stdin=subprocess.PIPE,
                        stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                        env=gst_env)
                    # V0.1.24 关键修复（真正的根因）：Linux 管道默认容量仅 65536 字节
                    # （16 页 × 4KB），而一帧 RGB 数据 = width*height*3（640×480 下是
                    # 921600 字节），远大于管道容量。`fdsrc` 的 `read()` 系统调用不会
                    # 等待凑满请求字节数，只返回“当前管道里有多少就读多少”——即使
                    # V0.1.22 已把 blocksize 设为 921600，`fdsrc` 每次实际只能读到
                    # 65536 字节（管道装满即止），推给 `videoconvert` 的仍是半帧，
                    # 报 `invalid buffer size 65536 < 921600`（V0.1.23 部署后实机日志
                    # 首次暴露的真实数字）。
                    # 解决：用 `F_SETPIPE_SZ` 把管道容量扩到至少 1 帧 + 余量，让 `fdsrc` 一次
                    # `read(blocksize)` 能真正读到完整一帧。受 `/proc/sys/fs/pipe-max-size`
                    # 限制（默认 1048576 字节），超过则失败降级为原默认容量并打日志。
                    self._enlarge_pipe_buffer(self.rtmp_proc.stdin)
                else:
                    self.rtmp_proc = subprocess.Popen(
                        gst_args,
                        stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                        env=gst_env)
                self._stream_session = session_id
                self._video_stop_event.clear()

                # 启动 stderr 读取线程
                threading.Thread(
                    target=self._drain_gst_stderr,
                    args=(self.rtmp_proc, session_id, bool(gst_debug)),
                    daemon=True).start()

                # ROS topic 模式：订阅影像话题并持续送帧
                if mode == "ros_topic":
                    self._start_ros_image_feed()

            except Exception as exc:  # noqa: BLE001
                logger.error("GStreamer 推流启动失败：%s", exc)
                self.rtmp_proc = None
                self._stream_session = None

    def stop_stream(self, reason=None):
        """stop 帧/退出→停推流并释放编码器（幂等）"""
        with self._video_lock:
            if self.rtmp_proc is None and self._stream_session is None:
                return
            logger.info("停止 RTMP 推流（reason=%s，session=%s）",
                        reason, self._stream_session)
            self._shutdown_proc_locked()

    def _drain_gst_stderr(self, proc, session_id, verbose=False):
        """后台线程读取 GStreamer 子进程 stderr 并记入日志。

        常见需要关注的错误关键字：
          - 'Cannot open device'  / 'Device or resource busy' → 相机被 realsense-ros 占用
          - 'No such element'     → 缺 gstreamer1.0-plugins-good
          - 'Connection refused'  → SRS 1935 不可达

        V0.1.21：`verbose=True`（现场把 `gst_debug` 设为非空以开启 GST_DEBUG）时，
        不再做关键字过滤，所有行直接以 INFO 落日志——排查“RTMP 已建立 TCP 连接
        但管道不进入 PLAYING”这类无显式报错的静默卡住时，必须靠 GStreamer 自己
        的状态变化日志才能定位卡在哪一步。
        """
        try:
            for raw in iter(proc.stderr.readline, b""):
                msg = raw.decode("utf-8", errors="replace").rstrip()
                if not msg:
                    continue
                if verbose:
                    logger.info("[GStreamer session=%s] %s", session_id, msg)
                    continue
                # 过滤掉 GStreamer INFO 级别噪声，只转写含关键字的行
                if any(kw in msg for kw in (
                        "ERROR", "WARN", "Critical", "Cannot", "Failed",
                        "refused", "busy", "No such", "not found",
                        "Could not", "Unable")):
                    logger.warning("[GStreamer session=%s] %s", session_id, msg)
                else:
                    logger.debug("[GStreamer] %s", msg)
        except Exception:  # noqa: BLE001
            pass
        finally:
            try:
                proc.stderr.close()
            except Exception:  # noqa: BLE001
                pass

    def _kill_stream_proc(self, proc):
        """不持锁的写线程专用：强制终止卡死的 GStreamer 子进程。

        不能直接调 `_shutdown_proc_locked`（那个要求调用方已持 `_video_lock`，而写
        线程本身不持锁，强拿锁会与 `start_stream`/`stop_stream` 的锁使用产生不确定
        的时序依赖）。这里只负责杀进程 + 关管道，不修改 `self.rtmp_proc`/`self._stream_session`
        本身（那些字段留给下一次持锁的 `start_stream`/`stop_stream` 在 `proc.poll()`
        已非 None 后自行清理）。
        """
        try:
            if proc.stdin:
                proc.stdin.close()
        except Exception:  # noqa: BLE001
            pass
        try:
            proc.terminate()
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:  # noqa: BLE001
            proc.kill()
        except Exception:  # noqa: BLE001
            pass

    def _shutdown_proc_locked(self):
        """终止推流子进程 + 销毁 image subscriber（调用方必须已持 _video_lock）"""
        # 停止 ROS image 订阅
        self._video_stop_event.set()
        if self._image_sub and self.ros_node:
            try:
                self.ros_node.destroy_subscription(self._image_sub)
            except Exception:  # noqa: BLE001
                pass
            self._image_sub = None
        # 终止 GStreamer 进程
        proc, self.rtmp_proc, self._stream_session = self.rtmp_proc, None, None
        if proc and proc.poll() is None:
            # 先关 stdin 让 fdsrc 收到 EOF 自然退出
            if proc.stdin:
                try:
                    proc.stdin.close()
                except Exception:  # noqa: BLE001
                    pass
            proc.terminate()
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:  # noqa: BLE001
                proc.kill()

    def _start_ros_image_feed(self):
        """启动 ROS image topic 订阅 + 独立写线程，将帧数据持续写入 GStreamer stdin。"""
        if not HAS_IMAGE_MSG or not self.ros_node or not HAS_ROS:
            logger.error("无法启动 ROS image 推流：sensor_msgs=%s ros_node=%s HAS_ROS=%s",
                         HAS_IMAGE_MSG, self.ros_node, HAS_ROS)
            return
        topic = self.config.get("image_topic", "/camera/camera/color/image_raw")
        # 使用 SENSOR_DATA QoS（best-effort, depth=5）匹配相机节点发布端
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        qos = QoSProfile(depth=3, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._image_sub = self.ros_node.create_subscription(
            RosImage, topic, self._on_image_frame, qos)
        # 启动独立写线程（daemon，随进程退出）
        self._drop_warned = False
        self._convert_logged = False
        # 清空上一会话可能残留的旧帧，避免新会话首帧写到过期内容
        try:
            while True:
                self._frame_queue.get_nowait()
        except queue.Empty:
            pass
        self._frame_writer_thread = threading.Thread(
            target=self._frame_writer_loop, daemon=True, name="gst-writer")
        self._frame_writer_thread.start()
        logger.info("[视频] ROS image topic 订阅已建立：%s（写线程已启动）", topic)

    def _on_image_frame(self, msg):
        """收到 ROS Image 帧，转成与推流 caps 完全一致的 rgb8 字节后非阻塞入队。

        V0.1.19 关键修复：旧版直接在 ROS 执行器线程写 proc.stdin，当 RTMP 未就绪时
        921KB 大帧写满 64KB 管道缓冲区后阻塞，导致 20Hz timer 饥饿。
        现在回调只做转换 + queue.put_nowait()，微秒级完成，不阻塞执行器。

        V0.1.20 关键修复（“灰→绿”根因）：旧版对 encoding != rgb8 只做警告就
        把原始字节直接透传给 fdsrc（“尝试原始转发，可能花屏”），一旦 RealSense 实际
        输出非 rgb8（如 bgr8/rgba8/mono8/bayer）或行距 msg.step 带填充字节，固定
        RGB24 caps 会把错位的字节当成像素内容误读，典型表现就是整体偏绿/偏品红。现
        在强制走 _convert_frame：要么产出与 caps 字节数完全一致的 rgb8，要么直接丢帧
        （绝不写入半成品/未知内容）。同时队列满时改为“丢旧留新”，避免卡住时反复
        写同一帧旧数据。
        """
        if self._video_stop_event.is_set():
            return
        proc = self.rtmp_proc
        if not proc or proc.poll() is not None:
            return
        rgb_bytes = self._convert_frame(msg)
        if rgb_bytes is None:
            return  # _convert_frame 内部已限频记录原因，不阻断后续帧
        try:
            self._frame_queue.put_nowait(rgb_bytes)
        except queue.Full:
            # 队列里还压着上一帧未写完：丢旧留新，保证画面不卡在过期内容上
            try:
                self._frame_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._frame_queue.put_nowait(rgb_bytes)
            except queue.Full:
                pass
            if not getattr(self, '_drop_warned', False):
                self._drop_warned = True
                logger.warning("[视频] 帧队列满（RTMP 未就绪？），开始丢旧帧留新帧")

    def _convert_frame(self, msg):
        """把 ROS Image 帧转成与推流 caps 字节数完全一致的 rgb8 字节串。

        必须同时处理三件事，缺一就会把画面“写坏”（历史上表现为整体偏绿）：
          1. encoding：非 rgb8 时必须真正转换通道顺序/位数，不能只警告就透传；
          2. step（行距）：部分驱动每行末尾有填充字节，必须按 step 取行、截掉
             src_w*channels 之后的填充，不能假定 len(data)==width*height*channels；
          3. 分辨率：源图尺寸与推流 caps 不一致时缩放对齐，否则 fdsrc 会把一帧
             拆成多帧或把多帧内容错位合并。

        任一步骤失败（数据截断/未知编码且 cv_bridge 不可用/依赖缺失）都返回 None，
        由调用方直接丢帧——绝不往 fdsrc 写不确定内容。
        """
        width, height = self._stream_size
        src_w, src_h = int(msg.width), int(msg.height)
        enc = (msg.encoding or "").lower()
        step = int(msg.step) or (src_w * 3)
        if not HAS_NUMPY:
            if not getattr(self, '_dep_warned', False):
                self._dep_warned = True
                logger.error("[视频] numpy 不可用，无法安全转换 ROS 图像，已停止写帧")
            return None

        raw = bytes(msg.data)  # 必须拷贝：msg.data 可能是 memoryview，回调返回后 DDS 缓冲区会被回收

        known_channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1, "8uc1": 1}
        arr = None
        if enc in known_channels:
            ch = known_channels[enc]
            if step < src_w * ch:
                logger.warning("[视频] step=%d < width*channels=%d，数据异常，丢帧", step, src_w * ch)
                return None
            need = step * src_h
            if len(raw) < need:
                logger.warning("[视频] 数据截断：len=%d < step*height=%d，丢帧", len(raw), need)
                return None
            buf = np.frombuffer(raw[:need], dtype=np.uint8).reshape(src_h, step)
            buf = buf[:, :src_w * ch].reshape(src_h, src_w, ch)
            if enc == "rgb8":
                arr = buf
            elif enc == "bgr8":
                arr = buf[:, :, ::-1]
            elif enc == "rgba8":
                arr = buf[:, :, :3]
            elif enc == "bgra8":
                arr = buf[:, :, (2, 1, 0)]
            else:  # mono8 / 8uc1
                arr = np.repeat(buf, 3, axis=2)
        elif self._cv_bridge is not None:
            try:
                img = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
                arr = np.asarray(img, dtype=np.uint8)
            except Exception as exc:  # noqa: BLE001
                logger.warning("[视频] cv_bridge 转换 encoding=%s 失败，丢帧：%s", enc, exc)
                return None
        else:
            if not getattr(self, '_enc_warned', False):
                self._enc_warned = True
                logger.error("[视频] encoding=%s 不在已知通道表且 cv_bridge 不可用，"
                             "不再透传原始字节（避免写坏颜色），已丢帧", enc)
            return None

        if arr is None or arr.ndim != 3 or arr.shape[2] != 3:
            logger.warning("[视频] 转换结果形状异常：%s，丢帧", getattr(arr, "shape", None))
            return None

        if (src_w, src_h) != (width, height):
            if not HAS_CV2:
                logger.error("[视频] 源尺寸(%dx%d)≠推流尺寸(%dx%d) 且 opencv 不可用，无法缩放，丢帧",
                             src_w, src_h, width, height)
                return None
            # cv2.resize 要求内存连续（bgr8 翻转等分支得到的是非连续视图），先拷贝再缩放
            arr = cv2.resize(np.ascontiguousarray(arr), (width, height),
                             interpolation=cv2.INTER_AREA)

        rgb_bytes = np.ascontiguousarray(arr, dtype=np.uint8).tobytes()
        if not self._convert_logged:
            self._convert_logged = True
            logger.info(
                "[视频] 首帧转换: encoding=%s src=%dx%d step=%d → rgb8 %dx%d len=%d "
                "raw[0:3]=%s rgb[0:3]=%s（raw 与 rgb 不一致说明确实做了通道/尺寸修正；"
                "若 raw[0:3] 本身就是绿色系，则问题在相机/驱动侧而非本节点）",
                enc, src_w, src_h, step, width, height, len(rgb_bytes),
                list(raw[:3]), list(rgb_bytes[:3]))

        return rgb_bytes

    def _frame_writer_loop(self):
        """独立写线程：从 _frame_queue 取帧写入 GStreamer stdin。
        阻塞在 queue.get() 上不影响任何 ROS 回调。

        V0.1.21 关键修复（“首帧后断供”根因）：旧版直接 `proc.stdin.write(data)`，
        一旦 GStreamer 子进程因 RTMP 握手/连接卡住而尚未进入 PLAYING（`fdsrc` 不
        从管道读数据），Linux 默认 64KB 管道缓冲写满后 `write()` 会永久阻塞——
        本线程从此卡死在首帧，队列堆满后 ROS 回调只能不断“丢旧留新”，浏览器
        因长时间收不到新帧而显示绿色兜底（与内容对不对色无关，是“没数据”时的
        通用占位色）。现在写前用 `select` 探测管道是否可写，超时则判定推流失败：
        杀进程 + 置停止位，下一次 boot 帧可重新起流（不靠本线程无限重试）。
        """
        first_frame = True
        frames_written = 0
        write_timeout = float(self.config.get("fdsrc_write_timeout_s", 5.0))
        while not self._video_stop_event.is_set():
            try:
                data = self._frame_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            proc = self.rtmp_proc
            if not proc or proc.poll() is not None:
                continue
            # 首帧诊断日志：确认数据长度和内容有效性
            if first_frame:
                first_frame = False
                logger.info("[视频] 首帧待写入: type=%s len=%d bytes[0:3]=%s",
                            type(data).__name__, len(data),
                            list(data[:3]) if len(data) >= 3 else 'N/A')
            # 带超时的分块写入：单次 select() 只能保证“当下管道有空位”，不能
            # 保证写完整个 921KB 不阻塞（写一半时管道重新写满仍会卡住），因此必须
            # 逐块 select + os.write，并在整帧维度上把总耗时限制在 write_timeout 内。
            ok = self._write_with_timeout(proc.stdin, data, write_timeout)
            if not ok:
                logger.error(
                    "[视频] fdsrc 管道 %0.1fs 内未能写完一帧（GStreamer 未进入 PLAYING？"
                    "RTMP 握手/连接可能卡住），判定推流失败，终止子进程等待下次 boot 帧重试",
                    write_timeout)
                self._kill_stream_proc(proc)
                self._video_stop_event.set()
                return
            frames_written += 1
            # 心跳日志：每 150 帧（约 10s@15fps）确认写线程仍在推进，不只在首帧打日志
            if frames_written % 150 == 0:
                logger.info("[视频] 推流写入心跳：累计已写 %d 帧", frames_written)

    @staticmethod
    def _write_with_timeout(fh, data, timeout):
        """向 fh（子进程 stdin）写满 data，整帧总耗时不超过 timeout 秒。

        返回 True=写完，False=超时或管道已断（调用方应视为推流失败并重启子进程）。

        必须先把 fd 设为非阻塞（`os.set_blocking(fd, False)`）：阻塞模式下
        `os.write(fd, buf)` 会卡在内核里直到 buf 全部写完才返回，完全无视
        `select` 只确认了“当下有空位”这个事实——只要下游消费得慢，这一句
        `os.write()` 就能阻塞几十秒而不触发任何超时。非阻塞后 `os.write()` 会
        立即返回实际写入的字节数（可能少于请求量），循环才能按 `deadline`
        逐轮重新 `select`+写入，真正把整帧耗时限制在 timeout 内。
        """
        deadline = time.monotonic() + timeout
        fd = fh.fileno()
        view = memoryview(data)
        offset = 0
        total = len(view)
        try:
            os.set_blocking(fd, False)
        except OSError:  # noqa: BLE001
            pass
        try:
            while offset < total:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                try:
                    _, writable, _ = select.select([], [fd], [], remaining)
                except (OSError, ValueError):  # noqa: BLE001
                    return False
                if not writable:
                    return False
                try:
                    n = os.write(fd, view[offset:])
                except BlockingIOError:
                    # 非阻塞下管道当前已满（极少见，select 刚说可写就被占满），重新循环等下一轮
                    continue
                except (BrokenPipeError, OSError):  # noqa: BLE001
                    return False
                if n <= 0:
                    return False
                offset += n
            return True
        finally:
            try:
                os.set_blocking(fd, True)
            except OSError:  # noqa: BLE001
                pass

    def _enlarge_pipe_buffer(self, fh):
        """V0.1.24：把子进程 stdin 管道容量扩到至少能装下 2 帧（默认 64KB 装不下
        一帧完整的 RGB 数据，导致 fdsrc 每次 read() 只能读到半帧，见 start_stream
        处注释）。失败不阻断启动，仅记录警告（降级为系统默认容量）。
        """
        size = getattr(self, "_stream_size", None)
        if not size:
            return
        frame_bytes = size[0] * size[1] * 3
        # 目标：至少能容纳 1 帧完整数据 + 一点余量，不必强求 2 帧——因为
        # 2 帧（1843200 字节）已超过 /proc/sys/fs/pipe-max-size 默认值（1048576），
        # 普通进程无权申请到那么大；1 帧 + 余量（987136 字节）刚好在默认上限内。
        want = frame_bytes + 65536
        F_SETPIPE_SZ = getattr(fcntl, "F_SETPIPE_SZ", 1031)  # Linux 专有，非 Linux 上不存在
        try:
            fcntl.fcntl(fh.fileno(), F_SETPIPE_SZ, want)
            logger.info("[视频] 管道容量已扩至 %d 字节（>= 1 帧 = %d 字节），避免 fdsrc 短读", want, frame_bytes)
        except OSError as exc:
            # 超出 /proc/sys/fs/pipe-max-size（默认 1048576）时会报 EINVAL/EPERM
            logger.warning("[视频] 无法扩大管道容量至 %d 字节（%s），将使用系统默认容量，"
                           "若仍报 'invalid buffer size' 请提高 /proc/sys/fs/pipe-max-size", want, exc)

    # ------------------------------------------------------------------
    # 主入口
    # ------------------------------------------------------------------
    def run(self):
        self.running = True
        self.resolve_vehicle_id()
        self.init_ros()
        self.init_kafka()
        logger.info("Remote Agent 启动：vehicle_id=%s，遥控通道=Kafka(%s)+WS(%s)，"
                    "视频=会话制 RTMP 推流（boot 帧起）",
                    self.vehicle_id,
                    "on" if self.kafka_consumer else "off",
                    "on" if (HAS_WS and self.config.get("enable_ws", False)) else "off")
        try:
            # V0.1.15：推流不再开机即起，改由 remote_control boot 帧触发（会话制）；
            # 主阻塞点：启用备用 WS 时跑 WS 循环（含断线重连），否则空转保活，
            # Kafka 消费/20Hz 发布/超时看门狗均在独立线程。
            if self.config.get("enable_ws", False):
                self.websocket_loop()
            else:
                while self.running:
                    time.sleep(1.0)
        finally:
            self.running = False
            self.stop_stream(reason="agent_shutdown")
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
            if self.rc_result_producer:
                self.rc_result_producer.flush(3.0)
        except Exception as exc:  # noqa: BLE001
            logger.warning("command_result producer flush 异常：%s", exc)
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
    "camera_device": "/dev/video0",          # D435 RGB（文档 13.2.1，仅 v4l2 模式用）
    "enable_video": True,
    # ── 视频采集模式（V0.1.19 新增）──
    # "ros_topic"：订阅 ROS image topic（RealSense 被 ROS 节点占用时的唯一可用方案）
    # "v4l2"：直接 v4l2src 读设备（专用 USB 摄像头，不与其他节点冲突时使用）
    "camera_source": "ros_topic",
    "image_topic": "/camera/camera/color/image_raw",  # V0.1.19 修正：RealSense 节点名 /camera/camera 带双层 namespace
    "stream_width": 640,                       # ros_topic 模式的推流分辨率
    "stream_height": 480,
    "prefer_hw_encode": False,                  # Jetson 硬编（nvv4l2h264enc），部分 JetPack 不存在默认关
    # ── 视频链路（V1.21.0：RTMP 推流 → SRS → 浏览器 WHEP 拉流）──
    "srs_rtmp_base_url": "",                  # 例 rtmp://192.168.31.35:1935/remote_control
    "srs_stream_prefix": "hunter",            # = 平台 RC_SRS_STREAM_PREFIX（契约不可自创）
    "video_fps": 15,                          # 匹配 RealSense D435 实际输出 640x480@15fps
    "video_bitrate_kbps": 3000,               # 回落码率（契约 2048~4096 kbps）
    "fdsrc_write_timeout_s": 5.0,             # V0.1.21：写 fdsrc 管道时的整帧 select+os.write 超时，超时判定推流失败并重启
    "gst_debug": "",                          # V0.1.21：非空（如 "2"~"4"）时传给子进程 GST_DEBUG 并把 stderr 全量转 INFO，排查 RTMP 握手卡住用
    "ws_url": "",                             # 文档 17.2 备用通道（例 ws://<SERVER_IP>/ws/remote）
    "enable_ws": False,                       # 主通道为 Kafka；备用 WS 需平台 JWT，默认关
    "cmd_topic": "/remote/command",           # 文档 13.4
    "ack_command_result": True,               # 逐帧回投 command_result（平台 500ms 无回执判异常）
    # Kafka 接入只保留路径（凭据与 mTLS 证书在 /etc/hunter/kafka/kafka.properties）
    "kafka_properties": DEFAULT_PROPERTIES_PATH,
    "kafka_bundle_dir": DEFAULT_BUNDLE_DIR,
    "max_velocity": 2.0,                      # 文档 13.4：远程限速 2.0 m/s
    "max_steering": 0.4,                      # 文档 4.3：转向 ±0.4 rad
    "cmd_timeout": 3.0,                       # V0.1.19 调高：平台 Kafka 实际送达 ~2s/帧（UI 20Hz 但 acks=all 延迟），3s 容兆底
    "remote_cmd_ttl": 4.0,                    # V0.1.19: 配合 cmd_timeout=3.0，避免 Kafka 延迟抖动误判过期
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
