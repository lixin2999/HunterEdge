# -*- coding: utf-8 -*-
r"""command_agent — HunterCore 平台下行指令接入（文档 14/16 章 + 接入包 Topic 契约）。

契约（HunterCore 接入包 README/kafka.properties，8 Topic/车）：
  下行 ``hunter.<vehicle_id>.command``         平台 → 车：指令
  上行 ``hunter.<vehicle_id>.command_result``  车 → 平台：执行回执（acks=all）

指令 → 车端动作**全部映射到既有 ROS2 接口**（.ai‑rules：禁止新增话题/消息字段）：

| 指令 type        | 车端动作                                                          |
|------------------|-------------------------------------------------------------------|
| MISSION_START    | 服务 /auto_mission/start_mapping_cruise（建图自动巡航，含航点热重载） |
| MISSION_STOP     | 服务 /auto_mission/stop_mapping_cruise                              |
| MAP_CONVERT      | 服务 /pcd_to_map/convert（建图产物 → .pgm/.yaml）                   |
| WAYPOINT_SAVE    | 服务 /waypoint_recorder/save                                        |
| WAYPOINT_CLEAR   | 服务 /waypoint_recorder/clear（需 params.confirm=true）            |
| ESTOP_ENGAGE     | /estop = true（锁存重发，直到 RELEASE）                            |
| ESTOP_RELEASE    | /estop = false（需车辆静止）                                        |
| TEST_MODE_ON/OFF | /safety/test_mode = true/false（0.1m/s 限速 + 异常即中止）         |
| REMOTE_RELEASE   | /remote/command 发 control_mode="AUTO" 帧 → 交还驾驶权（文档 13.5） |
| STATUS_QUERY     | 仅回车端状态快照，不改变任何状态                                    |
| HEARTBEAT_ACK    | 仅回 SUCCEEDED（平台连通性探测 / last_online_time 刷新）            |

**字段契约对齐（HunterCore contracts/kafka/schemas/command*.schema.json）**：

- 下行信封：契约 required = `[command_id, timestamp, command_type]`，可选
  `params` / `operator_id` / `timeout_ms` / `vehicle_id` / `target_filter`。
  本节点读契约名 `command_type`，**同时兼容**仓内既有/平台部分服务实际使用的
  `type`（remote-control 的 session_command 生产者）与旧字段 `issued_at` / `payload`；
- 下行语义：契约示例 `command_type="enable_auto_driving"` → 需求④“接收到自动驾驶
  指令后进入相应模式”，经 `DEFAULT_COMMAND_TYPE_ALIASES` 映射到车端既有动作
  `MISSION_START`（动作落地仍是既有 ROS 服务，不新增话题/消息）；
- 上行回执：契约 required = `[command_id, vehicle_id, timestamp, success]`，
  可选 `result_code` / `message` / `data`，`additionalProperties=false` ——
  旧版 `{type,status,reason,vehicle_state,ts}` 与契约零交集，平台 data-collector
  会判非法（schema_invalid）并转 DLQ，**平台侧永远看不到指令回执**，故本轮改为契约信封。

安全护栏（顺序即优先级；任一不过则**不落地执行**，只回相应状态）：
  1. 白名单：未登记 type 一律 REJECTED —— 平台侧新增指令不得在车端"默认放行"。
  2. 时效：``expires_at``（或 ``issued_at + command_ttl``）过期的指令绝不执行。
     消费组重建/断网重连都可能把历史指令重新推给车辆，执行一条几分钟前的
     "启动巡航"是实车事故级风险（auto.offset.reset 也已置 latest 双保险）。
  3. 幂等：command_id 去重（LRU），重复投递只回 DUPLICATE。
  4. 门控：急停锁存中 / SystemHealth=CRITICAL / 底盘 ESTOP|FAULT → 拒绝运动类指令。
  5. 确认：破坏性指令（清航点）需 params.confirm=true。
  6. 提交：执行完成 + 回执投递证实后才 commit offset（at-least-once，靠幂等收敛）。

车端执行留痕：每条回执以 INFO 级日志输出完整 JSON（journalctl -u hunter-\* 可核对
平台下发过什么、车端回了什么状态）；不额外发明 ROS 话题（.ai‑rules 禁止新增话题）。
"""
from __future__ import annotations

import collections
import json
import threading
import time
from typing import Any, Callable, Deque, Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from hunter_msgs.msg import ChassisCommand, ChassisState, SystemHealth
from nav_msgs.msg import Odometry

try:
    from hunter_kafka.client import make_consumer, make_producer, topic_name
    from hunter_kafka.config import (DEFAULT_BUNDLE_DIR, DEFAULT_PROPERTIES_PATH,
                                     KafkaConfigError)
    HAS_HUNTER_KAFKA = True
except ImportError:                                    # 未 source 公共包 → 指令链路关闭
    HAS_HUNTER_KAFKA = False
    DEFAULT_BUNDLE_DIR = "/etc/hunter/kafka"
    DEFAULT_PROPERTIES_PATH = "/etc/hunter/kafka/kafka.properties"


class Status:
    ACCEPTED = "ACCEPTED"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"
    REJECTED = "REJECTED"
    EXPIRED = "EXPIRED"
    DUPLICATE = "DUPLICATE"
    TIMEOUT = "TIMEOUT"


#: 运动类指令（急停/严重故障时拒绝）
MOTION_COMMANDS = frozenset({"MISSION_START", "TEST_MODE_ON", "REMOTE_RELEASE"})
#: 破坏性指令（需 params.confirm=true）
DESTRUCTIVE_COMMANDS = frozenset({"WAYPOINT_CLEAR"})

#: 车端执行状态 → HunterCore 契约 result_code（command_result.schema.json：
#: result_code 为字符串，"0"/SUCCESS 表示成功，其余为平台错误码表或车端自定义码）
RESULT_CODE_BY_STATUS = {
    Status.SUCCEEDED: "0",
    Status.ACCEPTED: "0",
    Status.DUPLICATE: "0",     # 幂等：同 command_id 重复投递，效果与成功一致（不重复执行）
    Status.REJECTED: "4001",   # 车端拒绝（白名单/门禁/确认缺失）
    Status.FAILED: "4002",     # 执行失败
    Status.TIMEOUT: "4003",    # 车端执行超时
    Status.EXPIRED: "4004",    # 指令过期
}

#: HunterCore 契约 command_type → 车端内部动作。
#: ⚠ 平台侧 `command_type` 取值域**尚未定稿**（HunterCore 契约 pending #17：
#: “取值域待设计文档 5.3 节核对，当前不设 enum”）。此处只固化两处有据可依的锚点：
#:   ① 契约示例 `enable_auto_driving`（command.schema.json examples[0]）——
#:      即需求④“车辆接收到自动驾驶的指令后进入相应模式”；
#:   ② 本仓既有内部动作名（MISSION_START/MISSION_STOP/…）保持可用。
#: 其余取值经参数 `command_type_aliases` 注入，**无需改代码**（禁止硬编码分支）。
DEFAULT_COMMAND_TYPE_ALIASES = {
    "enable_auto_driving": "MISSION_START",    # 进入自动驾驶（启动自主任务）
    "disable_auto_driving": "MISSION_STOP",    # 退出自动驾驶（停止自主任务）
}

#: ROS 参数形态（rcl_yaml_param_parser 只支持标量/标量数组，嵌套 map 不可用）：
#: 以 "平台类型=车端动作" 的字符串数组声明，解析见 _parse_type_aliases()
DEFAULT_COMMAND_TYPE_ALIAS_LIST = [f"{k}={v}" for k, v in DEFAULT_COMMAND_TYPE_ALIASES.items()]


def parse_type_aliases(entries: Any) -> Dict[str, str]:
    """把 ["enable_auto_driving=MISSION_START", ...] 解析成映射；非法项忽略并告警。"""
    aliases: Dict[str, str] = {}
    if not entries:
        return dict(DEFAULT_COMMAND_TYPE_ALIASES)
    for item in entries:
        if not isinstance(item, str) or "=" not in item:
            continue
        left, _, right = item.partition("=")
        left, right = left.strip(), right.strip()
        if left and right:
            aliases[left] = right
    return aliases or dict(DEFAULT_COMMAND_TYPE_ALIASES)


class CommandAgent(Node):
    """ROS2 节点：Kafka 指令消费 ↔ 车端服务/话题，回执 command_result。"""

    def __init__(self):
        super().__init__("command_agent")

        # ---------------- 参数 ----------------
        self.declare_parameter("vehicle_id", "")                 # 空 → 取 kafka.properties 的 SASL 用户名
        self.declare_parameter("kafka_properties", DEFAULT_PROPERTIES_PATH)
        self.declare_parameter("kafka_bundle_dir", DEFAULT_BUNDLE_DIR)
        self.declare_parameter("command_group_suffix", "command")
        self.declare_parameter("command_ttl", 10.0)               # 无 expires_at 时的时效窗口（s）
        self.declare_parameter("poll_interval", 0.05)             # Kafka poll 周期（s）
        self.declare_parameter("service_timeout", 10.0)           # ROS 服务调用超时（s）
        self.declare_parameter("estop_repeat_rate", 2.0)          # 急停锁存重发频率（Hz）
        self.declare_parameter("dedup_cache_size", 512)
        self.declare_parameter("allow_unknown_types", False)      # 生产必须 false
        self.declare_parameter("extra_allowed_types", [])         # 现场临时放开（仍走同一护栏）
        self.declare_parameter("static_velocity_threshold", 0.05)
        self.declare_parameter("result_retry_max", 200)
        # 契约 command_type → 车端内部动作（"平台类型=车端动作" 字符串数组；
        # rcl_yaml_param_parser 不支持嵌套 map，故用数组形态）。
        # 平台 command_type 取值域定稿后只改 YAML，不动代码。
        self.declare_parameter("command_type_aliases", DEFAULT_COMMAND_TYPE_ALIAS_LIST)
        # 接入包尚未部署时的内联参数（仅开发机；生产走 properties 单一可信源）
        self.declare_parameter("kafka_fallback_brokers", "")
        self.declare_parameter("kafka_fallback_username", "")
        self.declare_parameter("kafka_fallback_password", "")

        self._vehicle_id = str(self.get_parameter("vehicle_id").value or "")
        self._properties = str(self.get_parameter("kafka_properties").value)
        self._bundle_dir = str(self.get_parameter("kafka_bundle_dir").value)
        self._group_suffix = str(self.get_parameter("command_group_suffix").value)
        self._ttl = float(self.get_parameter("command_ttl").value)
        self._service_timeout = float(self.get_parameter("service_timeout").value)
        self._dedup_size = int(self.get_parameter("dedup_cache_size").value)
        self._allow_unknown = bool(self.get_parameter("allow_unknown_types").value)
        self._static_thr = float(self.get_parameter("static_velocity_threshold").value)
        self._retry_max = int(self.get_parameter("result_retry_max").value)
        # 契约 command_type 别名表（YAML 未配置时用内置锚点）
        self._type_aliases = parse_type_aliases(
            self.get_parameter("command_type_aliases").value)

        # ---------------- 车端状态缓存（门控依据）----------------
        self._lock = threading.RLock()
        self._health: Optional[SystemHealth] = None
        self._chassis: Optional[ChassisState] = None
        self._odom: Optional[Odometry] = None
        self._mission_state = ""
        self._estop_latched = False

        self.create_subscription(SystemHealth, "/system/health", self._health_cb, 10)
        self.create_subscription(
            ChassisState, "/chassis/state", self._chassis_cb,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Odometry, "/localization/odom", self._odom_cb, 10)
        self.create_subscription(String, "/auto_mission/status", self._mission_cb, 10)

        # ---------------- 下行执行通道（均为既有接口）----------------
        self._estop_pub = self.create_publisher(Bool, "/estop", 10)
        self._test_mode_pub = self.create_publisher(Bool, "/safety/test_mode", 10)
        self._remote_cmd_pub = self.create_publisher(ChassisCommand, "/remote/command", 10)

        self._clients: Dict[str, Any] = {
            "MISSION_START": self.create_client(Trigger, "/auto_mission/start_mapping_cruise"),
            "MISSION_STOP": self.create_client(Trigger, "/auto_mission/stop_mapping_cruise"),
            "MAP_CONVERT": self.create_client(Trigger, "/pcd_to_map/convert"),
            "WAYPOINT_SAVE": self.create_client(Trigger, "/waypoint_recorder/save"),
            "WAYPOINT_CLEAR": self.create_client(Trigger, "/waypoint_recorder/clear"),
        }

        # ---------------- 幂等 / 在途 / 补投 ----------------
        self._dedup: Deque[str] = collections.deque(maxlen=self._dedup_size)
        self._seen: set = set()
        self._settled: set = set()                 # 已发终态回执的 command_id（防超时+迟到双回执）
        self._settled_order: Deque[str] = collections.deque(maxlen=self._dedup_size)
        self._inflight: Optional[Dict[str, Any]] = None      # 同一时刻只允许 1 条服务指令在途
        self._retry_results: Deque[Dict[str, Any]] = collections.deque(maxlen=self._retry_max)

        # ---------------- Kafka ----------------
        self._consumer = None
        self._producer = None
        self._kafka_ready = False
        self._kafka_err = ""
        self._init_kafka()

        # ---------------- 定时器 ----------------
        self.create_timer(float(self.get_parameter("poll_interval").value), self._poll_kafka)
        self.create_timer(1.0 / max(0.5, float(self.get_parameter("estop_repeat_rate").value)),
                          self._estop_hold_step)
        self.create_timer(1.0, self._flush_retry_results)
        # 常驻看门狗：检查在途服务指令是否超时（不能每次调用新建 timer，否则定时器堆积）
        self.create_timer(0.5, self._watch_inflight)

        self.get_logger().info('command_agent 启动：vehicle_id=%s，指令链路=%s，白名单=%d 项%s' % (self._vehicle_id or "(未知)", "Kafka 已就绪" if self._kafka_ready else f"降级({self._kafka_err or 'hunter_kafka 不可用'})", len(self._allowed_types()), "，⚠allow_unknown_types=true" if self._allow_unknown else ""))

    # ==================================================================
    # 订阅回调（只更新缓存，不做业务）
    #
    # ⚠ 这四个回调**必须存在**：__init__ 里的 create_subscription 直接引用了它们。
    #   一旦缺失就是「启动即崩」——`AttributeError: 'CommandAgent' object has no
    #   attribute '_health_cb'`（HUNTER-001 实机日志确认：command_agent 一直卡在这里，
    #   表现为 `ros2 node list` 里没有 command_agent、平台指令永远无回执）。
    #   缓存用途：_motion_gate()（急停/故障门控）与 _snapshot()（回执里带车端状态）。
    # ==================================================================
    def _health_cb(self, msg: SystemHealth) -> None:
        with self._lock:
            self._health = msg

    def _chassis_cb(self, msg: ChassisState) -> None:
        with self._lock:
            self._chassis = msg

    def _odom_cb(self, msg: Odometry) -> None:
        with self._lock:
            self._odom = msg

    def _mission_cb(self, msg: String) -> None:
        # /auto_mission/status（std_msgs/String，形如「状态|原因」，见 auto_mission 文档）
        with self._lock:
            self._mission_state = msg.data

    # ==================================================================
    # Kafka 链路初始化
    # ==================================================================
    def _init_kafka(self) -> None:
        if not HAS_HUNTER_KAFKA:
            self._kafka_err = "hunter_kafka 未安装/未 source"
            self.get_logger().error(
                "hunter_kafka 公共包不可用：请先 source ~/HunterEdge/install/setup.bash"
                "（否则平台指令链路不会开启）")
            return
        try:
            if not self._vehicle_id:
                from hunter_kafka.config import load_properties, resolve_vehicle_id
                self._vehicle_id = resolve_vehicle_id(load_properties(self._properties))
            fallback = self._fallback_conf()
            self._producer = make_producer(
                self._vehicle_id, "command_result",
                properties_path=self._properties, bundle_dir=self._bundle_dir,
                dr_callback=self._on_result_dr, fallback=fallback)
            self._consumer = make_consumer(
                self._vehicle_id, "command",
                group_suffix=self._group_suffix,
                properties_path=self._properties, bundle_dir=self._bundle_dir,
                # 车端首次上线/重建消费组时不回灌历史指令（与 TTL 校验构成双保险）
                auto_offset_reset="latest",
                fallback=fallback)
            self._kafka_ready = True
        except KafkaConfigError as exc:
            self._kafka_err = "配置错误"
            self.get_logger().fatal('指令链路配置失败：%s（核对 %s 的 SCRAM 口令、证书权限 0600、pip3 install confluent-kafka）' % (exc, self._properties))
        except Exception as exc:  # noqa: BLE001
            self._kafka_err = "初始化异常"
            self.get_logger().fatal('指令链路初始化异常：%s' % (exc,))

    def _fallback_conf(self) -> Optional[Dict[str, str]]:
        brokers = str(self.get_parameter("kafka_fallback_brokers").value or "")
        if not brokers:
            return None
        self.get_logger().warn('使用 YAML 内联 Kafka 参数（仅限开发机；生产请部署接入包到 %s）' % (self._properties,))
        return {
            "bootstrap.servers": brokers,
            "security.protocol": "sasl_ssl",
            "sasl.mechanism": "SCRAM-SHA-512",
            "sasl.username": str(self.get_parameter("kafka_fallback_username").value or ""),
            "sasl.password": str(self.get_parameter("kafka_fallback_password").value or ""),
        }

    # ==================================================================
    # 消费循环
    # ==================================================================
    def _poll_kafka(self) -> None:
        if not self._kafka_ready:
            return
        self._producer.poll(0.0)                 # 驱动 on_delivery（回执送达证实）
        try:
            messages = self._consumer.consume(num_messages=1, timeout=0.0)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error('Kafka consume 异常：%s' % (exc,))
            return
        for msg in messages or []:
            if msg.error():
                self.get_logger().error('Kafka 消费错误：%s' % (msg.error(),))
                continue
            self._on_command_message(msg)

    def _on_command_message(self, msg) -> None:
        try:
            cmd = json.loads(msg.value().decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            # 非法报文无法执行也无法回执关联，直接提交 offset 丢弃（防堵分区）
            self.get_logger().warn('指令报文非法，丢弃：%s' % (exc,))
            self._commit(msg)
            return
        if not isinstance(cmd, dict):
            self.get_logger().warn("指令报文不是 JSON 对象，丢弃")
            self._commit(msg)
            return

        self.get_logger().info('收到平台指令：%s' % (self._brief(cmd),))
        result = self._guard(cmd)
        if result is not None:                   # 护栏拦截，没有进入执行
            self._finish(cmd, msg, result)
            return
        self._dispatch(cmd, msg)

    # ------------------------------------------------------------------
    # 契约字段归一：command.schema.json 用 command_type/timestamp/params/timeout_ms；
    # remote-control 的 session_command 生产者仍发 type/issued_at/payload；
    # 两者都必须能被同一套护栏与执行链处理（兼容而非二选一）。
    # ------------------------------------------------------------------
    def _command_type(self, cmd: Dict[str, Any]) -> str:
        """取指令类型：契约 `command_type` 优先，回落旧名 `type`，再经别名表映射。"""
        raw = cmd.get("command_type")
        if not isinstance(raw, str) or not raw.strip():
            raw = cmd.get("type")
        if not isinstance(raw, str) or not raw.strip():
            return ""
        name = raw.strip()
        return self._type_aliases.get(name, name)

    @staticmethod
    def _command_params(cmd: Dict[str, Any]) -> Dict[str, Any]:
        """取指令参数：契约 `params` 优先，回落旧名 `payload`。"""
        for key in ("params", "payload"):
            value = cmd.get(key)
            if isinstance(value, dict):
                return value
        return {}

    def _command_issued_at(self, cmd: Dict[str, Any]) -> Optional[float]:
        """指令签发时刻：`timestamp`（契约，自 remote-control 的 issued_at 向后兼容）。"""
        for key in ("timestamp", "issued_at"):
            value = cmd.get(key)
            if isinstance(value, (int, float)) and value > 0:
                return float(value)
        return None

    # ------------------------------------------------------------------
    # 护栏链：返回 None 表示放行
    # ------------------------------------------------------------------
    def _guard(self, cmd: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        ctype = self._command_type(cmd)
        cid = cmd.get("command_id")
        if not ctype:
            return self._result(cmd, Status.REJECTED,
                                "缺少 command_type（契约字段）或 type 字段")
        if not cid:
            return self._result(cmd, Status.REJECTED,
                                "缺少 command_id（车端无法做幂等去重与回执关联）")
        if ctype not in self._allowed_types() and not self._allow_unknown:
            return self._result(cmd, Status.REJECTED,
                                f"指令 {ctype} 不在车端白名单；确需放开请升级车端版本或临时配置 "
                                f"extra_allowed_types")
        if not self._is_fresh(cmd):
            return self._result(cmd, Status.EXPIRED,
                                f"指令已过期（timestamp={self._command_issued_at(cmd)} "
                                f"expires_at={cmd.get('expires_at')}），拒绝回灌执行")
        with self._lock:
            if cid in self._seen:
                return self._result(cmd, Status.DUPLICATE, f"command_id={cid} 已执行过")
        if self._inflight is not None:
            # 上一条服务指令还没返回：不排队、不并发，让平台重试（确定性优先）
            return self._result(cmd, Status.REJECTED,
                                f"上一条指令 {self._command_type(self._inflight['cmd'])} 仍在执行中")
        params = self._command_params(cmd)
        if ctype in DESTRUCTIVE_COMMANDS and params.get("confirm") is not True:
            return self._result(cmd, Status.REJECTED,
                                f"{ctype} 需 params.confirm=true（防误清航点库）")
        gate = self._motion_gate() if ctype in MOTION_COMMANDS else None
        if gate:
            return self._result(cmd, Status.REJECTED, gate)
        # 放行才记入去重表：被拦截的重复投递应继续走 REJECTED 而不是 DUPLICATE
        with self._lock:
            self._remember(cid)
        return None

    def _is_fresh(self, cmd: Dict[str, Any]) -> bool:
        now = time.time()
        exp = cmd.get("expires_at")
        if isinstance(exp, (int, float)) and exp > 0:
            return now <= float(exp)
        issued = self._command_issued_at(cmd)
        if issued is not None:
            # 契约的时效窗口：优先用指令自带 timeout_ms（毫秒），否则用 command_ttl（秒）。
            # 消费组重建/断网重连可能回灌历史指令，执行几分钟前的“启动巡航”是实车事故级风险。
            window = self._ttl
            timeout_ms = cmd.get("timeout_ms")
            if isinstance(timeout_ms, (int, float)) and timeout_ms > 0:
                window = max(self._ttl, float(timeout_ms) / 1000.0)
            return (now - issued) <= window
        self.get_logger().warn(
            "指令未带 timestamp/issued_at/expires_at，无法时效校验（按接收时刻放行）")
        return True

    def _motion_gate(self) -> Optional[str]:
        with self._lock:
            if self._estop_latched:
                return "平台急停锁存生效中，拒绝运动类指令（需先 ESTOP_RELEASE）"
            if self._health is not None and self._health.overall_status == "CRITICAL":
                return "SystemHealth=CRITICAL，拒绝运动类指令"
            if self._chassis is not None and (
                    self._chassis.vehicle_state in ("ESTOP", "FAULT") or
                    self._chassis.control_mode == "ESTOP"):
                return f"底盘状态 {self._chassis.vehicle_state} 不允许启动运动"
        return None

    # ==================================================================
    # 执行分发
    # ==================================================================
    def _dispatch(self, cmd: Dict[str, Any], msg) -> None:
        ctype = self._command_type(cmd)
        params = self._command_params(cmd)
        handler = self._immediate_handlers().get(ctype)
        if handler is not None:
            try:
                result = handler(cmd, params)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error('指令 %s 执行异常：%s' % (ctype, exc))
                result = self._result(cmd, Status.FAILED, f"执行异常：{exc}")
            self._finish(cmd, msg, result)
            return
        client = self._clients.get(ctype)
        if client is None:
            result = self._result(cmd, Status.REJECTED, f"指令 {ctype} 无车端实现")
            self._finish(cmd, msg, result)
            return
        self._call_service(cmd, msg, client)

    def _call_service(self, cmd: Dict[str, Any], msg, client) -> None:
        if not client.service_is_ready():
            result = self._result(
                cmd, Status.FAILED,
                f"服务 {client.srv_name} 不可用（对应节点未启动/未激活）")
            self._finish(cmd, msg, result)
            return

        self._inflight = {"cmd": cmd, "msg": msg, "deadline": time.monotonic() + self._service_timeout}
        future = client.call_async(Trigger.Request())

        def _on_done(fut) -> None:
            inflight = self._inflight
            if inflight is not None and inflight["cmd"] is cmd:
                inflight["future_done"] = True
            try:
                resp = fut.result()
            except Exception as exc:  # noqa: BLE001
                self._finish(cmd, msg, self._result(cmd, Status.FAILED, f"服务调用失败：{exc}"))
                return
            status = Status.SUCCEEDED if resp.success else Status.FAILED
            self._finish(cmd, msg, self._result(cmd, status, resp.message or ""))

        future.add_done_callback(_on_done)
        # 服务迟迟不返回 → 由常驻看门狗发 TIMEOUT 回执（不能让平台无限等，也不能不 commit）

    def _watch_inflight(self) -> None:
        inflight = self._inflight
        if inflight is None or inflight.get("future_done"):
            return
        if time.monotonic() < inflight["deadline"]:
            return
        cmd, msg = inflight["cmd"], inflight["msg"]
        self.get_logger().error('服务调用超时（%.0fs）：%s' % (self._service_timeout, self._brief(cmd)))
        self._finish(cmd, msg, self._result(
            cmd, Status.TIMEOUT, f"车端服务在 {self._service_timeout:.0f}s 内未返回"))

    # ------------------------------------------------------------------
    # 即时型指令（话题/纯本地）
    # ------------------------------------------------------------------
    def _immediate_handlers(self) -> Dict[str, Callable[[Dict[str, Any], Dict[str, Any]], Dict[str, Any]]]:
        return {
            "ESTOP_ENGAGE": self._do_estop_engage,
            "ESTOP_RELEASE": self._do_estop_release,
            "TEST_MODE_ON": lambda c, p: self._do_test_mode(c, True),
            "TEST_MODE_OFF": lambda c, p: self._do_test_mode(c, False),
            "REMOTE_RELEASE": self._do_remote_release,
            "STATUS_QUERY": lambda c, p: self._result(c, Status.SUCCEEDED, "状态快照"),
            "HEARTBEAT_ACK": lambda c, p: self._result(c, Status.SUCCEEDED, "车端在线"),
        }

    def _do_estop_engage(self, cmd, _params) -> Dict[str, Any]:
        with self._lock:
            self._estop_latched = True
        self._estop_pub.publish(Bool(data=True))
        self.get_logger().warn("平台急停已生效（按 estop_repeat_rate 锁存重发 /estop=true）")
        return self._result(cmd, Status.SUCCEEDED, "急停已生效（锁存重发中）")

    def _do_estop_release(self, cmd, _params) -> Dict[str, Any]:
        with self._lock:
            chassis = self._chassis
        if chassis is not None and abs(float(chassis.velocity)) > self._static_thr:
            return self._result(cmd, Status.REJECTED,
                                f"车辆仍在移动（v={float(chassis.velocity):.2f}m/s），"
                                "静止后方可解除急停")
        with self._lock:
            self._estop_latched = False
        self._estop_pub.publish(Bool(data=False))
        self.get_logger().info("平台急停已解除（/estop=false）")
        return self._result(cmd, Status.SUCCEEDED, "急停已解除")

    def _do_test_mode(self, cmd, value: bool) -> Dict[str, Any]:
        self._test_mode_pub.publish(Bool(data=value))
        self.get_logger().info('自动驾驶测试模式切换：/safety/test_mode=%s' % (value,))
        return self._result(cmd, Status.SUCCEEDED, f"/safety/test_mode={value}")

    def _do_remote_release(self, cmd, _params) -> Dict[str, Any]:
        """交还驾驶权：发一帧 control_mode="AUTO" 的 ChassisCommand，
        remote_agent 收到即停止发布 /remote/command → decision_making 判 remote 超时
        → 回 AUTO（文档 13.5 优先级 ESTOP > REMOTE > AUTO）。"""
        zero = ChassisCommand()
        zero.header.stamp = self.get_clock().now().to_msg()
        zero.header.frame_id = "base_link"
        zero.control_mode = "AUTO"
        zero.target_velocity = 0.0
        zero.target_steering = 0.0
        zero.emergency_stop = False
        for _ in range(5):                     # 连发数帧，对端 20Hz 定时器必然读到
            self._remote_cmd_pub.publish(zero)
        return self._result(cmd, Status.SUCCEEDED, "已发送交还帧（control_mode=AUTO）")

    # ==================================================================
    # 回执
    # ==================================================================
    def _finish(self, cmd: Dict[str, Any], msg, result: Dict[str, Any]) -> None:
        if self._inflight is not None and self._inflight["cmd"] is cmd:
            self._inflight = None
        cid = cmd.get("command_id")
        if cid in self._settled:
            # 超时回执已发、服务迟到的回调又来一次：同一 command_id 只允许一条终态回执
            self.get_logger().debug('指令 %s 已回执（%s），忽略重复终态' % (cid, (result.get("data") or {}).get("status")))
            if msg is not None:
                self._commit(msg)
            return
        self._settled.add(cid)
        self._settled_order.append(cid)
        while len(self._settled) > self._dedup_size:
            self._settled.discard(self._settled_order.popleft())
        self.get_logger().info('指令回执：%s' % (self._brief(result),))
        self._publish_result(result)
        if msg is not None:
            self._commit(msg)

    def _result(self, cmd: Dict[str, Any], status: str, reason: str) -> Dict[str, Any]:
        """构造 HunterCore 契约 command_result 回执（command_result.schema.json）。

        required = [command_id, vehicle_id, timestamp, success]，
        optional = [result_code, message, data]，additionalProperties=false ——
        因此车端执行状态/指令类型等附加信息**只能放进 data**（旧版平铺 status/reason/
        type/vehicle_state 会被平台判为非法报文 → DLQ，运营端看不到任何回执）。
        """
        success = status in (Status.SUCCEEDED, Status.ACCEPTED, Status.DUPLICATE)
        return {
            "command_id": cmd.get("command_id"),
            "vehicle_id": self._vehicle_id,
            "timestamp": time.time(),
            "success": success,
            "result_code": RESULT_CODE_BY_STATUS.get(status, "4000"),
            "message": f"[{status}] {reason}"[:512],
            "data": {
                "status": status,
                "command_type": self._command_type(cmd),
                "vehicle": self._snapshot(),
            },
        }

    def _publish_result(self, result: Dict[str, Any]) -> None:
        payload = json.dumps(result, ensure_ascii=False)
        self.get_logger().info('COMMAND_RESULT %s' % (payload,))   # 车端执行留痕（journal）
        if not self._kafka_ready:
            self._retry_results.append(result)
            self.get_logger().warn("指令链路未就绪，回执入补投队列")
            return
        try:
            self._producer.produce(
                topic_name(self._vehicle_id, "command_result"),
                value=payload.encode("utf-8"),
                key=self._vehicle_id.encode("utf-8"),
                on_delivery=self._on_result_dr)
            self._producer.poll(0.0)
        except BufferError:
            self._retry_results.append(result)
            self.get_logger().warn("Kafka 本地队列满，回执入补投队列")

    def _on_result_dr(self, err, _msg) -> None:
        if err is not None:
            self.get_logger().warn('command_result 投递失败：%s' % (err,))

    def _flush_retry_results(self) -> None:
        if not self._kafka_ready or not self._retry_results:
            return
        for _ in range(min(5, len(self._retry_results))):
            self._publish_result(self._retry_results.popleft())

    def _commit(self, msg) -> None:
        try:
            self._consumer.commit(offsets=[msg], asynchronous=False)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error('offset 提交失败：%s' % (exc,))

    # ==================================================================
    # 状态快照与工具
    # ==================================================================
    def _snapshot(self) -> Dict[str, Any]:
        with self._lock:
            snap: Dict[str, Any] = {
                "mission_state": self._mission_state,
                "health": self._health.overall_status if self._health else "",
                "estop": bool(self._estop_latched),
            }
            if self._chassis is not None:
                snap.update({
                    "velocity": round(float(self._chassis.velocity), 3),
                    "battery_soc": round(float(self._chassis.battery_soc), 1),
                    "control_mode": self._chassis.control_mode,
                    "vehicle_state": self._chassis.vehicle_state,
                })
            if self._odom is not None:
                pos = self._odom.pose.pose.position
                snap.update({"pose_x": round(pos.x, 3), "pose_y": round(pos.y, 3)})
            return snap

    def _allowed_types(self) -> Tuple[str, ...]:
        extra = self.get_parameter("extra_allowed_types").value or []
        base = tuple(sorted(self._clients)) + tuple(sorted(self._immediate_handlers()))
        return tuple(sorted(set(base) | {str(t) for t in extra}))

    def _remember(self, cid: str) -> None:
        self._dedup.append(cid)
        self._seen.add(cid)
        while len(self._seen) > self._dedup_size:
            self._seen.discard(self._dedup.popleft())

    @staticmethod
    def _brief(obj: Dict[str, Any]) -> str:
        """单行摘要（指令信封与回执信封共用）。"""
        data = obj.get("data") if isinstance(obj.get("data"), dict) else {}
        ctype = obj.get("command_type") or obj.get("type") or data.get("command_type")
        status = data.get("status") or obj.get("status", "")
        reason = obj.get("message") or obj.get("reason", "")
        return (f"type={ctype} id={obj.get('command_id')} "
                f"status={status} reason={reason}")

    def _estop_hold_step(self) -> None:
        if self._estop_latched:
            self._estop_pub.publish(Bool(data=True))

    def close(self) -> bool:
        if self._kafka_ready:
            deadline = time.time() + 3.0
            try:
                while self._producer.outq_len() > 0 and time.time() < deadline:
                    self._producer.poll(0.1)
                self._producer.flush(1.0)
                self._consumer.close()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn('Kafka 关闭异常：%s' % (exc,))
        return super().close()


def main(args=None) -> int:
    rclpy.init(args=args)
    node = CommandAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
