// Copyright 2026 HUNTER Development Team
// 数据采集 Agent 实现（文档第 14 章）
#include "data_agent/data_agent.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <utility>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/statvfs.h>
#include <sys/time.h>
#include <unistd.h>

namespace data_agent
{

namespace
{

// JSON 字符串转义（事件类型/节点名等含特殊字符时不破坏报文）
std::string jsonEscape(const std::string & raw)
{
  std::string out;
  out.reserve(raw.size() + 8);
  for (const char c : raw) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          char buf[7];
          std::snprintf(buf, sizeof(buf), "\\u%04x", c);
          out += buf;
        } else {
          out += c;
        }
    }
  }
  return out;
}

// 契约要求 minItems:1 的整型数组（chassis.motor_rpm / motor_current / motor_temp 的
// items 均为 **integer**；float 序列化成 45.000 会被严格的 integer 校验拒绝，
// 故温度先取整再序列化；缺失测量以 [0] 占位，平台按“无该测量”理解）
std::string jsonIntArrayNonEmpty(const float * data, std::size_t n)
{
  std::ostringstream oss;
  oss << '[';
  if (n == 0) {
    oss << '0';
  }
  for (std::size_t i = 0; i < n; ++i) {
    if (i > 0) {
      oss << ',';
    }
    oss << static_cast<int>(std::lround(data[i]));
  }
  oss << ']';
  return oss.str();
}

// 四元数 → 航向角（yaw，rad，[-π, π]）：契约 localization.heading
double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny, cosy);
}

// 已用内存 MB（契约 system.memory_usage_mb；/proc/meminfo 的
// MemTotal - MemAvailable，含 page cache 与共享内存的实际占用）
int usedMemoryMb()
{
  std::ifstream in("/proc/meminfo");
  if (!in.is_open()) {
    return 0;
  }
  // 逐行解析（不能用 >> 连续取值：部分行无单位字段，会串行错位）
  long total_kb = 0;
  long avail_kb = 0;
  std::string line;
  while (std::getline(in, line)) {
    if (total_kb == 0 && line.rfind("MemTotal:", 0) == 0) {
      total_kb = std::strtol(line.c_str() + 9, nullptr, 10);
    } else if (avail_kb == 0 && line.rfind("MemAvailable:", 0) == 0) {
      avail_kb = std::strtol(line.c_str() + 13, nullptr, 10);
    }
    if (total_kb > 0 && avail_kb > 0) {
      break;
    }
  }
  const long used_kb = total_kb - avail_kb;
  return used_kb > 0 ? static_cast<int>(used_kb / 1024) : 0;
}

// 根分区可用空间 MB（契约 health.free_storage_mb；OTA 存储门禁 ≥2048MB 的输入）
unsigned long long freeStorageMb()
{
  struct statvfs st{};
  if (statvfs("/", &st) != 0) {
    return 0;
  }
  return static_cast<unsigned long long>(st.f_bavail) *
         static_cast<unsigned long long>(st.f_frsize) / (1024ULL * 1024ULL);
}

// 到 Kafka broker 的 TCP 建连时延（契约 system.network_latency_ms）。
// 车端无独立网络探针，此处以“能否连通运营端 Kafka 入口 + 建连耗时”作为
// 车-云链路时延的可观测代理；1Hz 采样（在 publishHealth 中调用），
// 150ms 超时避免阻塞上报线程。返回 <0 表示不可达。
double tcpConnectLatencyMs(const std::string & host, int port)
{
  if (host.empty() || port <= 0) {
    return -1.0;
  }
  struct addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  struct addrinfo * res = nullptr;
  const std::string port_str = std::to_string(port);
  if (getaddrinfo(host.c_str(), port_str.c_str(), &hints, &res) != 0 || res == nullptr) {
    return -1.0;
  }

  double latency = -1.0;
  for (struct addrinfo * it = res; it != nullptr; it = it->ai_next) {
    const int fd = socket(it->ai_family, it->ai_socktype, it->ai_protocol);
    if (fd < 0) {
      continue;
    }
    // 非阻塞 + 超时，避免 DNS/网络抖动卡住 10Hz 上报线程
    const int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    const auto t0 = std::chrono::steady_clock::now();
    const int rc = connect(fd, it->ai_addr, it->ai_addrlen);
    if (rc == 0) {
      latency = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();
      close(fd);
      break;
    }
    if (errno == EINPROGRESS) {
      struct timeval tv{};
      tv.tv_sec = 0;
      tv.tv_usec = 150000;   // 150ms
      fd_set wfds;
      FD_ZERO(&wfds);
      FD_SET(fd, &wfds);
      const int sel = select(fd + 1, nullptr, &wfds, nullptr, &tv);
      if (sel > 0) {
        int soerr = 0;
        socklen_t len = sizeof(soerr);
        if (getsockopt(fd, SOL_SOCKET, SO_ERROR, &soerr, &len) == 0 && soerr == 0) {
          latency = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t0).count();
          close(fd);
          break;
        }
      }
    }
    close(fd);
  }
  freeaddrinfo(res);
  return latency;
}

// bootstrap.servers（"host1:port1,host2:port2"）→ 首个 host/port
void parseFirstBroker(const std::string & servers, std::string & host, int & port)
{
  host.clear();
  port = 0;
  if (servers.empty()) {
    return;
  }
  std::string first = servers.substr(0, servers.find(','));
  const auto colon = first.rfind(':');
  if (colon == std::string::npos) {
    host = first;
    port = 9092;
    return;
  }
  host = first.substr(0, colon);
  try {
    port = std::stoi(first.substr(colon + 1));
  } catch (const std::exception &) {
    port = 9092;
  }
}

}  // namespace

DataAgent::DataAgent(const rclcpp::NodeOptions & options)
: rclcpp::Node("data_agent", options),
  producer_(nullptr),
  event_producer_(nullptr),
  kafka_connected_(false),
  db_(nullptr),
  fused_object_count_(0),
  prev_velocity_(0.0),
  accel_duration_(0.0),
  prev_mode_(""),
  reconnect_backoff_(1.0)
{
  // 参数（文档 14.2/14.3/14.6）
  declare_parameter("vehicle_id", "");                 // 空 → 取 kafka.properties 的 SASL 用户名（== 证书 CN）
  declare_parameter("kafka_brokers", "");              // 仅开发机兜底，接入包存在时被 properties 覆盖
  declare_parameter("kafka_properties", kafka_access::DEFAULT_PROPERTIES_PATH);
  declare_parameter("kafka_bundle_dir", "");           // 空 → properties 同目录
  declare_parameter("db_path", "/data/data_agent/telemetry.db");
  declare_parameter("publish_rate", 10.0);
  declare_parameter("max_velocity", 2.0);
  declare_parameter("min_battery_soc", 20.0);
  declare_parameter("hard_accel", 3.0);
  declare_parameter("hard_turn", 0.8);
  declare_parameter("comm_loss_duration", 10.0);
  declare_parameter("cache_max_hours", 24.0);
  // 事件去重抑制期（秒）：同一 event_type 在此间隔内只上报一次（边沿触发）；
  // detectEvents 是 100ms 判定，无此门控会在低电/超速/急停持续期间每拍重复上报
  declare_parameter("event_min_interval", 10.0);
  declare_parameter("kafka_queue_limit", 50);         // 本地队列积压上限（条），超过转 SQLite 缓存
  declare_parameter("kafka_flush_timeout_ms", 5000);  // 退出时等待投递的超时（毫秒）
  declare_parameter("replay_batch_size", 50);         // 每轮回放条数（断点续传限流，防瞬时冲击链路）
  declare_parameter("replay_inflight_limit", 200);    // 在途（已投递未证实）回放上限
  // Kafka SASL_SSL 认证参数（未部署接入包的开发机兜底；生产凭据只写在接入包 properties，不入仓）
  declare_parameter("security_protocol", "SASL_SSL");
  declare_parameter("sasl_mechanism", "SCRAM-SHA-512");
  declare_parameter("sasl_username", "");
  declare_parameter("sasl_password", "");
  declare_parameter("ssl_ca_location", "/etc/ssl/certs/ca-certificates.crt");

  vehicle_id_ = get_parameter("vehicle_id").as_string();
  kafka_brokers_ = get_parameter("kafka_brokers").as_string();
  kafka_properties_path_ = get_parameter("kafka_properties").as_string();
  kafka_bundle_dir_ = get_parameter("kafka_bundle_dir").as_string();
  db_path_ = get_parameter("db_path").as_string();
  publish_rate_ = get_parameter("publish_rate").as_double();
  max_velocity_ = get_parameter("max_velocity").as_double();
  min_battery_soc_ = get_parameter("min_battery_soc").as_double();
  hard_accel_ = get_parameter("hard_accel").as_double();
  hard_turn_ = get_parameter("hard_turn").as_double();
  comm_loss_duration_ = get_parameter("comm_loss_duration").as_double();
  event_min_interval_ = get_parameter("event_min_interval").as_double();
  cache_max_hours_ = get_parameter("cache_max_hours").as_double();
  kafka_queue_limit_ = static_cast<int>(get_parameter("kafka_queue_limit").as_int());
  kafka_flush_timeout_ms_ = static_cast<int>(get_parameter("kafka_flush_timeout_ms").as_int());
  replay_batch_size_ = static_cast<int>(get_parameter("replay_batch_size").as_int());
  replay_inflight_limit_ = static_cast<int>(get_parameter("replay_inflight_limit").as_int());
  security_protocol_ = get_parameter("security_protocol").as_string();
  sasl_mechanism_ = get_parameter("sasl_mechanism").as_string();
  sasl_username_ = get_parameter("sasl_username").as_string();
  sasl_password_ = get_parameter("sasl_password").as_string();
  ssl_ca_location_ = get_parameter("ssl_ca_location").as_string();

  // ─── 接入包优先（单一可信源）：Broker / vehicle_id 以 kafka.properties 为准 ───
  kafka_access::StringMap props;
  std::string access_err;
  if (kafka_access::loadProperties(kafka_properties_path_, props, access_err)) {
    kafka_brokers_ = props["bootstrap.servers"];
    const std::string props_vid = kafka_access::resolveVehicleId(props);
    if (!vehicle_id_.empty() && !props_vid.empty() && vehicle_id_ != props_vid) {
      // Topic 名、证书 CN、SCRAM 用户名三者必须同源，否则平台按设备档案找不到该车
      RCLCPP_ERROR(get_logger(),
        "vehicle_id 冲突：YAML=%s 而接入包 SASL 用户名=%s（等同证书 CN）；以接入包为准",
        vehicle_id_.c_str(), props_vid.c_str());
      vehicle_id_ = props_vid;
    } else if (vehicle_id_.empty()) {
      vehicle_id_ = props_vid;
    }
    if (kafka_bundle_dir_.empty()) {
      const auto parent = std::filesystem::path(kafka_properties_path_).parent_path();
      kafka_bundle_dir_ = parent.empty() ? kafka_access::DEFAULT_BUNDLE_DIR : parent.string();
    }
    RCLCPP_INFO(get_logger(), "Kafka 接入源：%s（vehicle_id=%s, brokers=%s）",
      kafka_properties_path_.c_str(), vehicle_id_.c_str(), kafka_brokers_.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "%s；回落 YAML 内联参数（仅限开发机）", access_err.c_str());
    if (kafka_bundle_dir_.empty()) {
      kafka_bundle_dir_ = kafka_access::DEFAULT_BUNDLE_DIR;
    }
  }
  if (vehicle_id_.empty()) {
    // Topic 名、证书 CN、平台设备档案三者的主键必须一致，取不到就不能拼 Topic
    RCLCPP_FATAL(get_logger(),
      "vehicle_id 无法确定：请在 %s 写入 sasl.jaas.config 的 username，或显式配置 "
      "data_agent.vehicle_id 参数（接入包要求 username == 证书 CN == vehicle_id）",
      kafka_properties_path_.c_str());
  }

  // Kafka topic（接入包契约：hunter.{vehicle_id}.telemetry / .event / .health）
  telemetry_topic_ = "hunter." + vehicle_id_ + ".telemetry";
  event_topic_ = "hunter." + vehicle_id_ + ".event";
  health_topic_ = "hunter." + vehicle_id_ + ".health";

  // 运营端入口（用于契约 system.network_latency_ms 采样；取 bootstrap 首个 broker）
  parseFirstBroker(kafka_brokers_, broker_host_, broker_port_);

  // 订阅（文档 14.2.1：/chassis/state 10Hz + /chassis/feedback 50Hz→10Hz）
  chassis_sub_ = create_subscription<hunter_msgs::msg::ChassisState>(
    "/chassis/state", rclcpp::SensorDataQoS(),
    std::bind(&DataAgent::chassisCallback, this, std::placeholders::_1));
  chassis_feedback_sub_ = create_subscription<hunter_msgs::msg::ChassisState>(
    "/chassis/feedback", rclcpp::SensorDataQoS(),
    std::bind(&DataAgent::chassisFeedbackCallback, this, std::placeholders::_1));
  localization_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/localization/odom", rclcpp::SensorDataQoS(),
    std::bind(&DataAgent::localizationCallback, this, std::placeholders::_1));
  fused_sub_ = create_subscription<hunter_msgs::msg::DetectedObjectArray>(
    "/perception/fused_objects", rclcpp::SensorDataQoS(),
    std::bind(&DataAgent::fusedObjectsCallback, this, std::placeholders::_1));
  trajectory_sub_ = create_subscription<hunter_msgs::msg::Trajectory>(
    "/planning/trajectory", rclcpp::SensorDataQoS(),
    std::bind(&DataAgent::trajectoryCallback, this, std::placeholders::_1));
  control_sub_ = create_subscription<hunter_msgs::msg::ChassisCommand>(
    "/control/command", rclcpp::SensorDataQoS(),
    std::bind(&DataAgent::controlCallback, this, std::placeholders::_1));
  health_sub_ = create_subscription<hunter_msgs::msg::SystemHealth>(
    "/system/health", rclcpp::SensorDataQoS(),
    std::bind(&DataAgent::healthCallback, this, std::placeholders::_1));

  // Kafka + SQLite 初始化
  kafkaInit();
  sqliteInit();

  // 定时器
  pack_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / publish_rate_),
    std::bind(&DataAgent::packAndPublish, this));
  event_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&DataAgent::detectEvents, this));
  // health Topic：health_monitor 以 1Hz 发布 /system/health，此处同频上报平台
  //（接入包联调判据：平台侧 last_online_time 刷新）
  health_timer_ = create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&DataAgent::publishHealth, this));

  // 事件检测的上一帧时间必须用节点时钟初始化。
  // 否则 rclcpp::Time 默认时钟源(RCL_SYSTEM_TIME)与 this->now()(RCL_ROS_TIME)不同，
  // detectEvents 里 now - prev_time_ 会抛 "can't subtract times with different time sources"。
  prev_time_ = this->now();
  // 感知 FPS 统计窗口起点（契约 perception.fps）
  perception_window_start_ = this->now();

  RCLCPP_INFO(get_logger(), "data_agent 启动：vehicle=%s, brokers=%s",
    vehicle_id_.c_str(), kafka_brokers_.c_str());
}

DataAgent::~DataAgent()
{
  // 退出前尽力投递残留消息（文档 14.6）；两个 producer 都要 flush
  for (RdKafka::Producer * p : {producer_, event_producer_}) {
    if (!p) {
      continue;
    }
    const RdKafka::ErrorCode err = p->flush(kafka_flush_timeout_ms_);
    const int remaining = p->outq_len();
    if (remaining > 0) {
      RCLCPP_WARN(
        get_logger(), "Kafka 退出时仍有 %d 条消息未送达（flush: %s），"
        "该部分消息已在 SQLite 缓存中保留或丢失",
        remaining, RdKafka::err2str(err).c_str());
    }
    delete p;
  }
  producer_ = nullptr;
  event_producer_ = nullptr;
  if (db_) {
    sqlite3_close(db_);
    db_ = nullptr;
  }
}

void DataAgent::chassisCallback(const hunter_msgs::msg::ChassisState::SharedPtr msg)
{
  chassis_ = *msg;
  chassis_received_ = true;
}

void DataAgent::chassisFeedbackCallback(const hunter_msgs::msg::ChassisState::SharedPtr msg)
{
  chassis_feedback_ = *msg;
  chassis_feedback_received_ = true;
}

void DataAgent::localizationCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  localization_ = *msg;
  localization_received_ = true;
}

void DataAgent::fusedObjectsCallback(
  const hunter_msgs::msg::DetectedObjectArray::SharedPtr msg)
{
  // 采集契约 perception 段所需的三项（detected_objects / fps / latency_ms / object_types），
  // 仅统计不缓存完整目标数组（文档 14.2.1：遥测为快照，不搬运点云/目标明细）
  std::map<std::string, int> types;
  for (const auto & obj : msg->objects) {
    const std::string name = obj.class_name.empty() ? "other" : obj.class_name;
    types[name] += 1;
  }
  fused_object_types_ = std::move(types);
  fused_object_count_ = static_cast<int>(msg->objects.size());

  // FPS：单帧回调即计数，窗口满 1s 结算（契约 perception.fps）
  fused_frames_in_window_ += 1;
  const rclcpp::Time now = this->now();
  const double window = (now - perception_window_start_).seconds();
  if (window >= 1.0) {
    perception_fps_ = static_cast<double>(fused_frames_in_window_) / window;
    fused_frames_in_window_ = 0;
    perception_window_start_ = now;
  }

  // 端到端延迟：帧头时间戳 → 本节点收到（传输 + 处理），负值钳到 0；
  // 未填时间戳（stamp=0）时不做减法（否则得到 epoch 量级的假延迟）
  if (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0) {
    const double latency_ms = (now - rclcpp::Time(msg->header.stamp)).seconds() * 1000.0;
    perception_latency_ms_ = latency_ms > 0.0 ? latency_ms : 0.0;
  }
}

void DataAgent::trajectoryCallback(const hunter_msgs::msg::Trajectory::SharedPtr msg)
{
  // 规划段快照（契约 planning）：点数 / 累计弧长 / 计算延迟。
  // 不缓存完整轨迹点（文档 14.2.1），只在回调内算完即弃。
  const auto & pts = msg->points;
  trajectory_points_ = static_cast<int>(pts.size());
  double length = 0.0;
  for (std::size_t i = 1; i < pts.size(); ++i) {
    const double dx = static_cast<double>(pts[i].x) - pts[i - 1].x;
    const double dy = static_cast<double>(pts[i].y) - pts[i - 1].y;
    length += std::sqrt(dx * dx + dy * dy);
  }
  trajectory_length_ = length;
  trajectory_received_ = true;

  if (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0) {
    const double latency_ms = (this->now() - rclcpp::Time(msg->header.stamp)).seconds() * 1000.0;
    planning_latency_ms_ = latency_ms > 0.0 ? latency_ms : 0.0;
  }
}

void DataAgent::controlCallback(const hunter_msgs::msg::ChassisCommand::SharedPtr msg)
{
  control_ = *msg;
  control_received_ = true;
}

void DataAgent::healthCallback(const hunter_msgs::msg::SystemHealth::SharedPtr msg)
{
  health_ = *msg;
  health_received_ = true;
}

void DataAgent::packAndPublish()
{
  if (vehicle_id_.empty()) {
    return;   // Topic 主键未确定（启动时已打 FATAL 日志）
  }
  const std::string json = buildTelemetryJson();

  if (kafkaReady()) {
    if (!kafkaProduce(producer_, telemetry_topic_, json)) {
      // produce 失败（如本地队列满）→ 断线缓存
      sqliteCache(telemetry_topic_, json);
    }
    // 链路可用→把断网期间落库的报文按序回放（文档 14.6 断点续传）
    sqliteReplay();
  } else {
    // 链路未证实/已积压 → 本地缓存 + 重连
    sqliteCache(telemetry_topic_, json);
    kafkaReconnect();
  }
}

void DataAgent::publishHealth()
{
  if (vehicle_id_.empty()) {
    return;
  }
  // 运营端链路时延采样（契约 system.network_latency_ms）：1Hz，与 health 同频。
  // 采样失败（不可达）保留上次有效值，避免看板出现 0 的假健康。
  const double latency = tcpConnectLatencyMs(broker_host_, broker_port_);
  if (latency >= 0.0) {
    network_latency_ms_ = latency;
  }
  // health Topic 是平台侧判活（last_online_time）的主要依据，接入包契约 1Hz
  const std::string json = buildHealthJson();
  if (kafkaReady()) {
    if (!kafkaProduce(producer_, health_topic_, json)) {
      sqliteCache(health_topic_, json);
    }
  } else {
    sqliteCache(health_topic_, json);
    kafkaReconnect();
  }
}

bool DataAgent::kafkaReady()
{
  // V0.1.13 修复：开机即触发的发送死锁。
  // 旧写法把 kafka_connected_ 当作“能否 produce”的前置条件，但 kafka_connected_ 只能由
  // dr_cb 置真，而 dr_cb 只有真正 produce 之后才会触发；kafkaInit() 建好 producer 后又显式
  // 把 kafka_connected_ 置 false（等待投递证实）。于是开机后：kafka_connected_=false →
  // 本函数恒 false → packAndPublish/publishHealth/reportEvent 全部走 else 落 SQLite 缓存、
  // 永不 produce → dr_cb 永不触发 → kafka_connected_ 永远停在 false，形成闭环死锁：所有
  // telemetry/health/event 只落本地库、一条都不发往 broker，平台永远收不到 health（判活依据）
  // → 车辆“尚未上线”。hunter-kafka-check 是独立进程、自带 producer，不受此门控，故自检全绿
  // 却掩盖了该问题。
  // 修复：只要 producer 就绪且本地队列未积压即尝试 produce，连通性完全交由 dr_cb 异步
  // 证实/回退（成功置 kafka_connected_=true，失败置 false）。broker 真不可达时，消息在
  // message.timeout.ms 后经 dr_cb 失败，本地队列积压超 kafka_queue_limit_ 自动回落 SQLite
  // 缓存 + 触发 communication_loss——断网缓存与续传语义完整保留。
  if (!producer_) {
    return false;
  }
  // 本地队列积压超过上限 → 视为链路不可用（消息滞留队列最终会被
  // message.timeout 丢弃，及时转 SQLite 缓存兜底）
  return producer_->outq_len() < kafka_queue_limit_;
}

// 投递报告回调（librdkafka 内部线程调用）：
// produce() 成功只代表消息进入本地队列，只有这里才能证实是否真正送达
void DataAgent::dr_cb(RdKafka::Message & msg)
{
  // msg_opaque 非空 → 这是断点续传回放的消息：送达证实后才能删库中对应行
  void * opaque = msg.msg_opaque();
  std::int64_t row_id = -1;
  if (opaque) {
    row_id = *static_cast<std::int64_t *>(opaque);
    delete static_cast<std::int64_t *>(opaque);
    replay_inflight_.fetch_sub(1, std::memory_order_relaxed);
  }

  if (msg.err() == RdKafka::ERR_NO_ERROR) {
    dr_ok_count_.fetch_add(1, std::memory_order_relaxed);
    if (!kafka_connected_.load()) {
      RCLCPP_INFO(get_logger(), "Kafka 投递恢复（累计送达 %lu 条）",
        static_cast<unsigned long>(dr_ok_count_.load()));
    }
    kafka_connected_.store(true);
    // 链路已证实可用 → 重新武装 communication_loss 事件（下次中断可再触发）
    link_down_ = false;
    comm_loss_reported_ = false;
    if (row_id >= 0) {
      sqliteDeleteRow(row_id);
    }
  } else {
    dr_fail_count_.fetch_add(1, std::memory_order_relaxed);
    kafka_connected_.store(false);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "Kafka 投递失败：%s（转本地缓存）", msg.errstr().c_str());
    // 回放失败的行保留在 SQLite 中，下轮重试（至少一次语义，靠平台侧幂等收敛）
  }
}

std::string DataAgent::buildSystemSegment() const
{
  // 契约 system 段（telemetry 与 health 共用；字段与约束完全相同：
  // cpu/gpu 0-100、memory_usage_mb ≥0、network_rssi ≤0、network_latency_ms ≥0）
  const double cpu = health_received_ ? static_cast<double>(health_.cpu_usage) : 0.0;
  const double gpu = health_received_ ? static_cast<double>(health_.gpu_usage) : 0.0;
  const double cpu_temp = health_received_ ? static_cast<double>(health_.cpu_temp) : 0.0;
  const double gpu_temp = health_received_ ? static_cast<double>(health_.gpu_temp) : 0.0;

  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2);
  oss << "\"system\":{";
  oss << "\"cpu_usage\":" << std::min(100.0, std::max(0.0, cpu));
  oss << ",\"gpu_usage\":" << std::min(100.0, std::max(0.0, gpu));
  // health_monitor 的 memory_usage 是百分比；契约要 MB → 直接读 /proc/meminfo
  oss << ",\"memory_usage_mb\":" << usedMemoryMb();
  oss << ",\"gpu_temp\":" << gpu_temp;
  oss << ",\"cpu_temp\":" << cpu_temp;
  // 本车无蜂窝/无线制式上报通道：契约要求 network_rssi ≤ 0，上报 0（无信号语义）
  oss << ",\"network_rssi\":" << network_rssi_;
  oss << ",\"network_latency_ms\":" << (network_latency_ms_ >= 0.0 ? network_latency_ms_ : 0.0);
  oss << "}";
  return oss.str();
}

std::string DataAgent::vehicleStatusForContract() const
{
  // health.status = 车辆 8 态业务状态（契约受控词表，不可新增/更改）。
  // 车端可判定的子集：emergency（急停锁存）/ fault（底盘故障）/ remote_controlled（遥控）/
  // auto_driving（自主行驶中）/ online_idle（在线待命）。
  // upgrading / charging / offline 由平台侧状态机与超时判定（车端不臆造对平台的“离线”）。
  if (chassis_received_) {
    if (chassis_.vehicle_state == "ESTOP" || chassis_.control_mode == "ESTOP") {
      return "emergency";
    }
    if (chassis_.vehicle_state == "FAULT" || chassis_.fault_code != 0) {
      return "fault";
    }
    if (chassis_.control_mode == "REMOTE") {
      return "remote_controlled";
    }
    if (std::fabs(chassis_.velocity) > 0.05) {
      return "auto_driving";
    }
  }
  return "online_idle";
}

std::string DataAgent::buildTelemetryJson()
{
  // ── 契约：contracts/kafka/schemas/telemetry.schema.json ─────────────────
  // required = [vehicle_id, timestamp, seq, chassis, localization, perception,
  //             planning, control, system]，additionalProperties=false。
  // 平台 data-collector 消费侧以 schema_name="auto" 做契约校验：**结构不符即进 DLQ**
  // （旧版平铺字段 velocity/pose_x/... 与契约完全不匹配 → 平台 telemetry 表 0 行，
  //   表象为“车端发送成功、运营端永远看不到数据”，本次按契约逐段对齐）。
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(3);
  oss << "{";
  oss << "\"vehicle_id\":\"" << jsonEscape(vehicle_id_) << "\"";
  oss << ",\"timestamp\":" << this->now().seconds();
  oss << ",\"seq\":" << (++seq_);

  // ---- chassis（12 项，全部必填）----
  oss << ",\"chassis\":{";
  oss << "\"velocity\":" << (chassis_received_ ? chassis_.velocity : 0.0f);
  oss << ",\"steering_angle\":" << (chassis_received_ ? chassis_.steering_angle : 0.0f);
  oss << ",\"battery_voltage\":" << (chassis_received_ ? chassis_.battery_voltage : 0.0f);
  {
    // 契约 battery_soc 为整数 0-100（平台清洗也会裁剪，这里提前给出合法值）
    double soc = chassis_received_ ? static_cast<double>(chassis_.battery_soc) : 0.0;
    soc = std::min(100.0, std::max(0.0, soc));
    oss << ",\"battery_soc\":" << static_cast<int>(soc);
  }
  oss << ",\"battery_current\":" << (chassis_received_ ? chassis_.battery_current : 0.0f);
  oss << ",\"battery_temp\":" << (chassis_received_ ? chassis_.battery_temperature : 0.0f);
  oss << ",\"control_mode\":\"" << jsonEscape(chassis_received_ ? chassis_.control_mode : "") << "\"";
  oss << ",\"vehicle_state\":\"" << jsonEscape(chassis_received_ ? chassis_.vehicle_state : "") << "\"";
  oss << ",\"fault_code\":" << (chassis_received_ ? static_cast<int>(chassis_.fault_code) : 0);
  // 底盘驱动当前只发布 motor_temperature（ChassisState 契约字段，.ai‑rules 禁止新增消息字段）；
  // 契约三数组均要求 minItems:1 → 缺失项以 [0] 占位（平台按“无该测量”理解）
  oss << ",\"motor_rpm\":[0]";
  oss << ",\"motor_current\":[0]";
  oss << ",\"motor_temp\":" << jsonIntArrayNonEmpty(
    chassis_.motor_temperature.data(), chassis_.motor_temperature.size());
  oss << "}";

  // ---- localization（10 项，全部必填）----
  oss << ",\"localization\":{";
  if (localization_received_) {
    const auto & pose = localization_.pose.pose;
    const auto & twist = localization_.twist.twist;
    oss << "\"x\":" << pose.position.x;
    oss << ",\"y\":" << pose.position.y;
    oss << ",\"z\":" << pose.position.z;
    // 契约 localization 的 roll/pitch/heading 为欧拉角，EKF 输出四元数 → 此处换算
    const double roll = std::atan2(
      2.0 * (pose.orientation.w * pose.orientation.x + pose.orientation.y * pose.orientation.z),
      1.0 - 2.0 * (pose.orientation.x * pose.orientation.x + pose.orientation.y * pose.orientation.y));
    const double pitch = std::asin(std::max(-1.0, std::min(1.0,
      2.0 * (pose.orientation.w * pose.orientation.y - pose.orientation.z * pose.orientation.x))));
    oss << ",\"roll\":" << roll;
    oss << ",\"pitch\":" << pitch;
    oss << ",\"heading\":" << yawFromQuaternion(pose.orientation);
    oss << ",\"linear_velocity\":[" << twist.linear.x << "," << twist.linear.y
        << "," << twist.linear.z << "]";
    oss << ",\"angular_velocity\":[" << twist.angular.x << "," << twist.angular.y
        << "," << twist.angular.z << "]";
    // 协方差对角元 (x, yaw) → 定位健康度（契约 position_std / heading_std，均 ≥0）
    const double pos_std = std::sqrt(std::max(0.0, static_cast<double>(localization_.pose.covariance[0])));
    const double yaw_std = std::sqrt(std::max(0.0, static_cast<double>(localization_.pose.covariance[35])));
    oss << ",\"position_std\":" << pos_std;
    oss << ",\"heading_std\":" << yaw_std;
  } else {
    // 定位未就绪也必须给出契约结构（零位姿 + 超大 std，平台据此判“未收敛”）
    oss << "\"x\":0,\"y\":0,\"z\":0,\"roll\":0,\"pitch\":0,\"heading\":0";
    oss << ",\"linear_velocity\":[0,0,0],\"angular_velocity\":[0,0,0]";
    oss << ",\"position_std\":9999,\"heading_std\":9999";
  }
  oss << "}";

  // ---- perception（4 项，全部必填）----
  oss << ",\"perception\":{";
  oss << "\"detected_objects\":" << fused_object_count_;
  oss << ",\"fps\":" << (perception_fps_ >= 0.0 ? perception_fps_ : 0.0);
  oss << ",\"latency_ms\":" << (perception_latency_ms_ >= 0.0 ? perception_latency_ms_ : 0.0);
  oss << ",\"object_types\":{";
  {
    bool first = true;
    for (const auto & kv : fused_object_types_) {
      if (!first) {
        oss << ",";
      }
      oss << "\"" << jsonEscape(kv.first) << "\":" << kv.second;
      first = false;
    }
  }
  oss << "}}";

  // ---- planning（4 项，全部必填）----
  oss << ",\"planning\":{";
  oss << "\"trajectory_length\":" << trajectory_length_;
  oss << ",\"trajectory_points\":" << trajectory_points_;
  oss << ",\"planning_latency_ms\":" << planning_latency_ms_;
  // 当前行为：无独立行为话题时以底盘控制模式表示（契约 current_behavior 为自由字符串）
  oss << ",\"current_behavior\":\""
      << jsonEscape(chassis_received_ ? chassis_.control_mode : "") << "\"";
  oss << "}";

  // ---- control（5 项，全部必填；误差 = 目标 − 底盘反馈）----
  oss << ",\"control\":{";
  {
    const double target_v = control_received_ ? static_cast<double>(control_.target_velocity) : 0.0;
    const double target_s = control_received_ ? static_cast<double>(control_.target_steering) : 0.0;
    const double fb_v = chassis_feedback_received_ ? static_cast<double>(chassis_feedback_.velocity) : 0.0;
    const double fb_s = chassis_feedback_received_ ? static_cast<double>(chassis_feedback_.steering_angle) : 0.0;
    oss << "\"target_velocity\":" << target_v;
    oss << ",\"target_steer\":" << target_s;
    oss << ",\"velocity_error\":" << (target_v - fb_v);
    oss << ",\"steer_error\":" << (target_s - fb_s);
  }
  // 控制计算延迟无独立埋点：暂以 0 上报（契约 minimum 为 0，字段必须存在）
  oss << ",\"control_latency_ms\":0";
  oss << "}";

  // ---- system（7 项，全部必填）----
  oss << "," << buildSystemSegment();

  oss << "}";
  return oss.str();
}

std::string DataAgent::buildHealthJson()
{
  // ── 契约：contracts/kafka/schemas/health.schema.json ────────────────────
  // required = [vehicle_id, timestamp, status, system]；可选 gear / free_storage_mb / lat / lng。
  // 平台 data-collector-health 据此写 Redis 读模型 vehicle:status:{id} 并回写车辆台账
  // status/last_online_time（联调唯一判据：平台侧 last_online_time 刷新）。
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2);
  oss << "{";
  oss << "\"vehicle_id\":\"" << jsonEscape(vehicle_id_) << "\"";
  oss << ",\"timestamp\":" << this->now().seconds();
  oss << ",\"status\":\"" << vehicleStatusForContract() << "\"";
  // free_storage_mb（G-08：OTA 存储门禁 ≥2048MB；不上报即门禁缺数据、按安全默认拒绝）
  oss << ",\"free_storage_mb\":" << freeStorageMb();
  // gear / lat / lng（G-08 / G-11）：当前底盘驱动未提供档位与 GPS → **不臆造**，
  // 缺省即不上报，平台按 health.schema.json 描述的安全默认处理
  oss << "," << buildSystemSegment();
  oss << "}";
  return oss.str();
}

void DataAgent::detectEvents()
{
  if (!chassis_received_) {
    return;
  }

  const rclcpp::Time now = this->now();
  const double v = std::fabs(chassis_.velocity);
  const double dt = (now - prev_time_).seconds();

  // ── 契约事件（contracts/kafka/schemas/event.schema.json）─────────────────
  // event_type ∈ 19 种受控词表、event_level ∈ {info,warning,critical}，
  // 且 data-collector 会按 EVENT_LEVEL_BY_TYPE 做“类型↔等级”一致性校验：
  // 等级写错 → 消费侧视为非法（本次把旧版自造名 hard_deceleration/overspeed/
  // low_battery 与自造等级统一到契约词表，否则平台 events 表 0 行）。
  // 1. 急加速/急减速（文档 14.3：加速度 > 3.0 m/s² 持续 0.5s）
  if (dt > 0.0 && dt < 1.0) {
    const double accel = (chassis_.velocity - prev_velocity_) / dt;
    if (std::fabs(accel) > hard_accel_) {
      accel_duration_ += dt;
      if (accel_duration_ > 0.5) {
        char data[192];
        std::snprintf(data, sizeof(data),
          "{\"acceleration\":%.3f,\"threshold\":%.3f,\"velocity\":%.3f}",
          accel, hard_accel_, static_cast<double>(chassis_.velocity));
        if (accel > 0) {
          reportEvent("harsh_acceleration", "warning",
            "急加速：加速度超过阈值并持续 0.5s 以上", data);
        } else {
          reportEvent("harsh_braking", "warning",
            "急减速：减速度超过阈值并持续 0.5s 以上", data);
        }
        accel_duration_ = 0.0;
      }
    } else {
      accel_duration_ = 0.0;
    }
  }

  // 2. 急转弯（文档 14.3：横摆角速度 > 0.8 rad/s）
  if (localization_received_ &&
    std::fabs(localization_.twist.twist.angular.z) > hard_turn_)
  {
    const double yaw_rate = localization_.twist.twist.angular.z;
    char data[160];
    std::snprintf(data, sizeof(data),
      "{\"yaw_rate\":%.3f,\"threshold\":%.3f}", yaw_rate, hard_turn_);
    reportEvent("harsh_turning", "warning", "急转弯：横摆角速度超过阈值", data);
  }

  // 3. 超速（文档 14.3：速度 > 限速 × 1.1）
  if (v > max_velocity_ * 1.1) {
    char data[160];
    std::snprintf(data, sizeof(data),
      "{\"velocity\":%.3f,\"limit\":%.3f}", v, max_velocity_);
    // 契约等级（contracts/database/enums.md §3）：over_speed = **critical**（不是 warning）
    reportEvent("over_speed", "critical", "超速：车速超过限速 110%", data);
  }

  // 4. 电池低电量（文档 14.3：SOC < 20%）/ 严重低电（< 10%）
  if (chassis_.battery_soc > 0.0f && chassis_.battery_soc < min_battery_soc_) {
    const double soc = static_cast<double>(chassis_.battery_soc);
    char data[160];
    if (soc < min_battery_soc_ / 2.0) {
      std::snprintf(data, sizeof(data),
        "{\"battery_soc\":%.1f,\"threshold\":%.1f}", soc, min_battery_soc_ / 2.0);
      reportEvent("battery_critical", "critical", "电量严重不足：SOC 低于 10%", data);
    } else {
      std::snprintf(data, sizeof(data),
        "{\"battery_soc\":%.1f,\"threshold\":%.1f}", soc, min_battery_soc_);
      reportEvent("battery_low", "warning", "电量偏低：SOC 低于 20%", data);
    }
  }

  // 5. 紧急制动（文档 14.3：ESTOP 触发）
  if (chassis_.vehicle_state == "ESTOP" || chassis_.control_mode == "ESTOP") {
    reportEvent("emergency_stop", "critical",
      "紧急制动：底盘进入 ESTOP",
      "{\"vehicle_state\":\"" + jsonEscape(chassis_.vehicle_state) +
      "\",\"control_mode\":\"" + jsonEscape(chassis_.control_mode) + "\"}");
  }

  // 6. 人工接管（文档 14.3：模式切换为 REMOTE）
  if (prev_mode_ != "REMOTE" && chassis_.control_mode == "REMOTE") {
    reportEvent("manual_takeover", "info",
      "人工接管：驾驶模式切换为 REMOTE",
      "{\"previous_mode\":\"" + jsonEscape(prev_mode_) + "\"}");
  }

  prev_velocity_ = chassis_.velocity;
  prev_time_ = now;
  prev_mode_ = chassis_.control_mode;
}

void DataAgent::reportEvent(const std::string & event_type, const std::string & event_level,
  const std::string & description, const std::string & data_json)
{
  // ── 事件去重（边沿触发 + 抑制期）────────────────────────────────────────
  // detectEvents 是 100ms 无状态判定：低电/超速/急转弯/急停在持续期间会**每拍命中**，
  // 旧版于是每秒上报约 10 条重复事件，且 triggerBagRecord 每条都拉起一个
  // `ros2 bag record`（实机已见 `..._communication_loss` 录制进程）→ 磁盘与平台双爆、
  // 平台侧还会把它们当重复事件反复入库。这里做统一门控：同一 event_type 在
  // event_min_interval_（默认 10s）内只放行一次，其余静默丢弃（连日志也不刷）。
  const rclcpp::Time now = this->now();
  const auto last = last_event_at_.find(event_type);
  if (last != last_event_at_.end() &&
      (now - last->second).seconds() < event_min_interval_)
  {
    return;
  }
  last_event_at_[event_type] = now;

  // ── 契约：contracts/kafka/schemas/event.schema.json ─────────────────────
  // required = [vehicle_id, timestamp, event_type, event_level]，
  // additionalProperties=false：只允许 description / data / data_file_url 三个可选项。
  // （旧版发 {"type":..,"level":..} 与契约毫无交集 → 平台 events 表 0 行且被 DLQ 拒收）
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(3);
  oss << "{\"vehicle_id\":\"" << jsonEscape(vehicle_id_) << "\"";
  oss << ",\"timestamp\":" << this->now().seconds();
  oss << ",\"event_type\":\"" << jsonEscape(event_type) << "\"";
  oss << ",\"event_level\":\"" << jsonEscape(event_level) << "\"";
  if (!description.empty()) {
    oss << ",\"description\":\"" << jsonEscape(description) << "\"";
  }
  if (!data_json.empty()) {
    oss << ",\"data\":" << data_json;
  }
  oss << "}";
  const std::string json = oss.str();

  RCLCPP_WARN(get_logger(), "事件触发：%s (%s)", event_type.c_str(), event_level.c_str());
  if (vehicle_id_.empty()) {
    return;
  }

  // event 不可丢：走 acks=all 专用 producer；失败则落库等回放
  RdKafka::Producer * event_producer = event_producer_ ? event_producer_ : producer_;
  if (kafkaReady() && event_producer) {
    if (!kafkaProduce(event_producer, event_topic_, json)) {
      sqliteCache(event_topic_, json);  // produce 失败时事件也缓存
    }
  } else {
    sqliteCache(event_topic_, json);  // 断线时事件也缓存
  }

  triggerBagRecord(event_type);
}

void DataAgent::triggerBagRecord(const std::string & event_type)
{
  // 触发 rosbag 录制（文档 14.3.2：事件前后各 10 秒；14.4.2 录制话题）
  const std::string cmd =
    "ros2 bag record -o /data/rosbag/" +
    std::to_string(static_cast<int>(this->now().seconds())) + "_" + event_type +
    " /lidar_points /camera/camera/color/image_raw /imu/data /chassis/state &";
  std::system(cmd.c_str());
  RCLCPP_INFO(get_logger(), "触发 rosbag 录制（事件：%s）", event_type.c_str());
}

bool DataAgent::kafkaInit()
{
  // 接入包优先：kafka.properties 里含 SCRAM 凭据与 mTLS 证书路径，由 kafka_access
  // 装配成 librdkafka 配置（与 Python 侧 hunter_kafka 同一套语义）。
  // 未部署接入包时才回落 YAML 内联参数（无客户端证书，mTLS broker 会在握手期拒绝）。
  const bool has_props = std::filesystem::is_regular_file(kafka_properties_path_);
  // 重连路径可能重复调用：先释放旧实例，避免连接泄漏
  // （销毁前不 flush：旧实例在途消息不会再回调，因此回放计数必须归零，
  //   对应缓存行仍在库中，下轮重新投递 → 至少一次语义）
  if (producer_) {
    delete producer_;
    producer_ = nullptr;
  }
  if (event_producer_) {
    delete event_producer_;
    event_producer_ = nullptr;
  }
  replay_inflight_.store(0, std::memory_order_relaxed);

  auto createProducer = [this, has_props](
    const std::string & acks, const std::string & suffix) -> RdKafka::Producer * {
      kafka_access::StringMap kvs;
      std::string err;
      if (has_props) {
        kafka_access::ClientOptions opts;
        opts.properties_path = kafka_properties_path_;
        opts.bundle_dir = kafka_bundle_dir_;
        opts.client_id = vehicle_id_ + "-" + suffix;
        opts.is_producer = true;
        opts.acks = acks;
        opts.extra = {{"socket.keepalive.enable", "true"}};
        if (!kafka_access::loadClientConf(opts, kvs, err)) {
          RCLCPP_ERROR(get_logger(), "Kafka[%s] 配置失败：%s", suffix.c_str(), err.c_str());
          return nullptr;
        }
      } else {
        if (kafka_brokers_.empty()) {
          RCLCPP_ERROR(get_logger(), "Kafka[%s] 无可用接入参数", suffix.c_str());
          return nullptr;
        }
        kvs["bootstrap.servers"] = kafka_brokers_;
        std::string protocol = security_protocol_;
        std::transform(protocol.begin(), protocol.end(), protocol.begin(),
          [](unsigned char c) {return static_cast<char>(std::tolower(c)); });
        kvs["security.protocol"] = protocol;
        if (!sasl_mechanism_.empty()) {kvs["sasl.mechanism"] = sasl_mechanism_;}
        if (!sasl_username_.empty() && !sasl_password_.empty()) {
          kvs["sasl.username"] = sasl_username_;
          kvs["sasl.password"] = sasl_password_;
        }
        if (!ssl_ca_location_.empty()) {kvs["ssl.ca.location"] = ssl_ca_location_;}
        kvs["acks"] = acks;
        kvs["client.id"] = vehicle_id_ + "-" + suffix;
        kvs["compression.type"] = "lz4";
        kvs["linger.ms"] = "5";
        kvs["batch.size"] = "16384";
        kvs["retries"] = "3";
        RCLCPP_WARN(get_logger(),
          "Kafka[%s] 使用 YAML 内联参数（无 mTLS 客户端证书，接入 HunterCore 必须部署接入包到 %s）",
          suffix.c_str(), kafka_access::DEFAULT_BUNDLE_DIR);
      }
      // 链路兜底：broker 不可达时消息最多在本地队列滞留 60s，超时触发投递失败回调，
      // 避免消息无限积压（配合 kafka_queue_limit_ 提前转 SQLite 缓存）
      if (!kvs.count("message.timeout.ms")) {kvs["message.timeout.ms"] = "60000";}
      if (!kvs.count("reconnect.backoff.max.ms")) {kvs["reconnect.backoff.max.ms"] = "10000";}

      std::string errstr;
      RdKafka::Conf * conf = RdKafka::Conf::create(RdKafka::Conf::CONF_GLOBAL);
      if (!conf) {
        return nullptr;
      }
      // 投递报告回调：只有它能证实消息是否真正送达 broker
      if (conf->set("dr_cb", static_cast<RdKafka::DeliveryReportCb *>(this), errstr) !=
        RdKafka::Conf::CONF_OK)
      {
        RCLCPP_ERROR(get_logger(), "Kafka dr_cb 配置失败: %s", errstr.c_str());
        delete conf;
        return nullptr;
      }
      if (!kafka_access::applyToGlobalConf(conf, kvs, err)) {
        RCLCPP_ERROR(get_logger(), "Kafka[%s] %s", suffix.c_str(), err.c_str());
        delete conf;
        return nullptr;
      }
      // 脱敏后的配置快照（口令已打星）：现场排障时核对“到底用哪个证书/哪个 broker”
      RCLCPP_DEBUG(get_logger(), "Kafka[%s] 配置：%s", suffix.c_str(),
        kafka_access::redacted(kvs).c_str());
      RdKafka::Producer * producer = RdKafka::Producer::create(conf, errstr);
      delete conf;
      if (!producer) {
        RCLCPP_ERROR(get_logger(), "Kafka[%s] 生产者创建失败: %s",
          suffix.c_str(), errstr.c_str());
        return nullptr;
      }
      return producer;
    };

  // 接入包要求：高频 telemetry/health 用 acks=1，不可丢的 event 用 acks=all
  // —— 同一 producer 无法两种 acks 并存，因此分建两个实例
  producer_ = createProducer("1", "telemetry");
  event_producer_ = createProducer("all", "event");

  // 连通性由 dr_cb 证实后再置位（新建 producer 后需重新证实）
  kafka_connected_.store(false);

  if (!producer_) {
    RCLCPP_ERROR(get_logger(), "Kafka 链路未就绪：遥测将全部落 SQLite 缓存等回放");
    return false;
  }
  if (!event_producer_) {
    RCLCPP_WARN(get_logger(), "event 专用 producer 未创建，事件退回 telemetry 通道（acks=1）");
  }
  if (has_props) {
    RCLCPP_INFO(get_logger(), "Kafka 已按接入包装配（brokers=%s, mTLS 已启用, acks=1/all 双通道）",
      kafka_brokers_.c_str());
  }
  return true;
}

RdKafka::Producer * DataAgent::producerFor(const std::string & topic)
{
  // 契约：.event 走 acks=all 专用通道，telemetry/health 走高频低开销通道
  const std::string suffix = ".event";
  if (event_producer_ && topic.size() >= suffix.size() &&
    topic.compare(topic.size() - suffix.size(), suffix.size(), suffix) == 0)
  {
    return event_producer_;
  }
  return producer_;
}

bool DataAgent::kafkaProduce(
  RdKafka::Producer * producer, const std::string & topic,
  const std::string & payload, std::int64_t cache_row_id)
{
  if (!producer || topic.empty()) {
    return false;
  }
  // 先 poll 触发投递回调（更新连通性状态），再投递
  producer->poll(0);
  // 回放消息携带行号作为 msg_opaque：投递成功才能删行（至少一次语义）
  std::int64_t * opaque = nullptr;
  if (cache_row_id >= 0) {
    opaque = new std::int64_t(cache_row_id);
    replay_inflight_.fetch_add(1, std::memory_order_relaxed);
  }
  // key = vehicle_id：同一车辆固定落同一分区，保证分区内时序（接入包 README）
  const RdKafka::ErrorCode err = producer->produce(
    topic, RdKafka::Topic::PARTITION_UA, RdKafka::Producer::RK_MSG_COPY,
    const_cast<char *>(payload.c_str()), payload.size(),
    vehicle_id_.c_str(), vehicle_id_.size(),   // key, key_len
    0, opaque);                                 // timestamp, msg_opaque
  if (err != RdKafka::ERR_NO_ERROR) {
    if (opaque) {
      delete opaque;
      replay_inflight_.fetch_sub(1, std::memory_order_relaxed);
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 10000, "Kafka produce 失败(%s): %s",
      topic.c_str(), RdKafka::err2str(err).c_str());
    return false;
  }
  return true;
}

void DataAgent::kafkaReconnect()
{
  // 车-云链路中断事件（契约 event_type=communication_loss）：
  // 链路不可用持续 >= comm_loss_duration_（默认 10s，文档 14.3）时上报一次；
  // 恢复（dr_cb 成功）后重新武装，可再次触发。
  const rclcpp::Time now = this->now();
  if (!link_down_) {
    link_down_ = true;
    link_down_since_ = now;
  }
  const double down_seconds = (now - link_down_since_).seconds();
  if (!comm_loss_reported_ && down_seconds >= comm_loss_duration_) {
    comm_loss_reported_ = true;
    char data[192];
    std::snprintf(data, sizeof(data),
      "{\"duration_s\":%.1f,\"threshold_s\":%.1f,\"broker\":\"%s\"}",
      down_seconds, comm_loss_duration_, jsonEscape(broker_host_).c_str());
    reportEvent("communication_loss", "critical",
      "车-云通信中断：Kafka 链路持续不可用", data);
  }

  if (producer_ && event_producer_) {
    // librdkafka 内部自动重连（文档 14.6：指数退避 1s/2s/4s/8s，最大 30s），
    // 连通性由投递报告回调证实，此处不再直接置位 kafka_connected_
    producer_->poll(0);
    event_producer_->poll(0);
    return;
  }
  // 指数退避重连（文档 14.6）
  reconnect_backoff_ = std::min(reconnect_backoff_ * 2.0, 30.0);
  RCLCPP_INFO(get_logger(), "Kafka 重连尝试（当前退避 %.1fs）", reconnect_backoff_);
  if (kafkaInit()) {
    reconnect_backoff_ = 1.0;
    RCLCPP_INFO(get_logger(), "Kafka 重连成功（等待投递证实）");
  }
}

bool DataAgent::sqliteInit()
{
  // 先确保数据库所在目录存在：sqlite3_open 遇到父目录缺失会直接返回
  // SQLITE_CANTOPEN（"unable to open database file"），不会自动建目录。
  try {
    const std::filesystem::path db_path(db_path_);
    if (db_path.has_parent_path()) {
      std::filesystem::create_directories(db_path.parent_path());
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "创建数据库目录失败: %s", e.what());
    return false;
  }

  const int rc = sqlite3_open(db_path_.c_str(), &db_);
  if (rc != SQLITE_OK) {
    RCLCPP_ERROR(get_logger(), "SQLite 打开失败: %s",
      db_ ? sqlite3_errmsg(db_) : "unknown");
    return false;
  }
  // 表名沿用历史名 telemetry_cache，但实际存放 telemetry/health/event 三类报文，
  // 因此必须带 topic 列（回放时才能送回正确 Topic）
  const char * sql =
    "CREATE TABLE IF NOT EXISTS telemetry_cache ("
    "id INTEGER PRIMARY KEY AUTOINCREMENT, "
    "topic TEXT NOT NULL DEFAULT 'telemetry', "
    "timestamp REAL, "
    "payload TEXT);";
  char * errmsg = nullptr;
  if (sqlite3_exec(db_, sql, nullptr, nullptr, &errmsg) != SQLITE_OK) {
    RCLCPP_ERROR(get_logger(), "SQLite 建表失败: %s", errmsg ? errmsg : "unknown");
    sqlite3_free(errmsg);
    return false;
  }
  // 旧版本升级：无 topic 列时 ALTER（旧行默认归 telemetry）
  bool has_topic = false;
  sqlite3_stmt * pragma = nullptr;
  if (sqlite3_prepare_v2(db_, "PRAGMA table_info(telemetry_cache)", -1, &pragma, nullptr) ==
    SQLITE_OK)
  {
    while (sqlite3_step(pragma) == SQLITE_ROW) {
      const char * col = reinterpret_cast<const char *>(sqlite3_column_text(pragma, 1));
      if (col && std::string(col) == "topic") {
        has_topic = true;
      }
    }
    sqlite3_finalize(pragma);
  }
  if (!has_topic) {
    if (sqlite3_exec(db_,
      "ALTER TABLE telemetry_cache ADD COLUMN topic TEXT DEFAULT 'telemetry';",
      nullptr, nullptr, &errmsg) != SQLITE_OK)
    {
      RCLCPP_ERROR(get_logger(), "SQLite 缓存表升级失败: %s", errmsg ? errmsg : "unknown");
      sqlite3_free(errmsg);
      return false;
    }
    RCLCPP_INFO(get_logger(), "SQLite 缓存表已升级为带 topic 列（支持多 Topic 断点续传）");
  }
  return true;
}

void DataAgent::sqliteCache(const std::string & topic, const std::string & payload)
{
  std::lock_guard<std::mutex> guard(db_mutex_);
  if (!db_) {
    return;
  }
  // 清理超过 cache_max_hours_ 的旧数据（文档 14.6：最多缓存 24 小时）
  const std::string cleanup =
    "DELETE FROM telemetry_cache WHERE timestamp < " +
    std::to_string(this->now().seconds() - cache_max_hours_ * 3600.0);
  sqlite3_exec(db_, cleanup.c_str(), nullptr, nullptr, nullptr);

  // 插入缓存（预处理语句，避免注入）
  sqlite3_stmt * stmt = nullptr;
  const char * sql =
    "INSERT INTO telemetry_cache (topic, timestamp, payload) VALUES (?, ?, ?);";
  if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) == SQLITE_OK) {
    sqlite3_bind_text(stmt, 1, topic.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_double(stmt, 2, this->now().seconds());
    sqlite3_bind_text(stmt, 3, payload.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_step(stmt);
    sqlite3_finalize(stmt);
  }
}

void DataAgent::sqliteReplay()
{
  // 断点续传（文档 14.6）：链路已证实可用时按 id 升序回放缓存报文。
  // 删行时机在 dr_cb：只有送达证实才删，因此是“至少一次”，平台侧靠幂等收敛。
  if (!db_ || vehicle_id_.empty()) {
    return;
  }
  const int inflight = replay_inflight_.load(std::memory_order_relaxed);
  if (inflight >= replay_inflight_limit_) {
    return;   // 在途已多，下轮再说（避免把缓存当作“已发送”假象）
  }
  const int want = std::min(replay_batch_size_, replay_inflight_limit_ - inflight);

  struct Row
  {
    std::int64_t id;
    std::string topic;
    std::string payload;
  };
  std::vector<Row> rows;
  {
    std::lock_guard<std::mutex> guard(db_mutex_);
    sqlite3_stmt * stmt = nullptr;
    const char * sql =
      "SELECT id, topic, payload FROM telemetry_cache ORDER BY id ASC LIMIT ?;";
    if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) == SQLITE_OK) {
      sqlite3_bind_int(stmt, 1, want);
      while (sqlite3_step(stmt) == SQLITE_ROW) {
        Row row;
        row.id = sqlite3_column_int64(stmt, 0);
        const auto topic = reinterpret_cast<const char *>(sqlite3_column_text(stmt, 1));
        const auto payload = reinterpret_cast<const char *>(sqlite3_column_text(stmt, 2));
        row.topic = topic ? topic : "telemetry";
        row.payload = payload ? payload : "";
        rows.push_back(std::move(row));
      }
      sqlite3_finalize(stmt);
    }
  }

  int queued = 0;
  for (auto & row : rows) {
    // 旧行可能只存了类型名（'telemetry'/'event'/'health'），补全为完整 Topic
    if (row.topic.find('.') == std::string::npos) {
      row.topic = "hunter." + vehicle_id_ + "." + row.topic;
    }
    RdKafka::Producer * producer = producerFor(row.topic);
    if (!producer) {
      break;  // 对应通道不可用，该行留在库里等下轮
    }
    if (kafkaProduce(producer, row.topic, row.payload, row.id)) {
      ++queued;
    } else {
      break;  // 本地队列又满了 → 停止本轮，避免无效刷屏
    }
  }
  if (queued > 0) {
    RCLCPP_INFO(get_logger(), "断点续传：本轮回放 %d 条缓存报文（累计送达 %lu 条）",
      queued, static_cast<unsigned long>(dr_ok_count_.load()));
  }
}

void DataAgent::sqliteDeleteRow(std::int64_t row_id)
{
  std::lock_guard<std::mutex> guard(db_mutex_);
  if (!db_) {
    return;
  }
  sqlite3_stmt * stmt = nullptr;
  const char * sql = "DELETE FROM telemetry_cache WHERE id = ?;";
  if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) == SQLITE_OK) {
    sqlite3_bind_int64(stmt, 1, row_id);
    sqlite3_step(stmt);
    sqlite3_finalize(stmt);
  }
}

}  // namespace data_agent



