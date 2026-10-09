// Copyright 2026 HUNTER Development Team
// 数据采集 Agent 实现（文档第 14 章）
#include "data_agent/data_agent.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <utility>
#include <vector>

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
  fused_object_count_ = static_cast<int>(msg->objects.size());
}

void DataAgent::trajectoryCallback(const hunter_msgs::msg::Trajectory::SharedPtr)
{
  // 规划轨迹仅统计（文档 14.2.1），此处不缓存完整轨迹
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
  if (!producer_ || !kafka_connected_.load()) {
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

std::string DataAgent::buildTelemetryJson()
{
  // 构建遥测 JSON（文档 14.2.2）
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(3);
  oss << "{";
  oss << "\"vehicle_id\":\"" << vehicle_id_ << "\"";
  oss << ",\"timestamp\":" << this->now().seconds();
  if (chassis_received_) {
    oss << ",\"velocity\":" << chassis_.velocity;
    oss << ",\"steering\":" << chassis_.steering_angle;
    oss << ",\"battery_soc\":" << chassis_.battery_soc;
    oss << ",\"control_mode\":\"" << chassis_.control_mode << "\"";
    oss << ",\"vehicle_state\":\"" << chassis_.vehicle_state << "\"";
  }
  if (chassis_feedback_received_) {
    oss << ",\"fb_velocity\":" << chassis_feedback_.velocity;
    oss << ",\"fb_steering\":" << chassis_feedback_.steering_angle;
  }
  if (localization_received_) {
    oss << ",\"pose_x\":" << localization_.pose.pose.position.x;
    oss << ",\"pose_y\":" << localization_.pose.pose.position.y;
  }
  oss << ",\"fused_object_count\":" << fused_object_count_;
  if (control_received_) {
    oss << ",\"target_velocity\":" << control_.target_velocity;
    oss << ",\"target_steering\":" << control_.target_steering;
  }
  if (health_received_) {
    oss << ",\"overall_status\":\"" << jsonEscape(health_.overall_status) << "\"";
    oss << ",\"cpu_usage\":" << health_.cpu_usage;
    oss << ",\"cpu_temp\":" << health_.cpu_temp;
  }
  oss << "}";
  return oss.str();
}

std::string DataAgent::buildHealthJson()
{
  // health Topic（接入包契约）：平台侧设备健康页与 last_online_time 数据源
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2);
  oss << "{";
  oss << "\"vehicle_id\":\"" << jsonEscape(vehicle_id_) << "\"";
  oss << ",\"timestamp\":" << this->now().seconds();
  if (health_received_) {
    oss << ",\"overall_status\":\"" << jsonEscape(health_.overall_status) << "\"";
    oss << ",\"cpu_usage\":" << health_.cpu_usage;
    oss << ",\"gpu_usage\":" << health_.gpu_usage;
    oss << ",\"memory_usage\":" << health_.memory_usage;
    oss << ",\"disk_usage\":" << health_.disk_usage;
    oss << ",\"cpu_temp\":" << health_.cpu_temp;
    oss << ",\"gpu_temp\":" << health_.gpu_temp;
    oss << ",\"nodes\":[";
    for (size_t i = 0; i < health_.node_names.size(); ++i) {
      if (i > 0) {
        oss << ",";
      }
      const std::string state = i < health_.node_states.size() ? health_.node_states[i] : "";
      oss << "{\"name\":\"" << jsonEscape(health_.node_names[i])
          << "\",\"state\":\"" << jsonEscape(state) << "\"}";
    }
    oss << "]";
  } else {
    // 不能把“health_monitor 没数据”伪造成 OK
    oss << ",\"overall_status\":\"NO_DATA\"";
  }
  if (chassis_received_) {
    oss << ",\"battery_soc\":" << chassis_.battery_soc;
    oss << ",\"velocity\":" << chassis_.velocity;
    oss << ",\"control_mode\":\"" << jsonEscape(chassis_.control_mode) << "\"";
  }
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

  // 1. 急加速/急减速（文档 14.3：加速度 > 3.0 m/s² 持续 0.5s）
  if (dt > 0.0 && dt < 1.0) {
    const double accel = (chassis_.velocity - prev_velocity_) / dt;
    if (std::fabs(accel) > hard_accel_) {
      accel_duration_ += dt;
      if (accel_duration_ > 0.5) {
        reportEvent(accel > 0 ? "hard_acceleration" : "hard_deceleration", "warning");
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
    reportEvent("hard_turn", "warning");
  }

  // 3. 超速（文档 14.3：速度 > 限速 × 1.1）
  if (v > max_velocity_ * 1.1) {
    reportEvent("overspeed", "critical");
  }

  // 4. 电池低电量（文档 14.3：SOC < 20%）
  if (chassis_.battery_soc > 0.0f && chassis_.battery_soc < min_battery_soc_) {
    reportEvent("low_battery", "warning");
  }

  // 5. 紧急制动（文档 14.3：ESTOP 触发）
  if (chassis_.vehicle_state == "ESTOP" || chassis_.control_mode == "ESTOP") {
    reportEvent("emergency_stop", "critical");
  }

  // 6. 人工接管（文档 14.3：模式切换为 REMOTE）
  if (prev_mode_ != "REMOTE" && chassis_.control_mode == "REMOTE") {
    reportEvent("manual_takeover", "info");
  }

  prev_velocity_ = chassis_.velocity;
  prev_time_ = now;
  prev_mode_ = chassis_.control_mode;
}

void DataAgent::reportEvent(const std::string & type, const std::string & level)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(3);
  oss << "{\"vehicle_id\":\"" << jsonEscape(vehicle_id_) << "\"";
  oss << ",\"timestamp\":" << this->now().seconds();
  oss << ",\"type\":\"" << jsonEscape(type) << "\"";
  oss << ",\"level\":\"" << jsonEscape(level) << "\"}";
  const std::string json = oss.str();

  RCLCPP_WARN(get_logger(), "事件触发：%s (%s)", type.c_str(), level.c_str());
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

  triggerBagRecord(type);
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



