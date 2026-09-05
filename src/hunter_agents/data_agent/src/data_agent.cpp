// Copyright 2026 HUNTER Development Team
// 数据采集 Agent 实现（文档第 14 章）
#include "data_agent/data_agent.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <utility>

namespace data_agent
{

DataAgent::DataAgent(const rclcpp::NodeOptions & options)
: rclcpp::Node("data_agent", options),
  producer_(nullptr),
  kafka_connected_(false),
  db_(nullptr),
  fused_object_count_(0),
  chassis_received_(false),
  localization_received_(false),
  control_received_(false),
  health_received_(false),
  prev_velocity_(0.0),
  accel_duration_(0.0),
  prev_mode_(""),
  reconnect_backoff_(1.0)
{
  // 参数（文档 14.2/14.3/14.6）
  declare_parameter("vehicle_id", "hunter_001");
  declare_parameter("kafka_brokers", "localhost:9092");
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

  vehicle_id_ = get_parameter("vehicle_id").as_string();
  kafka_brokers_ = get_parameter("kafka_brokers").as_string();
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

  // Kafka topic（文档 14.2.2：hunter.{vehicle_id}.telemetry / .event）
  telemetry_topic_ = "hunter." + vehicle_id_ + ".telemetry";
  event_topic_ = "hunter." + vehicle_id_ + ".event";

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

  // 事件检测的上一帧时间必须用节点时钟初始化。
  // 否则 rclcpp::Time 默认时钟源(RCL_SYSTEM_TIME)与 this->now()(RCL_ROS_TIME)不同，
  // detectEvents 里 now - prev_time_ 会抛 "can't subtract times with different time sources"。
  prev_time_ = this->now();

  RCLCPP_INFO(get_logger(), "data_agent 启动：vehicle=%s, brokers=%s",
    vehicle_id_.c_str(), kafka_brokers_.c_str());
}

DataAgent::~DataAgent()
{
  if (producer_) {
    // 退出前尽力投递队列中的残留消息（文档 14.6）
    const RdKafka::ErrorCode err = producer_->flush(kafka_flush_timeout_ms_);
    const int remaining = producer_->outq_len();
    if (remaining > 0) {
      RCLCPP_WARN(
        get_logger(), "Kafka 退出时仍有 %d 条消息未送达（flush: %s），这部分消息将丢失",
        remaining, RdKafka::err2str(err).c_str());
    }
    delete producer_;
    producer_ = nullptr;
  }
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
  const std::string json = buildTelemetryJson();

  if (kafkaReady()) {
    if (!kafkaProduce(telemetry_topic_, json)) {
      // produce 失败（如本地队列满）→ 断线缓存
      sqliteCache(json);
    }
  } else {
    // 链路未证实/已积压 → 本地缓存 + 重连
    sqliteCache(json);
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
  if (msg.err() == RdKafka::ERR_NO_ERROR) {
    dr_ok_count_.fetch_add(1, std::memory_order_relaxed);
    if (!kafka_connected_.load()) {
      RCLCPP_INFO(get_logger(), "Kafka 投递恢复（累计送达 %lu 条）",
        static_cast<unsigned long>(dr_ok_count_.load()));
    }
    kafka_connected_.store(true);
  } else {
    dr_fail_count_.fetch_add(1, std::memory_order_relaxed);
    kafka_connected_.store(false);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "Kafka 投递失败：%s（转本地缓存）", msg.errstr().c_str());
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
    oss << ",\"overall_status\":\"" << health_.overall_status << "\"";
    oss << ",\"cpu_usage\":" << health_.cpu_usage;
    oss << ",\"cpu_temp\":" << health_.cpu_temp;
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
  oss << "{\"vehicle_id\":\"" << vehicle_id_ << "\"";
  oss << ",\"timestamp\":" << this->now().seconds();
  oss << ",\"type\":\"" << type << "\"";
  oss << ",\"level\":\"" << level << "\"}";
  const std::string json = oss.str();

  RCLCPP_WARN(get_logger(), "事件触发：%s (%s)", type.c_str(), level.c_str());

  if (kafkaReady()) {
    if (!kafkaProduce(event_topic_, json)) {
      sqliteCache(json);  // produce 失败时事件也缓存
    }
  } else {
    sqliteCache(json);  // 断线时事件也缓存
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
  std::string errstr;
  RdKafka::Conf * conf = RdKafka::Conf::create(RdKafka::Conf::CONF_GLOBAL);
  if (!conf) {
    return false;
  }
  if (conf->set("bootstrap.servers", kafka_brokers_, errstr) != RdKafka::Conf::CONF_OK) {
    RCLCPP_ERROR(get_logger(), "Kafka 配置失败: %s", errstr.c_str());
    delete conf;
    return false;
  }
  // 投递报告回调：只有它能证实消息是否真正送达 broker
  if (conf->set("dr_cb", this, errstr) != RdKafka::Conf::CONF_OK) {
    RCLCPP_ERROR(get_logger(), "Kafka dr_cb 配置失败: %s", errstr.c_str());
    delete conf;
    return false;
  }
  // broker 不可达时消息最多在本地队列滞留 60s，超时触发投递失败回调，
  // 避免消息无限积压（配合 kafka_queue_limit_ 提前转 SQLite 缓存）
  if (conf->set("message.timeout.ms", "60000", errstr) != RdKafka::Conf::CONF_OK) {
    RCLCPP_WARN(get_logger(), "Kafka message.timeout.ms 配置失败: %s", errstr.c_str());
  }
  if (conf->set("reconnect.backoff.max.ms", "10000", errstr) != RdKafka::Conf::CONF_OK) {
    RCLCPP_WARN(get_logger(), "Kafka reconnect.backoff.max.ms 配置失败: %s", errstr.c_str());
  }
  producer_ = RdKafka::Producer::create(conf, errstr);
  delete conf;
  if (!producer_) {
    RCLCPP_ERROR(get_logger(), "Kafka 生产者创建失败: %s", errstr.c_str());
    return false;
  }
  // 连通性由 dr_cb 证实后再置位（新建 producer 后需重新证实）
  kafka_connected_.store(false);
  return true;
}

bool DataAgent::kafkaProduce(const std::string & topic, const std::string & payload)
{
  if (!producer_) {
    return false;
  }
  // 先 poll 触发投递回调（更新连通性状态），再投递
  producer_->poll(0);
  const RdKafka::ErrorCode err = producer_->produce(
    topic, RdKafka::Topic::PARTITION_UA, RdKafka::Producer::RK_MSG_COPY,
    const_cast<char *>(payload.c_str()), payload.size(),
    nullptr, 0,   // key, key_len
    0, nullptr);  // timestamp, msg_opaque
  if (err != RdKafka::ERR_NO_ERROR) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 10000, "Kafka produce 失败: %s",
      RdKafka::err2str(err).c_str());
    return false;
  }
  return true;
}

void DataAgent::kafkaReconnect()
{
  if (producer_) {
    // librdkafka 内部自动重连（文档 14.6：指数退避 1s/2s/4s/8s，最大 30s），
    // 连通性由投递报告回调证实，此处不再直接置位 kafka_connected_
    producer_->poll(0);
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
  const char * sql =
    "CREATE TABLE IF NOT EXISTS telemetry_cache ("
    "id INTEGER PRIMARY KEY AUTOINCREMENT, "
    "timestamp REAL, "
    "payload TEXT);";
  char * errmsg = nullptr;
  if (sqlite3_exec(db_, sql, nullptr, nullptr, &errmsg) != SQLITE_OK) {
    RCLCPP_ERROR(get_logger(), "SQLite 建表失败: %s", errmsg ? errmsg : "unknown");
    sqlite3_free(errmsg);
    return false;
  }
  return true;
}

void DataAgent::sqliteCache(const std::string & payload)
{
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
    "INSERT INTO telemetry_cache (timestamp, payload) VALUES (?, ?);";
  if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) == SQLITE_OK) {
    sqlite3_bind_double(stmt, 1, this->now().seconds());
    sqlite3_bind_text(stmt, 2, payload.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_step(stmt);
    sqlite3_finalize(stmt);
  }
}

}  // namespace data_agent



