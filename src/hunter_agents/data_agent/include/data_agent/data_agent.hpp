// Copyright 2026 HUNTER Development Team
// 数据采集 Agent 节点声明（文档第 14 章）
#ifndef DATA_AGENT__DATA_AGENT_HPP_
#define DATA_AGENT__DATA_AGENT_HPP_

#include <chrono>
#include <memory>
#include <string>

#include <librdkafka/rdkafkacpp.h>
#include <sqlite3.h>

#include "hunter_msgs/msg/chassis_command.hpp"
#include "hunter_msgs/msg/chassis_state.hpp"
#include "hunter_msgs/msg/detected_object_array.hpp"
#include "hunter_msgs/msg/system_health.hpp"
#include "hunter_msgs/msg/trajectory.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace data_agent
{

class DataAgent : public rclcpp::Node
{
public:
  explicit DataAgent(const rclcpp::NodeOptions & options);
  ~DataAgent() override;

private:
  // 遥测回调（文档 14.2.1）
  void chassisCallback(const hunter_msgs::msg::ChassisState::SharedPtr msg);
  void chassisFeedbackCallback(const hunter_msgs::msg::ChassisState::SharedPtr msg);
  void localizationCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void fusedObjectsCallback(const hunter_msgs::msg::DetectedObjectArray::SharedPtr msg);
  void trajectoryCallback(const hunter_msgs::msg::Trajectory::SharedPtr msg);
  void controlCallback(const hunter_msgs::msg::ChassisCommand::SharedPtr msg);
  void healthCallback(const hunter_msgs::msg::SystemHealth::SharedPtr msg);

  // 定时器
  void packAndPublish();   // 遥测打包 + 上报
  void detectEvents();     // 事件检测（文档 14.3）

  // Kafka
  bool kafkaInit();
  bool kafkaProduce(const std::string & topic, const std::string & payload);
  void kafkaReconnect();   // 指数退避重连（文档 14.6）

  // SQLite 缓存（文档 14.6）
  bool sqliteInit();
  void sqliteCache(const std::string & payload);

  // 事件
  void reportEvent(const std::string & type, const std::string & level);
  void triggerBagRecord(const std::string & event_type);

  // JSON 打包（文档 14.2.2）
  std::string buildTelemetryJson();

  // 参数
  std::string vehicle_id_;
  std::string kafka_brokers_;
  std::string telemetry_topic_;
  std::string event_topic_;
  std::string db_path_;
  double publish_rate_;
  double max_velocity_;        // 文档 19.3：限速 2.0
  double min_battery_soc_;     // 文档 14.3：SOC < 20%
  double hard_accel_;          // 文档 14.3：3.0 m/s²
  double hard_turn_;           // 文档 14.3：0.8 rad/s
  double comm_loss_duration_;  // 文档 14.3：10s
  double cache_max_hours_;     // 文档 14.6：24 小时

  // Kafka / SQLite
  RdKafka::Producer * producer_;
  bool kafka_connected_;
  sqlite3 * db_;

  // 订阅
  rclcpp::Subscription<hunter_msgs::msg::ChassisState>::SharedPtr chassis_sub_;
  rclcpp::Subscription<hunter_msgs::msg::ChassisState>::SharedPtr chassis_feedback_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr localization_sub_;
  rclcpp::Subscription<hunter_msgs::msg::DetectedObjectArray>::SharedPtr fused_sub_;
  rclcpp::Subscription<hunter_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<hunter_msgs::msg::ChassisCommand>::SharedPtr control_sub_;
  rclcpp::Subscription<hunter_msgs::msg::SystemHealth>::SharedPtr health_sub_;

  // 定时器
  rclcpp::TimerBase::SharedPtr pack_timer_;
  rclcpp::TimerBase::SharedPtr event_timer_;

  // 遥测缓存（最新值）
  hunter_msgs::msg::ChassisState chassis_;
  hunter_msgs::msg::ChassisState chassis_feedback_;
  nav_msgs::msg::Odometry localization_;
  hunter_msgs::msg::ChassisCommand control_;
  hunter_msgs::msg::SystemHealth health_;
  int fused_object_count_;
  bool chassis_received_;
  bool chassis_feedback_received_;
  bool localization_received_;
  bool control_received_;
  bool health_received_;

  // 事件检测状态
  double prev_velocity_;
  rclcpp::Time prev_time_;
  double accel_duration_;
  std::string prev_mode_;

  // 重连状态
  double reconnect_backoff_;   // 指数退避
};

}  // namespace data_agent

#endif  // DATA_AGENT__DATA_AGENT_HPP_
