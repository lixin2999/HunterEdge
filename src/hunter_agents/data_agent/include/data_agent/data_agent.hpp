// Copyright 2026 HUNTER Development Team
// 数据采集 Agent 节点声明（文档第 14 章 + HunterCore 接入包 Topic 契约）
#ifndef DATA_AGENT__DATA_AGENT_HPP_
#define DATA_AGENT__DATA_AGENT_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>

#include <librdkafka/rdkafkacpp.h>
#include <sqlite3.h>

#include "data_agent/kafka_access.hpp"
#include "hunter_msgs/msg/chassis_command.hpp"
#include "hunter_msgs/msg/chassis_state.hpp"
#include "hunter_msgs/msg/detected_object_array.hpp"
#include "hunter_msgs/msg/system_health.hpp"
#include "hunter_msgs/msg/trajectory.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace data_agent
{

class DataAgent : public rclcpp::Node, public RdKafka::DeliveryReportCb
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
  void publishHealth();    // 心跳/健康上报（接入包 health Topic，1Hz）
  void detectEvents();     // 事件检测（文档 14.3）

  // Kafka
  // 接入包契约：telemetry/health 走 acks=1（高频、可容忍丢失），
  // event 走 acks=all（不可丢），二者必须分属不同 producer
  bool kafkaInit();
  bool kafkaProduce(RdKafka::Producer * producer, const std::string & topic,
    const std::string & payload, std::int64_t cache_row_id = -1);
  void kafkaReconnect();   // 指数退避重连（文档 14.6）
  // 链路可用判定：producer 存在 + 投递已被证实可用 + 本地队列未积压。
  // 注意：produce() 成功只代表消息进入本地队列，不等于送达 broker
  bool kafkaReady();
  // 按 Topic 选择 producer（.event → acks=all 通道）
  RdKafka::Producer * producerFor(const std::string & topic);
  // 投递报告回调（librdkafka 内部线程调用，异步证实消息是否真正送达）
  void dr_cb(RdKafka::Message & msg) override;

  // SQLite 缓存（文档 14.6）：断网期间按 Topic 落库，链路恢复后按 id 顺序回放
  bool sqliteInit();
  void sqliteCache(const std::string & topic, const std::string & payload);
  void sqliteReplay();                        // 断点续传：投递证实后才删行
  void sqliteDeleteRow(std::int64_t row_id);

  // 事件（契约 event.schema.json：event_type 19 种 / event_level 3 级）
  void reportEvent(const std::string & event_type, const std::string & event_level,
    const std::string & description, const std::string & data_json);
  void triggerBagRecord(const std::string & event_type);

  // JSON 打包（HunterCore contracts/kafka/schemas/telemetry.schema.json、
  // health.schema.json：**六段嵌套结构 + additionalProperties:false**，
  // 平台 data-collector 以 schema_name="auto" 校验，字段不符即转 DLQ）
  std::string buildTelemetryJson();
  std::string buildHealthJson();
  // 契约 system 段（遥测与 health 共用，契约明确“系统段字段与遥测完全一致”）
  std::string buildSystemSegment() const;
  // 契约 health.status：车辆 8 态业务状态（offline/online_idle/auto_driving/
  // remote_controlled/upgrading/charging/fault/emergency）
  std::string vehicleStatusForContract() const;

  // 参数
  std::string vehicle_id_;
  std::string kafka_brokers_;              // 仅开发机 fallback（properties 存在时被覆盖）
  std::string kafka_properties_path_;      // 接入包单一可信源
  std::string kafka_bundle_dir_;
  std::string telemetry_topic_;
  std::string event_topic_;
  std::string health_topic_;
  std::string db_path_;
  double publish_rate_;
  double max_velocity_;        // 文档 19.3：限速 2.0
  double min_battery_soc_;     // 文档 14.3：SOC < 20%
  double hard_accel_;          // 文档 14.3：3.0 m/s²
  double hard_turn_;           // 文档 14.3：0.8 rad/s
  double comm_loss_duration_;  // 文档 14.3：10s
  double cache_max_hours_;     // 文档 14.6：24 小时

  // Kafka SASL_SSL 认证参数（开发机内联兜底；生产凭据只来自 kafka.properties）
  std::string security_protocol_;   // "SASL_SSL"
  std::string sasl_mechanism_;      // "SCRAM-SHA-512"
  std::string sasl_username_;
  std::string sasl_password_;
  std::string ssl_ca_location_;

  // Kafka / SQLite
  RdKafka::Producer * producer_;        // acks=1：telemetry + health
  RdKafka::Producer * event_producer_;  // acks=all：event
  std::atomic<bool> kafka_connected_;  // 由投递报告回调异步更新（跨线程）
  int kafka_queue_limit_;              // 本地队列积压上限（条），超过转 SQLite 缓存
  int kafka_flush_timeout_ms_;         // 退出时 flush 超时（毫秒）
  std::atomic<uint64_t> dr_ok_count_{0};    // 投递成功计数
  std::atomic<uint64_t> dr_fail_count_{0};  // 投递失败计数
  std::atomic<int> replay_inflight_{0};     // 已投递未证实的缓存回放条数
  int replay_batch_size_;                   // 每轮回放上限（条）
  int replay_inflight_limit_;               // 在途回放上限（条）
  sqlite3 * db_;
  std::mutex db_mutex_;                       // dr_cb 线程与定时器线程共用连接

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
  rclcpp::TimerBase::SharedPtr health_timer_;

  // 遥测缓存（最新值）
  hunter_msgs::msg::ChassisState chassis_;
  hunter_msgs::msg::ChassisState chassis_feedback_;
  nav_msgs::msg::Odometry localization_;
  hunter_msgs::msg::ChassisCommand control_;
  hunter_msgs::msg::SystemHealth health_;
  int fused_object_count_;
  bool chassis_received_{false};
  bool chassis_feedback_received_{false};
  bool localization_received_{false};
  bool control_received_{false};
  bool health_received_{false};

  // ── HunterCore 契约上报所需的统计量（contracts/kafka/schemas/*.schema.json）──
  std::uint64_t seq_{0};                        // 遥测序号（契约 seq：单调递增，平台幂等/丢包检测）
  std::map<std::string, int> fused_object_types_;  // 感知按类别计数（契约 perception.object_types）
  std::size_t fused_frames_in_window_{0};       // 感知 FPS 统计窗口内帧数
  rclcpp::Time perception_window_start_;        // 感知 FPS 统计窗口起点
  double perception_fps_{0.0};                  // 契约 perception.fps
  double perception_latency_ms_{0.0};           // 契约 perception.latency_ms（帧头时间戳到本节点的时延）
  int trajectory_points_{0};                    // 契约 planning.trajectory_points
  double trajectory_length_{0.0};               // 契约 planning.trajectory_length（相邻点距离累加）
  bool trajectory_received_{false};
  double planning_latency_ms_{0.0};             // 契约 planning.planning_latency_ms
  double network_latency_ms_{0.0};              // 契约 system.network_latency_ms（到 Kafka broker 的 TCP 建连时延，1Hz 采样）
  int network_rssi_{0};                         // 契约 system.network_rssi（无无线制式上报 0，≤0 合规）
  std::string broker_host_;                     // 由 bootstrap.servers 解析出的首个 broker 主机
  int broker_port_{0};                          // 对应端口（用于 network_latency_ms 采样）

  // 车-云链路中断检测（契约 event_type=communication_loss）
  bool link_down_{false};
  bool comm_loss_reported_{false};
  rclcpp::Time link_down_since_;

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
