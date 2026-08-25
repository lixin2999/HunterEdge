// Copyright 2026 HUNTER Development Team
// 系统监控健康管理节点声明（文档第 15 章）
#ifndef HEALTH_MONITOR__HEALTH_MONITOR_HPP_
#define HEALTH_MONITOR__HEALTH_MONITOR_HPP_

#include <chrono>
#include <map>
#include <string>
#include <vector>

#include "hunter_msgs/msg/chassis_state.hpp"
#include "hunter_msgs/msg/system_health.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"

namespace health_monitor
{

// 传感器话题频率监控（文档 15.3）
struct TopicMonitor
{
  std::string name;
  double min_rate;        // 异常阈值（Hz）
  double duration;        // 持续时长（秒）
  int msg_count;
  rclcpp::Time window_start;
  bool anomaly;
  double anomaly_since;   // 异常起始（秒，-1 表示正常）
};

class HealthMonitor : public rclcpp::Node
{
public:
  explicit HealthMonitor(const rclcpp::NodeOptions & options);

private:
  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void chassisCallback(const hunter_msgs::msg::ChassisState::SharedPtr msg);

  void checkHealth();   // 1Hz：资源采集 + 频率监控 + 健康上报
  void checkNodes();    // 2s：节点存活监控 + 重启计数

  // 资源采集（Linux /proc、/sys）
  double readCpuUsage();
  double readMemoryUsage();
  double readDiskUsage();
  double readTemp(const std::string & zone);

  // 参数
  std::vector<std::string> critical_nodes_;
  double node_check_interval_;
  double report_interval_;
  int restart_limit_;      // 文档 15.2：5 分钟内重启 > 3 次
  double restart_window_;
  double warning_temp_;    // 文档 3.5：85℃
  double critical_temp_;   // 文档 3.5：95℃

  // 订阅（传感器频率监控）
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<hunter_msgs::msg::ChassisState>::SharedPtr chassis_sub_;

  // 发布
  rclcpp::Publisher<hunter_msgs::msg::SystemHealth>::SharedPtr health_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;

  // 定时器
  rclcpp::TimerBase::SharedPtr health_timer_;
  rclcpp::TimerBase::SharedPtr node_timer_;

  // 传感器监控（文档 15.3 阈值）
  TopicMonitor lidar_mon_;
  TopicMonitor camera_mon_;
  TopicMonitor imu_mon_;
  TopicMonitor can_mon_;

  // 节点重启计数（文档 15.2）
  std::map<std::string, int> restart_count_;
  std::map<std::string, bool> node_was_alive_;

  // CPU 采样（计算使用率需两次采样）
  unsigned long long prev_cpu_total_;
  unsigned long long prev_cpu_idle_;
  bool cpu_sampled_;
};

}  // namespace health_monitor

#endif  // HEALTH_MONITOR__HEALTH_MONITOR_HPP_
