// Copyright 2026 HUNTER Development Team
// 模式仲裁决策节点声明（文档 13.5 控制模式：ESTOP > REMOTE > AUTO）
#ifndef DECISION_MAKING__DECISION_MAKING_HPP_
#define DECISION_MAKING__DECISION_MAKING_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "hunter_msgs/msg/behavior_state.hpp"
#include "hunter_msgs/msg/chassis_command.hpp"
#include "hunter_msgs/msg/chassis_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/bool.hpp"

namespace decision_making
{

class DecisionMaking : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit DecisionMaking(const rclcpp::NodeOptions & options);

  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void remoteCallback(const hunter_msgs::msg::ChassisCommand::SharedPtr msg);
  void chassisCallback(const hunter_msgs::msg::ChassisState::SharedPtr msg);
  void estopCallback(const std_msgs::msg::Bool::SharedPtr msg);

  // 模式仲裁（定时器，50Hz）
  void arbitrate();

  // 参数
  std::string cmd_vel_topic_, remote_topic_, chassis_topic_, estop_topic_;
  std::string command_topic_, behavior_state_topic_;
  double wheelbase_;              // 轴距 0.46m（文档 3.1）
  double cmd_vel_timeout_;        // Nav2 指令超时
  double remote_timeout_;         // 远程指令超时
  double chassis_timeout_;        // 底盘通信超时
  double max_velocity_;           // 限速 2.0 m/s（文档 10.5）
  double max_steering_;           // 转向限幅 ±0.4 rad（文档 4.3）

  // 订阅
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<hunter_msgs::msg::ChassisCommand>::SharedPtr remote_sub_;
  rclcpp::Subscription<hunter_msgs::msg::ChassisState>::SharedPtr chassis_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;

  // 发布
  rclcpp_lifecycle::LifecyclePublisher<hunter_msgs::msg::ChassisCommand>::SharedPtr
    command_pub_;
  rclcpp_lifecycle::LifecyclePublisher<hunter_msgs::msg::BehaviorState>::SharedPtr
    behavior_state_pub_;

  // 定时器
  rclcpp::TimerBase::SharedPtr arbitrate_timer_;

  // 状态
  std::string mode_;               // AUTO / REMOTE / ESTOP
  geometry_msgs::msg::Twist cmd_vel_;
  hunter_msgs::msg::ChassisCommand remote_cmd_;
  hunter_msgs::msg::ChassisState chassis_state_;
  bool estop_;
  bool cmd_vel_received_;
  bool remote_received_;
  bool chassis_received_;
  rclcpp::Time cmd_vel_stamp_;
  rclcpp::Time remote_stamp_;
  rclcpp::Time chassis_stamp_;
};

}  // namespace decision_making

#endif  // DECISION_MAKING__DECISION_MAKING_HPP_
