// Copyright 2026 HUNTER Development Team
// 模式仲裁决策节点声明（文档 13.5 控制模式：ESTOP > REMOTE > AUTO）
// 修正：移除对 /cmd_vel 的订阅和对 /control/command 的发布，回归文档定义的"只输出行为状态"契约
#ifndef DECISION_MAKING__DECISION_MAKING_HPP_
#define DECISION_MAKING__DECISION_MAKING_HPP_

#include <chrono>
#include <memory>
#include <string>

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
  void remoteCallback(const hunter_msgs::msg::ChassisCommand::SharedPtr msg);
  void chassisCallback(const hunter_msgs::msg::ChassisState::SharedPtr msg);
  void estopCallback(const std_msgs::msg::Bool::SharedPtr msg);

  // 模式仲裁（定时器，10Hz）
  void arbitrate();

  // 参数
  std::string remote_topic_, chassis_topic_, estop_topic_;
  std::string behavior_state_topic_;
  double remote_timeout_;         // 远程指令超时
  double chassis_timeout_;        // 底盘通信超时

  // 订阅
  rclcpp::Subscription<hunter_msgs::msg::ChassisCommand>::SharedPtr remote_sub_;
  rclcpp::Subscription<hunter_msgs::msg::ChassisState>::SharedPtr chassis_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;

  // 发布
  rclcpp_lifecycle::LifecyclePublisher<hunter_msgs::msg::BehaviorState>::SharedPtr
    behavior_state_pub_;

  // 定时器
  rclcpp::TimerBase::SharedPtr arbitrate_timer_;

  // 状态
  std::string mode_;               // AUTO / REMOTE / ESTOP
  hunter_msgs::msg::ChassisCommand remote_cmd_;
  hunter_msgs::msg::ChassisState chassis_state_;
  bool estop_;
  bool remote_received_;
  bool chassis_received_;
  rclcpp::Time remote_stamp_;
  rclcpp::Time chassis_stamp_;
};

}  // namespace decision_making

#endif  // DECISION_MAKING__DECISION_MAKING_HPP_
