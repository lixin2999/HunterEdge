// Copyright 2026 HUNTER Development Team
// 模式仲裁决策节点实现（文档 13.5：ESTOP > REMOTE > AUTO）
#include "decision_making/decision_making.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace decision_making
{

DecisionMaking::DecisionMaking(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("decision_making", options),
  mode_("ESTOP"),   // 初始急停，安全起见
  estop_(false),
  remote_received_(false),
  chassis_received_(false)
{
  // 参数（文档 13.5/10.5/3.1）
  declare_parameter("remote_topic", "/remote/command");
  declare_parameter("chassis_topic", "/chassis/state");
  declare_parameter("estop_topic", "/estop");
  declare_parameter("behavior_state_topic", "/planning/behavior_state");
  declare_parameter("remote_timeout", 0.5);    // 文档 13.4：远程指令超时 500ms 停车
  declare_parameter("chassis_timeout", 0.1);   // 文档 10.5：CAN 丢失 100ms 制动
}

DecisionMaking::CallbackReturn
DecisionMaking::on_configure(const rclcpp_lifecycle::State &)
{
  remote_topic_ = get_parameter("remote_topic").as_string();
  chassis_topic_ = get_parameter("chassis_topic").as_string();
  estop_topic_ = get_parameter("estop_topic").as_string();
  behavior_state_topic_ = get_parameter("behavior_state_topic").as_string();
  remote_timeout_ = get_parameter("remote_timeout").as_double();
  chassis_timeout_ = get_parameter("chassis_timeout").as_double();

  // 订阅：只订阅所需状态，不订阅 /cmd_vel
  remote_sub_ = create_subscription<hunter_msgs::msg::ChassisCommand>(
    remote_topic_, rclcpp::SensorDataQoS(),
    std::bind(&DecisionMaking::remoteCallback, this, std::placeholders::_1));
  chassis_sub_ = create_subscription<hunter_msgs::msg::ChassisState>(
    chassis_topic_, rclcpp::SensorDataQoS(),
    std::bind(&DecisionMaking::chassisCallback, this, std::placeholders::_1));
  estop_sub_ = create_subscription<std_msgs::msg::Bool>(
    estop_topic_, rclcpp::SensorDataQoS(),
    std::bind(&DecisionMaking::estopCallback, this, std::placeholders::_1));

  // 发布：仅发布行为状态给 planning/control，不越权发布最终指令
  behavior_state_pub_ =
    create_publisher<hunter_msgs::msg::BehaviorState>(behavior_state_topic_, 10);

  RCLCPP_INFO(get_logger(), "decision_making 已配置");
  return CallbackReturn::SUCCESS;
}

DecisionMaking::CallbackReturn
DecisionMaking::on_activate(const rclcpp_lifecycle::State &)
{
  behavior_state_pub_->on_activate();

  // 仲裁定时器 10Hz（文档 17.1：/planning/behavior_state 频率）
  arbitrate_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&DecisionMaking::arbitrate, this));

  RCLCPP_INFO(get_logger(), "decision_making 已激活，模式仲裁 10Hz");
  return CallbackReturn::SUCCESS;
}

DecisionMaking::CallbackReturn
DecisionMaking::on_deactivate(const rclcpp_lifecycle::State &)
{
  arbitrate_timer_.reset();
  behavior_state_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

DecisionMaking::CallbackReturn
DecisionMaking::on_cleanup(const rclcpp_lifecycle::State &)
{
  remote_sub_.reset();
  chassis_sub_.reset();
  estop_sub_.reset();
  behavior_state_pub_.reset();
  return CallbackReturn::SUCCESS;
}

DecisionMaking::CallbackReturn
DecisionMaking::on_shutdown(const rclcpp_lifecycle::State &)
{
  arbitrate_timer_.reset();
  return CallbackReturn::SUCCESS;
}

void DecisionMaking::remoteCallback(
  const hunter_msgs::msg::ChassisCommand::SharedPtr msg)
{
  remote_cmd_ = *msg;
  remote_stamp_ = this->now();
  remote_received_ = true;
}

void DecisionMaking::chassisCallback(
  const hunter_msgs::msg::ChassisState::SharedPtr msg)
{
  chassis_state_ = *msg;
  chassis_stamp_ = this->now();
  chassis_received_ = true;
}

void DecisionMaking::estopCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  estop_ = msg->data;
}

void DecisionMaking::arbitrate()
{
  const rclcpp::Time now = this->now();

  // 超时判断
  const bool remote_stale =
    !remote_received_ || (now - remote_stamp_).seconds() > remote_timeout_;
  const bool chassis_stale =
    !chassis_received_ || (now - chassis_stamp_).seconds() > chassis_timeout_;

  // 底盘故障（文档 16.2：急停触发条件）
  const bool chassis_fault = chassis_received_ &&
    (chassis_state_.fault_code != 0 ||
     chassis_state_.vehicle_state == "FAULT" ||
     chassis_state_.vehicle_state == "ESTOP");

  // 模式仲裁（优先级 ESTOP > REMOTE > AUTO，文档 13.5）
  std::string new_mode;
  if (estop_ || chassis_fault || chassis_stale) {
    new_mode = "ESTOP";
  } else if (!remote_stale) {
    new_mode = "REMOTE";
  } else {
    new_mode = "AUTO";
  }

  if (new_mode != mode_) {
    RCLCPP_WARN(get_logger(), "模式切换：%s → %s", mode_.c_str(), new_mode.c_str());
  }
  mode_ = new_mode;

  // 修正：不再组装和发布 ChassisCommand 到 /control/command
  // 而是仅输出当前 BehaviorState 供下游规划/控制模块消费执行停车或避障逻辑
  hunter_msgs::msg::BehaviorState state;
  state.header.stamp = now;
  state.mode = mode_;
  behavior_state_pub_->publish(state);
}

}  // namespace decision_making

