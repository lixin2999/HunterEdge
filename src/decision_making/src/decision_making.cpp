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
  cmd_vel_received_(false),
  remote_received_(false),
  chassis_received_(false)
{
  // 参数（文档 13.5/10.5/3.1）
  declare_parameter("cmd_vel_topic", "/cmd_vel");
  declare_parameter("remote_topic", "/remote/command");
  declare_parameter("chassis_topic", "/chassis/state");
  declare_parameter("estop_topic", "/estop");
  declare_parameter("command_topic", "/control/command");
  declare_parameter("behavior_state_topic", "/planning/behavior_state");
  declare_parameter("wheelbase", 0.46);        // 文档 3.1：轴距
  declare_parameter("cmd_vel_timeout", 0.5);   // 文档 10.5：/cmd_vel 超时 500ms 停车
  declare_parameter("remote_timeout", 0.5);    // 文档 13.4：远程指令超时 500ms 停车
  declare_parameter("chassis_timeout", 0.1);   // 文档 10.5：CAN 丢失 100ms 制动
  declare_parameter("max_velocity", 2.0);      // 文档 10.5：限速 2.0 m/s
  declare_parameter("max_steering", 0.4);      // 文档 4.3：转向 ±0.4 rad
}

DecisionMaking::CallbackReturn
DecisionMaking::on_configure(const rclcpp_lifecycle::State &)
{
  cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
  remote_topic_ = get_parameter("remote_topic").as_string();
  chassis_topic_ = get_parameter("chassis_topic").as_string();
  estop_topic_ = get_parameter("estop_topic").as_string();
  command_topic_ = get_parameter("command_topic").as_string();
  behavior_state_topic_ = get_parameter("behavior_state_topic").as_string();
  wheelbase_ = get_parameter("wheelbase").as_double();
  cmd_vel_timeout_ = get_parameter("cmd_vel_timeout").as_double();
  remote_timeout_ = get_parameter("remote_timeout").as_double();
  chassis_timeout_ = get_parameter("chassis_timeout").as_double();
  max_velocity_ = get_parameter("max_velocity").as_double();
  max_steering_ = get_parameter("max_steering").as_double();

  // 订阅
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic_, rclcpp::SensorDataQoS(),
    std::bind(&DecisionMaking::cmdVelCallback, this, std::placeholders::_1));
  remote_sub_ = create_subscription<hunter_msgs::msg::ChassisCommand>(
    remote_topic_, rclcpp::SensorDataQoS(),
    std::bind(&DecisionMaking::remoteCallback, this, std::placeholders::_1));
  chassis_sub_ = create_subscription<hunter_msgs::msg::ChassisState>(
    chassis_topic_, rclcpp::SensorDataQoS(),
    std::bind(&DecisionMaking::chassisCallback, this, std::placeholders::_1));
  estop_sub_ = create_subscription<std_msgs::msg::Bool>(
    estop_topic_, rclcpp::SensorDataQoS(),
    std::bind(&DecisionMaking::estopCallback, this, std::placeholders::_1));

  // 发布
  command_pub_ =
    create_publisher<hunter_msgs::msg::ChassisCommand>(command_topic_, 10);
  behavior_state_pub_ =
    create_publisher<hunter_msgs::msg::BehaviorState>(behavior_state_topic_, 10);

  RCLCPP_INFO(get_logger(), "decision_making 已配置：限速=%.1fm/s, 转向限幅=%.2frad",
    max_velocity_, max_steering_);
  return CallbackReturn::SUCCESS;
}

DecisionMaking::CallbackReturn
DecisionMaking::on_activate(const rclcpp_lifecycle::State &)
{
  command_pub_->on_activate();
  behavior_state_pub_->on_activate();

  // 仲裁定时器 50Hz（文档 13.5 控制频率）
  arbitrate_timer_ = create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&DecisionMaking::arbitrate, this));

  RCLCPP_INFO(get_logger(), "decision_making 已激活，模式仲裁 50Hz");
  return CallbackReturn::SUCCESS;
}

DecisionMaking::CallbackReturn
DecisionMaking::on_deactivate(const rclcpp_lifecycle::State &)
{
  arbitrate_timer_.reset();
  command_pub_->on_deactivate();
  behavior_state_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

DecisionMaking::CallbackReturn
DecisionMaking::on_cleanup(const rclcpp_lifecycle::State &)
{
  cmd_vel_sub_.reset();
  remote_sub_.reset();
  chassis_sub_.reset();
  estop_sub_.reset();
  command_pub_.reset();
  behavior_state_pub_.reset();
  return CallbackReturn::SUCCESS;
}

DecisionMaking::CallbackReturn
DecisionMaking::on_shutdown(const rclcpp_lifecycle::State &)
{
  arbitrate_timer_.reset();
  return CallbackReturn::SUCCESS;
}

void DecisionMaking::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  cmd_vel_ = *msg;
  cmd_vel_stamp_ = this->now();
  cmd_vel_received_ = true;
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
  const bool cmd_vel_stale =
    !cmd_vel_received_ || (now - cmd_vel_stamp_).seconds() > cmd_vel_timeout_;
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

  // 输出最终 ChassisCommand（文档 4.3.3）
  hunter_msgs::msg::ChassisCommand cmd;
  cmd.header.stamp = now;
  cmd.header.frame_id = "base_link";

  if (mode_ == "ESTOP") {
    // 急停：零速度 + 紧急制动标志（文档 16.2）
    cmd.target_velocity = 0.0;
    cmd.target_steering = 0.0;
    cmd.control_mode = "ESTOP";
    cmd.emergency_stop = true;
  } else if (mode_ == "REMOTE") {
    // 远程：转发远程指令（限幅，文档 13.4）
    cmd.target_velocity = static_cast<float>(
      std::clamp(remote_cmd_.target_velocity, -max_velocity_, max_velocity_));
    cmd.target_steering = static_cast<float>(
      std::clamp(remote_cmd_.target_steering, -max_steering_, max_steering_));
    cmd.control_mode = "REMOTE";
    cmd.emergency_stop = remote_cmd_.emergency_stop;
  } else {
    // AUTO：Nav2 /cmd_vel（Twist）→ 阿克曼转向角
    if (cmd_vel_stale) {
      // Nav2 指令超时，自动停车（文档 10.5）
      cmd.target_velocity = 0.0;
      cmd.target_steering = 0.0;
    } else {
      const double v = cmd_vel_.linear.x;
      const double w = cmd_vel_.angular.z;
      // 阿克曼转向角 δ = atan2(L*ω, v)，L=轴距 0.46m（文档 11.4）
      const double steering = std::atan2(wheelbase_ * w, v);
      cmd.target_velocity = static_cast<float>(
        std::clamp(v, -max_velocity_, max_velocity_));
      cmd.target_steering = static_cast<float>(
        std::clamp(steering, -max_steering_, max_steering_));
    }
    cmd.control_mode = "CAN";
    cmd.emergency_stop = false;
  }
  command_pub_->publish(cmd);

  // 发布行为状态（文档 17.1：/planning/behavior_state）
  hunter_msgs::msg::BehaviorState state;
  state.header.stamp = now;
  state.mode = mode_;
  behavior_state_pub_->publish(state);
}

}  // namespace decision_making

