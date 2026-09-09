// Copyright 2026 HUNTER Development Team
// safety_guard — 碰撞防护与运动学安全约束节点（V0.0.85）
//
// 定位：串联在 velocity_smoother 与底盘（hunter_base 订阅 /cmd_vel）之间的
// 【最后一道物理安全闸】，独立于感知融合链（直接消费 /scan），与决策层
// （auto_mission OBSTACLE_AVOID/ESTOP）、模式仲裁（decision_making）
// 互为冗余。职责：
//   1. 碰撞闸：/scan 行进方向扇区最近障碍 < stop_dist → 立即零速；
//      < slow_dist → 线性限速（碰撞预警分级 SLOWDOWN/COLLISION_STOP）
//   2. 感知 fail-safe：/scan 超时或从未到达 → 零速（宁可停车不盲走）
//   3. 急停透传：/estop=true → 零速（decision_making 只停任务调度，
//      Nav2 goal 取消存在延迟，此处在速度指令通道兜底）
//   4. 阿克曼曲率钳制：|w| ≤ |v|/R_min（等价 δ=atan(L·w/v) ≤ atan(L/R_min)）。
//      实车事故根因：低速时 δ→±max_steer 打满转向；钳制后与 Smac
//      minimum_turning_radius 一致，杜绝打满转向缓慢前挪/甩尾
//   5. 速度硬限：|v| ≤ max_linear_vel（第二重限速，防止参数被误调回 2.0）
//   6. 输入看门狗：上游速度指令断流 > cmd_timeout → 零速心跳
//   7. 分级预警：/safety/state 发布 OK/SLOWDOWN/COLLISION_STOP/
//      SCAN_TIMEOUT/CMD_TIMEOUT/ESTOP_PASS
//
// 仅在导航模式（hunter_autonomous_nav mode:=nav）启动：mapping 模式下
// auto_mission cruise 直接发布 /cmd_vel（自带低速与急停逻辑），本节点若
// 在场会与之冲突。建图巡航安全由 cruise_* 参数与人工遥控接管保障。
//
// 参数（默认值与 HunterV2 匹配，AGX_V2 协议实车）：
//   wheelbase=0.65m（hunter_params.hpp HunterV2Params）
//   min_turn_radius=1.9m（与 nav2_params.yaml Smac minimum_turning_radius 一致）

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace hunter_safety
{

class SafetyGuard : public rclcpp::Node
{
public:
  explicit SafetyGuard(const rclcpp::NodeOptions & options)
  : Node("safety_guard", options)
  {
    // ---- 参数 ----
    declare_parameter<double>("wheelbase", 0.65);          // HunterV2 轴距（m）
    declare_parameter<double>("min_turn_radius", 1.9);     // 与 Smac 规划一致（m）
    declare_parameter<double>("max_linear_vel", 0.8);      // 纵向硬限（m/s）
    declare_parameter<double>("stop_dist", 0.6);           // 碰撞急停距离（m）
    declare_parameter<double>("slow_dist", 1.2);           // 减速预警距离（m）
    declare_parameter<double>("sector_half_deg", 60.0);    // 行进方向检测扇区半角（°）
    declare_parameter<double>("scan_timeout", 0.5);        // /scan 超时（s）
    declare_parameter<double>("cmd_timeout", 0.5);         // 上游指令超时（s）
    declare_parameter<double>("control_rate", 20.0);       // 主循环频率（Hz）

    wheelbase_ = get_parameter("wheelbase").as_double();
    min_turn_radius_ = get_parameter("min_turn_radius").as_double();
    max_linear_vel_ = get_parameter("max_linear_vel").as_double();
    stop_dist_ = get_parameter("stop_dist").as_double();
    slow_dist_ = get_parameter("slow_dist").as_double();
    sector_half_rad_ = get_parameter("sector_half_deg").as_double() * M_PI / 180.0;
    scan_timeout_ = get_parameter("scan_timeout").as_double();
    cmd_timeout_ = get_parameter("cmd_timeout").as_double();
    control_rate_ = get_parameter("control_rate").as_double();

    if (min_turn_radius_ <= wheelbase_ * 0.2) {
      RCLCPP_WARN(get_logger(),
        "min_turn_radius=%.2f 过小（<0.2×轴距），强制回 1.9", min_turn_radius_);
      min_turn_radius_ = 1.9;
    }
    if (stop_dist_ >= slow_dist_) {
      RCLCPP_WARN(get_logger(), "stop_dist ≥ slow_dist，修正 slow_dist=stop+0.5");
      slow_dist_ = stop_dist_ + 0.5;
    }

    // ---- 订阅 ----
    // 上游速度指令（velocity_smoother 输出，launch 重映射 cmd_vel_smoothed→cmd_vel_pre_safety）
    cmd_in_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_in", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(data_mutex_);
        last_cmd_ = *msg;
        last_cmd_time_ = now();
        cmd_received_ = true;
      });

    // /scan：唯一防撞数据源（pointcloud_to_laserscan 直出，10Hz、min_height
    // -0.2 已滤地面，不经过聚类/融合，单点失效风险最低）
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(data_mutex_);
        last_scan_ = *msg;
        last_scan_time_ = now();
        scan_received_ = true;
      });

    // 急停透传（decision_making / health_monitor / 人工均可发布）
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/estop", rclcpp::QoS(10).reliable(),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        estop_ = msg->data;
      });

    // 里程计仅用于速度反馈记录（控制不依赖，避免 EKF 单点影响本节点）
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/localization/odom", rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(data_mutex_);
        last_feedback_speed_ = msg->twist.twist.linear.x;
      });

    // ---- 发布 ----
    cmd_out_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    state_pub_ = create_publisher<std_msgs::msg::String>("/safety/state", 10);

    // ---- 主循环 ----
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / control_rate_));
    tick_timer_ = create_wall_timer(period, std::bind(&SafetyGuard::tick, this));

    RCLCPP_INFO(get_logger(),
      "safety_guard 启动：v_max=%.2fm/s, R_min=%.2fm（|w|≤|v|/R_min）, "
      "stop=%.2fm, slow=%.2fm, 扇区±%.0f°, scan超时%.2fs",
      max_linear_vel_, min_turn_radius_, stop_dist_, slow_dist_,
      sector_half_rad_ * 180.0 / M_PI, scan_timeout_);
  }

private:
  enum class State
  {
    OK, SLOWDOWN, COLLISION_STOP, SCAN_TIMEOUT, CMD_TIMEOUT, ESTOP
  };

  static const char * stateName(State s)
  {
    switch (s) {
      case State::OK: return "OK";
      case State::SLOWDOWN: return "SLOWDOWN";
      case State::COLLISION_STOP: return "COLLISION_STOP";
      case State::SCAN_TIMEOUT: return "SCAN_TIMEOUT";
      case State::CMD_TIMEOUT: return "CMD_TIMEOUT";
      case State::ESTOP: return "ESTOP_PASS";
    }
    return "UNKNOWN";
  }

  // 行进方向扇区内的最近有效距离（m）；无有效回波返回 +inf，超时返回 -1.0
  double nearestObstacle(bool forward, bool & scan_ok)
  {
    std::lock_guard<std::mutex> lk(data_mutex_);
    scan_ok = false;
    if (!scan_received_) {
      return -1.0;
    }
    if ((now() - last_scan_time_).seconds() > scan_timeout_) {
      return -1.0;
    }
    scan_ok = true;

    const auto & scan = last_scan_;
    // 行进方向中心角：前进 0（x 前），倒车 π
    const double center = forward ? 0.0 : M_PI;
    double nearest = std::numeric_limits<double>::infinity();

    double angle = static_cast<double>(scan.angle_min);
    for (const float r : scan.ranges) {
      // 归一化角度差到 [-π, π]
      const double diff =
        std::fabs(std::remainder(angle - center, 2.0 * M_PI));
      if (diff <= sector_half_rad_ && std::isfinite(r) &&
        r >= static_cast<double>(scan.range_min) &&
        r <= static_cast<double>(scan.range_max))
      {
        nearest = std::min(nearest, static_cast<double>(r));
      }
      angle += static_cast<double>(scan.angle_increment);
    }
    return nearest;
    // 扇区内全为 inf/无效回波 → infinity：视为无障碍。车前 60° 全无回波
    // 属异常场景，由 costmap 的 scan 清障层与 health_monitor 兜底。
  }

  void publishCmd(double v, double w)
  {
    geometry_msgs::msg::Twist out;
    out.linear.x = v;
    out.angular.z = w;
    cmd_out_pub_->publish(out);
  }

  void transition(State s, const std::string & detail)
  {
    if (s != state_) {
      const char * from = stateName(state_);
      const char * to = stateName(s);
      if (s == State::OK) {
        RCLCPP_INFO(get_logger(), "[安全状态] %s → %s（%s）", from, to, detail.c_str());
      } else {
        RCLCPP_WARN(get_logger(), "[安全状态] %s → %s（%s）", from, to, detail.c_str());
      }
      state_ = s;
    }
    // 预警话题 2Hz 心跳，供 health_monitor / data_agent / rviz 消费
    if ((now() - last_state_pub_).seconds() >= 0.5) {
      std_msgs::msg::String msg;
      msg.data = std::string(stateName(state_)) + "|" + detail;
      state_pub_->publish(msg);
      last_state_pub_ = now();
    }
  }

  void tick()
  {
    const rclcpp::Time now_t = now();

    // 1. 急停透传：最高优先级
    if (estop_) {
      publishCmd(0.0, 0.0);
      transition(State::ESTOP, "/estop=true，速度通道零速兜底");
      return;
    }

    // 2. 感知 fail-safe：/scan 缺失或超时 → 零速（宁可停车不盲走）
    bool scan_ok = false;
    const double d = nearestObstacle(true, scan_ok);
    if (!scan_ok) {
      publishCmd(0.0, 0.0);
      transition(State::SCAN_TIMEOUT,
        scan_received_ ? "/scan 超时 >" + std::to_string(scan_timeout_).substr(0, 4) +
        "s，fail-safe 停车" : "/scan 从未到达，fail-safe 停车");
      return;
    }

    // 3. 上游指令看门狗
    const bool cmd_valid =
      cmd_received_ && (now_t - last_cmd_time_).seconds() <= cmd_timeout_;
    if (!cmd_valid) {
      publishCmd(0.0, 0.0);
      transition(State::CMD_TIMEOUT,
        cmd_received_ ? "上游速度指令断流" : "上游速度指令未到达");
      return;
    }

    // 4. 按行进方向取最近障碍（指令快照在锁内取，避免回调并发撕裂）
    double v_in = 0.0, w_in = 0.0;
    {
      std::lock_guard<std::mutex> lk(data_mutex_);
      v_in = last_cmd_.linear.x;
      w_in = last_cmd_.angular.z;
    }
    const bool forward = v_in >= 0.0;
    const double dist = forward ? d : nearestObstacle(false, scan_ok);

    // 5. 碰撞闸分级：急停 → 线性限速 → 放行
    double v_allow = max_linear_vel_;
    std::string detail = "正常放行";
    State next = State::OK;
    if (dist < stop_dist_) {
      publishCmd(0.0, 0.0);
      transition(State::COLLISION_STOP,
        "行进方向最近障碍 " + std::to_string(dist).substr(0, 5) +
        "m < 急停距离 " + std::to_string(stop_dist_).substr(0, 4) + "m");
      return;
    }
    if (dist < slow_dist_) {
      v_allow = max_linear_vel_ * (dist - stop_dist_) / (slow_dist_ - stop_dist_);
      v_allow = std::max(v_allow, 0.0);
      next = State::SLOWDOWN;
      detail = "行进方向最近障碍 " + std::to_string(dist).substr(0, 5) +
        "m，限速 " + std::to_string(v_allow).substr(0, 5) + "m/s";
    }

    // 6. 速度硬限 + 碰撞限速
    double v_out = std::clamp(v_in, -max_linear_vel_, max_linear_vel_);
    v_out = std::clamp(v_out, -v_allow, v_allow);

    // 7. 阿克曼曲率钳制：|w| ≤ |v|/R_min（tan(atan(L/R))/L ≡ 1/R）
    double w_out = w_in;
    const double w_lim = std::fabs(v_out) / min_turn_radius_;
    w_out = std::clamp(w_out, -w_lim, w_lim);

    publishCmd(v_out, w_out);
    transition(next, detail);
  }

  // 参数
  double wheelbase_{0.65};
  double min_turn_radius_{1.9};
  double max_linear_vel_{0.8};
  double stop_dist_{0.6};
  double slow_dist_{1.2};
  double sector_half_rad_{M_PI / 3.0};
  double scan_timeout_{0.5};
  double cmd_timeout_{0.5};
  double control_rate_{20.0};

  // 状态
  State state_{State::CMD_TIMEOUT};
  rclcpp::Time last_state_pub_{0, 0, RCL_ROS_TIME};
  bool estop_{false};
  bool cmd_received_{false};
  geometry_msgs::msg::Twist last_cmd_;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  bool scan_received_{false};
  sensor_msgs::msg::LaserScan last_scan_;
  rclcpp::Time last_scan_time_{0, 0, RCL_ROS_TIME};
  double last_feedback_speed_{0.0};
  std::mutex data_mutex_;

  // ROS 接口
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_in_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_out_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
};

}  // namespace hunter_safety

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hunter_safety::SafetyGuard>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
