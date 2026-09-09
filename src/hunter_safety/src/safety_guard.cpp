// Copyright 2026 HUNTER Development Team
// safety_guard — 碰撞防护与运动学安全约束节点（V0.0.86）
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
//      SCAN_TIMEOUT/CMD_TIMEOUT/ESTOP_PASS/TEST_ABORTED
//   8. 自动驾驶测试模式（V0.0.86）：/safety/test_mode true/false 运行时开关。
//      开启后 0.1m/s 限速巡航、更严碰撞阈值（±60° 扇区 <1.0m 急停/<2.0m 减速），
//      并监控三类异常——疑似碰撞卡死（有指令无反馈）、goal 活跃而控制器
//      指令断流、Nav2 goal ABORTED——任一发生立即零速锁存并中止：
//      发布 /estop=true（latched，auto_mission 取消全部导航任务）+
//      async_cancel_all_goals() 取消 /navigate_to_pose 活动目标；
//      中止状态需人工重新发布 /safety/test_mode true 解除。
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
#include <cstdint>
#include <limits>
#include <mutex>
#include <string>

#include "action_msgs/msg/goal_status_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
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
    // V0.0.86 自动驾驶测试模式
    declare_parameter<bool>("enable_test_mode", false);    // 启动即进入测试模式
    declare_parameter<double>("test_max_linear_vel", 0.1); // 测试模式限速（m/s）
    declare_parameter<double>("test_stop_dist", 1.0);      // 测试模式急停距离（m）
    declare_parameter<double>("test_slow_dist", 2.0);      // 测试模式减速距离（m）
    declare_parameter<double>("stall_timeout", 1.0);       // 卡死判定时长（s）
    declare_parameter<double>("stall_cmd_vel_min", 0.05);  // 判"有指令"的最小指令速度（m/s）
    declare_parameter<double>("stall_fb_vel_max", 0.02);   // 判"没在动"的最大反馈速度（m/s）
    declare_parameter<double>("plan_fail_timeout", 2.0);   // 控制器指令断流判定（s）
    declare_parameter<bool>("estop_hold_on_abort", true);  // 中止时是否发布 /estop=true

    wheelbase_ = get_parameter("wheelbase").as_double();
    min_turn_radius_ = get_parameter("min_turn_radius").as_double();
    max_linear_vel_ = get_parameter("max_linear_vel").as_double();
    stop_dist_ = get_parameter("stop_dist").as_double();
    slow_dist_ = get_parameter("slow_dist").as_double();
    sector_half_rad_ = get_parameter("sector_half_deg").as_double() * M_PI / 180.0;
    scan_timeout_ = get_parameter("scan_timeout").as_double();
    cmd_timeout_ = get_parameter("cmd_timeout").as_double();
    control_rate_ = get_parameter("control_rate").as_double();
    enable_test_mode_ = get_parameter("enable_test_mode").as_bool();
    test_mode_ = enable_test_mode_;
    test_max_linear_vel_ = get_parameter("test_max_linear_vel").as_double();
    test_stop_dist_ = get_parameter("test_stop_dist").as_double();
    test_slow_dist_ = get_parameter("test_slow_dist").as_double();
    stall_timeout_ = get_parameter("stall_timeout").as_double();
    stall_cmd_vel_min_ = get_parameter("stall_cmd_vel_min").as_double();
    stall_fb_vel_max_ = get_parameter("stall_fb_vel_max").as_double();
    plan_fail_timeout_ = get_parameter("plan_fail_timeout").as_double();
    estop_hold_on_abort_ = get_parameter("estop_hold_on_abort").as_bool();

    if (min_turn_radius_ <= wheelbase_ * 0.2) {
      RCLCPP_WARN(get_logger(),
        "min_turn_radius=%.2f 过小（<0.2×轴距），强制回 1.9", min_turn_radius_);
      min_turn_radius_ = 1.9;
    }
    if (stop_dist_ >= slow_dist_) {
      RCLCPP_WARN(get_logger(), "stop_dist ≥ slow_dist，修正 slow_dist=stop+0.5");
      slow_dist_ = stop_dist_ + 0.5;
    }
    if (test_stop_dist_ >= test_slow_dist_) {
      RCLCPP_WARN(get_logger(),
        "test_stop_dist ≥ test_slow_dist，修正 test_slow_dist=test_stop+0.5");
      test_slow_dist_ = test_stop_dist_ + 0.5;
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

    // V0.0.86 自动驾驶测试模式：运行时开关（人工/上位机发布 true/false）
    test_mode_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/safety/test_mode", rclcpp::QoS(10).reliable(),
      std::bind(&SafetyGuard::testModeCallback, this, std::placeholders::_1));

    // 控制器原始指令（controller_server 输出，经 velocity_smoother 前置）：
    // 测试模式监控其断流——goal 活跃而 /cmd_vel_nav 断流 > plan_fail_timeout
    // 判为局部规划失效
    plan_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_nav", rclcpp::SensorDataQoS(),
      [this](const geometry_msgs::msg::Twist::SharedPtr) {
        std::lock_guard<std::mutex> lk(data_mutex_);
        last_plan_cmd_time_ = now();
        plan_cmd_received_ = true;
      });

    // Nav2 goal 状态（bt_navigator action status topic）：测试模式监控
    // goal ABORTED（局部规划失效，如 RPP 卡死被 Nav2 判死）
    nav_status_sub_ = create_subscription<action_msgs::msg::GoalStatusArray>(
      "/navigate_to_pose/_action/status", rclcpp::QoS(10).reliable(),
      [this](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
        if (!msg->status_list.empty()) {
          std::lock_guard<std::mutex> lk(data_mutex_);
          last_nav_status_ = msg->status_list.back().status;
        }
      });

    // 导航目标取消客户端（中止测试时取消全部活动 goal，双保险：
    // /estop 已让 auto_mission 取消，此处直接对 bt_navigator 再发一次）
    nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      this, "navigate_to_pose");

    // ---- 发布 ----
    cmd_out_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    state_pub_ = create_publisher<std_msgs::msg::String>("/safety/state", 10);
    // V0.0.86 测试模式中止时发布 /estop=true（transient_local 锁存：
    // 迟加入的 auto_mission 也能收到；auto_mission 收到即取消全部导航任务）
    estop_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/estop", rclcpp::QoS(1).reliable().transient_local());

    // ---- 主循环 ----
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / control_rate_));
    tick_timer_ = create_wall_timer(period, std::bind(&SafetyGuard::tick, this));

    RCLCPP_INFO(get_logger(),
      "safety_guard 启动：v_max=%.2fm/s, R_min=%.2fm（|w|≤|v|/R_min）, "
      "stop=%.2fm, slow=%.2fm, 扇区±%.0f°, scan超时%.2fs；"
      "测试模式=%s（限速%.2fm/s, 急停%.1fm, 减速%.1fm, 卡死判定%.1fs, 断流判定%.1fs）",
      max_linear_vel_, min_turn_radius_, stop_dist_, slow_dist_,
      sector_half_rad_ * 180.0 / M_PI, scan_timeout_,
      test_mode_ ? "ON" : "OFF",
      test_max_linear_vel_, test_stop_dist_, test_slow_dist_,
      stall_timeout_, plan_fail_timeout_);
  }

private:
  enum class State
  {
    OK, SLOWDOWN, COLLISION_STOP, SCAN_TIMEOUT, CMD_TIMEOUT, ESTOP, TEST_ABORTED
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
      case State::TEST_ABORTED: return "TEST_ABORTED";
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

    // 0. 测试模式状态快照（锁内取，避免与开关/订阅回调并发撕裂）
    bool test_active = false;
    rclcpp::Time last_plan_cmd{0, 0, RCL_ROS_TIME};
    bool plan_cmd_received = false;
    int8_t nav_status = 0;
    double fb_speed = 0.0;
    {
      std::lock_guard<std::mutex> lk(data_mutex_);
      test_active = test_mode_ && !test_aborted_;
      last_plan_cmd = last_plan_cmd_time_;
      plan_cmd_received = plan_cmd_received_;
      nav_status = last_nav_status_;
      fb_speed = last_feedback_speed_;
    }

    // 0.5 测试模式中止锁存：每拍零速，直到人工重新发布 /safety/test_mode
    // true。即便外部把 /estop 清回 false（auto_mission 收 false 会清自触发
    // 标记），本锁存仍保持零速——防止中止后被误恢复行驶。
    if (test_mode_ && !test_active) {
      publishCmd(0.0, 0.0);
      transition(State::TEST_ABORTED,
        "测试模式已中止（零速锁存），重新开启 /safety/test_mode 解除");
      return;
    }

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

    // 4.5 V0.0.86 测试模式生效参数：0.1m/s 限速 + 更严碰撞阈值
    const double v_lim = test_active ? test_max_linear_vel_ : max_linear_vel_;
    const double stop_d = test_active ? test_stop_dist_ : stop_dist_;
    const double slow_d = test_active ? test_slow_dist_ : slow_dist_;
    const char * test_tag = test_active ? "【测试模式】" : "";

    // 5. 碰撞闸分级：急停 → 线性限速 → 放行
    double v_allow = v_lim;
    std::string detail = std::string("正常放行") + test_tag;
    State next = State::OK;
    if (dist < stop_d) {
      publishCmd(0.0, 0.0);
      transition(State::COLLISION_STOP,
        "行进方向最近障碍 " + std::to_string(dist).substr(0, 5) +
        "m < 急停距离 " + std::to_string(stop_d).substr(0, 4) + "m" + test_tag);
      return;
    }
    if (dist < slow_d) {
      v_allow = v_lim * (dist - stop_d) / (slow_d - stop_d);
      v_allow = std::max(v_allow, 0.0);
      next = State::SLOWDOWN;
      detail = "行进方向最近障碍 " + std::to_string(dist).substr(0, 5) +
        "m，限速 " + std::to_string(v_allow).substr(0, 5) + "m/s" + test_tag;
    }

    // 6. 速度硬限 + 碰撞限速（测试模式 0.1m/s）
    double v_out = std::clamp(v_in, -v_lim, v_lim);
    v_out = std::clamp(v_out, -v_allow, v_allow);

    // 7. 阿克曼曲率钳制：|w| ≤ |v|/R_min（tan(atan(L/R))/L ≡ 1/R）
    double w_out = w_in;
    const double w_lim = std::fabs(v_out) / min_turn_radius_;
    w_out = std::clamp(w_out, -w_lim, w_lim);

    // 7.5 V0.0.86 测试模式异常监控：疑似碰撞卡死 / 控制器断流 / goal
    // ABORTED——任一命中即零速锁存中止
    if (test_active) {
      const std::string abort_reason = checkTestWatchdogs(
        now_t, v_out, fb_speed, last_plan_cmd, plan_cmd_received, nav_status);
      if (!abort_reason.empty()) {
        abortTest(abort_reason);
        publishCmd(0.0, 0.0);
        transition(State::TEST_ABORTED, abort_reason);
        return;
      }
    }

    publishCmd(v_out, w_out);
    transition(next, detail);
  }

  // V0.0.86 测试模式异常监控。返回中止原因（空串=正常）：
  //   a) 疑似碰撞/卡死：速度指令在给（> stall_cmd_vel_min）而底盘反馈速度
  //      ≈0（< stall_fb_vel_max）持续 stall_timeout——碰撞堵转或底盘失联
  //   b) 局部规划失效：Nav2 goal 判 ABORTED（RPP 卡死/无法推进被判死）
  //   c) 局部规划失效：goal 活跃但 /cmd_vel_nav 断流 > plan_fail_timeout
  //      （控制器崩溃/规划器无输出）
  std::string checkTestWatchdogs(
    const rclcpp::Time & now_t, double v_out, double fb_speed,
    const rclcpp::Time & last_plan_cmd, bool plan_cmd_received, int8_t nav_status)
  {
    // a) 碰撞/卡死检测（COLLISION_STOP 急停时指令为 0，不会误触发本条）
    if (std::fabs(v_out) > stall_cmd_vel_min_ &&
      std::fabs(fb_speed) < stall_fb_vel_max_)
    {
      if (!stall_active_) {
        stall_active_ = true;
        stall_start_ = now_t;
      } else if ((now_t - stall_start_).seconds() > stall_timeout_) {
        stall_active_ = false;
        return "疑似碰撞/卡死：输出指令 " + std::to_string(v_out).substr(0, 5) +
          "m/s 而底盘反馈≈0 持续 " +
          std::to_string(stall_timeout_).substr(0, 4) + "s";
      }
    } else {
      stall_active_ = false;
    }

    // b) goal 被 Nav2 判 ABORTED → 局部规划失效
    if (nav_status == action_msgs::msg::GoalStatus::STATUS_ABORTED) {
      return "局部规划失效：navigate_to_pose goal ABORTED";
    }

    // c) goal 活跃但控制器指令断流（需先收到过指令，规避启动期误判）
    const bool nav_active =
      nav_status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
      nav_status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
      nav_status == action_msgs::msg::GoalStatus::STATUS_CANCELING;
    if (nav_active && plan_cmd_received &&
      (now_t - last_plan_cmd).seconds() > plan_fail_timeout_)
    {
      return "局部规划失效：goal 活跃但 /cmd_vel_nav 断流 >" +
        std::to_string(plan_fail_timeout_).substr(0, 4) + "s";
    }
    return "";
  }

  // V0.0.86 测试模式异常中止：零速锁存 + /estop=true（latched，auto_mission
  // 收到即取消全部导航任务）+ 直接取消 bt_navigator 活动目标（双保险）。
  void abortTest(const std::string & reason)
  {
    {
      std::lock_guard<std::mutex> lk(data_mutex_);
      if (test_aborted_) {
        return;
      }
      test_aborted_ = true;
    }
    stall_active_ = false;
    RCLCPP_ERROR(get_logger(),
      "[测试模式中止] %s —— 零速锁存，需人工重新开启 /safety/test_mode",
      reason.c_str());
    if (estop_hold_on_abort_) {
      std_msgs::msg::Bool m;
      m.data = true;
      estop_pub_->publish(m);
      RCLCPP_ERROR(get_logger(),
        "[测试模式中止] 已发布 /estop=true（latched），auto_mission 将取消全部导航任务");
    }
    if (nav_client_ && nav_client_->action_server_is_ready()) {
      nav_client_->async_cancel_all_goals();
      RCLCPP_WARN(get_logger(), "[测试模式中止] 已请求取消 /navigate_to_pose 活动目标");
    }
  }

  // V0.0.86 测试模式开关回调：true→开启（同时清除中止锁存，视为人工确认
  // 现场安全）；false→关闭恢复常规阈值。中止锁存只能通过重新开启清除。
  void testModeCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lk(data_mutex_);
      if (msg->data && (!test_mode_ || test_aborted_)) {
        test_aborted_ = false;
      }
      test_mode_ = msg->data;
    }
    stall_active_ = false;
    if (msg->data) {
      RCLCPP_WARN(get_logger(),
        "[测试模式] 开启：限速 %.2fm/s、急停 %.1fm、减速 %.1fm（±%.0f° 扇区），异常自动中止",
        test_max_linear_vel_, test_stop_dist_, test_slow_dist_,
        sector_half_rad_ * 180.0 / M_PI);
    } else {
      RCLCPP_WARN(get_logger(), "[测试模式] 关闭：恢复常规安全阈值");
    }
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

  // V0.0.86 自动驾驶测试模式参数
  bool enable_test_mode_{false};
  bool test_mode_{false};
  double test_max_linear_vel_{0.1};
  double test_stop_dist_{1.0};
  double test_slow_dist_{2.0};
  double stall_timeout_{1.0};
  double stall_cmd_vel_min_{0.05};
  double stall_fb_vel_max_{0.02};
  double plan_fail_timeout_{2.0};
  bool estop_hold_on_abort_{true};

  // 状态
  State state_{State::CMD_TIMEOUT};
  rclcpp::Time last_state_pub_{0, 0, RCL_ROS_TIME};
  bool estop_{false};
  // V0.0.86 测试模式状态（test_aborted_/last_plan_cmd_time_/plan_cmd_received_/
  // last_nav_status_ 由 data_mutex_ 保护；stall_* 仅 tick 访问）
  bool test_aborted_{false};
  bool stall_active_{false};
  rclcpp::Time stall_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_plan_cmd_time_{0, 0, RCL_ROS_TIME};
  bool plan_cmd_received_{false};
  int8_t last_nav_status_{0};
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
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr test_mode_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr plan_cmd_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr nav_status_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_out_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
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
