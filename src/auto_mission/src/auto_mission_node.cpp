// Copyright 2026 HUNTER Development Team
// AUTO 模式自主任务调度节点实现
//
// 状态机转换：
//   IDLE ──[AUTO条件满足+建图模式]──▶ MAPPING
//   IDLE ──[AUTO条件满足+导航模式]──▶ WAITING_LOCALIZE
//   WAITING_LOCALIZE ──[定位收敛]──▶ NAVIGATING
//   WAITING_LOCALIZE ──[超时]──────▶ IDLE（告警停车）
//   NAVIGATING ──[障碍物近]────────▶ OBSTACLE_AVOID
//   NAVIGATING ──[goal成功]────────▶ NAVIGATING（下一航点）或 IDLE（完成）
//   NAVIGATING ──[goal失败]────────▶ NAVIGATING（重试/跳过）或 IDLE
//   OBSTACLE_AVOID ──[路清]────────▶ NAVIGATING（恢复）
//   OBSTACLE_AVOID ──[超时/极近]───▶ ESTOP
//   任意状态 ──[非AUTO/急停]────────▶ IDLE 或 ESTOP

#include "auto_mission/auto_mission_node.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace auto_mission
{

// ==========================================================================
// 构造函数
// ==========================================================================
AutoMissionNode::AutoMissionNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("auto_mission_node", options)
{
  declareParameters();
  loadWaypoints();

  // ---- 订阅 ----
  behavior_sub_ = create_subscription<hunter_msgs::msg::BehaviorState>(
    "/planning/behavior_state", rclcpp::QoS(10),
    std::bind(&AutoMissionNode::behaviorStateCallback, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/localization/odom", rclcpp::SensorDataQoS(),
    std::bind(&AutoMissionNode::odomCallback, this, std::placeholders::_1));

  fused_objects_sub_ = create_subscription<hunter_msgs::msg::DetectedObjectArray>(
    "/perception/fused_objects", rclcpp::SensorDataQoS(),
    std::bind(&AutoMissionNode::fusedObjectsCallback, this, std::placeholders::_1));

  health_sub_ = create_subscription<hunter_msgs::msg::SystemHealth>(
    "/system/health", rclcpp::QoS(10),
    std::bind(&AutoMissionNode::systemHealthCallback, this, std::placeholders::_1));

  estop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/estop", rclcpp::QoS(10).reliable(),
    std::bind(&AutoMissionNode::estopCallback, this, std::placeholders::_1));

  // ---- 发布 ----
  status_pub_ = create_publisher<std_msgs::msg::String>("/auto_mission/status", 10);
  waypoint_idx_pub_ = create_publisher<std_msgs::msg::Int32>("/auto_mission/current_waypoint", 10);
  estop_pub_ = create_publisher<std_msgs::msg::Bool>("/estop", rclcpp::QoS(10).reliable());

  // ---- Nav2 action 客户端 ----
  nav_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    this, "/navigate_to_pose");

  // ---- 初始化时间戳（防止启动时误判感知超时） ----
  last_perception_stamp_ = this->now();

  // ---- 主循环 10Hz ----
  main_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&AutoMissionNode::mainLoop, this));

  RCLCPP_INFO(get_logger(),
    "auto_mission_node 启动：模式=%s，航点数=%zu，循环=%s",
    mission_mode_.c_str(), waypoints_.size(), loop_waypoints_ ? "是" : "否");
}

// ==========================================================================
// 参数声明
// ==========================================================================
void AutoMissionNode::declareParameters()
{
  // 运行模式
  declare_parameter("mission_mode", "waypoint_loop");   // waypoint_loop | external

  // 安全距离
  declare_parameter("warn_obstacle_dist", 2.0);
  declare_parameter("stop_obstacle_dist", 0.8);
  declare_parameter("obstacle_fov_deg", 120.0);

  // 定位
  declare_parameter("localize_cov_threshold", 0.5);
  declare_parameter("localize_wait_timeout", 10.0);

  // 感知
  declare_parameter("perception_timeout", 2.0);

  // 巡航
  declare_parameter("loop_waypoints", true);
  declare_parameter("max_wp_failures", 3);
  declare_parameter("goal_timeout", 60.0);
  declare_parameter("obstacle_wait_timeout", 30.0);

  // 速度（合规性）
  declare_parameter("max_velocity", 2.0);

  // 航点（yaml 中以列表形式提供，每条格式："x,y,yaw,label"）
  declare_parameter("waypoints", std::vector<std::string>{});

  // 读取
  mission_mode_           = get_parameter("mission_mode").as_string();
  warn_obstacle_dist_     = get_parameter("warn_obstacle_dist").as_double();
  stop_obstacle_dist_     = get_parameter("stop_obstacle_dist").as_double();
  obstacle_fov_deg_       = get_parameter("obstacle_fov_deg").as_double();
  localize_cov_threshold_ = get_parameter("localize_cov_threshold").as_double();
  localize_wait_timeout_  = get_parameter("localize_wait_timeout").as_double();
  perception_timeout_     = get_parameter("perception_timeout").as_double();
  loop_waypoints_         = get_parameter("loop_waypoints").as_bool();
  max_wp_failures_        = static_cast<int>(get_parameter("max_wp_failures").as_int());
  goal_timeout_           = get_parameter("goal_timeout").as_double();
  obstacle_wait_timeout_  = get_parameter("obstacle_wait_timeout").as_double();
  max_velocity_           = get_parameter("max_velocity").as_double();

  // 最大速度合规检查（文档规定 ≤ 2.0 m/s）
  if (max_velocity_ > 2.0) {
    RCLCPP_WARN(get_logger(),
      "max_velocity=%.1f 超过文档限制 2.0 m/s，已强制限制为 2.0 m/s", max_velocity_);
    max_velocity_ = 2.0;
  }
}

// ==========================================================================
// 航点加载（从参数列表解析 "x,y,yaw,label"）
// ==========================================================================
void AutoMissionNode::loadWaypoints()
{
  const auto raw = get_parameter("waypoints").as_string_array();
  waypoints_.clear();
  for (const auto & entry : raw) {
    std::istringstream ss(entry);
    std::string token;
    std::vector<std::string> parts;
    while (std::getline(ss, token, ',')) {
      parts.push_back(token);
    }
    if (parts.size() < 3) {
      RCLCPP_WARN(get_logger(), "航点格式错误（需 x,y,yaw[,label]）：%s", entry.c_str());
      continue;
    }
    Waypoint wp;
    try {
      wp.x   = std::stod(parts[0]);
      wp.y   = std::stod(parts[1]);
      wp.yaw = std::stod(parts[2]);
      wp.label = (parts.size() >= 4) ? parts[3] : ("wp" + std::to_string(waypoints_.size()));
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "航点解析失败：%s → %s", entry.c_str(), e.what());
      continue;
    }
    waypoints_.push_back(wp);
  }
  RCLCPP_INFO(get_logger(), "共加载 %zu 个航点", waypoints_.size());
}

// ==========================================================================
// 订阅回调（只更新缓存）
// ==========================================================================
void AutoMissionNode::behaviorStateCallback(
  const hunter_msgs::msg::BehaviorState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(data_mutex_);
  latest_behavior_state_ = *msg;
}

void AutoMissionNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(data_mutex_);
  latest_odom_ = *msg;
}

void AutoMissionNode::fusedObjectsCallback(
  const hunter_msgs::msg::DetectedObjectArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(data_mutex_);
  latest_fused_objects_ = *msg;
  last_perception_stamp_ = this->now();
}

void AutoMissionNode::systemHealthCallback(
  const hunter_msgs::msg::SystemHealth::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(data_mutex_);
  latest_health_ = *msg;
}

void AutoMissionNode::estopCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(data_mutex_);
  estop_signal_ = msg->data;
}

// ==========================================================================
// 主循环（10Hz）
// ==========================================================================
void AutoMissionNode::mainLoop()
{
  // ---------- 获取快照（减少锁占用时间） ----------
  hunter_msgs::msg::BehaviorState behavior;
  nav_msgs::msg::Odometry odom;
  hunter_msgs::msg::SystemHealth health;
  bool estop_snap;
  {
    std::lock_guard<std::mutex> lk(data_mutex_);
    behavior = latest_behavior_state_;
    odom     = latest_odom_;
    health   = latest_health_;
    estop_snap = estop_signal_;
  }

  // ---------- 全局急停优先检查 ----------
  if (estop_snap && state_ != MissionState::ESTOP) {
    RCLCPP_ERROR(get_logger(), "[急停] 收到 /estop=true，立即终止所有导航任务");
    cancelCurrentGoal();
    state_ = MissionState::ESTOP;
    publishStatus();
    return;
  }

  // ---------- 非 AUTO 立即降级 ----------
  if (behavior.mode != "AUTO") {
    if (state_ != MissionState::IDLE && state_ != MissionState::ESTOP) {
      RCLCPP_WARN(get_logger(),
        "[降级] 模式由 %s 切换至 %s，取消导航任务",
        stateToString(state_).c_str(), behavior.mode.c_str());
      cancelCurrentGoal();
      state_ = MissionState::IDLE;
    }
    publishStatus();
    return;
  }

  // ---------- health 检查 ----------
  if (health.overall_status == "CRITICAL") {
    if (state_ != MissionState::IDLE && state_ != MissionState::ESTOP) {
      RCLCPP_ERROR(get_logger(),
        "[降级] SystemHealth=CRITICAL，停止导航，安全停车");
      cancelCurrentGoal();
      state_ = MissionState::IDLE;
    }
    publishStatus();
    return;
  }

  // ---------- 建图模式：AUTO+MAPPING 直接停在 MAPPING 状态 ----------
  if (mission_mode_ == "mapping") {
    if (state_ != MissionState::MAPPING) {
      state_ = MissionState::MAPPING;
      RCLCPP_INFO(get_logger(), "[建图] 进入建图模式，FAST-LIO2 在线建图中，不下发导航目标");
    }
    publishStatus();
    return;
  }

  // ---------- 以下为导航模式逻辑 ----------

  switch (state_) {
    // ------------------------------------------------------------------
    case MissionState::IDLE:
    // ------------------------------------------------------------------
    {
      if (!isAutoConditionMet()) {
        // 条件未满足，持续等待，每秒输出一次原因日志
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
          "[IDLE] AUTO 条件未满足，等待中（模式=%s，health=%s，定位收敛=%s，感知存活=%s）",
          behavior.mode.c_str(),
          health.overall_status.c_str(),
          isLocalizationValid() ? "是" : "否",
          isPerceptionAlive() ? "是" : "否");
        break;
      }
      if (waypoints_.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "[IDLE] 无可用航点，请在 autonomous_nav_params.yaml 中配置 waypoints");
        break;
      }
      // 条件满足，先检查定位收敛
      if (!isLocalizationValid()) {
        RCLCPP_INFO(get_logger(), "[IDLE→WAITING_LOCALIZE] AUTO 条件满足，等待定位收敛");
        localize_wait_started_ = true;
        localize_wait_start_ = this->now();
        state_ = MissionState::WAITING_LOCALIZE;
      } else {
        RCLCPP_INFO(get_logger(), "[IDLE→NAVIGATING] AUTO 条件满足，定位已收敛，开始导航");
        current_wp_idx_ = 0;
        wp_fail_count_ = 0;
        state_ = MissionState::NAVIGATING;
        sendNextWaypoint();
      }
      break;
    }

    // ------------------------------------------------------------------
    case MissionState::WAITING_LOCALIZE:
    // ------------------------------------------------------------------
    {
      if (!isAutoConditionMet()) {
        RCLCPP_WARN(get_logger(), "[WAITING_LOCALIZE] AUTO 条件丢失，回到 IDLE");
        state_ = MissionState::IDLE;
        localize_wait_started_ = false;
        break;
      }
      if (isLocalizationValid()) {
        RCLCPP_INFO(get_logger(), "[WAITING_LOCALIZE→NAVIGATING] 定位已收敛，开始导航");
        state_ = MissionState::NAVIGATING;
        current_wp_idx_ = 0;
        wp_fail_count_ = 0;
        localize_wait_started_ = false;
        sendNextWaypoint();
        break;
      }
      // 超时检查
      if (localize_wait_started_ &&
          (this->now() - localize_wait_start_).seconds() > localize_wait_timeout_)
      {
        RCLCPP_ERROR(get_logger(),
          "[WAITING_LOCALIZE] 等待定位收敛超时（%.0fs），安全停车，回到 IDLE",
          localize_wait_timeout_);
        localize_wait_started_ = false;
        state_ = MissionState::IDLE;
      }
      break;
    }

    // ------------------------------------------------------------------
    case MissionState::NAVIGATING:
    // ------------------------------------------------------------------
    {
      // AUTO 条件变化
      if (!isAutoConditionMet()) {
        RCLCPP_WARN(get_logger(), "[NAVIGATING] AUTO 条件丢失，取消导航，回到 IDLE");
        cancelCurrentGoal();
        state_ = MissionState::IDLE;
        break;
      }

      // goal 超时检查
      if (goal_in_flight_) {
        const double elapsed = (this->now() - goal_send_time_).seconds();
        if (elapsed > goal_timeout_) {
          RCLCPP_WARN(get_logger(),
            "[NAVIGATING] 航点[%zu]%s 导航超时（%.0fs），跳过该航点",
            current_wp_idx_, waypoints_[current_wp_idx_].label.c_str(), goal_timeout_);
          cancelCurrentGoal();
          wp_fail_count_++;
          if (wp_fail_count_ >= max_wp_failures_) {
            RCLCPP_ERROR(get_logger(),
              "[NAVIGATING] 连续失败 %d 次，停止巡航，进入 IDLE", max_wp_failures_);
            state_ = MissionState::IDLE;
          } else {
            current_wp_idx_ = (current_wp_idx_ + 1) % waypoints_.size();
            sendNextWaypoint();
          }
          break;
        }
      }

      // 障碍物检查
      const double obs_dist = nearestObstacleDist();
      if (obs_dist < stop_obstacle_dist_) {
        RCLCPP_ERROR(get_logger(),
          "[NAVIGATING→ESTOP] 障碍物距离 %.2fm < 急停阈值 %.2fm，触发 ESTOP",
          obs_dist, stop_obstacle_dist_);
        triggerEstop("障碍物过近");
        break;
      }
      if (obs_dist < warn_obstacle_dist_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "[NAVIGATING→OBSTACLE_AVOID] 障碍物距离 %.2fm < 警告阈值 %.2fm，暂停导航等待",
          obs_dist, warn_obstacle_dist_);
        // 暂不取消 goal，只切换状态等待障碍物离开
        // Nav2 的 RPP 控制器会自然减速；此状态下不主动取消
        obstacle_wait_start_ = this->now();
        obstacle_wait_started_ = true;
        state_ = MissionState::OBSTACLE_AVOID;
        break;
      }

      // 正常导航中，只在没有 goal 在飞时重新发送（防止重复发送）
      if (!goal_in_flight_) {
        RCLCPP_INFO(get_logger(), "[NAVIGATING] 无在途 goal，重新发送航点[%zu]",
          current_wp_idx_);
        sendNextWaypoint();
      }
      break;
    }

    // ------------------------------------------------------------------
    case MissionState::OBSTACLE_AVOID:
    // ------------------------------------------------------------------
    {
      if (!isAutoConditionMet()) {
        RCLCPP_WARN(get_logger(), "[OBSTACLE_AVOID] AUTO 条件丢失，回到 IDLE");
        cancelCurrentGoal();
        state_ = MissionState::IDLE;
        obstacle_wait_started_ = false;
        break;
      }

      const double obs_dist = nearestObstacleDist();

      // 障碍物极近 → 急停
      if (obs_dist < stop_obstacle_dist_) {
        RCLCPP_ERROR(get_logger(),
          "[OBSTACLE_AVOID→ESTOP] 障碍物距离 %.2fm < 急停阈值，触发 ESTOP", obs_dist);
        triggerEstop("障碍物避让期间过近");
        break;
      }

      // 等待超时 → 急停兜底
      if (obstacle_wait_started_ &&
          (this->now() - obstacle_wait_start_).seconds() > obstacle_wait_timeout_)
      {
        RCLCPP_ERROR(get_logger(),
          "[OBSTACLE_AVOID] 等待障碍物超时（%.0fs），触发 ESTOP",
          obstacle_wait_timeout_);
        triggerEstop("障碍物长时间未清除");
        break;
      }

      // 障碍物清除 → 恢复导航
      if (obs_dist >= warn_obstacle_dist_) {
        RCLCPP_INFO(get_logger(),
          "[OBSTACLE_AVOID→NAVIGATING] 障碍物已清除（%.2fm），恢复导航", obs_dist);
        obstacle_wait_started_ = false;
        state_ = MissionState::NAVIGATING;
        // goal_in_flight_ 若仍为 true，Nav2 会继续执行，无需重新发送
        if (!goal_in_flight_) {
          sendNextWaypoint();
        }
      }
      break;
    }

    // ------------------------------------------------------------------
    case MissionState::ESTOP:
    // ------------------------------------------------------------------
    {
      // 急停状态下等待外部清除：/estop 变 false 且 AUTO 条件恢复后方可退出
      if (!estop_snap && isAutoConditionMet()) {
        RCLCPP_INFO(get_logger(), "[ESTOP→IDLE] 急停信号解除且 AUTO 条件满足，恢复 IDLE");
        state_ = MissionState::IDLE;
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "[ESTOP] 保持急停状态（estop=%s，AUTO条件=%s）",
          estop_snap ? "true" : "false",
          isAutoConditionMet() ? "满足" : "未满足");
      }
      break;
    }

    // ------------------------------------------------------------------
    case MissionState::MAPPING:
    // ------------------------------------------------------------------
    {
      // 建图模式已在上方处理，此分支不会到达
      break;
    }
  }

  publishStatus();
}

// ==========================================================================
// AUTO 进入条件检查（全部满足才返回 true）
// ==========================================================================
bool AutoMissionNode::isAutoConditionMet()
{
  std::lock_guard<std::mutex> lk(data_mutex_);
  // 条件1：decision_making 输出 AUTO
  if (latest_behavior_state_.mode != "AUTO") {
    return false;
  }
  // 条件2：无急停
  if (estop_signal_) {
    return false;
  }
  // 条件3：health 非 CRITICAL
  if (latest_health_.overall_status == "CRITICAL") {
    return false;
  }
  // 条件4：定位协方差收敛（由 isLocalizationValid 单独调用，此处复用）
  // 条件5：感知数据新鲜（由 isPerceptionAlive 单独调用，此处复用）
  // 注意：isLocalizationValid / isPerceptionAlive 内部也访问数据，
  //       因为已持有 mutex，避免重复加锁，在此直接内联判断。
  // --- 定位协方差（inline） ---
  const auto & cov = latest_odom_.pose.covariance;
  // 协方差矩阵对角元素 [0,7,35] 对应 x,y,yaw
  const double cov_trace = cov[0] + cov[7] + cov[35];
  if (cov_trace > localize_cov_threshold_) {
    return false;
  }
  // --- 感知新鲜度（inline） ---
  const double perception_age = (this->now() - last_perception_stamp_).seconds();
  if (perception_age > perception_timeout_) {
    return false;
  }
  return true;
}

// ==========================================================================
// 定位有效性（协方差迹 < 阈值）
// ==========================================================================
bool AutoMissionNode::isLocalizationValid()
{
  std::lock_guard<std::mutex> lk(data_mutex_);
  const auto & cov = latest_odom_.pose.covariance;
  const double cov_trace = cov[0] + cov[7] + cov[35];
  return cov_trace <= localize_cov_threshold_;
}

// ==========================================================================
// 感知数据新鲜度
// ==========================================================================
bool AutoMissionNode::isPerceptionAlive()
{
  // last_perception_stamp_ 由 mutex 保护，此处直接读取（rclcpp::Time 是 trivially copyable）
  const double age = (this->now() - last_perception_stamp_).seconds();
  return age <= perception_timeout_;
}

// ==========================================================================
// 最近障碍物距离（前向扇区内，base_link 坐标系）
// ==========================================================================
double AutoMissionNode::nearestObstacleDist()
{
  std::lock_guard<std::mutex> lk(data_mutex_);
  const double half_fov_rad = (obstacle_fov_deg_ / 2.0) * M_PI / 180.0;
  double min_dist = std::numeric_limits<double>::max();

  for (const auto & obj : latest_fused_objects_.objects) {
    // base_link 坐标系：x 向前，y 向左
    const double dx = obj.pose.position.x;
    const double dy = obj.pose.position.y;
    const double dist = std::sqrt(dx * dx + dy * dy);
    const double angle = std::atan2(dy, dx);   // [-π, π]

    // 只考虑前向扇区内的障碍物
    if (std::abs(angle) <= half_fov_rad) {
      min_dist = std::min(min_dist, dist);
    }
  }
  return min_dist;
}

// ==========================================================================
// 向 Nav2 发送下一个航点
// ==========================================================================
void AutoMissionNode::sendNextWaypoint()
{
  if (waypoints_.empty()) {
    RCLCPP_WARN(get_logger(), "[sendNextWaypoint] 航点列表为空，无法发送");
    state_ = MissionState::IDLE;
    return;
  }

  if (!nav_action_client_->wait_for_action_server(std::chrono::seconds(3))) {
    RCLCPP_ERROR(get_logger(),
      "[sendNextWaypoint] Nav2 /navigate_to_pose action server 不可用，回到 IDLE");
    state_ = MissionState::IDLE;
    return;
  }

  const Waypoint & wp = waypoints_[current_wp_idx_];

  // 构造目标消息（map 帧）
  nav2_msgs::action::NavigateToPose::Goal goal_msg;
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.header.stamp    = this->now();
  goal_msg.pose.pose.position.x = wp.x;
  goal_msg.pose.pose.position.y = wp.y;
  goal_msg.pose.pose.position.z = 0.0;
  // yaw → 四元数（绕 Z 轴）
  goal_msg.pose.pose.orientation.z = std::sin(wp.yaw / 2.0);
  goal_msg.pose.pose.orientation.w = std::cos(wp.yaw / 2.0);
  goal_msg.behavior_tree = "";   // 使用 bt_navigator 默认行为树

  auto send_opts = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions{};
  send_opts.goal_response_callback =
    std::bind(&AutoMissionNode::goalResponseCallback, this, std::placeholders::_1);
  send_opts.feedback_callback =
    std::bind(&AutoMissionNode::feedbackCallback, this,
      std::placeholders::_1, std::placeholders::_2);
  send_opts.result_callback =
    std::bind(&AutoMissionNode::resultCallback, this, std::placeholders::_1);

  nav_action_client_->async_send_goal(goal_msg, send_opts);
  goal_send_time_ = this->now();
  goal_in_flight_ = true;

  RCLCPP_INFO(get_logger(),
    "[sendNextWaypoint] 发送航点[%zu] %s → (%.2f, %.2f, yaw=%.2f rad)",
    current_wp_idx_, wp.label.c_str(), wp.x, wp.y, wp.yaw);

  // 发布当前航点索引
  std_msgs::msg::Int32 idx_msg;
  idx_msg.data = static_cast<int32_t>(current_wp_idx_);
  waypoint_idx_pub_->publish(idx_msg);
}

// ==========================================================================
// 取消当前 goal
// ==========================================================================
void AutoMissionNode::cancelCurrentGoal()
{
  std::lock_guard<std::mutex> lk(goal_handle_mutex_);
  if (goal_handle_ && goal_in_flight_) {
    RCLCPP_INFO(get_logger(), "[cancelCurrentGoal] 取消当前导航 goal");
    nav_action_client_->async_cancel_goal(goal_handle_);
  }
  goal_handle_ = nullptr;
  goal_in_flight_ = false;
}

// ==========================================================================
// 触发急停
// ==========================================================================
void AutoMissionNode::triggerEstop(const std::string & reason)
{
  cancelCurrentGoal();
  state_ = MissionState::ESTOP;

  std_msgs::msg::Bool estop_msg;
  estop_msg.data = true;
  estop_pub_->publish(estop_msg);

  RCLCPP_ERROR(get_logger(), "[ESTOP] 触发急停：%s", reason.c_str());
}

// ==========================================================================
// Nav2 goal response 回调
// ==========================================================================
void AutoMissionNode::goalResponseCallback(
  const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & handle)
{
  std::lock_guard<std::mutex> lk(goal_handle_mutex_);
  if (!handle) {
    RCLCPP_ERROR(get_logger(), "[Nav2] goal 被服务端拒绝");
    goal_in_flight_ = false;
    wp_fail_count_++;
  } else {
    goal_handle_ = handle;
    RCLCPP_INFO(get_logger(), "[Nav2] goal 已被接受，开始导航至航点[%zu]",
      current_wp_idx_);
  }
}

// ==========================================================================
// Nav2 feedback 回调（仅做日志）
// ==========================================================================
void AutoMissionNode::feedbackCallback(
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
  const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
  RCLCPP_DEBUG(get_logger(),
    "[Nav2] 剩余距离: %.2f m，当前速度: %.2f m/s",
    feedback->distance_remaining,
    0.0);  // NavigateToPose feedback 只包含 distance_remaining
  (void)feedback;
}

// ==========================================================================
// Nav2 result 回调
// ==========================================================================
void AutoMissionNode::resultCallback(
  const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result)
{
  goal_in_flight_ = false;
  {
    std::lock_guard<std::mutex> lk(goal_handle_mutex_);
    goal_handle_ = nullptr;
  }

  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(get_logger(),
      "[Nav2] 航点[%zu] %s 导航成功",
      current_wp_idx_, waypoints_[current_wp_idx_].label.c_str());
    wp_fail_count_ = 0;  // 成功则重置失败计数

    // 判断是否还有下一个航点
    const size_t next_idx = current_wp_idx_ + 1;
    if (next_idx < waypoints_.size()) {
      current_wp_idx_ = next_idx;
      if (state_ == MissionState::NAVIGATING) {
        sendNextWaypoint();
      }
    } else {
      // 全部航点完成
      if (loop_waypoints_) {
        RCLCPP_INFO(get_logger(), "[Nav2] 所有航点完成，循环重新开始");
        current_wp_idx_ = 0;
        if (state_ == MissionState::NAVIGATING) {
          sendNextWaypoint();
        }
      } else {
        RCLCPP_INFO(get_logger(), "[Nav2] 所有航点完成，停止巡航（loop_waypoints=false）");
        state_ = MissionState::IDLE;
      }
    }
  } else {
    const char * reason =
      (result.code == rclcpp_action::ResultCode::ABORTED)   ? "ABORTED" :
      (result.code == rclcpp_action::ResultCode::CANCELED)  ? "CANCELED" : "UNKNOWN";
    RCLCPP_WARN(get_logger(),
      "[Nav2] 航点[%zu] %s 导航失败（%s），fail_count=%d/%d",
      current_wp_idx_, waypoints_[current_wp_idx_].label.c_str(),
      reason, wp_fail_count_ + 1, max_wp_failures_);
    wp_fail_count_++;

    if (wp_fail_count_ >= max_wp_failures_) {
      RCLCPP_ERROR(get_logger(),
        "[Nav2] 连续失败 %d 次达到上限，停止巡航，进入 IDLE", max_wp_failures_);
      state_ = MissionState::IDLE;
      return;
    }
    // 跳过当前航点，继续下一个
    if (state_ == MissionState::NAVIGATING) {
      current_wp_idx_ = (current_wp_idx_ + 1) % waypoints_.size();
      sendNextWaypoint();
    }
  }
}

// ==========================================================================
// 发布当前任务状态
// ==========================================================================
void AutoMissionNode::publishStatus()
{
  if (state_ != prev_state_) {
    RCLCPP_INFO(get_logger(), "[状态] %s → %s",
      stateToString(prev_state_).c_str(), stateToString(state_).c_str());
    prev_state_ = state_;
  }

  std_msgs::msg::String msg;
  msg.data = stateToString(state_);
  status_pub_->publish(msg);
}

// ==========================================================================
// 状态枚举转字符串
// ==========================================================================
std::string AutoMissionNode::stateToString(MissionState s)
{
  switch (s) {
    case MissionState::IDLE:            return "IDLE";
    case MissionState::MAPPING:         return "MAPPING";
    case MissionState::WAITING_LOCALIZE:return "WAITING_LOCALIZE";
    case MissionState::NAVIGATING:      return "NAVIGATING";
    case MissionState::OBSTACLE_AVOID:  return "OBSTACLE_AVOID";
    case MissionState::ESTOP:           return "ESTOP";
    default:                            return "UNKNOWN";
  }
}

}  // namespace auto_mission
