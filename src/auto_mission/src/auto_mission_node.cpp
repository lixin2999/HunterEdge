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
//   NAVIGATING ──[连续失败达上限]──▶ FAULT（V0.0.91：锁存停驻，不再静默重发）
//   FAULT ──[模式离开 AUTO]────────▶ IDLE（人工确认后重新进入）
//   任意状态 ──[非AUTO/急停]────────▶ IDLE 或 ESTOP

#include "auto_mission/auto_mission_node.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
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

  // 静态地图（map_server 激活时以 transient_local 发布一次，V0.0.82）：
  // 必须 transient_local + reliable 订阅，volatile 订阅会因晚于发布而漏收地图。
  // 缓存地图边界用于航点越界校验，防止 "Goal pose is out of costmap!" 死局。
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&AutoMissionNode::mapCallback, this, std::placeholders::_1));

  // V0.0.93 方案A：订阅全局重定位位姿 /relocalization/pose（原 /amcl_pose）。
  // hunter_relocalization(NDT) 以 2Hz 持续发布 map 系位姿：收敛时协方差小
  // （converged_covariance，默认 0.01），未收敛时发布大协方差（100）。
  // 用于 isLocalizationValid() 判断 map→base_link 全局定位可信度，
  // 避免定位尚在收敛时提前导航导致偏航。（成员名沿用 amcl_* 前缀，语义为重定位。）
  amcl_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/relocalization/pose", rclcpp::QoS(rclcpp::KeepLast(5)).transient_local().reliable(),
    std::bind(&AutoMissionNode::amclPoseCallback, this, std::placeholders::_1));

  // ---- 发布 ----
  status_pub_ = create_publisher<std_msgs::msg::String>("/auto_mission/status", 10);
  waypoint_idx_pub_ = create_publisher<std_msgs::msg::Int32>("/auto_mission/current_waypoint", 10);
  estop_pub_ = create_publisher<std_msgs::msg::Bool>("/estop", rclcpp::QoS(10).reliable());

  // ---- Nav2 action 客户端 ----
  nav_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    this, "/navigate_to_pose");

  // ---- bt_navigator lifecycle 状态查询 ----
  // bt_navigator 的 action server 在 configure 阶段即被发现，但 inactive 状态会拒绝
  // goal（日志"goal 被服务端拒绝"），发送前必须经 GetState 确认节点已 ACTIVE。
  nav_state_client_ = create_client<lifecycle_msgs::srv::GetState>(
    "/bt_navigator/get_state");

  // ---- 代价地图清除服务（V0.0.91）----
  // 遥控接管后重新自主的标配动作：接管期车辆会压过此前被标记的栅格，
  // 叠加定位跳变在地图上留下的假障碍，不清一次就大概率“起点在致命栅格”。
  clear_global_costmap_srv_ = create_client<nav2_msgs::srv::ClearEntireCostmap>(
    "/global_costmap/clear_entirely_global_costmap");
  clear_local_costmap_srv_ = create_client<nav2_msgs::srv::ClearEntireCostmap>(
    "/local_costmap/clear_entirely_local_costmap");

  // ---- 建图模式自动巡航设施 ----
  // /cmd_vel：mapping 模式下 Nav2 全栈未启动，无竞争发布者；
  // hunter_base 订阅 /cmd_vel 并按 bicycle model 换算阿克曼转向角。
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // FAST-LIO2 里程计（odom 系，原 camera_init；V0.0.93 方案A 帧名已归一，10Hz）——与 rviz "Publish Point"
  // 点击航点严格同源，巡航反馈直接使用该位姿，避免 EKF 系折算偏差。
  lio_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/Odometry", rclcpp::SensorDataQoS(),
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(data_mutex_);
      latest_lio_odom_ = *msg;
      last_lio_odom_arrive_ = this->now();
    });

  start_cruise_srv_ = create_service<std_srvs::srv::Trigger>(
    "/auto_mission/start_mapping_cruise",
    std::bind(&AutoMissionNode::startCruiseCallback, this,
      std::placeholders::_1, std::placeholders::_2));
  stop_cruise_srv_ = create_service<std_srvs::srv::Trigger>(
    "/auto_mission/stop_mapping_cruise",
    std::bind(&AutoMissionNode::stopCruiseCallback, this,
      std::placeholders::_1, std::placeholders::_2));

  cruise_timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int64_t>(1000.0 / std::max(1.0, cruise_cmd_rate_))),
    std::bind(&AutoMissionNode::cruiseControlStep, this));

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
  declare_parameter("amcl_cov_threshold", 0.60);   // V0.0.92：AMCL x+y 方差和收敛阈值
  declare_parameter("localize_wait_timeout", 15.0); // V0.0.92：等待超时 10s→15s（AMCL 需要更多扫描收敛）

  // 感知
  declare_parameter("perception_timeout", 2.0);

  // 巡航
  declare_parameter("loop_waypoints", true);
  declare_parameter("max_wp_failures", 3);
  declare_parameter("goal_timeout", 60.0);
  declare_parameter("obstacle_wait_timeout", 30.0);
  // V0.0.95 航点“已到达”预检与受阻检测（修“原地不动/无法绕行”）
  // V0.0.97：0.30 → 0.35m。现场（V0.0.96 日志 1790055967）：车与"起点"航点相距
  //   0.304m，刚过 0.30 阈值 ⇒ 预检不判“已到达”，下发该退化目标后又因阿克曼
  //   无法在这个距离上做微调而被阻 25s 才放弃。0.35m 与 Nav2 的 xy_goal_tolerance
  //   0.10m + 车辆最小转弯半径 1.9m 的"可微调圆"半径匹配，避免这类伪任务。
  //   ⚠ 上限不宜超过 0.4m：巡逻语义下会跳过本应前往的近邻航点。
  declare_parameter("already_reached_dist", 0.35);
  declare_parameter("stall_detect_time", 25.0);
  declare_parameter("stall_move_eps", 0.15);
  // V0.0.97 绕障机动里程上限（m）：累计行程 ≥ 本值即认定"车辆确实在多段机动
  //   （多点掉头/受限倒车绕障）"，不再按受阻换点，兜底交给单航点超时 goal_timeout。
  //   取 1.0m ≈ 一次"倒 0.4m + 进 0.5m"机动的总里程；若现场绕障动辄超 1.0m
  //   仍无净推进，说明阈值/场地几何不匹配，应先查 stop_dist/slow_dist 而非调大本值。
  declare_parameter("stall_path_allow_m", 1.0);
  // V0.0.96 航点净空校验（修“开到静态障碍膨胀区后所有规划全失败”）
  declare_parameter("waypoint_clearance_m", 0.50);

  // Nav2 就绪门控
  declare_parameter("nav_active_wait_timeout", 60.0);
  declare_parameter("nav_retry_backoff", 2.0);
  // V0.0.98 取消静置时长：主动取消后等旧 goal 结果回传的最长时间
  declare_parameter("goal_cancel_settle_time", 2.0);

  // 速度（合规性）
  declare_parameter("max_velocity", 2.0);

  // 航点（yaml 中以列表形式提供，每条格式："x,y,yaw,label"）
  declare_parameter("waypoints", std::vector<std::string>{});

  // ---- 建图模式自动巡航 ----
  // params_file：autonomous_nav_params.yaml 路径，start_mapping_cruise 服务
  // 触发时从此文件重新读取 waypoints（waypoint_recorder 边建图边写入）
  declare_parameter("params_file", "");
  declare_parameter("cruise_max_speed", 1.0);
  declare_parameter("cruise_turn_speed", 0.4);
  declare_parameter("cruise_min_speed", 0.2);
  declare_parameter("cruise_reach_dist", 0.6);
  declare_parameter("cruise_brake_dist", 2.0);
  declare_parameter("cruise_kp_yaw", 1.2);
  declare_parameter("cruise_max_yaw_rate", 0.5);
  declare_parameter("cruise_yaw_slow_deg", 45.0);
  declare_parameter("cruise_min_turn_radius", 1.9);
  declare_parameter("cruise_cmd_rate", 20.0);
  declare_parameter("cruise_odom_timeout", 1.0);

  // 读取
  mission_mode_           = get_parameter("mission_mode").as_string();
  warn_obstacle_dist_     = get_parameter("warn_obstacle_dist").as_double();
  stop_obstacle_dist_     = get_parameter("stop_obstacle_dist").as_double();
  obstacle_fov_deg_       = get_parameter("obstacle_fov_deg").as_double();
  localize_cov_threshold_ = get_parameter("localize_cov_threshold").as_double();
  amcl_cov_threshold_     = get_parameter("amcl_cov_threshold").as_double();
  localize_wait_timeout_  = get_parameter("localize_wait_timeout").as_double();
  perception_timeout_     = get_parameter("perception_timeout").as_double();
  loop_waypoints_         = get_parameter("loop_waypoints").as_bool();
  max_wp_failures_        = static_cast<int>(get_parameter("max_wp_failures").as_int());
  goal_timeout_           = get_parameter("goal_timeout").as_double();
  obstacle_wait_timeout_  = get_parameter("obstacle_wait_timeout").as_double();
  // V0.0.95 参数读取与合法性校验
  already_reached_dist_   = get_parameter("already_reached_dist").as_double();
  stall_detect_time_      = get_parameter("stall_detect_time").as_double();
  stall_move_eps_         = get_parameter("stall_move_eps").as_double();
  stall_path_allow_m_     = get_parameter("stall_path_allow_m").as_double();
  if (stall_path_allow_m_ < stall_move_eps_) {
    RCLCPP_WARN(get_logger(),
      "stall_path_allow_m=%.2fm 小于 stall_move_eps=%.2fm，已按 %.2fm 处理"
      "（必须大于单次机动净推进门槛，否则正常机动会被误判受阻）",
      stall_path_allow_m_, stall_move_eps_, stall_move_eps_ * 2.0);
    stall_path_allow_m_ = stall_move_eps_ * 2.0;
  }
  if (already_reached_dist_ <= 0.0) {
    RCLCPP_WARN(get_logger(),
      "already_reached_dist=%.2f ≤ 0，已按 0.35m 处理（禁用“已到达”预检会重现"
      "“目标=当前位姿”死锁）", already_reached_dist_);
    already_reached_dist_ = 0.35;
  }
  if (stall_detect_time_ < 5.0) {
    RCLCPP_WARN(get_logger(),
      "stall_detect_time=%.1fs 过小（< 5s），已按 5.0s 处理（须大于 Nav2 "
      "ProgressChecker 的 movement_time_allowance=10s 之前的首轮恢复窗口）",
      stall_detect_time_);
    stall_detect_time_ = 5.0;
  }
  if (stall_move_eps_ < 0.0) {
    stall_move_eps_ = 0.0;
  }
  // V0.0.96 航点净空校验半径：必须 > Nav2 内切半径（默认 footprint 半宽 0.32m），
  // 否则航点仍会落在膨胀/致命栅格内 → Smac 抛 "Starting point in lethal space"
  waypoint_clearance_m_ = get_parameter("waypoint_clearance_m").as_double();
  if (waypoint_clearance_m_ < 0.35) {
    RCLCPP_WARN(get_logger(),
      "waypoint_clearance_m=%.2f 过小（≤ Nav2 内切半径 0.32m + 余量），已按 0.50m 处理"
      "（过小会让航点落在静态障碍膨胀区内，规划必报 Starting point in lethal space）",
      waypoint_clearance_m_);
    waypoint_clearance_m_ = 0.50;
  }
  max_velocity_           = get_parameter("max_velocity").as_double();
  params_file_            = get_parameter("params_file").as_string();
  cruise_max_speed_       = get_parameter("cruise_max_speed").as_double();
  cruise_turn_speed_      = get_parameter("cruise_turn_speed").as_double();
  cruise_min_speed_       = get_parameter("cruise_min_speed").as_double();
  cruise_reach_dist_      = get_parameter("cruise_reach_dist").as_double();
  cruise_brake_dist_      = get_parameter("cruise_brake_dist").as_double();
  cruise_kp_yaw_          = get_parameter("cruise_kp_yaw").as_double();
  cruise_max_yaw_rate_    = get_parameter("cruise_max_yaw_rate").as_double();
  cruise_yaw_slow_deg_    = get_parameter("cruise_yaw_slow_deg").as_double();
  cruise_min_turn_radius_ = get_parameter("cruise_min_turn_radius").as_double();
  cruise_cmd_rate_        = get_parameter("cruise_cmd_rate").as_double();
  cruise_odom_timeout_    = get_parameter("cruise_odom_timeout").as_double();
  nav_active_wait_timeout_ = get_parameter("nav_active_wait_timeout").as_double();
  nav_retry_backoff_       = get_parameter("nav_retry_backoff").as_double();
  goal_cancel_settle_time_ = get_parameter("goal_cancel_settle_time").as_double();
  if (goal_cancel_settle_time_ < 0.5) {
    RCLCPP_WARN(get_logger(),
      "goal_cancel_settle_time=%.2f 过小（<0.5s 等于回到取消竞态窗口），按 2.0s 处理",
      goal_cancel_settle_time_);
    goal_cancel_settle_time_ = 2.0;
  }

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
    Waypoint wp;
    if (!parseWaypoint(entry, wp)) {
      continue;
    }
    waypoints_.push_back(wp);
  }
  RCLCPP_INFO(get_logger(), "共加载 %zu 个航点", waypoints_.size());
}

// ==========================================================================
// 单条航点解析："x,y,yaw[,label]"，label 可含中文（不含逗号）
// ==========================================================================
bool AutoMissionNode::parseWaypoint(const std::string & entry, Waypoint & wp)
{
  std::istringstream ss(entry);
  std::string token;
  std::vector<std::string> parts;
  while (std::getline(ss, token, ',')) {
    parts.push_back(token);
  }
  if (parts.size() < 3) {
    RCLCPP_WARN(get_logger(), "航点格式错误（需 x,y,yaw[,label]）：%s", entry.c_str());
    return false;
  }
  try {
    wp.x   = std::stod(parts[0]);
    wp.y   = std::stod(parts[1]);
    wp.yaw = std::stod(parts[2]);
    wp.label = (parts.size() >= 4) ? parts[3] : "wp";
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "航点解析失败：%s → %s", entry.c_str(), e.what());
    return false;
  }
  return true;
}

// ==========================================================================
// 从 params_file_ 重新读取 waypoints（热重载）
//
// 使用行扫描状态机而非引入 yaml-cpp 依赖：waypoint_recorder 保存的文件
// 结构固定为 auto_mission_node → ros__parameters → waypoints 下的 "- item"
// 列表（yaml block 风格），扫描 'waypoints:' 关键字后的连续 "- " 行即可。
// ==========================================================================
bool AutoMissionNode::reloadWaypointsFromFile(std::string & msg)
{
  if (params_file_.empty()) {
    msg = "params_file 参数为空，无法热重载航点（launch 需传入 params_file）";
    RCLCPP_ERROR(get_logger(), "[巡航] %s", msg.c_str());
    return false;
  }
  std::ifstream in(params_file_);
  if (!in.is_open()) {
    msg = "无法打开航点文件：" + params_file_;
    RCLCPP_ERROR(get_logger(), "[巡航] %s", msg.c_str());
    return false;
  }

  std::vector<std::string> entries;
  bool in_waypoints = false;
  std::string line;
  while (std::getline(in, line)) {
    // 去注释（行首 # 或空格后的 #）
    const auto hash_pos = line.find('#');
    if (hash_pos != std::string::npos &&
        line.find_first_not_of(" \t") == hash_pos)
    {
      line = line.substr(0, hash_pos);
    }
    const auto content = line.substr(line.find_first_not_of(" \t") == std::string::npos ?
                                     std::string::npos : line.find_first_not_of(" \t"));
    if (content.empty()) {
      continue;
    }
    if (in_waypoints) {
      if (content.rfind("- ", 0) == 0) {
        std::string item = content.substr(2);
        // 去除可能的 yaml 引号（含特殊字符时 yaml 会加引号）
        if (item.size() >= 2 &&
            ((item.front() == '\'' && item.back() == '\'') ||
             (item.front() == '\"' && item.back() == '\"')))
        {
          item = item.substr(1, item.size() - 2);
        }
        if (!item.empty()) {
          entries.push_back(item);
        }
      } else {
        // 列表结束（出现新的 key 或 dedent 到列表层级之外）
        in_waypoints = false;
      }
    } else if (content.rfind("waypoints:", 0) == 0) {
      in_waypoints = true;
    }
  }
  in.close();

  if (entries.empty()) {
    msg = "航点文件中未找到有效 waypoints：" + params_file_ +
          "（请先用 rviz2 Publish Point 点击航点）";
    RCLCPP_ERROR(get_logger(), "[巡航] %s", msg.c_str());
    return false;
  }

  std::vector<Waypoint> parsed;
  for (const auto & e : entries) {
    Waypoint wp;
    if (parseWaypoint(e, wp)) {
      parsed.push_back(wp);
    }
  }
  if (parsed.empty()) {
    msg = "waypoints 条目全部解析失败（共 " + std::to_string(entries.size()) + " 条）";
    RCLCPP_ERROR(get_logger(), "[巡航] %s", msg.c_str());
    return false;
  }

  waypoints_ = parsed;
  std::ostringstream oss;
  oss << "热重载成功，共 " << waypoints_.size() << " 个航点：";
  for (size_t i = 0; i < waypoints_.size(); ++i) {
    oss << "\n  [" << i << "] " << waypoints_[i].label
        << " (" << waypoints_[i].x << ", " << waypoints_[i].y << ")";
  }
  msg = oss.str();
  RCLCPP_INFO(get_logger(), "[巡航] %s", msg.c_str());
  return true;
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
  if (!msg->data) {
    // 急停解除（如 health_monitor CAN 恢复后发布 /estop=false）
    estop_self_triggered_.store(false);
  }
  // data==true 时不清 estop_self_triggered_：本节点自触发急停时也会收到
  // 自己发布的 /estop=true，误清理会丢失"自触发"标记导致无法自动解除
}

// ==========================================================================
// 建图模式自动巡航：20Hz /cmd_vel 控制（含安全约束）
//
// 反馈源：FAST-LIO2 /Odometry（odom 系，原 camera_init）——与 rviz "Publish Point"
// 点击航点严格同源；安全：复用 NAVIGATING 的障碍物 warn/stop 阈值与 /estop。
// 阿克曼约束：|w| ≤ v / R_min（HUNTER-SE 最小转弯半径 1.9m）。
// ==========================================================================
void AutoMissionNode::cruiseControlStep()
{
  // ---- 非巡航运行态：必要时补一帧零速后静默 ----
  if (mission_mode_ != "mapping" || !cruise_active_ || state_ != MissionState::MAPPING) {
    if (cruise_cmd_published_) {
      publishCruiseCmd(0.0, 0.0);
      cruise_cmd_published_ = false;
    }
    return;
  }

  // ---- 快照（减少锁占用时间） ----
  nav_msgs::msg::Odometry lio;
  rclcpp::Time lio_arrive(0, 0, RCL_SYSTEM_TIME);
  {
    std::lock_guard<std::mutex> lk(data_mutex_);
    lio = latest_lio_odom_;
    lio_arrive = last_lio_odom_arrive_;
  }

  // ---- 暂停（遥控接管 / CRITICAL）：停车等待，状态保持 MAPPING ----
  if (mapping_paused_.load()) {
    publishCruiseCmd(0.0, 0.0);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
      "[巡航] 已暂停（遥控接管或系统异常），停车等待中");
    return;
  }

  // ---- FAST-LIO2 定位新鲜度 ----
  const double lio_age = (lio_arrive.seconds() <= 0.0) ?
    1e9 : (this->now() - lio_arrive).seconds();
  if (lio_age > cruise_odom_timeout_) {
    publishCruiseCmd(0.0, 0.0);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
      "[巡航] FAST-LIO2 /Odometry 超时（>%.1fs），停车等待", cruise_odom_timeout_);
    return;
  }

  // ---- 感知存活（障碍物兜底依赖融合结果） ----
  if (!isPerceptionAlive()) {
    publishCruiseCmd(0.0, 0.0);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
      "[巡航] /perception/fused_objects 超时，停车等待");
    return;
  }

  // ---- 障碍物检查（与 NAVIGATING 相同阈值） ----
  const double obs_dist = nearestObstacleDist();
  if (obs_dist < stop_obstacle_dist_) {
    publishCruiseCmd(0.0, 0.0);
    RCLCPP_ERROR(get_logger(),
      "[巡航→ESTOP] 障碍物距离 %.2fm < 急停阈值 %.2fm，触发 ESTOP",
      obs_dist, stop_obstacle_dist_);
    triggerEstop("建图巡航中障碍物过近");
    return;
  }
  if (obs_dist < warn_obstacle_dist_) {
    publishCruiseCmd(0.0, 0.0);
    if (!cruise_obstacle_wait_) {
      cruise_obstacle_wait_ = true;
      cruise_obstacle_wait_start_ = this->now();
    } else if ((this->now() - cruise_obstacle_wait_start_).seconds() > obstacle_wait_timeout_) {
      RCLCPP_ERROR(get_logger(),
        "[巡航] 障碍物等待超时（%.0fs），触发 ESTOP", obstacle_wait_timeout_);
      triggerEstop("建图巡航中障碍物长时间未清除");
      return;
    }
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "[巡航] 障碍物距离 %.2fm < 警告阈值 %.2fm，停车避让中",
      obs_dist, warn_obstacle_dist_);
    return;
  }
  cruise_obstacle_wait_ = false;

  // ---- 目标航点 ----
  if (waypoints_.empty() || current_wp_idx_ >= waypoints_.size()) {
    publishCruiseCmd(0.0, 0.0);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "[巡航] 无有效目标航点，停车");
    return;
  }
  const Waypoint & wp = waypoints_[current_wp_idx_];

  // ---- 位姿（odom 系，原 camera_init，与航点同源） ----
  const double px = lio.pose.pose.position.x;
  const double py = lio.pose.pose.position.y;
  const auto & q = lio.pose.pose.orientation;
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  const double dx = wp.x - px;
  const double dy = wp.y - py;
  const double dist = std::hypot(dx, dy);

  // ---- 到达判定（距离阈值 ≥ 阿克曼停车精度） ----
  if (dist < cruise_reach_dist_) {
    RCLCPP_INFO(get_logger(), "[巡航] 到达航点 [%zu] %s（距离 %.2fm）",
                current_wp_idx_, wp.label.c_str(), dist);
    std_msgs::msg::Int32 idx_msg;
    idx_msg.data = static_cast<int32_t>(current_wp_idx_);
    waypoint_idx_pub_->publish(idx_msg);

    const size_t next = current_wp_idx_ + 1;
    if (next >= waypoints_.size()) {
      publishCruiseCmd(0.0, 0.0);
      cruise_active_ = false;
      RCLCPP_INFO(get_logger(),
        "[巡航] 全部 %zu 个航点巡航完成，车辆停车，建图继续进行。"
        "完成后 Ctrl+C 保存地图（或调用 /fast_lio2/map_save）",
        waypoints_.size());
      return;
    }
    current_wp_idx_ = next;
    publishCruiseCmd(0.0, 0.0);   // 切换目标先停车一拍，下一拍重新起步
    return;
  }

  // ---- 朝向目标的 P 控制律 ----
  const double bearing = std::atan2(dy, dx);
  const double yaw_err = wrapAngle(bearing - yaw);
  const double aerr = std::fabs(yaw_err);

  double v = cruise_max_speed_;
  if (aerr > cruise_yaw_slow_deg_ * M_PI / 180.0) {
    v = std::min(v, cruise_turn_speed_);          // 大航向偏差降速
  }
  if (dist < cruise_brake_dist_) {
    // 接近目标线性减速，但不低于 cruise_min_speed_（避免临门蠕动）
    v = std::min(v, std::max(cruise_min_speed_,
          cruise_max_speed_ * dist / cruise_brake_dist_));
  }

  double w = cruise_kp_yaw_ * yaw_err;
  // 角速度硬上限
  w = std::clamp(w, -cruise_max_yaw_rate_, cruise_max_yaw_rate_);
  // 阿克曼几何约束：转弯半径 R = v/|w| ≥ R_min（HUNTER-SE 无法原地转向）
  if (v > 0.05) {
    const double w_geom = v / cruise_min_turn_radius_;
    w = std::clamp(w, -w_geom, w_geom);
  }
  publishCruiseCmd(v, w);

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
    "[巡航] 目标[%zu/%zu] %s 距离=%.2fm 航向偏差=%.1f° v=%.2f w=%.2f 障碍=%.2fm",
    current_wp_idx_ + 1, waypoints_.size(), wp.label.c_str(),
    dist, aerr * 180.0 / M_PI, v, w, obs_dist);
}

void AutoMissionNode::publishCruiseCmd(double v, double w)
{
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = v;
  cmd.angular.z = w;
  cmd_vel_pub_->publish(cmd);
  cruise_cmd_published_ = true;
}

double AutoMissionNode::wrapAngle(double a)
{
  return std::remainder(a, 2.0 * M_PI);
}


// ==========================================================================
// 服务回调：启动建图巡航（热重载航点 + 定位可用性检查）
// ==========================================================================
void AutoMissionNode::startCruiseCallback(
  const std_srvs::srv::Trigger::Request::SharedPtr,
  std_srvs::srv::Trigger::Response::SharedPtr resp)
{
  if (mission_mode_ != "mapping") {
    resp->success = false;
    resp->message = "当前为 " + mission_mode_ + " 模式，自动巡航仅在建图模式可用";
    RCLCPP_WARN(get_logger(), "[巡航] %s", resp->message.c_str());
    return;
  }
  {
    std::lock_guard<std::mutex> lk(data_mutex_);
    if (estop_signal_) {
      resp->success = false;
      resp->message = "/estop 急停信号激活中，禁止启动巡航";
      RCLCPP_WARN(get_logger(), "[巡航] %s", resp->message.c_str());
      return;
    }
  }

  // 热重载航点（waypoint_recorder 边建图边写入 yaml）
  std::string msg;
  if (!reloadWaypointsFromFile(msg)) {
    resp->success = false;
    resp->message = "航点热重载失败：" + msg;
    return;
  }

  // FAST-LIO2 定位可用性
  {
    std::lock_guard<std::mutex> lk(data_mutex_);
    const double age = (last_lio_odom_arrive_.seconds() <= 0.0) ?
      1e9 : (this->now() - last_lio_odom_arrive_).seconds();
    if (age > cruise_odom_timeout_) {
      resp->success = false;
      resp->message = "FAST-LIO2 /Odometry 无有效数据，无法启动巡航；请确认 fast_lio2 正常输出";
      RCLCPP_ERROR(get_logger(), "[巡航] FAST-LIO2 /Odometry 无有效数据（age=%.1fs）", age);
      return;
    }
  }

  current_wp_idx_ = 0;
  cruise_obstacle_wait_ = false;
  cruise_active_ = true;
  std::ostringstream oss;
  oss << "自动巡航已启动，共 " << waypoints_.size() << " 个航点，"
      << "直行速度上限 " << cruise_max_speed_ << " m/s";
  resp->success = true;
  resp->message = oss.str();
  RCLCPP_INFO(get_logger(), "[巡航] %s", resp->message.c_str());
}

void AutoMissionNode::stopCruiseCallback(
  const std_srvs::srv::Trigger::Request::SharedPtr,
  std_srvs::srv::Trigger::Response::SharedPtr resp)
{
  if (!cruise_active_) {
    resp->success = true;
    resp->message = "当前没有进行中的自动巡航";
    return;
  }
  cruise_active_ = false;
  publishCruiseCmd(0.0, 0.0);
  resp->success = true;
  resp->message = "自动巡航已停止，车辆安全停车（FAST-LIO2 建图继续进行）";
  RCLCPP_INFO(get_logger(), "[巡航] %s", resp->message.c_str());
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
    if (cruise_active_) {
      cruise_active_ = false;
      mapping_paused_.store(false);
      publishCruiseCmd(0.0, 0.0);
      RCLCPP_WARN(get_logger(), "[急停] 建图巡航已终止，恢复后需重新调用 start_mapping_cruise");
    }
    state_ = MissionState::ESTOP;
    publishStatus();
    return;
  }

  // ---------- 建图模式：状态保持 MAPPING，巡航由 20Hz cruise timer 驱动 ----------
  // 注意：非 AUTO/CRITICAL 仅暂停巡航（mapping_paused_ → 停车），不切换状态，
  // 避免 pcd_to_map 把"遥控接管/临时异常"误判为 MAPPING→非MAPPING 的建图结束
  // 信号而提前触发地图转换。急停仍会切 ESTOP（急停 = 终止建图，语义一致）。
  if (mission_mode_ == "mapping") {
    // 自触发急停（障碍物类）解除：障碍物远离后自动发布 /estop=false，
    // 释放 decision_making 与 start_mapping_cruise 门控（巡航仍需手动重启）
    if (estop_snap && tryReleaseSelfEstop()) {
      estop_snap = false;
    }
    const bool pause = (behavior.mode != "AUTO") || (health.overall_status == "CRITICAL");
    mapping_paused_.store(pause);
    if (pause) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "[建图] 非 AUTO/health=CRITICAL，暂停巡航（状态保持 MAPPING），可随时遥控接管");
    }
    if (state_ != MissionState::MAPPING) {
      state_ = MissionState::MAPPING;
      RCLCPP_INFO(get_logger(),
        "[建图] 进入建图模式，FAST-LIO2 在线建图中，不下发导航目标"
        "（自动巡航请调用 /auto_mission/start_mapping_cruise 服务）");
    }
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
    // FAULT 不因 health 恢复而被默默清除（V0.0.91）：故障锁存只能由
    // “模式开关离开 AUTO 再回来”人工确认解除
    if (state_ != MissionState::IDLE && state_ != MissionState::ESTOP &&
      state_ != MissionState::FAULT)
    {
      RCLCPP_ERROR(get_logger(),
        "[降级] SystemHealth=CRITICAL，停止导航，安全停车");
      cancelCurrentGoal();
      state_ = MissionState::IDLE;
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
      // 条件满足，先确认 Nav2 就绪（bt_navigator ACTIVE，inactive 会拒收 goal）
      if (!nav_active_.load()) {
        queryNavigatorState();
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
          "[IDLE] AUTO 条件满足，但 bt_navigator 未激活（Nav2 bringup 进行中），等待");
        break;
      }
      // 再检查定位收敛
      if (!isLocalizationValid()) {
        RCLCPP_INFO(get_logger(), "[IDLE→WAITING_LOCALIZE] AUTO 条件满足，等待定位收敛");
        localize_wait_started_ = true;
        localize_wait_start_ = this->now();
        state_ = MissionState::WAITING_LOCALIZE;
      } else {
        RCLCPP_INFO(get_logger(), "[IDLE→NAVIGATING] AUTO 条件满足，定位已收敛，清图后开始导航");
        current_wp_idx_ = 0;
        wp_fail_count_ = 0;
        clearCostmapsOnStart();  // V0.0.92：清图后由 NAVIGATING 入口按序发送 goal
        state_ = MissionState::NAVIGATING;
        // V0.0.92：移除此处的 sendNextWaypoint() 调用。
        // 原来立即发 goal 会在清图完成前触发规划，导致 “起点在致命栅格”。
        // NAVIGATING 状态的“无在途 goal”检查会在清图完成后自动发送第一个目标。
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
        RCLCPP_INFO(get_logger(), "[WAITING_LOCALIZE→NAVIGATING] 定位已收敛，清图后开始导航");
        state_ = MissionState::NAVIGATING;
        current_wp_idx_ = 0;
        wp_fail_count_ = 0;
        localize_wait_started_ = false;
        clearCostmapsOnStart();  // V0.0.92：清图后由 NAVIGATING 入口按序发送 goal
        // V0.0.92：同 IDLE→NAVIGATING，移除立即发 goal，由状态机等待清图完成
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

      // Nav2 就绪门控：bt_navigator 非 ACTIVE（bringup 进行中/中途重启）时暂停发送
      if (!nav_active_.load()) {
        queryNavigatorState();
        if (!nav_wait_started_) {
          nav_wait_started_ = true;
          nav_wait_start_ = this->now();
        }
        if ((this->now() - nav_wait_start_).seconds() > nav_active_wait_timeout_) {
          RCLCPP_ERROR(get_logger(),
            "[NAVIGATING] 等待 bt_navigator 激活超时（%.0fs），回到 IDLE",
            nav_active_wait_timeout_);
          nav_wait_started_ = false;
          state_ = MissionState::IDLE;
          break;
        }
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
          "[NAVIGATING] bt_navigator 未激活（Nav2 bringup 进行中），暂不发送 goal");
        break;
      }
      nav_wait_started_ = false;

      // V0.0.92：代价地图清除门控（清图完成前不发 goal）。
      // 背景：clearCostmapsOnStart() 进入时若服务未就继，会留下 costmaps_clear_pending_=true。
      // 此处每 100ms（10Hz 主循环）重试一次，服务就绪后去除标志，再在下方发送 goal。
      if (costmaps_clear_pending_.load()) {
        const auto try_clear_retry =
          [this](const rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr & cli, const char * name) {
            if (!cli->service_is_ready()) {
              return false;
            }
            cli->async_send_request(std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>());
            RCLCPP_INFO(get_logger(), "[清图重试] 已请求清除 %s", name);
            return true;
          };
        const bool g_ok = try_clear_retry(clear_global_costmap_srv_, "global_costmap");
        const bool l_ok = try_clear_retry(clear_local_costmap_srv_, "local_costmap");
        if (g_ok && l_ok) {
          costmaps_clear_pending_.store(false);
          RCLCPP_INFO(get_logger(),
            "[NAVIGATING] 代价地图清除完成，下一个 tick 发送 goal");
        } else {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
            "[NAVIGATING] 等待清图服务就绪（global=%s local=%s），暂不发送 goal",
            g_ok ? "OK" : "PEND", l_ok ? "OK" : "PEND");
        }
        break;  // 本 tick 不发 goal，等待清图完成
      }

      // goal 超时检查
      if (goal_in_flight_) {
        const double elapsed = (this->now() - goal_send_time_).seconds();

        // V0.0.95 受阻检测（stall）：goal 在途但朝目标推进停滞 → 判“前方障碍
        // 无法绕行 / 无可达路径”。V0.0.96 判据由“位移标量”改为“朝目标推进量”：
        //   推进量 = 发 goal 时到航点距离 − 当前到航点距离；行为树脱困倒车会增大
        //   到航点距离（推进量为负）→ 仍判受阻，而位移标量会被“后退”骗过。
        // 现场依据（V0.0.95 实车日志）：车开到航点 A 附近后全局规划持续报
        //   "Starting point in lethal space!"（车停在静态障碍膨胀区内），
        //   行为树清图/等待均无效 → 需换点/锁存并给出可判读结论。
        // V0.0.97 追加【绕障机动宽容】：V0.0.97 放开 MPPI 倒车（vx_min=-0.20）+
        //   Smac REEDS_SHEPP 后，正常绕障是"倒 0.4m → 进 0.5m → 再倒"的多段机动，
        //   净推进可能长期 <0.15m 却始终在动。此时若按旧逻辑判受阻换点，等于把
        //   "正在绕障"误杀成"绕不过去"。故增加累计行程判据：
        //     累计行程 < stall_path_allow_m(1.0m) → 真的没动，仍判受阻；
        //     累计行程 ≥ 1.0m            → 判定为"多点机动中"，暂不换点，
        //       由单航点超时 goal_timeout_（autonomous_nav_params.yaml 配 45s，节点默认 60s）兜底，避免无限原地磨。
        if (goal_start_dist_ >= 0.0) {
          double px = 0.0, py = 0.0;
          double progress = 0.0;
          if (currentMapPose(px, py)) {
            progress = goal_start_dist_ - std::hypot(
              waypoints_[current_wp_idx_].x - px, waypoints_[current_wp_idx_].y - py);
            // 累计行程（map 系 |Δ位姿| 之和；位姿源 2Hz，10Hz tick 重复读数增量为 0）
            // V0.0.97 单步死区 0.02m：滤除 NDT 位姿微抖（静止时 ±mm~cm 级），
            //   否则 25s 内数十次抖动会被累加成一笔"假里程"，把真正卡死的情形放过。
            //   真实行驶每 2Hz 位姿更新位移 ≥0.1m（0.2m/s×0.5s），远超死区，不会漏计。
            if (has_last_stall_pose_) {
              const double step = std::hypot(px - last_stall_x_, py - last_stall_y_);
              if (step >= 0.02) {
                goal_path_len_ += step;
              }
            }
            last_stall_x_ = px;
            last_stall_y_ = py;
            has_last_stall_pose_ = true;
          }
          const bool stalled = elapsed > stall_detect_time_ && progress < stall_move_eps_;
          const bool maneuvering = goal_path_len_ >= stall_path_allow_m_;
          if (stalled && !maneuvering) {
            ++blocked_count_;
            RCLCPP_ERROR(get_logger(),
              "[NAVIGATING] 航点[%zu]%s 受阻：%.0fs 内向目标仅推进 %.2fm（< %.2fm，"
              "起始距离 %.2fm / 当前 %.2fm），累计行程仅 %.2fm（< %.2fm，车基本没动）"
              "——前方障碍无法绕行、航点无可达路径，或车已停在（静态地图）障碍膨胀区内"
              "致规划全失败；取消本 goal 并按失败处理（第 %d 次受阻）",
              current_wp_idx_, waypoints_[current_wp_idx_].label.c_str(),
              elapsed, progress, stall_move_eps_, goal_start_dist_,
              goal_start_dist_ - progress, goal_path_len_, stall_path_allow_m_,
              blocked_count_);
            cancelCurrentGoal();
            wp_fail_count_++;
            if (wp_fail_count_ >= max_wp_failures_) {
              enterFault("航点受阻（前方障碍无法绕行）连续达上限");
            } else {
              current_wp_idx_ = (current_wp_idx_ + 1) % waypoints_.size();
              // V0.0.98：不再立即发下一个 goal —— 由取消静置门控（goal_cancel_pending_
              //   + settle）在"无在途 goal"重发路径等旧 goal 结果回传后再发，
              //   消除"取消后毫秒级发新 goal 被旧 BT 失败状态波及"的竞态
            }
            break;
          }
          if (stalled && maneuvering) {
            // 正在多点掉头/受限倒车绕障：不换点，仅提示（可能持续到 goal_timeout 兜底）
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
              "[NAVIGATING] 航点[%zu]%s 净推进 %.2fm 但累计行程 %.2fm（多点掉头/倒车绕障中），"
              "暂不判受阻；若长期无进展请核对 /safety/state 净空、stop_dist(0.90)/"
              "slow_dist(1.40) 与 local_costmap 近车 0.7m 内是否有自反射假障碍",
              current_wp_idx_, waypoints_[current_wp_idx_].label.c_str(),
              progress, goal_path_len_);
          }
        }

        if (elapsed > goal_timeout_) {
          RCLCPP_WARN(get_logger(),
            "[NAVIGATING] 航点[%zu]%s 导航超时（%.0fs），跳过该航点",
            current_wp_idx_, waypoints_[current_wp_idx_].label.c_str(), goal_timeout_);
          cancelCurrentGoal();
          wp_fail_count_++;
          if (wp_fail_count_ >= max_wp_failures_) {
            enterFault("单航点导航超时连续达上限");
          } else {
            current_wp_idx_ = (current_wp_idx_ + 1) % waypoints_.size();
            // V0.0.98：同上，交给取消静置门控后重发，不立即发下一个 goal
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

      // 正常导航中，只在没有 goal 在飞时重新发送（防止重复发送）；
      // goal 被拒/失败后的退避窗口由 sendNextWaypoint 开头统一检查
      if (!goal_in_flight_) {
        // V0.0.98 取消静置门控：主动取消后，bt_navigator 需时间完成旧 goal 的
        //   取消与 BT 清理；立即发新 goal 会被同一轮失败波及（V0.0.97 现场：
        //   取消→2ms 后发的航点[3]被接受 2ms 即 ABORTED，fail_count 假增）。
        //   放行条件：旧 goal 结果已到 resultCallback（清 pending），或 settle
        //   超时兜底（结果不回传：bt_navigator 重启/崩溃等场景）。
        if (goal_cancel_pending_) {
          const double settle_elapsed = (this->now() - goal_cancel_time_).seconds();
          if (settle_elapsed > goal_cancel_settle_time_) {
            goal_cancel_pending_ = false;
            RCLCPP_WARN(get_logger(),
              "[NAVIGATING] 取消静置超时（%.1fs，旧 goal 结果未回传），兜底恢复发送 goal",
              goal_cancel_settle_time_);
          } else {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
              "[NAVIGATING] 等待上一 goal 取消静置（%.1f/%.1fs，等旧结果回传），暂不发航点[%zu]",
              settle_elapsed, goal_cancel_settle_time_, current_wp_idx_);
            break;
          }
        }
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
          "[NAVIGATING] 无在途 goal，重新发送航点[%zu]", current_wp_idx_);
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
      // 自触发急停（障碍物类）：危险解除后自动发布 /estop=false 释放下游；
      // 外部急停（health_monitor CAN 等）等待发布方恢复边沿的 /estop=false
      if (estop_snap && tryReleaseSelfEstop()) {
        estop_snap = false;
      }
      // 急停状态下等待 /estop 变 false 且 AUTO 条件恢复后方可退出
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
    case MissionState::FAULT:
    // ------------------------------------------------------------------
    {
      // 故障锁存：停车、不重发 goal（非 AUTO 降级分支已在主循环上方将其重置为
      // IDLE，能走到这里说明 AUTO 仍持有，即等待人工处置）
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "[FAULT] 任务已锁存（%s）：请检查 ①车身四周/脚底是否被假障碍占位（遥控移开后用 "
        "rviz2 2D Pose Estimate 重定位）②定位是否跳变（map→odom）③航点是否可达；"
        "处置后将模式开关离开 AUTO 再切回以重启任务",
        fault_reason_.empty() ? "未记录原因" : fault_reason_.c_str());
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
  // --- EKF 定位协方差（inline） ---
  const auto & cov = latest_odom_.pose.covariance;
  // 协方差矩阵对角元素 [0,7,35] 对应 x,y,yaw
  const double cov_trace = cov[0] + cov[7] + cov[35];
  if (cov_trace > localize_cov_threshold_) {
    return false;
  }
  // --- AMCL 粒子收敛（V0.0.92 inline） ---
  // 注意：amcl_pose_received_ 和 latest_amcl_pose_ 均由 data_mutex_ 保护，已持锁。
  if (!amcl_pose_received_) {
    return false;
  }
  const auto & amcl_cov = latest_amcl_pose_.pose.covariance;
  if (amcl_cov[0] + amcl_cov[7] > amcl_cov_threshold_) {
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
// 定位有效性（V0.0.92：EKF 协方差 + AMCL 粒子收敛双重检查）
// ==========================================================================
bool AutoMissionNode::isLocalizationValid()
{
  std::lock_guard<std::mutex> lk(data_mutex_);
  // ① EKF odom 协方差（快里程计收敛，反映 odom→base_link 质量）
  const auto & odom_cov = latest_odom_.pose.covariance;
  const double odom_trace = odom_cov[0] + odom_cov[7] + odom_cov[35];
  if (odom_trace > localize_cov_threshold_) {
    return false;
  }
  // ② 全局重定位收敛（map→base_link 全局定位质量，V0.0.93 由 NDT 提供）
  // 未收到任何 /relocalization/pose → 重定位尚未启动，不允许导航
  if (!amcl_pose_received_) {
    return false;
  }
  // x+y 方差和：NDT 收敛时发布小协方差（默认 0.01，和=0.02），未收敛时=100（和=200）
  const auto & amcl_cov = latest_amcl_pose_.pose.covariance;
  if (amcl_cov[0] + amcl_cov[7] > amcl_cov_threshold_) {
    return false;
  }
  return true;
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
// bt_navigator lifecycle 状态查询（异步，1Hz 节流，不阻塞主循环）
// ==========================================================================
void AutoMissionNode::queryNavigatorState()
{
  const rclcpp::Time now = this->now();
  if ((now - nav_state_query_time_).seconds() < 1.0) {
    return;   // 查询节流
  }
  nav_state_query_time_ = now;

  if (!nav_state_client_->service_is_ready()) {
    // 服务未上线 ⇒ bt_navigator 尚未 configure 完成
    nav_active_.store(false);
    return;
  }
  auto req = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
  nav_state_client_->async_send_request(req,
    std::bind(&AutoMissionNode::navigatorStateResponse, this, std::placeholders::_1));
}

void AutoMissionNode::navigatorStateResponse(
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture future)
{
  const auto & state = future.get()->current_state;
  nav_active_.store(state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
}

// ==========================================================================
// /map 回调：缓存静态地图边界（航点越界校验用，V0.0.82；V0.0.87 起同时
// 用于目标点未建图(unknown)栅格校验——矩形界内仍可能有建图空洞）
// ==========================================================================
void AutoMissionNode::mapCallback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lk(data_mutex_);
  const bool first_map = (latest_map_ == nullptr);
  latest_map_ = msg;
  map_min_x_ = msg->info.origin.position.x;
  map_min_y_ = msg->info.origin.position.y;
  map_max_x_ = map_min_x_ + static_cast<double>(msg->info.width) * msg->info.resolution;
  map_max_y_ = map_min_y_ + static_cast<double>(msg->info.height) * msg->info.resolution;
  if (first_map) {
    RCLCPP_INFO(get_logger(),
      "auto_mission 已缓存静态地图边界：x[%.2f, %.2f] y[%.2f, %.2f]（航点安全边距 %.2fm）",
      map_min_x_, map_max_x_, map_min_y_, map_max_y_, waypoint_map_margin_);
  }
}

// ==========================================================================
// V0.0.93 方案A：/relocalization/pose 回调（原 /amcl_pose，函数名/成员名保留）
// hunter_relocalization(NDT) 以 publish_rate（默认 2Hz）持续发布 map 系位姿：
// 收敛时 covariance[0]=[7]=converged_covariance（默认 0.01），未收敛时=100。
// 每次刷新缓存 latest，isLocalizationValid() 据当前方差和动态判收敛。
// ==========================================================================
void AutoMissionNode::amclPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(data_mutex_);
  latest_amcl_pose_ = *msg;
  const bool first_amcl = !amcl_pose_received_;
  amcl_pose_received_ = true;
  if (first_amcl) {
    const double xy_cov = msg->pose.covariance[0] + msg->pose.covariance[7];
    RCLCPP_INFO(get_logger(),
      "auto_mission 已缓存全局重定位位姿（x+y 方差和=%.3f，收敛阈值=%.3f）",
      xy_cov, amcl_cov_threshold_);
  }
}

// ==========================================================================
// 航点是否落在已采集地图区域内：V0.0.82 矩形边界+安全边距；V0.0.87 增加
// 未建图(unknown)栅格校验（落点在 unknown 上必报 "Goal pose is out of
// costmap!" 死局）。地图未就绪时放行（退回旧行为）
// ==========================================================================
bool AutoMissionNode::waypointInsideMap(const Waypoint & wp)
{
  return waypointMapCheckDetail(wp).empty();
}

// 航点地图校验明细：空串=通过；否则为拒绝原因（供日志区分处置方式）
// V0.0.96 新增两级校验：
//   ③ 目标格为【占据栅格】——旧版只查未建图(-1)，占据(100)竟被放行（本次 V0.0.95 实车
//      "开到静态障碍里再卡死"的直接缺口）；
//   ④ 目标点【净空】不足——距最近占据栅格 < waypoint_clearance_m(0.50m) 时，
//      该点落在 Nav2 inflation_layer（radius 0.40）与内切半径(0.32)的致命/膨胀区内：
//      • SmacPlannerHybrid 的 areInputsValid() 用 GridCollisionChecker 判起点有效性，
//        代价为 LETHAL(254)/INSCRIBED(253)/UNKNOWN(255，allow_unknown=false) 即抛
//        "Starting point in lethal space! Cannot create feasible plan."；
//      • 局部代价地图为滚动窗口且不含静态层 → MPPI 不会避开仅在静态地图中的障碍，
//        会把车一路开到该点；此后从该点出发的所有规划全部失败，行为树清图也救不回来
//        （StaticLayer::reset() 只置 has_updated_data_ 并由静态地图重新盖章）。
std::string AutoMissionNode::waypointMapCheckDetail(const Waypoint & wp)
{
  std::unique_lock<std::mutex> lk(data_mutex_);
  if (!latest_map_) {
    return "";
  }
  const bool outside_rect =
    wp.x < map_min_x_ + waypoint_map_margin_ || wp.x > map_max_x_ - waypoint_map_margin_ ||
    wp.y < map_min_y_ + waypoint_map_margin_ || wp.y > map_max_y_ - waypoint_map_margin_;
  if (outside_rect) {
    return "静态地图矩形边界外";
  }
  // V0.0.87：目标点栅格必须为已采集区域（occupancy != -1）。矩形界内仍可能
  // 存在未建图空洞（建图边缘不齐/遮挡盲区），V0.0.87 起 global_costmap
  // track_unknown_space=true 保留 unknown，该处目标/路径必被 Nav2 拒绝。
  const int cx = static_cast<int>(std::floor(
      (wp.x - latest_map_->info.origin.position.x) / latest_map_->info.resolution));
  const int cy = static_cast<int>(std::floor(
      (wp.y - latest_map_->info.origin.position.y) / latest_map_->info.resolution));
  if (cx < 0 || cx >= static_cast<int>(latest_map_->info.width) ||
    cy < 0 || cy >= static_cast<int>(latest_map_->info.height))
  {
    return "静态地图矩形边界外";
  }
  const size_t cell = static_cast<size_t>(cy) * latest_map_->info.width + cx;
  const int8_t occ = latest_map_->data[cell];
  if (occ == -1) {
    return "目标点落在未建图(unknown)栅格上";
  }
  // V0.0.96 ③ 占据栅格（阈值 50 与 map_server occupied_thresh 语义对齐，trinary 下为 100）
  if (occ >= 50) {
    return "目标点落在占据栅格上（静态地图障碍）";
  }
  // V0.0.96 ④ 净空校验（nearestObstacleClearance 内部需再加锁，故先解锁；
  //   std::unique_lock 可在作用域末尾安全二次析构，不可用显式析构的 lock_guard）
  lk.unlock();
  const double clearance =
    nearestObstacleClearance(wp.x, wp.y, waypoint_clearance_m_ + 0.5);
  if (clearance < waypoint_clearance_m_) {
    return "距静态地图障碍仅 " + std::to_string(clearance).substr(0, 5) +
           "m < 净空要求 " + std::to_string(waypoint_clearance_m_).substr(0, 4) +
           "m（处于 Nav2 膨胀/致命区内）";
  }
  return "";
}

// ==========================================================================
// V0.0.96 点到最近【占据栅格】的欧氏距离（m）
//   • 搜索半径内无占据栅格 → 返回 max_search_radius
//   • 地图未就绪 / 点在图外 → 返回 +inf（越界由 waypointMapCheckDetail 前置拦截）
//   用途：① 航点净空校验（避免航点落在静态障碍的膨胀/致命区内，见
//   waypointMapCheckDetail 的长注释）；② 当前位姿净空诊断日志——车辆一旦停在
//   膨胀区内，Smac 会持续抛 "Starting point in lethal space"，此日志可直接判读。
// ==========================================================================
double AutoMissionNode::nearestObstacleClearance(double x, double y, double max_search_radius)
{
  std::lock_guard<std::mutex> lk(data_mutex_);
  if (!latest_map_ || max_search_radius <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  const double res = latest_map_->info.resolution;
  const double ox = latest_map_->info.origin.position.x;
  const double oy = latest_map_->info.origin.position.y;
  const int w = static_cast<int>(latest_map_->info.width);
  const int h = static_cast<int>(latest_map_->info.height);

  const double cx_f = (x - ox) / res;
  const double cy_f = (y - oy) / res;
  if (cx_f < 0.0 || cy_f < 0.0 || cx_f >= w || cy_f >= h) {
    return std::numeric_limits<double>::infinity();
  }
  const int cx = static_cast<int>(std::floor(cx_f));
  const int cy = static_cast<int>(std::floor(cy_f));

  const int span = static_cast<int>(std::ceil(max_search_radius / res));
  double best = max_search_radius;         // 搜索范围内无占据栅格 → 视为"足够远"
  const double span_sq = max_search_radius * max_search_radius;
  for (int dy = -span; dy <= span; ++dy) {
    const int yy = cy + dy;
    if (yy < 0 || yy >= h) {
      continue;
    }
    for (int dx = -span; dx <= span; ++dx) {
      const int xx = cx + dx;
      if (xx < 0 || xx >= w) {
        continue;
      }
      // 占据判据与 waypointMapCheckDetail 一致（trinary 地图为 100，阈值 50 留余量）
      if (latest_map_->data[static_cast<size_t>(yy) * w + xx] < 50) {
        continue;
      }
      const double ddx = (static_cast<double>(xx) + 0.5) * res + ox - x;
      const double ddy = (static_cast<double>(yy) + 0.5) * res + oy - y;
      const double d2 = ddx * ddx + ddy * ddy;
      if (d2 <= span_sq) {
        const double d = std::sqrt(d2);
        if (d < best) {
          best = d;
        }
      }
    }
  }
  return best;
}

// ==========================================================================
// V0.0.95 当前 map 系位姿快照（/relocalization/pose）
// 未收到全局重定位位姿时返回 false —— 调用方按“无法判定”走旧行为，
// 不因缺少位姿而误判（定位门控 isLocalizationValid() 已在此之前拦截）。
// ==========================================================================
bool AutoMissionNode::currentMapPose(double & x, double & y)
{
  std::lock_guard<std::mutex> lk(data_mutex_);
  if (!amcl_pose_received_) {
    return false;
  }
  x = latest_amcl_pose_.pose.pose.position.x;
  y = latest_amcl_pose_.pose.pose.position.y;
  return true;
}

// ==========================================================================
// V0.0.95 航点“已到达”预检
//
// 现场故障链（V0.0.94 日志）：航点[0]"0.0,0.0,0.0,起点" 与车辆起始位姿
// (-0.16, 0.11) 仅差 0.19m → Nav2 收到“目标≈自身”的退化 goal：
//   • Smac 规划的路径长度≈0，且要求终止朝向 yaw=0；
//   • 阿克曼无法原地转向，MPPI 只能持续打满转向（日志 set steering angle
//     恒为 ±0.386428 rad = 曲率钳制上限 |w|=|v|/R_min 对应的最大内轮转角）；
//   • 纵向净挪 ≈0 → ProgressChecker(0.1m/10s) 必判 Failed to make progress；
//   • V0.0.89 起 BT 恢复池只有非运动手段（清图+Wait），无法脱困 →
//     “发送→10s 无进展→ABORT→恢复→再发送”静默循环，车原地不动。
//
// 处置：位置已在到达半径内（位置重合）即视为该航点已完成，跳过不发 goal；
//   朝向不做判定（巡检任务不要求精确朝向；朝向对齐依赖原地转向，阿克曼
//   不可行，强行要求必然死锁）。tolerance 取值须 > 阿克曼停车精度且
//   ≥ 2× Nav2 xy_goal_tolerance(0.10m)，默认 0.30m。
// ==========================================================================
bool AutoMissionNode::waypointAlreadyReached(
  const Waypoint & wp, double & dist, double & yaw_err)
{
  double px = 0.0, py = 0.0, pyaw = 0.0;
  {
    std::lock_guard<std::mutex> lk(data_mutex_);
    if (!amcl_pose_received_) {
      dist = -1.0;
      yaw_err = 0.0;
      return false;                   // 无 map 系位姿：不跳过（定位门控兜底）
    }
    px = latest_amcl_pose_.pose.pose.position.x;
    py = latest_amcl_pose_.pose.pose.position.y;
    const auto & q = latest_amcl_pose_.pose.pose.orientation;
    pyaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }
  dist = std::hypot(wp.x - px, wp.y - py);
  yaw_err = std::fabs(wrapAngle(wp.yaw - pyaw));
  return dist <= already_reached_dist_;
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

  // goal 被拒/失败后的退避窗口：窗口内静默放弃，由调用方下个周期再尝试
  if ((this->now() - nav_retry_not_before_).seconds() < 0.0) {
    return;
  }

  // bt_navigator 未激活（inactive 会拒绝 goal），不发送，等待其激活
  if (!nav_active_.load()) {
    queryNavigatorState();
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
      "[sendNextWaypoint] bt_navigator 未激活，暂不发送 goal");
    return;
  }

  // action server 未发现：Nav2 可能尚未启动完成。保持当前状态等待而非回 IDLE，
  // 避免 Nav2 bringup 期间 IDLE⇄NAVIGATING 反复跳变
  if (!nav_action_client_->wait_for_action_server(std::chrono::seconds(0))) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
      "[sendNextWaypoint] Nav2 /navigate_to_pose action server 尚未发现，等待");
    return;
  }

  // ---- 航点预检（V0.0.95：已到达 + 越界双判，逐圈轮转） ----
  // ① 已到达（V0.0.95 新增）：航点与当前 map 系位姿重合（≤ already_reached_dist）
  //    时不得下发——"目标=当前位姿"对阿克曼是退化目标，必然 Fail to make progress
  //    后原地抖动（见 waypointAlreadyReached() 注释的完整故障链）。
  // ② 越界（V0.0.82/0.0.87）：目标超出静态地图边界或落在未建图(unknown)栅格上时，
  //    SmacPlannerHybrid 必报 "Goal pose is out of costmap!" → BT 恢复行为空转 →
  //    fail_count 耗尽（实车：航点 (5,5)/(0,5) 的 y=5 超出地图 y≤3.85）。
  // 两者均按"跳过并轮转下一个航点"处理；整圈都不可用时锁存 FAULT（V0.0.95 起，
  // 原先只回 IDLE —— 而 IDLE 下一拍又会重新进入 NAVIGATING 重发同一批不可用航点，
  // 形成静默抖动且无对外故障上报，与 V0.0.91 的教训一致）。
  size_t precheck_skips = 0;
  while (precheck_skips < waypoints_.size()) {
    const Waypoint & cand = waypoints_[current_wp_idx_];
    const std::string map_check = waypointMapCheckDetail(cand);
    if (!map_check.empty()) {
      RCLCPP_ERROR(get_logger(),
        "[sendNextWaypoint] 航点[%zu]%s (%.2f, %.2f) 不在已采集地图区域内"
        "（%s；边界 x[%.2f, %.2f] y[%.2f, %.2f]，安全边距 %.2fm），跳过该航点",
        current_wp_idx_, cand.label.c_str(), cand.x, cand.y, map_check.c_str(),
        map_min_x_, map_max_x_, map_min_y_, map_max_y_, waypoint_map_margin_);
      current_wp_idx_ = (current_wp_idx_ + 1) % waypoints_.size();
      ++precheck_skips;
      continue;
    }
    double reached_d = 0.0, reached_yaw = 0.0;
    if (waypointAlreadyReached(cand, reached_d, reached_yaw)) {
      RCLCPP_INFO(get_logger(),
        "[sendNextWaypoint] 航点[%zu]%s (%.2f, %.2f) 已在到达半径内"
        "（距当前位姿 %.2fm ≤ %.2fm，朝向差 %.2f rad），视为已完成并跳过",
        current_wp_idx_, cand.label.c_str(), cand.x, cand.y,
        reached_d, already_reached_dist_, reached_yaw);
      current_wp_idx_ = (current_wp_idx_ + 1) % waypoints_.size();
      ++precheck_skips;
      continue;
    }
    break;
  }
  if (precheck_skips >= waypoints_.size()) {
    enterFault(
      "全部航点均不可用：或与当前位姿重合（已在到达半径内），或位于未建图/地图外区域");
    return;
  }
  if (precheck_skips > 0) {
    RCLCPP_WARN(get_logger(), "[sendNextWaypoint] 已跳过 %zu 个不可用航点，改发航点[%zu] %s",
      precheck_skips, current_wp_idx_, waypoints_[current_wp_idx_].label.c_str());
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
  // V0.0.97 goal 世代号：在发送前自增并取值，绑定进两个回调。
  //   过期 goal（本节点已主动取消并换了新航点）的结果/响应携带的是旧世代号，
  //   回调据此直接丢弃 —— 旧实现只挡 CANCELED 结果，而旧行为树失败回传的是
  //   ABORTED，会被当成新航点的失败（一次受阻连跳 2 点 → FAULT 误锁存），
  //   并把在途 goal 的 handle 清空导致 cancelCurrentGoal() 失效。
  ++goal_epoch_;
  const uint64_t this_epoch = goal_epoch_;
  send_opts.goal_response_callback =
    std::bind(&AutoMissionNode::goalResponseCallback, this, this_epoch, std::placeholders::_1);
  send_opts.feedback_callback =
    std::bind(&AutoMissionNode::feedbackCallback, this,
      std::placeholders::_1, std::placeholders::_2);
  send_opts.result_callback =
    std::bind(&AutoMissionNode::resultCallback, this, this_epoch, std::placeholders::_1);

  nav_action_client_->async_send_goal(goal_msg, send_opts);
  goal_send_time_ = this->now();
  goal_in_flight_ = true;
  // V0.0.96 受阻检测基准：记录发 goal 时【车辆到该航点的距离】，
  //   后续以"朝目标推进量 = 起始距离 − 当前距离"判受阻（倒车会使推进量为负，仍判受阻）
  {
    double px = 0.0, py = 0.0;
    if (currentMapPose(px, py)) {
      goal_start_dist_ = std::hypot(wp.x - px, wp.y - py);
      // V0.0.96 当前位姿净空诊断：车一旦停在静态障碍的膨胀/致命区内，
      //   Smac 会持续抛 "Starting point in lethal space!"（清图无效——静态层会重新盖章），
      //   此处提前给出可判读告警（行为树恢复池已含受限倒车脱困，会尝试倒车驶离）
      const double here_clearance = nearestObstacleClearance(px, py, waypoint_clearance_m_);
      if (here_clearance < waypoint_clearance_m_) {
        RCLCPP_WARN(get_logger(),
          "[sendNextWaypoint] ⚠ 当前位姿地图净空仅 %.2fm（< %.2fm，处于 Nav2 膨胀/致命区内）："
          "全局规划可能报 Starting point in lethal space!，行为树将尝试受限倒车脱困；"
          "若反复失败请人工移车或重标航点",
          here_clearance, waypoint_clearance_m_);
      }
    } else {
      goal_start_dist_ = -1.0;
    }
    blocked_count_ = 0;
    goal_cancel_by_mission_ = false;   // 新 goal 已发出，清除上一轮的主动取消标记
    goal_cancel_pending_ = false;      // V0.0.98 新 goal 已发，取消静置完成
    // V0.0.97 绕障机动里程清零（受阻判据的"是否真的在动"依据）
    goal_path_len_ = 0.0;
    has_last_stall_pose_ = false;
  }

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
  // V0.0.98 取消静置门控：无论是否拿到 handle，都记录"已请求取消"——
  //   重发路径需等旧 goal 结果回传（resultCallback 清 pending）或 settle
  //   超时，杜绝"取消后毫秒级发新 goal 被旧 BT 失败状态波及"的竞态。
  goal_cancel_pending_ = true;
  goal_cancel_time_ = this->now();
  // V0.0.97：判据由 "goal_handle_ && goal_in_flight_" 放宽为 "goal_handle_ 非空"。
  //   旧写法在"句柄已拿到、但 resultCallback 已把 goal_in_flight_ 置 false"的
  //   竞态下会静默跳过取消 → 行为树继续驱动车辆（现场 FAULT 后仍机动 10 分钟）。
  if (goal_handle_) {
    RCLCPP_INFO(get_logger(), "[cancelCurrentGoal] 取消当前导航 goal");
    nav_action_client_->async_cancel_goal(goal_handle_);
    // V0.0.95：标记为"本节点主动取消"，使 resultCallback 不再重复计失败/换点
    goal_cancel_by_mission_ = true;
    goal_handle_ = nullptr;
    goal_in_flight_ = false;
    return;
  }
  if (goal_in_flight_) {
    // 极窄竞态窗口：goal 已发出但服务端响应（含 handle）尚未回到本节点。
    //   此时无法主动取消；该 goal 的结果会被世代号判为过期而丢弃（若期间已发新点），
    //   行为树将执行到自身超时。日志留痕便于现场定位。
    RCLCPP_WARN(get_logger(),
      "[cancelCurrentGoal] goal 在途但服务端响应未到（无 handle），本次无法主动取消："
      "该 goal 的结果将按过期世代丢弃，行为树可能继续执行到自身超时");
  }
  goal_in_flight_ = false;
}

// ==========================================================================
// 触发急停
// ==========================================================================
void AutoMissionNode::triggerEstop(const std::string & reason)
{
  cancelCurrentGoal();
  state_ = MissionState::ESTOP;

  // 标记自触发（障碍物类）：危险解除后由 tryReleaseSelfEstop 发布 /estop=false。
  // 置位需先于 publish——单线程顺序回调下，订阅回调会在本函数返回后收到该消息
  estop_self_triggered_.store(true);

  std_msgs::msg::Bool estop_msg;
  estop_msg.data = true;
  estop_pub_->publish(estop_msg);

  RCLCPP_ERROR(get_logger(), "[ESTOP] 触发急停：%s", reason.c_str());
}

// ==========================================================================
// 任务故障锁存（V0.0.91）
//
// 现场教训（问题②“遥控接管后重新自主，车停在原地不动”）：旧实现达失败上限时
// 回到 IDLE，而 IDLE 分支下一拍就发现“AUTO 条件仍满足”→ 重置计数并重新
// 从航点[0] 发送同一批不可规划的目标，形成 IDLE⇄NAVIGATING 静默抖动：
//   • 车辆永不移动，也无任何对外故障上报，旁人无法从日志判断“已放弃”；
//   • 行为树（V0.0.89 起）恢复池仅剩“清图 + Wait”非运动手段，帮不了
//     “起点在致命栅格”——这种需要重定位/人工移车的死局。
// 因此改为锁存到 FAULT：不再发 goal、不再重置计数，持续输出带处置指引的告警，
// 需模式开关离开 AUTO（本节点主循环上方的降级分支会清回 IDLE）才能重启任务。
// 注：不发 /estop——车已停且非危险场景，保持急停通道语义纯净。
// ==========================================================================
void AutoMissionNode::enterFault(const std::string & reason)
{
  fault_reason_ = reason;
  RCLCPP_ERROR(get_logger(),
    "[FAULT] 连续失败 %d/%d 次达上限（%s），停止巡航并锁存。"
    "高频成因：起点位于致命栅格/定位跳变/假障碍累积——清图+等待无法自救，"
    "需人工移车并用 rviz2 2D Pose Estimate 重定位，然后将模式开关离开 AUTO 再切回",
    wp_fail_count_, max_wp_failures_, reason.c_str());
  cancelCurrentGoal();
  wp_fail_count_ = 0;   // 计数归零，但锁存不因此解除（靠状态而非计数拦截重发）
  state_ = MissionState::FAULT;
}

// ==========================================================================
// 任务（重）启动时主动清除两张代价地图（V0.0.91）
//
// 背景：接管后重新自主时，global/local costmap 里往往残留接管期间累积的
//   假障碍（旧配置下 lidar_cloud 源 clearing:false 只增不减；观测时基错位
//   使 raytrace 被跳过），起点直接落在致命栅格 → Smac 返
//   "Starting point in lethal space"，1Hz 重规划全失败。行为树的清图只在
//   规划失败后触发，本函数将其前置到“起步前”，两者配合才能避免死循环。
// V0.0.92 升级：进入 NAVIGATING 时如果服务未就继，将不再直接放弃。
// 改为设置 costmaps_clear_pending_ 标志，NAVIGATING 状机入口每 100ms 重试
// 直到服务就绪并完成清除后才允许发送第一个 goal。
// ==========================================================================
void AutoMissionNode::clearCostmapsOnStart()
{
  // 无论立即还是重试，先置挂起标志
  costmaps_clear_pending_.store(true);

  const auto try_clear =
    [this](const rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr & cli, const char * name) {
      if (!cli->service_is_ready()) {
        RCLCPP_WARN(get_logger(),
          "[清图] %s 未就绪，将在 NAVIGATING 中每 100ms 重试", name);
        return false;
      }
      cli->async_send_request(std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>());
      RCLCPP_INFO(get_logger(), "[清图] 已请求清除 %s", name);
      return true;
    };
  const bool g_ok = try_clear(
    clear_global_costmap_srv_, "global_costmap/clear_entirely_global_costmap");
  const bool l_ok = try_clear(
    clear_local_costmap_srv_, "local_costmap/clear_entirely_local_costmap");
  if (g_ok && l_ok) {
    // 两个服务均就绪，清图请求已发出，去除挂起标志
    // Nav2 服务处理约 1~2 个时钟周期（<100ms），10Hz 主循环下一个 tick 可发 goal
    costmaps_clear_pending_.store(false);
  }
}

// ==========================================================================
// 自触发急停解除检查（仅障碍物类自触发急停；外部急停由发布方负责解除）
// 危险解除判据：前向扇区最近障碍物退至减速阈值（warn_obstacle_dist_）之外。
// 返回 true 表示本次调用完成了解除动作（已发布 /estop=false 并清理标志）。
// ==========================================================================
bool AutoMissionNode::tryReleaseSelfEstop()
{
  if (!estop_self_triggered_.load()) {
    return false;
  }
  if (nearestObstacleDist() <= warn_obstacle_dist_) {
    return false;   // 障碍物仍在警戒范围内，保持急停
  }

  std_msgs::msg::Bool estop;
  estop.data = false;
  estop_pub_->publish(estop);
  {
    std::lock_guard<std::mutex> lk(data_mutex_);
    estop_signal_ = false;
  }
  estop_self_triggered_.store(false);
  RCLCPP_INFO(get_logger(),
    "[急停解除] 自触发急停的障碍物已远离（> %.1fm），发布 /estop=false", warn_obstacle_dist_);
  return true;
}

// ==========================================================================
// Nav2 goal response 回调
// ==========================================================================
void AutoMissionNode::goalResponseCallback(
  uint64_t epoch,
  const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & handle)
{
  // V0.0.97 过期响应门控：新 goal 已发出时，迟到的旧 goal 响应（极少见）立即取消，
  //   避免两个 goal 并存互相抢占、并把旧 handle 写进 goal_handle_ 污染取消动作。
  if (epoch != goal_epoch_) {
    if (handle) {
      nav_action_client_->async_cancel_goal(handle);
    }
    RCLCPP_WARN(get_logger(),
      "[Nav2] 丢弃过期 goal 响应（世代 %lu ≠ 当前 %lu）%s",
      static_cast<unsigned long>(epoch), static_cast<unsigned long>(goal_epoch_),
      handle ? "并已请求取消该过期 goal" : "");
    return;
  }

  // V0.0.97：【先释放锁再处理故障】。旧实现在持有 goal_handle_mutex_ 时调用
  //   enterFault()，而 enterFault() → cancelCurrentGoal() 会再次锁同一把
  //   std::mutex（非递归）→ 自死锁（现场未触发，但一旦"连续被拒收"就是挂死）。
  bool need_fault = false;
  {
    std::lock_guard<std::mutex> lk(goal_handle_mutex_);
    if (!handle) {
      goal_in_flight_ = false;
      wp_fail_count_++;
      // 退避 + 失败上限：杜绝 10Hz 高频重发；拒收达上限回 IDLE 等待条件重置
      nav_retry_not_before_ = this->now() + rclcpp::Duration::from_seconds(nav_retry_backoff_);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "[Nav2] goal 被服务端拒绝（fail_count=%d/%d），%.0fs 后重试",
        wp_fail_count_, max_wp_failures_, nav_retry_backoff_);
      if (wp_fail_count_ >= max_wp_failures_) {
        need_fault = true;
      }
    } else {
      goal_handle_ = handle;
      RCLCPP_INFO(get_logger(), "[Nav2] goal 已被接受，开始导航至航点[%zu]",
        current_wp_idx_);
    }
  }
  if (need_fault) {
    enterFault("goal 连续被 bt_navigator 拒收");
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
  uint64_t epoch,
  const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result)
{
  // V0.0.97 过期结果门控（必须放在最前，先于任何状态清理）：
  //   世代号不同于当前值 ⇒ 本结果属于【已被本节点放弃的旧 goal】。这种结果
  //   ① 不能计入 wp_fail_count_（否则一次受阻连跳 2 个航点 → FAULT 误锁存）；
  //   ② 不能触发换点（换点由取消方完成）；
  //   ③ 更不能清空 goal_in_flight_/goal_handle_ —— 旧实现在回调开头无条件清空，
  //      导致在途（新）goal 的句柄丢失、后续 cancelCurrentGoal() 变空操作，
  //      行为树在 FAULT 后仍持续驱动车辆满舵/倒车机动（现场 10 分钟）。
  if (epoch != goal_epoch_) {
    RCLCPP_DEBUG(get_logger(),
      "[Nav2] 丢弃过期 goal 结果（世代 %lu ≠ 当前 %lu，航点[%zu]，code=%d）",
      static_cast<unsigned long>(epoch), static_cast<unsigned long>(goal_epoch_),
      current_wp_idx_, static_cast<int>(result.code));
    return;
  }

  goal_in_flight_ = false;
  {
    std::lock_guard<std::mutex> lk(goal_handle_mutex_);
    goal_handle_ = nullptr;
  }

  // V0.0.95：本节点主动取消（受阻换点/超时跳过/降级/急停）产生的 CANCELED
  // 结果不再重复计失败与换点——取消方已完成，否则一次受阻会连跳两个航点。
  if (result.code == rclcpp_action::ResultCode::CANCELED && goal_cancel_by_mission_) {
    goal_cancel_by_mission_ = false;
    goal_cancel_pending_ = false;   // V0.0.98 取消已静置完成，重发路径可放行
    RCLCPP_DEBUG(get_logger(),
      "[Nav2] 航点[%zu] 的 goal 已按本节点指令取消，失败计数与换点由取消方处理",
      current_wp_idx_);
    return;
  }

  // V0.0.98：主动取消时行为树已处失败态的结果会以【ABORTED（而非 CANCELED）】
  //   回传且世代号匹配（bt_navigator 处理取消请求时对已失败 goal 报 ABORTED）。
  //   这不是新一次失败：不计 fail_count、不换点（取消方已换），仅清静置标记
  //   放行下一个 goal（旧实现在此会把"被取消的旧 goal"计成新航点的失败）。
  if (result.code == rclcpp_action::ResultCode::ABORTED && goal_cancel_by_mission_) {
    goal_cancel_by_mission_ = false;
    goal_cancel_pending_ = false;
    RCLCPP_INFO(get_logger(),
      "[Nav2] 已主动取消的旧 goal 以 ABORTED 回传（取消时 BT 已处失败态），"
      "不计入失败计数；取消静置完成，可发送下一个 goal");
    return;
  }

  // V0.0.96：ABORT 但车辆【其实已经到达】该航点 → 判为完成，不计失败。
  //   现场场景（V0.0.95 实车日志）：车已行驶到航点 A 附近（距目标 0.07m，
  //   已在 xy_goal_tolerance 0.10m 内），但 Nav2 行为树要求先
  //   ComputePathToPose 成功才会执行 FollowPath；车此刻恰好停在静态地图
  //   障碍的膨胀区内 → Smac 持续抛 "Starting point in lethal space!" →
  //   规划失败 → 整树 ABORT（车辆实际已到位）。旧逻辑会把这种"已到达"
  //   计成一次失败，导致 fail_count 迅速达 3 而 FAULT 锁存、任务静默终止。
  //   判据与航点"已到达"预检同源（already_reached_dist_），语义一致。
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED &&
    !waypoints_.empty() && current_wp_idx_ < waypoints_.size())
  {
    double px = 0.0, py = 0.0;
    if (currentMapPose(px, py)) {
      const double d = std::hypot(
        waypoints_[current_wp_idx_].x - px, waypoints_[current_wp_idx_].y - py);
      if (d <= already_reached_dist_) {
        RCLCPP_WARN(get_logger(),
          "[Nav2] 航点[%zu]%s 规划/跟踪失败（%s），但车辆已在其到达半径内"
          "（距目标 %.2fm ≤ %.2fm），判为【已到达】并推进下一航点（不计失败）",
          current_wp_idx_, waypoints_[current_wp_idx_].label.c_str(),
          (result.code == rclcpp_action::ResultCode::ABORTED) ? "ABORTED" : "CANCELED",
          d, already_reached_dist_);
        wp_fail_count_ = 0;
        const size_t next_idx = current_wp_idx_ + 1;
        if (next_idx < waypoints_.size()) {
          current_wp_idx_ = next_idx;
        } else if (loop_waypoints_) {
          RCLCPP_INFO(get_logger(), "[Nav2] 所有航点完成，循环重新开始");
          current_wp_idx_ = 0;
        } else {
          RCLCPP_INFO(get_logger(), "[Nav2] 所有航点完成，停止巡航（loop_waypoints=false）");
          state_ = MissionState::IDLE;
          return;
        }
        if (state_ == MissionState::NAVIGATING) {
          sendNextWaypoint();
        }
        return;
      }
    }
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
    // 重试退避：给 Nav2 恢复/系统稳定留窗口，防止立刻重发
    nav_retry_not_before_ = this->now() + rclcpp::Duration::from_seconds(nav_retry_backoff_);

    if (wp_fail_count_ >= max_wp_failures_) {
      enterFault("航点导航连续 ABORTED/CANCELED 达上限");
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
    case MissionState::FAULT:           return "FAULT";
    default:                            return "UNKNOWN";
  }
}

}  // namespace auto_mission
