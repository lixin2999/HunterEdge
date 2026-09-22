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
//   NAVIGATING ──[单航点失败]──────▶ NAVIGATING（V0.1.00：放弃该点 + 清图重规划 +
//                                        轮转下一个可用航点；连续失败只隔离【该点】）
//   NAVIGATING ──[任务级失败达上限]▶ FAULT（V0.1.00：自愈态，不再是"永久锁存等人工解锁"）
//   FAULT ──[静置 + 自检通过]──────▶ IDLE（自动清图、解除临时隔离并重启任务）
//   FAULT ──[模式离开 AUTO]────────▶ IDLE（保留的人工复位通道：切回 AUTO 即重启）
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

  // ---- V0.1.00 需求②：车身近身"假障碍占位"诊断数据源（均为系统已有话题） ----
  // /local_costmap/costmap：帧=odom 的滚动局部代价地图，反映"Nav2 认为车身四周/
  //   脚底有没有东西"；与激光目标(base_link)、融合目标(base_link，含仅相机确认者)
  //   交叉比对即可自动区分"真障碍"与"脏图假障碍"，不再需要人拿 rviz2 看图。
  // ⚠ QoS 必须与 Nav2Costmap2DPublisher 对齐（transient_local + reliable + KeepLast(1)），
  //   且该 publisher 仅在"有订阅者"时发布、平时只在窗口几何变化时发整图 ——
  //   故 nav2_params.yaml 的 local_costmap 需开 always_send_full_costmap:=true
  //   （V0.1.00 同步改动），否则车辆停驻期间（正是需要诊断的时刻）拿不到刷新。
  {
    const auto grid_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    local_costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      local_costmap_topic_, grid_qos,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lk(data_mutex_);
        latest_local_costmap_ = msg;
        last_local_costmap_arrive_ = this->now();
      });

    // 激光目标：lidar_perception 每帧必发（无目标也发空数组），故"话题新鲜 + 数组为空"
    //   可正面解读为"近身确实没有实体"
    lidar_objects_sub_ = create_subscription<hunter_msgs::msg::DetectedObjectArray>(
      lidar_objects_topic_, rclcpp::SensorDataQoS(),
      [this](const hunter_msgs::msg::DetectedObjectArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(data_mutex_);
        latest_lidar_objects_ = *msg;
        last_lidar_objects_arrive_ = this->now();
      });

    // 相机目标原始帧为 camera_color_optical_frame，本节点不引 TF 依赖做几何换算，
    //   只用其【新鲜度 + 目标数】参与结论描述（相机确认的实体的几何判定走 fused_objects）
    vision_objects_sub_ = create_subscription<hunter_msgs::msg::DetectedObjectArray>(
      vision_objects_topic_, rclcpp::SensorDataQoS(),
      [this](const hunter_msgs::msg::DetectedObjectArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(data_mutex_);
        last_vision_objects_arrive_ = this->now();
        vision_objects_count_ = static_cast<int>(msg->objects.size());
      });
  }

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
    "auto_mission_node 启动：模式=%s，航点数=%zu，循环=%s\n"
    "  V0.1.00 任务自愈：逐点隔离=%s(冷却%.0fs/单点上限%d次/任务上限%d次)，"
    "近身假障碍诊断=%s，地图可达性=%s，FAULT 自愈=%s(静置%.0fs/重试%.0fs~%.0fs)",
    mission_mode_.c_str(), waypoints_.size(), loop_waypoints_ ? "是" : "否",
    "开", wp_isolation_cooldown_, max_wp_failures_, max_consec_failures_,
    self_check_enable_ ? "开" : "关",
    reachability_enable_ ? "开" : "关",
    fault_auto_recover_ ? "开" : "关", fault_hold_time_,
    fault_retry_interval_, fault_retry_max_interval_);
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

  // ================== V0.1.00 任务自愈四件套 ==================
  // ---- 需求①：逐航点失败隔离与自动轮转 ----
  // ⚠ max_wp_failures 语义变更：由"跨航点累计"改为【单个航点连续失败】阈值，
  //   达阈值只隔离该航点（冷却后自动重试），不再锁存整条巡航线。
  declare_parameter("wp_isolation_cooldown", 120.0);
  // 任务级兜底：连续失败期间无任何一次成功到达，达此上限才进 FAULT（自愈态）
  declare_parameter("max_consec_failures", 12);

  // ---- 需求②：车身近身假障碍诊断（代价地图 vs 激光 + 相机） ----
  declare_parameter("self_check_enable", true);
  declare_parameter("self_check_margin", 0.35);
  // 车身框与 nav2_params.yaml 的 local/global_costmap footprint 对齐：
  //   前缘 0.45 / 后缘 0.37 / 半宽 0.32（base_link 系）
  declare_parameter("self_check_box_front", 0.45);
  declare_parameter("self_check_box_rear", 0.37);
  declare_parameter("self_check_box_half_width", 0.32);
  // Nav2 cost_translation_table：0=自由、99=INSCRIBED（内切）、100=LETHAL、-1=UNKNOWN
  declare_parameter("self_check_cost_min", 99);
  declare_parameter("self_check_confirm_count", 3);
  declare_parameter("self_check_interval", 5.0);
  declare_parameter("self_check_data_timeout", 1.0);
  declare_parameter("local_costmap_topic", "/local_costmap/costmap");
  declare_parameter("lidar_objects_topic", "/perception/lidar_objects");
  declare_parameter("vision_objects_topic", "/perception/vision_objects");

  // ---- 需求③：航点地图可达性（静态地图可通行连通域） ----
  declare_parameter("reachability_enable", true);
  declare_parameter("reach_clearance", 0.35);
  declare_parameter("reach_recompute_dist", 1.0);
  declare_parameter("reach_bfs_max_cells", 400000);

  // ---- 需求④：FAULT 由"永久锁存等人工"改为自愈态 ----
  declare_parameter("fault_auto_recover", true);
  declare_parameter("fault_hold_time", 20.0);
  declare_parameter("fault_retry_interval", 30.0);
  declare_parameter("fault_retry_max_interval", 180.0);
  declare_parameter("max_mission_recoveries", 5);

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

  // ---- V0.1.00 参数读取与合法性校验 ----
  // 需求①：逐航点隔离
  wp_isolation_cooldown_ = get_parameter("wp_isolation_cooldown").as_double();
  max_consec_failures_   = static_cast<int>(get_parameter("max_consec_failures").as_int());
  if (max_wp_failures_ < 1) {
    max_wp_failures_ = 1;   // 单点失败 1 次即隔离（不允许多次重复下发同一退化目标）
  }
  if (wp_isolation_cooldown_ < 10.0) {
    RCLCPP_WARN(get_logger(),
      "wp_isolation_cooldown=%.0fs 过小，已按 10s 处理（必须 > 一轮 Nav2 恢复 + 人工反应时间，"
      "否则同一去不了的航点会被反复重试而形成慢循环）", wp_isolation_cooldown_);
    wp_isolation_cooldown_ = 10.0;
  }
  if (max_consec_failures_ < max_wp_failures_ + 2) {
    RCLCPP_WARN(get_logger(),
      "max_consec_failures=%d 不大于单航点上限 max_wp_failures=%d，已按 %d 处理"
      "（任务级兜底必须比单点隔离阈值宽，否则还来不及轮转就进了 FAULT）",
      max_consec_failures_, max_wp_failures_, max_wp_failures_ + 2);
    max_consec_failures_ = max_wp_failures_ + 2;
  }
  // 需求②：近身假障碍诊断
  self_check_enable_        = get_parameter("self_check_enable").as_bool();
  self_check_margin_        = get_parameter("self_check_margin").as_double();
  self_check_box_front_     = get_parameter("self_check_box_front").as_double();
  self_check_box_rear_      = get_parameter("self_check_box_rear").as_double();
  self_check_box_half_width_ = get_parameter("self_check_box_half_width").as_double();
  self_check_cost_min_      = static_cast<int>(get_parameter("self_check_cost_min").as_int());
  self_check_confirm_count_ = static_cast<int>(get_parameter("self_check_confirm_count").as_int());
  self_check_interval_      = get_parameter("self_check_interval").as_double();
  self_check_data_timeout_  = get_parameter("self_check_data_timeout").as_double();
  local_costmap_topic_      = get_parameter("local_costmap_topic").as_string();
  lidar_objects_topic_      = get_parameter("lidar_objects_topic").as_string();
  vision_objects_topic_     = get_parameter("vision_objects_topic").as_string();
  if (self_check_box_front_ <= 0.0 || self_check_box_rear_ <= 0.0 ||
    self_check_box_half_width_ <= 0.0)
  {
    RCLCPP_WARN(get_logger(),
      "self_check_box_* 存在非正值（front=%.2f rear=%.2f half_width=%.2f），已按默认车身尺"
      "寸 0.45/0.37/0.32m 处理（必须与 nav2 footprint 一致，否则近身判据整条失效）",
      self_check_box_front_, self_check_box_rear_, self_check_box_half_width_);
    self_check_box_front_ = 0.45;
    self_check_box_rear_ = 0.37;
    self_check_box_half_width_ = 0.32;
  }
  if (self_check_margin_ < 0.0) {
    self_check_margin_ = 0.0;   // 0 = 只查“脚底”（原始车身框内）
  }
  self_check_cost_min_ = std::max(1, std::min(100, self_check_cost_min_));
  self_check_confirm_count_ = std::max(1, self_check_confirm_count_);
  if (self_check_interval_ < 1.0) {
    self_check_interval_ = 1.0;
  }
  if (self_check_data_timeout_ < 0.3) {
    self_check_data_timeout_ = 0.3;
  }
  // 需求③：地图可达性
  reachability_enable_     = get_parameter("reachability_enable").as_bool();
  reach_clearance_         = get_parameter("reach_clearance").as_double();
  reach_recompute_dist_    = get_parameter("reach_recompute_dist").as_double();
  reach_bfs_max_cells_     = static_cast<int>(get_parameter("reach_bfs_max_cells").as_int());
  if (reach_clearance_ < 0.32) {
    RCLCPP_WARN(get_logger(),
      "reach_clearance=%.2fm 小于 Nav2 内切半径 0.32m，已按 0.35m 处理"
      "（过小会把“实际过不去的窄缝”判成连通，可达性校验失去意义）", reach_clearance_);
    reach_clearance_ = 0.35;
  }
  if (reach_recompute_dist_ < 0.2) {
    reach_recompute_dist_ = 0.2;   // 节流：车位移动不足此距离不重建连通域
  }
  if (reach_bfs_max_cells_ < 10000) {
    RCLCPP_WARN(get_logger(),
      "reach_bfs_max_cells=%d 过小，已按 10000 格处理（BFS 被截断时不可达结论一律放行）",
      reach_bfs_max_cells_);
    reach_bfs_max_cells_ = 10000;
  }
  // 需求④：FAULT 自愈
  fault_auto_recover_      = get_parameter("fault_auto_recover").as_bool();
  fault_hold_time_         = get_parameter("fault_hold_time").as_double();
  fault_retry_interval_    = get_parameter("fault_retry_interval").as_double();
  fault_retry_max_interval_ = get_parameter("fault_retry_max_interval").as_double();
  max_mission_recoveries_  = static_cast<int>(get_parameter("max_mission_recoveries").as_int());
  if (fault_hold_time_ < 5.0) {
    RCLCPP_WARN(get_logger(),
      "fault_hold_time=%.0fs 过短（<5s，来不及移车/障碍离开），已按 5.0s 处理",
      fault_hold_time_);
    fault_hold_time_ = 5.0;
  }
  if (fault_retry_interval_ < 5.0) {
    fault_retry_interval_ = 5.0;
  }
  if (fault_retry_max_interval_ < fault_retry_interval_) {
    fault_retry_max_interval_ = fault_retry_interval_;
  }
  if (max_mission_recoveries_ < 1) {
    max_mission_recoveries_ = 1;
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
  syncWaypointRuntime();   // V0.1.00：逐航点运行时状态与航点表同尺寸
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
  syncWaypointRuntime();   // V0.1.00：新增航点以"零失败"入列，删除的裁掉尾部
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
  // V0.1.00：/localization/odom 的位姿在 odom 系（ekf_params.world_frame=odom），
  //   用于把 odom 系的局部代价地图格元换算回 base_link 做近身占位判定
  last_odom_arrive_ = this->now();
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

  // ---------- V0.1.00 需求④：模式边沿复位（遥控接管后切回 AUTO 即自动复驶） ----------
  // 下降沿（离开 AUTO：遥控接管 / 切 ESTOP / 急停）→ 全量复位任务状态，含清除
  //   【地图不可达】永久隔离（人工可能已移车 / 重标航点 / 换了地图）；
  // 上升沿（切回 AUTO）无需特殊动作：state_ 已是 IDLE，IDLE 分支按 AUTO 条件
  //   自动清图、进 WAITING_LOCALIZE/NAVIGATING 重启任务 —— 人工不再需要
  //   "离开 AUTO 再切回"来解故障锁存（该通道保留为即时复位手段）。
  // ESTOP 不在此复位：急停信号及其解除由上方/下方专属通道负责，避免抹除急停语义。
  const bool mode_auto = (behavior.mode == "AUTO");
  if (prev_mode_auto_ && !mode_auto && state_ != MissionState::ESTOP) {
    resetMissionState("模式开关离开 AUTO（遥控接管 / 降级）", true);
  }
  prev_mode_auto_ = mode_auto;

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
    // FAULT 不因 health 恢复而被默默清除：CRITICAL 期间不做自愈重试（下方
    // faultRecoveryStep 的 AUTO 条件门控已含 health 非 CRITICAL），
    // 待 health 恢复后由自愈流程自动复驶，或人工离开 AUTO 即时复位
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
        // V0.1.00：任务启动即复位失败计数与临时隔离（保留地图不可达的永久判定）
        consec_fail_ = 0;
        goal_reject_count_ = 0;
        for (auto & rt : wp_rt_) {
          rt.fail_count = 0;
          rt.isolated = rt.permanent;
        }
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
        consec_fail_ = 0;
        goal_reject_count_ = 0;
        for (auto & rt : wp_rt_) {
          rt.fail_count = 0;
          rt.isolated = rt.permanent;
        }
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
            // V0.1.00 需求①：不再"整条任务锁存"——只放弃当前航点（达阈值则仅隔离
            // 该点）+ 清图重规划 + 轮转下一个可用航点；任务级兜底交给 consec_fail_
            handleWaypointFailure(
              "航点受阻（" + std::to_string(static_cast<int>(stall_detect_time_)) +
              "s 内净推进不足：前方障碍无法绕行 / 车已停在障碍膨胀区致规划全失败）");
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
            "[NAVIGATING] 航点[%zu]%s 导航超时（%.0fs），放弃该航点并重新规划",
            current_wp_idx_, waypoints_[current_wp_idx_].label.c_str(), goal_timeout_);
          cancelCurrentGoal();
          // V0.1.00 需求①（用户日志对应的就是本分支）：超时不再锁存整条任务
          handleWaypointFailure(
            "单航点导航超时（>" + std::to_string(static_cast<int>(goal_timeout_)) + "s）");
          break;
        }
      }

      // ---------- V0.1.00 需求②：近身假障碍周期巡检（NAVIGATING 中） ----------
      // 背景（实车日志）：车停在脏图留下的假障碍上时，全局规划持续报
      //   "Starting point in lethal space!" → 行为树清图→重试→再失败，车原地磨到
      //   超时。本巡检把"人工用 rviz2 看图"前置为自动判定：代价地图标了车身
      //   四周/脚底占位，而激光与相机均无实体 ⇒ 判为假障碍，主动清图重规划。
      if (self_check_enable_ &&
        (this->now() - last_self_check_).seconds() >= self_check_interval_)
      {
        last_self_check_ = this->now();
        const SurroundReport rep = diagnoseSelfSurroundings();
        if (rep.phantom_confirmed) {
          RCLCPP_ERROR(get_logger(),
            "[近身自检] 车身四周/脚底被代价地图标为占位（%d 格，最近距车身框 %.2fm），"
            "但激光与相机在近身范围内均无实体 ⇒ 判定为【假障碍占位】，立即清图并重新规划：%s",
            rep.occupied_cells, rep.nearest_cost_dist, rep.detail.c_str());
          requestCostmapClear("近身自检判为假障碍占位");
          // 清图后若本 goal 仍卡在原地，由 stall / goal_timeout 路径换点（已不再锁存）
        } else if (rep.body_occupied && rep.valid) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
            "[近身自检] 车身近身占位且有传感器实体（真障碍/车体自反射）：不属假障碍，"
            "需遥控移车或等人/物离开——%s", rep.detail.c_str());
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
      // V0.1.00：FAULT 由"永久锁存 + 人工切开关解锁"改为【自愈态】——
      //   静置 fault_hold_time 后周期性做：近身占位诊断 + 清图 + 航点可达性复判 +
      //   AUTO 条件确认，条件满足即自动解除临时隔离并重启任务（自愈失败则按
      //   fault_retry_interval × 轮次递增至 fault_retry_max_interval 重试，不永久锁死）。
      faultRecoveryStep();
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
  // V0.1.00：地图修订号自增 → 距离场/连通域缓存自动重建，之前基于旧图得出的
  //   "永久隔离（不可达）"结论在新图下会重新判定（换图/重定位后无需人工解锁）
  ++map_revision_;
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

  // ---- 航点预检（V0.1.00：三级筛选，逐点隔离而不是一键锁存整条任务） ----
  // ① 隔离态：临时隔离（导航失败累计）在冷却期满 wp_isolation_cooldown 后自动
  //    重判并重试；永久隔离（地图判定不可达）同样按冷却周期复判，因而
  //    移车/换图/重标航点后无需人工解锁；
  // ② 地图校验（矩形界 / unknown / 占据 / 净空 / **可通行连通域**，需求③）：
  //    这类失败与当次导航运气无关，属客观不可达 → 直接**永久隔离**，不再像
  //    旧版那样逐圈重复下发、重复失败；
  // ③ 已到达（车与航点重合，V0.0.95）：视为已完成并跳过（不计失败）。
  // 三级筛完整圈无可用航点 → enterFault（V0.1.00 起为自愈态：自动诊断 + 周期重试）。
  size_t precheck_skips = 0;
  while (precheck_skips < waypoints_.size()) {
    const size_t idx = current_wp_idx_;
    const Waypoint & cand = waypoints_[idx];

    // ① 隔离态筛选（冷却到期自动复判/解除）
    promoteExpiredIsolation(idx);
    if (idx < wp_rt_.size() && wp_rt_[idx].isolated) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "[sendNextWaypoint] 航点[%zu]%s 隔离中（%s），暂不下发，轮转下一个航点",
        idx, cand.label.c_str(), wp_rt_[idx].last_reason.empty() ? "未记录" : "见隔离日志");
      current_wp_idx_ = (current_wp_idx_ + 1) % waypoints_.size();
      ++precheck_skips;
      continue;
    }

    // ②-a 地图矩形边界 / 未建图 / 占据栅格 / 净空校验（V0.0.82/0.0.87/0.0.96）
    const std::string map_check = waypointMapCheckDetail(cand);
    if (!map_check.empty()) {
      RCLCPP_ERROR(get_logger(),
        "[sendNextWaypoint] 航点[%zu]%s (%.2f, %.2f) 地图校验不通过（%s；"
        "边界 x[%.2f, %.2f] y[%.2f, %.2f]，安全边距 %.2fm）→ 永久隔离该航点",
        idx, cand.label.c_str(), cand.x, cand.y, map_check.c_str(),
        map_min_x_, map_max_x_, map_min_y_, map_max_y_, waypoint_map_margin_);
      isolateWaypoint(idx, "地图校验：" + map_check, true);
      current_wp_idx_ = (current_wp_idx_ + 1) % waypoints_.size();
      ++precheck_skips;
      continue;
    }

    // ②-b 地图可达性（需求③）：在静态地图上做可通行连通域搜索，判定“从当前
    //     车位能否沿可通行区域抵达该航点”——不可达的航点不该反复去试
    const std::string reach_check = waypointReachabilityDetail(cand.x, cand.y);
    if (!reach_check.empty()) {
      RCLCPP_ERROR(get_logger(),
        "[sendNextWaypoint] 航点[%zu]%s (%.2f, %.2f) 按地图判定不可达：%s → 永久隔离该航点",
        idx, cand.label.c_str(), cand.x, cand.y, reach_check.c_str());
      isolateWaypoint(idx, "地图可达性：" + reach_check, true);
      current_wp_idx_ = (current_wp_idx_ + 1) % waypoints_.size();
      ++precheck_skips;
      continue;
    }

    // ③ 已到达预检
    double reached_d = 0.0, reached_yaw = 0.0;
    if (waypointAlreadyReached(cand, reached_d, reached_yaw)) {
      RCLCPP_INFO(get_logger(),
        "[sendNextWaypoint] 航点[%zu]%s (%.2f, %.2f) 已在到达半径内"
        "（距当前位姿 %.2fm ≤ %.2fm，朝向差 %.2f rad），视为已完成并跳过",
        idx, cand.label.c_str(), cand.x, cand.y,
        reached_d, already_reached_dist_, reached_yaw);
      current_wp_idx_ = (current_wp_idx_ + 1) % waypoints_.size();
      ++precheck_skips;
      continue;
    }
    break;
  }
  if (precheck_skips >= waypoints_.size()) {
    enterFault(
      "全部航点均不可用：或已隔离（导航失败累计/地图不可达），或已在到达半径内，"
      "或位于未建图/地图外区域");
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
// 任务故障停驻（V0.0.91 引入；V0.1.00 起为【自愈态】入口）
//
// 现场教训一（V0.0.91）：旧实现达失败上限时回到 IDLE，而 IDLE 分支下一拍就发现
//   "AUTO 条件仍满足"→ 重置计数并从航点[0] 重发同一批不可规划目标，形成
//   IDLE⇄NAVIGATING 静默抖动：车不动、无对外故障上报、旁人无法从日志判断已放弃。
//   故改为锁存到 FAULT。
// 现场教训二（V0.1.00，本次修正）：锁存只能靠"模式开关离开 AUTO 再切回"人工解除，
//   而日志里那三类成因（近身假障碍占位 / 定位跳变 / 航点不可达）本节点完全有能力
//   自己判定并处理。因此 FAULT 不再是一个终态，而是"停车 + 自动诊断 + 周期重试"
//   的自愈态：静置 fault_hold_time 后由 faultRecoveryStep() 接手（见该函数）。
//   人工通道保留：离开 AUTO 再切回仍是即时全量复位。
// 注：不发 /estop —— 车已停且非危险场景，保持急停通道语义纯净。
// ==========================================================================
void AutoMissionNode::enterFault(const std::string & reason)
{
  const bool reentry = (state_ == MissionState::FAULT);
  if (!reentry) {
    ++mission_recovery_round_;
  }
  fault_reason_ = reason;
  cancelCurrentGoal();

  // 进入 FAULT 即刻做一次自动诊断，把结论直接写给现场（需求②③）：
  //   旧版本让人拿 rviz2 看图、猜"是假障碍还是定位跳变还是航点不可达"，
  //   现在由本节点用代价地图 + 激光 + 相机 + 静态地图连通域给出可判读结论。
  const SurroundReport rep = diagnoseSelfSurroundings();
  size_t n_temp = 0, n_perm = 0;
  for (size_t i = 0; i < wp_rt_.size(); ++i) {
    if (!wp_rt_[i].isolated) {
      continue;
    }
    if (wp_rt_[i].permanent) {
      ++n_perm;
    } else {
      ++n_temp;
    }
  }
  std::ostringstream oss;
  oss << "近身占位诊断：";
  if (!rep.valid) {
    oss << "数据不足（代价地图/里程计/激光目标超时），本次不可判定";
  } else if (!rep.body_occupied) {
    oss << "车身四周/脚底无代价地图占位（非假障碍问题）";
  } else if (rep.lidar_present || rep.fused_present) {
    oss << "车身近身确有实体（激光/相机可见）⇒ 真障碍或车体自反射，需移车或清障";
  } else {
    oss << "代价地图占位而激光与相机均无实体 ⇒ 判定为【假障碍占位】，清图即可恢复";
  }
  oss << "｜" << rep.detail
      << "｜航点：共 " << waypoints_.size()
      << " 个，临时隔离 " << n_temp << " 个（冷却 "
      << static_cast<int>(wp_isolation_cooldown_) << "s 后自动重试），永久隔离 " << n_perm
      << " 个（地图判定不可达，换地图/移车后按冷却周期复判）";
  fault_diagnosis_ = oss.str();

  fault_enter_time_ = this->now();
  fault_next_check_ = fault_enter_time_ + rclcpp::Duration::from_seconds(fault_hold_time_);
  consec_fail_ = 0;   // 计数归零，但停驻不因此解除（靠状态而非计数拦截重发）

  RCLCPP_ERROR(get_logger(),
    "[FAULT] %s\n  → 第 %d 轮自愈，%.0fs 后自动复检（无需人工解锁）\n  → 自动诊断：%s\n"
    "  → 自愈条件：AUTO 条件满足 + bt_navigator ACTIVE + 车身近身非【真实障碍】包围 + "
    "至少 1 个按地图可达的航点；通过则自动清图、解除临时隔离并重启任务\n"
    "  → 人工通道（仍可用）：遥控接管后将模式开关离开 AUTO 再切回 = 立即全量复位",
    reason.c_str(), mission_recovery_round_, fault_hold_time_, fault_diagnosis_.c_str());

  state_ = MissionState::FAULT;
}

// ==========================================================================
// V0.1.00 需求④：FAULT 自愈一步（由 mainLoop 在 FAULT 分支每 100ms 调用）
//
// 与旧"永久锁存"的区别：把人工处置清单里的三项（查假障碍、查定位、查航点可达）
// 变成周期性自动执行的检查，并在条件满足时自行重启任务；条件不满足则按
// fault_retry_interval × 轮次（上限 fault_retry_max_interval）放慢频率并持续
// 输出可判读诊断，绝不永久锁死。
// ==========================================================================
void AutoMissionNode::faultRecoveryStep()
{
  const rclcpp::Time now = this->now();

  if (!fault_auto_recover_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "[FAULT] 自愈已禁用（fault_auto_recover=false）：%s；人工处置：将模式开关离开 AUTO 再切回",
      fault_reason_.empty() ? "未记录原因" : fault_reason_.c_str());
    return;
  }

  const double remain = (fault_next_check_ - now).seconds();
  if (remain > 0.0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "[FAULT] 任务停驻中（%s），%.0fs 后自动复检｜%s",
      fault_reason_.empty() ? "未记录原因" : fault_reason_.c_str(),
      remain, fault_diagnosis_.empty() ? "诊断待生成" : fault_diagnosis_.c_str());
    return;
  }

  // 先推进下一次检查时刻（即使本次检查失败也不会 10Hz 刷屏）
  const double backoff = std::min(
    fault_retry_interval_ * static_cast<double>(std::max(1, mission_recovery_round_)),
    fault_retry_max_interval_);
  fault_next_check_ = now + rclcpp::Duration::from_seconds(backoff);

  if (!nav_active_.load()) {
    queryNavigatorState();   // 刷新 bt_navigator 激活态（Nav2 重启过的场景）
  }

  // ① 近身占位诊断（需求②）：假障碍 → 清图；真障碍 → 不具备自愈条件
  const SurroundReport rep = diagnoseSelfSurroundings();
  const bool phantom_like =
    rep.valid && rep.body_occupied && !rep.lidar_present && !rep.fused_present;
  if (phantom_like) {
    RCLCPP_ERROR(get_logger(),
      "[FAULT] 自愈检查：%s ⇒ 判为【假障碍占位】，先清图再复检：%s",
      "车身近身被代价地图占位而激光/相机无实体", rep.detail.c_str());
    requestCostmapClear("FAULT 自愈：近身假障碍占位");
  } else if (rep.valid && !rep.body_occupied) {
    // 近身干净：若服务仍就绪再清一次（全图残留同样会让起点落入致命栅格）
    requestCostmapClear("FAULT 自愈：例行清图后重规划");
  }

  // ② 航点复判（需求①③）：隔离冷却到期者重新纳入；地图不可读者转永久隔离
  int available = 0;
  int first_avail = -1;
  for (size_t i = 0; i < waypoints_.size(); ++i) {
    promoteExpiredIsolation(i);
    if (i < wp_rt_.size() && wp_rt_[i].isolated) {
      continue;
    }
    const std::string mc = waypointMapCheckDetail(waypoints_[i]);
    if (!mc.empty()) {
      isolateWaypoint(i, "地图校验：" + mc, true);
      continue;
    }
    const std::string rr = waypointReachabilityDetail(waypoints_[i].x, waypoints_[i].y);
    if (!rr.empty()) {
      isolateWaypoint(i, "地图可达性：" + rr, true);
      continue;
    }
    if (first_avail < 0) {
      first_avail = static_cast<int>(i);
    }
    ++available;
  }

  // ③ 自愈门控
  const bool stack_ok = isAutoConditionMet() && nav_active_.load();
  const bool body_really_blocked =
    rep.valid && rep.body_occupied && (rep.lidar_present || rep.fused_present);
  RCLCPP_WARN(get_logger(),
    "[FAULT] 自愈检查（第 %d 轮）：AUTO/Nav2就绪=%s，可用航点=%d/%zu，近身真障碍=%s，"
    "近身假障碍=%s｜%s",
    mission_recovery_round_, stack_ok ? "是" : "否", available, waypoints_.size(),
    body_really_blocked ? "是" : "否", phantom_like ? "是（已清图）" : "否",
    rep.detail.c_str());

  if (!stack_ok || available <= 0 || body_really_blocked) {
    std::ostringstream why;
    if (!stack_ok) {
      why << "AUTO 条件或 bt_navigator 未就绪（查定位收敛/感知新鲜度/health/Nav2 激活）；";
    }
    if (available <= 0) {
      why << "无可用航点（全部被隔离：请核实地图/重标航点或人工移车）；";
    }
    if (body_really_blocked) {
      why << "车身近身确有实体（激光/相机可见），需遥控移车或等障碍离开；";
    }
    RCLCPP_ERROR(get_logger(),
      "[FAULT] 自愈未通过：%s%.0fs 后重试%s",
      why.str().c_str(), backoff,
      mission_recovery_round_ > max_mission_recoveries_ ?
        "（已超过预期自愈轮次，建议人工介入：遥控移车 1m 以上→切回 AUTO）" : "");
    return;
  }

  // ④ 自愈成功：解除临时隔离（保留地图不可达判定）并回 IDLE，由 IDLE 自动重启任务
  RCLCPP_ERROR(get_logger(),
    "[FAULT→IDLE] 自愈成功（第 %d 轮）：近身无真实障碍、AUTO 条件与 Nav2 就绪、"
    "%d 个航点可用（首个：航点[%d]%s）→ 自动清图并重启自主导航",
    mission_recovery_round_, available, first_avail,
    (first_avail >= 0 && static_cast<size_t>(first_avail) < waypoints_.size()) ?
      waypoints_[static_cast<size_t>(first_avail)].label.c_str() : "-");
  resetMissionState("FAULT 自愈成功（自动复驶）", false);
}

// ==========================================================================
// V0.1.00 需求①：使 wp_rt_ 与 waypoints_ 同尺寸（加载 / 热重载后调用）
// ==========================================================================
void AutoMissionNode::syncWaypointRuntime()
{
  if (wp_rt_.size() > waypoints_.size()) {
    wp_rt_.resize(waypoints_.size());     // 航点减少：裁掉尾部运行时状态
  }
  while (wp_rt_.size() < waypoints_.size()) {
    wp_rt_.emplace_back();               // 航点增加：新点以"零失败"入列
  }
  if (current_wp_idx_ >= waypoints_.size()) {
    current_wp_idx_ = 0;
  }
}

// ==========================================================================
// V0.1.00 统一清图入口（需求①"重新规划导航"的具体动作）
// 复用 V0.0.92/0.0.94 的清图 + 挂起重试机制：服务未就绪时置
// costmaps_clear_pending_，由 NAVIGATING 入口每 100ms 重试，清图完成前不发新 goal
// ==========================================================================
void AutoMissionNode::requestCostmapClear(const std::string & why)
{
  clearCostmapsOnStart();
  RCLCPP_INFO(get_logger(), "[清图] %s：已请求清除全局/局部代价地图（重新规划前置动作）",
    why.c_str());
}

// ==========================================================================
// V0.1.00 需求①：隔离单个航点
//   permanent=false：临时隔离（导航失败累计），冷却期满自动重试
//   permanent=true ：永久隔离（地图校验/可达性不通过），按冷却周期复判
// ==========================================================================
void AutoMissionNode::isolateWaypoint(size_t idx, const std::string & reason, bool permanent)
{
  if (idx >= waypoints_.size() || idx >= wp_rt_.size()) {
    return;
  }
  WaypointRuntime & rt = wp_rt_[idx];
  const bool was_isolated = rt.isolated;
  rt.isolated = true;
  rt.permanent = rt.permanent || permanent;   // 永久判定不因后续临时失败而降级
  rt.isolated_at = this->now();
  rt.last_reason = reason;

  if (rt.permanent) {
    if (was_isolated && permanent) {
      return;   // 已报告过，不刷日志（复判时每轮只记一次）
    }
    RCLCPP_ERROR(get_logger(),
      "[航点隔离] 航点[%zu]%s (%.2f, %.2f) 永久隔离：%s"
      "（与当次导航运气无关；移车/换图/重标航点后按 %.0fs 冷却周期自动复判）",
      idx, waypoints_[idx].label.c_str(), waypoints_[idx].x, waypoints_[idx].y,
      reason.c_str(), wp_isolation_cooldown_);
  } else if (!was_isolated) {
    RCLCPP_ERROR(get_logger(),
      "[航点隔离] 航点[%zu]%s 连续失败 %d/%d 次 → 隔离 %.0fs，期间轮转其他航点（%s）",
      idx, waypoints_[idx].label.c_str(), rt.fail_count, max_wp_failures_,
      wp_isolation_cooldown_, reason.c_str());
  }
}

// ==========================================================================
// V0.1.00 需求①③：隔离冷却到期时的自动复判
//   临时隔离 → 直接解除（给航点一次全新的机会）；
//   永久隔离 → 重跑地图校验 + 可达性：通过则解除（现场移车/重定位/换图后
//   自然恢复，无需人工解锁），仍不通过则续一个冷却周期。
// ==========================================================================
void AutoMissionNode::promoteExpiredIsolation(size_t idx)
{
  if (idx >= wp_rt_.size() || idx >= waypoints_.size()) {
    return;
  }
  WaypointRuntime & rt = wp_rt_[idx];
  if (!rt.isolated) {
    return;
  }
  if ((this->now() - rt.isolated_at).seconds() < wp_isolation_cooldown_) {
    return;
  }
  const std::string label = waypoints_[idx].label;

  if (!rt.permanent) {
    rt.isolated = false;
    rt.fail_count = 0;
    RCLCPP_INFO(get_logger(),
      "[航点隔离] 航点[%zu]%s 隔离冷却到期（%.0fs）→ 解除隔离，重新纳入巡航（上次原因：%s）",
      idx, label.c_str(), wp_isolation_cooldown_, rt.last_reason.c_str());
    return;
  }

  // 永久隔离：按地图事实复判
  const std::string mc = waypointMapCheckDetail(waypoints_[idx]);
  if (mc.empty()) {
    const std::string rr = waypointReachabilityDetail(waypoints_[idx].x, waypoints_[idx].y);
    if (rr.empty()) {
      rt.isolated = false;
      rt.permanent = false;
      rt.fail_count = 0;
      rt.last_reason.clear();
      RCLCPP_INFO(get_logger(),
        "[航点隔离] 航点[%zu]%s 永久隔离复判通过（地图已可通行）→ 解除隔离并重新纳入巡航",
        idx, label.c_str());
      return;
    }
    rt.isolated_at = this->now();   // 仍不可达 → 续一个冷却周期
    rt.last_reason = "地图可达性：" + rr;
    return;
  }
  rt.isolated_at = this->now();
  rt.last_reason = "地图校验：" + mc;
}

// ==========================================================================
// V0.1.00 需求①：轮转到下一个可用航点（false = 全部不可用）
// ==========================================================================
bool AutoMissionNode::advanceToNextAvailableWaypoint()
{
  if (waypoints_.empty()) {
    return false;
  }
  const size_t n = waypoints_.size();
  for (size_t step = 1; step <= n; ++step) {
    const size_t idx = (current_wp_idx_ + step) % n;
    promoteExpiredIsolation(idx);
    if (idx < wp_rt_.size() && wp_rt_[idx].isolated) {
      continue;
    }
    current_wp_idx_ = idx;
    return true;
  }
  return false;
}

// ==========================================================================
// V0.1.00 需求①：单航点失败的标准处置（代替旧的"整条任务锁存"）
//   放弃当前航点 → 逐点计数（达阈值仅隔离该点）→ 近身假障碍诊断 → 清图重规划
//   → 轮转下一个可用航点（重发由主循环经"取消静置门控"放行，不在此直发）
// ⚠ 调用方必须已释放 goal_handle_mutex_（本函数会间接调用 cancel/clear/诊断，
//   而诊断与清图均在主线程串行执行；历史上的 std::mutex 自死锁就出在这里）
// ==========================================================================
void AutoMissionNode::handleWaypointFailure(const std::string & reason)
{
  if (waypoints_.empty()) {
    enterFault("无航点可执行（waypoints 未配置或全部解析失败）");
    return;
  }
  const size_t idx = current_wp_idx_ % waypoints_.size();
  int fail_count = max_wp_failures_;   // wp_rt_ 异常时按"已达上限"保守处理
  if (idx < wp_rt_.size()) {
    fail_count = ++(wp_rt_[idx].fail_count);
    wp_rt_[idx].last_reason = reason;
  }
  ++consec_fail_;
  goal_reject_count_ = 0;

  const SurroundReport rep = diagnoseSelfSurroundings();   // 需求②：失败当下即定性
  RCLCPP_ERROR(get_logger(),
    "[航点失败] 航点[%zu]%s 第 %d/%d 次失败（%s）｜任务级连续失败 %d/%d｜自动诊断：%s",
    idx, waypoints_[idx].label.c_str(), fail_count, max_wp_failures_,
    reason.c_str(), consec_fail_, max_consec_failures_, rep.detail.c_str());

  // ① 该点达上限 → 只隔离这一个航点
  if (fail_count >= max_wp_failures_) {
    isolateWaypoint(idx, reason, false);
  }
  // ② 需求①的"重新规划导航"：清两张代价地图（脏图/假膨胀是重规划失败的头号成因）
  requestCostmapClear("航点失败后重新规划");
  // ③ 任务级兜底：期间一次都没成功过且累计失败过多 → 进 FAULT（自愈态）
  if (consec_fail_ >= max_consec_failures_) {
    enterFault("任务级连续失败达上限（" + std::to_string(consec_fail_) + " 次）：" + reason);
    return;
  }
  // ④ 放弃当前航点，轮转到下一个可用航点
  if (!advanceToNextAvailableWaypoint()) {
    enterFault("全部航点均已隔离（临时/永久），无可用目标");
    return;
  }
  RCLCPP_WARN(get_logger(),
    "[航点失败] 已放弃航点[%zu]，改发航点[%zu]%s（清图完成后由主循环重新下发并重新规划）",
    idx, current_wp_idx_, waypoints_[current_wp_idx_].label.c_str());
}

// ==========================================================================
// V0.1.00 需求④：任务状态全量复位
//   clear_permanent=true ：连"地图不可达"判定一并清除（离开 AUTO 的人工复位，
//                         因为人工可能已移车 / 重标航点 / 换了地图）；
//   clear_permanent=false：仅解除临时隔离（自愈成功重启，客观不可达判定仍保留，
//                         避免重启后立刻又去碰同一个不可达点）。
// ==========================================================================
void AutoMissionNode::resetMissionState(const std::string & why, bool clear_permanent)
{
  cancelCurrentGoal();
  size_t n_iso = 0, n_perm = 0;
  for (auto & rt : wp_rt_) {
    if (rt.isolated) {
      ++n_iso;
    }
    if (rt.permanent) {
      ++n_perm;
    }
    rt.fail_count = 0;
    if (rt.permanent && !clear_permanent) {
      rt.isolated = true;      // 保留地图不可达判定
      continue;
    }
    rt.isolated = false;
    rt.permanent = false;
    rt.last_reason.clear();
  }
  consec_fail_ = 0;
  goal_reject_count_ = 0;
  blocked_count_ = 0;
  phantom_confirm_ = 0;
  mission_recovery_round_ = 0;
  current_wp_idx_ = 0;
  goal_start_dist_ = -1.0;
  goal_path_len_ = 0.0;
  has_last_stall_pose_ = false;
  goal_cancel_pending_ = false;
  goal_cancel_by_mission_ = false;
  // V0.1.00：自增 goal 世代号——上面 cancelCurrentGoal() 清掉了主动取消标记，
  //   若不抬高世代号，旧 goal 的 CANCELED/ABORTED 结果会在复位后新任务已起
  //   （state_=NAVIGATING）时被当成一次真失败；抬高后旧结果一律按过期丢弃。
  ++goal_epoch_;
  costmaps_clear_pending_.store(false);
  localize_wait_started_ = false;
  obstacle_wait_started_ = false;
  nav_wait_started_ = false;
  nav_retry_not_before_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  fault_reason_.clear();
  fault_diagnosis_.clear();
  state_ = MissionState::IDLE;

  RCLCPP_INFO(get_logger(),
    "[任务复位] %s：%s｜原有隔离 %zu 个（其中永久 %zu 个）已%s，"
    "回到 IDLE，AUTO 条件满足即自动重启任务",
    clear_permanent ? "全量（含地图不可达判定）" : "轻量（保留地图不可达判定）",
    why.c_str(), n_iso, n_perm,
    clear_permanent ? "全部清除" : "临时隔离已解除");
}

// ==========================================================================
// V0.1.00 需求②：车身近身"假障碍占位"诊断
//
// 判据（三路信息交叉）：
//   A 代价地图（/local_costmap/costmap，帧=odom）——车身框（含外扩边距）内有
//     内切/致命格（cost ≥ self_check_cost_min）⇒ "Nav2 认为近身有东西"；
//   B 激光目标（/perception/lidar_objects，base_link）与融合目标
//     （/perception/fused_objects，base_link，含仅由相机确认的目标）——近身有实体
//     ⇒ "传感器确实看到东西"；相机原始帧为 camera_color_optical_frame，本函数不做
//     TF 换算，只以新鲜度/目标数参与结论描述；
//   ⇒ A 成立而 B 不成立 ⇒ 【假障碍】（脏图、自反射、接管期压过旧栅格等）。
// 防抖：连判 self_check_confirm_count 次才确认（单帧噪声不触发清图）。
// odom→base_link 换算用 /localization/odom（EKF，world_frame=odom）的位姿做逆旋转，
//   不引入 TF 依赖；若地图帧为 map/base_link 则自动改选对应位姿源（容错配置差异）。
// ==========================================================================
AutoMissionNode::SurroundReport AutoMissionNode::diagnoseSelfSurroundings()
{
  SurroundReport rep;
  if (!self_check_enable_) {
    rep.detail = "近身自检已禁用（self_check_enable=false）";
    return rep;   // valid=false：调用方按"不可判定"处理
  }

  nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid;
  nav_msgs::msg::Odometry odom;
  hunter_msgs::msg::DetectedObjectArray lidar_objs;
  hunter_msgs::msg::DetectedObjectArray fused_objs;
  geometry_msgs::msg::PoseWithCovarianceStamped reloc;
  rclcpp::Time t_grid(0, 0, RCL_ROS_TIME), t_odom(0, 0, RCL_ROS_TIME);
  rclcpp::Time t_lidar(0, 0, RCL_ROS_TIME), t_vision(0, 0, RCL_ROS_TIME);
  int vision_count = 0;
  bool reloc_ok = false;
  {
    std::lock_guard<std::mutex> lk(data_mutex_);
    grid       = latest_local_costmap_;
    t_grid     = last_local_costmap_arrive_;
    odom       = latest_odom_;
    t_odom     = last_odom_arrive_;
    lidar_objs = latest_lidar_objects_;
    t_lidar    = last_lidar_objects_arrive_;
    fused_objs = latest_fused_objects_;
    t_vision   = last_vision_objects_arrive_;
    vision_count = vision_objects_count_;
    reloc      = latest_amcl_pose_;
    reloc_ok   = amcl_pose_received_;
  }
  // 融合目标自身的新鲜度（与 isPerceptionAlive() 同源）
  const rclcpp::Time now = this->now();
  const double fused_age = (now - last_perception_stamp_).seconds();

  const auto age_of = [&now](const rclcpp::Time & t) {
    return (t.seconds() <= 0.0) ? 1e9 : (now - t).seconds();
  };
  const double grid_age  = age_of(t_grid);
  const double odom_age  = age_of(t_odom);
  const double lidar_age = age_of(t_lidar);
  const double vision_age = age_of(t_vision);

  const bool grid_ok  = (grid != nullptr) && grid_age <= self_check_data_timeout_;
  const bool lidar_ok = lidar_age <= std::max(self_check_data_timeout_, 2.0);
  const bool fused_ok = fused_age <= std::max(self_check_data_timeout_, 2.0);
  rep.camera_alive = vision_age <= std::max(self_check_data_timeout_, 2.0);
  // 可判定条件：代价地图新鲜 + 至少一路目标（激光或融合）新鲜
  rep.valid = grid_ok && (lidar_ok || fused_ok);

  // ---- 车身框（base_link）与代价地图帧→base_link 的位姿源选择 ----
  const double m = self_check_margin_;
  const double bx0 = -self_check_box_rear_, bx1 = self_check_box_front_;
  const double by0 = -self_check_box_half_width_, by1 = self_check_box_half_width_;
  double xr = 0.0, yr = 0.0, yawr = 0.0;
  std::string pose_src = "odom";
  if (grid_ok) {
    std::string gf = grid->header.frame_id;
    if (!gf.empty() && gf.front() == '/') {
      gf.erase(0, 1);
    }
    if (gf == "base_link" || gf.empty()) {
      xr = yr = yawr = 0.0;                 // 同帧：无需换算
      pose_src = "base_link";
    } else if (gf == "map" && reloc_ok) {
      xr = reloc.pose.pose.position.x;
      yr = reloc.pose.pose.position.y;
      const auto & q = reloc.pose.pose.orientation;
      yawr = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      pose_src = "map";
    } else {
      // 默认：odom 帧（Nav2 local_costmap 标准配置）——需 EKF 位姿新鲜
      if (odom_age > self_check_data_timeout_) {
        rep.valid = false;
      }
      xr = odom.pose.pose.position.x;
      yr = odom.pose.pose.position.y;
      const auto & q = odom.pose.pose.orientation;
      yawr = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }
  }

  // ---- A：代价地图近身占位格统计 ----
  if (grid_ok) {
    const double res = grid->info.resolution;
    const double ox  = grid->info.origin.position.x;
    const double oy  = grid->info.origin.position.y;
    const int w = static_cast<int>(grid->info.width);
    const int h = static_cast<int>(grid->info.height);
    const double c = std::cos(yawr), s = std::sin(yawr);
    double best_d = 1e9;
    if (res > 1e-6 && w > 0 && h > 0) {
      for (int j = 0; j < h; ++j) {
        const double wy = oy + (static_cast<double>(j) + 0.5) * res;
        const double dyo = wy - yr;
        for (int i = 0; i < w; ++i) {
          if (grid->data[static_cast<size_t>(j) * w + i] < self_check_cost_min_) {
            continue;
          }
          const double wx = ox + (static_cast<double>(i) + 0.5) * res;
          const double dxo = wx - xr;
          // odom → base_link：逆旋转变换（仅 2D，代价地图为程度图，无高度）
          const double xb = c * dxo + s * dyo;
          const double yb = -s * dxo + c * dyo;
          if (xb < bx0 - m || xb > bx1 + m || yb < by0 - m || yb > by1 + m) {
            continue;
          }
          ++rep.occupied_cells;
          // 到【原始车身框】边缘的距离：0 = 在框内（即"脚底"）
          const double ddx = std::max({bx0 - xb, xb - bx1, 0.0});
          const double ddy = std::max({by0 - yb, yb - by1, 0.0});
          best_d = std::min(best_d, std::hypot(ddx, ddy));
        }
      }
    }
    rep.body_occupied = rep.occupied_cells > 0;
    rep.nearest_cost_dist = rep.body_occupied ?
      (best_d >= 1e9 ? 0.0 : best_d) : 0.0;
  }

  // ---- B：激光 / 融合（含相机）目标是否在近身范围有实体 ----
  // 目标以"中心 + 半径"包络参与判定（尺寸缺失时按 0.15m 最小半径兼容）
  const auto near_body = [&](const hunter_msgs::msg::DetectedObject & o) {
    const double oxo = o.pose.position.x, oyo = o.pose.position.y;
    const double r = std::max(0.15, 0.5 * std::max(o.dimensions.x, o.dimensions.y));
    const double ddx = std::max({bx0 - oxo, oxo - bx1, 0.0});
    const double ddy = std::max({by0 - oyo, oyo - by1, 0.0});
    return std::hypot(ddx, ddy) <= (r + m);
  };
  int lidar_near = 0, fused_near = 0;
  if (lidar_ok) {
    for (const auto & o : lidar_objs.objects) {
      if (near_body(o)) {
        ++lidar_near;
      }
    }
    rep.lidar_present = lidar_near > 0;
  }
  if (fused_ok) {
    for (const auto & o : fused_objs.objects) {
      if (near_body(o)) {
        ++fused_near;
      }
    }
    rep.fused_present = fused_near > 0;
  }

  // ---- 结论与防抖 ----
  rep.phantom = rep.valid && rep.body_occupied && !rep.lidar_present && !rep.fused_present;
  if (rep.phantom) {
    ++phantom_confirm_;
  } else {
    phantom_confirm_ = 0;
  }
  rep.phantom_confirmed = rep.phantom && phantom_confirm_ >= self_check_confirm_count_;

  std::ostringstream oss;
  oss << "代价地图(" << pose_src << " 帧, 龄期" <<
      (grid_age > 1e8 ? 999.0 : grid_age) << "s): 近身占位格 " << rep.occupied_cells <<
      " 格（最近距车身框 " << rep.nearest_cost_dist << "m，0=脚底）"
      "｜激光目标: " << (lidar_ok ? std::to_string(lidar_objs.objects.size()) : std::string("超时"))
      << " 个（近身 " << lidar_near << "）"
      << "｜融合目标(含相机): "
      << (fused_ok ? std::to_string(fused_objs.objects.size()) : std::string("超时"))
      << " 个（近身 " << fused_near << "）"
      << "｜相机目标: " << (rep.camera_alive ?
        ("在线(" + std::to_string(vision_count) + " 目标)") : "超时/未接入")
      << "｜假障碍计数 " << phantom_confirm_ << "/" << self_check_confirm_count_;
  rep.detail = oss.str();
  return rep;
}

// ==========================================================================
// V0.1.00 需求③：地图可通行连通域缓存（惰性重建，独立 geom_mutex_）
//
// 两步派生：
//   ① 距离场：每格到最近【占据/未建图】格的近似欧氏距离（5/7 chamfer 两遍扫描，
//      1/10 格单位），用于把"太贴墙/太贴障"的格剔除出可通行集；
//   ② 连通域：从当前车位所在可通行格出发做 8 邻域 BFS（斜向需两侧正交格均可通行，
//      杜绝"贴对角缝穿墙"），预算超 reach_bfs_max_cells_ 则截断并标记结论不可靠。
// 锁级次：先取 data_mutex_ 拷贝 shared_ptr 快照并释放，再拿 geom_mutex_ 算几秒级
//   的派生数据（不持 data_mutex_ 跑长循环，避免阻塞 10Hz 主循环与回调）。
// ==========================================================================
bool AutoMissionNode::ensureReachabilityCache(double px, double py)
{
  if (!reachability_enable_) {
    return false;
  }
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_snap;
  uint64_t rev = 0;
  {
    std::lock_guard<std::mutex> lk(data_mutex_);
    map_snap = latest_map_;
    rev = map_revision_;
  }
  if (!map_snap) {
    return false;   // 地图未就绪：不可判定（调用方放行）
  }
  const int w = static_cast<int>(map_snap->info.width);
  const int h = static_cast<int>(map_snap->info.height);
  const double res = map_snap->info.resolution;
  const double ox = map_snap->info.origin.position.x;
  const double oy = map_snap->info.origin.position.y;
  if (w <= 0 || h <= 0 || res <= 1e-6) {
    return false;
  }
  const size_t n = static_cast<size_t>(w) * static_cast<size_t>(h);
  if (px < ox || py < oy || px >= ox + w * res || py >= oy + h * res) {
    return false;   // 车位在图外：交由地图边界校验处理
  }

  std::lock_guard<std::mutex> gk(geom_mutex_);

  // ---- ① 距离场（仅在地图修订号变化时重建） ----
  if (occ_dist_rev_ != rev || occ_dist_field_.size() != n) {
    const int32_t BIG = 1 << 20;
    std::vector<int32_t> d(n, BIG);
    for (size_t i = 0; i < n; ++i) {
      const int8_t v = map_snap->data[i];
      if (v >= 50 || v < 0) {
        d[i] = 0;   // 占据 / 未建图 均为"障碍源"
      }
    }
    for (int y = 0; y < h; ++y) {
      const size_t row = static_cast<size_t>(y) * w;
      for (int x = 0; x < w; ++x) {
        const size_t i = row + x;
        int32_t best = d[i];
        if (x > 0) { best = std::min(best, d[i - 1] + 10); }
        if (y > 0) { best = std::min(best, d[i - w] + 10); }
        if (x > 0 && y > 0) { best = std::min(best, d[i - w - 1] + 14); }
        if (x + 1 < w && y > 0) { best = std::min(best, d[i - w + 1] + 14); }
        d[i] = best;
      }
    }
    for (int y = h - 1; y >= 0; --y) {
      const size_t row = static_cast<size_t>(y) * w;
      for (int x = w - 1; x >= 0; --x) {
        const size_t i = row + x;
        int32_t best = d[i];
        if (x + 1 < w) { best = std::min(best, d[i + 1] + 10); }
        if (y + 1 < h) { best = std::min(best, d[i + w] + 10); }
        if (x + 1 < w && y + 1 < h) { best = std::min(best, d[i + w + 1] + 14); }
        if (x > 0 && y + 1 < h) { best = std::min(best, d[i + w - 1] + 14); }
        d[i] = best;
      }
    }
    occ_dist_field_.resize(n);
    for (size_t i = 0; i < n; ++i) {
      occ_dist_field_[i] = (d[i] >= BIG) ?
        1e6f : static_cast<float>(d[i]) * 0.1f * static_cast<float>(res);
    }
    occ_dist_rev_ = rev;
    reach_rev_ = 0;   // 距离场变了 → 连通域必须重建
  }

  // ---- ② 可通行格判据 ----
  const double min_clear = reach_clearance_;
  const auto passable = [&](int x, int y) -> bool {
    if (x < 0 || y < 0 || x >= w || y >= h) {
      return false;
    }
    const size_t i = static_cast<size_t>(y) * w + x;
    const int8_t v = map_snap->data[i];
    if (v >= 50 || v < 0) {
      return false;
    }
    return occ_dist_field_[i] >= static_cast<float>(min_clear);
  };

  int sx = static_cast<int>(std::floor((px - ox) / res));
  int sy = static_cast<int>(std::floor((py - oy) / res));
  sx = std::max(0, std::min(w - 1, sx));
  sy = std::max(0, std::min(h - 1, sy));

  // 车位本格不可通行（车停在障碍/膨胀区内）→ 1m 内找最近可通行格做 BFS 起点；
  //   找不到则"无法判定"（返回 false 放行），杜绝把全部航点误判为不可达
  if (!passable(sx, sy)) {
    const int span = static_cast<int>(std::ceil(1.0 / res));
    int bx = -1, by = -1;
    double bd = 1e9;
    for (int dy = -span; dy <= span; ++dy) {
      for (int dx = -span; dx <= span; ++dx) {
        if (!passable(sx + dx, sy + dy)) {
          continue;
        }
        const double dd = std::hypot(static_cast<double>(dx), static_cast<double>(dy)) * res;
        if (dd < bd) {
          bd = dd;
          bx = sx + dx;
          by = sy + dy;
        }
      }
    }
    if (bx < 0) {
      return false;
    }
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
      "[地图可达性] 车当前所在格净空不足（<%.2fm），改以 %.2fm 外最近可通行格为连通域起点",
      reach_clearance_, bd);
    sx = bx;
    sy = by;
  }

  // ---- ③ 连通域（地图变了 / 车位动了超阈 才重建） ----
  const bool need_rebuild =
    (reach_rev_ != rev) || (reach_mask_.size() != n) || (reach_cx0_ < 0) ||
    (std::hypot(static_cast<double>(sx - reach_cx0_), static_cast<double>(sy - reach_cy0_)) *
      res > reach_recompute_dist_);
  if (need_rebuild) {
    reach_mask_.assign(n, 0);
    std::vector<int32_t> q;
    q.reserve(4096);
    const int32_t s0 = sy * w + sx;
    reach_mask_[static_cast<size_t>(s0)] = 1;
    q.push_back(s0);
    reach_truncated_ = false;
    static const int kOff[8][2] = {
      {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
    size_t head = 0;
    int expanded = 0;
    for (; head < q.size(); ++head) {
      if (++expanded > reach_bfs_max_cells_) {
        reach_truncated_ = true;
        break;
      }
      const int ci = q[head] % w;
      const int cj = q[head] / w;
      for (const auto & k : kOff) {
        const int nx = ci + k[0];
        const int ny = cj + k[1];
        if (k[0] != 0 && k[1] != 0 && (!passable(ci + k[0], cj) || !passable(ci, cj + k[1]))) {
          continue;   // 斜向需两侧正交格均可通行
        }
        if (!passable(nx, ny)) {
          continue;
        }
        const size_t ni = static_cast<size_t>(ny) * w + nx;
        if (reach_mask_[ni]) {
          continue;
        }
        reach_mask_[ni] = 1;
        q.push_back(ny * w + nx);
      }
    }
    reach_rev_ = rev;
    reach_cx0_ = sx;
    reach_cy0_ = sy;
    RCLCPP_INFO(get_logger(),
      "[地图可达性] 连通域已重建（地图 %dx%d@%.2fm，净空门槛 %.2fm，可通行格 %d%s）",
      w, h, res, reach_clearance_, expanded, reach_truncated_ ? "，已被预算截断" : "");
  }
  return true;
}

// ==========================================================================
// V0.1.00 需求③：航点地图可达性明细
//   返回空串 = 可达或"无法判定"（放行）；否则为不可达原因
// ==========================================================================
std::string AutoMissionNode::waypointReachabilityDetail(double wx, double wy)
{
  if (!reachability_enable_) {
    return "";
  }
  double px = 0.0, py = 0.0;
  if (!currentMapPose(px, py)) {
    return "";    // 无全局位姿：不判定
  }
  if (!ensureReachabilityCache(px, py)) {
    return "";    // 地图未就绪 / 车位无净空 / 信息不足：不判定（放行）
  }
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_snap;
  uint64_t rev = 0;
  {
    // 地图快照与其修订号必须在同一次 data_mutex_ 临界区内取，否则可能与
    //   下方 geom_mutex_ 保护的缓存对不上（map_revision_ 裸读的数据竞争）
    std::lock_guard<std::mutex> lk(data_mutex_);
    map_snap = latest_map_;
    rev = map_revision_;
  }
  if (!map_snap) {
    return "";
  }
  std::lock_guard<std::mutex> gk(geom_mutex_);
  const int w = static_cast<int>(map_snap->info.width);
  const int h = static_cast<int>(map_snap->info.height);
  const double res = map_snap->info.resolution;
  const size_t n = static_cast<size_t>(w) * static_cast<size_t>(h);
  if (reach_mask_.size() != n || reach_rev_ != rev) {
    return "";    // 缓存与本帧地图不对应：不判定
  }
  const int gx = static_cast<int>(std::floor(
      (wx - map_snap->info.origin.position.x) / res));
  const int gy = static_cast<int>(std::floor(
      (wy - map_snap->info.origin.position.y) / res));
  if (gx < 0 || gy < 0 || gx >= w || gy >= h) {
    return "";    // 越界由 waypointMapCheckDetail 负责拦截
  }
  const size_t gi = static_cast<size_t>(gy) * w + gx;
  if (reach_mask_[gi]) {
    return "";    // 与车位同一可通行连通域
  }
  if (reach_truncated_) {
    return "";    // BFS 被预算截断 → 不可达结论不可靠，放行
  }
  const double dist = std::hypot(wx - px, wy - py);
  return "该航点与当前车位不在同一【可通行连通域】内（直线距离 " +
    std::to_string(static_cast<int>(dist * 100.0) / 100.0) +
    "m，静态地图判定不可达：被障碍割开或位于孤立区域）";
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
      // V0.1.00：拒收单独计数（不再占用航点失败计数——bt_navigator 不接 goal
      //   与"这个航点往不去"是两回事），接受即清零
      ++goal_reject_count_;
      nav_retry_not_before_ = this->now() + rclcpp::Duration::from_seconds(nav_retry_backoff_);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "[Nav2] goal 被服务端拒收（连续 %d/%d 次），%.0fs 后重试",
        goal_reject_count_, max_wp_failures_, nav_retry_backoff_);
      if (goal_reject_count_ >= max_wp_failures_) {
        need_fault = true;
      }
    } else {
      goal_handle_ = handle;
      goal_reject_count_ = 0;   // V0.1.00：一旦接受即证明 Nav2 健康，拒收计数归零
      RCLCPP_INFO(get_logger(), "[Nav2] goal 已被接受，开始导航至航点[%zu]",
        current_wp_idx_);
    }
  }
  if (need_fault) {
    enterFault("goal 连续被 bt_navigator 拒收（Nav2 未就绪 / 并发上限 / lifecycle 非 ACTIVE）");
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
  //   ① 不能计入航点失败计数（否则一次受阻连跳 2 个航点 → 误进 FAULT）；
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
        // V0.1.00："已到达"等价于本点成功 → 清该点计数与任务级计数
        if (current_wp_idx_ < wp_rt_.size()) {
          wp_rt_[current_wp_idx_].fail_count = 0;
          wp_rt_[current_wp_idx_].isolated = false;
        }
        consec_fail_ = 0;
        mission_recovery_round_ = 0;
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
    // V0.1.00：一次成功到达即同时清零：该点失败计数、任务级连续失败计数、
    //   拒收计数与自愈轮次（航点能走通就说明之前的隔离/诊断已不适用）
    if (current_wp_idx_ < wp_rt_.size()) {
      wp_rt_[current_wp_idx_].fail_count = 0;
      wp_rt_[current_wp_idx_].isolated = false;
      wp_rt_[current_wp_idx_].last_reason.clear();
    }
    consec_fail_ = 0;
    goal_reject_count_ = 0;
    mission_recovery_round_ = 0;

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
    // V0.1.00：非导航态的残余结果不再参评（降级/急停已取消过 goal）
    if (state_ != MissionState::NAVIGATING && state_ != MissionState::OBSTACLE_AVOID) {
      RCLCPP_INFO(get_logger(),
        "[Nav2] 航点[%zu] %s 结果 %s 到达于非导航态（%s），不计失败也不换点",
        current_wp_idx_, waypoints_[current_wp_idx_].label.c_str(), reason,
        stateToString(state_).c_str());
      return;
    }
    RCLCPP_WARN(get_logger(),
      "[Nav2] 航点[%zu] %s 导航失败（%s）",
      current_wp_idx_, waypoints_[current_wp_idx_].label.c_str(), reason);
    // 重试退避：给 Nav2 恢复/系统稳定留窗口，防止立刻重发
    nav_retry_not_before_ = this->now() + rclcpp::Duration::from_seconds(nav_retry_backoff_);
    // V0.1.00 需求①：失败处置统一交给 handleWaypointFailure（逐点隔离 + 清图重规划
    //   + 换点），不在本回调里直接换点重发——重发由 NAVIGATING 的"无在途 goal"
    //   路径经清图/取消静置门控放行，避开旧版"ABORT→立即发下一点→被旧 BT 波及"
    if (state_ == MissionState::NAVIGATING || state_ == MissionState::OBSTACLE_AVOID) {
      handleWaypointFailure("Nav2 返回 " + std::string(reason));
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
