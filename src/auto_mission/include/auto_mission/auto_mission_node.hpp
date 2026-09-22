// Copyright 2026 HUNTER Development Team
// AUTO 模式自主任务调度节点头文件
// 功能：航点巡航任务管理、AUTO 进入条件守护、障碍物安全约束（减速/避让/急停）、
//       航点地图越界校验（矩形边界+安全边距+未建图栅格，V0.0.82/0.0.87）、
//       V0.1.00 任务自愈四件套：
//         ① 航点级失败隔离与自动轮转（单点超时/受阻不再拖垮整条巡航线）；
//         ② 车身近身【假障碍占位】诊断（代价地图 vs 激光 + 相机双传感器交叉验证）；
//         ③ 航点【地图可达性】判定（静态地图矩形/栅格/净空 + 可通行连通域）；
//         ④ FAULT 由"永久锁存等人解锁"改为"自愈态"（遥控接管后切回 AUTO 即自动复驶）。
#ifndef AUTO_MISSION__AUTO_MISSION_NODE_HPP_
#define AUTO_MISSION__AUTO_MISSION_NODE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <atomic>
#include <cstdint>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"  // V0.0.92：AMCL 收敛检查
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/empty.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"   // V0.0.93：清图服务真实类型（CycloneDDS 下 std_srvs/Empty 无法就绪）
#include "lifecycle_msgs/srv/get_state.hpp"

#include "hunter_msgs/msg/behavior_state.hpp"
#include "hunter_msgs/msg/detected_object_array.hpp"
#include "hunter_msgs/msg/system_health.hpp"

namespace auto_mission
{

// ---------------------------------------------------------------------------
// 任务状态机枚举
// ---------------------------------------------------------------------------
enum class MissionState : uint8_t
{
  IDLE = 0,          // 空闲：等待 AUTO 条件或外部触发
  MAPPING,           // 建图模式（FAST-LIO2 在线建图，不下发导航目标）
  WAITING_LOCALIZE,  // 等待定位收敛
  NAVIGATING,        // 正在执行导航目标
  OBSTACLE_AVOID,    // 障碍物减速等待
  ESTOP,             // 急停状态
  FAULT,             // 任务故障停驻（V0.0.91 引入锁存语义）：连续失败达上限后停驻，
                     // 不再静默重发；
                     // V0.1.00 起为【自愈态】：停驻期间周期性做近身占位诊断 + 航点可达性
                     // 复检，条件满足即自动清图、解隔离并重启任务（模式开关离开 AUTO 再切回
                     // 仍是即时的人工复位通道）
};

// ---------------------------------------------------------------------------
// 航点结构体
// ---------------------------------------------------------------------------
struct Waypoint
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};   // 目标朝向（弧度）
  std::string label; // 便于日志识别
};

// ---------------------------------------------------------------------------
// AutoMissionNode 主类
// ---------------------------------------------------------------------------
class AutoMissionNode : public rclcpp::Node
{
public:
  explicit AutoMissionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
  ~AutoMissionNode() = default;

private:
  // ---- 参数加载 ----
  void declareParameters();
  void loadWaypoints();

  // ---- 订阅回调 ----
  void behaviorStateCallback(const hunter_msgs::msg::BehaviorState::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void fusedObjectsCallback(const hunter_msgs::msg::DetectedObjectArray::SharedPtr msg);
  void systemHealthCallback(const hunter_msgs::msg::SystemHealth::SharedPtr msg);
  void estopCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void mapCallback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);  // /map 边界缓存（航点越界校验，V0.0.82/0.0.87）
  void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);  // V0.0.92：AMCL 收敛门控

  // ---- 主循环（10Hz 定时器） ----
  void mainLoop();

  // ---- 条件检查 ----
  bool isAutoConditionMet();     // AUTO 进入条件全部满足
  bool isLocalizationValid();    // 位置协方差是否收敛
  bool isPerceptionAlive();      // 感知数据是否新鲜（< 2s）
  double nearestObstacleDist();  // 最近融合障碍物距离（base_link，前向扇区）

  // ---- V0.1.00 航点级失败隔离（需求①：单航点失败只废该点，不废整条任务） ----
  struct WaypointRuntime
  {
    int fail_count{0};              // 该航点连续失败次数（达 max_wp_failures → 隔离）
    bool isolated{false};           // 隔离中：本轮不再下发
    bool permanent{false};          // 永久隔离：地图不可达（换地图修订号 / 任务复位后重判）
    rclcpp::Time isolated_at{0, 0, RCL_ROS_TIME};
    std::string last_reason;        // 最近一次失败/隔离原因（日志判读用）
  };

  // ---- V0.1.00 车身近身占位诊断结论（需求②：假障碍由激光 + 相机共同判定） ----
  struct SurroundReport
  {
    bool valid{false};              // 数据新鲜度足够，本次判据可用（false = 不可判定）
    bool body_occupied{false};      // 代价地图在车身框（含外扩边距）内标了内切/致命格
    bool lidar_present{false};      // 激光感知在近身范围内有目标
    bool fused_present{false};      // 融合目标（含仅相机确认者）在近身范围内有目标
    bool camera_alive{false};       // 相机目标话题在线（新鲜度判据）
    bool phantom{false};            // 单次判据成立：地图占位而双传感器均无实体
    bool phantom_confirmed{false};  // 连续 self_check_confirm_count 次成立（防抖）
    int occupied_cells{0};          // 车身框内被占格数
    double nearest_cost_dist{0.0};  // 最近占位格到【原始车身框】边缘的距离（m，0=框内/脚底）
    std::string detail;             // 可直接读给现场的结论串
  };

  // ---- 导航控制 ----
  void sendNextWaypoint();
  void cancelCurrentGoal();
  void triggerEstop(const std::string & reason);
  void enterFault(const std::string & reason);  // 任务故障停驻（V0.1.00 起为自愈态入口）
  void clearCostmapsOnStart();                  // 任务（重）启动时异步清一次全局/局部代价地图（V0.0.91）
  // ---- V0.1.00 任务自愈 ----
  void syncWaypointRuntime();                        // 使 wp_rt_ 与 waypoints_ 同尺寸（加载/热重载后）
  void requestCostmapClear(const std::string & why); // 统一清图入口（置挂起标志，由 NAVIGATING 门控重试）
  void isolateWaypoint(size_t idx, const std::string & reason, bool permanent);  // 隔离某航点
  void promoteExpiredIsolation(size_t idx);          // 冷却到期的临时隔离自动解除
  bool advanceToNextAvailableWaypoint();             // 轮转到下一个可用航点（false = 全部不可用）
  void handleWaypointFailure(const std::string & reason);  // 放弃当前航点 → 诊断 → 清图重规划 → 换点
  void resetMissionState(const std::string & why, bool clear_permanent);
  void faultRecoveryStep();                          // FAULT 自愈：周期诊断，条件满足自动重启任务
  SurroundReport diagnoseSelfSurroundings();         // 近身假障碍诊断（代价地图 vs 激光/相机）
  bool ensureReachabilityCache(double px, double py);  // 地图可通行连通域（惰性重建，geom_mutex_）
  // 航点可达性明细：空串 = 可达或"无法判定"（放行）；否则为不可达原因（需求③）
  std::string waypointReachabilityDetail(double wx, double wy);
  bool tryReleaseSelfEstop();  // 自触发急停（障碍物类）解除：危险消除后发布 /estop=false
  bool waypointInsideMap(const Waypoint & wp);  // 航点在已采集地图区域内（边界+边距+非未建图栅格+净空，V0.0.82/0.0.87/0.0.96）
  std::string waypointMapCheckDetail(const Waypoint & wp);  // 拒绝原因（空=通过）：边界外 / 未建图 / 占据栅格 / 净空不足
  // V0.0.96 map 系位姿 (x,y) 到最近【占据栅格】的欧氏距离（m）；
  //   搜索半径 max_search_radius 内无占据栅格 → 返回 max_search_radius（表示"足够远"）；
  //   地图未就绪或点在图外 → 返回 +inf（由边界校验负责拦截）。
  double nearestObstacleClearance(double x, double y, double max_search_radius);
  // V0.0.95 航点“已到达”预检：车已在航点到达半径内（位置重合）时不得再发 goal——
  //   “目标=当前位姿”对阿克曼是退化目标（左/右满舵都到不了），MPPI 会持续打满转向而
  //   纵向零进挪，ProgressChecker(0.1m/10s) 必判 Failed to make progress，
  //   BT 恢复池（非运动：清图+Wait）也救不了 → 车辆原地抖动、任务卡死。
  bool waypointAlreadyReached(const Waypoint & wp, double & dist, double & yaw_err);
  // V0.0.95 当前 map 系位姿快照（/relocalization/pose）；未收到位姿返回 false（调用方放行旧行为）
  bool currentMapPose(double & x, double & y);

  // ---- Nav2 就绪门控（bt_navigator lifecycle 状态） ----
  void queryNavigatorState();    // 异步查询 bt_navigator 状态（1Hz 节流，不阻塞主循环）
  void navigatorStateResponse(
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture future);

  // ---- 建图模式自动巡航（mapping auto cruise）----
  void startCruiseCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr resp);
  void stopCruiseCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr resp);
  bool parseWaypoint(const std::string & entry, Waypoint & wp);
  bool reloadWaypointsFromFile(std::string & msg);   // 重新读取 params_file 的 waypoints
  void cruiseControlStep();                          // 20Hz 巡航控制（含安全约束）
  void publishCruiseCmd(double v, double w);
  static double wrapAngle(double a);                 // 归一化到 [-π, π]

  // ---- Nav2 action 回调 ----
  // V0.0.97：两个回调都带 goal 世代号 epoch，用于丢弃【过期 goal】的响应/结果
  void goalResponseCallback(
    uint64_t epoch,
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & handle);
  void feedbackCallback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback);
  void resultCallback(
    uint64_t epoch,
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result);

  // ---- 状态发布 ----
  void publishStatus();
  static std::string stateToString(MissionState s);

  // ---- 话题 / Action ----
  rclcpp::Subscription<hunter_msgs::msg::BehaviorState>::SharedPtr behavior_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<hunter_msgs::msg::DetectedObjectArray>::SharedPtr fused_objects_sub_;
  rclcpp::Subscription<hunter_msgs::msg::SystemHealth>::SharedPtr health_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;  // V0.0.92

  // ---- V0.1.00 近身假障碍诊断数据源（均为系统已有话题，只新增订阅，不新增话题） ----
  // /local_costmap/costmap：Nav2 局部代价地图（帧=odom，滚动窗口，含动态障碍层）
  // /perception/lidar_objects：激光感知目标（base_link）
  // /perception/vision_objects：相机目标（camera_color_optical_frame，本节点只用其新鲜度/计数）
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_sub_;
  rclcpp::Subscription<hunter_msgs::msg::DetectedObjectArray>::SharedPtr lidar_objects_sub_;
  rclcpp::Subscription<hunter_msgs::msg::DetectedObjectArray>::SharedPtr vision_objects_sub_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr waypoint_idx_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;

  // ---- 建图模式自动巡航 ----
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_odom_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_cruise_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_cruise_srv_;
  rclcpp::TimerBase::SharedPtr cruise_timer_;

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_action_client_;

  // ---- Nav2 就绪门控状态 ----
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr nav_state_client_;
  std::atomic<bool> costmaps_clear_pending_{false};  // V0.0.92：进入 NAVIGATING 后若服务未就绪则重试清图

  // ---- 代价地图清除客户端（V0.0.91：任务（重）启动时主动清障） ----
  // 注意：Nav2 clear_entirely_*_costmap 服务主类型为 nav2_msgs/srv/ClearEntireCostmap，
  // 虽序列化兼容 std_srvs/Empty，但 CycloneDDS graph 匹配只认主类型，用 Empty 会导致
  // service_is_ready() 恒 false、清图门控永久 PEND、goal 永不发送。故此处必须用原生类型。
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_global_costmap_srv_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_local_costmap_srv_;
  std::atomic<bool> nav_active_{false};        // bt_navigator 是否 ACTIVE
  rclcpp::Time nav_state_query_time_{0, 0, RCL_ROS_TIME};  // 上次状态查询时刻（1Hz 节流）
  rclcpp::Time nav_wait_start_{0, 0, RCL_ROS_TIME};        // 开始等待 bt_navigator 激活的时刻
  bool nav_wait_started_{false};
  rclcpp::Time nav_retry_not_before_{0, 0, RCL_ROS_TIME};  // goal 被拒/失败后的退避截止时刻

  rclcpp::TimerBase::SharedPtr main_timer_;

  // ---- 缓存的最新订阅数据（mutex 保护） ----
  std::mutex data_mutex_;
  hunter_msgs::msg::BehaviorState latest_behavior_state_;
  nav_msgs::msg::Odometry latest_odom_;
  hunter_msgs::msg::DetectedObjectArray latest_fused_objects_;
  hunter_msgs::msg::SystemHealth latest_health_;
  bool estop_signal_{false};
  std::atomic<bool> estop_self_triggered_{false};  // 急停由本节点触发（障碍物类）；外部急停由发布方解除
  // ---- V0.0.93 方案A：全局重定位(NDT)收敛缓存（原 AMCL，data_mutex_ 保护） ----
  geometry_msgs::msg::PoseWithCovarianceStamped latest_amcl_pose_;
  bool amcl_pose_received_{false};  // 收到过至少一帧 /relocalization/pose

  // ---- V0.1.00 近身诊断缓存（data_mutex_ 保护） ----
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_local_costmap_;
  rclcpp::Time last_local_costmap_arrive_{0, 0, RCL_ROS_TIME};
  hunter_msgs::msg::DetectedObjectArray latest_lidar_objects_;
  rclcpp::Time last_lidar_objects_arrive_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_vision_objects_arrive_{0, 0, RCL_ROS_TIME};
  int vision_objects_count_{0};              // 相机最近一帧目标数（仅用于结论描述）
  rclcpp::Time last_odom_arrive_{0, 0, RCL_ROS_TIME};  // /localization/odom 到达时刻

  // ---- 建图模式自动巡航缓存 ----
  nav_msgs::msg::Odometry latest_lio_odom_;   // FAST-LIO2 /Odometry（odom 系，原 camera_init）
  rclcpp::Time last_lio_odom_arrive_{0, 0, RCL_SYSTEM_TIME};  // 最近一次到达时刻（本节点时钟）
  std::atomic<bool> mapping_paused_{false};   // 非 AUTO / CRITICAL → 暂停巡航（保持 MAPPING）

  rclcpp::Time last_perception_stamp_;   // 感知数据最后到达时间

  // ---- 状态机 ----
  MissionState state_{MissionState::IDLE};
  MissionState prev_state_{MissionState::IDLE};

  // ---- 航点列表与索引 ----
  std::vector<Waypoint> waypoints_;
  size_t current_wp_idx_{0};
  // V0.1.00 需求①：失败计数下沉到【逐个航点】（旧 wp_fail_count_ 为跨航点总量，
  //   三个不同航点各失败一次就把整条任务锁存，而实际只是"那一个点去不了"）
  std::vector<WaypointRuntime> wp_rt_;
  int consec_fail_{0};                 // 期间无任何一次成功到达的累计失败次数（任务级兜底）
  int goal_reject_count_{0};           // bt_navigator 连续拒收计数（接受即清零）
  int mission_recovery_round_{0};      // 已进行的任务自愈轮次（一次成功后清零，决重试间隔）
  bool goal_in_flight_{false};         // 是否有 goal 在飞
  std::string fault_reason_;           // FAULT 停驻原因（V0.0.91，仅日志用）
  std::string fault_diagnosis_;        // 进入 FAULT 时的自动诊断结论（需求②③的可判读输出）
  rclcpp::Time fault_enter_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time fault_next_check_{0, 0, RCL_ROS_TIME};   // 下一次自愈重试检查时刻
  rclcpp::Time last_self_check_{0, 0, RCL_ROS_TIME};    // 上一次近身巡检时刻
  int phantom_confirm_{0};             // 假障碍连续判定计数（防抖）
  bool prev_mode_auto_{false};         // 上一拍是否处于 AUTO（用于检测"接管后切回"边沿）
  // V0.0.95 受阻（无法绕行）计数：goal 在途而长时间无位移时递增，供日志与诊断
  int blocked_count_{0};
  // V0.0.96 受阻判定基准：发 goal 时车辆到该航点的距离（负值 = 位姿未知，本航点不做受阻判定）。
  //   判据由"位移标量"改为"朝目标推进量"（= 起始距离 − 当前距离）：
  //   行为树脱困倒车会增大到目标距离 → 推进量为负 → 仍判受阻，而位移标量会被"后退"骗过。
  double goal_start_dist_{-1.0};
  // V0.0.97 绕障机动里程：自本 goal 发出起累计的行驶路程（|Δ位姿| 之和，map 系）。
  //   作用：区分"真卡死（不动）"与"多点掉头/受限倒车绕障中（净推进为负但在动）"。
  //   现场（V0.0.96 日志）：车被 1.0m 碰撞闸按停 → 满舵原地死磕 20s、位姿纹丝不动
  //   （累计行程 ≈0）→ 该判受阻；而 V0.0.97 放开倒车 + REEDS_SHEPP 后，绕障会出现
  //   "倒 0.4m→进 0.5m→再倒"的多段机动，净推进可能长期 <0.15m 却始终在机动，
  //   用里程上限（stall_path_allow_m）把这类正常机动从"受阻"里摘出去，
  //   真兜底交给单航点超时 goal_timeout_。
  double goal_path_len_{0.0};
  double last_stall_x_{0.0};
  double last_stall_y_{0.0};
  bool has_last_stall_pose_{false};
  // V0.0.95 主动取消标记：取消与换点已由取消方（受阻/超时/降级）完成，
  //   resultCallback 收到 CANCELED 时据此跳过重复的 fail_count++ 与换点，
  //   否则一次受阻会连跳两个航点并提前触发 FAULT。
  bool goal_cancel_by_mission_{false};

  // ---- 静态地图边界缓存（/map transient_local；V0.0.82 矩形边界校验，
  //      V0.0.87 增加未建图(unknown)栅格校验） ----
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_map_;
  double map_min_x_{0.0};
  double map_max_x_{0.0};
  double map_min_y_{0.0};
  double map_max_y_{0.0};
  double waypoint_map_margin_{0.5};  // 航点距地图边界的最小安全边距（m）
  // V0.0.96 航点净空（膨胀）校验半径（m）：航点距最近【占据栅格】的欧氏距离必须 ≥ 此值。
  //   必要性：SmacPlannerHybrid 的 areInputsValid() 用 GridCollisionChecker 判起点有效性，
  //   起点格代价为 LETHAL(254)/INSCRIBED(253)/UNKNOWN-with-traverse_unknown=false(255) 时
  //   直接抛 "Starting point in lethal space! Cannot create feasible plan."；
  //   而 Nav2 局部代价地图为滚动窗口且【不含静态层】→ MPPI 不会避开仅存在于静态地图中的
  //   障碍，会把车一路开到航点；一旦车停在静态障碍的膨胀/致命区内，此后所有规划全部失败。
  //   默认 0.50 > Nav2 inscribed_radius(0.32) + 定位误差余量；改小会在障碍旁制造死局。
  double waypoint_clearance_m_{0.50};

  // ---- V0.1.00 地图可达性（可通行连通域）派生几何缓存 ----
  // ⚠ 锁级次：latest_map_ 归 data_mutex_ 保护，而距离场/BFS 可达秒级计算，因而不
  //   能持 data_mutex_ 跑完（会阻塞 10Hz 主循环的全部回调）；改用独立的 geom_mutex_
  //   + 地图快照（shared_ptr 拷贝）计算，既无嵌套加锁也无自死锁风险。
  std::mutex geom_mutex_;
  uint64_t map_revision_{0};            // /map 每收到一新地图自增（使旧缓存/永久隔离失效）
  std::vector<float> occ_dist_field_;   // 每格到最近占据栅格的近似欧氏距离（m，3-4 chamfer）
  uint64_t occ_dist_rev_{0};            // occ_dist_field_ 对应的地图修订号
  std::vector<uint8_t> reach_mask_;     // 从当前车位出发的可通行连通域（1=可达）
  uint64_t reach_rev_{0};               // reach_mask_ 对应的地图修订号
  int reach_cx0_{-1};                   // reach_mask_ 计算起点格（地图变化/车位移动超阈则重建）
  int reach_cy0_{-1};
  bool reach_truncated_{false};         // BFS 达上限被截断 ⇒ 不可达结论不可靠（放行）

  // ---- 定位等待计时 ----
  rclcpp::Time localize_wait_start_;
  bool localize_wait_started_{false};

  // ---- 障碍物等待计时 ----
  rclcpp::Time obstacle_wait_start_;
  bool obstacle_wait_started_{false};

  // ---- Nav2 goal handle ----
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_;
  std::mutex goal_handle_mutex_;
  // V0.0.97 goal 世代号（epoch）：每次 async_send_goal【之前】自增，并把当时的取值
  //   绑定进 goal_response/result 回调。回调据此丢弃"已放弃的旧 goal"留下的结果。
  //   现场故障链（V0.0.96 实车日志 1790055992~1790055995）：本节点因"受阻"主动
  //   cancel 旧 goal 后立刻发新 goal，旧 goal 的结果以 **ABORTED**（而非 CANCELED，
  //   因行为树当时已失败）返回 →
  //     ① 旧结果被当成【新航点】的失败：一次受阻连跳 2 个航点、fail_count 迅速
  //        3/3 → FAULT 误锁存（车其实还在正常行驶）；
  //     ② 旧结果在回调开头清空 goal_in_flight_/goal_handle_，导致随后的
  //        cancelCurrentGoal() 变成空操作 → FAULT 后行为树仍在驱动车辆满舵倒车
  //        机动 10 分钟（1790056004~1790056157），现场看到"车不停乱动"。
  //   有了世代号，过期结果一律丢弃：既不误计失败，也不破坏在途 goal 的句柄。
  uint64_t goal_epoch_{0};
  // V0.0.98 取消静置门控：cancelCurrentGoal() 后不得立即发下一个 goal。
  //   现场（V0.0.97 日志）：取消后 2ms 即发航点[3]，bt_navigator 尚在清理旧
  //   BT 的取消/恢复状态，新 goal 被同一轮失败波及（接受后 2ms ABORTED）。
  //   改为：取消时置 pending，NAVIGATING 的"无在途 goal 重发"路径需等
  //   旧 goal 结果到达（resultCallback 清 pending）或 settle 超时才放行。
  bool goal_cancel_pending_{false};
  rclcpp::Time goal_cancel_time_{0, 0, RCL_ROS_TIME};

  // ---- 参数 ----
  // 模式
  std::string mission_mode_;           // "mapping"（建图）| "nav"（自主导航）
  // 安全距离
  double warn_obstacle_dist_{2.0};     // 减速阈值（m）
  double stop_obstacle_dist_{0.8};     // 急停阈值（m）
  double obstacle_fov_deg_{120.0};     // 前向检测扇区（度）
  // 定位
  double localize_cov_threshold_{0.5}; // EKF 协方差迹阈值
  double amcl_cov_threshold_{0.60};    // V0.0.92：AMCL x+y 方差和阈值（初始帧=0.5，略宽松避免锁死）
  double localize_wait_timeout_{15.0}; // 等待收敛超时（s）
  // 感知
  double perception_timeout_{2.0};     // 感知新鲜度阈值（s）
  // 巡航
  bool loop_waypoints_{true};          // 循环/停车
  int max_wp_failures_{3};             // 最大连续失败次数
  double goal_timeout_{60.0};          // 单点导航超时（s）
  double nav_active_wait_timeout_{60.0}; // NAVIGATING 中等待 bt_navigator 激活的超时（s）
  double nav_retry_backoff_{2.0};      // goal 被拒/失败后的重试退避（s）
  double goal_cancel_settle_time_{2.0};  // V0.0.98 主动取消后等待旧 goal 结果回传的
                                         //   最长静置时长（s）：结果先到即放行；
                                         //   超时无结果兜底放行（bt_navigator 重启等场景）
  // V0.0.95 航点“已到达”预检与受阻检测
  double already_reached_dist_{0.35};  // 航点到达判定半径（m）：车与航点位置重合即跳过，不再发 goal
                                       // V0.0.97：0.30 → 0.35（修"0.30m 微调目标被阻 25s"）
  double stall_detect_time_{25.0};     // 受阻判定时长（s）：goal 在途而位移停滞超此时长判“无法绕行”
  double stall_move_eps_{0.15};        // 受阻判定净推进下限（m）：窗口内朝目标推进小于此值即停滞
  double stall_path_allow_m_{1.0};     // V0.0.97 绕障机动里程上限（m）：累计行程达到此值即视为
                                       //   正在多点机动，不判受阻（兜底由 goal_timeout 承担）
  // V0.0.96 航点净空校验半径（m）
  // 障碍物等待
  double obstacle_wait_timeout_{30.0}; // 障碍物等待超时（s）
  // 最大速度（仅日志/合规性检查；实际限速由 Nav2 params 控制）
  double max_velocity_{2.0};

  // ---- V0.1.00 需求①：航点隔离与任务级兜底 ----
  double wp_isolation_cooldown_{120.0};   // 临时隔离冷却（s），到期后自动重新接受该航点
  int max_consec_failures_{12};           // 任务级连续失败上限（无任何成功到达时），达上限进 FAULT 自愈

  // ---- V0.1.00 需求②：近身假障碍诊断（激光 + 相机 vs 代价地图） ----
  bool self_check_enable_{true};
  double self_check_margin_{0.35};        // 车身框外扩监视边距（m）——"车身四周"
  double self_check_box_front_{0.45};     // 车身框前缘（base_link x，与 nav2 footprint 一致）
  double self_check_box_rear_{0.37};      // 车身框后缘（base_link -x）
  double self_check_box_half_width_{0.32};  // 车身框半宽（base_link |y|）
  int self_check_cost_min_{99};           // 代价地图"占据"阈值（Nav2 转换表：99=内切、254→100=致命）
  int self_check_confirm_count_{3};       // 连续判定次数（防抖，避免单帧噪声触发清图）
  double self_check_interval_{5.0};       // NAVIGATING 中周期巡检间隔（s）
  double self_check_data_timeout_{1.0};   // 代价地图/里程计/激光目标新鲜度阈值（s）
  std::string local_costmap_topic_{"/local_costmap/costmap"};
  std::string lidar_objects_topic_{"/perception/lidar_objects"};
  std::string vision_objects_topic_{"/perception/vision_objects"};

  // ---- V0.1.00 需求③：地图可达性（连通域） ----
  bool reachability_enable_{true};
  double reach_clearance_{0.35};          // 连通域可通行格的最小净空（m，≈内切半径 0.32+余量）
  double reach_recompute_dist_{0.5};      // 车位移动超此距离才重建连通域（节流）
  int reach_bfs_max_cells_{400000};       // BFS 预算上限（超限被截断 ⇒ 不可达结论放行）

  // ---- V0.1.00 需求④：FAULT 自愈 ----
  bool fault_auto_recover_{true};
  double fault_hold_time_{20.0};          // 进 FAULT 后首次自愈检查前的静置时长（s）
  double fault_retry_interval_{30.0};     // 自愈重试基础间隔（s），随轮次递增
  double fault_retry_max_interval_{180.0};  // 自愈重试间隔上限（s）
  int max_mission_recoveries_{5};         // 超此轮次后仅放慢重试频率并给人工介入结论（不永久锁死）

  // ---- 建图模式自动巡航参数 ----
  std::string params_file_;              // autonomous_nav_params.yaml 路径（launch 传入，供热重载）
  double cruise_max_speed_{1.0};         // 巡航直行最大速度（m/s），建图建议 ≤1.0
  double cruise_turn_speed_{0.4};        // 大航向偏差时的限速（m/s）
  double cruise_min_speed_{0.2};         // 接近航点时的最低速度（m/s），避免蠕动
  double cruise_reach_dist_{0.6};        // 到达判定距离（m），≥阿克曼停车精度
  double cruise_brake_dist_{2.0};        // 进入减速区的距离（m）
  double cruise_kp_yaw_{1.2};            // 航向 P 增益
  double cruise_max_yaw_rate_{0.5};      // 最大角速度（rad/s），硬上限
  double cruise_yaw_slow_deg_{45.0};     // 航向偏差超过此值降速（度）
  double cruise_min_turn_radius_{1.9};   // HUNTER-SE 最小转弯半径（m），|w| ≤ v/R
  double cruise_cmd_rate_{20.0};         // /cmd_vel 发布频率（Hz），需 > 2×(1/cmd_vel_timeout)
  double cruise_odom_timeout_{1.0};      // /Odometry 超时（s），超时停车

  // ---- 建图模式自动巡航状态 ----
  bool cruise_active_{false};            // 巡航已启动（start 服务触发）
  bool cruise_cmd_published_{false};     // 本轮巡航是否发过指令（停止时补一帧零速）
  rclcpp::Time cruise_obstacle_wait_start_{0, 0, RCL_SYSTEM_TIME};
  bool cruise_obstacle_wait_{false};

  // ---- goal 发送时间（超时检测） ----
  rclcpp::Time goal_send_time_;
};

}  // namespace auto_mission

#endif  // AUTO_MISSION__AUTO_MISSION_NODE_HPP_
