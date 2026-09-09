// Copyright 2026 HUNTER Development Team
// AUTO 模式自主任务调度节点头文件
// 功能：航点巡航任务管理、AUTO 进入条件守护、障碍物安全约束（减速/避让/急停）
#ifndef AUTO_MISSION__AUTO_MISSION_NODE_HPP_
#define AUTO_MISSION__AUTO_MISSION_NODE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <atomic>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
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

  // ---- 主循环（10Hz 定时器） ----
  void mainLoop();

  // ---- 条件检查 ----
  bool isAutoConditionMet();     // AUTO 进入条件全部满足
  bool isLocalizationValid();    // 位置协方差是否收敛
  bool isPerceptionAlive();      // 感知数据是否新鲜（< 2s）
  double nearestObstacleDist();  // 最近融合障碍物距离（base_link，前向扇区）

  // ---- 导航控制 ----
  void sendNextWaypoint();
  void cancelCurrentGoal();
  void triggerEstop(const std::string & reason);
  bool tryReleaseSelfEstop();  // 自触发急停（障碍物类）解除：危险消除后发布 /estop=false

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
  void goalResponseCallback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & handle);
  void feedbackCallback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback);
  void resultCallback(
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

  // ---- 建图模式自动巡航缓存 ----
  nav_msgs::msg::Odometry latest_lio_odom_;   // FAST-LIO2 /Odometry（camera_init 系）
  rclcpp::Time last_lio_odom_arrive_{0, 0, RCL_SYSTEM_TIME};  // 最近一次到达时刻（本节点时钟）
  std::atomic<bool> mapping_paused_{false};   // 非 AUTO / CRITICAL → 暂停巡航（保持 MAPPING）

  rclcpp::Time last_perception_stamp_;   // 感知数据最后到达时间

  // ---- 状态机 ----
  MissionState state_{MissionState::IDLE};
  MissionState prev_state_{MissionState::IDLE};

  // ---- 航点列表与索引 ----
  std::vector<Waypoint> waypoints_;
  size_t current_wp_idx_{0};
  int wp_fail_count_{0};           // 连续失败计数
  bool goal_in_flight_{false};     // 是否有 goal 在飞

  // ---- 定位等待计时 ----
  rclcpp::Time localize_wait_start_;
  bool localize_wait_started_{false};

  // ---- 障碍物等待计时 ----
  rclcpp::Time obstacle_wait_start_;
  bool obstacle_wait_started_{false};

  // ---- Nav2 goal handle ----
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_;
  std::mutex goal_handle_mutex_;

  // ---- 参数 ----
  // 模式
  std::string mission_mode_;           // "mapping"（建图）| "nav"（自主导航）
  // 安全距离
  double warn_obstacle_dist_{2.0};     // 减速阈值（m）
  double stop_obstacle_dist_{0.8};     // 急停阈值（m）
  double obstacle_fov_deg_{120.0};     // 前向检测扇区（度）
  // 定位
  double localize_cov_threshold_{0.5}; // 协方差迹阈值
  double localize_wait_timeout_{10.0}; // 等待收敛超时（s）
  // 感知
  double perception_timeout_{2.0};     // 感知新鲜度阈值（s）
  // 巡航
  bool loop_waypoints_{true};          // 循环/停车
  int max_wp_failures_{3};             // 最大连续失败次数
  double goal_timeout_{60.0};          // 单点导航超时（s）
  double nav_active_wait_timeout_{60.0}; // NAVIGATING 中等待 bt_navigator 激活的超时（s）
  double nav_retry_backoff_{2.0};      // goal 被拒/失败后的重试退避（s）
  // 障碍物等待
  double obstacle_wait_timeout_{30.0}; // 障碍物等待超时（s）
  // 最大速度（仅日志/合规性检查；实际限速由 Nav2 params 控制）
  double max_velocity_{2.0};

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
