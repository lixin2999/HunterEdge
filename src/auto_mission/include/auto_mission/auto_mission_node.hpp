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
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"

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

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_action_client_;

  rclcpp::TimerBase::SharedPtr main_timer_;

  // ---- 缓存的最新订阅数据（mutex 保护） ----
  std::mutex data_mutex_;
  hunter_msgs::msg::BehaviorState latest_behavior_state_;
  nav_msgs::msg::Odometry latest_odom_;
  hunter_msgs::msg::DetectedObjectArray latest_fused_objects_;
  hunter_msgs::msg::SystemHealth latest_health_;
  bool estop_signal_{false};

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
  std::string mission_mode_;           // "waypoint_loop" | "external"
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
  // 障碍物等待
  double obstacle_wait_timeout_{30.0}; // 障碍物等待超时（s）
  // 最大速度（仅日志/合规性检查；实际限速由 Nav2 params 控制）
  double max_velocity_{2.0};

  // ---- goal 发送时间（超时检测） ----
  rclcpp::Time goal_send_time_;
};

}  // namespace auto_mission

#endif  // AUTO_MISSION__AUTO_MISSION_NODE_HPP_
