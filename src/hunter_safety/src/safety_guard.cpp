// Copyright 2026 HUNTER Development Team
// safety_guard — 碰撞防护与运动学安全约束节点（V0.0.98）
//
// 定位：串联在 velocity_smoother 与底盘（hunter_base 订阅 /cmd_vel）之间的
// 【最后一道物理安全闸】，独立于感知融合链（直接消费 /scan），与决策层
// （auto_mission OBSTACLE_AVOID/ESTOP）、模式仲裁（decision_making）
// 互为冗余。职责：
//   1. 碰撞闸（V0.0.98 轨迹扫掠弧）：不再按"直线矩形走廊"判净空，而是把
//      实际指令 (v,w) 积分成阿克曼预测轨迹（前 reaction_lag 秒直线 + 其后
//      圆弧），扫描弧带扫掠区域到最近障碍的弧长距离 < stop_dist → 立即零速；
//      < slow_dist → 线性限速（碰撞预警分级 SLOWDOWN/COLLISION_STOP）。
//      静止/低速时退回直线走廊判据。旧直线走廊与阿克曼绕障弧线互斥——
//      绕障横移期间直线投影恒有障碍，车被自己的安全闸按停（V0.0.97 现场
//      "净空 0.569m 急停 0.1s 即释放→满舵死磕"的根源）
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
//   9. 地图边界监护（V0.0.87）：车辆行驶范围不得超出已采集地图区域——
//      订阅 /map（transient_local）+ /relocalization/pose（map 系位姿），构建"到最近
//      未建图(unknown)/界外栅格"的距离场（Chamfer 3-4 两遍扫描，一次性），
//      运行时 O(1) 查询分级介入：距边界 < map_edge_stop_dist(0.5m) → 零速
//      （测试模式升级为中止锁存）；< map_edge_slow_dist(1.5m) → 线性限速
//      （/safety/state 新增 MAP_EDGE_STOP / MAP_EDGE_SLOWDOWN）。建图模式无
//      /map 与重定位位姿，监护自动静默不介入（建图巡航安全由 cruise_* 与人工保障）。
//      规划层（Nav2 track_unknown_space + allow_unknown=false）与任务层
//      （auto_mission 航点校验）为前两道防线，本监护为行驶中最后一道。
//
// 仅在导航模式（hunter_autonomous_nav mode:=nav）启动：mapping 模式下
// auto_mission cruise 直接发布 /cmd_vel（自带低速与急停逻辑），本节点若
// 在场会与之冲突。建图巡航安全由 cruise_* 参数与人工遥控接管保障。
//
// 参数（默认值与 HUNTER-SE 实车匹配）：
//   wheelbase=0.46m（HUNTER-SE 轴距，前/后轴中心距；仅用于 δ_max=atan(L/R_min)
//     的转向几何叙述——曲率钳制本身以 min_turn_radius 为准，不依赖本值）
//   min_turn_radius=1.9m（与 nav2_params.yaml Smac minimum_turning_radius 一致）

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include "action_msgs/msg/goal_status_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
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
    declare_parameter<double>("wheelbase", 0.46);          // HUNTER-SE 轴距（m）
    declare_parameter<double>("min_turn_radius", 1.9);     // 与 Smac 规划一致（m）
    declare_parameter<double>("max_linear_vel", 0.5);       // 纵向硬限（m/s）
                                                           // V0.1.02：《方案》§8.3 对齐——默认值
                                                           //   由 0.8 改 0.5，与 nav2_params.yaml
                                                           //   desired_linear_vel/vx_max/velocity_smoother
                                                           //   max_velocity[0] 同源。本参数是"速度硬限"
                                                           //   （第二重限速，防参数被误调回 2.0），
                                                           //   默认值若大于生产值，一旦 launch 忘记传参
                                                           //   就会静默放行 0.8m/s（=1.6 倍巡航速度，
                                                           //   制动距离 4 倍）→ 默认值必须等于生产值
    declare_parameter<double>("stop_dist", 0.90);           // 碰撞急停距离（m）
    declare_parameter<double>("slow_dist", 1.40);           // 减速预警距离（m）
    // V0.0.97 与 hunter_autonomous_nav.launch.py 同步重标定（原 0.6/1.2 为上游默认值，
    //   与实车 0.90/1.40 不一致会让"单独起 safety_guard"时行为突变）：
    //   · stop 0.90 = 场景可行性与制动安全的折中：> range_min(0.70)+0.15 = 0.85 盲区地板，
    //     且前保险杠停车净空 0.90−0.45 = 0.45m ≫ 0.5m/s 制动距离 ≈0.18m；
    //   · 不可再轻易下调：阿克曼绕障需"提前转向距离" s ≥ √(2R·(半宽+余量)) ≈ 1.33m，
    //     碰撞闸远大于 1.33m 会在车进入可转向区之前按停（V0.0.96 现场 stop=1.0 即此死锁）；
    //   · 也不可低于 range_min：小于 range_min 的障碍在 /scan 中根本不存在，
    //     阈值低于它等于关闭碰撞闸（V0.0.91 撞墙事故根因）。
    //   另注：自车反射可达 ~0.6m，故 range_min 不宜 <0.65（否则自反射变"永久障碍"）。
    declare_parameter<double>("sector_half_deg", 60.0);    // 行进方向检测扇区半角（°）
    // V0.0.91 走廊几何判定：碰撞闸由"扇区内最小径向距离"改为"前方矩形走廊内
    // 最小纵向净空"，并用自车包络盒过滤自身反射（替代上游 range_min 粗截断）
    declare_parameter<double>("corridor_half_width", 0.45);  // 安全走廊半宽（m）
    declare_parameter<double>("footprint_front", 0.45);      // 车体前缘 x（文档 9.3）
    declare_parameter<double>("footprint_rear", 0.37);       // 车体后缘 x
    declare_parameter<double>("footprint_half_width", 0.32); // 车体半宽
    declare_parameter<double>("self_margin", 0.12);          // 自车包络外扩余量（m）
    // V0.0.95 阈值滞环 + 幽灵点门控（消除“原地抖动不前进”与单点误急停）
    //   release_hysteresis：进入用 stop/slow_dist，【退出】用 +hysteresis。
    //   现场日志净空在阈值附近 ±6mm 抖动（0.991↔1.006m）时，旧实现每 0.1~0.2s
    //   在 SLOWDOWN↔COLLISION_STOP 间往返切换（56s 内 47 次），速度被反复归零 →
    //   车辆“抖动但不前进”，且日志被状态转移刷屏。
    //   min_obstacle_points/obstacle_cluster_span：走廊内最近回波纵向 ±span 内的
    //   回波点数少于 min_obstacle_points 时判为孤立噪点（单束噪声/玻璃反光/
    //   雨雾/幽灵点）忽略；真障碍（墙/人/车）在 16 线雷达上同一纵向跨度内必有
    //   数点以上回波。默认 3 点/±0.25m 兼顾“薄立柱”与“噪点抑制”。
    declare_parameter<double>("stop_release_hysteresis", 0.25);  // 退出 STOP 滞环（m）
    declare_parameter<double>("slow_release_hysteresis", 0.20);  // 退出 SLOWDOWN 滞环（m）
    declare_parameter<int>("min_obstacle_points", 3);            // 走廊内最少回波点数
    declare_parameter<double>("obstacle_cluster_span", 0.25);    // 最近回波簇纵向跨度（m）
    // V0.0.98 轨迹扫掠弧碰撞闸（"精确计算转向角度/速度以绕障"的算法核心）：
    //   reaction_lag：指令 (v,w) 的执行滞后——前 0.4s 按直线积分（转向机构/底盘
    //     响应需时），其后才按指令角速度走圆弧。该设定使"贴到障碍跟前才打舵"
    //     的乐观解无法骗过安全闸：直线段先撞上去即判接触。
    //   brake_decel：弧前瞻覆盖距离所需的制动减速度，与 velocity_smoother.max_decel 一致。
    //   vel_trust_eps：车速低于此值时不信任"弧意图"（静止时按直线走廊评估），
    //     防止"停着时也判净空=∞→立即放行"的抖动。
    declare_parameter<double>("reaction_lag", 0.4);              // 指令执行滞后（s）
    declare_parameter<double>("brake_decel", 1.5);               // 制动减速度（m/s²）
    declare_parameter<double>("vel_trust_eps", 0.03);            // 弧判据最低车速（m/s）
    declare_parameter<double>("scan_timeout", 0.5);        // /scan 超时（s）
    declare_parameter<double>("cmd_timeout", 0.5);         // 上游指令超时（s）
    declare_parameter<double>("control_rate", 20.0);       // 主循环频率（Hz）
    // V0.0.86 自动驾驶测试模式
    declare_parameter<bool>("enable_test_mode", false);    // 启动即进入测试模式
    declare_parameter<double>("test_max_linear_vel", 0.1); // 测试模式限速（m/s）
    declare_parameter<double>("test_stop_dist", 0.90);      // 测试模式急停距离（m，V0.0.97 与正式模式同源）
    declare_parameter<double>("test_slow_dist", 1.40);      // 测试模式减速距离（m）

    declare_parameter<double>("stall_timeout", 1.0);       // 卡死判定时长（s）
    declare_parameter<double>("stall_cmd_vel_min", 0.05);  // 判"有指令"的最小指令速度（m/s）
    declare_parameter<double>("stall_fb_vel_max", 0.02);   // 判"没在动"的最大反馈速度（m/s）
    declare_parameter<double>("plan_fail_timeout", 2.0);   // 控制器指令断流判定（s）
    // V0.0.89 窄小测试场地：放宽起步期误判——首个航点尚不可规划/非运动恢复
    // 的 Wait 会造成 goal 短暂 ABORTED 与 /cmd_vel_nav 断流，不应一票否决整个测试。
    //   test_max_goal_aborts : 连续 ABORTED 达此次数才锁存中止（默认 1=旧行为）
    //   EXECUTING（车确实在动）会清零计数；行驶中再次 ABORT 仍会中止，安全网保留
    declare_parameter<int>("test_max_goal_aborts", 1);
    declare_parameter<bool>("estop_hold_on_abort", true);  // 中止时是否发布 /estop=true
    // V0.0.87 地图边界监护
    declare_parameter<bool>("enable_map_fence", true);     // 边界监护开关
    declare_parameter<double>("map_edge_stop_dist", 0.5);  // 距未建图/界外栅格停车距离（m）
    declare_parameter<double>("map_edge_slow_dist", 1.5);  // 距未建图/界外栅格减速距离（m）

    wheelbase_ = get_parameter("wheelbase").as_double();
    min_turn_radius_ = get_parameter("min_turn_radius").as_double();
    max_linear_vel_ = get_parameter("max_linear_vel").as_double();
    stop_dist_ = get_parameter("stop_dist").as_double();
    slow_dist_ = get_parameter("slow_dist").as_double();
    sector_half_rad_ = get_parameter("sector_half_deg").as_double() * M_PI / 180.0;
    corridor_half_width_ = get_parameter("corridor_half_width").as_double();
    footprint_front_ = get_parameter("footprint_front").as_double();
    footprint_rear_ = get_parameter("footprint_rear").as_double();
    footprint_half_width_ = get_parameter("footprint_half_width").as_double();
    self_margin_ = get_parameter("self_margin").as_double();
    stop_release_hysteresis_ = get_parameter("stop_release_hysteresis").as_double();
    slow_release_hysteresis_ = get_parameter("slow_release_hysteresis").as_double();
    min_obstacle_points_ = get_parameter("min_obstacle_points").as_int();
    obstacle_cluster_span_ = get_parameter("obstacle_cluster_span").as_double();
    reaction_lag_ = get_parameter("reaction_lag").as_double();
    brake_decel_ = get_parameter("brake_decel").as_double();
    vel_trust_eps_ = get_parameter("vel_trust_eps").as_double();
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
     test_max_goal_aborts_ = get_parameter("test_max_goal_aborts").as_int();
     if (test_max_goal_aborts_ < 1) {
       test_max_goal_aborts_ = 1;
     }
    estop_hold_on_abort_ = get_parameter("estop_hold_on_abort").as_bool();
    enable_map_fence_ = get_parameter("enable_map_fence").as_bool();
    map_edge_stop_dist_ = get_parameter("map_edge_stop_dist").as_double();
    map_edge_slow_dist_ = get_parameter("map_edge_slow_dist").as_double();

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
    // V0.0.91 走廊必须严于自车包络，否则自身反射会被当成"前方障碍"常年急停；
    // 二者矛盾时以包络为准放宽走廊（宁可漏报由 costmap 层兜底，不可误报到不能走）
    if (corridor_half_width_ <= footprint_half_width_ + self_margin_) {
      RCLCPP_WARN(get_logger(),
        "corridor_half_width=%.2f ≤ 车体半宽+余量 %.2f，走廊会看到自身反射，已放宽到 %.2f",
        corridor_half_width_, footprint_half_width_ + self_margin_,
        footprint_half_width_ + self_margin_ + 0.05);
      corridor_half_width_ = footprint_half_width_ + self_margin_ + 0.05;
    }
    // V0.0.95 滞环与幽灵点门控参数合法性校验
    if (stop_release_hysteresis_ < 0.0) {
      RCLCPP_WARN(get_logger(), "stop_release_hysteresis < 0，按 0 处理（无滞环）");
      stop_release_hysteresis_ = 0.0;
    }
    if (slow_release_hysteresis_ < 0.0) {
      RCLCPP_WARN(get_logger(), "slow_release_hysteresis < 0，按 0 处理（无滞环）");
      slow_release_hysteresis_ = 0.0;
    }
    if (min_obstacle_points_ < 1) {
      RCLCPP_WARN(get_logger(), "min_obstacle_points < 1，按 1 处理（不抑制单点）");
      min_obstacle_points_ = 1;
    }
    if (obstacle_cluster_span_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "obstacle_cluster_span ≤ 0，按 0.25m 处理");
      obstacle_cluster_span_ = 0.25;
    }
    if (reaction_lag_ < 0.0) {
      RCLCPP_WARN(get_logger(), "reaction_lag < 0，按 0.4s 处理");
      reaction_lag_ = 0.4;
    }
    if (brake_decel_ < 0.3) {
      RCLCPP_WARN(get_logger(), "brake_decel=%.2f 过小（弧前瞻会爆炸），按 1.5m/s² 处理", brake_decel_);
      brake_decel_ = 1.5;
    }
    if (vel_trust_eps_ < 0.01) {
      RCLCPP_WARN(get_logger(), "vel_trust_eps=%.3f 过小，按 0.03m/s 处理", vel_trust_eps_);
      vel_trust_eps_ = 0.03;
    }
    if (!enable_map_fence_) {
      RCLCPP_WARN(get_logger(),
        "地图边界监护已关闭（enable_map_fence=false）：越界防护仅剩 Nav2 规划层"
        "（track_unknown_space）与 auto_mission 航点校验，行驶中无兜底");
    } else if (map_edge_stop_dist_ >= map_edge_slow_dist_) {
      RCLCPP_WARN(get_logger(),
        "map_edge_stop_dist ≥ map_edge_slow_dist，修正 map_edge_slow_dist=stop+0.5");
      map_edge_slow_dist_ = map_edge_stop_dist_ + 0.5;
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

    // V0.0.87 地图边界监护数据源：
    //   /map —— map_server 以 transient_local 发布一次（volatile 订阅会漏收，
    //   同 V0.0.82 auto_mission 教训）；缓存后构建"到最近未建图(unknown)/
    //   界外栅格"距离场，静态地图仅构建一次；
    //   /relocalization/pose —— V0.0.93 方案A：hunter_relocalization(NDT) 发布的
    //   map 系位姿（原 /amcl_pose，AMCL 已移除）。仅导航模式存在（~2Hz），
    //   监护足够。建图模式两者皆无 → 监护自动不介入。
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&SafetyGuard::mapCallback, this, std::placeholders::_1));
    amcl_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/relocalization/pose", rclcpp::SensorDataQoS(),
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(data_mutex_);
        fence_pose_x_ = msg->pose.pose.position.x;
        fence_pose_y_ = msg->pose.pose.position.y;
        fence_pose_received_ = true;
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
      "stop=%.2fm, slow=%.2fm, 走廊±%.2fm（车体包络 %.2f/%.2f/%.2f+m%.2f）, "
      "扇区±%.0f°, scan超时%.2fs；"
      "释放滞环=+%.2f/+%.2f m（退 STOP/退 SLOWDOWN，消除阈值抖振）；"
      "幽灵点门控=≥%d 点/±%.2fm（少于则判孤立噪点忽略）；"
      "V0.0.98 轨迹扫掠弧=行进中启用（v>%.2fm/s 时按指令(v,w)积分判净空，静止用直线走廊；"
      "反应滞后%.1fs，制动%.1fm/s²，前瞻=slow+v²/2a+0.3m≤3m）；"
      "测试模式=%s（限速%.2fm/s, 急停%.1fm, 减速%.1fm, 卡死判定%.1fs, 断流判定%.1fs）；"
      "地图边界监护=%s（停车%.2fm, 减速%.2fm，/map+/relocalization/pose 就绪后生效）",
      max_linear_vel_, min_turn_radius_, stop_dist_, slow_dist_,
      corridor_half_width_, footprint_front_, footprint_rear_, footprint_half_width_,
      self_margin_, sector_half_rad_ * 180.0 / M_PI, scan_timeout_,
      stop_release_hysteresis_, slow_release_hysteresis_,
      min_obstacle_points_, obstacle_cluster_span_,
      vel_trust_eps_, reaction_lag_, brake_decel_,
      test_mode_ ? "ON" : "OFF",
      test_max_linear_vel_, test_stop_dist_, test_slow_dist_,
      stall_timeout_, plan_fail_timeout_,
      enable_map_fence_ ? "ON" : "OFF", map_edge_stop_dist_, map_edge_slow_dist_);
  }

private:
  enum class State
  {
    OK, SLOWDOWN, COLLISION_STOP, SCAN_TIMEOUT, CMD_TIMEOUT, ESTOP, TEST_ABORTED,
    MAP_EDGE_SLOWDOWN, MAP_EDGE_STOP
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
      case State::MAP_EDGE_SLOWDOWN: return "MAP_EDGE_SLOWDOWN";
      case State::MAP_EDGE_STOP: return "MAP_EDGE_STOP";
    }
    return "UNKNOWN";
  }

  // 行进方向“安全走廊”净空（m）：把极坐标回波投影到车体系，
  //   x = 沿行进方向的纵向净空（正=前方），y = 横向偏移
  // 判定域为矩形走廊 [车体边缘, +∞) × |y| ≤ corridor_half_width，而非原来的
  // “扇区内最小径向距离 r”。两个关键差异（V0.0.91 撞墙事故直接相关）：
  //   ① 用纵向净空 x 而非斜距 r：斜 45° 的墙角 r=0.70 → x=0.50，旧逻辑按 0.70
  //     处理会漏判（实际纵向只剩 0.5m 已不够制动）；
  //   ② 用横向走廊限制而非扇区半角：平行侧墙（|y|≈0.5）不再常年触发急停，
  //     窄场地里“因为怕误停而把阈值调得过小”的恶性循环得以解除。
  // 自车包络（footprint + self_margin 外扩）内的回波直接丢弃，取代上游
  //   pointcloud_to_laserscan 用大 range_min 粗截断的做法——后者会在车前
  //   造出“越近越安全”的盲区（本次撞墙根因：range_min 0.8 > stop_dist 0.5）。
  // 返回 false 表示 /scan 缺失或超时。返回 true 时输出：
  //   d_clear     —— 走廊内最小纵向净空（m，有效障碍），无有效障碍 = +inf
  //   y_at        —— 最近回波横向偏移（诊断用，即使被判噪点也保留）
  //   raw_x       —— 走廊内最近回波纵向距离（含将被丢弃的噪点，诊断用）
  //   cluster_pts —— 最近回波纵向 ±obstacle_cluster_span_ 内的回波点数
  // V0.0.95 幽灵点门控：cluster_pts < min_obstacle_points_ 时视为孤立噪点
  //   （单束噪声/玻璃反光/雨雾），d_clear 保持 +inf 不参与制动；
  //   真障碍（墙/人/车/货架）在 16 线雷达同一纵向跨度内必有数点以上回波。
  bool corridorClearance(
    bool forward, double & d_clear, double & y_at, double & raw_x, int & cluster_pts)
  {
    std::lock_guard<std::mutex> lk(data_mutex_);
    d_clear = std::numeric_limits<double>::infinity();
    y_at = 0.0;
    raw_x = std::numeric_limits<double>::infinity();
    cluster_pts = 0;
    if (!scan_received_ || (now() - last_scan_time_).seconds() > scan_timeout_) {
      return false;
    }

    const auto & scan = last_scan_;
    // 行进方向中心角：前进 0（x 前），倒车 π
    const double center = forward ? 0.0 : M_PI;
    // 自车包络过滤线：沿行进方向的车体边缘 + 余量，及横向半宽 + 余量
    const double self_x = (forward ? footprint_front_ : footprint_rear_) + self_margin_;
    const double self_y = footprint_half_width_ + self_margin_;

    // 第 1 遍：走廊内最小纵向净空
    double min_x = std::numeric_limits<double>::infinity();
    double min_y = 0.0;
    double angle = static_cast<double>(scan.angle_min);
    for (const float r : scan.ranges) {
      // 归一化角度差到 [-π, π]
      const double rel = std::remainder(angle - center, 2.0 * M_PI);
      angle += static_cast<double>(scan.angle_increment);
      if (std::fabs(rel) > sector_half_rad_) {
        continue;                        // 扇区外（含侧后方）不参与行进方向判定
      }
      if (!std::isfinite(r) ||
        r < static_cast<double>(scan.range_min) ||
        r > static_cast<double>(scan.range_max))
      {
        continue;                        // inf/无效回波（use_inf=true 下无障碍）
      }
      const double x = static_cast<double>(r) * std::cos(rel);
      const double y = static_cast<double>(r) * std::sin(rel);
      if (x <= 0.0 || std::fabs(y) > corridor_half_width_) {
        continue;                        // 走廊外：不构成行进方向威胁
      }
      if (x <= self_x && std::fabs(y) <= self_y) {
        continue;                        // 自车包络内：车身/支架自身反射
      }
      if (x < min_x) {
        min_x = x;
        min_y = y;
      }
    }
    if (!std::isfinite(min_x)) {
      return true;                       // 走廊内全为 inf/无效回波 → 无障碍
    }
    raw_x = min_x;
    y_at = min_y;

    // 第 2 遍：最近回波纵向跨度内的回波点数（幽灵点门控依据）
    int pts = 0;
    const double span = min_x + obstacle_cluster_span_;
    angle = static_cast<double>(scan.angle_min);
    for (const float r : scan.ranges) {
      const double rel = std::remainder(angle - center, 2.0 * M_PI);
      angle += static_cast<double>(scan.angle_increment);
      if (std::fabs(rel) > sector_half_rad_) {
        continue;
      }
      if (!std::isfinite(r) ||
        r < static_cast<double>(scan.range_min) ||
        r > static_cast<double>(scan.range_max))
      {
        continue;
      }
      const double x = static_cast<double>(r) * std::cos(rel);
      const double y = static_cast<double>(r) * std::sin(rel);
      if (x <= 0.0 || std::fabs(y) > corridor_half_width_) {
        continue;
      }
      if (x <= self_x && std::fabs(y) <= self_y) {
        continue;
      }
      if (x <= span) {
        ++pts;
      }
    }
    cluster_pts = pts;
    if (pts < min_obstacle_points_) {
      return true;                       // 孤立回波：d_clear 保持 +inf（噪点抑制）
    }
    d_clear = min_x;
    return true;
    // 走廊内全为 inf/无效回波 → +inf：视为无障碍。车前扇区全无回波属异常场景，
    // 由 SCAN_TIMEOUT 看门狗、costmap 的 scan 清障层与 health_monitor 兜底。
  }

  // V0.0.98 轨迹扫掠弧净空（"精确计算转向角度/速度以绕障"的算法核心）：
  // 把指令 (v,w) 按阿克曼运动学积分成候选轨迹——前 reaction_lag_ 秒按直线行进
  //   （转向机构/底盘响应需时，"贴到障碍跟前才打舵"的规避解在直线段即判接触），
  //   其后按指令角速度走圆弧（|w| 钳制到 |v|/R_min，与第 7 步曲率钳制一致，
  //   物理不可执行的弧不予采信）。
  // 输出与直线走廊同量纲：d_arc = 首次接触时已行驶弧长 + 行进方向车体边缘到
  //   中心的距离（把"弧上何时撞上"折算回"当前纵向等价净空"，STOP/SLOWDOWN
  //   阈值与滞环无需任何改动即可复用）。
  // 候选点筛选不再限扇区半角：绕障弧线会侧向扫到初始角度 60°+ 的障碍
  //   （墙角/桌角），按"扫掠域内接圆盒"粗选（斜边长 ≤ s_need 的矩形）。
  // 幽灵点门控：接触后 0.2s 窗口内累计接触点数 < min_obstacle_points_ 判孤立
  //   噪点（真障碍碰盒角必有连续多点），d_arc 保持 +inf 不参与制动。
  // 返回 false = /scan 缺失或超时（与 corridorClearance 同语义）。
  // 算力：候选点 ≤ 扫掠域内回波 × 积分步 ≤ 60（darc=5cm，前瞻 ≤3m，与车速无关），
  //   每拍 ≤ 2.2万次盒内判定（若干乘加），20Hz 下 <2% 单核——远低于被砍的
  //   MPPI batch 开销。
  bool sweptArcClearance(
    double v, double w, double slow_d, double & s_contact, int & hit_pts)
  {
    s_contact = std::numeric_limits<double>::infinity();
    hit_pts = 0;
    // 锁内拷贝扫描快照（与 corridorClearance 同样的并发约定）
    sensor_msgs::msg::LaserScan scan;
    {
      std::lock_guard<std::mutex> lk(data_mutex_);
      if (!scan_received_ || (now() - last_scan_time_).seconds() > scan_timeout_) {
        return false;
      }
      scan = last_scan_;
    }

    // 车体包络盒（含 self_margin 外扩）：在"航向系"（+x=行进方向）下积分，
    //   lead_edge=沿行进方向车体边缘，trail_edge=其反方向边缘
    const double front = footprint_front_ + self_margin_;
    const double rear = footprint_rear_ + self_margin_;
    const double half = footprint_half_width_ + self_margin_;
    const bool forward = (v >= 0.0);
    const double lead_edge = forward ? front : rear;    // 沿行进方向车体边缘
    const double trail_edge = forward ? rear : front;   // 逆行进方向车体边缘

    // 前瞻弧长：slow 线 + 本车速制动距离 + 0.3m 余量，上限 3m（≈低速 4s，
    //   与 MPPI 预测时域同量级；更远处下一拍会重新评估，无需一次扫完）
    const double speed = std::fabs(v);
    const double s_need = std::min(
      slow_d + speed * speed / (2.0 * std::max(brake_decel_, 0.1)) + 0.3, 3.0);

    // 指令角速度钳制：与第 7 步阿克曼曲率钳制同式，保证评估的弧真实可执行
    const double w_lim = speed / min_turn_radius_;
    const double w_cmd = std::clamp(w, -w_lim, w_lim);

    // 候选回波 → 航向系坐标（倒车时以 -x 为"行进前方"），按扫掠域盒粗选
    std::vector<std::pair<double, double>> pts;
    pts.reserve(scan.ranges.size());
    double angle = static_cast<double>(scan.angle_min);
    for (const float r : scan.ranges) {
      const double rr = static_cast<double>(r);
      const double a = angle;
      angle += static_cast<double>(scan.angle_increment);
      if (!std::isfinite(rr) ||
        rr < static_cast<double>(scan.range_min) ||
        rr > static_cast<double>(scan.range_max))
      {
        continue;
      }
      const double gpx = rr * std::cos(a);   // base_link 系
      const double gpy = rr * std::sin(a);
      const double px = forward ? gpx : -gpx;  // 航向系：+x = 行进方向
      const double py = forward ? gpy : -gpy;
      // 扫掠域粗选：弧任一点的相对位置必落在该盒内
      if (px > -(trail_edge + 0.05) && px < s_need + lead_edge + 0.05 &&
        std::fabs(py) < half + s_need)
      {
        pts.emplace_back(px, py);
      }
    }
    if (pts.empty()) {
      return true;                          // 扫掠域无障碍：净空 = inf
    }

    // 轨迹积分（按弧长等步长，darc=5cm → 步数恒 ≤ s_need/0.05 ≤ 60，与车速无关）：
    // 位姿 (X,Y,θ) 在航向系下描述当前车位；航向系始终以速度方向为 +x
    //   （前进/倒车皆然，倒车时航向系基座旋转 π，速度方向变化率仍为 w）；
    // 每步把候选点变换进步中车体系做包络盒接触判定；反应滞后期内
    //   （已行驶弧长 ≤ speed×reaction_lag_）w=0 直行。
    const double darc = 0.05;
    const int steps = std::max(1, static_cast<int>(std::ceil(s_need / darc)));
    const double s_lag = speed * reaction_lag_;          // 滞后对应的直线弧长
    const double s_tail = std::max(0.2 * speed, 0.1);    // 幽灵点续扫弧长窗口
    double X = 0.0, Y = 0.0, th = 0.0, arc = 0.0;
    int first_hit_step = -1;
    double first_hit_arc = 0.0;
    std::vector<char> touched(pts.size(), 0);
    for (int i = 1; i <= steps; ++i) {
      const double w_eff = (arc + darc <= s_lag) ? 0.0 : w_cmd;
      X += darc * std::cos(th);
      Y += darc * std::sin(th);
      th += w_eff * darc / std::max(speed, 0.05);          // dθ = w·dt = w·darc/v
      arc += darc;
      const double ct = std::cos(th), st = std::sin(th);
      int step_hits = 0;
      for (size_t k = 0; k < pts.size(); ++k) {
        const double dx = pts[k].first - X;
        const double dy = pts[k].second - Y;
        const double lx = dx * ct + dy * st;      // 步中车体系纵向（行进方向）
        const double ly = -dx * st + dy * ct;     // 步中车体系横向
        if (lx <= lead_edge && lx >= -trail_edge && std::fabs(ly) <= half) {
          if (!touched[k]) {
            touched[k] = 1;
            ++hit_pts;
          }
          ++step_hits;
        }
      }
      if (step_hits > 0) {
        if (first_hit_step < 0) {
          first_hit_step = i;
          first_hit_arc = arc;
          s_contact = arc + lead_edge;            // 折算回"纵向等价净空"
        }
        if (arc - first_hit_arc >= s_tail) {
          break;                                   // 续扫窗口足够判簇，提前终止
        }
      }
    }
    // 幽灵点门控（弧版）：接触簇点数不足 → 噪点放行（诊断日志由 tick 打）
    if (std::isfinite(s_contact) && hit_pts < min_obstacle_points_) {
      s_contact = std::numeric_limits<double>::infinity();
    }
    return true;
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
    //    V0.0.91：此处只查新鲜度，净空测量在第 4 步按实际行进方向算一次
    //    （旧实现先按“前进”扫一遍取 scan_ok，倒车时白算且方向可能与指令相反）
    bool scan_ok = false;
    {
      std::lock_guard<std::mutex> lk(data_mutex_);
      scan_ok = scan_received_ &&
        (now_t - last_scan_time_).seconds() <= scan_timeout_;
    }
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
    //    V0.0.98 双判据：静止/低速（|v|≤vel_trust_eps）用直线走廊净空；
    //    行驶中（见 4.7）改按实际指令 (v,w) 的轨迹扫掠弧净空【替换】直线走廊——
    //    不能取两者最小：绕障横移期间直线投影恒有障碍，取小等于回到
    //    V0.0.97 "车被自己的安全闸按停"的几何死锁。静止时保留直线判据防抖
    //    （避免"停着时弧净空=∞→放行→带舵起步撞上去"的极限环）；直线走廊的
    //    横向/点数诊断量 y_lat/cluster_pts 两种状态下都照常刷新。
    double v_in = 0.0, w_in = 0.0;
    {
      std::lock_guard<std::mutex> lk(data_mutex_);
      v_in = last_cmd_.linear.x;
      w_in = last_cmd_.angular.z;
    }
    const bool forward = v_in >= 0.0;
    const bool moving = std::fabs(v_in) > vel_trust_eps_;
    double dist = 0.0;
    double y_lat = 0.0;
    double raw_x = 0.0;
    int cluster_pts = 0;
    if (!corridorClearance(forward, dist, y_lat, raw_x, cluster_pts)) {
      // 指令快照与取数之间存在并发窗口，期间 /scan 刚好断流 → 同样 fail-safe
      publishCmd(0.0, 0.0);
      transition(State::SCAN_TIMEOUT, "/scan 在指令快照期间超时，fail-safe 停车");
      return;
    }
    // V0.0.95 幽灵点抑制诊断：走廊内最近回波点数不足被判噪点时给出可判读日志
    // （区分“真障碍挡路”与“单点噪声/玻璃反光造成的假急停”）
    if (std::isfinite(raw_x) && cluster_pts < min_obstacle_points_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "走廊内最近回波 x=%.2fm（侧偏 %.2fm）仅 %d 点 < 最少 %d 点（±%.2fm 跨度），"
        "判为孤立噪点忽略（幽灵点抑制）",
        raw_x, y_lat, cluster_pts, min_obstacle_points_, obstacle_cluster_span_);
    }

    // 4.5 V0.0.86 测试模式生效参数：0.1m/s 限速 + 更严碰撞阈值
    const double v_lim = test_active ? test_max_linear_vel_ : max_linear_vel_;
    double stop_d = test_active ? test_stop_dist_ : stop_dist_;
    double slow_d = test_active ? test_slow_dist_ : slow_dist_;
    const char * test_tag = test_active ? "【测试模式】" : "";

    // 4.6 V0.0.91 盲区一致性强制：/scan 小于 range_min 的回波在上游投影阶段
    //   已被丢弃，若急停阈值 ≤ range_min，“急停”在数学上不可达——本次撞墙
    //   事故的确切机制（range_min=0.8 而 stop_dist=0.5）：碰撞闸全程未触发，
    //   日志最近障碍恒为 0.800m 地板值，障碍再靠近就从 /scan 消失→回到 OK
    //   全速放行，直到撞上。这里按 range_min 强制抬升有效急停距离并一次性告警，
    //   使参数矛盾不可能再“静默失效”。
    {
      std::lock_guard<std::mutex> lk(data_mutex_);
      const double blind_floor = static_cast<double>(last_scan_.range_min) + 0.15;
      if (stop_d <= blind_floor) {
        if (!blind_zone_warned_) {
          blind_zone_warned_ = true;
          RCLCPP_ERROR(get_logger(),
            "急停距离 %.2fm ≤ /scan range_min %.2fm + 0.15m 余量：碰撞闸存在盲区，"
            "已按 %.2fm 强制抬升。请修正 launch 参数（stop_dist 必须大于 range_min）",
            stop_d, static_cast<double>(last_scan_.range_min), blind_floor);
        }
        stop_d = blind_floor;
        if (slow_d <= stop_d) {
          slow_d = stop_d + 0.5;
        }
      }
    }

    // 4.7 V0.0.98 行驶中扫掠弧判据：按实际指令 (v,w) 积分的候选轨迹评净空，
    //   替换（非取小）直线走廊——阿克曼绕障必须横移离开直线走廊
    //   （提前转向距离 s ≥ √(2R·c) ≈ 1.33m），旧直线判据把所有合法绕障弧
    //   误拦为"碰撞"（V0.0.97 现场满舵死磕的直接成因）。需要 slow_d 定前瞻
    //   距离，故放在盲区强制之后。
    if (moving) {
      double d_arc = std::numeric_limits<double>::infinity();
      int arc_pts = 0;
      if (!sweptArcClearance(v_in, w_in, slow_d, d_arc, arc_pts)) {
        publishCmd(0.0, 0.0);
        transition(State::SCAN_TIMEOUT, "/scan 在弧净空评估期间超时，fail-safe 停车");
        return;
      }
      dist = d_arc;
      if (std::isfinite(d_arc)) {
        cluster_pts = arc_pts;   // 日志"回波点"改报弧接触簇点数（诊断更对口）
      }
    }

    // 5. 碰撞闸分级：急停 → 线性限速 → 放行
    //    dist 为本拍采纳的净空判据：静止=直线走廊；行驶=轨迹扫掠弧（V0.0.98）；
    //    均为沿行进方向的纵向/弧长等价距离，非斜距，与制动距离同一量纲
    //    V0.0.95 释放滞环：进入用 stop_dist/slow_dist，退出用 +release_hysteresis。
    //    实测（V0.0.94 现场日志）净空在阈值附近 ±6mm 抖动时，旧实现每 0.1~0.2s
    //    在 SLOWDOWN↔COLLISION_STOP 间往返切换（56s 内 47 次状态转移），速度被
    //    反复归零 → 车辆“抖动但不前进”，且日志被状态刷屏掩盖其它故障。
    //    滞环使“已停”状态只有在净空明显恢复（+0.25m）后才放行，
    //    物理含义：一次急停后必须等出足够余量再走，避免贴着阈值蹭行。
    //    ⚠ 与行为树 BackUp 的强耦合：滞环必须 < backup_dist(0.45m)，否则
    //    “倒完仍不放行”，车辆被滞环锁死原地（文档已标注）。
    const bool stop_engaged = state_ == State::COLLISION_STOP;
    const bool slow_engaged = state_ == State::SLOWDOWN;
    const double stop_release = stop_d + stop_release_hysteresis_;
    const double slow_release = slow_d + slow_release_hysteresis_;

    double v_allow = v_lim;
    std::string detail = std::string("正常放行") + test_tag;
    State next = State::OK;
    const std::string echo_info = "，回波点 " + std::to_string(cluster_pts);
    const std::string gate = moving ? "扫掠弧净空" : "直线走廊净空";
    if (dist < stop_d || (stop_engaged && dist < stop_release)) {
      publishCmd(0.0, 0.0);
      const std::string held = (dist >= stop_d)
        ? "，滞环保持（释放阈值 " + std::to_string(stop_release).substr(0, 4) + "m）"
        : "";
      transition(State::COLLISION_STOP,
        gate + " " + std::to_string(dist).substr(0, 5) +
        "m（侧偏 " + std::to_string(y_lat).substr(0, 5) + "m" + echo_info + "）< 急停距离 " +
        std::to_string(stop_d).substr(0, 4) + "m" + held + test_tag);
      return;
    }
    if (dist < slow_d || (slow_engaged && dist < slow_release)) {
      v_allow = v_lim * (dist - stop_d) / (slow_d - stop_d);
      v_allow = std::max(v_allow, 0.0);
      next = State::SLOWDOWN;
      const std::string held = (dist >= slow_d)
        ? "，滞环保持（释放阈值 " + std::to_string(slow_release).substr(0, 4) + "m）"
        : "";
      detail = gate + " " + std::to_string(dist).substr(0, 5) +
        "m（侧偏 " + std::to_string(y_lat).substr(0, 5) + "m" + echo_info + "），限速 " +
        std::to_string(v_allow).substr(0, 5) + "m/s" + held + test_tag;
    }

    // 5.5 V0.0.87 地图边界监护：行驶范围不得超出已采集（已建图）地图区域。
    // 与碰撞闸同构分级——减速区线性限速、停车区零速；测试模式越界升级为
    // 中止锁存（越界即异常，人工确认后重开）。距离场/位姿未就绪（建图模式、
    // 启动早期、重定位未输出）时不介入，由规划层与航点校验兜底。
    if (enable_map_fence_) {
      const double edge_d = mapEdgeDistance();
      if (edge_d >= 0.0) {
        if (edge_d < map_edge_stop_dist_) {
          publishCmd(0.0, 0.0);
          const std::string pos = "距未建图/界外栅格 " +
            std::to_string(edge_d).substr(0, 5) + "m < 停车距离 " +
            std::to_string(map_edge_stop_dist_).substr(0, 4) + "m";
          if (test_active) {
            const std::string reason = "地图越界：" + pos;
            abortTest(reason);
            transition(State::TEST_ABORTED, reason + "（测试模式自动中止）");
          } else {
            transition(State::MAP_EDGE_STOP,
              pos + "，零速（回到已建图区域自动恢复）");
          }
          return;
        }
        if (edge_d < map_edge_slow_dist_) {
          double v_edge = v_lim * (edge_d - map_edge_stop_dist_) /
            (map_edge_slow_dist_ - map_edge_stop_dist_);
          v_edge = std::max(v_edge, 0.0);
          if (v_edge < v_allow) {
            v_allow = v_edge;
            next = State::MAP_EDGE_SLOWDOWN;
            detail = "距未建图/界外栅格 " + std::to_string(edge_d).substr(0, 5) +
              "m，限速 " + std::to_string(v_allow).substr(0, 5) + "m/s" + test_tag;
          }
        }
      }
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

    // b) goal 被 Nav2 判 ABORTED → 局部规划失效。V0.0.89：上升沿计数，连续达
    //    test_max_goal_aborts 次才锁存中止；观察到 EXECUTING（车确实在动）则清零。
    //    避免窄场地起步期不可规划航点的瞬时 ABORT 一票否决，行驶中反复 ABORT 仍会中止。
    if (nav_status == action_msgs::msg::GoalStatus::STATUS_ABORTED) {
      if (prev_nav_status_ != static_cast<int8_t>(action_msgs::msg::GoalStatus::STATUS_ABORTED)) {
        ++goal_abort_count_;
      }
      if (goal_abort_count_ >= test_max_goal_aborts_) {
        return "局部规划失效：navigate_to_pose goal 连续 ABORTED " +
          std::to_string(goal_abort_count_) + " 次";
      }
    } else if (nav_status == static_cast<int8_t>(action_msgs::msg::GoalStatus::STATUS_EXECUTING)) {
      goal_abort_count_ = 0;
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
    prev_nav_status_ = nav_status;
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
    prev_nav_status_ = 0;
    goal_abort_count_ = 0;
    if (msg->data) {
      RCLCPP_WARN(get_logger(),
        "[测试模式] 开启：限速 %.2fm/s、急停 %.1fm、减速 %.1fm（±%.0f° 扇区），异常自动中止",
        test_max_linear_vel_, test_stop_dist_, test_slow_dist_,
        sector_half_rad_ * 180.0 / M_PI);
    } else {
      RCLCPP_WARN(get_logger(), "[测试模式] 关闭：恢复常规安全阈值");
    }
  }

  // V0.0.87 地图边界监护：/map 回调——构建"到最近未建图(unknown)/界外栅格"
  // 的近似欧氏距离场。unknown/界外格种子 0，已建图格取到最近种子的 Chamfer
  // 3-4 距离（两遍扫描，误差 <8%，对 0.5/1.5m 阈值足够）。地图静态（
  // map_server 仅发布一次）→ 构建一次性 O(width×height)，常规 0.05m 地图
  // 数万格毫秒级；运行时 O(1) 查询。
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    const int w = static_cast<int>(msg->info.width);
    const int h = static_cast<int>(msg->info.height);
    if (w <= 0 || h <= 0 || msg->info.resolution <= 0.0f) {
      RCLCPP_WARN(get_logger(), "[边界监护] 收到异常 /map（尺寸/分辨率非法），忽略");
      return;
    }
    std::vector<float> dist(static_cast<size_t>(w) * h,
      std::numeric_limits<float>::infinity());
    const int8_t * data = msg->data.data();
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        if (data[static_cast<size_t>(y) * w + x] == -1) {
          dist[static_cast<size_t>(y) * w + x] = 0.0f;   // unknown（未建图）→ 种子
        }
      }
    }
    // Chamfer 3-4 两遍距离变换（整数格距：直邻 3、对角 4，最后统一换算米）
    constexpr int CH = 3, CD = 4;
    for (int y = 0; y < h; ++y) {              // 正扫：左/上/左上/右上
      for (int x = 0; x < w; ++x) {
        const size_t i = static_cast<size_t>(y) * w + x;
        if (dist[i] == 0.0f) {
          continue;
        }
        float d = dist[i];
        if (x > 0) {                           d = std::min(d, dist[i - 1] + CH); }
        if (y > 0) {                           d = std::min(d, dist[i - w] + CH); }
        if (x > 0 && y > 0) {                  d = std::min(d, dist[i - w - 1] + CD); }
        if (x < w - 1 && y > 0) {              d = std::min(d, dist[i - w + 1] + CD); }
        dist[i] = d;
      }
    }
    for (int y = h - 1; y >= 0; --y) {         // 反扫：右/下/右下/左下
      for (int x = w - 1; x >= 0; --x) {
        const size_t i = static_cast<size_t>(y) * w + x;
        if (dist[i] == 0.0f) {
          continue;
        }
        float d = dist[i];
        if (x < w - 1) {                       d = std::min(d, dist[i + 1] + CH); }
        if (y < h - 1) {                       d = std::min(d, dist[i + w] + CH); }
        if (x < w - 1 && y < h - 1) {          d = std::min(d, dist[i + w + 1] + CD); }
        if (x > 0 && y < h - 1) {              d = std::min(d, dist[i + w - 1] + CD); }
        dist[i] = d;
      }
    }
    const float scale = static_cast<float>(msg->info.resolution) / 3.0f;
    for (auto & d : dist) {
      if (std::isfinite(d)) {
        d *= scale;
      }
    }
    {
      std::lock_guard<std::mutex> lk(data_mutex_);
      fence_dist_ = std::move(dist);
      fence_w_ = w;
      fence_h_ = h;
      fence_origin_x_ = msg->info.origin.position.x;
      fence_origin_y_ = msg->info.origin.position.y;
      fence_res_ = msg->info.resolution;
      fence_ready_ = true;
    }
    RCLCPP_INFO(get_logger(),
      "[边界监护] 已就绪：栅格 %dx%d @%.3fm/cell，车辆距未建图/界外区域 "
      "<%.2fm 零速、<%.2fm 限速（数据源 /map + /relocalization/pose）",
      w, h, msg->info.resolution, map_edge_stop_dist_, map_edge_slow_dist_);
  }

  // 查询机器人当前位置到最近未建图(unknown)/地图界外栅格的距离（m）。
  // 返回 -1.0 = 监护不可用（地图未就绪或 /relocalization/pose 未到达，调用方不介入）；
  // 位姿在栅格界外 → 0（直接视为越界）。
  double mapEdgeDistance()
  {
    std::lock_guard<std::mutex> lk(data_mutex_);
    if (!fence_ready_ || !fence_pose_received_) {
      return -1.0;
    }
    const int cx = static_cast<int>(std::floor(
        (fence_pose_x_ - fence_origin_x_) / fence_res_));
    const int cy = static_cast<int>(std::floor(
        (fence_pose_y_ - fence_origin_y_) / fence_res_));
    if (cx < 0 || cx >= fence_w_ || cy < 0 || cy >= fence_h_) {
      return 0.0;
    }
    return fence_dist_[static_cast<size_t>(cy) * fence_w_ + cx];
  }

  // 参数
  double wheelbase_{0.46};
  double min_turn_radius_{1.9};
  double max_linear_vel_{0.8};
  double stop_dist_{0.6};
  double slow_dist_{1.2};
  double sector_half_rad_{M_PI / 3.0};
  // V0.0.91 走廊几何与自车包络过滤
  double corridor_half_width_{0.45};
  double footprint_front_{0.45};
  double footprint_rear_{0.37};
  double footprint_half_width_{0.32};
  double self_margin_{0.12};
  // V0.0.95 阈值滞环与幽灵点门控
  double stop_release_hysteresis_{0.25};
  double slow_release_hysteresis_{0.20};
  int min_obstacle_points_{3};
  double obstacle_cluster_span_{0.25};
  bool blind_zone_warned_{false};   // 盲区参数矛盾仅告警一次
  // V0.0.98 轨迹扫掠弧碰撞闸
  double reaction_lag_{0.4};        // 指令执行滞后（s）
  double brake_decel_{1.5};         // 制动减速度（m/s²，与 velocity_smoother 一致）
  double vel_trust_eps_{0.03};      // 弧判据最低车速（m/s）
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
  int test_max_goal_aborts_{1};            // V0.0.89 连续 ABORTED 容忍次数
  bool estop_hold_on_abort_{true};

  // V0.0.87 地图边界监护参数
  bool enable_map_fence_{true};
  double map_edge_stop_dist_{0.5};
  double map_edge_slow_dist_{1.5};
  // 距离场与位姿缓存（data_mutex_ 保护；fence_dist_ 下标 = cy*w+cx）
  bool fence_ready_{false};
  bool fence_pose_received_{false};
  int fence_w_{0};
  int fence_h_{0};
  double fence_origin_x_{0.0};
  double fence_origin_y_{0.0};
  double fence_res_{0.05};
  double fence_pose_x_{0.0};
  double fence_pose_y_{0.0};
  std::vector<float> fence_dist_;

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
  int8_t prev_nav_status_{0};               // V0.0.89 goal 状态上升沿检测
  int goal_abort_count_{0};                 // V0.0.89 连续 ABORTED 计数
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
  // V0.0.87 地图边界监护
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
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
