"""hunter_autonomous_nav.launch.py — HUNTER 自主建图/导航一体启动文件

支持两种运行模式（通过参数 mode 切换）：
  mode:=mapping  — 建图模式（车辆由人工遥控驾驶扫描，系统侧流程自动）
      启动：fast_lio2 参数覆盖（pcd_save_en=true、map_file_path 自动注入）
            + auto_mission_node(mapping 状态，不下发导航目标——车辆由人工遥控)
            + pcd_to_map 节点（监听建图结束信号，自动完成 PCD→PGM+YAML 转换）
            + waypoint_recorder 节点（订阅 /clicked_point，自动写入 yaml）
      操作：人工遥控（REMOTE 模式）驾驶遍历全部目标区域；rviz2 用 Publish Point
            （快捷键 P）点击记录巡检航点（User_Manual §4.4）。
      结束：在启动终端按 Ctrl+C → fast_lio2 在退出瞬间（rclcpp::spin 返回后）
            才把全部点云写入 map_file_path（V0.0.90 起按 pcd_save.save_voxel_size
            =0.1m 体素去重后累加，写盘秒级完成；localization.launch.py 已给
            fast_lio2 配 120s 退出宽限，不会再被 SIGTERM 截断成半截 PCD；
            运行中文件不存在属正常）；
            Ctrl+C 会同时终止 auto_mission/pcd_to_map，运行期 MAPPING→非MAPPING
            跳变不会发生——因此 pcd_to_map 在退出时派生一个独立会话
            （setsid）的后台转换进程，等 PCD 写完整后自动生成 .pgm + .yaml
            （V0.0.90：即使 PCD 截断/写盘超时，也会按已写入部分尽力转换，
            不再直接放弃；进度：tail -f maps/pcd_to_map_final.log；
            产物：ls -lh maps/）。
      也可不退出节点，手动保存当前快照：
            ros2 service call /fast_lio2/map_save std_srvs/srv/Trigger

  mode:=nav      — 定位导航模式（默认）
      启动：fast_lio2 参数覆盖（pcd_save_en=false）
            + Nav2 全栈（使用 autonomous_navigate.xml 扩展行为树）
            + auto_mission_node（按 autonomous_nav_params.yaml 执行航点巡航）
      加载：nav2_map_server 加载已保存的静态地图（map_yaml_path 参数）
      自愈：pcd_to_map 常驻——地图 YAML 缺失而 PCD 存在时（建图结束转换
            竞态失败遗留），启动后自动补转换；本次 map_server 若已因缺图
            启动失败，转换完成后重启本 launch 即可

集成方式：
  由 hunter_full.launch.py 通过 use_autonomous_nav:=true 可选加载，
  或独立运行（需要传感器驱动、CAN驱动等已就绪）。

话题说明（参见模块契约）：
  读取：/planning/behavior_state  /system/health  /localization/odom
        /perception/fused_objects  /estop
  发布：/auto_mission/status  /auto_mission/current_waypoint  /estop
  Action客户端：/navigate_to_pose（Nav2 bt_navigator）
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


# ---------------------------------------------------------------------------
# 辅助函数
# ---------------------------------------------------------------------------
def _nav2_params_with_bt(context, *args, **kwargs):
    """运行时拼接 nav2_params 覆盖项：将 bt_navigator 默认行为树指向 autonomous_navigate.xml。

    行为树 XML 需要运行时解析包内绝对路径，所以用 OpaqueFunction 动态生成 Node。
    ⚠ 参数名必须是 default_nav_to_pose_bt_xml——本 fork（Humble）bt_navigator 只读
    该名（nav2_bt_navigator/src/navigators/navigate_to_pose.cpp:73）；旧名
    default_bt_xml_filename 会被静默忽略 → 自定义树不生效，回退内置默认树
    （默认树恢复池含 Spin，阿克曼车被命令原地旋转、转向角打满画圈，V0.0.81 实车事故）。
    """
    pkg_bringup = get_package_share_directory('hunter_bringup')
    pkg_auto    = get_package_share_directory('auto_mission')

    nav2_params_file = os.path.join(pkg_bringup, 'config', 'nav2_params.yaml')
    auto_bt_file     = os.path.join(pkg_bringup, 'behavior_trees', 'autonomous_navigate.xml')
    auto_params_file = os.path.join(pkg_auto,    'config', 'autonomous_nav_params.yaml')
    map_yaml_path    = LaunchConfiguration('map_yaml_path').perform(context)
    use_sim_time     = LaunchConfiguration('use_sim_time').perform(context)
    autostart        = LaunchConfiguration('autostart').perform(context)

    # V0.0.93 方案A：全局重定位地图 = 与 .yaml 同名的先验 .pcd（map 系全局点云地图）
    try:
        reloc_cfg_file = os.path.join(
            get_package_share_directory('hunter_relocalization'),
            'config', 'relocalization_params.yaml')
    except Exception:
        reloc_cfg_file = ''
    global_map_pcd = os.path.splitext(map_yaml_path)[0] + '.pcd'

    # 导航模式：确保 fast_lio2 不写 PCD（pcd_save_en=false），避免磁盘无限增长
    fast_lio2_nav_param_node = Node(
        package='auto_mission',
        executable='fast_lio2_param_injector',
        name='fast_lio2_param_injector',
        output='screen',
        parameters=[{
            'target_node':   'fast_lio2',
            'pcd_save_en':   False,
            'map_file_path': '',
            'use_sim_time':  use_sim_time == 'true',
        }],
    )

    lifecycle_nodes = [
        'map_server',
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'velocity_smoother',
    ]
    # V0.0.93 方案A：移除 AMCL。map→odom 由 hunter_relocalization（非 lifecycle 普通节点）提供。

    # ---- map_server：加载已保存的静态 PGM 地图 ----
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time == 'true'},
            {'yaml_filename': map_yaml_path},
        ],
    )

    # ---- controller_server（V0.0.93 方案A：MPPI 局部控制，参数见 nav2_params.yaml） ----
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )

    # ---- planner_server（SmacPlannerHybrid） ----
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file],
    )

    # ---- behavior_server ----
    # 恢复行为（BackUp/Wait）的 cmd_vel 必须与 controller 一样重映射到 cmd_vel_nav，
    # 统一经 velocity_smoother 后以 /cmd_vel 下发底盘。旧版未重映射时恢复行为
    # 直发 /cmd_vel 绕过平滑器，而正常跟随指令断链——形成"只有恢复行为能动"。
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )

    # ---- bt_navigator：使用扩展行为树 autonomous_navigate.xml ----
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            nav2_params_file,
            # ⚠ 必须用 default_nav_to_pose_bt_xml（本 fork 实际读取的参数名）；
            #   旧名 default_bt_xml_filename 被静默忽略 → 自定义树不生效。
            {'default_nav_to_pose_bt_xml': auto_bt_file},
            {'use_sim_time': use_sim_time == 'true'},
        ],
    )

    # ---- velocity_smoother ----
    # 输入 cmd_vel→cmd_vel_nav（接 controller_server / behavior_server），
    # 输出 cmd_vel_smoothed→cmd_vel_pre_safety（V0.0.82：不再直达底盘，
    # 而是先过 hunter_safety/safety_guard 碰撞闸——scan 碰撞急停/限速、
    # 阿克曼曲率钳制、速度硬限、estop 透传，再由 safety_guard 发布 /cmd_vel）。
    # 旧版缺重映射：controller 指令到不了底盘 → 车辆不动 → Failed to make progress。
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel_pre_safety'),
        ],
    )

    # ---- safety_guard（碰撞防护与运动学安全约束，V0.0.82） ----
    # 速度指令链：controller/behavior → velocity_smoother → safety_guard → 底盘。
    # 仅导航模式启动（mapping 模式 auto_mission cruise 直发 /cmd_vel，避免双发布者）。
    safety_guard = Node(
        package='hunter_safety',
        executable='safety_guard',
        name='safety_guard',
        output='screen',
        remappings=[('cmd_vel_in', '/cmd_vel_pre_safety')],
        parameters=[{
            'wheelbase': 0.65,          # HunterV2Params::wheelbase（AGX_V2 实车）
            'min_turn_radius': 1.9,     # 与 Smac minimum_turning_radius 一致
            'max_linear_vel': 0.5,      # V0.0.89 测试场地：与 desired_linear_vel/velocity_smoother 一致 0.5
            # V0.0.91 撞墙事故修正：急停距离必须严格大于 /scan 的 range_min，
            # 否则"障碍越近越看不见→越近越安全"，碰撞闸形同虚设（旧值 stop 0.5 <
            # range_min 0.8：急停在数学上不可达，日志里最近障碍恒为 0.800m 地板值）。
            # 旧值之所以被压到 0.5，是因为径向+扇区判据把平行侧墙误报成急停；
            # 现改由走廊矩形（corridor_half_width）判据消除侧墙误报，阈值可回到安全值。
            # ---- V0.0.97 阈值重标定（修"无法绕开障碍物"）----
            # 旧值 stop 1.0m 在室内窄场地构成【几何死锁】：阿克曼绕开正前方障碍需
            # "提前转向距离" s ≥ √(2R·(半宽+余量)) = √(2×1.9×0.42) ≈ 1.33m，而碰撞闸
            # 在障碍 1.0m 处就把前进指令按停 → 车永远进不到"能转向"的位置；配合
            # 旧版禁倒车（MPPI vx_min=0），现场表现为满舵 0.386rad 原地死磕 →
            # "Failed to make progress" → 倒车 0.45m → 又贴回 1.0m → 循环至 FAULT。
            # 新值 stop 0.90m：① 车体前保险杠停车净空 = 0.90 − 0.45 = 0.45m，
            #   远大于 0.5m/s 下的制动距离 v²/(2a)+v·t_lat = 0.083+0.10 ≈ 0.18m；
            # ② 与 range_min 0.70 的盲区地板 0.70+0.15=0.85m 保持 0.05m 余量
            #   （safety_guard 运行时会校验并要求 stop > range_min+0.15）；
            # ③ 配合 MPPI vx_min=-0.20 与 REEDS_SHEPP，车可"退一步再转"，把死锁
            #   变成多点掉头。
            # slow 1.40m 取"转向提前量 1.33m"量级：车一进入可转向区就已被限速到
            # ~0.3m/s，保证低速精确绕障。
            'stop_dist': 0.90,          # 行进走廊净空 < 0.90m 零速（> range_min 0.70 + 0.15）
            'slow_dist': 1.40,          # 净空 < 1.40m 线性限速
            'sector_half_deg': 60.0,    # 检测扇区半角（°）
            # V0.0.91 走廊几何：只判车前方 [车体前缘, stop/slow] × |y| ≤ 0.45 的矩形区，
            # 平行侧墙（|y|≈0.5）不再误急停；自车包络内回波按 footprint 丢弃，
            # 不再依赖上游 range_min 粗截断（文档 9.3 footprint 0.45/-0.37/±0.32）
            'corridor_half_width': 0.45,
            'footprint_front': 0.45,
            'footprint_rear': 0.37,
            'footprint_half_width': 0.32,
            'self_margin': 0.12,
            'scan_timeout': 0.5,        # /scan 断流 fail-safe（s）
            'cmd_timeout': 0.5,         # 上游指令断流看门狗（s）
            # V0.0.95 阈值滞环 + 幽灵点门控（修“阈值处原地抖动不前进/单点误急停”）：
            #   现场日志净空在阈值附近 ±6mm 抖动（0.991↔1.006m）时，旧实现每 0.1~0.2s
            #   在 SLOWDOWN↔COLLISION_STOP 间往返切换（56s 内 47 次），速度被反复归零，
            #   车辆“抖动但不前进”，并刷屏掩盖其它故障 → 引入释放滞环：已急停后须
            #   净空恢复到 stop+0.25m 才放行，已减速后须恢复到 slow+0.20m 才全速。
            #   ⚠ 与行为树 BackUp 的强耦合：滞环 < BT 的 backup_dist（0.45m），
            #     否则“倒完仍不放行”→ 车辆被滞环锁死原地（本版取 0.25 vs 0.45，
            #     余量 0.20m）。改任一值时必须同步核算。
            'stop_release_hysteresis': 0.25,   # 退出 STOP 的滞环（m）
            'slow_release_hysteresis': 0.20,   # 退出 SLOWDOWN 的滞环（m）
            # 幽灵点门控：走廊内最近回波纵向 ±0.25m 内回波点数 < 3 时判为孤立噪点
            #   （单束噪声/玻璃反光/雨雾），不作为刹车依据；真障碍必有数点以上回波。
            #   ⚠ 极细立柱/远距离薄结构可下调为 2；设为 1 即恢复旧行为（不抑制）。
            'min_obstacle_points': 3,
            'obstacle_cluster_span': 0.25,
            'enable_test_mode': False,  # 测试模式运行时经 /safety/test_mode 开关
            # V0.0.89 窄小测试场地低速档：测试模式限速抬到 0.3m/s（原 0.1 “基本不动”），
            # 碰撞阈值与正式模式同源（V0.0.97：0.90/1.40，见上方 stop_dist 注释），
            # 并容忍起步期瞬时 ABORT/断流：
            'test_max_linear_vel': 0.3,
            'test_stop_dist': 0.90,
            'test_slow_dist': 1.40,
            'plan_fail_timeout': 10.0,        # > 非运动恢复 Wait 总时长，避免清图/等待期误判断流
            'test_max_goal_aborts': 3,        # 连续 ABORTED 达 3 次才锁存中止（EXECUTING 会清零）
            # V0.0.87 地图边界监护（/map + /relocalization/pose；建图模式无源自动不介入）
            'enable_map_fence': True,   # 行驶范围不得超出已采集地图区域（行驶中最后防线）
            'map_edge_stop_dist': 0.4,  # V0.0.89 窄场地略收紧（原 0.5）；距未建图/界外 <此值零速
            'map_edge_slow_dist': 1.0,  # V0.0.89 距未建图/界外 <1.0m 线性限速（原 1.5）
            'use_sim_time': use_sim_time == 'true',
        }],
    )

    # ---- 全局定位（map→odom）：V0.0.93 方案A —— NDT 点云重定位 ----
    # TF 链：map --hunter_relocalization(NDT)--> odom --FAST-LIO2--> base_link。
    # hunter_relocalization 将 FAST-LIO2 实时点云（/cloud_registered，odom 系）配准到
    # 先验全局 .pcd（map 系），估计/持续修正 map→odom；初值默认≈建图起点（单位阵），
    # 偏差大时用 rviz2 "2D Pose Estimate"（/initialpose）或直接发 /initialpose 重置。
    # /scan（pointcloud_to_laserscan 投影）保留供 costmap 障碍层与 safety_guard 使用，
    # 不再因定位而条件化。
    cloud_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[('cloud_in', '/lidar_points'), ('scan', '/scan')],
        parameters=[{
            'target_frame': 'base_link',        # 点云 rslidar → base_link（URDF TF）
            'transform_tolerance': 0.1,
            'min_height': -0.2,                 # base_link 系高度切片：滤除地面反射
            'max_height': 0.8,                  # 拦腰高度（车顶雷达俯视场景）
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.008726646,     # 0.5°/束 → 720 束
            'scan_time': 0.1,                   # /lidar_points 10Hz
            'range_min': 0.70,                  # ⚠ 与 safety_guard 的阈值强耦合（V0.0.97：0.8→0.70）：
                                            # 小于本值的回波在投影阶段就被丢弃，车前形成盲区；
                                            # safety_guard 运行时会校验 stop_dist > 本值+0.15，
                                            # 不满足即 ERROR 并强制抬升（本地 0.70+0.15=0.85 < stop 0.90 ✔）。
                                            # 取 0.70 而非更小值的硬约束——【车体自反射】：
                                            # V0.0.88 现场实测 range_min 0.5 时 safety_guard
                                            # 常年看到 ~0.6m 假障碍而急停，即自反射可达 ~0.6m。
                                            # 0.70 留 0.10m 余量；再往下降必须先在 rviz2 确认
                                            # /scan 在 0.4~0.7m 无自车回波。
                                            # 车身自反射的第一道防线仍是 safety_guard 的 footprint
                                            # 包络盒过滤（0.45+0.12=0.57m）。
            'range_max': 50.0,
            'use_inf': True,
            'use_sim_time': use_sim_time == 'true',
        }],
    )

    # ---- hunter_relocalization（map→odom，方案A 替代 AMCL） ----
    relocalization = Node(
        package='hunter_relocalization',
        executable='ndt_relocalization_node',
        name='hunter_relocalization',
        output='screen',
        parameters=[
            ([reloc_cfg_file] if reloc_cfg_file else []),
            {
                'global_map_path': global_map_pcd,
                'use_sim_time':    use_sim_time == 'true',
            },
        ],
    )

    # ---- lifecycle_manager（含 map_server） ----
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_autonomous_nav',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time == 'true'},
            {'autostart': autostart == 'true'},
            {'node_names': lifecycle_nodes},
        ],
    )

    # ---- auto_mission_node ----
    auto_mission = Node(
        package='auto_mission',
        executable='auto_mission_node',
        name='auto_mission_node',
        output='screen',
        parameters=[
            auto_params_file,
            {'mission_mode': 'waypoint_loop'},
            {'use_sim_time': use_sim_time == 'true'},
        ],
    )

    # ---- pcd_to_map（nav 模式自愈守护） ----
    # maps/ 下只有 .pcd 而 .yaml 缺失（建图结束自动转换竞态失败遗留）时，
    # 启动后自动补转换并尝试热重载 map_server；本次 map_server 若已因缺图
    # 启动失败，转换完成后重启本 launch 即可。
    map_pcd_path = os.path.splitext(map_yaml_path)[0] + '.pcd'
    yaml_exists  = os.path.isfile(map_yaml_path)
    pcd_exists   = os.path.isfile(map_pcd_path)

    pcd_to_map_nav = Node(
        package='auto_mission',
        executable='pcd_to_map',
        name='pcd_to_map',
        output='screen',
        parameters=[{
            'pcd_file':       map_pcd_path,
            'map_output_dir': os.path.dirname(map_yaml_path),
            'map_name':       os.path.splitext(os.path.basename(map_yaml_path))[0],
            'auto_reload_map':        True,   # map_server 正常运行时转换后热重载
            'trigger_on_mapping_end': False,  # nav 模式无建图结束信号
            'use_sim_time':           use_sim_time == 'true',
        }],
    )

    precheck_logs = []
    if not yaml_exists and pcd_exists:
        precheck_logs.append(LogInfo(
            msg='[hunter_autonomous_nav] 警告：地图 YAML 缺失而 PCD 存在，'
                'pcd_to_map 将自动补转换；本次 map_server 可能启动失败，'
                '转换完成后重启本 launch 即可'))
    elif not yaml_exists:
        precheck_logs.append(LogInfo(
            msg='[hunter_autonomous_nav] 错误：地图 YAML 与 PCD 均不存在，'
                '请先完成建图（use_autonomous_nav:=true autonomous_nav_mode:=mapping）'))

    return precheck_logs + [
        fast_lio2_nav_param_node,
        map_server,
        relocalization,
        cloud_to_scan,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        velocity_smoother,
        safety_guard,
        lifecycle_manager,
        auto_mission,
        pcd_to_map_nav,
    ]


def _mapping_nodes(context, *args, **kwargs):
    """建图模式：
      1. 通过 SetParameter 方式向已运行的 fast_lio2 节点注入 pcd_save_en=true 和
         map_file_path（ROS2 中无法在运行后直接覆盖另一节点的启动参数，
         但 fast_lio2 在节点析构时读取 pcd_save_en 标志写出文件，
         因此这里额外启动一个参数设置节点在 fast_lio2 启动后立即注入参数）。
      2. 启动 auto_mission_node（mapping 模式，不下发导航目标）。
      3. 启动 pcd_to_map 节点（监听 /auto_mission/status，建图结束自动转换 PCD→PGM）。
      4. 启动 waypoint_recorder 节点（订阅 /clicked_point，自动写入 yaml）。
    """
    pkg_auto         = get_package_share_directory('auto_mission')
    auto_params_file = os.path.join(pkg_auto, 'config', 'autonomous_nav_params.yaml')
    map_file_path    = LaunchConfiguration('map_file_path').perform(context)
    map_output_dir   = str(Path(map_file_path).parent)
    map_name         = Path(map_file_path).stem      # 去掉 .pcd 后缀作为地图名
    use_sim_time     = LaunchConfiguration('use_sim_time').perform(context)
    params_file_path = LaunchConfiguration('params_file_path').perform(context)
    final_wait       = LaunchConfiguration('final_convert_wait_timeout').perform(context)

    # ------------------------------------------------------------------
    # 1. 向 fast_lio2 节点设置 pcd_save 参数
    #    fast_lio2 节点名为 'fast_lio2'（见 localization.launch.py）
    #    使用 ros2 的 SetParametersAtom —— launch_ros 提供的参数注入方式
    #    实现：启动一个专用的参数代理节点，在 on_activate 时调用 /fast_lio2/set_parameters
    # ------------------------------------------------------------------
    fast_lio2_param_node = Node(
        package='auto_mission',
        executable='fast_lio2_param_injector',
        name='fast_lio2_param_injector',
        output='screen',
        parameters=[{
            'target_node':    'fast_lio2',
            'pcd_save_en':    True,
            'map_file_path':  map_file_path,
            'pcd_save_interval': -1,          # -1 = 全部帧合并为一个 PCD 文件
            'use_sim_time':   use_sim_time == 'true',
        }],
    )

    # ------------------------------------------------------------------
    # 2. auto_mission_node（mapping 模式）
    # ------------------------------------------------------------------
    auto_mission_mapping = Node(
        package='auto_mission',
        executable='auto_mission_node',
        name='auto_mission_node',
        output='screen',
        parameters=[
            auto_params_file,
            {'mission_mode': 'mapping'},
            # start_mapping_cruise 服务触发时从此文件热重载 waypoints
            # （waypoint_recorder 在建图过程中持续写入同一文件）
            {'params_file': auto_params_file},
            {'use_sim_time': use_sim_time == 'true'},
        ],
    )

    # ------------------------------------------------------------------
    # 3. pcd_to_map 节点：建图结束时自动 PCD → PGM + YAML
    # ------------------------------------------------------------------
    pcd_to_map = Node(
        package='auto_mission',
        executable='pcd_to_map',
        name='pcd_to_map',
        output='screen',
        parameters=[{
            'pcd_file':               map_file_path,
            'map_output_dir':         map_output_dir,
            'map_name':               map_name,
            'resolution':             0.05,
            'z_min':                  0.3,   # 过滤地面反射及车体自身结构，避免原点障碍
            'z_max':                  2.0,
            'occupied_thresh':        0.65,
            'free_thresh':            0.25,
            'padding_m':              0.5,
            'clear_origin_radius':    1.0,   # 建图起点周围 1m 清空，确保起步位置无障碍
            'auto_reload_map':        False,   # 建图模式下 map_server 未启动，禁用重载
            'trigger_on_mapping_end': True,
            'convert_on_start_if_missing': False,  # 建图开始时无图可转，禁用启动自愈
            # ---- 退出兜底转换（V0.0.88）----
            # Ctrl+C 时 auto_mission/pcd_to_map 与 fast_lio2 同时被 SIGINT 终止，
            # 运行期 MAPPING→非MAPPING 跳变不会发生 → 原自动转换从不启动；
            # 由本节点在退出路径派生独立会话的后台转换进程，等 FAST-LIO2 把
            # PCD 写完整后生成 .pgm/.yaml（日志 maps/pcd_to_map_final.log）
            'final_convert_on_shutdown': True,
            'final_wait_timeout': float(final_wait),
            'convert_wait_timeout': 60.0,   # 运行期触发（map_save 后）等待写盘上限
            'use_sim_time':           use_sim_time == 'true',
        }],
    )

    # ------------------------------------------------------------------
    # 4. waypoint_recorder 节点：订阅 /clicked_point 自动写入 yaml
    # ------------------------------------------------------------------
    waypoint_recorder = Node(
        package='auto_mission',
        executable='waypoint_recorder',
        name='waypoint_recorder',
        output='screen',
        parameters=[{
            'params_file':  params_file_path,
            'auto_save':    True,
            'yaw_default':  0.0,
            'use_sim_time': use_sim_time == 'true',
        }],
    )

    return [
        fast_lio2_param_node,
        auto_mission_mapping,
        pcd_to_map,
        waypoint_recorder,
    ]


# ---------------------------------------------------------------------------
# generate_launch_description
# ---------------------------------------------------------------------------
def generate_launch_description():
    pkg_bringup = get_package_share_directory('hunter_bringup')
    pkg_auto    = get_package_share_directory('auto_mission')

    # ---- 参数声明 ----

    # 运行模式：nav（导航）或 mapping（建图）
    declare_mode = DeclareLaunchArgument(
        'mode',
        default_value='nav',
        description='运行模式：nav=定位导航模式（默认），mapping=建图模式',
    )

    # 导航模式下的静态地图 YAML 路径
    declare_map_yaml = DeclareLaunchArgument(
        'map_yaml_path',
        default_value='/home/agilex/HunterEdge/maps/hunter_map.yaml',
        description='[nav 模式] nav2_map_server 加载的地图 YAML 文件绝对路径',
    )

    # 建图模式下 PCD 保存路径（传递给 fast_lio2 参数）
    declare_map_file = DeclareLaunchArgument(
        'map_file_path',
        default_value='/home/agilex/HunterEdge/maps/hunter_map.pcd',
        description='[mapping 模式] FAST-LIO2 保存 PCD 文件的绝对路径',
    )

    # waypoint_recorder 写入的 yaml 配置文件路径
    declare_params_file_path = DeclareLaunchArgument(
        'params_file_path',
        default_value=os.path.join(
            get_package_share_directory('auto_mission'),
            'config', 'autonomous_nav_params.yaml'),
        description='[mapping 模式] waypoint_recorder 写入的 autonomous_nav_params.yaml 路径',
    )

    # 建图模式退出兜底转换：等待 FAST-LIO2 写完 PCD 的最长时间（s）。
    # 大图（20.7M 点 ≈ 664MB）写盘需数秒~数十秒，eMMC/机械盘可按需调大
    declare_final_wait = DeclareLaunchArgument(
        'final_convert_wait_timeout',
        default_value='120.0',
        description='[mapping 模式] 退出兜底转换等待 FAST-LIO2 写完 PCD 的最长时间（s）',
    )

    # 仿真时钟
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='是否使用仿真时钟',
    )

    # Nav2 生命周期自动启动
    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Nav2 lifecycle_manager 是否自动激活节点',
    )

    # 全局定位开关（V0.0.93 方案A 已废弃）：定位改为 hunter_relocalization NDT，
    # 不再启动 AMCL。本参数保留仅为兼容旧调用，已无任何作用。
    declare_use_amcl = DeclareLaunchArgument(
        'use_amcl',
        default_value='false',
        description='[已废弃 V0.0.93] AMCL 已被 hunter_relocalization(NDT) 取代，本参数无效',
    )

    # ---- 模式判断条件 ----
    is_nav_mode     = PythonExpression(["'", LaunchConfiguration('mode'), "' == 'nav'"])
    is_mapping_mode = PythonExpression(["'", LaunchConfiguration('mode'), "' == 'mapping'"])

    # ---- 模式日志 ----
    log_nav = LogInfo(
        condition=IfCondition(is_nav_mode),
        msg='[hunter_autonomous_nav] 启动：定位导航模式（Nav2 + auto_mission_node）',
    )
    log_mapping = LogInfo(
        condition=IfCondition(is_mapping_mode),
        msg='[hunter_autonomous_nav] 启动：建图模式（FAST-LIO2 在线建图，不下发导航目标）',
    )

    # ---- 导航模式节点（OpaqueFunction 运行时解析路径） ----
    nav_nodes = OpaqueFunction(
        function=_nav2_params_with_bt,
        condition=IfCondition(is_nav_mode),
    )

    # ---- 建图模式节点 ----
    mapping_nodes = OpaqueFunction(
        function=_mapping_nodes,
        condition=IfCondition(is_mapping_mode),
    )

    return LaunchDescription([
        # 参数
        declare_mode,
        declare_map_yaml,
        declare_map_file,
        declare_params_file_path,
        declare_final_wait,
        declare_use_sim_time,
        declare_autostart,
        declare_use_amcl,
        # 日志
        log_nav,
        log_mapping,
        # 节点组（按模式分支）
        nav_nodes,
        mapping_nodes,
    ])
