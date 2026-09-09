"""hunter_autonomous_nav.launch.py — HUNTER 自主建图/导航一体启动文件

支持两种运行模式（通过参数 mode 切换）：
  mode:=mapping  — 建图模式（车辆由人工遥控驾驶扫描，系统侧流程自动）
      启动：fast_lio2 参数覆盖（pcd_save_en=true、map_file_path 自动注入）
            + auto_mission_node(mapping 状态，不下发导航目标——车辆由人工遥控)
            + pcd_to_map 节点（监听建图结束信号，自动完成 PCD→PGM+YAML 转换）
            + waypoint_recorder 节点（订阅 /clicked_point，自动写入 yaml）
      操作：人工遥控（REMOTE 模式）驾驶遍历全部目标区域；rviz2 用 Publish Point
            （快捷键 P）点击记录巡检航点（User_Manual §4.4）。
      结束：在启动终端按 Ctrl+C → fast_lio2 退出时才把全部点云写入
            map_file_path（PCD 仅在节点退出瞬间写盘，运行中文件不存在属正常）；
            pcd_to_map 检测 /auto_mission/status MAPPING→非MAPPING 跳变后
            自动触发转换，无需手动执行任何命令。
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
    use_amcl         = LaunchConfiguration('use_amcl').perform(context)

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
    # AMCL 在 map_server 之后激活（依赖其发布的 /map），负责发布 map→odom
    if use_amcl == 'true':
        lifecycle_nodes.insert(1, 'amcl')

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

    # ---- controller_server（RPP，与原 navigation.launch.py 一致） ----
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
            'max_linear_vel': 0.8,      # 与 nav2_params.yaml desired_linear_vel 一致
            'stop_dist': 0.6,           # scan 行进方向扇区急停距离（m）
            'slow_dist': 1.2,           # scan 行进方向扇区减速距离（m）
            'sector_half_deg': 60.0,    # 检测扇区半角（°）
            'scan_timeout': 0.5,        # /scan 断流 fail-safe（s）
            'cmd_timeout': 0.5,         # 上游指令断流看门狗（s）
            'enable_test_mode': False,  # V0.0.86 测试模式启动默认关（运行时经 /safety/test_mode 开关）
            'use_sim_time': use_sim_time == 'true',
        }],
    )

    # ---- 全局定位（map→odom）：AMCL + 3D→2D 激光投影 ----
    # TF 链：map --AMCL--> odom --EKF--> base_link。
    # AMCL 以 /scan（由 /lidar_points 经 pointcloud_to_laserscan 投影）匹配静态地图，
    # 自动输出并持续修正 map→odom；初始位姿默认地图原点（假设上电位姿≈建图起点），
    # 偏差大时用 rviz2 "2D Pose Estimate" 向 /initialpose 发布真实位姿重定位。
    use_amcl_cond = IfCondition(PythonExpression(
        ["'", LaunchConfiguration('use_amcl'), "' == 'true'"]))

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
            'range_min': 0.5,
            'range_max': 50.0,
            'use_inf': True,
            'use_sim_time': use_sim_time == 'true',
        }],
        condition=use_amcl_cond,
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            nav2_params_file,
            {'use_sim_time': use_sim_time == 'true'},
        ],
        condition=use_amcl_cond,
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
        amcl,
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
            'z_min':                  0.1,
            'z_max':                  2.0,
            'occupied_thresh':        0.65,
            'free_thresh':            0.25,
            'padding_m':              0.5,
            'auto_reload_map':        False,   # 建图模式下 map_server 未启动，禁用重载
            'trigger_on_mapping_end': True,
            'convert_on_start_if_missing': False,  # 建图开始时无图可转，禁用启动自愈
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

    # 全局定位开关（默认 true）：关闭时不启动 AMCL/激光投影——将没有 map→odom，
    # global_costmap 与 bt_navigator 无法工作，仅用于调试其余组件
    declare_use_amcl = DeclareLaunchArgument(
        'use_amcl',
        default_value='true',
        description='[nav 模式] 启动 AMCL + pointcloud_to_laserscan 全局定位（发布 map→odom）',
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
