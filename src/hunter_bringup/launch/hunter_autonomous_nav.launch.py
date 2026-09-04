"""hunter_autonomous_nav.launch.py — HUNTER 自主建图/导航一体启动文件

支持两种运行模式（通过参数 mode 切换）：
  mode:=mapping  — 建图模式（全自动，无需手动操作）
      启动：fast_lio2 参数覆盖（pcd_save_en=true、map_file_path 自动注入）
            + auto_mission_node(mapping 状态)
            + pcd_to_map 节点（监听建图结束信号，自动完成 PCD→PGM+YAML 转换）
            + waypoint_recorder 节点（订阅 /clicked_point，自动写入 yaml）
      结束：切换到 nav 模式时 pcd_to_map 自动检测并触发转换，
            无需手动执行任何命令

  mode:=nav      — 定位导航模式（默认）
      启动：fast_lio2 参数覆盖（pcd_save_en=false）
            + Nav2 全栈（使用 autonomous_navigate.xml 扩展行为树）
            + auto_mission_node（按 autonomous_nav_params.yaml 执行航点巡航）
      加载：nav2_map_server 加载已保存的静态地图（map_yaml_path 参数）

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

    因为 bt_navigator default_bt_xml_filename 在 nav2_params.yaml 里是字符串，
    需要在 launch 运行时才能解析包路径，所以用 OpaqueFunction 动态生成 Node。
    """
    pkg_bringup = get_package_share_directory('hunter_bringup')
    pkg_auto    = get_package_share_directory('auto_mission')

    nav2_params_file = os.path.join(pkg_bringup, 'config', 'nav2_params.yaml')
    auto_bt_file     = os.path.join(pkg_bringup, 'behavior_trees', 'autonomous_navigate.xml')
    auto_params_file = os.path.join(pkg_auto,    'config', 'autonomous_nav_params.yaml')
    map_yaml_path    = LaunchConfiguration('map_yaml_path').perform(context)
    use_sim_time     = LaunchConfiguration('use_sim_time').perform(context)
    autostart        = LaunchConfiguration('autostart').perform(context)

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
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file],
    )

    # ---- bt_navigator：使用扩展行为树 autonomous_navigate.xml ----
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            nav2_params_file,
            {'default_bt_xml_filename': auto_bt_file},
            {'use_sim_time': use_sim_time == 'true'},
        ],
    )

    # ---- velocity_smoother ----
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params_file],
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

    return [
        fast_lio2_nav_param_node,
        map_server,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        velocity_smoother,
        lifecycle_manager,
        auto_mission,
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
        # 日志
        log_nav,
        log_mapping,
        # 节点组（按模式分支）
        nav_nodes,
        mapping_nodes,
    ])
