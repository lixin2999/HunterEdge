"""HUNTER 全系统总启动（文档 4.4 启动管理）。

严格启动顺序（文档 4.4）：
  ROS2 daemon(自动) → 传感器驱动 → CAN驱动 → 定位 → 感知 → 融合
  → 决策 → 规划 → 控制 → Agent → health_monitor

参数开关：use_perception / use_navigation / use_data_agent / use_autonomous_nav

use_autonomous_nav 说明：
  false（默认）：保持原有启动行为不变，Nav2 由 navigation.launch.py 启动
  true          ：关闭原 navigation.launch.py（use_navigation 自动置 false），
                  改由 hunter_autonomous_nav.launch.py 启动自主导航全栈；
                  配合 autonomous_nav_mode 参数选择建图（mapping）或导航（nav）子模式。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                             IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace


def _isolated(source, *, condition=None, launch_arguments=None):
    """将子 launch 包裹在 GroupAction 中，重置所有父级 LaunchConfiguration，
    避免 use_perception 等参数被透传到不认识它们的子包（如 realsense2_camera）。
    launch_arguments: 可选的 dict，用于向子 launch 显式传参，例如 {'port_name': 'can0'}。"""
    kwargs = {}
    if condition is not None:
        kwargs['condition'] = condition
    include_kwargs = {}
    if launch_arguments is not None:
        include_kwargs['launch_arguments'] = launch_arguments.items()
    return GroupAction(
        scoped=True,
        forwarding=False,
        actions=[IncludeLaunchDescription(source, **include_kwargs)],
        **kwargs,
    )


def generate_launch_description():
    pkg_share = get_package_share_directory('hunter_bringup')

    # ---- 参数开关（文档 4.4） ----
    use_perception = LaunchConfiguration('use_perception')
    use_navigation = LaunchConfiguration('use_navigation')
    use_data_agent = LaunchConfiguration('use_data_agent')
    use_autonomous_nav  = LaunchConfiguration('use_autonomous_nav')
    autonomous_nav_mode = LaunchConfiguration('autonomous_nav_mode')

    declare_use_perception      = DeclareLaunchArgument('use_perception',      default_value='true')
    declare_use_navigation      = DeclareLaunchArgument('use_navigation',      default_value='true')
    declare_use_data_agent      = DeclareLaunchArgument('use_data_agent',      default_value='true')
    # 新增：自主导航开关（默认关闭，不破坏原有启动行为）
    declare_use_autonomous_nav  = DeclareLaunchArgument(
        'use_autonomous_nav',
        default_value='false',
        description='true: 启动自主建图/导航全栈（auto_mission + Nav2扩展行为树），'
                    '同时自动禁用原 navigation.launch.py 避免重复启动 Nav2',
    )
    # 自主导航子模式：nav（定位导航，默认）或 mapping（在线建图）
    declare_autonomous_nav_mode = DeclareLaunchArgument(
        'autonomous_nav_mode',
        default_value='nav',
        description='[use_autonomous_nav=true 时生效] nav=导航巡航模式，mapping=建图模式',
    )
    # 导航地图路径（传递给 hunter_autonomous_nav.launch.py）
    declare_map_yaml_path = DeclareLaunchArgument(
        'map_yaml_path',
        default_value='/home/hunter/maps/hunter_map.yaml',
        description='[use_autonomous_nav=true, autonomous_nav_mode=nav] 静态地图 YAML 文件路径',
    )
    # 建图 PCD 保存路径
    declare_map_file_path = DeclareLaunchArgument(
        'map_file_path',
        default_value='/home/hunter/maps/hunter_map.pcd',
        description='[use_autonomous_nav=true, autonomous_nav_mode=mapping] PCD 保存路径',
    )

    def src(pkg, *path_parts):
        return PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(pkg), *path_parts))

    def local_src(*path_parts):
        return PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', *path_parts))

    # ---- 1. 传感器驱动（文档 2.3 驱动层） ----
    lidar_driver   = _isolated(src('rslidar_sdk',      'launch', 'humble_start.py'))
    camera_driver  = _isolated(src('realsense2_camera','launch', 'rs_launch.py'))
    imu_driver     = _isolated(src('ch10x_driver',     'launch', 'ch10x_driver.launch.py'))

    # ---- 2. CAN 驱动（hunter_base，文档 11） ----
    # port_name 显式指定为 can0，覆盖 hunter_base 默认的 can_car
    can_driver     = _isolated(src('hunter_base',      'launch', 'hunter_base.launch.py'),
                               launch_arguments={'port_name': 'can0'})

    # ---- 3. 定位（fast_lio + EKF，文档 7） ----
    localization   = _isolated(local_src('localization.launch.py'))

    # ---- 4. 感知 + 融合（lidar/vision/fusion，文档 5/6） ----
    perception     = _isolated(local_src('perception.launch.py'),
                               condition=IfCondition(use_perception))

    # ---- 5. 决策（decision_making，文档 13.5） ----
    decision_making = _isolated(src('decision_making', 'launch', 'decision_making.launch.py'))

    # ---- 6. 规划 + 控制（Nav2，文档 8/9/10） ----
    # use_autonomous_nav=true 时自动禁用，由 hunter_autonomous_nav.launch.py 接管 Nav2
    # 条件：use_navigation=true 且 use_autonomous_nav=false
    _nav_condition = PythonExpression([
        "'true' == '", use_navigation, "' and 'true' != '", use_autonomous_nav, "'"
    ])
    navigation     = _isolated(local_src('navigation.launch.py'),
                               condition=IfCondition(_nav_condition))

    # ---- 6b. 自主导航全栈（可选，use_autonomous_nav=true 时启动） ----
    # 显式传入子模式和路径参数，因为 _isolated(scoped=True, forwarding=False) 会阻断参数透传。
    # 注意：launch_arguments 的值必须是 substitution 列表形式 [LaunchConfiguration(...)]，
    # 直接传 LaunchConfiguration 对象在 scoped GroupAction 里会因作用域隔离报 does not exist。
    autonomous_nav = _isolated(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'hunter_autonomous_nav.launch.py')
        ),
        condition=IfCondition(use_autonomous_nav),
        launch_arguments={
            'mode':          [autonomous_nav_mode],
            'map_yaml_path': [LaunchConfiguration('map_yaml_path')],
            'map_file_path': [LaunchConfiguration('map_file_path')],
        },
    )

    # ---- 7. Agent（数据采集，文档 14；remote/ota 为 systemd 服务） ----
    data_agent     = _isolated(src('data_agent', 'launch', 'data_agent.launch.py'),
                               condition=IfCondition(use_data_agent))

    # ---- 8. health_monitor（文档 15） ----
    health_monitor = _isolated(src('health_monitor', 'launch', 'health_monitor.launch.py'))

    return LaunchDescription([
        declare_use_perception,
        declare_use_navigation,
        declare_use_data_agent,
        declare_use_autonomous_nav,
        declare_autonomous_nav_mode,
        declare_map_yaml_path,
        declare_map_file_path,
        # 1. 传感器驱动
        lidar_driver,
        camera_driver,
        imu_driver,
        # 2. CAN 驱动
        can_driver,
        # 3. 定位
        localization,
        # 4. 感知 + 融合
        perception,
        # 5. 决策
        decision_making,
        # 6a. 原有 Nav2（use_autonomous_nav=false 时启动）
        navigation,
        # 6b. 自主导航全栈（use_autonomous_nav=true 时启动）
        autonomous_nav,
        # 7. Agent
        data_agent,
        # 8. health_monitor
        health_monitor,
    ])
