"""HUNTER 全系统总启动（文档 4.4 启动管理）。

严格启动顺序（文档 4.4）：
  ROS2 daemon(自动) → 传感器驱动 → CAN驱动 → 定位 → 感知 → 融合
  → 决策 → 规划 → 控制 → Agent → health_monitor

参数开关：use_perception / use_navigation / use_data_agent
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                             IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace


def _isolated(source, *, condition=None):
    """将子 launch 包裹在 GroupAction 中，重置所有父级 LaunchConfiguration，
    避免 use_perception 等参数被透传到不认识它们的子包（如 realsense2_camera）。"""
    kwargs = {}
    if condition is not None:
        kwargs['condition'] = condition
    return GroupAction(
        actions=[IncludeLaunchDescription(source)],
        **kwargs,
    )


def generate_launch_description():
    pkg_share = get_package_share_directory('hunter_bringup')

    # ---- 参数开关（文档 4.4） ----
    use_perception = LaunchConfiguration('use_perception')
    use_navigation = LaunchConfiguration('use_navigation')
    use_data_agent = LaunchConfiguration('use_data_agent')

    declare_use_perception = DeclareLaunchArgument('use_perception', default_value='true')
    declare_use_navigation = DeclareLaunchArgument('use_navigation', default_value='true')
    declare_use_data_agent = DeclareLaunchArgument('use_data_agent', default_value='true')

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
    can_driver     = _isolated(src('hunter_base',      'launch', 'hunter_base.launch.py'))

    # ---- 3. 定位（fast_lio + EKF，文档 7） ----
    localization   = _isolated(local_src('localization.launch.py'))

    # ---- 4. 感知 + 融合（lidar/vision/fusion，文档 5/6） ----
    perception     = _isolated(local_src('perception.launch.py'),
                               condition=IfCondition(use_perception))

    # ---- 5. 决策（decision_making，文档 13.5） ----
    decision_making = _isolated(src('decision_making', 'launch', 'decision_making.launch.py'))

    # ---- 6. 规划 + 控制（Nav2，文档 8/9/10） ----
    navigation     = _isolated(local_src('navigation.launch.py'),
                               condition=IfCondition(use_navigation))

    # ---- 7. Agent（数据采集，文档 14；remote/ota 为 systemd 服务） ----
    data_agent     = _isolated(src('data_agent', 'launch', 'data_agent.launch.py'),
                               condition=IfCondition(use_data_agent))

    # ---- 8. health_monitor（文档 15） ----
    health_monitor = _isolated(src('health_monitor', 'launch', 'health_monitor.launch.py'))

    return LaunchDescription([
        declare_use_perception,
        declare_use_navigation,
        declare_use_data_agent,
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
        # 6. 规划 + 控制
        navigation,
        # 7. Agent
        data_agent,
        # 8. health_monitor
        health_monitor,
    ])
