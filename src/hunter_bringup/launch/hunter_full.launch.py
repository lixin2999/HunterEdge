"""HUNTER 全系统总启动（文档 4.4 启动管理）。

严格启动顺序（文档 4.4）：
  ROS2 daemon(自动) → 传感器驱动 → CAN驱动 → 定位 → 感知 → 融合
  → 决策 → 规划 → 控制 → Agent → health_monitor

参数开关：use_perception / use_navigation / use_data_agent
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('hunter_bringup')

    # ---- 参数开关（文档 4.4） ----
    use_perception = LaunchConfiguration('use_perception')
    use_navigation = LaunchConfiguration('use_navigation')
    use_data_agent = LaunchConfiguration('use_data_agent')

    declare_use_perception = DeclareLaunchArgument('use_perception', default_value='true')
    declare_use_navigation = DeclareLaunchArgument('use_navigation', default_value='true')
    declare_use_data_agent = DeclareLaunchArgument('use_data_agent', default_value='true')

    # ---- 1. 传感器驱动（文档 2.3 驱动层） ----
    lidar_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rslidar_sdk'),
                         'launch', 'humble_start.py')),
        launch_arguments={}.items())
    camera_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'),
                         'launch', 'rs_launch.py')),
        launch_arguments={}.items())
    imu_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ch10x_driver'),
                         'launch', 'ch10x_driver.launch.py')),
        launch_arguments={}.items())

    # ---- 2. CAN 驱动（hunter_base，文档 11） ----
    can_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('hunter_base'),
                         'launch', 'hunter_base.launch.py')),
        launch_arguments={}.items())

    # ---- 3. 定位（fast_lio2 + EKF，文档 7） ----
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'localization.launch.py')),
        launch_arguments={}.items())

    # ---- 4. 感知 + 融合（lidar/vision/fusion，文档 5/6） ----
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'perception.launch.py')),
        launch_arguments={}.items(),
        condition=IfCondition(use_perception))

    # ---- 5. 决策（decision_making，文档 13.5） ----
    decision_making = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('decision_making'),
                         'launch', 'decision_making.launch.py')),
        launch_arguments={}.items())

    # ---- 6. 规划 + 控制（Nav2，文档 8/9/10） ----
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'navigation.launch.py')),
        launch_arguments={}.items(),
        condition=IfCondition(use_navigation))

    # ---- 7. Agent（数据采集，文档 14；remote/ota 为 systemd 服务） ----
    data_agent = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('data_agent'),
                         'launch', 'data_agent.launch.py')),
        launch_arguments={}.items(),
        condition=IfCondition(use_data_agent))

    # ---- 8. health_monitor（文档 15） ----
    health_monitor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('health_monitor'),
                         'launch', 'health_monitor.launch.py')),
        launch_arguments={}.items())

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
