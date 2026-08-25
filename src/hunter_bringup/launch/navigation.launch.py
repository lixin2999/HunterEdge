"""HUNTER Nav2 导航栈启动：SmacPlannerHybrid 全局规划 + RPP 局部控制（文档第 8/9/10 章）。

适配 HUNTER-SE 阿克曼车辆：REEDS_SHEPP 运动模型、最小转弯半径 1.9m、footprint。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('hunter_bringup')
    params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    bt_file = os.path.join(pkg_share, 'behavior_trees', 'navigate_to_pose.xml')

    declare_params_file = DeclareLaunchArgument('params_file', default_value=params_file)
    declare_autostart = DeclareLaunchArgument('autostart', default_value='true')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')

    lifecycle_nodes = [
        'controller_server', 'planner_server', 'recoveries_server',
        'bt_navigator', 'velocity_smoother']

    # RPP 局部控制器（文档 10.2）
    # 输出 cmd_vel_nav，经 velocity_smoother 平滑后下发底盘
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )

    # SmacPlannerHybrid 全局规划器（文档 9.2：REEDS_SHEPP + 1.9m 转弯半径）
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    # 恢复行为服务
    recoveries_server = Node(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    # 行为树导航器（文档 2.3 nav2_bt_navigator）
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'default_bt_xml_filename': bt_file},
        ],
    )

    # 速度平滑器（阿克曼：订阅 cmd_vel_nav → 发布 cmd_vel）
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    # 生命周期管理器
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'autostart': LaunchConfiguration('autostart')},
            {'node_names': lifecycle_nodes},
        ],
    )

    return LaunchDescription([
        declare_params_file,
        declare_autostart,
        declare_use_sim_time,
        controller_server,
        planner_server,
        recoveries_server,
        bt_navigator,
        velocity_smoother,
        lifecycle_manager,
    ])
