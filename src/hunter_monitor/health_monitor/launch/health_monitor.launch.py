"""系统监控健康管理节点启动文件（文档第 15 章）。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('health_monitor')
    params_file = os.path.join(pkg_share, 'config', 'health_monitor_params.yaml')

    health_monitor_node = Node(
        package='health_monitor',
        executable='health_monitor_node',
        name='health_monitor',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        health_monitor_node,
    ])
