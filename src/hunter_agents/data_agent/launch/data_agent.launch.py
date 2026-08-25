"""数据采集 Agent 节点启动文件（文档第 14 章）。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('data_agent')
    params_file = os.path.join(pkg_share, 'config', 'data_agent_params.yaml')

    data_agent_node = Node(
        package='data_agent',
        executable='data_agent_node',
        name='data_agent',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        data_agent_node,
    ])
