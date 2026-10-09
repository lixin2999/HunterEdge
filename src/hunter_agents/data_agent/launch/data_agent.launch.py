"""数据采集 Agent 节点启动文件（文档第 14 章）。

现场改参不用重编：把参数文件放到 /etc/hunter/ 后用 params_file 指过去（
凭据仍只读 /etc/hunter/kafka/kafka.properties，不写进 YAML）。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('data_agent')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'config', 'data_agent_params.yaml'),
            description='data_agent 参数文件路径',
        ),
        Node(
            package='data_agent',
            executable='data_agent_node',
            name='data_agent',
            output='screen',
            parameters=[params_file],
        ),
    ])
