"""command_agent 启动文件（HunterCore 平台指令接入）。

单独启动：ros2 launch command_agent command_agent.launch.py
整栈启动时由 hunter_bringup/launch/hunter_full.launch.py 统一拉起。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('command_agent')
    default_params = os.path.join(pkg_share, 'config', 'command_agent_params.yaml')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='command_agent 参数文件路径（凭据不在此文件，见 /etc/hunter/kafka/kafka.properties）',
        ),
        Node(
            package='command_agent',
            executable='command_agent',
            name='command_agent',
            output='screen',
            parameters=[params_file],
        ),
    ])
