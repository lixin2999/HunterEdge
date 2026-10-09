"""数据采集 Agent 启动入口（文档 §7.4：ros2 launch hunter_bringup data_agent.launch.py）。

真正的节点定义在 data_agent 包内（launch/data_agent.launch.py），本文件只做转发，
避免两处各写一份参数/节点导致改一处漏一处。需要指定现场参数文件时：

  ros2 launch hunter_bringup data_agent.launch.py \
    params_file:=/etc/hunter/data_agent_params.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    data_agent_share = get_package_share_directory('data_agent')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(data_agent_share, 'config', 'data_agent_params.yaml'),
            description='data_agent 参数文件（Kafka 凭据不在此文件，见 /etc/hunter/kafka/kafka.properties）',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(data_agent_share, 'launch', 'data_agent.launch.py')),
            launch_arguments={'params_file': params_file}.items(),
        ),
    ])
