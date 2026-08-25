"""多传感器数据融合节点启动文件（文档第 6 章）。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('sensor_fusion')
    params_file = os.path.join(pkg_share, 'config', 'sensor_fusion_params.yaml')

    sensor_fusion_node = Node(
        package='sensor_fusion',
        executable='sensor_fusion_node',
        name='sensor_fusion',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        sensor_fusion_node,
    ])
