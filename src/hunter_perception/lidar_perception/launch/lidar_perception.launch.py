"""激光感知节点启动文件（文档 5.1）。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('lidar_perception')
    params_file = os.path.join(pkg_share, 'config', 'lidar_perception_params.yaml')

    lidar_perception_node = Node(
        package='lidar_perception',
        executable='lidar_perception_node',
        name='lidar_perception',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        lidar_perception_node,
    ])
