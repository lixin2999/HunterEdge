"""视觉感知节点启动文件（文档 5.2）。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('vision_perception')
    params_file = os.path.join(pkg_share, 'config', 'vision_perception_params.yaml')

    vision_perception_node = Node(
        package='vision_perception',
        executable='vision_perception_node',
        name='vision_perception',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        vision_perception_node,
    ])
