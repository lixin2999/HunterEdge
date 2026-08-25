"""模式仲裁决策节点启动文件（文档 13.5）。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('decision_making')
    params_file = os.path.join(pkg_share, 'config', 'decision_making_params.yaml')

    decision_making_node = Node(
        package='decision_making',
        executable='decision_making_node',
        name='decision_making',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        decision_making_node,
    ])
