"""CH10X IMU 驱动节点启动文件（文档 3.3.3）。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('ch10x_driver')
    params_file = os.path.join(pkg_share, 'config', 'ch10x_params.yaml')

    ch10x_node = Node(
        package='ch10x_driver',
        executable='ch10x_driver_node',
        name='ch10x_driver',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        ch10x_node,
    ])
