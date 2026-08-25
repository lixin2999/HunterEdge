"""加载 HUNTER SE URDF/Xacro 并发布静态 TF（base_link → lidar/camera_color_optical_frame/imu）。

依据文档 7.4 坐标系定义、附录 D 传感器安装示意。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('hunter_bringup')
    xacro_file = os.path.join(pkg_share, 'urdf', 'hunter_se.urdf.xacro')

    # robot_state_publisher：解析 xacro → robot_description，发布静态 TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro', ' ', xacro_file]),
            'use_sim_time': False,
        }],
    )

    return LaunchDescription([
        robot_state_publisher,
    ])
