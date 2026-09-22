"""relocalization.launch.py — 方案A 全局 NDT 重定位（独立调试用）
正常集成时由 hunter_autonomous_nav.launch.py 直接内联启动本节点，
本文件仅用于单独跑重定位、验证 map→odom 是否收敛。
用法：
  ros2 launch hunter_relocalization relocalization.launch.py \
      global_map_path:=/home/agilex/HunterEdge/maps/hunter_map.pcd
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_cfg = os.path.join(
        get_package_share_directory('hunter_relocalization'), 'config',
        'relocalization_params.yaml')

    declare_map = DeclareLaunchArgument(
        'global_map_path', default_value='',
        description='全局点云地图 .pcd 绝对路径（必填）')

    node = Node(
        package='hunter_relocalization',
        executable='ndt_relocalization_node',
        name='hunter_relocalization',
        output='screen',
        parameters=[
            pkg_cfg,
            {'global_map_path': LaunchConfiguration('global_map_path')},
        ],
    )
    return LaunchDescription([declare_map, node])
