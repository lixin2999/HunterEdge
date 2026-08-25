"""感知模块启动：lidar_perception + vision_perception + sensor_fusion（文档 5/6 章）。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    lidar_params = os.path.join(
        get_package_share_directory('lidar_perception'),
        'config', 'lidar_perception_params.yaml')
    vision_params = os.path.join(
        get_package_share_directory('vision_perception'),
        'config', 'vision_perception_params.yaml')
    fusion_params = os.path.join(
        get_package_share_directory('sensor_fusion'),
        'config', 'sensor_fusion_params.yaml')

    # 激光感知（文档 5.1）
    lidar_perception = Node(
        package='lidar_perception', executable='lidar_perception_node',
        name='lidar_perception', output='screen', parameters=[lidar_params])

    # 视觉感知（文档 5.2）
    vision_perception = Node(
        package='vision_perception', executable='vision_perception_node',
        name='vision_perception', output='screen', parameters=[vision_params])

    # 目标级数据融合（文档第 6 章）
    sensor_fusion = Node(
        package='sensor_fusion', executable='sensor_fusion_node',
        name='sensor_fusion', output='screen', parameters=[fusion_params])

    return LaunchDescription([
        lidar_perception,
        vision_perception,
        sensor_fusion,
    ])
