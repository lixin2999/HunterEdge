# vehicle_ingest_agent.launch.py — 仅用于开发机联调（生产走 systemd unit，非 ROS 节点）。
#
# 本 Agent 是单进程 asyncio systemd 服务（设计文档 §3.2 / §12），核心链路不依赖 rclpy；
# 这里用 ExecuteProcess 起同一入口，方便本地 `ros2 launch` 拉起并注入 --config。
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    default_cfg = os.path.join(
        get_package_share_directory('vehicle_ingest_agent'),
        'config', 'vehicle_ingest_agent.yaml')
    cfg = os.environ.get('HUNTER_VIA_CONFIG', default_cfg)
    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', '-m', 'vehicle_ingest_agent.agent', '--config', cfg],
            output='screen',
            name='vehicle_ingest_agent'),
    ])
