r"""sil_bridge.launch.py — 真 SIL 桥接 Agent 启动入口（HunterEdge V0.1.38）。

与 remote_agent.launch.py 同形态：只拉起一个独立进程（进程内自建 ROS 节点），
参数经 --config 传入 YAML，不用 ros__parameters 声明式参数。

用法::

    # 单独启动（需自行 source ROS/工作空间）
    ros2 launch sil_bridge_agent sil_bridge.launch.py
    ros2 launch sil_bridge_agent sil_bridge.launch.py \
        params_file:=/etc/hunter/sil_bridge_params.yaml

    # 整栈 SIL 模式（推荐：不启动真实传感器驱动/CAN/LIO）
    ros2 launch hunter_bringup hunter_sil.launch.py

⚠ 本 launch **只启动桥接**，不启动感知/决策/auto_mission；被测全栈由
hunter_sil.launch.py 统一编排（避免这里重复启动造成双发布者）。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('sil_bridge_agent')
    default_params = os.path.join(pkg_share, 'config', 'sil_bridge_params.yaml')

    params_file = LaunchConfiguration('params_file')
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=os.environ.get('HUNTER_SIL_CONFIG', default_params),
        description='SIL 桥接参数 YAML（不含任何 Kafka 凭据；凭据取 /etc/hunter/kafka）',
    )

    # executable 用 scripts/sil_bridge_node（装到 lib/<pkg>/），**不用 console_scripts**：
    # --symlink-install 下入口包装会因找不到发行版元数据抛 PackageNotFoundError，
    # HUNTER-001 实机上 remote-agent/ota-agent 曾因此陷入 activating 重启循环。
    bridge = Node(
        package='sil_bridge_agent',
        executable='sil_bridge_node',
        name='sil_bridge_agent',
        output='screen',
        arguments=['--config', params_file],
        # 与 hunter-edge 主栈同一 RMW，否则跨进程话题不可见（remote_agent V0.1.19 教训）
        additional_env={
            'RMW_IMPLEMENTATION': os.environ.get('HUNTER_RMW_IMPLEMENTATION', ''),
            'PYTHONUNBUFFERED': '1',
        },
        # SIGINT 让 Python 抛 KeyboardInterrupt → run() 的 finally 补零速 + flush + 关节点
        sigterm_timeout='20',
        sigkill_timeout='10',
    )

    return LaunchDescription([declare_params, bridge])
