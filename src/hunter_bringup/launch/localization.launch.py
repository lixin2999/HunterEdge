"""HUNTER 定位模块启动：FAST-LIO2 紧耦合里程计 + robot_localization EKF 融合（文档第 7 章）。

输入话题：/lidar_points、/imu/data、/chassis/feedback（经 hunter_ros2 转为 /odom）
输出话题：/localization/odom（50Hz）、/localization/pose（50Hz）
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('hunter_bringup')
    fast_lio2_params = os.path.join(pkg_share, 'config', 'fast_lio2_params.yaml')
    ekf_params = os.path.join(pkg_share, 'config', 'ekf_params.yaml')

    # =====================================================================
    # 1. FAST-LIO2 紧耦合雷达惯导里程计（文档 7.2）
    #    输入：/lidar_points(RS-Helios-16P, 10Hz) + /imu/data(CH10X, 100Hz)
    #    输出：/Odometry(10Hz, 高置信)
    #    算法：IEKF + ikd-Tree；外参 LiDAR-IMU 见 fast_lio2_params.yaml
    # =====================================================================
    fast_lio2_node = Node(
        package='fast_lio2',
        executable='fastlio_mapping',
        name='fast_lio2',
        output='screen',
        parameters=[fast_lio2_params],
    )

    # =====================================================================
    # 2. robot_localization EKF 融合（文档 7.3，15 维状态）
    #    融合：/Odometry(FAST-LIO2, 9维) + /odom(轮式里程计, vx) + /imu/data(IMU, 6维)
    #    输出：/localization/odom(50Hz)、/localization/pose(50Hz)
    #    注：文档 7.3 将轮速里程计记为 /chassis/state（任务08 记为 /chassis/feedback），
    #        实际由 hunter_ros2 将底盘 ChassisState 反馈转换为 nav_msgs/Odometry 的 /odom。
    # =====================================================================
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params],
    )

    # =====================================================================
    # 3. 静态 TF（文档 7.4）
    #    base_link → lidar/camera_color_optical_frame/imu：由 robot_state_publisher 发布
    #    odom → base_link：由 EKF publish_tf=true 发布（ekf_params.yaml）
    #    map → odom：由后续全局定位模块发布（本 launch 不含）
    # =====================================================================
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'description.launch.py')
        )
    )

    # =====================================================================
    # 降级策略（文档 7.6）——由三传感器独立配置 + 超时参数自动实现：
    #   • LiDAR点云缺失 > 1s  → FAST-LIO2 无输出 → EKF 自动降级为轮速+IMU
    #   • FAST-LIO2 发散       → 位姿跳变，EKF 切换为纯轮速+IMU 推算，告警
    #   • 轮速信号丢失         → odom1 无数据 → EKF 仅融合 FAST-LIO2+IMU，告警
    #   • IMU 故障            → imu0 无数据 → 纯 LiDAR 里程计模式，告警并建议停车
    # =====================================================================

    return LaunchDescription([
        fast_lio2_node,
        ekf_node,
        description_launch,
    ])
