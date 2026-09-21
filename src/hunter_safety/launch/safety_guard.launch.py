"""safety_guard.launch.py — 碰撞防护与运动学安全约束节点独立启动（调试用）

正常部署时由 hunter_autonomous_nav.launch.py（mode:=nav）自动启动本节点，
位于 velocity_smoother 与底盘之间。本文件仅用于单独调试：

    ros2 launch hunter_safety safety_guard.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hunter_safety',
            executable='safety_guard',
            name='safety_guard',
            output='screen',
            remappings=[('cmd_vel_in', '/cmd_vel_pre_safety')],
            parameters=[{
                'wheelbase': 0.65,          # HunterV2Params::wheelbase（AGX_V2 实车）
                'min_turn_radius': 1.9,     # 与 Smac minimum_turning_radius 一致
                'max_linear_vel': 0.8,      # 与 nav2_params.yaml desired_linear_vel 一致
                # V0.0.91：急停距离必须 > /scan 的 range_min（0.8m），否则碰撞闸存在
                # 盲区（旧值 0.6 小于盲区地板，急停永不可达）
                'stop_dist': 1.0,           # 行进走廊纵向净空急停距离（m）
                'slow_dist': 1.8,           # 行进走廊纵向净空减速距离（m）
                'sector_half_deg': 60.0,    # 检测扇区半角（°）
                'corridor_half_width': 0.45,   # 安全走廊半宽（m）
                'footprint_front': 0.45,    # 文档 9.3 车体 footprint
                'footprint_rear': 0.37,
                'footprint_half_width': 0.32,
                'self_margin': 0.12,        # 自车包络外扩：盒内回波判为自身反射
                'scan_timeout': 0.5,        # /scan 断流 fail-safe（s）
                'cmd_timeout': 0.5,         # 上游指令断流看门狗（s）
                'enable_test_mode': False,  # V0.0.86 测试模式启动默认关（运行时经 /safety/test_mode 开关）
                # V0.0.87 地图边界监护（单独调试时需 map_server + AMCL 已运行）
                'enable_map_fence': True,
                'map_edge_stop_dist': 0.5,
                'map_edge_slow_dist': 1.5,
                'use_sim_time': False,
            }],
        ),
    ])
