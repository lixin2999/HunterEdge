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
                'wheelbase': 0.46,          # HUNTER-SE 轴距（m，δ_max 几何叙述用；曲率钳制以 min_turn_radius 为准）
                'min_turn_radius': 1.9,     # 与 Smac minimum_turn_radius 一致
                'max_linear_vel': 0.5,      # V0.0.89 与 desired_linear_vel/velocity_smoother 一致 0.5
                # V0.0.97 阈值重标定（与 hunter_autonomous_nav.launch.py 生产配置同源）：
                #   旧值 stop 1.0m 在室内窄场地构成【几何死锁】——阿克曼绕开正前方障碍需
                #   "提前转向距离" s ≥ √(2R·(半宽+余量)) ≈ 1.33m，碰撞闸在 1.0m 处就把前进
                #   指令按停 → 车永远进不到"能转向"的位置。现 stop 0.90（< 1.33 且 > 盲区地板
                #   range_min 0.70 + 0.15 = 0.85），slow 1.40 取"转向提前量"量级。
                #   ⚠ 调试配置曾滞留旧值 1.0/1.8，单独起本节点会复现 V0.0.96 死锁，务必与生产同步。
                'stop_dist': 0.90,          # 行进净空 < 0.90m 零速（> range_min 0.70 + 0.15）
                'slow_dist': 1.40,          # 净空 < 1.40m 线性限速
                'sector_half_deg': 60.0,    # 检测扇区半角（°）
                'corridor_half_width': 0.45,   # 安全走廊半宽（m）
                'footprint_front': 0.45,    # 文档 9.3 车体 footprint
                'footprint_rear': 0.37,
                'footprint_half_width': 0.32,
                'self_margin': 0.12,        # 自车包络外扩：盒内回波判为自身反射
                # V0.0.95 阈值滞环 + 幽灵点门控（消除阈值抖振与单点误急停，与生产同源）：
                #   ⚠ stop_release_hysteresis 必须 < 行为树 BackUp 的 backup_dist(0.45)，否则锁死原地
                'stop_release_hysteresis': 0.25,
                'slow_release_hysteresis': 0.20,
                'min_obstacle_points': 3,
                'obstacle_cluster_span': 0.25,
                # V0.0.98 轨迹扫掠弧碰撞闸（行进中按指令 (v,w) 积分阿克曼轨迹+包络盒扫掠判接触，
                #   静止/蠕行退回直线走廊）——与生产 hunter_autonomous_nav.launch.py 同源：
                'reaction_lag': 0.4,        # 指令执行滞后（s），期内按直线积分
                'brake_decel': 1.5,         # 制动减速度（m/s²），与 velocity_smoother 一致
                'vel_trust_eps': 0.03,      # |v| ≤ 此值用直线走廊防抖
                'scan_timeout': 0.5,        # /scan 断流 fail-safe（s）
                'cmd_timeout': 0.5,         # 上游指令断流看门狗（s）
                'enable_test_mode': False,  # V0.0.86 测试模式启动默认关（运行时经 /safety/test_mode 开关）
                # V0.0.87 地图边界监护（单独调试时需 map_server + 重定位已运行）
                'enable_map_fence': True,
                'map_edge_stop_dist': 0.4,  # V0.0.89 窄场地（与生产一致，原 0.5）
                'map_edge_slow_dist': 1.0,  # V0.0.89 距未建图/界外 <1.0m 限速（与生产一致，原 1.5）
                'use_sim_time': False,
            }],
        ),
    ])
