r"""hunter_sil.launch.py — HunterEdge 真 SIL 模式整栈启动（V0.1.38）。

与 hunter_edge.launch.py（实车生产）的关系
------------------------------------------
**被测软件完全相同，只换输入来源**：SIL 模式下不启动任何真实硬件驱动与 LIO 定位，
改由 sil_bridge_agent 把 CARLA 仿真传感器经 Kafka 注入到**同一批 ROS 话题**，
全栈（感知 → 融合 → 决策 → auto_mission）照常运行，产出的 /cmd_vel 再经桥接回传仿真。

    CARLA → HunterSim EdgeLinkRuntime → Kafka carla.{sim_id}.edge_sensor
      → sil_bridge_agent → /lidar_points /camera/camera/... /imu/data
                           /chassis/state /localization/odom /Odometry + TF
      → lidar/vision/fusion → decision_making → auto_mission(mapping cruise)
      → /cmd_vel → sil_bridge_agent → Kafka carla.{sim_id}.edge_control
      → HunterSim control_driver hook → 仿真自车

不启动的节点（逐项理由）
------------------------
* rslidar_sdk / realsense2_camera / ch10x_driver：真实传感器驱动。sil_bridge 独占
  它们的输出话题，同时启动会形成**双发布者**（点云/图像互相覆盖，感知结果不可信）。
* hunter_base（CAN 底盘）：SIL 无 CAN 硬件；/chassis/state 由 sil_bridge 用仿真真值
  发布，/cmd_vel 由桥接回传仿真而非下发底盘。
* localization.launch.py（fast_lio2 + EKF）：SIL 用**仿真真值定位**替代 LIO，
  排除定位误差这一变量（评估裁决的是全栈决策/控制表现，不是 LIO 精度）。
  odom→base_link TF 原由 EKF publish_tf=true 发布，现由 sil_bridge 接管。
* hunter_autonomous_nav.launch.py：其 mapping 分支会带起 fast_lio2_param_injector
  与 pcd_to_map（都依赖 fast_lio2 写 PCD），nav 分支还会起 Nav2 + NDT 重定位。
  SIL 两者都不需要，故本文件**直接编排 auto_mission_node(mapping)**，
  并自带 pointcloud_to_laserscan（原只在 nav 分支里）。
* hunter_relocalization（NDT，map→odom）：SIL 真值世界系即 map 系，由 sil_bridge
  发 map→odom 恒等 TF。**已核实 mapping cruise 不依赖 /relocalization/pose**：
  startCruiseCallback 只校验 mapping 模式 / 非急停 / 航点热重载 / /Odometry 新鲜；
  isLocalizationValid()（要求 NDT 收敛）仅在 nav 模式状态机里调用。
* safety_guard：mapping 模式下实车栈本就不启动它，auto_mission cruise 是 /cmd_vel
  的**唯一所有者**（见 hunter_autonomous_nav.launch.py 注释）。SIL 保持一致，
  避免双发布者；障碍制动由 auto_mission 自身的 stop/warn_obstacle_dist 阈值负责。
* rviz2：车端无头运行纪律（V0.1.30 防复发加固①），SIL 同样严禁默认起 GUI。

巡航启动门控链（**排查「车不动」按此顺序查**）
----------------------------------------------
1. decision_making 必须输出 AUTO：其仲裁为 ESTOP > REMOTE > AUTO，而
   `estop_ || chassis_fault || chassis_stale → ESTOP`。故 **/chassis/state 必须新鲜**
   —— 它由 sil_bridge 从 `ego_state` 流发布，**ego_state 是必需流**，缺它整条巡航
   被 ESTOP 闸死。同时 fault_code=0 / vehicle_state=NORMAL（桥接已保证）。
2. auto_mission mapping：`pause = (behavior.mode != "AUTO") || health==CRITICAL`。
   故默认**不启动 health_monitor**：SIL 无 CAN，它会持续报 CRITICAL → 巡航永久暂停。
3. `/Odometry` 新鲜（cruise_odom_timeout_）：sil_bridge 用仿真真值发布。
4. `/perception/fused_objects` 新鲜（isPerceptionAlive）：感知子栈必须启动。
5. 航点非空：startCruiseCallback 会热重载 params_file 里的 waypoints。
   **SIL 航点必须是 CARLA 场景坐标**（Town 地图与实车地图不匹配，这正是选
   mapping cruise 而非 nav 的原因）；用 auto_params_file 指向 SIL 专用 YAML。

时钟纪律
--------
SIL **固定墙钟**，故意不提供 `use_sim_time` 旋钮：链路里没有 /clock 发布者，置 true
会让所有节点定时器停摆（表现为整栈静默不动且无报错），故各节点一律定 False。
sil_bridge 也一律用**墙钟**打 stamp（CARLA 仿真经过秒不是 Unix epoch，当 stamp 会被
TF/消息滤波器判为远古消息全量丢弃）。

用法
----
    # Jetson 上（先 source ROS2 与工作空间；凭据取 /etc/hunter/kafka）
    ros2 launch hunter_bringup hunter_sil.launch.py
    # auto_params_file 默认即 /etc/hunter/sil_waypoints.yaml（与 sil_bridge 的
    # waypoints_active_file 默认一致：部署 /etc/hunter/sil_waypoints.d/ 后，
    # boot 时按仿真地图自动装载，无需手工指定）

    # 仿真侧会话 boot 后，启动巡航（无需先验地图）
    ros2 service call /auto_mission/start_mapping_cruise std_srvs/srv/Trigger

    # 停止巡航（安全停车）
    ros2 service call /auto_mission/stop_mapping_cruise std_srvs/srv/Trigger

⚠ 实车模式严禁启用本 launch 与 sil-bridge.service（会与真实驱动/底盘抢话题）。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _isolated(source, *, condition=None, launch_arguments=None):
    """与 hunter_full.launch.py 同构：用 GroupAction 隔离父级 LaunchConfiguration。

    避免 use_perception 等参数被透传到不认识它们的子包。
    """
    kwargs = {}
    if condition is not None:
        kwargs['condition'] = condition
    include_kwargs = {}
    if launch_arguments is not None:
        include_kwargs['launch_arguments'] = launch_arguments.items()
    return GroupAction(
        scoped=True,
        forwarding=False,
        actions=[IncludeLaunchDescription(source, **include_kwargs)],
        **kwargs,
    )


def generate_launch_description():
    pkg_share = get_package_share_directory('hunter_bringup')
    sil_share = get_package_share_directory('sil_bridge_agent')

    use_perception = LaunchConfiguration('use_perception')
    use_data_agent = LaunchConfiguration('use_data_agent')
    use_command_agent = LaunchConfiguration('use_command_agent')
    use_health_monitor = LaunchConfiguration('use_health_monitor')

    declares = [
        DeclareLaunchArgument(
            'sil_params_file',
            default_value=os.environ.get(
                'HUNTER_SIL_CONFIG',
                os.path.join(sil_share, 'config', 'sil_bridge_params.yaml')),
            description='sil_bridge 参数 YAML（不含凭据；凭据取 /etc/hunter/kafka）'),
        DeclareLaunchArgument(
            'auto_params_file',
            default_value='/etc/hunter/sil_waypoints.yaml',
            description='auto_mission 参数 YAML（航点为 CARLA 场景坐标）；'
                        '默认与 sil_bridge waypoints_active_file 一致，'
                        'boot 时按地图从 sil_waypoints.d 自动装载'),
        DeclareLaunchArgument(
            'use_perception', default_value='true',
            description='激光/视觉感知与融合（cruise 的 isPerceptionAlive 门控依赖它）'),
        DeclareLaunchArgument(
            'use_data_agent', default_value='true',
            description='遥测/健康/事件上报（hunter.<vid>.telemetry|health|event）'),
        DeclareLaunchArgument(
            'use_command_agent', default_value='true',
            description='平台指令接入（hunter.<vid>.command → command_result）'),
        DeclareLaunchArgument(
            'use_health_monitor', default_value='false',
            description='默认关闭：SIL 无 CAN，health_monitor 会报 CRITICAL → '
                        'auto_mission mapping_paused_ 永久暂停巡航'),
    ]

    def local_src(*parts):
        return PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', *parts))

    def pkg_src(pkg, *parts):
        return PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(pkg), *parts))

    # ---- 1. URDF 静态 TF（base_link → rslidar/camera_color_optical_frame/imu）----
    # 必须保留：pointcloud_to_laserscan 的 target_frame=base_link 与感知的外参换算
    # 全靠它；缺它则 /scan 全丢帧。动态 TF（odom→base_link、map→odom）由 sil_bridge 发。
    description = _isolated(local_src('description.launch.py'))

    # ---- 2. SIL 桥接（替代传感器驱动 + CAN + LIO/EKF + 全局定位）----
    # 与 hunter_full.launch.py §6b 同一坑：GroupAction(scoped=True, forwarding=False)
    # 建立空作用域后才对 launch_arguments 里的 LaunchConfiguration 求值，父作用域
    # 的 sil_params_file 已被清空 → "launch configuration does not exist"。
    # 故用 OpaqueFunction 在 execute 阶段（Declare 已完成）读父作用域的值，
    # 再以字面量传入 _isolated，保留作用域隔离语义。
    def _launch_sil_bridge(context, *args, **kwargs):
        params_file = context.launch_configurations.get(
            'sil_params_file',
            os.path.join(sil_share, 'config', 'sil_bridge_params.yaml'))
        return [_isolated(
            pkg_src('sil_bridge_agent', 'launch', 'sil_bridge.launch.py'),
            launch_arguments={'params_file': params_file})]

    sil_bridge = OpaqueFunction(function=_launch_sil_bridge)

    # ---- 3. 点云 → 激光扫描（原只在 hunter_autonomous_nav 的 nav 分支）----
    # 参数与 nav 分支同源（range_min 0.70 / 1° 束 / transform_tolerance 0.3），
    # 保证 SIL 与实车的 /scan 几何一致，评估结论才可迁移。
    cloud_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[('cloud_in', '/lidar_points'), ('scan', '/scan')],
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.3,
            'min_height': -0.2,
            'max_height': 0.8,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.017453293,
            'scan_time': 0.1,
            'range_min': 0.70,
            'range_max': 50.0,
            'use_inf': True,
            # SIL 不变量：无 /clock 发布者，固定墙钟（见模块 docstring「时钟纪律」）
            'use_sim_time': False,
        }],
    )

    # ---- 4. 感知 + 融合（被测对象，与实车完全同一套参数）----
    perception = _isolated(local_src('perception.launch.py'),
                           condition=IfCondition(use_perception))

    # ---- 5. 决策仲裁（输出 AUTO 是 cruise 的前置门控）----
    decision_making = _isolated(
        pkg_src('decision_making', 'launch', 'decision_making.launch.py'))

    # ---- 6. auto_mission（mapping cruise，/cmd_vel 唯一所有者）----
    # 参数与 hunter_autonomous_nav.launch.py 的 _mapping_nodes 逐项对齐，
    # 只去掉 fast_lio2_param_injector / pcd_to_map / waypoint_recorder（依赖 LIO 写盘）。
    auto_mission = Node(
        package='auto_mission',
        executable='auto_mission_node',
        name='auto_mission_node',
        output='screen',
        parameters=[
            LaunchConfiguration('auto_params_file'),
            {'mission_mode': 'mapping'},
            # start_mapping_cruise 服务触发时从此文件热重载航点
            {'params_file': LaunchConfiguration('auto_params_file')},
            {'enable_mapping_remote_bridge': True},
            {'remote_cmd_topic': '/remote/command'},
            {'mapping_remote_timeout': 2.0},
            {'mapping_wheelbase': 0.46},
            {'mapping_remote_max_vel': 2.0},
            {'use_sim_time': False},   # SIL 不变量：固定墙钟，无 /clock 发布者
        ],
    )

    # ---- 7. 平台互联 Agent（车端→平台契约不变）----
    data_agent = _isolated(pkg_src('data_agent', 'launch', 'data_agent.launch.py'),
                           condition=IfCondition(use_data_agent))
    command_agent = _isolated(
        pkg_src('command_agent', 'launch', 'command_agent.launch.py'),
        condition=IfCondition(use_command_agent))

    # ---- 8. health_monitor（默认关闭，见 use_health_monitor 说明）----
    health_monitor = _isolated(
        pkg_src('health_monitor', 'launch', 'health_monitor.launch.py'),
        condition=IfCondition(use_health_monitor))

    banner = LogInfo(msg=(
        '\n'
        '════════════ HunterEdge 真 SIL 模式（被测全栈 = 实车同一套） ════════════\n'
        '  · 输入：sil_bridge_agent ← Kafka carla.{sim_id}.edge_sensor（CARLA 传感器）\n'
        '  · 定位：仿真真值（/localization/odom + /Odometry + odom/map TF），不启 LIO/EKF\n'
        '  · 输出：/cmd_vel → Kafka carla.{sim_id}.edge_control → 仿真自车\n'
        '  · 不启动：传感器驱动 / CAN / fast_lio2 / EKF / NDT / Nav2 / safety_guard / rviz2\n'
        '  启动巡航：ros2 service call /auto_mission/start_mapping_cruise std_srvs/srv/Trigger\n'
        '  车不动排查：/chassis/state 新鲜? → behavior.mode=AUTO? → /Odometry 新鲜?\n'
        '              → /perception/fused_objects 新鲜? → 航点非空(CARLA 坐标)?\n'
        '══════════════════════════════════════════════════════════════════════'
    ))

    return LaunchDescription(declares + [
        banner,
        description,
        sil_bridge,
        cloud_to_scan,
        perception,
        decision_making,
        auto_mission,
        data_agent,
        command_agent,
        health_monitor,
    ])
