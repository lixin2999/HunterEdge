"""hunter_edge.launch.py — HunterEdge 生产开机自启入口（需求②③④）。

与 hunter_full.launch.py 的关系
--------------------------------
**不重复实现启动逻辑**：本文件只是生产默认值的“薄封装”，把整栈启动收敛成一个
可被 systemd 直接调用的入口（`hunter-edge.service` → `hunter_edge_up.sh` → 本文件）。

生产默认值（与 `hunter_full.launch.py` 的调试默认值差异即“上电即用”语义）：

| 参数 | hunter_full 默认 | 本文件默认 | 理由 |
|---|---|---|---|
| `use_autonomous_nav` | `false` | **`true`** | 需求②“车辆进入自动驾驶准备状态”：定位/感知/Nav2/auto_mission 全部就绪待命，收到平台指令即可进入相应模式（`command_agent` 消费 `hunter.<vid>.command`） |
| `autonomous_nav_mode` | `nav` | `nav` | 生产为已建图后的导航巡航；建图调试用 `HUNTER_EDGE_NAV_MODE=mapping` |
| `use_data_agent` | `true` | `true` | 需求③“按 Kafka 消息格式上传数据到运营端”（telemetry 10Hz / health 1Hz / event） |
| `use_command_agent` | `true` | `true` | 需求④“接收自动驾驶指令并进入相应模式 + 回执 command_result” |

话题/消息/服务零新增（.ai‑rules 第 3 条）：本文件不引入任何新的 ROS 接口，
仅改变既有 launch 的默认开关取值。

⚠ 车端无头运行纪律（V0.1.30 防复发加固①）
------------------------------------------------
本入口及其透传的 hunter_full.launch.py **不启动 RViz**（全 launch 树无
`executable='rviz2'` 节点，rviz2 仅出现在注释里作为人工联调手段）。
根因回顾：车端只有 12 核，调试期手工拉起 rviz2 点云累积显示会抢占 ~3 核
（实测 load 20+ / rviz 306% CPU），使 FAST-LIO2 起步即被 CPU 超订饿到
**里程计发散**（/Odometry 位姿冲到 1e12 m、ikd-tree 内存暴涨、掉帧死亡螺旋），
进而 map→odom TF 断裂 → NDT 重定位拿不到合理 odom→base 也发散 → safety_guard
把垃圾位姿判为地图越界 → MAP_EDGE_STOP 假锁死 → 遥控指令车不动。
因此：**生产/车上严禁默认起 RViz**；确需可视化请优先用远程侧（回传话题到工位机
rviz），或在整车空载、fast_lio2 已稳定 10Hz 收敛后短时开启、用完即关。
加固③（OMP 限线程 + systemd Nice 提优先级）已保证即便有人误起 GUI，fast_lio2
仍能优先拿到 CPU 不被饿死；本条是首要的行为约束。

用法
----
    # 与开机自启完全同一条路径（推荐；含 CAN 复查 + 运营端预检）
    bash src/hunter_bringup/scripts/hunter_edge_up.sh

    # 单独调试（需自行 source ROS/工作空间）
    ros2 launch hunter_bringup hunter_edge.launch.py
    ros2 launch hunter_bringup hunter_edge.launch.py use_autonomous_nav:=false   # 仅链路联调
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('hunter_bringup')
    full_launch = os.path.join(pkg_share, 'launch', 'hunter_full.launch.py')

    # ── 生产默认（上电即用；均可用 launch 参数或环境变量覆盖）────────────
    use_autonomous_nav = LaunchConfiguration('use_autonomous_nav')
    autonomous_nav_mode = LaunchConfiguration('autonomous_nav_mode')
    use_data_agent = LaunchConfiguration('use_data_agent')
    use_command_agent = LaunchConfiguration('use_command_agent')
    use_perception = LaunchConfiguration('use_perception')
    map_yaml_path = LaunchConfiguration('map_yaml_path')
    map_file_path = LaunchConfiguration('map_file_path')

    declares = [
        DeclareLaunchArgument(
            'use_autonomous_nav', default_value='true',
            description='true（默认）= 进入自动驾驶准备状态；false = 仅启动链路（不含 Nav2/auto_mission）'),
        DeclareLaunchArgument(
            'autonomous_nav_mode', default_value='nav',
            description='nav = 导航巡航（默认）；mapping = 建图模式'),
        DeclareLaunchArgument(
            'use_data_agent', default_value='true',
            description='遥测/健康/事件上报（hunter.<vid>.telemetry|health|event）'),
        DeclareLaunchArgument(
            'use_command_agent', default_value='true',
            description='平台指令接入（hunter.<vid>.command → 车端服务/话题 → command_result）'),
        DeclareLaunchArgument(
            'use_perception', default_value='true',
            description='激光/视觉感知与融合'),
        DeclareLaunchArgument(
            'map_yaml_path',
            default_value=os.path.expanduser('~/HunterEdge/maps/hunter_map.yaml'),
            description='[nav] 静态地图 YAML'),
        DeclareLaunchArgument(
            'map_file_path',
            default_value=os.path.expanduser('~/HunterEdge/maps/hunter_map.pcd'),
            description='[mapping] PCD 保存路径 / [nav] 先验点云地图'),
    ]

    # 透传给整栈 launch（不重写任何子模块逻辑）
    full = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(full_launch),
        launch_arguments={
            'use_perception': use_perception,
            'use_data_agent': use_data_agent,
            'use_command_agent': use_command_agent,
            'use_autonomous_nav': use_autonomous_nav,
            'autonomous_nav_mode': autonomous_nav_mode,
            'map_yaml_path': map_yaml_path,
            'map_file_path': map_file_path,
        }.items(),
    )

    banner = LogInfo(msg=(
        '\n'
        '════════════ HunterEdge 自动驾驶准备状态 ════════════\n'
        '  · 底盘 CAN：can2 @500k（由 hunter-can.service 提前拉起并抓帧验证）\n'
        '  · 运营端接入：telemetry 10Hz / health 1Hz / event 事件即发\n'
        '                下行 command 消费 + command_result 回执\n'
        '  · 自动驾驶：定位 → 感知 → Nav2 → auto_mission 就绪待命\n'
        '  判据：平台侧该车 last_online_time 刷新；车端 hunter-kafka-check 退 0\n'
        '  现场状态：bash src/hunter_bringup/scripts/hunter_status.sh\n'
        '════════════════════════════════════════════════════'
    ))

    return LaunchDescription(declares + [banner, full])
