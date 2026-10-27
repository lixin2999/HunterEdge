# HunterEdge版本更新文档

## V0.0.1

Date：2026-8-25
版本主要有以下功能：
1. 模块契约摘要
根据设计文档4.2章节，HunterEdge ROS2工作空间包含以下模块契约：
-核心模块分类：
--hunter_bringup: 启动配置与参数管理
--hunter_drivers: 自定义传感器驱动（ch10x_driver）
--hunter_perception: 感知算法（lidar_perception, vision_perception）
--hunter_agents: 平台交互Agent（data_agent, ota_agent, remote_agent）
--hunter_common: 公共库（hunter_msgs, hunter_utils）
--hunter_monitor: 系统监控（health_monitor）
--第三方依赖包（待添加）：
---ugv_sdk, hunter_ros2, rslidar_sdk, realsense-ros, fast_lio2, robot_localization, navigation2, yolo_trt_ros
2. 生成的工作空间结构，生成器创建以下结构：
~/HunterEdge/src/
├── hunter_bringup/                    # 启动配置与参数
│   ├── launch/
│   │   ├── hunter_full.launch.py      # 全系统启动
│   │   ├── perception.launch.py       # 感知模块启动
│   │   ├── navigation.launch.py       # Nav2导航启动
│   │   ├── localization.launch.py     # 定位模块启动
│   │   └── data_agent.launch.py       # 数据采集启动
│   ├── config/
│   │   ├── sensor_params.yaml         # 传感器参数
│   │   ├── nav2_params.yaml           # Nav2参数
│   │   ├── fast_lio2_params.yaml      # FAST-LIO2参数
│   │   ├── ekf_params.yaml            # robot_localization EKF参数
│   │   └── system_params.yaml         # 系统参数
│   ├── behavior_trees/
│   │   └── navigate_to_pose.xml       # Nav2行为树
│   ├── urdf/
│   │   └── hunter_se.urdf.xacro       # 车辆URDF模型
│   ├── CMakeLists.txt
│   └── package.xml
│
├── hunter_drivers/                    # 传感器驱动(自定义)
│   └── ch10x_driver/                  # CH10X IMU驱动
│       ├── src/
│       ├── include/ch10x_driver/
│       ├── launch/
│       ├── config/
│       ├── CMakeLists.txt
│       └── package.xml
│
├── hunter_perception/                 # 感知算法(自定义)
│   ├── lidar_perception/              # 激光感知(地面分割+欧式聚类)
│   │   ├── src/
│   │   ├── include/lidar_perception/
│   │   ├── launch/
│   │   ├── config/
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── vision_perception/             # 视觉感知(YOLO+TensorRT)
│       ├── src/
│       ├── include/vision_perception/
│       ├── launch/
│       ├── config/
│       ├── vision_perception/
│       │   └── __init__.py
│       ├── setup.py
│       ├── setup.cfg
│       └── package.xml
│
├── hunter_agents/                     # 平台交互Agent(自定义)
│   ├── data_agent/                    # 数据采集上传(Kafka+MinIO)
│   │   ├── src/
│   │   ├── include/data_agent/
│   │   ├── launch/
│   │   ├── config/
│   │   ├── data_agent/
│   │   │   └── __init__.py
│   │   ├── setup.py
│   │   ├── setup.cfg
│   │   └── package.xml
│   ├── ota_agent/                     # OTA升级
│   │   ├── src/
│   │   ├── include/ota_agent/
│   │   ├── launch/
│   │   ├── config/
│   │   ├── ota_agent/
│   │   │   └── __init__.py
│   │   ├── setup.py
│   │   ├── setup.cfg
│   │   └── package.xml
│   └── remote_agent/                  # 远程操控(WebRTC)
│       ├── src/
│       ├── include/remote_agent/
│       ├── launch/
│       ├── config/
│       ├── remote_agent/
│       │   └── __init__.py
│       ├── setup.py
│       ├── setup.cfg
│       └── package.xml
│
├── hunter_common/                     # 公共库
│   ├── hunter_msgs/                   # 自定义消息类型
│   │   ├── msg/
│   │   │   └── .gitkeep
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── hunter_utils/                  # 工具函数库
│       ├── src/
│       ├── include/hunter_utils/
│       ├── launch/
│       ├── config/
│       ├── CMakeLists.txt
│       └── package.xml
│
└── hunter_monitor/                    # 系统监控
    └── health_monitor/
        ├── src/
        ├── include/health_monitor/
        ├── launch/
        ├── config/
        ├── CMakeLists.txt
        └── package.xml
3. 关键包package.xml模板
已为以下关键包生成package.xml模板：
-hunter_bringup: 包含ROS2 launch和配置依赖
-ch10x_driver: C++包，包含串口通信依赖
-lidar_perception: C++包，包含PCL点云处理依赖
-vision_perception: Python包，包含OpenCV和深度学习依赖
-data_agent: Python包，包含Kafka和MinIO依赖
-ota_agent: Python包，包含HTTP请求依赖
-remote_agent: Python包，包含WebRTC和视频处理依赖
-hunter_msgs: 消息定义包，包含ROS2消息生成依赖
-hunter_utils: C++工具库，包含数学和坐标变换依赖
-health_monitor: Python包，包含系统监控依赖

## V0.0.2

Date：2026-8-25
版本主要有以下功能：
第三方依赖包添加完成
已成功添加所有设计文档中指定的第三方依赖包到HunterEdge工作空间：
1. 添加的第三方包清单
- 单包结构：
--ugv_sdk: AgileX官方UGV SDK占位包
--hunter_ros2: AgileX官方HUNTER ROS2驱动占位包
--rslidar_sdk: RoboSense LiDAR驱动占位包
--fast_lio2: FAST-LIO2雷达惯导里程计占位包
--robot_localization: EKF传感器融合占位包
--yolo_trt_ros: YOLO TensorRT推理ROS封装占位包
- 多包结构：
--realsense-ros: Intel D435驱动组
--realsense2_camera (相机驱动)
--realsense2_description (URDF描述)
--realsense2_camera_msgs (自定义消息)
--navigation2: Nav2导航栈
--nav2_bringup (启动脚本)
--nav2_common (公共工具)
--nav2_controller (控制器RPP/MPPI)
--nav2_planner (规划器SmacPlannerHybrid)
--nav2_behavior_tree (行为树)
--nav2_costmap_2d (代价地图)
--nav2_core (核心库)
2. 生成的文件结构
每个第三方包都包含：
package.xml: 符合ROS2规范的包配置文件
CMakeLists.txt: 构建配置文件（占位符）
标准目录结构: src/, include/, launch/, config/, params/等
头文件目录: 按包名命名的include子目录

## V0.0.3

Date：2026-8-25
版本主要有以下功能：
1. 实现hunter_msgs 是 ROS2 ament_cmake + rosidl 接口包，位于 src/hunter_common/hunter_msgs/，实现文档 4.3 全部 6 个自定义消息。
消息	分类	字段数	用途
DetectedObject	感知	9	单个障碍物目标（类别/位姿/尺寸/速度/跟踪）
DetectedObjectArray	感知	2	障碍物目标列表
TrajectoryPoint	规划	8	轨迹点（位姿/速度/加速度/转角/时间）
Trajectory	规划	2	轨迹（点序列）
ChassisState	底盘	13	底盘完整状态（速度/转向/电池/电机/故障）
ChassisCommand	底盘	5	底盘控制指令（目标速度/转向/模式/急停）
依赖：仅 std_msgs（Header）与 geometry_msgs（Pose/Vector3/Twist），与文档 4.3 实际使用严格一致（已移除多余的 sensor_msgs/nav_msgs）。
2. 编译验证指令，在目标环境（Ubuntu 22.04 + ROS2 Humble）执行：
cd ~/HunterEdge
colcon build --packages-select hunter_msgs --symlink-install
source install/setup.bash
-- 校验接口已生成
ros2 interface list | grep hunter_msgs
-- 期望输出 6 条：
---   hunter_msgs/msg/ChassisCommand
---   hunter_msgs/msg/ChassisState
---   hunter_msgs/msg/DetectedObject
---   hunter_msgs/msg/DetectedObjectArray
---   hunter_msgs/msg/Trajectory
---   hunter_msgs/msg/TrajectoryPoint
-- 抽查单个消息定义
ros2 interface show hunter_msgs/msg/DetectedObject


## V0.0.4

Date：2026-8-25
版本主要有以下功能：
1. 车辆URDF/Xacro模型与静态TF
坐标系/link	| 原点 |	轴向	| 相对 base_link 偏移(附录D)	| rpy 旋转
base_link	|车辆后轴中心(距地面150mm) |	X前/Y左/Z上	| 原点	| —
lidar	    |LiDAR 光心	| X前/Y左/Z上	| x=0.23, y=0, z=0.35	| 0 0 0
camera_color_optical_frame	|D435 光心	| Z前/X右/Y下	| x=0.40, y=0, z=0.30	| 0 −π/2 π/2
imu	IMU 中心 |	X前/Y左/Z上	| x=0.10, y=0, z=0.20	| 0 0 0
2. __编译验证指令__（Ubuntu + ROS2 Humble）：
cd ~/HunterEdge
colcon build --packages-select hunter_bringup --symlink-install
source install/setup.bash
ros2 launch hunter_bringup description.launch.py
-- 校验静态 TF 树与偏移
ros2 run tf2_ros tf2_echo base_link lidar          # 应输出 translation (0.23, 0.00, 0.35)
ros2 run tf2_ros tf2_echo base_link imu            # 应输出 translation (0.10, 0.00, 0.20)
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
-- camera 应输出 translation (0.40, 0.00, 0.30)，rotation 对应 rpy(0, -π/2, π/2)

## V0.0.5

Date：2026-8-25
版本主要有以下功能：
1. hunter\_bringup 基础配置 yaml 集合
配置文件	|对应节点	|文档出处	|核心内容
sensor_params.yaml	|传感器驱动	|附录 C + 3.3	|LiDAR/Camera/IMU 话题、频率、ROI、硬件规格
fast_lio2_params.yaml	|fast_lio2	|7.2 + 3.3.1/3.3.3 + 附录 D	|16线雷达、100Hz IMU、ikd-Tree、IEKF、外参
ekf_params.yaml	|robot_localization	|7.3	|15维状态、三传感器融合、50Hz、协方差矩阵
nav2_params.yaml	|Nav2 栈	|附录 C + 8.3/9.2/9.3/10.2	|SmacPlannerHybrid、RPP、代价地图、footprint
system_params.yaml	|health_monitor	15/3.5/10.5/14.3/19.3	|限速、温度阈值、传感器超时、事件检测

## V0.0.6

Date：2026-8-25
版本主要有以下功能：
1. 实现CH10X UART读取、协议解析，发布/imu/data；处理串口断开、数据超时异常。
项	|内容
节点名	|ch10x_driver（rclcpp_lifecycle::LifecycleNode）
发布话题	|/imu/data（sensor_msgs/msg/Imu，SensorDataQoS）
发布频率	|100Hz（由设备帧率驱动，文档 3.3.3）
坐标系	|frame_id = "imu"（文档 7.4）
订阅话题	|无（纯传感器驱动）
串口	|UART TTL，默认 /dev/ttyUSB0 @ 115200，8N1
协议	|自定义帧：0xAA 0x55 帧头 + 类型(0x01) + 长度 + 载荷 + 8-bit 校验和；载荷含四元数/欧拉角/角速度/加速度
2. __编译验证指令__（Ubuntu + ROS2 Humble）：

```bash
cd ~/HunterEdge
colcon build --packages-select ch10x_driver --symlink-install
source install/setup.bash
ros2 launch ch10x_driver ch10x_driver.launch.py
# 另开终端校验话题与 frame_id
ros2 topic echo /imu/data --field header.frame_id   # 应输出 "imu"
ros2 topic hz /imu/data                              # 应接近 100Hz
```

## V0.0.7

Date：2026-8-25
版本主要有以下功能：
1. 激光感知节点完整链路全部实现
项	|内容
节点名	|lidar_perception（rclcpp_lifecycle::LifecycleNode）
输入话题	|/lidar_points（sensor_msgs/PointCloud2，10Hz）、/imu/data（sensor_msgs/Imu）
输出话题	|/perception/lidar_objects（DetectedObjectArray，10Hz）
调试点云	|/perception/lidar/ground_cloud、/perception/lidar/obstacle_cloud（PointCloud2）
调试 marker	|/perception/lidar/cluster_markers（MarkerArray）
坐标系	|输出 frame_id = base_link（TF：lidar → base_link）
2. 处理链路
PointCloud2 → ①预处理 → ②运动去畸变(IMU) → ③坐标系转换(lidar→base_link)
→ ④ROI裁剪(车前120°/50m) → ⑤离群点去除(SOR) → ⑥射线坡度法地面分割
→ ⑦欧式聚类 → ⑧OBB包围盒提取 → ⑨卡尔曼+匈牙利多目标跟踪 → 发布
3. __编译验证指令__（Ubuntu + ROS2 Humble）：

```bash
cd ~/HunterEdge
colcon build --packages-select lidar_perception --symlink-install
source install/setup.bash
ros2 launch lidar_perception lidar_perception.launch.py
# 校验话题与频率
ros2 topic list | grep perception
ros2 topic echo /perception/lidar_objects --field header.frame_id   # base_link
ros2 topic hz /perception/lidar_objects                              # ~10Hz
```

## V0.0.8

Date：2026-8-25
版本主要有以下功能：
1. 视觉感知节点（含 TensorRT 推理）
项	|内容
节点名	|vision_perception（rclcpp_lifecycle::LifecycleNode）
输入话题	|/camera/color/image_raw（sensor_msgs/Image，30Hz）、/camera/depth/image_rect_raw（Image，16UC1）
输出话题	|/perception/vision_objects（DetectedObjectArray，15Hz）
坐标系	|frame_id = camera_color_optical_frame（文档 7.4，Z前/X右/Y下）
推理	|YOLOv8n TensorRT FP16
2. 处理链路
RGB(30Hz) → ①预处理(OpenCV) → ②YOLOv8n TensorRT 推理 → ③深度图2D→3D投影 → ④发布(15Hz)
深度图 → 缓存(线程安全)
3. __编译验证指令__（Ubuntu + JetPack）：

```bash
cd ~/HunterEdge
colcon build --packages-select vision_perception --symlink-install
source install/setup.bash
ros2 launch vision_perception vision_perception.launch.py
ros2 topic hz /perception/vision_objects          # ~15Hz
ros2 topic echo /perception/vision_objects --field header.frame_id # camera_color_optical_frame
```

## V0.0.9

Date：2026-8-25
版本主要有以下功能：
1. 多传感器目标级数据融合节点，时间对齐、匈牙利算法关联匹配、加权状态融合；构建可行驶区域OccupancyGrid。
项	|内容
节点名	|sensor_fusion（rclcpp::Node）
输入话题	|/perception/lidar_objects（DetectedObjectArray，10Hz）、/perception/vision_objects（15Hz）
输出话题	|/perception/fused_objects（10Hz）、/perception/freespace（nav_msgs/OccupancyGrid，10Hz）
坐标系	|base_link
融合方式	|目标级后期融合
2. __处理链路__：
lidar_objects(10Hz) ─┐
                       ├→ 时间对齐 → 匈牙利关联 → 加权融合 → fused_objects(10Hz)
vision_objects(15Hz) ─┘                                    └→ freespace(10Hz)
3. __编译验证指令__（Ubuntu + ROS2 Humble）：

```bash
cd ~/HunterEdge
colcon build --packages-select sensor_fusion --symlink-install
source install/setup.bash
ros2 launch sensor_fusion sensor_fusion.launch.py
ros2 topic hz /perception/fused_objects   # ~10Hz
ros2 topic hz /perception/freespace       # ~10Hz
ros2 topic echo /perception/fused_objects --field header.frame_id   # base_link
```

## V0.0.10

Date：2026-8-25
版本主要有以下功能：
1. 定位模块启动配置：FAST‑LIO2 + robot_localization EKF
项	|内容
launch 文件	|hunter_bringup/launch/localization.launch.py
启动节点	|fast_lio2（fastlio_mapping）、ekf_filter_node（robot_localization）、robot_state_publisher（静态 TF）
输入话题	|/lidar_points（10Hz）、/imu/data（100Hz）、/chassis/feedback（经 hunter_ros2 转 /odom）
中间话题	|/Odometry（FAST-LIO2，10Hz）
输出话题	|/localization/odom（50Hz）、/localization/pose（50Hz）
加载参数	|fast_lio2_params.yaml、ekf_params.yaml
2. __编译/启动验证__（Ubuntu + ROS2 Humble）：

```bash
cd ~/HunterEdge
colcon build --packages-select hunter_bringup --symlink-install
source install/setup.bash
ros2 launch hunter_bringup localization.launch.py
# 校验输出频率与 TF
ros2 topic hz /localization/odom    # ~50Hz
ros2 run tf2_ros tf2_echo odom base_link   # EKF 发布 odom→base_link
```

## V0.0.11

Date：2026-8-25
版本主要有以下功能：
1. Nav2导航栈启动配置 SmacPlannerHybrid + RPP控制器
项	|内容
launch 文件	|hunter_bringup/launch/navigation.launch.py
行为树	|hunter_bringup/behavior_trees/navigate_to_pose.xml
启动组件	|controller_server(RPP)、planner_server(SmacPlannerHybrid)、recoveries_server、bt_navigator、velocity_smoother、lifecycle_manager
参数	|nav2_params.yaml（任务 03 + 本次补充 bt_navigator/velocity_smoother）
运动模型	|REEDS_SHEPP
最小转弯半径	|1.9m
footprint	|[ [0.45,0.32],[0.45,-0.32],[-0.37,-0.32],[-0.37,0.32] ]
2. __启动验证__（Ubuntu + ROS2 Humble）：

```bash
cd ~/HunterEdge
colcon build --packages-select hunter_bringup --symlink-install
source install/setup.bash
ros2 launch hunter_bringup navigation.launch.py
ros2 node list    # 应含 controller_server/planner_server/bt_navigator 等
```

## V0.0.12

Date：2026-8-25
版本主要有以下功能：
1. 模式仲裁决策节点 decision_making。
项	|内容
节点名	|decision_making（rclcpp_lifecycle::LifecycleNode）
输入话题	|/cmd_vel（Nav2，Twist）、/remote/command（ChassisCommand）、/chassis/state（ChassisState）、/estop（Bool）
输出话题	|/control/command（ChassisCommand，50Hz）、/planning/behavior_state（BehaviorState）
模式状态机	|AUTO / REMOTE / ESTOP，优先级 ESTOP > REMOTE > AUTO（文档 13.5）
新增消息	|hunter_msgs/msg/BehaviorState.msg
2. __仲裁逻辑__：

```javascript
estop信号 / 底盘故障 / 底盘通信丢失 ──→ ESTOP（零速度 + 紧急制动）
远程指令有效（未超时）              ──→ REMOTE（转发远程指令）
否则                              ──→ AUTO（Nav2 /cmd_vel → 阿克曼转向）
```
3. __编译/启动验证__（Ubuntu + ROS2 Humble）：

```bash
cd ~/HunterEdge
colcon build --packages-select hunter_msgs decision_making --symlink-install
source install/setup.bash
ros2 launch decision_making decision_making.launch.py
ros2 topic echo /planning/behavior_state --field mode   # 观察 AUTO/REMOTE/ESTOP 切换
```

## V0.0.13

Date：2026-8-25
版本主要有以下功能：
1. 系统监控健康管理节点 health\_monitor
节点存活监控；传感器话题频率监控；CPU/GPU/内存/磁盘/温度采集；故障降级策略；发布/system/health；节点崩溃自动重启计数；关键节点故障触发安全停车。
项	|内容
节点名	|health_monitor（rclcpp::Node）
监控输入	|/lidar_points、/camera/color/image_raw、/imu/data、/chassis/state
输出话题	|/system/health（SystemHealth，1Hz）、/estop（Bool，安全停车）
新增消息	|hunter_msgs/msg/SystemHealth.msg
功能	|节点存活监控、传感器频率监控、资源采集、故障降级、重启计数
2. __编译/启动验证__（Ubuntu + ROS2 Humble）：

```bash
cd ~/HunterEdge
colcon build --packages-select hunter_msgs health_monitor --symlink-install
source install/setup.bash
ros2 launch health_monitor health_monitor.launch.py
ros2 topic hz /system/health     # ~1Hz
ros2 topic echo /system/health --field overall_status # OK/WARNING/CRITICAL
```

## V0.0.14

Date：2026-8-25
版本主要有以下功能：
1. 数据采集 Agent data\_agent ROS 节点
订阅指定ROS话题降采样遥测打包；内置事件检测规则；Kafka生产者上报遥测、事件；本地SQLite缓存；触发rosbag录制；文件上传MinIO。
项	|内容
节点名	|data_agent（rclcpp::Node）
订阅话题	|/chassis/state、/localization/odom、/perception/fused_objects、/planning/trajectory、/control/command、/system/health（文档 14.2.1）
输出	|Kafka 遥测 hunter.{vehicle_id}.telemetry、事件 hunter.{vehicle_id}.event；触发 rosbag 录制
遥测频率	|10Hz（文档 14.2.2）
缓存	|SQLite 本地缓存（断线时），最多 24 小时
依赖	|librdkafka（Kafka）、sqlite3
2. __编译/启动验证__（Ubuntu + ROS2 Humble）：

```bash
cd ~/HunterEdge
colcon build --packages-select data_agent --symlink-install
source install/setup.bash
ros2 launch data_agent data_agent.launch.py
# Kafka 侧观察遥测/事件消息；断线时观察 SQLite 缓存
```

## V0.0.15

Date：2026-8-25
版本主要有以下功能：
1. OTA Agent systemd 服务（非 ROS 进程）
项	|内容
服务类型	|独立 systemd 服务（非 ROS 节点，Python 实现）
主程序	|ota_agent/ota_agent.py
状态机	|IDLE → PENDING → DOWNLOAD → INSTALL → TEST → SUCCESS/ROLLBACK/FAILED
Kafka	|订阅 hunter.{vehicle_id}.ota_notify，上报 hunter.{vehicle_id}.ota_status
安全	|SHA-256 完整性校验 + RSA-2048 数字签名
回滚	|备份恢复方案
2. __部署命令__：

```bash
sudo cp ota-agent.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now ota-agent
```

## V0.0.16

Date：2026-8-25
版本主要有以下功能：
1. Remote Agent 远程操控 systemd 服务（非 ROS 进程）
项	|内容
服务类型	|独立 systemd 服务（非 ROS 节点，Python 实现）
主程序	|remote_agent/remote_agent.py
视频	|GStreamer AGX 硬件 H264 编码（D435 RGB，1280×720@30fps，3Mbps CBR）
信令	|WebSocket 连接 SRS 媒体服务器
指令输入	|WebSocket 接收 JSON（velocity/steering_angle/brake）
指令输出	|ChassisCommand 发布 /remote/command
状态上报	|Kafka hunter.{vehicle_id}.remote_status

## V0.0.17

Date：2026-8-25
版本主要有以下功能：
1. 全系统总启动：ROS2 daemon →传感器驱动→CAN驱动→定位→感知→融合→决策→规划→控制→Agent→health_monitor。
项	|内容
总启动	|hunter_bringup/launch/hunter_full.launch.py
感知子 launch	|hunter_bringup/launch/perception.launch.py（新建）
参数开关	|use_perception、use_navigation、use_data_agent（默认 true）
启动顺序	|传感器驱动 → CAN驱动 → 定位 → 感知 → 融合 → 决策 → 规划 → 控制 → Agent → health_monitor
2. 启动验证（Ubuntu + ROS2 Humble）：
cd ~/HunterEdge
colcon build --packages-select hunter_bringup --symlink-install
source install/setup.bash
全系统启动
ros2 launch hunter_bringup hunter_full.launch.py
仅规划控制（关闭感知）
ros2 launch hunter_bringup hunter_full.launch.py use_perception:=false
数据采集模式
ros2 launch hunter_bringup hunter_full.launch.py use_navigation:=false use_data_agent:=true

## V0.0.18

Date：2026-8-25
版本主要有以下功能：
1. 运维 Shell 工具集，查看节点、资源；日志导出；CAN调试；bag录制回放。
脚本	|功能	
hunter_status.sh	|查看节点、关键节点存活、CPU/内存/磁盘/温度、话题频率	
hunter_log.sh	|查看/导出 ROS 日志 + 系统日志（tar.gz 打包）	
hunter_can_test.sh	|CAN 配置、监听、发送、反馈报文检测	
hunter_bag.sh	|ROS Bag 录制/回放/信息查询/列表	

## V0.0.19

Date：2026-8-25
版本主要有以下功能：
1. 导入第三方依赖包：
| 包 | 用途 |
|----|------|
| `hunter_ros2` / `ugv_sdk` | AgileX HUNTER SE 底盘驱动 |
| `rslidar_sdk` | RS-Helios-16P 激光雷达驱动 |
| `realsense-ros` | Intel RealSense D435 相机驱动 |
| `fast_lio2` | 紧耦合雷达惯导里程计 |
| `robot_localization` | EKF 多传感器融合定位 |
| `navigation2` | Nav2 导航栈（SmacPlannerHybrid + RPP） |
| `yolo_trt_ros` | YOLO TensorRT 推理封装（可选） |

## V0.0.20

Date：2026-8-26
版本主要有以下功能：
1. 导入第三方依赖包脚本

## V0.0.21

Date：2026-8-27
版本主要有以下功能：
1. ✅ ChassisState.msg 消息结构修正
修正内容: 移除了私自新增的 motor_rpm 和 motor_current 字段
影响: 消息契约恢复与设计文档4.3.3节完全一致
2. ✅ decision_making 架构违规修正
修正内容:
移除对 /cmd_vel 的订阅
移除对 /control/command 的发布
回归为仅发布 /planning/behavior_state (10Hz)
调整仲裁定时器频率从50Hz到10Hz
影响: 恢复正确的数据流转，符合文档16.2节架构设计
3. ✅ remote_agent 频率违规修正
修正内容:
引入20Hz独立定时器 self.timer = self.ros_node.create_timer(0.05, self.timer_callback)
将超时检查融入定时器逻辑
WebSocket事件仅更新最新指令值
影响: 确保 /remote/command 严格遵循文档17章规定的20Hz频率
4. ✅ sensor_fusion 坐标转换Bug修复
修正内容:
初始化TF2: tf_buffer_ 和 tf_listener_
在 visionCallback 中强制将视觉目标从 camera_color_optical_frame 转换到 base_link
坐标转换失败时放弃该帧融合
影响: 修复激光与视觉坐标系错位的严重Bug
5. ✅ lidar_perception 硬编码参数抽离
修正内容:
新增参数: scan_period、marker_lifetime、marker_alpha
更新YAML配置文件添加参数定义
代码中使用参数值替代硬编码常量
影响: 提高参数可配置性，便于不同硬件环境调优
文档更新
6. README.md 更新
添加版本信息和更新日期
新增"重要更新与修正"章节
说明架构合规性修正内容
添加用户手册引用
7. USER_MANUAL.md 更新
完整的用户手册文档
详细的架构合规性验证说明
每个修正项的问题、修正和影响说明
系统架构、构建部署、启动指南
接口定义、运维监控、安全机制
开发调试和故障诊断指南

## V0.0.22

Date：2026-8-27
版本主要有以下功能：
1. `hunter_ros2` 会带入其仓库自带的 `hunter_msgs` 包（`<name>hunter_msgs</name>`），与系统自定义的 `hunter_common/hunter_msgs` __包名重复__，colcon 构建将因同名包冲突而失败。
- 将 AgileX `hunter_msgs` 的 __5 条底盘消息__（`HunterStatus`/`HunterActuatorState`/`HunterLightCmd`/`HunterLightState`/`HunterRCState`）复制进自定义 `hunter_common/hunter_msgs/msg/`；
- 更新自定义 `hunter_common/hunter_msgs/CMakeLists.txt` 的 `msg_files`，登记这 5 条消息（现共 __13 条__）；
- 删除重复包 `src/hunter_ros2/hunter_msgs`。

## V0.0.23

Date：2026-8-27
版本主要有以下功能：
1. 修复 `src/hunter_common/hunter_msgs/CMakeLists.txt`__（补充依赖查找）：

```cmake
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)         # 新增
find_package(geometry_msgs REQUIRED)    # 新增
```
2. 排除重复包 `src/hunter_ros2/hunter_msgs`__：创建了 `COLCON_IGNORE` 文件，让 colcon 跳过该包（非破坏、可随时删除文件恢复；其消息已并入自定义包，`hunter_base` 等通过 `find_package(hunter_msgs)` 会自动解析到合并后的自定义包）。

## V0.0.24

Date：2026-8-28
版本主要有以下功能：
1. 修复ota_agent编译问题，setup.py 文件异常导致无法编译。

## V0.0.25

Date：2026-8-29
版本主要有以下功能：
1. Nav2 编译跳过
`import_vendor.sh` 中 navigation2 部分改&#x4E3A;__&#x68C0;测 apt 是否已装__
2. yolo_trt_ros 重复包名去重 
在 clone `yolo_trt_ros` 后新增处理：__仅保留 `src/jetson`，其余同名包 `package.xml` 重命名为 `.disabled`__（移出 colcon 扫描）

## V0.0.26

Date：2026-8-29
版本主要有以下功能：
1. `sensor_fusion/CMakeLists.txt`
补充 `find_package` 与 `ament_target_dependencies`
2. `sensor_fusion/package.xml`
补充依赖：
```xml
<depend>tf2</depend>
<depend>tf2_ros</depend>
<depend>tf2_geometry_msgs</depend>
```
3. `sensor_fusion/src/sensor_fusion.cpp`
补充头文件（保证 `tf2::doTransform` 对 `geometry_msgs::PoseStamped` 可用）
4. `lidar_perception/package.xml`（同类隐患预防）
其 CMakeLists 已用 `tf2_geometry_msgs` 但 package.xml 未声明，已补充：
```xml
<depend>tf2_geometry_msgs</depend>
```
5. `README.md` §5.2
补充 `data_agent` 所需的系统库依赖（Kafka + SQLite）

## V0.0.27

Date：2026-8-29
版本主要有以下功能：
1. 视觉感知模块只发布 vision_objects，无 freespace：
新增 `freespace_pub_` + `buildFreespace()`，colorCallback 发布；话题用可配置的 `/perception/vision_freespace`（避免与 sensor_fusion 的 `/perception/freespace` 争用
2. 视觉感知模块预处理未用 OpenCV CUDA：
改为 `cv::cuda::GpuMat` + `cv::cuda::resize` + `cv::cuda::cvtColor`
3. 视觉感知模块依赖缺 `nav_msgs`：
CMakeLists + package.xml 补充 `<depend>nav_msgs</depend>`

## V0.0.28

Date：2026-8-29
版本主要有以下功能：
代码 Review 校验修复：
1. `data_agent` 缺 `/chassis/feedback` 订阅：
修复：
- `data_agent.hpp`：新增 `chassisFeedbackCallback`、`chassis_feedback_sub_`、`chassis_feedback_`、`chassis_feedback_received_`；
- `data_agent.cpp`：新增 `/chassis/feedback` 订阅与回调，并在遥测 JSON 补充 `fb_velocity`/`fb_steering`。
2. `sensor_fusion` freespace 未体现"未知 -1"：
修复_：`buildFreespace()` 改为 `grid.data.assign(..., -1)`（未知），再按车前已知范围标记自由 0、障碍物 100。
3. `vision_perception` freespace 话题：
文档 §5.2.1 写视觉输出 `/perception/freespace`，但 §17.1 明确 `/perception/freespace` 由 `sensor_fusion` 发布。为避免两节点争用，视觉侧发布到可配置的 `/perception/vision_freespace`（已在注释说明）。属文档内部矛盾的工程处理，非私自新增话题，予以保留。

## V0.0.29

Date：2026-9-1
版本主要有以下功能：
1.  新增 src/hunter_ros2/hunter_msgs/COLCON_IGNORE（空文件）。当前工作空间里 colcon 会跳过这份 vendor 包。
2.  更新 src/hunter_bringup/scripts/import_vendor.sh：clone / 已存在 hunter_ros2 后都会 touch COLCON_IGNORE，避免下次导入再冲突。

## V0.0.30

Date：2026-9-1
版本主要有以下功能：
1. 恢复 tensorrt_yolo_ros2的package.xml + 修正导入脚本。

## V0.0.31

Date：2026-9-1
版本主要有以下功能：
1. 修复导入第三方依赖包脚本错误。

## V0.0.32

Date：2026-9-1
版本主要有以下功能：
1. 在HunterEdge\src\hunter_bringup\scripts\import_vendor.sh中，执行完git clone https://github.com/Livox-SDK/livox_ros_driver2.git后，对livox_ros_driver2文件夹创建一个package.xml文件，直接把package_ROS2.xml的内容复制到package.xml中。

## V0.0.33

Date：2026-9-1
版本主要有以下功能：
1. clone livox_ros_driver2 后立即 touch COLCON_IGNORE，colcon 扫描时直接跳过该包（已有的存量目录重跑脚本也会补上这个文件）。
2. fast_lio2/CMakeLists.txt
：移除 find_package(livox_ros_driver2 REQUIRED) 和 dependencies 列表中的 livox_ros_driver2。
3. fast_lio2/package.xml
：移除 <depend>livox_ros_driver2</depend>。

## V0.0.34

Date：2026-9-1
版本主要有以下功能：
1. 执行 colcon build --symlink-install时，跳过构建 livox_ros_driver2。

## V0.0.35

Date：2026-9-1
版本主要有以下功能：
1. 修改import_vendor.sh，在clone完fast_lio后，把子模块ikd-Tree也递归拉取。

## V0.0.36

Date：2026-9-2
版本主要有以下功能：
1. 修复构建fast_lio2依赖的问题。

## V0.0.37

Date：2026-9-2
版本主要有以下功能：
1. 修复vision_perception 依赖 TensorRT（NvInfer.h），而 TensorRT 依赖 CUDA（cuda_runtime_api.h）。当前机器上 CUDA 已安装（TensorRT 头文件存在于 /usr/include/aarch64-linux-gnu/），但 CUDA 的头文件路径没有被 CMake 找到。在 target_include_directories 里补上 ${CUDA_INCLUDE_DIRS}。

## V0.0.38

Date：2026-9-2
版本主要有以下功能：
1. 把 import_vendor.sh 里的 Python 脚本处理块去掉，rosidl_generate_interfaces（消息接口生成）在守卫之外，无论 SDK 是否存在都会执行，fast_lio2 需要的 custom_msg.hpp 照常生成。

## V0.0.39

Date：2026-9-2
版本主要有以下功能：
1. 修改import_vendor.sh，把需要在目标机器上执行的改掉两个 REQUIRED和加上守卫块（if + endif）放在import_vendor.sh 里clone livox_ros_driver2后执行，免去每次 clone后都需要手动跑一次。

## V0.0.40

Date：2026-9-2
版本主要有以下功能：
1. health_monitor.cpp的RCLCPP_WARN_THROTTLE(logger, clock, ms, ...) 的第二个参数必须是 rclcpp::Clock &，而代码传入的是 rclcpp::Time（时间点，没有 .now() 方法）。
位置	|修改内容
checkTopicFrequency 签名	|新增 rclcpp::Clock & clock 参数
checkTopicFrequency 内 RCLCPP_WARN_THROTTLE	|now → clock
checkHealth 内 4 处调用	|get_logger() → get_logger(), *get_clock()
checkHealth 内 RCLCPP_WARN_THROTTLE	|now → *get_clock()

## V0.0.41

Date：2026-9-3
版本主要有以下功能：
1. vision_perception 的CMakeLists.txt 中的检测逻辑需要补上 OpenCV_INCLUDE_DIRS 的显式包含，并且把 CUDA 模块的 include 路径也加进去。

## V0.0.42

Date：2026-9-3
版本主要有以下功能：
1. lidar_perception.cpp修改，解决构建错误：
-msg->header.stamp 和 imu_queue_.front().stamp 都是 builtin_interfaces::msg::Time，不支持直接相减。加上 rclcpp::Time(...) 包装后，ROS2 的时间差操作符就能正常工作了。
-第239行：pcl_ros::transformPointCloud 的第三个参数接受 TransformStamped（即整个 tf），而不是其中的 .transform 字段（那是 geometry_msgs::msg::Transform 类型，没有对应的重载）。
2. 构建出现TensorRT API 版本不兼容的问题，将所有废弃 API 替换为新版接口，同时在 header 里改用智能指针管理生命周期。

## V0.0.43

Date：2026-9-3
版本主要有以下功能：
1. 修复data_agent构建的错误，data_agent.cpp补充了 key_len（0）、timestamp（0，表示让 broker 自动分配）、msg_opaque（nullptr）三个参数。

## V0.0.44

Date：2026-9-3
版本主要有以下功能：
1. launch 文件里把包名写成了 hunter_ros2，但根据错误日志里的已安装路径，实际包名是 hunter_base（来自 /home/agilex/agilex_ws/install/hunter_base）。
修改内容：get_package_share_directory('hunter_ros2') → get_package_share_directory('hunter_base')。

## V0.0.45

Date：2026-9-3
版本主要有以下功能：
1. 修复launch 文件里 rslidar_sdk launch引用文件错误问题。rslidar_sdk 的实际 launch 文件是 humble_start.py（针对 ROS2 Humble），而不是 rs_lidar.launch.py。

## V0.0.46

Date：2026-9-3
版本主要有以下功能：
1. localization.launch.py：
package='fast_lio2' → package='fast_lio'（package.xml 和 CMakeLists.txt 里项目名均为 fast_lio，安装路径也是 install/fast_lio）
2. hunter_full.launch.py — 参数透传问题：
给所有 IncludeLaunchDescription 加了 launch_arguments={}.items()
这样父级声明的 use_perception/use_navigation/use_data_agent 就不会被自动注入到子 launch 文件中，消除那三条 Warning。

## V0.0.47

Date：2026-9-3
版本主要有以下功能：
1. hunter_se.urdf.xacro — 删除 pi 重定义 新版 xacro 已内置 pi 常量，自定义同名属性会产生 stderr warning。robot_state_publisher 用 Command() 调用 xacro，任何 stderr 输出都会让 launch 框架抛异常。直接删掉那一行，用 xacro 内置的 pi 即可（xacro 文件里原本就在用 ${pi} 表达式，不受影响）。
2. hunter_full.launch.py — 用 GroupAction 真正隔离参数 launch_arguments={}.items() 只是"不额外传参"，不能阻止父级已有的 LaunchConfiguration 值被子 launch 继承。GroupAction 会创建一个独立的作用域，父级 LaunchConfiguration 不会泄漏进去，这才是正确的隔离方式。同时做了小重构，用 _isolated() 辅助函数减少重复。
3. nav2_recoveries 包找不到 — Nav2 Humble 中该包已改名为 nav2_behaviors。

## V0.0.48

Date：2026-9-3
版本主要有以下功能：
1. import_vendor.sh — 在 clone rslidar_sdk 之后新增了配置检查逻辑：
-用 grep -oP 读取当前 lidar_type 值
-若已是 RSHELIOS_16P 则打印跳过提示
-否则用 sed -i 原地替换，并打印修改日志（包括从什么值改成了什么）
-若配置文件不存在也会给出警告而不是静默失败（脚本有 set -euo pipefail，配置文件不存在不该中止整个导入）
2. hunter_base.launch.py — port_name 默认值已是 can_car（上轮已改，这次确认无误）。

## V0.0.49

Date：2026-9-3
版本主要有以下功能：
1. 建图部分：基于FAST‑LIO2实现LiDAR‑IMU SLAM建图模式；支持【建图模式】/【定位导航模式】切换；建图完成支持保存LiDAR地图文件；导航模式加载已保存地图。
2. Nav2自主导航增强：扩展Nav2行为树；支持定点导航、多点巡航waypoint跟随；融合/perception/fused_objects障碍物输出到Nav2代价地图；障碍物触发减速/避让行为；
3. AUTO模式条件保护：只有decision_making输出状态为AUTO，无ESTOP、无REMOTE接管、定位收敛有效、感知数据正常，才允许执行导航任务；不满足条件禁止下发导航目标，输出告警日志。
4. 提供两种使用方式：
   方式A：本地预设航点列表，AUTO模式下自动依次执行waypoint巡航，完成后循环/停车；
   方式B：支持ROS2 action接口，外部可以下发导航目标点；
5. 安全约束：
   - 检测到融合障碍物距离小于安全阈值，执行减速、避让，严重风险触发急停ESTOP；
   - 定位失效、感知全部失效，立即停止导航，安全停车；
   - 遵循系统限速，最大速度由yaml配置，不超过文档规定2.0m/s；
6. 新增/修改文件范围：
   - 新增 launch：hunter_autonomous_nav.launch.py，由hunter_full.launch可选择参数use_autonomous_nav:=true来加载；不破坏原有全系统启动；
   - 扩展行为树xml：autonomous_navigate.xml，增加障碍物避让、定位有效性判断节点；
   - 新增yaml配置：autonomous_nav_params.yaml（航点列表、安全距离、建图保存路径、巡航参数）；
   - 可选：简单的自主任务调度c++节点 auto_mission_node，用于本地航点循环巡航；
7. 话题与交互：
    读取：/system/health、/planning/behavior_state（decision_making输出）、/localization/odom定位、/perception/fused_objects融合障碍物；
    调用Nav2 BT Navigator action接口下发导航目标；
    输出任务状态话题，上报当前巡航任务进度。
8. pcd_save 参数自动注入（fast_lio2_param_injector）
fast_lio2 节点由 localization.launch.py 已经启动，无法直接修改它的启动参数。fast_lio2_param_injector 作为 one-shot 节点，启动后通过 /fast_lio2/set_parameters ROS2 标准服务接口向运行中的 fast_lio2 节点注入参数，注入完成后自动退出。建图模式注入 pcd_save_en=true + 实际路径，导航模式注入 pcd_save_en=false 防止意外写盘。
9. PCD→PGM 自动转换（pcd_to_map）
订阅 /auto_mission/status，检测到 MAPPING → 非MAPPING 的跳变时，等待 3s（确保磁盘写入完成），再等待文件大小稳定，然后自动执行纯 Python 的 PCD 解析 → z 轴切片 → 2D 占据栅格投影 → PGM/YAML 写出。整个流程无需 libpcl 或外部工具。
10. 航点坐标自动采集（waypoint_recorder）
在建图模式下同时启动，持续订阅 rviz2 的 /clicked_point。每次在 rviz2 地图上用 "Publish Point" 工具点击，坐标自动追加写入 autonomous_nav_params.yaml 的 waypoints 字段，节点退出时再保存一次，不丢失任何点击记录。

## V0.0.50

Date：2026-9-4
版本主要有以下功能：
1. 相机实际发布的是 /camera/camera/color/image_raw，但 sensor_params.yaml 里配置的是 /camera/color/image_raw（少一层 camera）。这会导致 vision_perception 等订阅相机话题的节点收不到数据。

## V0.0.51

Date：2026-9-4
版本主要有以下功能：
1. hunter_full.launch.py两处改动：
-_isolated 函数新增了 launch_arguments 可选参数，支持向子 launch 显式传参，其他调用处不受影响
-can_driver 传入 launch_arguments={'port_name': 'can0'}，覆盖 hunter_base 默认的 can_car

## V0.0.52

Date：2026-9-4
版本主要有以下功能：
1. 消除来自 realsense2_camera_node的Warning，在 _isolated 里加 scoped=True：
scoped=True, forwarding=False 会阻止父级参数向子 launch 透传，这正是 _isolated 函数注释里一直说要做到的隔离效果，之前实际上没有生效。

## V0.0.53

Date：2026-9-4
版本主要有以下功能：
1.	controller_server / planner_server SIGABRT：nav2_params.yaml 的 width/height 写成 100.0/10.0，但 Nav2 Humble 将这两个参数声明为 integer 类型，类型不匹配触发 abort	，改为整数 100 / 10。
2.	auto_mission_node SIGABRT（rcl 解析失败）：autonomous_nav_params.yaml 有裸顶层 key mapping: 和 navigation:，rcl 要求所有顶层 key 必须是节点名	，将这两个块移入 auto_mission_node/ros__parameters 下。
3.	ch10x_driver 串口 Permission：denied	agilex 用户不在 dialout 组	，运行时执行：sudo usermod -aG dialout agilex && newgrp dialout。
4.	fast_lio2 SIGSEGV：旧参数文件扁平结构导致 extrinsic_T/R 未读取，为空 vector，VEC_FROM_ARRAY 越界访问，重写为嵌套结构，节点名对齐 fast_lio2，确保外参被正确加载。

## V0.0.54

Date：2026-9-4
版本主要有以下功能：
1.	fast_lio2 Failed to find match for field 'time'/'ring'：	rslidar_sdk 默认编译 XYZI（无 ring/time 字段），且 XYZIRT 模式下字段名为 timestamp 而非 time	，rslidar_sdk CMakeLists.txt 改 POINT_TYPE=XYZIRT；pcl_point_cloud_msg.hpp 把注册别名 timestamp 改为 time；fast_lio2_params.yaml 改 timestamp_unit=0。
2.	/imu/data QoS 不兼容：ch10x_driver 发布用 BEST_EFFORT，fast_lio2 订阅用 RELIABLE（queue=10）	laserMapping.cpp IMU 订阅改为 rclcpp::SensorDataQoS()。
3.	hunter_base_node protocol UNKNOWN：	CAN 总线上没有收到底盘心跳帧（0x151/0x221/0x241）。	硬件问题，代码无需修改；需确认底盘电源已上电并 candump can_car 验证。
4.	map_server 地图文件不存在：_isolated(scoped=True, forwarding=False) 阻断了参数透传，mode 始终是默认值 nav，建图模式也启动了 map_server。	hunter_full.launch.py 的 autonomous_nav 调用里显式传入 mode/map_yaml_path/map_file_path。

## V0.0.55

Date：2026-9-4
版本主要有以下功能：
1. 把 launch_arguments 的值从 LaunchConfiguration(...) 对象改成 [LaunchConfiguration(...)]（包在列表里），这是 ROS2 launch substitution 链的正确传法。

## V0.0.56

Date：2026-9-4
版本主要有以下功能：
1. GroupAction(forwarding=False) + launch_arguments 里传 LaunchConfiguration 是一个死锁：它要用父作用域的值，但父作用域已经被清空。不能用 LaunchConfiguration 对象做值，要用 perform 时机更晚的方式，或者直接用 OpaqueFunction 在运行时解析值再构建 action。最简单可靠的方案是把 autonomous_nav 从 _isolated 里抽出来，改用 OpaqueFunction 在运行时读取父作用域的值再构建带参数的 IncludeLaunchDescription。

## V0.0.57

Date：2026-9-4
版本主要有以下功能：
1. 把关联到系统的根目录hunter和hunter_ws改为HunterEdge。

## V0.0.58

Date：2026-9-4
版本主要有以下功能：
1. 在代码或 config 配置文件中将 can0 改为 can_car。

## V0.0.59

Date：2026-9-4
版本主要有以下功能：
1. 底盘状态缺失 → health_monitor 误急停 / decision_making 永远 ESTOP：
`src/hunter_ros2/hunter_base/include/hunter_base/hunter_messenger.hpp`
- 新增 `/chassis/state`、`/chassis/feedback` 两个 `hunter_msgs::ChassisState` 发布者（`rclcpp::SensorDataQoS`，与 `health_monitor/decision_making/data_agent` 的 BEST_EFFORT 订阅匹配）。
- 在 `PublishStateToROS()`（50Hz 驱动回调）里用 ugv_sdk 状态填充 velocity/steering/battery_voltage、`control_mode`(CAN/REMOTE/STANDBY)、`vehicle_state`(NORMAL/ESTOP/FAULT)、`fault_code`、电机温度并发布。
- 效果：CAN 正常时 `/chassis/state` 不再恒 0Hz → `health_monitor` 不再反复发 `/estop=true`，`auto_mission` 可进入 MAPPING；`decision_making` 的 `chassis_stale` 不再把模式锁在 ESTOP。
2. FAST-LIO2 无有效点云输入（缺 ring/timestamp）

- `src/rslidar_sdk/config/config.yaml`

  - `use_lidar_clock: true → false`：点云头时间改用主机时钟，与 `/imu/data`（CH10X 主机时钟）同域，避免 “IMU and LiDAR not Synced”。
  - `ros_send_point_cloud_topic: /rslidar_points → /lidar_points`：与 fast_lio2/health_monitor/lidar_perception 消费端统一。
  - 注释明确要求：`/lidar_points` 必须&#x7531;__&#x7F16;译期 `POINT_TYPE=XYZIRT`__ 输出 `ring + timestamp`（工作区 `rslidar_sdk/CMakeLists.txt` 已把 `XYZI` 改为 `XYZIRT`，属未提交改动，需重编）。

- `src/fast_lio2/src/preprocess.h`
  - `velodyne_ros::Point::time` 由 `float` 改为 `double`，注册宏同步改为 `(double, time, timestamp)`：rslidar_sdk 的 `timestamp` 字段是 __FLOAT64__，与成员类型严格对齐，避免 pcl 反序列化时 4/8 字节错位。
3. IMU 链路（QoS + 可靠性）

- `src/hunter_drivers/ch10x_driver/src/ch10x_driver.cpp`

  - `/imu/data` 发布 QoS 由 `SensorDataQoS()`(深度5) 改为 `SensorDataQoS(KeepLast(500))`，仍是 BEST_EFFORT，减少 100Hz IMU 丢帧。
  - 日志里出现的 `RELIABILITY` 不兼容警告说明车上跑的&#x662F;__&#x65E7;构建/未重编产物__（当前源码中 ch10x/fast_lio2/health_monitor/robot_localization 的 `/imu/data` 端均已 BEST_EFFORT）。此项必须在车上重编后再次验证；同时确认 CH10X 串口真在出数。
4. PCD 存不下来 / 注入超时

- `src/fast_lio2/src/laserMapping.cpp`

  - 解除 `publish_frame_world()` 里被整段注释的 `pcl_wait_save` 地图累积逻辑（否则退出/服务写出的 PCD 恒为空）。
  - `save_to_pcd()` 改为写 `pcl_wait_save` 到 `map_file_path`（原来写的是空的 `pcl_wait_pub`）。
  - Ctrl+C 退出时优先写 `map_file_path`（不再硬编码 `ROOT_DIR/PCD/scans.pcd`），并新增 `ensure_parent_dir()`（`mkdir -p`）。

- `src/auto_mission/scripts/fast_lio2_param_injector.py`

  - `map_file_path` 做 `os.path.expanduser`（`:=~/…` 在 bash 里不会展开）。
  - client 只创建一次（原实现每次重试都 `create_client` 同一服务，属隐患）。
  - 服务调用超时 3s→5s，超时后自动重试并保留节点，全失败才退出。

- `src/auto_mission/scripts/pcd_to_map.py`

  - `pcd_file/map_output_dir` 做 `os.path.expanduser`。
  - 检测到 MAPPING→非MAPPING 时，先调用 `/fast_lio2/map_save` 服务触发写盘，成功后再自动 PCD→PGM+YAML；15s 无响应自动回退原“等文件”逻辑。

- `src/hunter_agents/data_agent/src/data_agent.cpp`（P1 顺带修复）
  - `sqliteInit()` 先 `std::filesystem::create_directories()` 建 `/data/data_agent`，消除 `SQLite 打开失败: unable to open database file`。
5. `src/hunter_drivers/ch10x_driver`：
__`include/ch10x_driver/ch10x_driver.hpp`__
- 帧常量改为真实协议：帧头 `5A A5`，长度字段（小端），CRC16 占 2 字节，`0x91` 数据域 76 字节（去掉原来虚构的 `AA 55 | TYPE | ...`）。
__`src/ch10x_driver.cpp`__
- 新增 `crc16XmodemUpdate()`：CRC-16/XMODEM（poly `0x1021`，初值 0，无反射无异或），先算 `buf[0..4)`，再接着算 `buf[6..6+LEN)`，与帧内 `buf[4]|buf[5]<<8` 比对（用你手册样例 + 车机抓帧 3 帧全部验证通过）。
- `parseFrame()` 重写：
  1. `5A A5` 帧头同步；
  2. 读小端长度并做上限保护；
  3. CRC16 校验（失败丢 1 字节重同步）；
  4. 只处理 `tag=0x91` 的 76 字节数据包；
  5. 按厂商布局解析 `timestamp(ms)`、`acc[3](G)`、`gyr[3](°/s)`、`mag[3](µT)`、`eul[3](°)`、`quat[4](w,x,y,z)`；
  6. 单位换算到 ROS 约定：__角速度 °/s → rad/s__、__加速度 G → m/s²__、欧拉角 ° → rad。
  
## V0.0.60

Date：2026-9-4
版本主要有以下功能：
1. 配置文件里模型文件名是 yolov8s.onnx。

## V0.0.61

Date：2026-9-4
版本主要有以下功能：
1. vision_perception_params.yaml：	color_topic / depth_topic 加 /camera/ 前缀。
2. vision_perception.cpp：	declare_parameter 默认值同步对齐。
3. health_monitor.cpp：	相机话题订阅路径修正。
4. data_agent.cpp	rosbag： 录制命令中的话题路径修正。

## V0.0.62

Date：2026-9-4
版本主要有以下功能：
1. waypoint_recorder 双重 shutdown：waypoint_recorder.py加 rclpy.ok() 判断。
2. pcd_to_map 双重 shutdown：pcd_to_map.py加 rclpy.ok() 判断。
3. realsense 关闭超时 5s：rs_launch.py加 sigterm_timeout=3。

## V0.0.63

Date：2026-9-4
版本主要有以下功能：
1. vision_perception.cpp：跳过空图像、非法尺寸和错误类型深度图。
2. tensorrt_inference.cpp：推理前校验输入，过滤 NaN、无效宽高检测框。
3. pcd_to_map.py：防止 map_save 迟到回调与兜底定时器重复启动转换。
4. fast_lio2_param_injector.py：超时后不再取消并重复提交可能已在服务端执行的请求，改为等待原请求结果。

## V0.0.64

Date：2026-9-5
版本主要有以下功能：
1. vision_perception 崩溃修复（`vision_perception.cpp/.hpp`）：
- 两个图像回调在转换前校验消息自洽性（`width/height > 0`、`step ≥ width×元素字节数`、`data.size() ≥ step×height`），坏帧直接丢弃并带完整元数据打 WARN——若再次出现即可精确定位来源；
- 颜色帧主流程抽出为 `processColorFrame`，回调外层捕获 `cv::Exception`/`std::exception`：任何异常只丢当前帧，节点绝不 abort。
2. data_agent Kafka连接失败修复（`data_agent.cpp/.hpp` + yaml）：
- 注册 librdkafka 投递报告回调 `dr_cb`：只有它能证实消息真正送达；只有“投递已证实可用且本地队列未积压（`kafka_queue_limit` 默认 50 条）”才直发 Kafka，否则一律写 SQLite 缓存；
- `kafkaReconnect()` 不再盲目置位连接状态（此前它会立刻把断连状态改回 true，掩盖真实链路）；
- 新增 `message.timeout.ms=60s`（积压消息不再无限滞留）、`reconnect.backoff.max.ms=10s`；
- 退出时按 `kafka_flush_timeout_ms`（默认 5s）flush，仍有残留&#x5219;__&#x5982;实告警条数；
- ⚠️ `kafka_brokers` 仍是占位域名 `platform.example.com:9093`，部署时必须改为真实 broker 地址（yaml 已加醒目注释）。
3. health_monitor 误判重启修复（`health_monitor.cpp/.hpp` + yaml）：
- 新增 `startup_grace_period`（默认 15s）：宽限期内 lidar/camera/imu 不判异常，`checkNodes` 不告警不计数；
- 引入 `node_ever_alive_` 状态机：__从未在线 → 首次上线只打 INFO 不计重启__；只有“曾真实在线 → 掉线 → 恢复”才计重启；
- 宽限期结束后仍未上线的节点会得到明确的“疑似未启动”告警（而非崩溃告警）；
- CAN 是安全链路，不享受宽限期——CAN 断连仍然立即告警并触发安全停车（安全语义未削弱）。

## V0.0.64

Date：2026-9-5
版本主要有以下功能：
1. 帧名统一方向反转为 `rslidar`。
2. vision_perception `setSize` 断言修复（`vision_perception.cpp` 两处回调）：增加 `width/height ≤ 8192` 上限 + 乘法改 `uint64_t`，彻底堵住回绕。修复后损坏帧会被正确拦截并打出“彩色图像消息无效”。
3. 系统的所有配置can_car改成can0。

## V0.0.65

Date：2026-9-5
版本主要有以下功能：
1. hunter_can_test.sh 的 up 分支改为仅当接口不存在时才尝试 `modprobe gs_usb`：现场确认本车 CAN 为 Jetson 板载 mttcan 控制器（c310000.mttcan，内核自带），无需加载模块；README §13.5 同步说明（UP 状态下改波特率报 Device or resource busy 属正常现象，建议 `ip link set can0 type can restart-ms 100`）。现场 can0 已 UP @500Kbps、ERROR-ACTIVE、berr 计数 0，接口侧就绪；candump 无帧，剩余问题在底盘侧（电源/急停开关、CAN_H/L 接线、120Ω 终端电阻）。

## V0.0.66

Date：2026-9-5
版本主要有以下功能：
1. 最终确认接口为 can2：底盘经 USB-CAN 适配器接入 AGX Xavier，适配器枚举为 can2（candump 已见 0x211/0x221/0x231/0x241/0x251 等成套反馈帧，总线健康）；can0 为 Jetson 板载 mttcan、未接线束，candump 静默属正常。全系统配置由 can0 统一改为 can2：hunter_base.launch.py 默认 port_name、hunter_full.launch.py 显式传参、hunter_base_ros.cpp 节点内默认值、hunter_can_test.sh 默认 CAN_IF、ota_agent 自检、README/User_Manual/.ai-rules 文档；README §13.5 补充拓扑说明。hunter_base 有 C++ 改动，需重编 hunter_base/hunter_bringup/ota_agent 后重新 bring-up。

## V0.0.67

Date：2026-9-5
版本主要有以下功能：
1. vision_perception `setSize` 断言根因确认与修复（`vision_perception/CMakeLists.txt`）：
- 根因排除消息损坏：V0.0.64 校验器已确认编入二进制（`grep -ac`/`step=%u` 双指纹），重启后线上无任何“彩色图像消息无效”日志而异常依旧每帧必现（5s 节流）——线上 640x480 消息本身是好的，负尺寸不可能来自消息字段，“RealSense 坏帧回绕”假说作废；
- 决定性线索来自异常文本：`OpenCV(4.10.0) /home/agilex/opencv_build/... matrix.cpp:246 s >= 0 in setSize`——/usr/local 实为源码编译的 OpenCV 4.10.0（CUDA）而非此前误读的 4.1.0（soname 规则：410=4.10.0，4.5d=4.5.4）；
- 真根因：CMake `find_package(OpenCV)` 默认选中 /usr/local 的 4.10.0，节点按 4.10 编译链接；系统 cv_bridge 却按 OpenCV 4.5.4 构建。同一进程两套 OpenCV 符号交叉绑定（ldd 可见 410 与 4.5d 并存 + "may conflict" 警告），破坏 cv::Mat 不变量，每帧必现断言。已复核整条视觉链路（toCvCopy → GpuMat/CPU 预处理 → convertTo → 后处理/深度投影），无任何以计算尺寸创建 Mat 的路径，进一步排除代码侧负尺寸来源；
- 修复：vision_perception 强制使用系统 OpenCV（`OpenCV_DIR=/usr/lib/aarch64-linux-gnu/cmake/opencv4`，路径存在才设置），进程内只保留与 cv_bridge 同源的一套 OpenCV；GPU 预处理分支（HAVE_OPENCV_CUDA）自动关闭改走 CPU，TensorRT 推理仍在 GPU 不受影响。恢复 GPU 预处理的前置条件：cv_bridge 针对 /usr/local 4.10 重新编译，保证全进程单一版本；
- 部署：`rm -rf build/vision_perception install/vision_perception && colcon build --packages-select vision_perception`（symlink-install 工作区必须先删净再编），重启 bring-up 后验证；若 WARN 仍出现，用 gdb 挂节点抓 `cv::error` 完整 backtrace 定位调用方。

## V0.0.68

Date：2026-9-5
版本主要有以下功能：
1. 方向变更：放弃 V0.0.67 的"降级到系统 OpenCV 4.5.4"方案，vision_perception 全程使用 /usr/local OpenCV 4.10.0（4.5.4 无 CUDA，GPU 预处理恢复）。硬约束不变：进程内只允许一套 OpenCV（混链 = matrix.cpp:246 setSize 断言根因）。
2. 实现——vision_perception 解除对 cv_bridge 的依赖（`vision_perception.cpp` + CMakeLists.txt + package.xml）：
- 系统 cv_bridge 是预编译库（NEED libopencv_*.so.4.5d），无法重定向到 4.10，只要它还在链接里，410/4.5d 混链就无解。波及面评估：realsense2_camera 主节点自身不链 cv_bridge（cv_bridge 仅被 workspace 内 realsense2_rgbd_plugin（RViz 插件，不在 bring-up 进程）与未参与 bring-up 的 yolo_trt_ros/tensorrt_yolo_core 使用），解除依赖不影响其他节点；yolo_trt_ros 若未来启用，需按同思路解耦或 shadow 重编 cv_bridge；
- `vision_perception.cpp` 匿名命名空间新增 `imageMsgToMat(msg, desired)`：与 toCvCopy 语义等价（深拷贝、输出连续存储、step 行距正确；"bgr8" 请求：rgb8 源走 cv::cvtColor COLOR_RGB2BGR、bgr8 源 clone；"16UC1" 请求：16UC1/mono16 源 clone；其余组合返回空 Mat 走既有告警路径），全部符号落在 4.10 内；
- 两处调用点替换：depthCallback `toCvCopy(msg, TYPE_16UC1)` → `imageMsgToMat(msg, "16UC1")`（转换失败保留上一帧有效深度，不误置 depth_received_）；processColorFrame `toCvCopy(msg, "bgr8")` → `imageMsgToMat(msg, "bgr8")`；`catch (cv_bridge::Exception)` 相应移除（cv::Exception 兜底保留）；
- CMakeLists：删除 `find_package(cv_bridge REQUIRED)` 及 ament_target_dependencies 中的 cv_bridge；OpenCV_DIR 强制 `/usr/local/lib/cmake/opencv4`（EXISTS 守卫 + CACHE 写法——V0.0.67 已验证普通 set() 会被 find_package 无视）；CUDA 检测块不变，configure 应输出 `OpenCV CUDA modules found — GPU preprocessing enabled`；
- package.xml 移除 `<depend>cv_bridge</depend>`。
3. 部署与验收（预期指纹，与 V0.0.67 相反方向）：
- `rm -rf build/vision_perception install/vision_perception && colcon build --packages-select vision_perception`（务必删净再编）；
- configure 输出应为 `vision_perception: OpenCV 4.10.0 选自 /usr/local/lib/cmake/opencv4`；
- 编译 stderr 不得再出现任何 `may conflict with libopencv_*.so` 警告（cv_bridge 已不在链接里）；
- `ldd build/vision_perception/vision_perception_node | grep opencv` 只见 410、无 libcv_bridge；
- 重启 bring-up：setSize WARN 应消失；若异常依旧且链接干净（纯 4.10），则根因另有所在，用 gdb 挂节点抓 `cv::error` backtrace。

## V0.0.69

Date：2026-9-5
版本主要有以下功能：
1. 现场结果（robot，2026-09-05）与第二轮修复（GPU 预处理降级兜底）：
- 验收指纹全部命中：configure `OpenCV 4.10.0 选自 /usr/local/lib/cmake/opencv4` + `GPU preprocessing enabled`；`ldd | grep opencv` 全部 so.410（含 cudaimgproc/cudawarping/cudafilters/cudaarithm）、零 4.5d、无 libcv_bridge——**setSize WARN 彻底消失，混链根因修复确认 ✅**；
- 暴露第二层根因（环境层）：`resize.cu:175 error (-217) no kernel image is available for execution on the device` = /home/agilex/opencv_build 编译 4.10.0 时 CUDA_ARCH_BIN 与本机 GPU 不匹配——**设备实为 AGX Orin（计算能力 8.7；JetPack 6/CUDA 12.6/内核 5.15.148-tegra，`/proc/device-tree/model` 确认）**，而旧库 `cuobjdump --list-elf` 实锤只编了 sm_72.cubin（当时照抄了 Xavier 参数）；GPU 分支抛异常→colorCallback 外层 catch 整帧丢弃→/perception/vision_objects 0Hz 停摆（每 5s 节流 WARN）；
- 代码兜底（tensorrt_inference.cpp/hpp）：infer() 预处理 GPU 分支包 try-catch，`cv::Exception` 时置 `cuda_preprocess_degraded_`（hpp 新成员）并**永久降级 CPU 路径**，仅 `[TensorRT]` stderr 提示一次；未定义 HAVE_OPENCV_CUDA 时行为不变；感知输出恢复不断流——重编 OpenCV 后无需改代码，重启节点即自动恢复 GPU（标志为进程内）；
- 环境正解（robot 上执行，已实测定案）：① `cat /proc/device-tree/model` → NVIDIA Jetson AGX Orin Developer Kit → ARCH=8.7（判据：Orin=8.7 / Xavier=7.2，勿照抄教程；内核 5.15.148-tegra + CUDA 12.6 亦为 JetPack 6/Orin 特征）；② 修复 build 目录 root 文件混入（此前 sudo 操作残留，configure_file Permission denied）：`sudo chown -R agilex:agilex ~/opencv_build`；③ 旧 build 目录内 `cmake -DCUDA_ARCH_BIN=8.7 -DCUDA_ARCH_PTX=8.7 .`（摘要确认 `NVIDIA GPU arch: 87`）→ `make -j12 && sudo make install`（Orin 全量 CUDA 重编约 1-2h）；④ 编后验证 `cuobjdump --list-elf /usr/local/lib/libopencv_cudawarping.so.410` 应为 sm_87；⑤ 仅重启 vision_perception；
- 环境修复已完成（robot，2026-09-05）：AGX Orin 上 `cmake -DCUDA_ARCH_BIN=8.7 -DCUDA_ARCH_PTX=8.7` 重编 + install 成功；`cuobjdump --list-elf libopencv_cudawarping.so.410` → sm_87.cubin（旧为 sm_72），`python3 cv2.__version__` = 4.10.0；trtexec 侧双重确认 `Device 0: "Orin"`、`Compute Capability: 8.7`、TensorRT 10.3.0；
- 回归指纹（已验证）：重启 bring-up 后节点日志零 OpenCV 异常、零"永久降级 CPU"提示、零跨设备引擎警告——GPU 预处理路径生效；yolov8s.engine 已在本机重生成。

## V0.0.70

Date：2026-9-5
版本主要有以下功能：
1. 现场问题三：sensor_fusion TF 树分裂（视觉复活后暴露的存量问题，非回归）：
- 现象：`视觉目标 TF 转换失败: base_link ↔ camera_color_optical_frame not part of the same tree, two or more unconnected trees`（每 2s 持续）；
- 根因：URDF（hunter_se.urdf.xacro，文档 7.4）已定义 fixed `base_link→camera_color_optical_frame`（robot_state_publisher 发布，xyz=0.40/0/0.30, rpy=0/-π/2/π/2），而 realsense2_camera 驱动默认 `publish_tf=true` 自建 `camera_link→camera_color_frame→camera_color_optical_frame` 树——同一 frame 双父，TF 树分裂为 base_link 系与 camera_link 系两棵，lookupTransform 必败；此前视觉长期 0Hz、sensor_fusion 从未走到 TF 分支，故从未暴露——视觉链路修复后才现形；
- 修复：hunter_full.launch.py 的 camera_driver 增加 `launch_arguments={'publish_tf': 'false'}`——驱动 TF 树无任何消费者（相机外参唯一来源是 URDF 静态 TF，文档 7.4 不含 camera_link；align_depth 在驱动内部完成不依赖 ROS TF），关闭即修复且符合"tf 树不允许改动"约束；副作用仅 RViz 失去 realsense 原生 TF 视角（调试时可单独补）；
- 附带定性：`彩色图像超时 1.30s` 为启动竞态一次性告警（节点 127.2s 激活 vs 彩色流 127.7s 就绪），仅出现一次无重现，暂不处理；
- 部署：`colcon build --packages-select hunter_bringup`（launch 装进 install，需重编）+ 重启 bring-up；验证：`ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame` 输出 translation (0.40, 0.00, 0.30)、sensor_fusion TF WARN 清零、融合输出恢复；
- 本轮状态基线：vision_perception 全绿（零 OpenCV 异常、零降级提示、零跨设备引擎警告），GPU 预处理已生效；TF 分裂 WARN 由本版本修复处置。
- 回归指纹：节点日志出现一次"永久降级 CPU"后 vision_objects 恢复 ~15Hz；OpenCV 重编完成并重启后，降级提示不再出现、WARN 清零，可用 `nvidia-smi` 观察 GPU 占用上升佐证 GPU 路径恢复。


## V0.0.71

Date：2026-9-6
版本主要有以下功能：
1. 建图模式自动巡航（mapping auto cruise）：建图不再必须遥控驾驶，车辆可沿 rviz 点击的航点自动巡航采集点云：
   - auto_mission_node 新增 20Hz 巡航控制器：mapping 模式下直接发布 `/cmd_vel`（此时 Nav2 未启动、无话题竞争），反馈源为 FAST-LIO2 `/Odometry`（camera_init 系，与 rviz "Publish Point" 点击航点严格同源，规避 EKF odom 系折算偏差）；控制律为朝向目标的 P 控制（大偏差降速 + 接近减速），并满足 HUNTER-SE 阿克曼几何约束 `|w| ≤ v/1.9m`（最小转弯半径，无法原地转向）；
   - 新增服务：`/auto_mission/start_mapping_cruise`（热重载 yaml 最新航点 + 校验定位/模式/急停后启动）、`/auto_mission/stop_mapping_cruise`（停车，建图继续）；巡航参数 `cruise_*` 集中在 autonomous_nav_params.yaml（默认直行 ≤1.0 m/s，建图质量优先）；
   - 安全约束与 NAVIGATING 一致：障碍物 <2m 停车避让（30s 未清除转 ESTOP）、<0.8m 直接 ESTOP；`/Odometry` 超时（1s）/融合感知超时 → 停车等待；`/estop` → 终止巡航切 ESTOP（急停=终止建图，语义一致，pcd_to_map 随之触发转换）；
   - 状态机调整（兼容性关键修复）：mapping 分支提前到非 AUTO/CRITICAL 检查之前——遥控接管（切 REMOTE）或 health CRITICAL 时仅暂停巡航（停车、状态保持 MAPPING），不再把状态切到 IDLE。修复了旧逻辑下"遥控接管会被 pcd_to_map 误判为 MAPPING→非MAPPING 建图结束、提前触发地图转换导致产出不完整 PGM"的隐患；切回 AUTO 后需重新调用 start 服务恢复巡航；
   - launch：auto_mission_mapping 节点传入 params_file（与 waypoint_recorder 写入同一文件），支撑航点热重载；waypoint_recorder 新增点击坐标系一致性检查（rviz2 Fixed Frame 必须为 camera_init，否则告警）；
   - User_Manual §4.5.1 新增"方式B：自动巡航采集"操作流程（航点布置建议间距 ≥3m、转向温和）。
2. 部署：`colcon build --packages-select auto_mission hunter_bringup`（C++ 与 launch/config 均有改动，必须重编）+ 重启 bring-up；验证：mapping 模式下 `ros2 service list | grep cruise` 可见两个服务，点击航点后调 start 服务，观察 `/cmd_vel` 输出与车辆运动、`/auto_mission/status` 保持 MAPPING。

## V0.0.72

Date：2026-9-7
版本主要有以下功能：
1. 修复auto_mission构建错误问题。

## V0.0.73

Date：2026-9-7
版本主要有以下功能：
1. 新增《Deployment_Guide.md》部署操作文档（V1.0，对应软件基线 V0.0.72），面向首次部署工程师与现场运维人员，覆盖全流程六大部分：
   - 系统概述与部署总览（自定义/第三方包清单、数据链路、部署流程图）；
   - 环境要求（硬件清单、软件版本矩阵含 OpenCV 4.10 CUDA/sm_87 与 TensorRT 10.3、网络接口、性能约束）；
   - 环境配置（Jetson 功耗、目录规划、can2@500Kbps 配置与"重启不保留"警示、dialout/video 权限、DDS/ROS_DOMAIN_ID）；
   - 环境安装（ROS2 Humble、apt 二进制依赖、OpenCV 4.10.0 CUDA 源码编译五步法、import_vendor.sh + ugv_sdk/hunter_ros2/rslidar_sdk 源码安装、trtexec 目标机引擎生成）；
   - 详细部署过程（rosdep、全量编译、vision_perception 专项清净重编 + 指纹/ldd 验收、编译报错对照表含 std_srvs 案例、参数核对清单〔kafka_brokers 占位/waypoints 占位等必改项〕、systemd 服务、部署验收 8 项）；
   - 功能操作步骤（上电自检、启动/停止与参数开关、建图方式A 遥控采集、方式B 自动巡航采集〔start/stop_mapping_cruise、安全行为表、首次测试建议〕、地图产物与手动转换、航点管理、导航巡航 AUTO 五条件、外部目标点、Bag/data_agent、远程操控、OTA、运维脚本）；
   - 异常问题解决方法（按编译部署/硬件驱动/感知定位TF/自主导航建图/Agent云端/系统六类故障表 + 降级策略参考，全部收编 V0.0.60~72 现场教训：can2 接口、Device busy、TF 分裂勿改 publish_tf、PCD 退出瞬间落盘、REMOTE 接管保持 MAPPING、Kafka 占位域名等）；
   - 附录（话题/服务/Action 速查、配置文件路径、运维脚本、首次部署检查单、安全要点摘录）。
2. README §14 文档索引同步收录 Deployment_Guide.md。

## V0.0.74

Date：2026-9-7
版本主要有以下功能：
1. 修复 FAST-LIO2 `/Odometry` 零输出——实机建图链路自集成以来从未出过里程计的静默死锁（mapping 自动巡航的最大 blocker）：
   - 现象指纹（判据价值最高）：`/Odometry` 无输出，但发布者数=1、`/fast_lio2` 存活、`/lidar_points` 9.99Hz 正常、QoS RELIABLE→BEST_EFFORT 兼容、fast_lio2 零告警零日志——传输/进程/QoS 全部健康的"假阳性组合"；
   - 根因（字段语义错位，非字段错位）：rslidar_sdk 点内 `timestamp` 字段为绝对秒（~1.78e9，与 `header.stamp` 同量纲），而 fast_lio2 `velodyne_handler` 按 Velodyne 惯例把"末点 time>0"判定为帧内相对偏移（preprocess.cpp:317）→ `curvature = time×1000` 达 ~1e12 ms → `lidar_end_time = beg_time + curvature/1000` 超前 IMU 数十年（laserMapping.cpp:407）→ `sync_packages` 的 `last_timestamp_imu < lidar_end_time` 永久成立、永久 return false 且该路径零日志（:416）→ 点云在 lidar_buffer 无限堆积、IMU 正常入缓冲并完成初始化，节点一切"正常"但永不处理任何一帧；
   - 修复（preprocess.cpp `velodyne_handler` 共 3 处）：`given_offset_time` 判定后取帧内最早点为基准 `base_time`，feature/非 feature 两分支的 `curvature` 均改为 `(time − base_time) × time_unit_scale`——rslidar（绝对秒）→ 0~100ms 帧内偏移；真实 Velodyne（本就相对偏移、首点≈0）→ base≈0 行为不变，双向兼容；
   - 现场验证（robot，2026-9-7）：`/Odometry` 9.99Hz 稳定（std 0.006s）、`tf2_echo camera_init→body` 正常出值；成功标志日志 = `IMU Initial Done` **且** `Initialize the map kdtree`（死锁时仅有前者）；启动头两条 `No point, skip this scan!` 为点云晚于节点就绪的良性竞态；
   - 排障过程沉淀三条工程教训：
     a) **colcon 包名陷阱**：包名 `fast_lio`（package.xml）≠ 源码目录名 `fast_lio2`，`colcon build --packages-select fast_lio2` 报 `ignoring unknown package` 且其余包照常编译，极易漏看当成功——"ignoring unknown package" 必须当错误处理；
     b) **重编必须重启**：运行中进程不加载新二进制；增量编译秒级完成也可能只是无改动空跑，以 install 二进制 mtime 判断编译是否真实发生；
     c) **静默死锁排查法**：传输健康（pub=1+频率正常+QoS 兼容+节点存活）却零输出零日志时，直接读源码找"无日志的 return false"分支（本例 Velodyne 分支连 not-synced 告警都没有，Livox 分支反而有），比继续堆诊断订阅更快；
   - 部署：`colcon build --packages-select fast_lio` + 重启 bring-up。
2. 配套文档：Deployment_Guide §7.2 新增包名陷阱、重编必重启两行；§7.4 新增 `/Odometry` 静默死锁条目（判据口诀：传输健康+pub=1+节点活+零日志+零输出 ⇒ 查 sync_packages 时间戳）。

## V0.0.75

Date：2026-9-8
版本主要有以下功能：
1. 新增`\HunterEdge\src\hunter_bringup\rviz\mapping_click.rviz` — 建图点航点专用配置。
2. 修改`\HunterEdge\src\hunter_bringup\CMakeLists.txt` — install 列表加入 `rviz` 目录。

## V0.0.76

Date：2026-9-8
版本主要有以下功能：
1. 修复mapping_click.rviz配置里Grid 显示项漏写了 `Class: rviz_default_plugins/Grid` 一行——YAML 语法合法，但 rviz2 不知道该用哪个插件实例化的问题。

## V0.0.77

Date：2026-9-8
版本主要有以下功能：
1. 改动巡航采图的配置src/auto_mission/config/autonomous_nav_params.yaml，使采图时驾驶安全。

## V0.0.78

Date：2026-9-8
版本主要有以下功能：
1. `src/auto_mission/scripts/pcd_to_map.py`
  - 核心修复：`data_type` 默认改空串，循环后从原始字节补解析 DATA 行（`raw[idx+1:idx+40]`），binary/ascii 均正确识别；未知格式（如 `binary_compressed`）明确报错而非静默走错分支
  - 启动自愈：新增 `convert_on_start_if_missing` 参数（默认 true），节点启动时检测到“YAML 缺失 + PCD 存在”则 2 秒后自动补转换（走现有 `_do_convert` 流程，含文件稳定等待与重入锁）
2. `src/hunter_bringup/launch/hunter_autonomous_nav.launch.py`
  - nav 模式挂载 `pcd_to_map` 常驻自愈节点（`trigger_on_mapping_end:=false`、`auto_reload_map:=true`——map_server 正常时可直接热重载）
  - 启动预检日志：YAML 缺失而 PCD 存在 → 打印警告；两者都缺 → 打印错误提示先建图
  - mapping 模式的 `pcd_to_map` 显式关闭启动自愈（避免建图开始时误转旧图、干扰建图结束触发）

## V0.0.79

Date：2026-9-8
版本主要有以下功能：
1. 修复`nav2_params.yaml` 的 costmap 配置只在 `plugins:` 名单里列了层名，但每个层都缺必填的 `plugin` 参数。

## V0.0.80

Date：2026-9-8
版本主要有以下功能：
1. 修复（`src/hunter_bringup/config/nav2_params.yaml`）
- `general_goal_checker` 补 `plugin: "nav2_controller::SimpleGoalChecker"`__——这是本次 FATAL 的直接原因。
- 两图 `inflation_radius` 0.2/0.3 → 0.35__（大于内切半径 0.33，消除碰撞风险与规划器拒路径风险。

## V0.0.81

Date：2026-9-8
版本主要有以下功能：
1. 修复一：auto_mission_node「goal 拒绝风暴」：
- lifecycle 门控：新增 `/bt_navigator/get_state` 异步查询（1Hz 节流），`nav_active_` 原子标志；IDLE 与 NAVIGATING __双重门控__，未 ACTIVE 不发送（日志 3s 节流）
- sendNextWaypoint 三重保险：退避窗口 → ACTIVE 门控 → server 发现检查（`wait_for_action_server(0)` 非阻塞，不再回 IDLE，消除 IDLE⇄NAVIGATING 跳变）
- 拒绝退避：goal 被拒/失败后 2s 退避 + 失败计数达上限（3 次）回 IDLE；连续拒收 `fail_count` 日志
- 新参数：`nav_active_wait_timeout: 60.0`、`nav_retry_backoff: 2.0`（yaml 已补注释）
2. 修复二：AMCL 全局定位 map→odom：
- nav2_params.yaml	新增 amcl 段（42 参数）：likelihood_field 模型匹配 PGM 地图、 DifferentialMotionModel（阿克曼低速近似）、set_initial_pose 地图原点、tf_broadcast: true、laser_min/max_range 与雷达 ROI 对齐。
- hunter_autonomous_nav.launch.py	新增 use_amcl 参数（默认 true）；启动 pointcloud_to_laserscan（/lidar_points→/scan，base_link 系 -0.2~0.8m 切片滤地面）+ amcl；lifecycle 顺序 map_server→amcl→...。
- package.xml（bringup）	补 nav2_amcl、pointcloud_to_laserscan 依赖。
- autonomous_nav_params.yaml	use_amcl: true + 说明。
- Deployment_Guide.md	排障表 +3 行：goal 拒绝刷屏（已修）、map→odom 缺失（AMCL 已接管）、AMCL 不收敛处置。

## V0.0.82

Date：2026-9-8
版本主要有以下功能：
1. - `src/hunter_bringup/config/nav2_params.yaml`：
- 删除不完整的 `plugin_lib_names` 与无效参数 `default_bt_xml_filename`（该 fork 实际读 `default_nav_to_pose_bt_xml`），留警示注释——bt_navigator 改用代码默认 49 项全量插件列表。
2. `src/hunter_monitor/health_monitor/{hpp,cpp}`：
  - `TopicMonitor` 新增 `ever_received`：启动宽限只豁免“从未上线”的话题——CAN 启动竞态修复，线后掉线 1s 内仍立即判异常，安全语义不降级__；
  - CAN 恢复边沿发布 `/estop=false` 并补发 5 次（覆盖 decision_making 侧 best-effort 订阅丢包）；
  - 删除 `checkNodes` 里 2s 周期的重复 `true` 发布（防止竞态下 `true` 晚于 `false` 再次锁死下游）。
3. `src/auto_mission/{hpp,cpp}`__：
新增 `estop_self_triggered_` 原子标志 + `tryReleaseSelfEstop()`——障碍物类自触发急障碍物退至减速阈值（2.0m）外后自动发布 `/estop=false` 并回 IDLE 重新规划（nav 与 mapping 两条路径都接入）；外部急停仍由 health_monitor 解除，人工锁死/解除通道保留（`ros2 topic pub /estop ...`）。

## V0.0.83

Date：2026-9-9
版本主要有以下功能：
1. 修复一（实车根因，正常导航车辆完全不动）：`hunter_autonomous_nav.launch.py` / `navigation.launch.py` cmd_vel 重映射链补齐：
   - `velocity_smoother` 补 `('cmd_vel','cmd_vel_nav')` + `('cmd_vel_smoothed','cmd_vel')`——此前 controller 输出 cmd_vel_nav 无人接收，FollowPath 指令到不了底盘（日志：52s 内 `Passing new path to controller` 刷屏但底盘零指令 → `Failed to make progress`）；
   - `behavior_server` 补 `('cmd_vel','cmd_vel_nav')`——此前 Spin/BackUp 恢复行为直发 `/cmd_vel` 绕过平滑器直达底盘，形成"只有恢复行为能动"：BackUp 以 0.2m/s 倒车（用户观察"倒着开、很慢"）、Spin 打满转向画圈（"转向迟缓"），最终车辆信念位姿被挪出地图（-19.5, 5.4）→ `Robot is out of bounds of the costmap!` 死循环。
2. 修复二（自定义行为树从未生效）：launch 中 bt_navigator 参数名 `default_bt_xml_filename` 改为本 fork 实际读取的 `default_nav_to_pose_bt_xml`（nav2_bt_navigator/src/navigators/navigate_to_pose.cpp:73）——旧名被静默忽略导致运行内置默认树（恢复池含 Spin），阿克曼车被命令原地旋转。
3. 阿克曼适配（`nav2_params.yaml`）：
   - planner `motion_model_for_search: REEDS_SHEPP→DUBIN` + `allow_reversing: false`——本 fork RPP 不支持倒车跟随，规划含倒车段必死磕；
   - RPP `use_rotate_to_heading: true→false`——开启时朝向差大命令 v=0+w，经底盘自行车模型 δ=atan(L·w/v) 换算为打满转向 0.58rad 原地不动；
   - `bt_loop_duration: 10→20`——消除 Jetson 上 "Behavior Tree tick rate was exceeded" 刷屏。
4. 航点越界防护（`auto_mission` hpp/cpp）：订阅 `/map`（transient_local+reliable，volatile 会漏收一次性发布）缓存静态地图边界；`sendNextWaypoint` 发送前校验，越界航点自动轮转跳过（ERROR 日志含边界与安全边距 0.5m），全部越界回 IDLE 防跳过风暴。实测：航点 (5,5)/(0,5) 超出地图 y≤3.85 曾引发 `Goal pose is out of costmap!` 死局。
5. `Deployment_Guide.md` 排障表 +4 行：cmd_vel 断链自检命令、spin 出现的参数名根因、航点越界处置（含重录航点流程）、lethal start 处置。
6. 部署动作：重编 `colcon build --packages-select hunter_bringup auto_mission`；重启后按需重录航点（地图 y 上限 3.85m，航点须距边界 ≥0.5m）。

## V0.0.84

Date：2026-9-9
版本主要有以下功能：
1. 修复（实车首启暴露：bt_navigator 激活失败 → lifecycle_manager `Failed to bring up` → auto_mission 卡 IDLE）：`autonomous_navigate.xml` BT 插件端口不匹配——本 fork `TransformAvailableCondition` 端口为 `child`/`parent`（transform_available_condition.hpp:65），XML 原写 `child_frame`/`parent_frame`，加载即抛 `Possible typo? ... port "parent_frame" ... does not contain a port with this name`（修复一使自定义树真正进入加载流程，方才暴露此错）。同轮核对全树端口：移除 `SpeedController` 包装（本 fork 端口为 `min_rate/max_rate/min_speed/max_speed`，按平滑速度调子树 tick 周期，无"障碍物距离→限速"语义，原 `min_distance/max_distance/time_allowance` 属性同样必崩；减速职责交 RPP `use_cost_regulated_linear_velocity_scaling` + approach 减速）；`Wait.wait_duration` 为 int 端口，`2.0/5.0→2/5`。日志同时确认此前三项修复已生效：`Using motion model: Dubin.`、`auto_mission 已缓存静态地图边界：x[-12.39, 7.41] y[-12.95, 3.85]（安全边距 0.50m）`、自定义树进入真实加载流程。

## V0.0.85

Date：2026-9-9
版本主要有以下功能（实车自主巡航安全审计 + 三层碰撞防护落地）：

背景：实车自主巡航暴露"车速过快（≈2m/s）、完全没有避障、转向打满"三联征。全链审计定位四条事故链，修复并补齐 smoother→底盘之间缺失的物理安全闸。

1. 事故链一（速度过快）：RPP `desired_linear_vel: 2.0` 全速行驶，而 Humble RPP 是**纯路径跟随器、无局部避障语义**（避障仅靠 1Hz 全局重规划），2.0m/s 下反应盲区数米且 max_decel -1.5 刹不住。修复（`nav2_params.yaml`）：`desired_linear_vel 2.0→0.8`、`max_linear_accel 1.0→0.5`、`max_linear_decel -1.5→-2.5`、新增 `max_angular_accel: 1.5`；`velocity_smoother` max_velocity `[2.0,0,2.5]→[0.8,0,1.0]`、max_decel→`[-2.5,0,-2.5]`、`velocity_timeout 1.0→0.5`；`auto_mission max_velocity 1.5→0.8` 对齐；行为树重规划 `RateController 1.0→2.0Hz`。
2. 事故链二（避障数据单点故障）：global/local 两张 costmap 的障碍层**唯一来源**为 `/perception/lidar/obstacle_cloud`（lidar_perception 聚类输出，依赖地面分割/聚类正确性）——该链异常时障碍层全空，RPP 碰撞检测/代价降速/重规划绕行全部失效。修复：`obstacle_layer` 改**双源冗余** `[lidar_cloud, scan]`——`/scan` 由 pointcloud_to_laserscan 高度切片直出（不经聚类，`clearing: true` 射线清障 + `inf_is_valid: true`）；inflation `0.35→0.40`；local_costmap 更新 `5→10Hz`、global `1→2Hz`；RPP 显式 `use_collision_detection: true` + `max_allowed_time_to_collision_up_to_carrot: 1.5`（参数存在性见 regulated_pure_pursuit_controller.cpp:89）。
3. 事故链三（转向打满）：实车日志 `set steering angle: -1.02 / -0.58`——-0.58 即 HunterV2 `max_steer_angle` 物理限幅，-1.02 为 bicycle 换算**未钳制**原始值；低速时 δ=atan(L·w/v) 发散，起步瞬间必然打满转向缓慢前挪、跟随中甩尾偏航（1s 内 3 次 `Passing new path` 佐证）。修复：新增阿克曼曲率钳制 `|w| ≤ |v|/R_min`（R_min=1.9 与 Smac `minimum_turning_radius` 一致），δ 上限 atan(0.65/1.9)≈0.33rad < 0.46（V2 central 限幅），物理上不可能再打满。
4. 新增 `src/hunter_safety` 包（safety_guard 节点）：**速度指令链最后一级**——controller/behavior → velocity_smoother → **safety_guard** → 底盘（launch 中 velocity_smoother 输出改重映射 `cmd_vel_smoothed→cmd_vel_pre_safety`）。能力：
   - 碰撞闸：`/scan` 行进方向 ±60° 扇区最近障碍 <0.6m 立即零速（COLLISION_STOP）、<1.2m 线性限速（SLOWDOWN）；
   - fail-safe：`/scan` 超时 0.5s/未到达 → 零速（SCAN_TIMEOUT）；上游指令断流 0.5s → 零速（CMD_TIMEOUT）；`/estop=true` → 零速透传（ESTOP_PASS，弥补 Nav2 goal 取消延迟窗口）；
   - 阿克曼曲率钳制 + 速度硬限 0.8m/s（第二重限速）；
   - `/safety/state` 2Hz 分级预警心跳（OK/SLOWDOWN/COLLISION_STOP/SCAN_TIMEOUT/CMD_TIMEOUT/ESTOP_PASS）。
   仅导航模式启动（mapping 模式 auto_mission cruise 直发 /cmd_vel，避免双发布者）。
5. 文档：Deployment_Guide §7.5 排障表 +1 行（三联征根因与自检链）；README §4.1 模块表/§10.1 架构图/§10.7 文件清单同步。
6. 部署动作：`colcon build --packages-select hunter_safety hunter_bringup auto_mission`（hunter_safety 为新包必须重编）；自检链：导航中 `ros2 topic echo /cmd_vel_nav`（controller 输出）→ `ros2 topic echo /cmd_vel_pre_safety`（smoother 输出）→ `ros2 topic echo /cmd_vel`（safety_guard 输出，底盘侧）均有速度流转且 |v|≤0.8；`ros2 topic echo /safety/state` 应为 `OK|…`；障碍物逼近 1.2m 内应看到 SLOWDOWN、0.6m 内 COLLISION_STOP 且车辆实际停住。

## V0.0.86

Date：2026-9-9
版本主要有以下功能（sensor_fusion 时间对齐与置信度门控 + safety_guard 自动驾驶测试模式）：

1. sensor_fusion 时间对齐（`src/hunter_perception/sensor_fusion`）：
   - `fuseAndPublish` 在双传感器有效时新增激光-视觉时间戳对齐检查：|Δt| > `max_sync_diff`(0.10s) → 本帧降级 laser_only + 节流告警。原因：视觉帧滞后（推理/传输延迟）在转向时目标位置错位（0.5rad/s 转向 × 0.3s 滞后在 5m 处横向偏差可达 ~1.5m），强行融合会拉偏位置甚至错误关联；宁缺一个旧目标，不采用错误位置；
   - 新增参数 `max_sync_diff=0.10`，启动日志同步输出。
2. sensor_fusion 置信度门控与置信度加权（vision confidence 来自 YOLO 分数，lidar 恒 0.8 无语义，故仅对 vision 通道）：
   - `visionCallback` TF 转换成功后丢弃 confidence < `vision_conf_min`(0.45) 的视觉检测（YOLO 误检/边缘抖动），防止假目标以 vision_only 身份进入决策层；门控后整帧可为空——"此刻视觉确认无高置信目标"同样是有效观测，时间戳照常更新供时间对齐判断；
   - `fusePair` 视觉权重按置信度线性缩放（conf ≥ `conf_full_scale`(0.9) 满权重 0.3/0.2，低于阈值已在门控处丢弃，再与激光权重归一化）——低置信视觉只轻微修正位置，高置信视觉才显著拉动，避免边缘检测抖动直接扰动融合输出；
   - 新增参数 `vision_conf_min=0.45`、`conf_full_scale=0.9`。
3. safety_guard 自动驾驶测试模式（`src/hunter_safety/safety_guard.cpp`，V0.0.85 碰撞防护之上新增，低速实车联调专用）：
   - 开关：`/safety/test_mode`（std_msgs/Bool，reliable）运行时 true/false 切换；参数 `enable_test_mode` 启动默认 false（`safety_guard.launch.py` 与 `hunter_autonomous_nav.launch.py` 均显式传入，改参即可启动即开）；
   - 生效时：速度硬限 0.1m/s（`test_max_linear_vel`）；±60° 扇区更严碰撞阈值 <1.0m 急停（`test_stop_dist`）/<2.0m 减速（`test_slow_dist`）；`/safety/state` 心跳 detail 追加【测试模式】标记；
   - 异常自动中止（新增 `TEST_ABORTED` 状态，零速锁存）：a) 疑似碰撞/卡死——输出指令 >`stall_cmd_vel_min`(0.05m/s) 而底盘反馈 <`stall_fb_vel_max`(0.02m/s) 持续 `stall_timeout`(1s)；b) 局部规划失效——`/navigate_to_pose` goal 被 Nav2 判 ABORTED（订阅 `_action/status` GoalStatusArray）；c) goal 活跃（ACCEPTED/EXECUTING/CANCELING）但 `/cmd_vel_nav`（controller 原始指令）断流 >`plan_fail_timeout`(2s)——需先收到过指令才启用，规避启动期误判；
   - 中止动作：`test_aborted_` 锁存（每拍零速，外部清 `/estop` 也不恢复）+ 发布 `/estop=true`（QoS(1).reliable+transient_local 锁存，auto_mission 收到即取消全部导航任务，QoS 与其 reliable/volatile 订阅兼容）+ `rclcpp_action::async_cancel_all_goals()` 直取消 bt_navigator 活动目标（双保险）；解除唯一途径：人工重新发布 `/safety/test_mode true`（视为确认现场安全）；
   - `/estop` 由 health_monitor 透传改由本节点条件发布（仅中止时），estop 透传订阅行为不变。
4. 构建配置：`hunter_safety/CMakeLists.txt` + `package.xml` 新增依赖 `rclcpp_action`、`nav2_msgs`、`action_msgs`（GoalStatusArray；package.xml description 同步测试模式说明）。
5. 文档：README §4.1 模块表/§17 话题表/§10.1 架构图/§10.8 测试模式说明；Deployment_Guide §6.6 测试模式操作、§7.5 排障 +1 行、附录 A 话题 +3 行、附录 E 安全要点 +1 条。
6. 部署动作：`colcon build --packages-select sensor_fusion hunter_safety hunter_bringup`（两包均有代码/依赖变更，须重编）。验证：`ros2 topic echo /safety/state`；开启测试模式后巡航车速应 ≤0.1m/s；障碍物进 1.0m 应 COLLISION_STOP、1.0~2.0m 限速；人为断开 `/cmd_vel_nav` 或制造 goal ABORTED 应看到 `[测试模式中止]` ERROR 且车辆停止、导航任务取消；重新 `ros2 topic pub --once /safety/test_mode std_msgs/msg/Bool "{data: true}"` 后恢复。

## V0.0.87

Date：2026-9-9
版本主要有以下功能（地图边界约束三层防线——行驶范围/航点/规划路径不得超出已采集地图区域 + 三文档完善）：
1. 规划层（`hunter_bringup/config/nav2_params.yaml`）：
   - global_costmap 新增 `track_unknown_space: true`——保留静态地图未采集区域为 NO_INFORMATION（默认 false 会把 unknown 当自由空间），配合 SmacPlannerHybrid 显式 `allow_unknown: false`（Humble 默认即 false，显式声明防回归）：全局规划路径不穿越未建图区域；目标点在未采集区域/地图外时规划直接失败（BT 快速回报），杜绝"路径悄悄伸到地图边缘外"；
   - 注意：AMCL 未重定位（车体位姿被判在 unknown 上）时规划同样失败——先重定位再下发导航，属保护行为，勿删除该参数绕过。
2. 任务层（`auto_mission` hpp/cpp，V0.0.82 航点校验增强）：`waypointInsideMap` 升级为"矩形边界+0.5m 安全边距" **且** "目标点栅格为已采集区域（occupancy != -1）"——矩形界内的建图空洞（建图边缘不齐/遮挡盲区）上的航点同样拦截；新增 `waypointMapCheckDetail` 区分拒绝原因（"静态地图矩形边界外" / "目标点落在未建图(unknown)栅格上"），`sendNextWaypoint` ERROR 日志带原因；越界航点轮转跳过、全部越界回 IDLE 的既有行为不变。
3. 执行层（`hunter_safety/safety_guard.cpp`，新增"地图边界监护"）：
   - 数据源：`/map`（transient_local+reliable，volatile 会漏收——同 V0.0.82 教训）+ `/amcl_pose`（AMCL map 系位姿，SensorDataQoS）；建图模式两者皆无 → 监护自动静默不介入；
   - `/map` 到达时构建"每个已建图栅格到最近未建图(unknown)/界外栅格"的 Chamfer 3-4 两遍距离场（一次性 O(W×H)，常规 0.05m 地图毫秒级），运行时 O(1) 查询车辆位姿处距离（界外直接判 0）；
   - tick 碰撞闸之后分级介入（与碰撞闸同构）：距未建图/界外区域 < `map_edge_stop_dist`(0.5m) → 零速，`/safety/state` → `MAP_EDGE_STOP`（常规模式回已建图区域自动恢复，无锁存；测试模式升级为 `abortTest` 中止锁存 + `/estop` + 取消全部 goal）；< `map_edge_slow_dist`(1.5m) → 线性限速（与碰撞减速取更严者），→ `MAP_EDGE_SLOWDOWN`；
   - 新参数 `enable_map_fence`(true) / `map_edge_stop_dist`(0.5) / `map_edge_slow_dist`(1.5)，`hunter_autonomous_nav.launch.py` 与 `safety_guard.launch.py` 均显式注入；`/safety/state` 状态集新增 MAP_EDGE_SLOWDOWN/MAP_EDGE_STOP；启动日志追加监护开关与阈值，就绪后打印 `[边界监护] 已就绪：栅格 WxH @0.050m/cell ...`。
4. `hunter_safety/package.xml` description 同步 V0.0.87 说明（依赖无变更：OccupancyGrid∈nav_msgs、PoseWithCovarianceStamped∈geometry_msgs 均已声明）。
5. 文档：README §8 话题表/§10.8 能力表+三层防线说明/§13.2 安全要点；User_Manual §4.5.2 边界约束行为/§6.2 排障 +3 行/§8.3 地理围栏落地说明；Deployment_Guide §5.5 参数核对 +2 行/§6.6 三层防线表/§7.5 排障 +3 行/附录 A 话题 +1 行（/amcl_pose）/附录 E 安全要点 +1 条。
6. 部署动作：`colcon build --packages-select hunter_safety auto_mission hunter_bringup`（三包均有代码/配置变更，须重编；hunter_safety 上一轮 async_cancel_all_goals 修复一并验证）。验证链：① 启动后 safety_guard 日志出现 `[边界监护] 已就绪`；② `ros2 topic echo /safety/state` 应为 `OK|…`；③ 手动向图外/unknown 区域 rviz 下发单点 goal → 应被 Nav2 拒绝（ABORTED），车辆不动；④ 遥控（REMOTE）把车开到距地图边缘 1.5m 内切 AUTO 下发目标 → 应见 `MAP_EDGE_SLOWDOWN`，更近则 `MAP_EDGE_STOP` 零速，车回图内自动恢复；⑤ 测试模式下重复 ④ → 应见 `[测试模式中止] 地图越界` 且任务取消，重发 `/safety/test_mode true` 解除。

## V0.0.88

Date：2026-9-21
版本主要有以下功能（建图模式 Ctrl+C 后 `.pgm/.yaml` 缺失修复——地图产物闭环）：

1. 问题（现象 + 根因）：建图模式采集完数据按 `Ctrl+C` 后，`maps/` 只有 `hunter_map.pcd`，没有 `hunter_map.pgm` / `hunter_map.yaml`。三条因素叠加：
   - FAST-LIO2 在 `main()` 中于 `rclcpp::spin()` 返回**之后**才把累积点云写盘（`fast_lio2/src/laserMapping.cpp` 退出保存逻辑），实测 20.7M 点 ≈ 664MB 需数秒至数十秒；
   - `Ctrl+C` 把 launch 全部子进程（含 `auto_mission_node` / `pcd_to_map`）一并 SIGINT 终止，运行期 `/auto_mission/status` 的 `MAPPING→非MAPPING` 跳变**永远不会发生**，`pcd_to_map` 原有"建图结束自动转换"从不启动；
   - 原 binary PCD 解析为逐点 `struct.unpack_from` 循环（20.7M 点需数十秒且产生 GB 级临时元组），即使触发也难以在退出窗口内完成。
2. 修复（`src/auto_mission/scripts/pcd_to_map.py`）：
   - **退出兜底转换**：新增参数 `final_convert_on_shutdown`（建图模式置 true）/ `final_wait_timeout`（默认 120s）/ `final_log_file`（默认 `maps/pcd_to_map_final.log`）。节点在自身退出路径（`main()` finally，先于 rclpy 关闭）调用 `finalize_on_shutdown()`，派生**独立会话（setsid / `start_new_session=True`）**的后台转换进程——不受终端 Ctrl+C 进程组信号与 launch 对子进程的 SIGINT/SIGTERM/SIGKILL 影响；后台进程等 PCD **写完整**后转换，进度写入日志，结束时按 PID 清理锁文件 `.pcd_to_map_final.lock`（防止重复 Ctrl+C 派生多个转换进程）；
   - **写完整判定**：新增 `PcdParser.expected_file_size()`（只读文件头，按 `POINTS` 或 `WIDTH×HEIGHT` 与字段行宽推算完整字节数）精确识别"写完"，不再依赖盲等；ASCII/未知行宽退化为"文件大小连续 2 次稳定"并保证足够观察窗口；
   - **binary 解析向量化**：新增 `_parse_binary_numpy()`（numpy 结构化 dtype + `np.frombuffer`），FAST-LIO2 输出（8 字段 float32）实测 20.7M 点 664MB 解析+建栅格+写盘全程 ≈4.5s（原逐点循环在该数据量下不可用）；不适用字段布局自动回退原逐点实现；
   - **原子写盘**：`.pgm`/`.yaml` 先写 `.tmp` 再 `os.replace`——转换进程被中途终止也不会留下"半个地图"（`.yaml` 缺失会导致 map_server 加载失败）；
   - **可脱离 ROS 运行**：ROS 导入移入 try/except + `from __future__ import annotations`，`python3 pcd_to_map.py --finalize --pcd-file <pcd> --map-output-dir <dir> [--force]` 可在任意装有 numpy 的环境离线转换；
   - 原有路径行为不变：运行期 `MAPPING→非MAPPING` 触发（先 `/fast_lio2/map_save` 再转换）、手动 `/pcd_to_map/convert`、nav 模式启动自愈；运行期自动转换新增 `convert_wait_timeout`（默认 60s）等待写盘完成。
3. `src/hunter_bringup/launch/hunter_autonomous_nav.launch.py`（mapping 模式）：`pcd_to_map` 注入 `final_convert_on_shutdown=true`、`final_wait_timeout`（新 launch 参数 `final_convert_wait_timeout`，默认 120.0，eMMC/慢盘可调大）与 `convert_wait_timeout=60.0`；launch 文件头"结束"说明同步。
4. 验证（离线合成 PCD 自测 6 用例全过 + 真实地图回放，`--finalize` 无需 ROS 即可复跑）：
   - 完整 binary PCD → `.pgm`+`.yaml`（P5 头/尺寸/origin、YAML 字段、无 `.tmp` 残留）；
   - PCD 先写一半、6s 后补齐：后台进程等待至"写完整"才转换（不提前转换半张图）；
   - `.yaml` 比 `.pcd` 新 → 跳过转换不重复写；ASCII PCD → 成功；PCD 缺失 → 退出码 2；
   - 真实地图（`maps/hunter_map.pcd`，633.6MB/20,761,701 点）：全程 4.5s，栅格 396×336 @0.05m，origin (-12.389, -12.951)，与已知地图边界 x[-12.39, 7.41] / y[-12.95, 3.85] 一致。
5. 文档：README §10.1/§10.5/§13.8；User_Manual §4.5.1/§6.2；Deployment_Guide §6.2/§6.3/§6.5/§7.5/§5.8 版本表。
6. 部署动作：`colcon build --packages-select auto_mission hunter_bringup`（脚本/launch 安装变更）。验证链：① 建图模式 `Ctrl+C` 后数十秒内 `ls -lh maps/` 应出现 `.pgm/.yaml`；② `tail -f maps/pcd_to_map_final.log` 可见 `PCD 已写完整 → 转换成功`；③ 若仍缺产物可手动兜底：`python3 ~/HunterEdge/install/auto_mission/lib/auto_mission/pcd_to_map --finalize --pcd-file ~/HunterEdge/maps/hunter_map.pcd --force`。

## V0.0.89

Date：2026-9-21
版本主要有以下功能（Nav2 Humble `observation_sources` 类型修复 + 窄小测试场地低速自主巡航全面调优）：

背景：实车进入导航巡航模式后车辆不动、且频繁向后倒退。日志复盘出两条故障链——
- **导航栈根本起不来**：Humble `nav2_costmap_2d::ObstacleLayer` 将 `observation_sources` 声明为**空格分隔字符串**，而 `nav2_params.yaml` 写成 YAML 列表 `[lidar_cloud, scan]` 被解析为 `string_array` → `Wrong parameter type ... is of type {string}, setting it to {string_array} is not allowed` → local/global_costmap `configure` 抛异常 → `global_costmap/clear_entirely_global_costmap` 服务不存在 → bt_navigator 激活失败 → `lifecycle_manager` `Aborting bringup`，全栈不可用；
- **能起来后只退不进**：起步位姿落在致命栅格（近地杂波 + 车顶雷达自反射把车身/脚底标为 lethal）→ SmacPlanner `Starting point in lethal space` 规划失败 → 行为树恢复 `BackUp` 倒车 0.3m（AMCL 位姿随倒车跳变 (3.77,5.28)→(6.34,0.88)）→ 6 次重试仍失败 → `Goal failed/ABORTED`；测试模式下首条 `goal ABORTED` 即触发 `abortTest` 零速锁存 + 0.1m/s 限速 → “车辆基本不动”。

1. **阻断性修复**（`hunter_bringup/config/nav2_params.yaml`）：global/local 两处 `obstacle_layer.observation_sources: [lidar_cloud, scan]` → `"lidar_cloud scan"`（Humble 要求的空格分隔字符串），导航栈恢复可正常 activate。
2. **杜绝倒退**（`hunter_bringup/behavior_trees/autonomous_navigate.xml`）：移除一切运动型恢复（`BackUp` 两处 + 隐含 `Spin`），恢复退化为“清除全局/局部代价地图 + `Wait` 后重试”的非运动组合；`TrackingRecovery` 的 `BackUp 0.3m` → `Wait 2`，`RecoveryFallback` 轮询池 `BackUp/Wait 5` → `Wait 3`。窄场地不再把车倒进更差栅格、不再位姿跳变。
3. **感知自反射抑制 + 融合对齐放宽**：`hunter_autonomous_nav.launch.py` 中 `pointcloud_to_laserscan.range_min 0.5→0.8`（> 车身最大外接半径 ≈0.55m，滤除车顶雷达看到自身/支架的近距反射，消除 safety_guard 常年 ~0.6m 假障碍急停）；`amcl.laser_min_range 0.5→0.8` 对齐；`sensor_fusion_params.yaml` 新增 `max_sync_diff: 0.20`（实测 Realsense 滞后稳定 0.11~0.15s，旧 0.10s 使每帧降级 laser_only，低速 ≤0.5m/s 下 0.20s 内目标位移 <0.1m 可安全放宽），视觉真正参与融合。
4. **解决起点致命**（`nav2_params.yaml` 两张 costmap `obstacle_layer`）：点云源 `min_obstacle_height 0.0→0.3`（跳过底盘/地面近地杂波）、`max_obstacle_height 2.0→1.5`；两源 `obstacle_range 5.0→3.0`、`raytrace_range 6.0→4.0`（窄场地尺度，避免远距杂波 flood 车脚）。⚠ 若静态地图在车辆下电位姿处确有黑色栅格，仍需人工把车驶入自由区并用 rviz `2D Pose Estimate` 重定位（数据/定位问题，参数不可完全绕过）。
5. **低速控制（统一 ≤0.5m/s、禁倒车）**：RPP `desired_linear_vel 0.8→0.5`、`max_linear_accel 0.5→0.3`、`max_linear_decel -2.5→-1.5`、`regulated_linear_scaling_min_speed 0.2→0.1`、`cost_scaling_dist 0.6→0.4`、`max_angular_accel 1.5→1.0`；`velocity_smoother` `max_velocity [0.8,0,1.0]→[0.5,0,0.8]`、`min_velocity` 线性 `-0.5→0.0`（**硬禁倒车**）、`max_accel [0.5,0,2.0]→[0.3,0,1.0]`、`max_decel [-2.5,0,-2.5]→[-1.5,0,-1.5]`。
6. **决策层适配**（`auto_mission/config/autonomous_nav_params.yaml`）：`max_velocity 0.8→0.5`；`warn_obstacle_dist 2.0→1.5`、`stop_obstacle_dist 0.8→0.6`（略大于 footprint 前缘 0.45）；`goal_timeout 60→90`（非运动恢复周期变长）、`nav_retry_backoff 2.0→3.0`。
7. **安全闸与测试模式调优**（`hunter_safety/src/safety_guard.cpp` + `hunter_autonomous_nav.launch.py`）：
   - 新增参数 `test_max_goal_aborts`（默认 1=旧行为）：测试模式对 `goal ABORTED` **上升沿计数**，连续达阈值才锁存中止，观察到 `EXECUTING` 清零——避免窄场地起步期不可规划航点的瞬时 ABORT 一票否决整个测试，行驶中反复 ABORT 仍会中止（安全网保留）；launch 注入 `3`；
   - 测试场地档：常规 `max_linear_vel 0.8→0.5`、`stop_dist 0.6→0.5`、`slow_dist 1.2→1.0`；测试模式 `test_max_linear_vel 0.1→0.3`、`test_stop_dist 1.0→0.5`、`test_slow_dist 2.0→1.0`（原 0.1m/s 是“基本不动”主因之一）；`plan_fail_timeout 2.0→10.0`（> 非运动恢复 Wait 总时长，避免清图/等待期误判控制器断流）；`map_edge_stop_dist 0.5→0.4`、`map_edge_slow_dist 1.5→1.0`。
8. 文档：README、User_Manual、Deployment_Guide 同步测试场地低速档与排障链（倒退/不动根因、observation_sources 类型、重定位前置要求）。
9. 部署动作：`colcon build --packages-select hunter_safety hunter_bringup sensor_fusion auto_mission`（hunter_safety 代码 + 其余配置/launch 变更）。验证链：① 启动不再出现 `Wrong parameter type` 且 `lifecycle_manager` 走完 activate；② 导航中车辆**只前进不后退**，`ros2 topic echo /cmd_vel` 的 `linear.x ≥ 0` 且 ≤0.5；③ 起步位姿落致命栅格时清图重试而非倒车，人工把车驶入自由区 + rviz `2D Pose Estimate` 重定位后可正常规划；④ 开测试模式巡航车速应 ≤0.3m/s，短暂规划失败不再立即锁死，连续 3 次 ABORT 才中止；⑤ `ros2 topic echo /safety/state` 常态 `OK|…`。



## V0.0.90

Date：2026-9-21
版本主要有以下功能（建图退出后 `.pgm/.yaml` 仍缺失的残余故障链修复：PCD 保存体素去重 + 退出宽限 + 截断容错）：

背景：V0.0.88 兜底机制上线后实机仍复现“建图 23 分钟、Ctrl+C 后 maps/ 只有 .pcd 无 .pgm/.yaml”。日志复盘出三环相扣的故障链——
- **点云无界累积**：`pcl_wait_save` 逐帧全量累加无降采样，实测累积 **381,848,913 点 ≈ 6.1GB** binary PCD，且仅在退出瞬间（rclcpp::spin 返回后）写盘，需数分钟；
- **写盘被腰斩**：launch 对 fast_lio2 默认只给 5s SIGINT 宽限（日志 `failed to terminate '5' seconds ... escalating to 'SIGTERM'`，`exit code -15`），6.1GB 只写出零头，PCD 截断；
- **兜底转换必然超时**：V0.0.88 后台进程按 header 推算“写完整 = 6.1GB”，对截断文件死等 120s 后**直接放弃转换**，不留任何部分地图。

1. **根因修复——保存体素去重**（`fast_lio2/src/laserMapping.cpp` + `hunter_bringup/config/fast_lio2_params.yaml`）：新增参数 `pcd_save.save_voxel_size`（默认 0.1m，0=关闭）；保存链路按全局体素键（坐标量化 + 64bit 哈希集）去重后再累加，点数下降 1~2 个数量级（重复扫面/静止段点全部合并），写盘秒级完成；0.1m 精度优于 0.05m 栅格，不损失占据图细节；内存/磁盘占用同步下降（Ctrl+C 瞬间的 6.1GB 写盘 → 百 MB 量级）。
2. **退出宽限**（`hunter_bringup/launch/localization.launch.py`）：fast_lio2 节点配 `sigterm_timeout='120'`——Ctrl+C 后给 120s 写盘窗口，写完即退（去重后实际秒级），不再存在“大地图必然被 SIGTERM 截断”；`sigkill_timeout='10'` 显式化。
3. **兜底转换截断容错**（`auto_mission/scripts/pcd_to_map.py`）：
   - `wait_for_complete_pcd`：可推算完整字节数但达不到时，若文件大小连续 ≥3 轮采样（~6s）不变，判定“写盘方已终止”，按截断图接受（旧逻辑此场景必死等到超时后放弃）；
   - `convert_pcd_to_map` / `--finalize`：等待超时但 PCD 非空时不再直接退出，按已写入部分尽力转换（binary 解析本就按可用字节数容忍截断），产物在日志/返回消息中注明“部分地图”；仅 PCD 始终不出现才报错；
   - **退出链加固**：实机日志两次复现 `pcd_to_map` 收到 SIGINT 后 5s 不退→SIGTERM→10s→SIGKILL——`rclpy.spin` 阻塞在 cyclonedds waitset 的 C 层时，Python 信号处理器抛出的 KeyboardInterrupt 长时间不被执行，退出兜底派生机命悬一线。改为 0.3s 分时 `spin_once`（信号异常≤ 0.3s 内得到处理）+ 兜底派生后直接 `os._exit(0)`（不再执行会同样挂死的 destroy/shutdown，退出控在亚秒级）；派生信息同时 `print(flush)` 一份，保证 launch 日志可见证据。
4. 文档：`hunter_autonomous_nav.launch.py` 建图结束流程说明、README/User_Manual/Deployment_Guide 软件基线同步 V0.0.90。
5. 部署动作：`colcon build --packages-select fast_lio2 hunter_bringup auto_mission`（fast_lio2 为 C++ 代码变更，必须重编；余为配置/脚本/launch）。验证链：① 建图数分钟后 `Ctrl+C`，日志应见 `Mapping finished, saving …` 后 **快速出现** `[fast_lio2] map saved`，且 fast_lio2 不再以 `exit code -15` 被杀；② `ls -lh maps/` 齐 `.pcd/.pgm/.yaml`（pcd 体积应比旧版下降 1~2 个数量级）；③ `maps/pcd_to_map_final.log` 见 `PCD 已写完整 → 转换成功`；④ 旧场景回归：人为 kill -9 写盘中的 fast_lio2，兜底进程应输出“写盘方疑已终止/部分地图”并仍产出 .pgm/.yaml。
6. 历史数据救回：对本次已截断的 `hunter_map.pcd`，升级后可离线尽力转换：`python3 install/auto_mission/lib/auto_mission/pcd_to_map --finalize --pcd-file ~/HunterEdge/maps/hunter_map.pcd --wait-timeout 5 --force`（注意：3.8 亿点全量解析峰值内存 ~10GB+，若机载端 OOM 则拷至 PC 离线转，或直接用新流程重新建图）。



## V0.0.91

Date：2026-9-21
版本主要有以下功能（实车撞墙事故复盘：碰撞闸盲区修复 + 接管后“起点致命栅格”死锁链修复）：

背景：车端导航日志（5596 行）复盘出两条独立故障链——

**问题①“行驶途中撞上墙壁，避障没做好”＝三层避障同时失效**：
- `hunter_autonomous_nav.launch.py` 中 `pointcloud_to_laserscan.range_min=0.8` 与 `safety_guard.stop_dist=0.5` **参数互相矛盾**：小于 0.8m 的回波在投影阶段即被丢弃，`dist < stop_dist` 在数学上不可达 → 急停闸是死代码。日志决定性证据：全程 **0 次 COLLISION_STOP**，而 safety_guard 报出的“最近障碍”最小值恰为 **0.800m**（盲区地板值）——障碍一旦靠近到 0.8m 内就从 /scan 消失，状态回 `OK` 全速放行，直到撞上；
- RPP `use_collision_detection` 依赖的 local_costmap 因观测原点越界（见问题②）无法清障/更新，只在事后报 `detected collision ahead!` 并 ABORT；
- auto_mission 第三层（基于 `/perception/fused_objects`，warn 1.5/stop 0.6）全程未触发——聚类/融合链没把墙当目标。

**问题②“遥控接管移车后重新自主，车停在原地不动”＝假障碍永久累加 + 观测时基错位 + 状态机静默抖动**：
- `nav2_params.yaml` 两张 costmap 的 `lidar_cloud` 源 `clearing: false` → 只标不清，`clear_entirely_*` 清完立刻被下一帧重标，致命栅格铺满车体下方；接管后所有航点对每个停车位置（(5.78,0.66)/(6.47,2.17)/(-4.82,1.71)/(12.73,8.65)…）持续报 `GridBased: failed to create plan, invalid use: Starting point in lethal space!`；
- `lidar_perception.transformToBaseLink()` 用 `tf2::TimePointZero`（=最新 TF）把点云变换到 base_link，却沿用输入消息的 stamp 作为输出时间戳 → 几何属于“now”、时间戳属于“stamp”。costmap 按 stamp 反查 sensor origin 得到旧位姿，日志表现为原点与 rolling window 中心**恒偏 ~6m**：`Sensor origin at (4.60,-4.59) is out of map bounds (2.35,-15.10) to (12.32,-5.13)`（local）、global 同样越界并伴 `Robot is out of bounds of the costmap!` → `raytraceFreespace` 整体被跳过，清障彻底失效；
- 行为树 V0.0.89 起恢复池仅剩“清图 + Wait”非运动手段（`behavior_server: Running wait / wait completed successfully` 循环刷屏），救不了“起点致命”这种需重定位/移车的死局；
- auto_mission 达失败上限时回 IDLE，而下一拍 AUTO 条件仍满足 → 重置计数、重新从航点[0] 发送同一批不可规划目标，IDLE⇄NAVIGATING 静默抖动，车辆永不移动且无任何对外故障上报。

1. **碰撞闸去盲区**（`hunter_safety/src/safety_guard.cpp`）：判据由“扇区内最小径向距离 r”改为“**前方矩形安全走廊内最小纵向净空 x**”——极坐标回波投影到车体系（x=r·cosθ, y=r·sinθ），只统计 `|y| ≤ corridor_half_width(0.45)` 且位于车体前缘之外的回波。两个效果：① 斜 45° 的墙角不再被斜距“美化”（r=0.70 → x=0.50，旧逻辑按 0.70 漏判）；② 平行侧墙（|y|≈0.5）不再常年误急停，正是这条误报曾迫使 V0.0.89 把 stop_dist 压到 0.5。同时新增自车包络屏蔽盒（`footprint_front/rear/half_width + self_margin`）过滤车身/支架自身反射，取代靠抬大 `range_min` 粗截断的做法。
2. **盲区一致性强制校验**（同上）：tick 内按 `/scan` 实际 `range_min + 0.15` 强制抬升有效急停距离并一次性 ERROR 告警，杜绝“参数再次悄悄失效”；走廊半宽 ≤ 车体半宽+余量时启动即告警并放宽（宁可漏报由 costmap 兜底，不可误报到不能走）。日志文案改为“行进方向走廊净空 x m（侧偏 y m）”，便于现场判读触发点的纵向/横向位置。
3. **阈值对齐**（`hunter_autonomous_nav.launch.py` + `safety_guard.launch.py`）：`stop_dist 0.5→1.0`、`slow_dist 1.0→1.8`，测试模式 `test_stop_dist/test_slow_dist 0.5/1.0→1.0/1.8`（急停必须 > `range_min 0.8`）；新增并显式注入 `corridor_half_width/footprint_front/footprint_rear/footprint_half_width/self_margin`；`range_min` 保持 0.8（与 `amcl.laser_min_range` 对齐，避免污染 costmap/AMCL）并加注释声明与 stop_dist 的强耦合关系。
4. **代价地图可清障**（`hunter_bringup/config/nav2_params.yaml`）：global/local 两处 `lidar_cloud` 源 `clearing: false → true`，使假障碍可被 raytrace 消退，`clear_entirely_*` 不再“清完即重标”。
5. **观测时基修正**（`hunter_perception/lidar_perception/src/lidar_perception.cpp`）：`transformToBaseLink` 改按**消息时间戳**查 TF（与输出 header 同源），仅该时刻 TF 不可用时降级取最新 TF 并 WARN 提示“请核查 /lidar_points 时延与对时”。这是让 costmap 清障真正生效的前提。
6. **任务故障锁存**（`auto_mission`）：`MissionState` 新增 `FAULT`，三处“达失败上限回 IDLE”（result ABORTED/CANCELED、goal 被拒、单点超时）统一改调 `enterFault()`：取消 goal、计数归零但**靠状态拦截重发**，每 5s 输出带处置指引的告警（检查脚底假障碍/重定位/航点可达性），需模式开关离开 AUTO 再切回才能重启任务；`health=CRITICAL` 不再顺带清掉 FAULT。不发 `/estop`（车已停且非危险场景，保持急停通道语义纯净）。
7. **起步前主动清图**（同上）：新增 `clearCostmapsOnStart()`，在 `IDLE→NAVIGATING` 与 `WAITING_LOCALIZE→NAVIGATING` 两条起步路径上异步请求 `global/local_costmap/clear_entirely_*`——把清图从“规划失败后的补救”前置到“起步前”，配合行为树原有恢复打破死循环。
8. 文档：README §10.8 能力表（碰撞闸判据/新阈值）、User_Manual 排障链、Deployment_Guide §5.5 参数核对与 §7 排障同步本次两条故障链与“接管后重新自主前置检查”。
9. 部署动作：`colcon build --packages-select hunter_safety auto_mission lidar_perception hunter_bringup`。验证链：
   - ① 低速贴近墙面行驶，`/safety/state` 应在净空 ≈1.0m 时出现 `COLLISION_STOP|行进方向走廊净空 …< 急停距离 1.00m`，**不再出现“最近障碍恒为 0.800m 后消失”**；
   - ② 启动日志出现 `safety_guard 启动：… 走廊±0.45m（车体包络 0.45/0.37/0.32+m0.12）`，且无“急停距离 ≤ range_min”ERROR；
   - ③ `ros2 topic echo /perception/lidar/obstacle_cloud --once` 的 header.stamp 与到达时刻差应 <0.3s；若明显偏大须先解决雷达对时/时延（`ros2 topic delay /lidar_points /scan`）；
   - ④ 遥控接管移车后重新 AUTO：日志应见 `[清图] 已请求清除 …`，规划不应再持续报 `Starting point in lethal space`；连续 3 次失败后状态应锁存为 `FAULT` 并每 5s 输出处置指引（不再无限重发）。
10. 遗留风险（本次未改，需现场验证）：`ekf_params.yaml` 以 `odom0: /Odometry`（FAST-LIO2 `camera_init` 绝对位姿，`relative:false`、`differential:false`）整包 6DOF 注入 `odom` 系 EKF，若 fast_lio2 重启（camera_init 归零）会使 odom 系整体跳变，日志中接管期的位姿大幅跳变与该嫌疑吻合；auto_mission 第三层 `stop_obstacle_dist(0.6)` 与融合感知对墙面类大面积低矮障碍的漏检也需专项优化。



## V0.0.92

Date：2026-9-21
版本主要有以下功能（导航启动阶段“走几步即停”故障链修复：AMCL 收敛门控 + 起步清图同步等待 + 重规划降频）：

背景：车端日志（4522 行）复盘——车辆自主导航正常启动、进入 AUTO 后走几步即停下不再巡航。故障链四环相扣：
- **AMCL 未收敛即放行导航**：`isLocalizationValid()` 只查 EKF（`odom→base_link`）协方差，不查 AMCL（`map→base_link`）协方差。日志显示 `IDLE→NAVIGATING` 时 AMCL 仅初始化 2.5s，粒子远未收敛，全局位姿估计仍有较大不确定性，但已被判定“定位已收敛”直接起步；
- **代价地图未清除即发 goal**：V0.0.91 的 `clearCostmapsOnStart()` 为 fire-and-forget，进入 NAVIGATING 时若 `clear_entirely_global/local_costmap` 服务尚未就绪则直接跳过、不重试，紧接着 `sendNextWaypoint()` 已把 goal 发给带着接管期假障碍的脏 costmap；
- **2Hz 重规划放大抖动**：`autonomous_navigate.xml` 的 `RateController hz=2.0`（V0.0.85/82 为动态障碍提速所致）叠加上述脏 costmap 里的假障碍，每 0.5s 一次重规划产出的路径方向都在变，RPP 跟一个尚未收敛的 AMCL 位姿去追一个每 0.5s 就换方向的目标，转向角在 ±22°（阿克曼满舵）间全幅振荡，实测车辆原地蛇形约 30s；
- **蛇形偏航驶入静态地图墙体，无任何恢复手段**：振荡累积偏航把车带进静态地图（`static_layer`，`clearing` 不影响）的黑色栅格区，此后每次重规划均报 `Starting point in lethal space!`，而行为树 V0.0.89 已移除 `Spin`/`BackUp`，非运动恢复（清图+Wait）对静态层致命区无能为力，最终 `FAULT` 锁存。

1. **AMCL 收敛门控**（`auto_mission`）：新增 `/amcl_pose` 订阅（`transient_local + reliable`，与 AMCL 发布 QoS 对齐，`set_initial_pose: true` 场景下订阅建立即可收到首帧）；`isLocalizationValid()`/`isAutoConditionMet()` 在原 EKF 检查之后追加 AMCL `cov[0]+cov[7]`（x+y 方差和）检查，未收敛禁止进入 `WAITING_LOCALIZE→NAVIGATING`；新增参数 `amcl_cov_threshold`（默认 0.60，**故意略高于** `set_initial_pose:true` 初始帧方差和 0.5，使“上电位姿=建图起点”这一常规场景可即时放行，仅在位姿跳变/粒子发散导致方差膨胀时才拦截；若车辆上电不在建图起点，需调低至 0.10 并手动 rviz2 `2D Pose Estimate` 重定位）。
2. **起步清图改为同步门控**（同上）：`clearCostmapsOnStart()` 不再 fire-and-forget——先置 `costmaps_clear_pending_=true`，若服务已就绪则同步清除并复位标志；若未就绪，`IDLE→NAVIGATING`/`WAITING_LOCALIZE→NAVIGATING` 两条入口路径**移除原先紧接着的 `sendNextWaypoint()` 调用**，改由 `NAVIGATING` 状态入口每 100ms（10Hz 主循环）重试清图，直到两个服务均就绪并完成清除后才 fall-through 到“无在途 goal 则发送”分支，确保第一个 goal 永远打在干净的代价地图上。
3. **重规划降频防振荡**（`hunter_bringup/behavior_trees/autonomous_navigate.xml`）：`RateController hz` `2.0→1.0`（回退 V0.0.85/82 的提速决定）——动态障碍避让本就由 `safety_guard` 物理碰撞闸 + RPP `use_cost_regulated_linear_velocity_scaling` 近障碍降速兜底，2Hz 的额外收益远低于其在假障碍场景下诱发全舵振荡的代价；配合上述“清图同步门控”，从源头消除“脏图 × 高频重规划”的组合风险。
4. **参数与文档**：`autonomous_nav_params.yaml` 新增 `amcl_cov_threshold: 0.60`，`localize_wait_timeout` `10.0→15.0`（AMCL 收敛需要多帧激光扫描，旧 10s 阈值实测偶有超时误判）；README §10.2/§10.4/§10.6、User_Manual §2.2/§4.5.2/§4.5.3、Deployment_Guide 版本头与 §5.5 同步。
5. 部署动作：`colcon build --packages-select auto_mission hunter_bringup`（auto_mission 为 C++ 代码变更，必须重编；hunter_bringup 为行为树 XML + launch 配置）。验证链：
   - ① 冷启动、AMCL 尚未收到 `/scan` 更新（车静止）时，日志应见 `[IDLE] AUTO 条件未满足...定位收敛=否`，不应立即跳 `IDLE→NAVIGATING`（若车已按 `set_initial_pose:true` 落在建图起点，`amcl_cov_threshold=0.60` 允许即刻放行，属预期行为）；
   - ② 手动重启 Nav2（服务短暂不可用）后立即切 AUTO，日志应见 `[清图] ... 未就绪，将在 NAVIGATING 中每 100ms 重试`，随后 `[清图重试] 已请求清除 ...`，`NAVIGATING` 入口不应在清图完成前出现任何 `sendNextWaypoint`/`Starting point in lethal space` 日志；
   - ③ 起步后车辆应平稳前进，转向角不再出现 ±22°（满舵）连续振荡；`/compute_path_to_pose` 相关日志节奏应与 1Hz 重规划一致而非 2Hz。



## V0.0.93

Date：2026-9-21
版本主要有以下功能（**方案A：定位架构重构**——TF 单一所有权 + FAST-LIO2 独占 odom→base_link + NDT 点云重定位替代 AMCL + 局部控制 RPP→MPPI，彻底根治“无法自主导航巡航”）：

背景：车端导航日志（477 行）复盘——车辆始终卡在 `IDLE`（`定位收敛=否`）无法进入巡航，且 `map bounds` 在 1s 内跳变 30~40m、持续 `Robot is out of bounds` / `Sensor origin out of map bounds`。根因不是 AMCL 参数，而是 **`odom→base_link` TF 存在两个发布者打架**：
- `hunter_base`（`hunter_messenger.hpp` 的 `PublishOdometryToROS()` 无条件 `sendTransform`，用轮速死推算 odom→base_link）与 `robot_localization` EKF（`ekf_params.yaml publish_tf: true`）**同时广播同一条 TF 边**；
- 二者数值随时间发敢 → TF 树 odom 原点来回拖动 → costmap 无法 raytrace 清障（观测原点反复越界）→ AMCL 粒子永不收敛（`map bounds` 跳变）→ `isLocalizationValid()` 恒为假→ 卡在 IDLE。V0.0.92 的 AMCL 收敛门控只是治标（因打架而永不收敛，门控恒关）；与 V0.0.91 遗留风险⑩的预测吻合。

参考业界低速自动驾驶标准分层，本版本按“**多传感器 LIO/SLAM 定位 → 全局路径 → 局部避障(MPPI) → 独立安全监控 → 状态机健康闸门**”重构：TF 链固定为 `map --NDT--> odom --FAST-LIO2--> base_link`，每条边**唯一发布者**。

1. **底盘让出 TF 所有权（P0 根因修复）**（`hunter_ros2/hunter_base`：`hunter_messenger.hpp`/`hunter_base_ros.cpp`/`hunter_base.launch.py` + `hunter_bringup/launch/hunter_full.launch.py`）：新增 `publish_tf` 参数（默认 true 以兼容单独调试），集成启动 `hunter_full.launch.py` 对底盘传 `publish_tf:=false`——底盘不再广播 `odom→base_link`，仅发布 `/odom` 话题供 EKF 融合。消除与定位层的双发布者冲突。
2. **FAST-LIO2 帧名归一**（`fast_lio2/src/laserMapping.cpp`）：世界系/机体系由 `camera_init`/`body` 统一为 ROS 标准 `odom`/`base_link`（含 /Odometry、TF、/cloud_registered、/path 等全部发布）；FAST-LIO2 成为 `odom→base_link` 的**唯一拥有者**。
3. **EKF 退出 TF 广播**（`hunter_bringup/config/ekf_params.yaml`）：`publish_tf: true → false`。EKF 仍输出 `/odometry/filtered` 供诊断/备用，但不再进入 TF 树（避免 `odom→base_link` 二次冲突与 `map→odom` 与 NDT 冲突）。
4. **新增 `hunter_relocalization` 包（NDT 全局重定位）**（`src/hunter_relocalization/`：`ndt_relocalization_node.cpp` + `package.xml`/`CMakeLists.txt`/`config/relocalization_params.yaml`/`launch/relocalization.launch.py`）：以 PCL `NormalDistributionsTransform` 将 FAST-LIO2 实时点云（/cloud_registered，odom 系）配准到先验全局 `.pcd`（map 系），估计/持续修正 `map→odom` TF，**替代 AMCL**（AMCL 2D 粒子滤波对 3D 低矮结构/大范围地图不收敛）。全局地图体素降采样为 NDT target；滚动时间窗累积局部子图并降采样为 source；初值默认单位阵（上电≈建图起点），支持 `/initialpose` 重置；带收敛判据（hasConverged+fitness阈值）与跳变闸门（单周期平移/旋转上限）。发布 `/relocalization/pose`（收敛时小协方差、未收敛时大协方差）。
5. **健康闸门改判据**（`auto_mission`）：`isLocalizationValid()`/回调订阅话题由 `/amcl_pose` 改为 `/relocalization/pose`（成员名/函数名保留，最小改动）；`autonomous_nav_params.yaml` 中 `amcl_cov_threshold: 0.60 → 0.10`（NDT 收敛发布 0.01×2=0.02 放行，未收敛 100 拦截），`use_amcl` 废弃。
6. **局部控制 RPP→MPPI**（`hunter_bringup/config/nav2_params.yaml`）：`FollowPath` 插件改 `nav2_mppi_controller::MPPIController`，`motion_model: Ackermann`，低速档参数（vx_max 0.5/vx_min 0 禁倒车/wz_max 0.8）+ 一整套 critics（Constraint/Cost/Goal/GoalAngle/PathAlign/PathFollow/PathAngle/PreferForward）——模型预测控制天然处理阿克曼最小转弯半径/平滑转向/近障减速，消除 RPP 大朝向差与窄通道“死磕”。
7. **导航栈接线**（`hunter_autonomous_nav.launch.py`）：移除 `amcl` 节点与 lifecycle `amcl` 注入；新增 `hunter_relocalization` 节点（`global_map_path` 由 `map_yaml_path` 去后缀 + `.pcd` 自动派生）；`pointcloud_to_laserscan`（/lidar_points→/scan）去除 `use_amcl` 条件，常驻供 costmap 障碍层与 safety_guard 使用；`use_amcl` 参数保留但标废弃。
8. **安全监控位姿源**（`hunter_safety/src/safety_guard.cpp`）：地图边界监护的 map 系位姿订阅由 `/amcl_pose` 改为 `/relocalization/pose`（走廊净空急停/限速/cmd·scan 看门狗逻辑不变）。
9. **建图航点一致性**（`auto_mission/scripts/waypoint_recorder.py` + `hunter_bringup/rviz/mapping_click.rviz`）：因 FAST-LIO2 世界系改名，航点点击坐标系校验与 rviz `Fixed Frame` 由 `camera_init` 改为 `odom`（数值不变，仅帧名同步）。
10. 文档：README §10（架构/定位链/控制器表）、User_Manual（版本头/定位自检/行为树执行/起步要求）、Deployment_Guide（部署与参数核对/排障）同步 V0.0.93。
11. 部署动作：`colcon build --packages-select hunter_base fast_lio hunter_relocalization hunter_safety auto_mission hunter_bringup`（hunter_base/fast_lio/hunter_relocalization/hunter_safety/auto_mission 为 C++ 代码变更，必须重编；hunter_bringup 为配置/launch。⚠ **FAST-LIO2 包名为 `fast_lio`（package.xml）而非目录名 `fast_lio2`，写 `fast_lio2` 会被 colcon 报 `ignoring unknown package` 并静默跳过，导致帧名归一变更未编译——必须核对无此警告**）。新包 `hunter_relocalization` 需 `sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup libpcl-dev`（PCL 已随 fast_lio 依赖存在）。
12. 验证链：
   - ① `ros2 run tf2_ros tf2_echo map base_link` 应稳定连续变化；`ros2 run tf2_tools view_frames` 确认 `odom→base_link` 唯一父节点为 `fast_lio2`、`map→odom` 唯一父节点为 `hunter_relocalization`，**无任何一条边两个发布者**（`tf2_monitor` 无 multiple children 告警）；
   - ② `ros2 topic echo /relocalization/pose --once`：收敛时 `covariance[0]/[7]≈0.01`；日志见 `[relocalization] ✓ 收敛 fit=...`；冷启动若车不在建图起点，用 rviz2 `2D Pose Estimate` 发 `/initialpose` 后应重新收敛；
   - ③ `isLocalizationValid` 通过后，日志应见 `IDLE→WAITING_LOCALIZE→NAVIGATING`，车辆平稳起步巡航，不再卡 IDLE、不再刷 `Robot is out of bounds`；
   - ④ `ros2 param get /controller_server FollowPath.plugin` 为 MPPI；低速前进转向平滑、无满舵振荡。
13. 遗留风险（本次未改，需现场验证）：`maps/hunter_map.pcd` 必须先验存在（与 .yaml 同名）否则 NDT 无目标图，`hunter_relocalization` 会明确报“全局地图加载失败”并置大协方差使门控不开；NDT 对纯长走廊/低特征环境可能退化，需现场调 `ndt_resolution/target_voxel_size/fitness_max`；感知对大面积低矮墙漏检（V0.0.91 遗留）仍为独立专项；`pcl::makeShared`/PCL 版本兼容若编译报错改 `boost::make_shared`。



## V0.0.94

Date：2026-9-22
版本主要有以下功能（**“开启自主巡航车辆原地不动”残余故障链彻底修复**——V0.0.93 方案A 上线后实车仍原地不动，逐层剥洋葱式暴露出 8 处缺陷，涉及启动即崩、自死锁、TF 陈旧、清图类型不匹配、收敛门控冻结死循环）：

背景：车端 `hunter_full.launch.py use_autonomous_nav:=true autonomous_nav_mode:=nav` 启动后车辆始终原地不动。日志随每层修复逐层前进，最终定位到 8 个连锁缺陷（前 6 层为“启动即崩/定位断裂”，第 7–8 层为“定位健康但行驶即停”）：

1. **`nav2_params.yaml` YAML 语法崩溃**（`hunter_bringup/config/nav2_params.yaml`）：第 81 行 `ConstraintCritic: enabled: true` 单行双冒号为非法 YAML，`ros2 --params-file` 解析报 `Error parsing a event near line 81`，导致 controller/planner/behavior/bt_navigator/velocity_smoother **全部 exit -6 崩溃**。拆为两行（`ConstraintCritic:` 换行 `  enabled: true`）。
2. **重定位 Eigen 越界断言崩溃**（`hunter_relocalization/src/ndt_relocalization_node.cpp`）：从 4×4 齐次矩阵取旋转块误写为 `block<3,3>(0,3)`（列偏移 3、宽 3 → 3+3>4 越界），触发 Eigen `Block.h` 静态断言、节点启动即 `exit -6`。跳变闸门/`sendTf`/`publishPose` 三处统一改为 `block<3,3>(0,0)`。
3. **`/relocalization/pose` QoS DURABILITY 不兼容致定位断流**（同上）：发布端原为默认 `VOLATILE`，而 `auto_mission`/`safety_guard` 订阅端为 `TRANSIENT_LOCAL`（要求晚加入者仍能收到最后位姿）→ QoS 不兼容、订阅收不到 → 健康闸门恒判“未收敛”。发布端改 `rclcpp::QoS(KeepLast(5)).transient_local().reliable()`（向后兼容 VOLATILE 订阅者）。
4. **无数据期 `map` 系永缺**（同上）：`alignOnce` 的 `target_==null` 与 `src->size()<50`（数据未就绪）两个提前返回分支只 `publishPose` 不 `sendTf` → 传感器到位前 `map→odom` 从不广播 → `map` 坐标系在 TF 树中始终不存在（`tf2_echo map odom` 报 `frame does not exist`）、costmap/controller 全链等待。两分支补 `sendTf(keep, stamp)` 保活（位姿取当前缓存 `T_map_odom_`）。
5. **收敛分支 `std::mutex` 自死锁**（同上）：收敛分支在 `std::lock_guard lk(t_mtx_)` 持锁作用域内调用 `publishPose`，而 `publishPose` 内部再次 `lock(t_mtx_)`——`std::mutex` 非递归，同线程二次加锁 → **永久阻塞**（表现为节点 CPU 0.0% 假活、只打一条收敛日志、`map→odom` 仅发一次即停更、退出需 SIGKILL）。将共享状态写入（`T_map_odom_`/`has_prev_`）用 `{}` 收窄到极短持锁作用域，`sendTf`/`publishPose` 全部移到锁外。
6. **清图客户端服务类型不匹配使 goal 永不下发**（`auto_mission`）：清图客户端误声明为 `rclcpp::Client<std_srvs::srv::Empty>`，但 Nav2 `clear_entirely_global/local_costmap` 服务**主类型是 `nav2_msgs/srv/ClearEntireCostmap`**（`std_srvs/Empty` 仅序列化兼容副类型）。在 **CycloneDDS** 下 `service_is_ready()` 的 graph 类型匹配只认主类型 → 恒返回 false → 清图门控永久 `等待清图服务就绪（global=PEND local=PEND）`、`costmaps_clear_pending_` 永不清零、**goal 永不发送**。客户端类型全部改用原生 `nav2_msgs::srv::ClearEntireCostmap`（其 `Request` 为空，行为不变），并 `include nav2_msgs/srv/clear_entire_costmap.hpp`。
7. **`map→odom` TF 陈旧致 controller 外推失败反复 Abort**（`hunter_relocalization/src/ndt_relocalization_node.cpp`）：`map→odom` 此前仅在 `alignOnce`（NDT 重计算，2Hz/500ms）里广播，而 `odom→base_link` 高频；controller 以当前时刻位姿查 `odom→map` 时最新 `map→odom` 样本最多滞后 500ms → tf2 报 `Lookup would require extrapolation into the future ... from frame [odom] to frame [map]` → `Unable to transform robot pose` → `follow_path Aborting handle` 反复中止（goal 能接受却走几步即 Abort）。**将 TF 广播与 NDT 解耦**：新增 20Hz `tf_timer_` → `broadcastTfOnly()`，短暂持锁读缓存 `T_map_odom_` 后在锁外 `sendTf` 高频持续广播（NDT 仍 2Hz 精炼值），使下游恒能取到新鲜 TF。
8. **收敛门控重构：位姿跟踪与对外置信度解耦**（同上 + `relocalization_params.yaml`）：V0.0.93 原逻辑把“是否刷新 `map→odom`”与“`fit<=fitness_max`”绑死——车辆一动、2s 滚动窗口点云畸变抬高 `fit`（实测起步 ~1.03、行驶中 ~2.1），一旦越阈值就**冻结位姿不更新** → 车继续动则局部云相对被冻的 map 系越走越偏 → `fit` 更高（曾锁死在 ~2.1/4.4 永不回弹）→ **死亡螺旋**；且对单帧抖动零容忍造成“进 `NAVIGATING`→秒退 `IDLE`”启停抖振。重构为：**位姿刷新只看跳变闸门 `step_ok` 且 `fit<=fitness_hard_ceiling`（默认 3.0）**——运动畸变帧照样跟踪、可自动回弹重收敛；**对外置信度（协方差，供 `auto_mission` 门控）才用 `fit<=fitness_max`** 并加**连续失败去抖** `diverge_tolerance_cycles`（默认 5，2Hz 下 ≈2.5s，其间仍报收敛、持续刷新位姿），严重失准（`fit>ceiling`）或跳变才立即宣告失效、绝不在错误位姿上虚假收敛而行驶。阈值随实测调参：`fitness_max 1.0→1.5→2.5`。新增参数 `diverge_tolerance_cycles`、`fitness_hard_ceiling`。
9. 文档（均为 文档版本 V1.5→V1.6、基线 V0.0.93→V0.0.94，头部与脚注同步）：README（§7.6 TF 表补 `map→odom` 20Hz 保活与跟踪/置信度解耦、§10.2 AUTO 门控表与§10.7 重规划条补清图客户端类型 `nav2_msgs/ClearEntireCostmap` 与收敛门控解耦去抖、文档索引表、脚注）；User_Manual（新增 §4.5.3.1“起步即停/goal 不下发/反复 Abort”排查表 + safety_guard 按停非故障提示、§5.5 TF 自检补 `tf2_echo map odom`≈20Hz、脚注）；Deployment_Guide（新增“V1.6 相对 V1.5 主要变化”块、参数核对表 `relocalization_params.yaml` 行补 `fitness_max:2.5/fitness_hard_ceiling:3.0/diverge_tolerance_cycles:5`、§7.5 排障表新增启动即崩/定位断流假活/清图 PEND/外推 Abort/收敛冻结于 5 条、将遗留 AMCL 重定位注意事项更新为 NDT、**修正脚注长期停留在 V1.4/V0.0.92 的版本漂移**）同步 V0.0.94。
10. 部署动作：`colcon build --packages-select hunter_relocalization auto_mission hunter_bringup`（前两者为 C++ 代码变更，必须重编；hunter_bringup 为 YAML 配置）。⚠ 车端经 Windows 同步的文件 mtime 可能旧于二进制导致 colcon 增量跳过、静默不重编——务必 `rm -rf build/<pkg> install/<pkg>` 强制干净重编，并核对二进制 mtime 新于源码。
11. 验证链：
   - ① Nav2 各节点不再 exit -6、`ros2 launch` 无 `Error parsing ... line 81`；`ros2 node list` 有 `hunter_relocalization` 且进程稳定（`top` CPU 非 0% 假活）；
   - ② `ros2 run tf2_ros tf2_echo map odom` 有连续 `map→odom`（约 20Hz），日志有 `✓ 收敛 fit=…`；起步行驶中 `fit` 抬升后能**回弹**、`map→odom` 位置随车变化不再冻结；
   - ③ `ros2 service list -t` 下清图门控通过：日志见 `[清图] 已请求清除 …`、不再刷 `global=PEND local=PEND`，随后 `[Nav2] goal 已被接受`；
   - ④ controller 不再报 `extrapolation into the future`/`Aborting handle`；`ros2 topic hz /cmd_vel_nav ≈ 20Hz` 且有非零 `linear.x`、`map→odom` 随实际行驶推进。
12. 遗留项（**非代码，属环境/安全调参**，本版本末实车验证暴露）：以上全部修复后车辆已能被正常驱动、稳定定位收敛、`/cmd_vel_nav` 持续输出，但 `safety_guard` 会因**车头正前方约 1m、侧偏 0.35m 处障碍**（走廊净空 ≈ 急停距离 1.00m）持续 `COLLISION_STOP`/限速 ~0.005m/s 而按停前进 → 航点超时/ABORT。此为安全层正确履职：需清除起点正前方障碍或移车到前方 ≥2m 空旷处再巡航；若为自车回波/假障碍或包络过保守，则需在感知侧加自车区域排除或下调 `safety_guard.stop_dist/slow_dist`（安全调参需现场确认后另行处理）。此外 `realsense` 相机启动失败（`No such device`）致 `vision_perception` 彩色图超时、`health_monitor` 报 camera 0Hz（`health=WARNING`）——WARNING 不拦 AUTO 门控（仅 CRITICAL 拦），属独立硬件问题。


## V0.0.95

Date：2026-9-22
版本主要有以下功能（**“遇到障碍物无法绕开障碍物自动驾驶 / 开启自主巡航车辆原地抖动不前进”分层修复**——V0.0.94 收尾时实车日志暴露的最后一段残余故障链，按“任务层 → 安全层 → 行为层 → 感知层”四层定位并修复）：

背景：`ros2 launch hunter_bringup hunter_full.launch.py use_autonomous_nav:=true autonomous_nav_mode:=nav`（日志 `/tmp/hunt7.log`）。V0.0.94 已使系统“能被驱动、定位收敛、`/cmd_vel_nav` 持续输出”，但车辆进入 AUTO 后**始终原地不前**，且 `/safety/state` 在 `SLOWDOWN↔COLLISION_STOP` 间高频往返。日志证据链：

1. `auto_mission` `[IDLE→NAVIGATING]` 后 `[sendNextWaypoint] 发送航点[0] 起点 → (0.00, 0.00, yaw=0.00 rad)`，而 bt_navigator 报 `Begin navigating from current location (-0.16, 0.11) to (0.00, 0.00)` —— **goal 与车辆当前位姿仅差 0.19m（“目标≈自身”的退化 goal）**；
2. `hunter_base_node` 连续打印 `set steering angle: ±0.386428`（打满后恒定）——该值即 safety_guard 阿克曼曲率钳制上限（`|w|≤|v|/R_min=1.9m`）对应的最大内轮转角，说明 **MPPI 在持续请求大于阿克曼极限的转向**（目标含终止朝向，阿克曼无法原地转向）；
3. `safety_guard` 在 56s 内 **47 次** `SLOWDOWN↔COLLISION_STOP` 状态往返，净空始终在 0.991~1.006m（≈急停距离 1.00m ± 6mm）——**净空贴着阈值抖动而判据进/出同阈值、无滞环**，速度被反复归零（车“抖动但不前进”）；
4. `controller_server` 约 10s 一轮 `Failed to make progress` → `[follow_path] Aborting handle` → `behavior_server Running wait`（行为树 V0.0.89 起恢复池**只有清图+Wait 非运动手段**）→ 清图重试 → 再次无进展，形成静默循环，车辆零位移；
5. `realsense2_camera` 启动即 `xioctl(VIDIOC_QBUF) failed Last Error: No such device` → `Failed to resolve the request: Z16 848x480`，`vision_perception` 连续 `彩色图像超时`（最长 55.4s）、`health_monitor` 每 2s 报 `camera 话题频率异常：0.0Hz` —— 视觉避障链整段退化，仅剩 `/scan` 单源。


修复清单（按层）：

1. **任务层：航点“已到达”预检（新增 `already_reached_dist` 0.30m）**（`auto_mission/src/auto_mission_node.cpp`、`include/auto_mission/auto_mission_node.hpp`）：`sendNextWaypoint()` 发送前用 `/relocalization/pose`（map 系位姿）预检——航点与车辆位置重合（≤0.30m）即视为已完成并**跳过不发 goal**（只判位置、不判朝向：朝向对齐依赖原地转向，阿克曼不可行，强判会重新引入死锁；取值须 > 阿克曼停车精度且 ≥2× Nav2 `xy_goal_tolerance`）。效果：航点[0]“起点”不再产生退化 goal，任务直接推进到航点[1]，由 Smac + costmap 对真实障碍规划绕行。保护：**整圈航点全部不可用（全部与车位重合/越界）→ `FAULT` 锁存**并给出处置指引（原实现回 IDLE，而 IDLE 下一拍又重发同一批不可用航点形成静默抖动）。
2. **任务层：受阻（stall）检测（新增 `stall_detect_time` 25s / `stall_move_eps` 0.15m）**：`NAVIGATING` 中 goal 在途而窗口内 map 系位移 <0.15m → 判“**前方障碍无法绕行 / 航点无可达路径**”，立即取消本 goal 并按失败换点（连续 3 次 → `FAULT` 锁存），日志明确给出结论与处置指引；`goal_timeout` 90→45s（受阻已由 stall 提前定性，超时只兜底“缓慢但可达”）。取值约束：须 > 一轮 Nav2 恢复周期（ProgressChecker 10s + 清图 + 受限倒车脱困），过小会打断倒车脱困重试。
3. **任务层：主动取消的 CANCELED 不再重复计失败（新增 `goal_cancel_by_mission_` 标记）**：受阻/超时路径先 `cancelCurrentGoal()` 并自行 `wp_fail_count_++` + 换点，随后 action 返回的 `CANCELED` 结果会使 `resultCallback` **再计一次失败并再跳一个航点**（一次受阻连跳两个航点、提前触发 FAULT）。已在主动取消时打标，`resultCallback` 收到 CANCELED 直接返回。
4. **安全层：阈值释放滞环（新增 `stop_release_hysteresis` 0.25m / `slow_release_hysteresis` 0.20m）**（`hunter_safety/src/safety_guard.cpp` + `hunter_bringup/launch/hunter_autonomous_nav.launch.py`）：碰撞闸进入用 `stop_dist/slow_dist`，**退出用 +hysteresis**（急停后须净空恢复到 1.25m、减速后恢复到 2.0m 才放行），状态转移日志附带“滞环保持（释放阈值 X.XXm）”。物理含义：一次急停后必须等出足够余量再走，杜绝“贴着阈值 0.999↔1.006m 每 0.1~0.2s 抖一次”的蹭行（56s 内 47 次状态往返、速度反复归零的根因）。
5. **安全层：幽灵点门控（新增 `min_obstacle_points` 3 / `obstacle_cluster_span` 0.25m）**：`corridorClearance()` 改两遍扫描——先取走廊内最小纵向净空，再统计该最近回波纵向 ±0.25m 跨度内的回波点数；**点数 <3 判为孤立噪点（单束噪声/玻璃反光/雨雾），不作为刹车依据**（真障碍在 16 线雷达同一纵向跨度内必有数点以上回波），并以 2s 节流 WARN 输出“走廊内最近回波 x=…（侧偏 …）仅 N 点”诊断。可调：极细立柱场景可降为 2，设 1 即恢复旧行为（不抑制）。
6. **行为层：恢复受限倒车脱困 `BackUp(0.45m @ 0.10m/s, time_allowance 10s)`**（`hunter_bringup/behavior_trees/autonomous_navigate.xml`）：`FollowPath` 失败恢复序列由“清局部代价地图 + Wait(2s)”升级为“清局部代价地图 + **BackUp(0.45m)** + Wait(1s)”。依据：①“阿克曼 + 车头正对障碍”时纯非运动恢复永远无法脱困（本次实车即此情形）；② V0.0.89 移除 BackUp 的顾虑（倒进致命栅格 + AMCL 位姿跳变）已不成立——V0.0.93 起全局定位为 NDT 点云重配准（不依赖倒车运动模型），且 safety_guard 在指令为负时**自动切换后方走廊判据**（`footprint_rear 0.37+0.12=0.49m` 起算，后净空 <1.0m 即零速），外加地图边界监护，倒车有双重物理兜底；③ 倒车后 1Hz 重规划重新给出绕行路径（costmap obstacle_layer 已标真实障碍）。`Spin`（原地旋转）保持移除（阿克曼不能原地转向）。
7. **行为层：放开 `velocity_smoother` 倒车通道（`min_velocity[0]` 0.0 → −0.20）**（`hunter_bringup/config/nav2_params.yaml`）：该节点对**全部**速度源做绝对值钳制（`std::clamp(…, min_velocities_[0], …)`），`0.0` 会把 BackUp 的负线速度直接钳成 0——表现为“恢复行为报成功但车不倒”，V0.0.94 及以前因此彻底失去倒车能力。禁倒车仍由控制器/规划层保证（MPPI `vx_min: 0.0` + Smac `allow_reversing: false`），−0.20 上限仅对行为层倒车通道生效。
8. **安全层 ↔ 行为层参数强耦合校验说明**：`stop_release_hysteresis`(0.25m) **必须 < `backup_dist`(0.45m)**（余量 0.20m），否则“倒完净空仍不足以放行”→ 车辆被滞环锁死原地。两值分别位于 `hunter_autonomous_nav.launch.py` 与 `autonomous_navigate.xml`，改动任一必须同步核算（文档已标注）。
9. **感知层：D435 启动即深度流失败修复**（`hunter_bringup/launch/hunter_full.launch.py`）：显式下发 `depth_module.depth_profile` / `depth_module.infra_profile` / `rgb_camera.color_profile = 640,480,30`（与 `sensor_params.yaml` camera.resolution 对齐，USB 带宽更稳）与 `depth_format: Z16`、`color_format: RGB8`，关闭 infra 三路与 `enable_sync`，并保留 `initial_reset` 开关（默认 false；设备枚举异常/`/dev/video*` 消失时置 true 强制复位重枚举）。设计依据：默认 profile（848x480x30）驱动会先落参再“停传感器→重开”，**重开瞬间 USB 节点消失（ENODEV）**——显式声明目标 profile 可避免这次 stop/start 重配；仍复现时按硬件链排查（直连 USB3、勿经 HUB、`dmesg` 查掉线、关 USB 自动挂起）。
10. 文档（均为文档版本 V1.6→V1.7、基线 V0.0.94→V0.0.95，头部与脚注同步）：README（版本头、§7.5 巡航前置检查、§10.3 状态机新增“已到达/受阻”迁移、§10.4 参数表新增 3 项与 `goal_timeout` 45s、§10.6 BackUp 恢复说明、§10.8 防护表与“绕障四层协同”、§13.8 新增 5 条排障、脚注）；User_Manual（版本头、§4.5.2 低速档/防线表改为“四层防线”、新增 §4.5.3.2 六步排查表与航点布置约束、§6.2 新增相机 0Hz/原地抖动/受阻/倒车不动 4 条、脚注）；Deployment_Guide（新增“V1.7 相对 V1.6 主要变化”块、§5.5 参数核对表更新 `velocity_smoother.min_velocity`/safety_guard 新参数/auto_mission 新参数、§5.8 增量升级表补 V0.0.94/V0.0.95 行、§6.6 防护表修正旧阈值漂移（0.6/1.2→1.0/1.8 与 0.8→0.5 硬限）并补滞环/幽灵点门控、§7.4 新增相机 0Hz 条、§7.5 新增 4 条 V0.0.95 排障、§8.5 安全要点更新、脚注）。
11. 部署动作：`colcon build --packages-select hunter_safety auto_mission hunter_bringup`（前两者 C++、后者 YAML/BT/launch；⚠ 车端经 Windows 同步的文件 mtime 可能旧于二进制导致 colcon 增量静默跳过，务必 `rm -rf build/hunter_safety build/auto_mission build/hunter_bringup install/hunter_safety install/auto_mission install/hunter_bringup` 强制干净重编，并核对二进制 mtime 新于源码）。
12. 验证链：
   - ① safety_guard 启动日志含 `释放滞环=+0.25/+0.20 m` 与 `幽灵点门控=≥3 点/±0.25m`；`/safety/state` 不再出现 0.1~0.2s 级 `SLOWDOWN↔COLLISION_STOP` 往返；
   - ② 车辆在起点（航点与车位重合）切 AUTO：日志应见 `[sendNextWaypoint] 航点[0]…已在到达半径内（距当前位姿 0.19m ≤ 0.30m…）视为已完成并跳过` → 改发航点[1]，不再发“目标=自身”的 goal；
   - ③ 车头正对障碍：`FollowPath` 失败后应出现 `Running backup`，`/cmd_vel` 的 `linear.x` 为负、`/chassis/state.velocity` 为负，倒车 0.45m 后 `/safety/state` 从 `COLLISION_STOP（滞环保持）`释放并重新规划绕行；若日志报倒车成功但车不动 → 核查 `velocity_smoother.min_velocity[0]` 是否仍为 0.0；
   - ④ 障碍无法绕行时：25s 内位移 <0.15m → 日志 `[NAVIGATING] 航点[i] 受阻…`，取消 goal 换点；连续 3 次 → `FAULT` 锁存并打印处置指引（不再无声抖动 90s）；
   - ⑤ `ros2 topic hz /camera/camera/color/image_raw` ≈30Hz、`/perception/vision_objects` ≈15Hz，`彩色图像超时` 与 `camera 话题频率异常` 告警消失；
   - ⑥ 日志分析前置过滤：`grep -v 'set steering angle'`（底盘 vendor 驱动在 `/cmd_vel` 回调未节流打印转向角，约 90% 行数为噪声，属 vendor 目录（git-ignored），本次不改）。
13. 遗留项：① 若起点正前方障碍为**真实**固定物且通道宽度 < 车体包络（实测净空 ~1.0m、侧向仅 ±0.45m），倒车脱困后仍无绕行空间 → 任务按“受阻”换点/`FAULT`，属安全层正确履职（需移车/清障，**勿调低 `stop_dist`**）；② data_agent 仍连接占位 broker `platform.example.com:9093`（DNS 无法解析），每 1~2s 报 Kafka 错误刷屏——不影响驾驶，属部署遗留（Deployment_Guide §7.6）；③ 底盘驱动 `set steering angle` 日志刷屏属 vendor 代码（`src/hunter_ros2`，git-ignored），本次以日志过滤方式规避，建议后续由 vendor 侧节流。

## V0.0.96

Date：2026-9-22
版本主要有以下功能（**“进入自动驾驶后行驶一小段路立即停下、不再继续漫游”根因修复**——V0.0.95 四项修复已实车生效，暴露出最后一段连锁缺陷：**航点落在静态地图障碍的膨胀区内 → Nav2 起点被判致命 → 全线规划失败 → 恢复池无运动行为 → FAULT 锁存**）：

**一、V0.0.95 修复的实车验证（13:22 日志，已确认生效）**：
- `safety_guard` 启动日志出现 `释放滞环=+0.25/+0.20 m`、`幽灵点门控=≥3 点/±0.25m` → 新参数已注入；
- `[sendNextWaypoint] 航点[0]起点 (0.00, 0.00) 已在到达半径内（距当前位姿 0.03m ≤ 0.30m，朝向差 0.00 rad），视为已完成并跳过` + `已跳过 1 个不可用航点，改发航点[1] 检查点A` → “目标=自身”死锁消除；
- `bt_navigator: Begin navigating from current location (-0.02,-0.02) to (5.00, 0.00)` 且 `set steering angle` 全程为 ±0.03rad 级平滑值（旧版恒为 ±0.386428 满舵）→ **车辆自主行驶约 5m**，无 SLOWDOWN↔COLLISION_STOP 抖振（旧版 56s 内 47 次）。

**二、残余故障链（本次根因，逐条附日志证据）**：
1. 车到航点 A 附近后，`planner_server` 持续报
   `GridBased: failed to create plan, invalid use: Starting point in lethal space! Cannot create feasible plan..`
   该文案来自 `nav2_smac_planner/src/a_star.cpp:219` `areInputsValid()`——由 `GridCollisionChecker` 判**起点格**无效：代价为 `LETHAL_OBSTACLE(254)`、`INSCRIBED_INFLATED_OBSTACLE(253)`，或 `NO_INFORMATION(255)` 且 `allow_unknown=false`（本工程 `track_unknown_space: true` + Smac `allow_unknown: false`）。
2. 该失败在 **10 次 `Received request to clear entirely the global_costmap` 之后依旧存在** → 致命格**不是动态障碍层标记**，而是来自 `static_layer`：本 fork `StaticLayer::reset()` 只做 `has_updated_data_ = true; current_ = false;`（地图保留，下一次 update 由静态地图重新盖章）；只有障碍层的动态标记会被清掉。**“清图”对静态障碍无效**。
3. 同时 `safety_guard` 全程**无** `SLOWDOWN`/`COLLISION_STOP`（走廊净空始终 ≥1.8m，即车前方 ±0.45m 走廊内无实时障碍），也**无** `MAP_EDGE_STOP`（地图边界监护未介入 → 车不在未建图/界外区域）⇒ 排除“真实近距障碍”与“未建图区”，结论：**航点 A(5.00, 0.00) 落在静态地图某障碍的膨胀区（<0.32m 内切 / <0.40m 膨胀）内**——仓库示例航点是占位值（`autonomous_nav_params.yaml` 已注明），与实车地图不匹配。
4. **车为什么能一路开到那里**？Nav2 局部代价地图为滚动窗口（odom 系）且插件仅 `obstacle_layer + inflation_layer`、**不含 static_layer** → MPPI 只能避开“实时感知到”的障碍，无法避开“仅存在于静态 PGM 地图中”的障碍；而 Smac 侧的**目标严格校验被 tolerance(0.25m) 旁路**（`getToleranceHeuristic() < 0.001` 才校验目标），于是规划“成功”并把车带进膨胀区。
5. **到达后为什么彻底卡死**？行为树恢复池（V0.0.89 起）只剩“清图 + Wait”两种**非运动**手段：清图无效（见②），等待无效（起点有效性不随时间变化）→ 唯一出路是**物理驶离膨胀区**，而恢复池里没有任何运动行为（`Running wait` 循环，全程无 `Running backup`）。
6. **次生问题**：车实际已停在距目标 **0.07m** 处（已进入 Nav2 `xy_goal_tolerance: 0.10`），但因行为树要求“先 `ComputePathToPose` 成功才会执行 `FollowPath`”而被整树 ABORT（`Goal failed`）→ auto_mission 记为**一次航点失败**（`fail_count=1/3`）；连续航点（5,3）、(0,3) 因同样的“起点致命”而 ABORT（`2/3`、`3/3`）→ `FAULT` 锁存，任务静默终止——用户看到的“行驶一小段路立即停下来，不再继续漫游”。
**三、修复清单（按层，全部为“让死局不再发生 + 一旦发生能自行脱困 + 失败可判读”）**：
1. **任务层：航点占据栅格 + 净空（膨胀）校验**（`auto_mission/src/auto_mission_node.cpp` 的 `waypointMapCheckDetail()`，新增参数 `waypoint_clearance_m` 默认 0.50m）：
   - 旧版只校验 `-1`（未建图），**占据栅格(100)竟被放行**——这是本次“把车开进静态障碍”的直接缺口；现补 `occ >= 50` 判据；
   - 新增**净空校验**：`nearestObstacleClearance()` 在搜索半径内求航点到最近占据栅格的欧氏距离，`< waypoint_clearance_m`(0.50m) 即拒绝，日志给出实测值（如“距静态地图障碍仅 0.18m < 净空要求 0.50m（处于 Nav2 膨胀/致命区内）”）；取值必须 > Nav2 内切半径 0.32m，否则航点仍会落在膨胀区内；
   - 全部航点被拒 → `FAULT` 锁存并要求重标航点（沿用 V0.0.95 逻辑）。
2. **任务层：当前位姿净空告警**（`sendNextWaypoint()`）：发 goal 时若车辆距静态障碍 `< waypoint_clearance_m`，立即 WARN 说明“处于膨胀区内，规划可能报 Starting point in lethal space，行为树将尝试倒车脱困”，把“规划器报错”与“车在哪”直接对应起来。
3. **任务层：受阻判据由“位移标量”改“朝目标推进量”**（`goal_start_dist_`）：`progress = 发 goal 时到航点距离 − 当前距离`；行为树脱困倒车会增大到目标距离（progress 变负）仍判受阻，而旧的位移标量会被“后退”骗过（倒车 3m 也算“有位移”而漏判）。日志同时打印起始/当前距离与推进量。
4. **任务层：ABORT 但车已在到达半径内 ⇒ 判为完成**（`resultCallback()`）：对应故障链第 6 条——车已到 0.07m 却因“起点致命”导致整树 ABORT；现以与“已到达”预检同源的 `already_reached_dist`(0.30m) 判据把这种“事实已到达”判为完成、不计失败并推进下一航点，避免 `fail_count` 迅速达 3 而 FAULT 静默终止任务。
5. **行为层：恢复池首位加受限倒车（关键）**（`hunter_bringup/behavior_trees/autonomous_navigate.xml`）：`RecoveryFallback` 的 `RoundRobin` 由 `[清全局图, 清局部图, Wait(3)]` 改为 **`[BackUp(0.45m@0.10m/s, allowance 6s), 清全局图, 清局部图, Wait(2)]`**——把“物理驶离”放在第一个恢复动作：规划失败（起点致命）时 1~2s 内即倒车 0.45m（> 内切半径 0.32m，足以驶离膨胀区），随后 1Hz 重规划即可成功。安全兜底同 V0.0.95：倒车时 `safety_guard` 按**后方走廊**判据（后净空 <1.0m 即零速）+ 地图边界监护；`BackUp` 受限，失败也只损失约 1 个恢复周期。V0.0.95 在 `FollowPath` 失败序列中的 BackUp 保留（两条失败路径都可脱困）。
6. **感知层：相机 `initial_reset` 由 false → true**（`hunter_full.launch.py`）：13:22 日志**仍稳定复现** `xioctl(VIDIOC_QBUF) failed: No such device`（本次为 `Z16 640x480`，即 V0.0.95 的 profile 修复已生效、掉节点位置不变）→ 表明是**设备/USB 链路级**问题：开机先硬件复位可清掉设备残留/半初始化状态（代价仅启动多 2~4 秒；稳定后可改回 false）。若置 true 仍复现，必须按 Deployment_Guide §7.3 走硬件排查（直连 USB3、勿经 HUB、`dmesg` 查掉线、关 USB 自动挂起）——该故障独立于导航（`health=WARNING` 不拦 AUTO 门控），但会使视觉避障退化为单雷达源。
7. 配置注释与文档同步：`nav2_params.yaml` 局部代价地图段补“**rolling 局部代价地图不含 static_layer**”的局限说明与三层防护关系；`autonomous_nav_params.yaml` 补 `waypoint_clearance_m` 与航点标定方法（含“示例航点 (5,0) 已确认落在实车地图障碍膨胀区内，必须重标”的警示）。
8. 文档（均为文档版本 V1.7→V1.8、基线 V0.0.95→V0.0.96，头部与脚注同步）：README（版本头、§10.4 参数表新增 `waypoint_clearance_m` 与受阻判据更新、§10.8/§10「绕障四层协同」补静态层局限与恢复池 BackUp、§13.8 新增 3 条排障、脚注）；User_Manual（版本头、新增 §4.5.3.3“进入 AUTO 后行驶一小段即停（Starting point in lethal space）”排查表、§6.2 补条、航点标定要求、脚注）；Deployment_Guide（新增“V1.8 相对 V1.7 主要变化”块、§5.5 参数核对表补 `waypoint_clearance_m` 与航点净空要求、§5.8 增量升级表补 V0.0.96 行、§7.3 相机 USB 排障细化、§7.5 新增“起点致命/膨胀区”排障、§8.5 安全要点、脚注）。
9. 部署动作：`colcon build --packages-select auto_mission hunter_bringup`（前者 C++、后者 YAML/BT/launch；⚠ 车端经 Windows 同步的文件 mtime 可能旧于二进制导致 colcon 增量静默跳过，务必 `rm -rf build/auto_mission build/hunter_bringup install/auto_mission install/hunter_bringup` 强制干净重编，并核对二进制 mtime 新于源码）。`hunter_safety` 本次无代码变更（V0.0.95 已重编过即可）。
10. 验证链：
   - ① 启动日志应出现 `waypoint_clearance_m` 生效后的航点校验结果：合法航点直接发送；不合法航点被拒并打印实测净空（`距静态地图障碍仅 x.xxm < 净空要求 0.50m`）；
   - ② 若车辆不慎停在膨胀区内：日志出现 `[sendNextWaypoint] ⚠ 当前位姿地图净空仅 x.xxm…`；Nav2 报 `Starting point in lethal space!` 后应能在 1~2s 内看到 `behavior_server: Running backup`，且 `/chassis/state.velocity` 为负、车辆真实后退 0.45m，随后 `Passing new path to controller` 恢复跟踪；
   - ③ 车辆到达航点附近但规划失败时：日志出现 `航点[x] 规划/跟踪失败（ABORTED），但车辆已在其到达半径内（距目标 0.0xm ≤ 0.30m），判为【已到达】并推进下一航点（不计失败）`，不再累积 fail_count；
   - ④ 受阻（无法绕行）时：日志 `[NAVIGATING] 航点[i] 受阻：25s 内向目标仅推进 x.xxm…`（含起始/当前距离），取消 goal 换点；连续 3 次 → FAULT 并给出处置指引；
   - ⑤ 相机：若 `initial_reset:=true` 后 `ros2 topic hz /camera/camera/color/image_raw` 恢复 ≈30Hz 则问题解决；仍 0Hz 且 `dmesg` 见 USB 掉线 → 按 Deployment_Guide §7.3 处理硬件（此时导航仍可运行，但避障退化为单雷达源）。
11. 遗留项：① **仓库示例航点必须在实车地图上重标**（已确认 (5.0,0.0) 落在障碍膨胀区内）；重标前系统会持续拒绝该航点并最终 FAULT，属设计行为；② 若实车地图含“已移走的物体”的幽灵障碍（建图期存在、现已不在），静态层会永久保留 —— 需重跑建图（`autonomous_nav_mode:=mapping`）或在 rviz2 中确认后重制 PGM；本次**未**把 static_layer 加入滚动局部代价地图（StaticLayer 需 map 系非滚动窗口，改动面大），故“局部避障不含静态障碍”是当前架构的已知限制，由航点净空校验 + 全局规划绕行 + 恢复池倒车脱困三层兜底；③ 相机 ENODEV 若 `initial_reset` 无效，属 USB 硬件问题（本次日志与 V0.0.95 掉节点位置一致，profile 修复不是根因）；④ `controller_server: Control loop missed its desired rate of 20.0000Hz` 偶发（MPPI batch 1000 + Jetson 负载），不阻塞行驶，如需消除可下调 `batch_size`/`time_steps`（本次未改，避免引入新变量）。

## V0.0.97

Date：2026-9-22
版本主要有以下功能（**“自动驾驶能走但绕不开障碍物：满舵死磕 → FAULT 锁存，且 FAULT 后车辆仍原地机动 10 分钟”根因修复**——V0.0.96 的“相机可用 + 航点净空校验”已实车生效，本轮日志暴露最后两个瓶颈：① **阿克曼绕障几何死锁**（R_min 1.9m 与碰撞闸 1.0m + 控制器禁倒车的三角矛盾）；② **过期 goal 结果污染任务状态**（主动取消后的旧结果以 ABORTED 而非 CANCELED 返回，被当成新航点的失败 → FAULT 误锁存 + 取消失效）：

**一、V0.0.96 修复的实车验证（13:46 日志，已确认生效）**
- **相机已完全修复**：`initial_reset: 'true'` 生效 —— 日志依次出现 `Resetting device...` → `Open profile: stream_type: Depth(0), Format: Z16, Width: 640, Height: 480, FPS: 30`、`stream_type: Color(0), Format: RGB8, …FPS: 30` → `RealSense Node Is Up!`，`data_agent` 成功订阅 `/camera/camera/color/image_raw`（V0.0.95/96 的“相机全程 0Hz”故障消失；仅剩一条非致命 `Depth stream start failure / Hardware Error` 通知，流已恢复）；
- **航点净空校验生效**：`[sendNextWaypoint] 航点[1]检查点A (5.00, 0.00) 不在已采集地图区域内（距静态地图障碍仅 0.400m < 净空要求 0.50m（处于 Nav2 膨胀/致命区内）…），跳过该航点` —— 仓库占位航点被正确拒绝（不再把车开进死局）；
- **受阻判据生效**：`[NAVIGATING] 航点[0]起点 受阻：25s 内向目标仅推进 0.00m（< 0.15m，起始距离 0.30m / 当前 0.30m）`；
- **恢复池倒车生效**：`behavior_server: Running backup` 多次出现且有 `backup completed successfully`（车辆真实后退，`velocity_smoother.min_velocity[0]=-0.20` 通道有效）。

**二、根因 1：阿克曼绕障几何死锁（“绕不开障碍物”的数学原因）**
1. 现场证据：车被前方真实障碍（走廊净空 0.99~1.47m、回波点 30~69 个连续面）逼停，`set steering angle` **恒为 ±0.386428**（v/R_min 钳出的满舵值），持续 20s 位姿纹丝不动 → `controller_server: Failed to make progress` → 恢复 `BackUp 0.45m` → 又贴回 1.0m → 循环至 FAULT。
2. 几何推导（详见 Deployment_Guide §5.5.1）：阿克曼以最小转弯半径 R 绕开正前方障碍、并在障碍处保持横向净空 `c = 半宽 0.32 + 余量 0.10 = 0.42m` 时，转弯起点到障碍的**提前转向距离**
   `s ≥ √(2·R·c) = √(2×1.9×0.42) ≈ 1.33 m`（等价判据：障碍到该圆弧的最短距离 `√(s²+R²) − R ≥ c`）。
   即 **车与障碍距离 <1.33m 时，单圆弧绕障在几何上不可能**，唯一出路是**先倒车退到 ≥1.33m 再打舵**（或走“倒—打舵—进”的多点掉头）。
3. 旧配置三重封死：`safety_guard.stop_dist = 1.0m`（障碍到 1.0m 即零速 → 车永远进不了可转向区）+ MPPI `vx_min = 0.0`（控制器禁倒车，倒车只能靠恢复行为 0.45m，且倒完又被 1.0m 闸按停）+ Smac `motion_model_for_search: DUBIN`（规划路径不含倒车段）。
4. 修复（四层同时放开，缺一不可）：
   - **安全层阈值重标定**（`hunter_autonomous_nav.launch.py`）：`stop_dist 1.0 → 0.90`、`slow_dist 1.8 → 1.40`；`pointcloud_to_laserscan.range_min 0.8 → 0.70`（维持 `stop > range_min+0.15` 盲区一致性：0.70+0.15 = 0.85 < 0.90 ✔）。取值依据：① 前保险杠停车净空 = 0.90 − 0.45 = 0.45m ≫ 0.5m/s 制动距离 ≈0.18m；② `range_min` 不得低于 ~0.65（V0.0.88 实测**车体自反射可达 ~0.6m**，再降会让自反射变成“永久障碍”把车钉死）；③ 0.90m < 1.33m 转向提前量，车终于能进入可转向区；
   - **控制器放开低速倒车**（`nav2_params.yaml`）：MPPI `vx_min 0.0 → -0.20`（`PathAngleCritic` 依 `vx_min<0` 自动由 `Reversing not allowed` 变 `Reversing allowed`，见 `path_angle_critic.cpp:30`）、`PreferForwardCritic.cost_weight 5.0 → 1.5`（原权重把 MPPI 锁在前进解上）、`PathAngleCritic.reverse_penalty_weight 1.0 → 0.5`、新增 `vy_max: 0.0`（不声明时 `ConstraintCritic` 用默认 `vy_max=0.5` 算出反向限速 −0.54，会放过超速倒车）；
   - **规划器放开倒车段**：`motion_model_for_search: DUBIN → REEDS_SHEPP`、`reverse_penalty 2.0 → 1.5`、`change_penalty 0.5 → 0.3`（多点掉头需 2~3 次方向切换，惩罚过大即退回死锁）；同时把 `allow_reversing` 注释更正为 **本 fork 无此参数**（已核对 `smac_planner_hybrid.cpp` 只读 `motion_model_for_search/reverse_penalty/change_penalty`），避免后人误以为改它有效；
   - **同值一致性**：`velocity_smoother.min_velocity[0]` 保持 `-0.20`（与 MPPI `vx_min` 同值；不同值时 MPPI 规划出倒车却被钳成 0，表现为“车原地不动”）。
5. 量化绕障数据（低速档：L=0.65m、R_min=1.9m、半宽 0.32m）：

   | 量 | 公式 | 数值 |
   |---|---|---|
   | 满舵转向角 | δ_max = atan(L/R_min) | atan(0.65/1.9) = **0.330 rad ≈ 18.9°** |
   | 绕障推荐车速 | v_maneuver | **0.20~0.30 m/s**（`slow_dist` 1.40m 内被限速到 ≈0.3m/s） |
   | 对应角速度 | w = v/R_min | 0.25 m/s → **0.132 rad/s**（0.15 m/s → 0.079 rad/s） |
   | 单次倒车行程 | d_back | **0.30~0.45 m**（创造提前量 / 掉头换向） |
   | 提前转向距离 | s ≥ √(2R·c), c=0.42 | **≥ 1.33 m** |
   | 90° 转弯弧长/耗时 | R·π/2 | 2.98 m / 12 s @0.25m/s |
   | 90° 转弯通道宽度需求 | R + 半宽 + 余量 | **≈2.5 m**（<2.5m 必须多点掉头） |
   | 前保险杠停车净空 | stop_dist − 车体前缘 | 0.90 − 0.45 = **0.45 m** |
   | 制动距离 @0.5m/s | v²/(2a)+v·t_lat | 0.083+0.10 = **≈0.18 m** |
   | 绕障机动里程上限 | stall_path_allow_m | **1.0 m** |

**三、根因 2：过期 goal 结果污染任务状态（FAULT 误锁存 + 取消失效）**
1. 现场证据：`[cancelCurrentGoal] 取消当前导航 goal`（受阻换点）后 2ms 内 `[sendNextWaypoint] 发送航点[2]`，紧接着 `[Nav2] 航点[2] 检查点B 导航失败（ABORTED），fail_count=2/3` —— 这是**被取消的旧 goal** 的结果（行为树当时已失败，`bt_navigator` 回传 **ABORTED 而非 CANCELED**）；3s 后同样的过期结果把 `fail_count` 推到 `3/3` → `FAULT` 锁存，**而车其实一直能正常行驶**。
2. 两个后果：① 一次受阻连跳 2 个航点、`fail_count` 迅速达上限 → **误锁存 FAULT**；② `resultCallback` 开头无条件清空 `goal_in_flight_/goal_handle_`，使随后的 `cancelCurrentGoal()` 变**空操作** → 行为树在 FAULT 后仍持续驱动车辆满舵/倒车机动 **约 10 分钟**（日志 1790056004~1790056157：`Failed to make progress` / `Running backup` / `Passing new path` 循环）。
3. 修复（`auto_mission`）：新增 **goal 世代号 `goal_epoch_`** —— `sendNextWaypoint()` 在 `async_send_goal` **之前**自增并把取值绑定进 goal_response/result 回调；两个回调首行比较世代号，**过期结果一律丢弃**（不计失败、不换点、不清句柄）；`cancelCurrentGoal()` 判据由 `goal_handle_ && goal_in_flight_` 放宽为 `goal_handle_` 非空（消除竞态下取消被静默跳过），无句柄时告警留痕。
4. 附带修复：`goalResponseCallback` 原在**持有 `goal_handle_mutex_` 时**调用 `enterFault()` → `cancelCurrentGoal()` 会再次锁同一把非递归 mutex = **自死锁**（现场未触发，一旦“goal 连续被拒收”即挂死）；现改为**先释放锁再处理故障**。

**四、任务层：受阻判据增加“绕障机动宽容”（防把正常多段机动误杀成受阻）**
- 问题：放开倒车 + REEDS_SHEPP 后，正常绕障是“倒 0.4m → 进 0.5m → 再倒”的多段机动，**净推进**可能长期 <0.15m（倒车时到目标距离反而增大）→ 旧判据会误判“受阻”并换点。
- 修复：新增 `stall_path_allow_m`（默认 **1.0m**）与**累计行程**统计（自本 goal 发出起 map 系 `|Δ位姿|` 之和）。受阻判据变为：持续 25s 净推进 <0.15m **且** 累计行程 <1.0m（真的没动）；累计行程 ≥1.0m 则判为“多点机动中”，打印节流告警但**不换点**，兜底交给单航点超时 `goal_timeout`（`autonomous_nav_params.yaml` = 45s，节点默认 60s）。受阻 ERROR 日志同步补上累计行程数值，便于区分“没动”与“在动但绕不过”。
- 另：`already_reached_dist 0.30 → 0.35m` —— 现场车与“起点”航点相距 0.304m，刚过旧阈值 → 下发退化目标后被阻 25s；0.35m 与 Nav2 `xy_goal_tolerance`(0.10m) + R_min(1.9m) 的“可微调尺度”匹配。

**五、感知/算力：相机修好后的“感知链饥饿”处置**
- 现象：`planner_server`/`controller_server` 反复报 `The /perception/lidar/obstacle_cloud observation buffer has not been updated for 0.6~2.8 seconds`、`Message Filter dropping message: frame 'base_link' … earlier than all the data in the transform cache`、`Control loop missed its desired rate of 20.0000Hz` —— 典型 CPU/IO 饥饿（相机 30fps + vision 15Hz + `data_agent` 同时录制 `/lidar_points`、`/camera/color/image_raw`、`/imu/data`、`/chassis/state`）。
- 处置：相机帧率 **30 → 15fps**（`hunter_full.launch.py`，与 `sensor_params.yaml` 的 camera 声明一致，砍掉约一半采集/编码/录制负载；视觉链本就按 15Hz 设计）；两个 costmap 的 `obstacle_layer` 显式声明 `footprint_clearing_enabled: true`（默认即 true，固化以求可读）——`range_min` 下调后它是“自反射被标成致命栅格 → `Starting point in lethal space`”的关键防线。
- 可选的进一步降载（不影响功能）：下调 MPPI `batch_size`(1000)/`time_steps`(30)，或调整 `data_agent` 录制话题清单（**建议排除 `/lidar_points`**：点云单帧数据量巨大、写盘 IO 压力高）。

**六、部署动作**
```bash
cd ~/HunterEdge
rm -rf build/auto_mission build/hunter_bringup build/hunter_safety \
       install/auto_mission install/hunter_bringup install/hunter_safety
colcon build --packages-select auto_mission hunter_bringup hunter_safety
```
本轮**三个包均有变更**：`auto_mission`（C++：世代号/受阻判据/参数）、`hunter_bringup`（`nav2_params.yaml`、两个 launch、BT 注释）、`hunter_safety`（默认阈值与说明，虽 launch 已显式传值，仍建议重编保持一致）。⚠ 经 Windows 同步的文件 mtime 可能旧于二进制，务必先 `rm -rf` 强制干净重编（Deployment_Guide §5.8）。

**七、验证链**
1. 启动即核对 `safety_guard` 日志：应为 `stop=0.90m, slow=1.40m`（旧 1.00/1.80）且含 `释放滞环=+0.25/+0.20 m`；
2. `ros2 topic echo /scan --field range_min` 应为 `0.7`；**并用 rviz2 看 local_costmap：车体四周 0.4~0.7m 内不应出现自发自收的假障碍**——若出现，说明自反射强于预期，把 `range_min` 回调 0.8 并同步 `stop_dist ≥ 0.95`（盲区一致性会自动校验）；
3. 规划器启动日志应打印 `Using motion model: ReedsShepp`（旧为 `Dubin`）；MPPI 日志应打印 `PathAngleCritic … Reversing allowed.`（旧为 `not allowed`）；
4. 让车正对 1.0~1.5m 处的障碍：预期 ① 先倒车（`/cmd_vel.linear.x<0` 或 `behavior_server: Running backup`）② 随后满舵 + 低速前进绕过 ③ 无 `Failed to make progress` 连发；日志出现 `净推进 x.xxm 但累计行程 x.xxm（多点掉头/倒车绕障中）` 说明绕障机动判据在工作；
5. 受阻场景核对新日志格式：`…累计行程仅 0.xxm（< 1.00m，车基本没动）…`（含累计行程才判受阻）；
6. 任务结束（正常/FAULT）后**车辆必须停住**：`ros2 topic echo /cmd_vel` 应在 0.5s 内归零（`safety_guard` 的 `CMD_TIMEOUT` 兜底），不应再现“FAULT 后仍机动 10 分钟”。

**八、遗留项与风险**
1. **`range_min` 0.70 是硬约束**：低于 ~0.65 会看到自车反射（V0.0.88 实测 ~0.6m），届时 `stop_dist` 会被盲区一致性强制抬升、反而更走不动；下调前必须先在 rviz2 确认 `/scan` 在 0.4~0.7m 无自车回波；
2. **REEDS_SHEPP 的代价**：窄通道内规划可能给出含倒车段的路径，车辆会**主动倒车**（≤0.20m/s）——属预期行为，现场须保证车后 ≥0.5m 无人无障碍（`safety_guard` 后方走廊判据 + 地图边界监护兜底）；若现场不接受主动倒车，可回退 `motion_model_for_search: DUBIN` 并把 MPPI `vx_min` 改回 `0.0`，同时接受“车头正对障碍 1.33m 内无法绕障”的几何限制；
3. **仍未把 static_layer 加入滚动局部代价地图**（架构限制保持）：静态障碍绕行继续依赖全局规划 + 航点净空校验 + 恢复池倒车三层兜底；
4. **仓库示例航点仍为占位值**（(5.0,0.0) 已确认落在实车地图障碍膨胀区内）——实车部署必须重标（四周 ≥0.5m 无占据栅格）；
5. 相机若再现 `VIDIOC_QBUF … ENODEV`，按 Deployment_Guide §7.3 走 USB 硬件排查（`initial_reset` 已默认开）；
6. **C++ 变更（`auto_mission`）本机无 ROS2 编译环境、未实际编译**：需车端 `colcon build` 通过并按上述验证链确认；若编译报错请回传 `colcon` 输出以便修正。

---

## V0.0.98（Date：2026-09-22）

**主题："V0.0.97 放开倒车/重标定阈值后车辆仍无法绕开障碍物"的第二层根因修复——安全层直线走廊碰撞闸与绕障弧线**互斥**（几何死锁解除后，安全层反过来把 MPPI 生成的绕障轨迹按停）+ MPPI 预测时域看不到完整绕障弧 + goal 取消竞态残留 + /scan 观测链断流**。对应文档：Deployment_Guide **V2.0**、README **V2.0**、User_Manual **V2.0**。

**一、故障现象与日志证据**（2026-09-22 自主巡航日志，V0.0.97 基线，5099 行）
1. 车距办公桌类正前方障碍 1.4~1.8m 即触发 `SLOWDOWN`、1.0m 附近触发 `COLLISION_STOP（行进方向走廊净空 X.XXXm < 急停距离）`——**净空按“指令方向的直线投影”计算**：MPPI 已生成含横移分量的绕障弧（`set steering angle` 不再是恒 0.386 的满舵死磕，说明控制器已在工作），但 safety_guard 把该弧线指令当作“直行撞障碍”按停 → **绕障轨迹刚被执行 0.1~0.3s 就被清零**，车在 stop/slow 阈值间往复抖振，25s 净推进 <0.15m → 受阻换点 → 3 次后 `FAULT`；
2. `Goal was canceled by goal client` 后 **2ms** 内即见新 goal 发出，随后新 goal 以 `ABORTED` 返回并计入 `fail_count`——V0.0.97 的世代号只挡了“过期结果污染”，**没挡“新 goal 被旧 BT 失败状态波及”**（behavior_server 取消传播未完成即重发）；
3. `Control loop missed its desired rate of 20.0000Hz`、`The /scan observation buffer has not been updated for 2.x seconds`、`Message Filter dropping message ... earlier than all the data in the transform cache` 高频出现——CPU 饥饿 + TF 外推容差 0.1s 过紧，观测层将 /scan 判过期并整层停更（障碍“冻结”后突然跳出，进一步劣化绕障质量）。

**二、根因分析**（五因叠加，RC1~RC5）
- **RC1（主因，安全层与避障互斥）**：碰撞闸对 `/cmd_vel_pre_safety` 的判据是**纯直线走廊**——把 (v,w) 指令仅按行进方向投影求纵向净空，**完全忽略 w 带来的横向避让分量**。阿克曼绕障（§V0.0.97 几何：提前转向距离 ≥1.33m、绕障弧横移量 c=0.42m）在 0.9~1.4m 距离上**任何绕障弧的起点都落在直线走廊内** → `stop_release_hysteresis` 释放前指令恒被清零 → MPPI 永远得不到执行机会，“能规划、能出弧、就是走不出去”。这正是“V0.0.97 后满舵死磕消失、但车依旧绕不过去”的原因。
- **RC2（控制层时域不足）**：MPPI `time_steps 30 × model_dt 0.05 = 1.5s` 预测时域，@0.5m/s 仅前瞻 0.75m、@绕障实际车速 0.25m/s 仅 **0.38m**——看不到 1.33m 提前转向 + ≈3m 绕障弧的前半段，弧尾代价全部落在时域外 → 采样评分退化为“贴路径直行”，与 CostCritic 权重 3.0（对膨胀区代价梯度不敏感）叠加，控制器即使允许倒车也倾向直行死磕。
- **RC3（critic 权重惩罚绕行）**：`PathAlignCritic 14.0` + `max_path_occupancy_ratio 0.05`——绕障横移使轨迹点偏离全局路径，>5% 轨迹点落入占据格即评分饱和 → 绕行被按“最重档”惩罚，MPPI 宁选贴径直行撞停。
- **RC4（取消竞态残留）**：受阻/超时换点时 `cancelCurrentGoal()` 后**立即** `sendNextWaypoint()`，behavior_server 侧旧 goal 取消尚未收敛（BT 仍处失败状态），新 goal 被连带 `ABORTED` → 计入 fail_count（V0.0.97 世代号已确认该 ABORTED 属新 goal 合法回执，无法用世代号过滤）。
- **RC5（观测链断流）**：相机 15fps + 720 束 /scan + 点云双源在 CPU 饥饿期更新周期 >2s，`expected_update_rate 0.5`（容忍 2s）频繁超时停更；`transform_tolerance 0.1` 过紧使 /scan 投影成片丢帧（`Message Filter dropping`）。

**三、修复一（核心算法）：safety_guard「轨迹扫掠弧（swept-arc）碰撞闸」**（`hunter_safety/src/safety_guard.cpp`）
把碰撞闸从“直线走廊”升级为“**按指令 (v,w) 积分真实阿克曼轨迹 + 车体包络盒逐步扫掠判定**”，输出与直线走廊同量纲的“纵向等价净空”，实现“**绕得开就放行、真撞才拦**”：
1. **轨迹积分**：前 `reaction_lag`（0.4s，指令执行滞后）按直线积分，其后按圆弧 `θ̇ = w_eff = clamp(w, ±v/R_min)`（天然满足阿克曼曲率约束）沿**弧长等步长** darc=5cm 积分（低速不增加步数、恒 ≤60 步/tick）；倒车以 |v| 在航向系反向积分（包络用行进方向前缘/后缘判定）；
2. **包络扫掠**：车体盒（`footprint_* + self_margin`，半宽 0.44m）随积分位姿逐格判断候选障碍点（先按扫掠域粗选），首次接触即得 `d_arc = 接触弧长 + 行进方向车体边缘`——**与直线走廊净空同量纲**，STOP/SLOWDOWN 阈值、滞环、限速插值全部复用不改语义；
3. **双判据切换**：`|v| > vel_trust_eps`(0.03m/s) 时用扫掠弧**替换**直线走廊（不能取两者最小——取小即回到 V0.0.97 死锁）；静止/蠕行时仍用直线走廊（无指令可信度，防抖）；幽灵点门控同弧生效（接触后 0.2s×speed 弧长窗口内累计接触点 <3 判噪点）；
4. **前瞻需求自洽**：弧判据只需回答“沿该弧多久后撞上”，扫掠上限 `s_need = slow_dist + v²/(2·brake_decel) + 0.3 ≤ 3.0m`——恰好覆盖“释放滞环 + 反应期 + 制动距离”，不再虚高；
5. **绕障过程量化数据**（要求3 的直接落实，全部进日志与启动横幅）：满舵 δ_max=atan(0.65/1.9)=**0.330rad(18.9°)**；绕障带速 v=0.20~0.30m/s（SLOWDOWN 线性限速天然落在此带）对应 `w=v/R_min`=0.105~0.158rad/s；反应期 0.4s@0.25m/s 直线段 0.10m；制动距离 0.25²/(2×1.5)≈**0.02m**、0.5m/s 时 0.08m；日志 STOP/SLOWDOWN 措辞改标「扫掠弧净空 / 直线走廊净空」双态。
新增参数（launch 注入，均有默认值）：`reaction_lag: 0.4`、`brake_decel: 1.5`（与 velocity_smoother 一致）、`vel_trust_eps: 0.03`。

**四、修复二：MPPI 预测时域与 critic 重标定**（`hunter_bringup/config/nav2_params.yaml`）
| 参数 | 旧→新 | 理由 |
|---|---|---|
| `time_steps × model_dt` | 30×0.05(1.5s) → **50×0.08(4.0s)** | 时域内前瞻 ≈1.0~2.0m（@0.25~0.5m/s）≥ 提前转向 1.33m + 弧前半段，能看到“打舵绕出去”的代价收益 |
| `batch_size` | 1000 → **700** | 时域 +67% 的算力对冲，20Hz 控制回路实测余量恢复（RC5 降载之一） |
| `wz_std` / `prune_distance` | 0.4→**0.5** / 1.5→**3.0** | 转向采样多样性（绕障弧靠 w 采样分化）；剪枝域对齐弧判据 3.0m 上限 |
| `CostCritic.cost_weight` | 3.0 → **6.0** | 膨胀区代价梯度加倍：同在膨胀区内“净空大的弧”与“贴障碍的弧”被拉开区分度，精确择净空 |
| `PathAlignCritic` | 14 → **8**，`max_path_occupancy_ratio` 0.05→**0.15** | 允许绕障段合理离径（离径罚从饱和降回线性区），全局路径仍是权威 |
| `PreferForwardCritic` | 1.5 → **1.0** | 与 REEDS_SHEPP 倒车绕障进一步协同 |
| local `inflation_radius` | 0.40 → **0.45** | 与扫掠弧包络半宽 0.44m 对齐，膨胀区不再比真实包络更保守/更激进 |

**五、修复三：auto_mission 取消静置门控**（`src/auto_mission`，C++）
新增 `goal_cancel_pending_` 标记 + `goal_cancel_settle_time`（**2.0s**，配置 <0.5 强制回 2.0）：`cancelCurrentGoal()` 置 pending 后，受阻/超时分支**不再立即** `sendNextWaypoint()`；换点重发经 settle 门控放行（旧 goal 的 CANCELED/**ABORTED**（by_mission 取消）结果先到即提前放行，否则 2s 超时兜底），彻底消除 RC4 的“取消未收敛即重发 → 新 goal 被波及 ABORTED → fail_count 误累积 → FAULT”。`resultCallback` 新增 **ABORTED + goal_cancel_by_mission_** 分支：不计失败、不换点、INFO 记录“取消静置完成”。

**六、修复四：/scan 链降载与断流缓解**（`hunter_bringup/launch/hunter_autonomous_nav.launch.py`、`nav2_params.yaml`）
1. `pointcloud_to_laserscan`：`transform_tolerance 0.1→0.3`（消 `Message Filter dropping` 成片丢帧）、`angle_increment 0.5°→1.0°`（720→360 束，CPU 降载；对 0.45m 走廊/0.44m 弧包络的角度分辨率 1.0°@1m ≈17mm 仍充分）；
2. 两个 costmap 全部 4 处观测源 `expected_update_rate 0.5→0.25`（断流容忍 2s→4s，消除饥饿期整层停更的“障碍冻结”）。

**七、部署与重编**
```bash
cd ~/HunterEdge && source /opt/ros/humble/setup.bash
# ⚠ Windows 同步 mtime 陷阱：先强制干净重编本轮 3 个包
rm -rf build/hunter_safety build/auto_mission build/hunter_bringup \
     install/hunter_safety install/auto_mission install/hunter_bringup
colcon build --packages-select hunter_safety auto_mission hunter_bringup
source install/setup.bash   # Ctrl+C 重启 bring-up 生效
```

**八、验证链**
1. safety_guard 启动日志出现 `V0.0.98 轨迹扫掠弧=行进中启用（v>0.03m/s 时按指令(v,w)积分判净空，静止用直线走廊；反应滞后0.4s，制动1.5m/s²，前瞻=slow+v²/2a+0.3m≤3m）`；
2. 正对 1.0~1.5m 障碍：车应**低速（0.2~0.3m/s）连续横移绕过**，不再出现“`SLOWDOWN`→行进 0.2s→`COLLISION_STOP`→后退→贴回”循环；STOP/SLOWDOWN 日志显示净空来源为「扫掠弧净空」（行进中）或「直线走廊净空」（静止）；
3. `ros2 topic echo /scan --field ranges` 束数 ≈360；`Control loop missed` 频率显著下降；`/scan observation buffer has not been updated` 应 >4s 才出现；
4. 换点日志应见 `等待上一 goal 取消静置…`（INFO 节流）而非 2ms 后立即发 goal；`已主动取消的旧 goal 以 ABORTED 回传（不计失败）` 属正常静置完成；
5. 连续巡航 ≥5 个航点无 FAULT（此前日志 1~2 个航点即锁存）。

**九、遗留项与风险**
1. **C++ 变更（`hunter_safety`、`auto_mission`）本机无 ROS2 环境未编译**：车端 `colcon build` 若报错请回传输出修正；
2. 弧判据信任 `/cmd_vel_pre_safety` 指令即时性：底盘速度执行若显著滞后（>0.4s 达速），把 `reaction_lag` 上调至实测值（过大会放松拦截，需现场评估）；
3. `vel_trust_eps 0.03` 以下的蠕行仍走直线走廊——低速绕障启动瞬间可能有一次短暂 STOP-释放往返，属预期；若现场敏感可上调 `stop_release_hysteresis`（须保持 < `backup_dist 0.45`）；
4. 360 束 /scan 对细杆类障碍（≤3cm @3m）检出概率下降，感知侧 `lidar_cloud` 点云源仍是主障碍源，/scan 为安全/清障链路，风险可控；
5. 遗留项沿用 V0.0.97：`range_min ≥ 0.65` 硬约束、REEDS_SHEPP 主动倒车现场安全半径、仓库示例航点占位待重标、局部代价地图不含 static_layer。


## V0.0.99（Date：2026-09-22）

**主题：阿克曼几何参数一致性与安全层配置同步——复核 V0.0.97/V0.0.98「绕障四层协同」全量落地，修正 `safety_guard` 轴距误用（0.65 → 0.46）与独立调试启动文件滞后于生产配置的问题；并**依据官方《HUNTER SE 用户手册 V2.0.2》§1.2 权威参数**（长宽 820×640×310mm、轴距 460mm、轮距 550mm、整备 42kg）校正 URDF 车辆模型、底盘 `robot_model` 与 `HunterSEParams`。对应文档：Deployment_Guide **V2.1**、README **V2.1**、User_Manual **V2.1**。**

**一、绕障四层优化复核（确认 V0.0.97/V0.0.98 已全部落地，无功能变更）**
| 层 | 措施 | 落点（已核对） |
|---|---|---|
| 安全层（几何死锁） | `stop_dist/slow_dist = 0.90/1.40`（<1.33m 提前转向距离）+ 放开倒车 | `safety_guard.cpp` 默认值 + `hunter_autonomous_nav.launch.py` |
| 安全层（判据错位，核心） | 轨迹扫掠弧（swept-arc）：按指令 (v,w) 积分阿克曼轨迹 + 包络盒扫掠判接触，替换直线走廊 | `safety_guard.cpp::sweptArcClearance()` |
| 控制层 | 预测时域 50×0.08=4.0s、`vx_min=-0.20`、`PathAlignCritic 14→8`、`max_path_occupancy_ratio 0.05→0.15`、`PreferForwardCritic 1.5→1.0` | `nav2_params.yaml` |
| 规划层 | `REEDS_SHEPP`（含倒车多点掉头）、`reverse_penalty 2.0→1.5`、`change_penalty 0.5→0.3` | `nav2_params.yaml` `GridBased` |
| 任务层 | 取消静置门控 2.0s + 受阻判据累计行程 `stall_path_allow_m` | `auto_mission_node.cpp` |
| 抖振 | 释放滞环 0.25/0.20 + 局部重规划 1Hz | `safety_guard.cpp` + `autonomous_navigate.xml` `RateController hz=1.0` |

**二、修复一：`safety_guard` 轴距误用（0.65 → 0.46，一致性修正，非功能性）**
- **现象**：`safety_guard.cpp` 与两处 launch 的 `wheelbase` 默认取 **0.65**（`HunterV2Params` 值 / 约等于车长），而本车型为 **HUNTER-SE**：README 硬件表记录轴距 **460 mm**、`decision_making_params.yaml` 亦为 0.46——三处对「轴距」这一物理量取值互相矛盾（Deployment_Guide §2.1 旧「参数差异提示」即记录此冲突）。
- **影响面界定**：曲率钳制 `|w| ≤ |v|/R_min`、MPPI Ackermann 模型、Smac `REEDS_SHEPP` **均只依赖 `R_min=1.9m`、不依赖轴距**，故本次改动**不改变任何规划/控制/安全行为**；轴距仅用于 δ_max 的几何叙述（及底盘侧 `hunter_base` 的指令→转角换算，后者由 `robot_model` 选 `HunterSEParams/HunterV2Params`，与导航层无耦合）。
- **改动**：`safety_guard.cpp` `wheelbase` 默认 0.65→0.46（含头部注释、成员初值、参数注释），`hunter_autonomous_nav.launch.py` 与 `safety_guard.launch.py` 的 `wheelbase` 同步 0.46；δ_max 由 atan(0.65/1.9)=0.330rad(18.9°) 修正为 **atan(0.46/1.9)≈0.238rad(13.6°)**（四份文档同步）。

**三、修复二：独立调试启动 `hunter_safety/launch/safety_guard.launch.py` 与生产同步（关键安全一致性）**
- **问题**：该调试文件仍滞留 V0.0.96 前的 `stop_dist: 1.0 / slow_dist: 1.8 / max_linear_vel: 0.8`，且缺 V0.0.95 滞环/幽灵点与 V0.0.98 扫掠弧参数。**`stop_dist=1.0` 正是 V0.0.96 几何死锁的确切值**——单独 `ros2 launch hunter_safety safety_guard.launch.py` 调试会静默复现「车进不到可转向位置即被按停」。
- **改动**：与生产 `hunter_autonomous_nav.launch.py` 逐项对齐——`stop_dist 0.90`、`slow_dist 1.40`、`max_linear_vel 0.5`、`stop/slow_release_hysteresis 0.25/0.20`、`min_obstacle_points 3`、`obstacle_cluster_span 0.25`、`reaction_lag 0.4`、`brake_decel 1.5`、`vel_trust_eps 0.03`、`map_edge_stop/slow_dist 0.4/1.0`。

**四、依手册校正：URDF 车辆模型 / 底盘 robot_model / HunterSEParams（新增）**
依据附件《HUNTER SE 用户手册 V2.0.2》§1.2 性能参数（权威实车规格）逐项对齐：

| 项 | 修正前 | 修正后（手册） | 文件 |
|---|---|---|---|
| URDF 轴距 | `wheelbase=0.55`（错将轮距当轴距） | **0.46**（轴距 460mm）；新增 `track=0.55`、`max_steer_angle=0.384`（22°） | `hunter_bringup/urdf/hunter_se.urdf.xacro` |
| URDF 质量 | `mass=45.0` | **42.0**（整备重量 42kg） | 同上 |
| 底盘车型选择 | 未传 `robot_model`→回退上游默认 `hunter2`（V2 参数） | 显式 `robot_model='hunter_se'` | `hunter_bringup/launch/hunter_full.launch.py` |
| SE 轴距参数 | `HunterSEParams::wheelbase=0.52` | **0.46**（手册 460mm） | `hunter_base/include/hunter_base/hunter_params.hpp` |

- **手册存长确认**：§1.2 长 820mm → footprint 总长应为 **0.82m**；现有 `[0.45, -0.37]`×半宽 0.32 已与手册一致（总长 0.82 × 总宽 0.64），**无需修改**（曾提“车长 0.65m”与手册不符，经确认保留 0.82m）。
- **影响面**：`hunter_params.hpp::wheelbase` 用于底盘侧 `hunter_base` 的指令↔转角换算与里程计反推，改为 0.46 后与实车一致；`robot_model:=hunter_se` 使底盘走 `HunterSEParams`（track 0.55 / wheelbase 0.46 / 转向限幅）而非 V2；URDF 无轮子关节，`wheelbase/track` 为信息量（不改变 TF 几何），修正主要为文档/模型一致性。

**五、部署与重编**
```bash
cd ~/HunterEdge && source /opt/ros/humble/setup.bash
# 本轮改 hunter_safety 默认参数/launch + hunter_base 头文件 + hunter_bringup URDF/launch：
rm -rf build/hunter_safety install/hunter_safety \
      build/hunter_base   install/hunter_base
colcon build --packages-select hunter_safety hunter_base hunter_bringup
source install/setup.bash   # 重启 bring-up 生效
```

**六、验证链**
1. `safety_guard` 启动横幅 `stop=0.90m, slow=1.40m`（生产与调试两种启动方式一致）；
2. 独立调试启动不再复现满舵死锁；绕障现象同 V0.0.98 §八；
3. `xacro hunter_se.urdf.xacro` 可正常解析（新属性无未定义引用）；`ros2 launch hunter_bringup hunter_full.launch.py` 时 `hunter_base` 日志应显示选用 **hunter_se** 参数集（轴距 0.46）。

**七、说明与遗留**
1. **不改动既有整定**：`stop_dist/slow_dist`、footprint（前缘 0.45/后缘 0.37/半宽 0.32，总长 0.82m 已对应手册 §1.2 外形 820×640）、滞环与 `backup_dist(0.45)` 的强耦合、`range_min 0.70` 下限——均保持 V0.0.97/98 定值，避免触碰已实车验证的安全裕量；
2. 历史版本（V0.0.82/V0.0.97/V0.0.98）正文中 `atan(0.65/1.9)` 等为该发布当时所述车辆参数，保留不改（changelog 忠实性），当前基线以本 V0.0.99 与四份文档为准；
3. ~~底盘 `hunter_base` 的 `robot_model` 默认 `hunter2`~~——**本 V0.0.99 已解决**：经手册确认实车为 HUNTER-SE，已在 `hunter_full.launch.py` 显式传 `robot_model: hunter_se` 并将 `HunterSEParams::wheelbase` 0.52→0.46（见上「四」）。

---

## V0.1.00（Date：2026-09-22）

**主题：实车日志「`[FAULT] 任务已锁存（单航点导航超时连续达上限）……处置后将模式开关离开 AUTO 再切回以重启任务`」的彻底修复——`auto_mission` 任务层「自愈四件套」：① 逐航点失败隔离 + 自动轮转（单点超时不再拖垮整条巡航线）；② 车身四周/脚底「假障碍占位」自动诊断（局部代价地图 × 激光 × 相机 交叉验证，替代人拿 rviz2 看图）；③ 航点可达性按静态地图可通行连通域判定（替代人猜「航点是否可达」）；④ FAULT 由「永久锁存等人工解锁」改为自愈态 + 遥控接管后切回 AUTO 即自动复驶。对应文档：Deployment_Guide **V2.2**、README **V2.2**、User_Manual **V2.2**。**

**一、故障现象与根因（为什么旧版本必须人工解锁）**
1. **失败计数是「跨航点总量」而非「逐航点」**：旧 `wp_fail_count_` 对整条任务累加，A/B/C 三个**不同**航点各失败一次即 `3/3` → `enterFault()`。而事实往往只是「那一个点去不了」——一条巡航线被一个坏航点永久判死。
2. **FAULT 唯一出口是「模式降级分支」**：只有 `behavior.mode != "AUTO"` 的分支才清 `state_`，而车辆故障后仍停在 AUTO 档 → **FAULT 在 AUTO 内永久锁死**，故日志不得不要求人工「离开 AUTO 再切回」。
3. **人工处置清单三项全靠肉眼**：「①查假障碍 ②查定位跳变 ③查航点可达」——节点不给结论，现场只能拿 rviz2 对 `local_costmap` 猜；而这三项**全部可用系统已有话题自动判定**。
4. 诱因层：「单航点导航超时」绝大多数并非「航点客观不可达」，而是**起点被脏局部代价地图占位**（接管期压过旧栅格 / 自反射 / 只标不清的残留）→ `Starting point in lethal space` → 重规划必失败。清图 + 换点即可恢复，不该升格为整任务故障。

**二、需求①：逐航点失败隔离与自动轮转（`auto_mission_node.{hpp,cpp}`）**
- 新增 `struct WaypointRuntime{fail_count, isolated, permanent, isolated_at, last_reason}` 与 `std::vector<WaypointRuntime> wp_rt_`（与 `waypoints_` 同尺寸，`syncWaypointRuntime()` 在加载/热重载后对齐），**删除** `wp_fail_count_`。
- `handleWaypointFailure(reason)` 统一取代旧的「三次即锁存」（受阻、超时、Nav2 返回失败三条路径全部改走此函数）：
  `++wp_rt_[i].fail_count` → **立即** `diagnoseSelfSurroundings()`（失败当下即定性，日志直接给结论）→ 达 `max_wp_failures` 仅 `isolateWaypoint(i, …, permanent=false)` **隔离该点** → `requestCostmapClear("航点失败后重新规划")`（＝需求①的「重新规划导航」具体动作，复用 V0.0.92/0.0.94 的清图 + `costmaps_clear_pending_` 挂起重试门控）→ 任务级 `consec_fail_ >= max_consec_failures` 才 `enterFault()` → 否则 `advanceToNextAvailableWaypoint()` **放弃当前航点、改发下一个可用航点**（重发仍由主循环经 V0.0.98 `goal_cancel_settle_time` 静置门控放行，不在此直发，避免取消竞态）。
- `isolateWaypoint()/promoteExpiredIsolation()`：临时隔离冷却 `wp_isolation_cooldown`(120s) 到期**自动解除并清零该点失败数**；`permanent`（地图判定不可达）不降级、但同按冷却周期**复判**（重跑地图校验 + 可达性，通过即解除）——避免「永久隔离」变成新的死锁。
- `resultCallback()`：成功/「已在到达半径内」分支清 `wp_rt_[i].fail_count/isolated` 且 `consec_fail_=0`、`mission_recovery_round_=0`（一次成功即视为任务健康）；非 `NAVIGATING/OBSTACLE_AVOID` 的残余结果不计失败；`goalResponseCallback()` 改用 `goal_reject_count_`（任一 goal 被接受即清零），连续拒收才 `enterFault`。

**三、需求②：车身四周/脚底「假障碍占位」自动诊断（激光 + 相机交叉验证）**
`SurroundReport diagnoseSelfSurroundings()`：
1. **A 路（Nav2 怎么看）**：订阅 `/local_costmap/costmap`（帧 `odom`，QoS 对齐 `Costmap2DPublisher` 的 `KeepLast(1)+transient_local+reliable`），统计落在**车身框 + 外扩 `self_check_margin`(0.35m)** 内且 `cost ≥ self_check_cost_min`(99，即内切/致命) 的格数与「最近距车身框距离」（0 = 脚底）。
2. **B 路（传感器真看到什么）**：订阅 `/perception/lidar_objects`(base_link) 与 `/perception/fused_objects`(base_link，**含仅由相机确认的目标**)，按「中心 + `max(0.15, 0.5×max(dim))` 半径 + 边距」判近身是否有实体；`/perception/vision_objects` 原始帧为 `camera_color_optical_frame`，本节点**不引入 TF 依赖**做几何换算，只以「新鲜度 + 目标数」参与结论描述。
3. **结论四态**：数据不足→不可判定；近身无占位→非假障碍问题；占位且激光/相机可见→**真障碍/自反射，需移车**；占位而两路传感器均无实体→**【假障碍占位】⇒ 清图即可恢复**，连判 `self_check_confirm_count`(3) 次防抖后自动 `requestCostmapClear`。
4. `odom→base_link` 用 `/localization/odom`（EKF，`world_frame=odom`）位姿做**逆旋转**（`xb=c·dxo+s·dyo, yb=-s·dxo+c·dyo`），并按 `grid->header.frame_id` 自动改选 `map`（`/relocalization/pose`）/`base_link` 位姿源，容错配置差异。
5. **触发时机**：`NAVIGATING` 周期巡检（`self_check_interval` 5s）、每次航点失败当下、每次 FAULT 自愈检查、以及**进入 FAULT 即刻**（结论直接写进 `[FAULT]` 日志，取代旧的人工三项清单）。
6. ⚠ **配置依赖（本版本同步改动）**：`nav2_params.yaml` 的 `local_costmap` 必须开 `always_send_full_costmap: true`——`Costmap2DPublisher::publishCostmap()` 仅在「该参数为真 **或** 窗口几何变化」时发整图，否则平时只发 `*_updates`；而车辆停驻待诊断时恰恰「几何不变、只有代价变化」，不开本项诊断将永远收到过期图。开销 200×200 格 @5Hz ≈ 0.2 MB/s（本机 DDS）。

**四、需求③：航点可达性按「地图范围」判定（可通行连通域）**
在旧四级校验（矩形界/未建图/占据栅格/净空 `waypoint_clearance_m`）之上再加**第五级**：
1. `ensureReachabilityCache(px,py)` 两步派生（惰性重建，独立 `geom_mutex_`）：① **5/7 chamfer 距离场**（1/10 格单位，占据与未建图同为障碍源）→ 每格到最近障碍的近似欧氏净空；② 从车位格出发 **8 邻域 BFS 连通域**，**斜向需两侧正交格均可通行**（杜绝贴对角缝「穿墙」），预算 `reach_bfs_max_cells`(400000 格) 截断。可通行判据 = 非占据、非未建图且净空 ≥ `reach_clearance`(0.35m ≈ 内切半径 0.32 + 余量)。
2. `waypointReachabilityDetail()`：航点格不在车位连通域内 → 判**客观不可达**，`isolateWaypoint(…, permanent=true)` **直接永久隔离**（不再逐圈重复下发、重复失败），日志给直线距离与原因。
3. **宁误放勿误杀**：地图未就绪 / 车位在图外 / BFS 被预算截断 / 车位格无净空且 1m 内找不到可通行格 → 一律返回「不可判定」并**放行**（误杀全部航点会人为造出 FAULT）。
4. 缓存失效：`mapCallback()` 中 `++map_revision_`（换图/重定位地图更新即重建），车位移动超 `reach_recompute_dist`(1.0m) 才重建（节流）；锁级次固定「取 `data_mutex_` 拷贝 `shared_ptr` 快照 → 释放 → 再拿 `geom_mutex_` 跑秒级派生」，不阻塞 10Hz 主循环。

**五、需求④：FAULT 由「永久锁存」改为自愈态 + 接管后切回 AUTO 自动复驶**
1. `enterFault()`：`++mission_recovery_round_`、取消在途 goal、**即刻生成自动诊断结论**（近身占位四态 + 连通域复判摘要 + 临时/永久隔离计数）写入日志与 `fault_diagnosis_`，并置 `fault_next_check_ = now + fault_hold_time`(20s)。日志不再要求「离开 AUTO 再切回」，改为「第 N 轮自愈 + 自愈条件 + 人工通道仍可用」。
2. `faultRecoveryStep()`（FAULT 分支每 100ms 调用）：静置期只节流提示 → 到点则**先推进**下一次检查时刻（`backoff = min(fault_retry_interval × 轮次, fault_retry_max_interval)` = 30s~180s，防刷屏与慢死循环）→ 刷新 `bt_navigator` 激活态 → 近身诊断（假障碍→清图 / 近身干净→例行清图）→ **遍历全部航点复判**（隔离到期推进 + 地图校验 + 可达性）→ **三重门控** `AUTO 条件 & Nav2 ACTIVE` ∧ `可用航点 ≥ 1` ∧ `近身非真实障碍包围` → 通过即 `resetMissionState("FAULT 自愈成功（自动复驶）", false)` 回 IDLE，由 IDLE 分支自动清图并重启任务。
3. **模式边沿复位**（`mainLoop()` 顶部 `prev_mode_auto_` 下降沿）：离开 AUTO（遥控接管/降级，ESTOP 除外）即 `resetMissionState(…, clear_permanent=true)` **全量复位含永久隔离**——人工很可能已移车/重标航点/换图；**切回 AUTO 的上升沿无需任何特殊动作**（`state_` 已是 IDLE，IDLE 分支按 AUTO 条件自动重启），即需求④「遥控控制车辆切回 AUTO 后自动恢复自动驾驶」。
4. `resetMissionState()`：取消 goal → 复位 `wp_rt_`（`clear_permanent=false` 时保留客观不可达判定，避免重启后立刻再碰同一个不可达点）→ 清全部计数/计时/挂起标志 → **`++goal_epoch_`**（自查修正：否则复位后旧 goal 的 `CANCELED/ABORTED` 会在新一轮任务已起时被误判为真实失败）→ `state_ = IDLE`。
5. `fault_auto_recover: false` 可退回「只停驻等人工」的旧行为；超过 `max_mission_recoveries`(5) 轮只提高告警强度并给人工建议，**仍不锁死**。

**六、参数与配置**
- `autonomous_nav_params.yaml`（`src/auto_mission/config`）**新增 23 项**（需求①2 + 需求②12 + 需求③4 + 需求④5），`max_wp_failures` 语义由「跨航点累计」改为「**单个航点**连续失败上限」；节点侧全部有合法值兜底：`max_wp_failures<1→1`、`wp_isolation_cooldown<10→10s`、`max_consec_failures < max_wp_failures+2 → 抬到 +2`（否则来不及轮转就进 FAULT）、`self_check_box_*` 非正→回退 0.45/0.37/0.32、`self_check_cost_min` clamp[1,100]、`self_check_interval≥1s`、`self_check_data_timeout≥0.3s`、`reach_clearance<0.32→0.35`、`reach_recompute_dist≥0.2`、`reach_bfs_max_cells≥10000`、`fault_hold_time≥5s`、`fault_retry_max_interval≥fault_retry_interval`、`max_mission_recoveries≥1`。
- `nav2_params.yaml`（`src/hunter_bringup/config`）：`local_costmap.ros__parameters` 新增 `always_send_full_costmap: true`（见上「三 6」）。
- **未新增任何 ROS 话题/消息/服务类型**，只新增三个订阅（`/local_costmap/costmap`、`/perception/lidar_objects`、`/perception/vision_objects`，均为系统既有话题），`hunter_msgs` 无改动。

**七、代码变更清单**
| 文件 | 变更 |
|---|---|
| `src/auto_mission/include/auto_mission/auto_mission_node.hpp` | `WaypointRuntime`/`SurroundReport` 结构体；11 个新方法声明；3 个订阅句柄与诊断缓存成员；`wp_rt_/consec_fail_/goal_reject_count_/mission_recovery_round_/fault_diagnosis_/fault_enter_time_/fault_next_check_/last_self_check_/phantom_confirm_/prev_mode_auto_`（**替换 `wp_fail_count_`**）；`geom_mutex_/map_revision_/occ_dist_field_/reach_mask_/reach_rev_/reach_cx0_/reach_cy0_/reach_truncated_`；23 个参数字段 |
| `src/auto_mission/src/auto_mission_node.cpp` | 头部状态机注释、构造函数 3 订阅 + 启动横幅打印自愈配置、23 项参数声明/读取/校验、`mainLoop()` AUTO 边沿复位 + NAVIGATING 周期巡检 + 三条失败路径改走 `handleWaypointFailure` + FAULT 分支改 `faultRecoveryStep`、`sendNextWaypoint()` 三级预检（隔离态 → 地图校验含可达性 → 已到达）、`enterFault` 重写 + 11 个新方法实现、`resultCallback/goalResponseCallback` 计数改造（2066 → 3174 行） |
| `src/auto_mission/config/autonomous_nav_params.yaml` | V0.1.00 四组参数块（含需求逐条注释与取值约束） |
| `src/hunter_bringup/config/nav2_params.yaml` | `local_costmap.always_send_full_costmap: true` |
| 四份文档 | 本条 + README/User_Manual/Deployment_Guide 升 V2.2 |

**八、部署与重编**
```bash
cd ~/HunterEdge && source /opt/ros/humble/setup.bash
# ⚠ Windows 同步 mtime 陷阱：强制干净重编本轮 2 个包
rm -rf build/auto_mission install/auto_mission \
      build/hunter_bringup install/hunter_bringup
colcon build --packages-select auto_mission hunter_bringup
source install/setup.bash   # Ctrl+C 重启 bring-up 生效
```

**九、验证链**
1. 启动横幅应打印 `V0.1.00 任务自愈：逐点隔离=开(冷却120s/单点上限3次/任务上限12次)，近身假障碍诊断=开，地图可达性=开，FAULT 自愈=开(静置20s/重试30s~180s)`；
2. `ros2 topic info /local_costmap/costmap -v` 的 Publisher count ≥1 且车辆静止时 `ros2 topic hz /local_costmap/costmap` ≈5Hz（验证 `always_send_full_costmap` 生效，否则诊断收不到刷新）；
3. 人为让某个航点不可达（rviz2 里在航点前画一条障碍带 / 或把航点改到未建图区）：日志应见 `[地图可达性] 连通域已重建…` 与 `[航点隔离] 航点[i] … 永久隔离…（移车/换图/重标航点后按 120s 冷却周期自动复判）`，同时**巡航线继续跑其他航点、不再锁存整条任务**；
4. 让车停在脏栅格上（接管压过旧障碍标记后原地不动）：`[航点失败] … 自动诊断：代价地图(odom 帧…): 近身占位格 N 格（最近距车身框 X.XXm，0=脚底）｜激光目标…｜融合目标…｜相机目标…｜假障碍计数 k/3`，k 达 3 后自动 `[清图] …已请求清除全局/局部代价地图`；
5. 触发一次 FAULT：日志应为 `[FAULT] … → 第 N 轮自愈，20s 后自动复检（无需人工解锁）→ 自动诊断：… → 自愈条件：…`；把车移到空旷处（或遥控移开障碍）后**不做任何解锁动作**，应见 `[FAULT] 自愈检查（第 N 轮）：AUTO/Nav2就绪=是，可用航点=k/n，近身真障碍=否…` → `[FAULT→IDLE] 自愈成功…→ 自动清图并重启自主导航`；
6. 遥控接管（切 REMOTE）再切回 AUTO：接管瞬间应见 `[任务复位] 全量（含地图不可达判定）：模式开关离开 AUTO…`，切回 AUTO 后应自动清图 → `WAITING_LOCALIZE` → `NAVIGATING` **重启任务，无需人工再动模式开关**；
7. 长跑验收：一条含 1 个坏航点的巡航线连续巡航 ≥3 圈不出现永久停驻（旧版本第 3 次失败即锁死）。

**十、遗留项与风险**
1. **C++ 变更（`auto_mission`）本机无 ROS2 环境未编译**：需车端 `colcon build` 通过并按上述验证链确认；若报错请回传 `colcon` 输出以便修正（本轮已做括号配平与成员/方法声明一致性静态自检）。
2. **`always_send_full_costmap: true` 是需求②的硬依赖**：若现场关闭，`diagnoseSelfSurroundings()` 会因整图过期而长期落到「数据不足·不可判定」，此时**不会误判也不会自动清图**（安全侧退化为「只诊断不出手」），需求①③④不受影响。
3. 相机链路仅以「新鲜度 + 目标数」参与结论描述（原始帧 `camera_color_optical_frame`，本节点刻意不引 TF 依赖）；相机确认但激光未确认的实体由 `/perception/fused_objects` 承载几何判定——**若 sensor_fusion 停跑，近身「真/假」判定退化为仅激光**（`fused_ok=false` 时不参评，仍偏保守：宁可判「不可判定」也不误清图）。
4. 可达性连通域按「车位置信度最高的 `map` 系位姿」为源：定位跳变（`map→odom` 漂移）时可能把可达航点误判为「另一连通域」→ 永久隔离；已按「冷却周期自动复判 + 换图 `++map_revision_` 失效 + 离开 AUTO 全量清除」三重兜底，但若现场定位频繁大跳，应先修定位（`hunter_relocalization` fitness 门控）再考虑调小 `reach_clearance`。
5. 「永久隔离」航点在单点航点列表（`waypoints` 只有 1 个点）中等同停摆——属预期（无处轮转），日志会明说「全部航点均已隔离」；巡检语义下建议航点数 ≥3。
6. 遗留项沿用 V0.0.97/98：`range_min ≥ 0.65` 硬约束、REEDS_SHEPP 主动倒车现场安全半径、仓库示例航点占位待重标、局部代价地图不含 static_layer。

---

## V0.1.01（Date：2026-9-22）

**主题：health_monitor「相机崩溃/重启风暴」误报修复——频率看门狗把系统过载下的正常帧率抖动误判为进程崩溃**

**一、故障复盘（运行日志根因链修正）**
实车日志（约 7 分钟）出现成片 `camera 节点/话题异常（疑似崩溃）` → `camera 节点重启（第 N 次）` → `camera 节点在 300s 内重启超过 3 次，标记为故障`，极易被误判为 realsense 驱动崩溃并去查 USB/供电。逐行核查证据链：
1. **全日志无任何 realsense 崩溃栈、无 `No sample received`、无 USB/ENODEV 报错**；恢复时实测频率为 12/14/15/12.5Hz——相机从未真正死过。
2. 告警源头是 `health_monitor::checkNodes()`：**它不 respawn 任何进程**，仅按 `/camera/camera/color/image_raw` 话题 1s 窗实测频率做「异常↔恢复」边沿计数——跌破 `camera_min_rate` 即打「疑似崩溃」，恢复边沿即 `restart_count_++` 打「重启第 N 次」，`>restart_limit(3)` 打「标记故障」。
3. 阈值错配：相机标称帧率 **15Hz**（V0.0.97 起由 30 降到 15，见 User_Manual §4.5 自检表），而旧 `camera_min_rate=10Hz`。整机 CPU 过载时（同期 `Control loop missed 20Hz`、costmap `observation buffer not updated`）帧率偶发跌破 10Hz 即被逐次累加为「崩溃/重启风暴」，把**感知链过载症状**包装成**相机故障**。
4. 定位层末端 `定位收敛=否` 亦为**设计内正确行为**（非 bug）：NDT `fit` 因 LiDAR 数据饥饿（日志 `局部点云过少(0)`）发散到 ~5.2 > 硬 ceiling 3.0，`alignOnce` 走 else 分支发大协方差宣告失效——「跟踪/置信度解耦」与 `/relocalization/pose` 的 `transient_local().reliable()` QoS 修复（V0.0.94）均已在位且工作正常。

**二、修复（仅根因①，措辞 + 阈值；不触碰定位/导航逻辑）**
| 文件 | 变更 |
|---|---|
| `src/hunter_monitor/health_monitor/config/health_monitor_params.yaml` | `camera_min_rate: 10.0 → 5.0`（对应真实断流下限，避免 15Hz 标称帧率的正常抖动被误判）；补注释说明 |
| `src/hunter_monitor/health_monitor/src/health_monitor.cpp` | `checkNodes()` 措辞去误导（**逻辑与对外语义完全不变**：`restart_count_`/`restart_limit_`/`/system/health` 的 `fault` 位均保留）：`节点/话题异常（疑似崩溃）`→`话题频率持续低于阈值（疑似数据饥饿/过载，非进程崩溃）`；`节点重启（第 N 次）`→`话题频率恢复（异常-恢复第 N 次）`；`节点在 300s 内重启超过 3 次，标记为故障`→`话题在 300s 内频率异常-恢复超过 3 次，标记为故障`；补注释明确 health_monitor 不做 respawn |

**三、不在本次修复范围（下游/实机项）**
- 根因②：`/scan`、`/perception/lidar/obstacle_cloud`、`/cloud_registered` 间歇饥饿 + CPU 过载——**实机排查项**（`ros2 topic hz` 佐证），代码/日志层无 bug。
- 根因③：定位层两处历史修复（NDT 解耦、QoS）均已在位；末端锁存是正确安全行为，不改。
- 根因④：导航/任务层 `Failed to make progress`→45s 超时→FAULT 锁存为下游结果，上游（②）恢复后自动消退；V0.1.00 任务自愈四件套已覆盖。

**四、部署与重编**
```bash
cd ~/HunterEdge && source /opt/ros/humble/setup.bash
rm -rf build/hunter_monitor install/hunter_monitor   # ⚠ Windows 同步 mtime 陷阱，强制干净重编
colcon build --packages-select hunter_monitor
source install/setup.bash   # 重启 bring-up 生效
```
YAML 阈值改动重启 bring-up 即生效；`.cpp` 措辞改动需重编 `hunter_monitor`。本版本仅动 `health_monitor` 一个包，无需重编其他模块。

**五、验证链**
1. 相机话题 `ros2 topic hz /camera/camera/color/image_raw` 稳定 ≈15Hz 时，health_monitor 不再刷「疑似崩溃/重启」；
2. 帧率短暂跌到 6~9Hz（过载）不再触发异常（阈值已降 5Hz）；真断流（<5Hz 持续 2s）仍会告警，措辞为「话题频率持续低于阈值（疑似数据饥饿/过载，非进程崩溃）」；
3. `/system/health` 的 camera `node_states` 语义不变（仍 anomaly→fault）。

**六、说明与遗留**
1. 本次仅消除**误报放大**，未改动任何安全停机逻辑：CAN 中断→`/estop`、温度保护、定位/感知门控均不变。
2. 若现场相机确为高频真崩溃（伴随 `VIDIOC_QBUF failed`/`No sample received`/0Hz），仍按 User_Manual §6.2 相机 USB 链路条目排查——那是另一类故障，不在本误报修复范畴。
3. 本条同步升四份文档基线 V0.1.00→V0.1.01、文档版本 V2.2→V2.3。

## V0.1.02（Date：2026-09-22）

**主题：《HUNTER SE 低速自动驾驶避障解决方案》逐条对表落地——避障主链经核对**已全部在位**，本轮只补齐代价地图尺度、感知参数与安全层默认值 5 处量化参数，并把 6 项有意保留的偏差写明理由**

**一、对表审计结论（先说"不需要改什么"）**
拿到《避障方案》（基线 V0.1.00）后先做的是**逐条核对**而非重写。核对锚点与结果（完整表见 Deployment_Guide §5.5.2(1)，可逐条上机验证）：

| 方案章节 | 核对结果（锚点） |
|---|---|
| §3 六层架构 L1~L6 | ✅ 在位：`lidar/vision_perception → sensor_fusion → fast_lio2 + hunter_relocalization → Nav2 → safety_guard → auto_mission` |
| §4.1 激光 9 步链 / §4.3 融合 | ✅ 在位：SOR → 射线坡度地面分割 → 欧氏聚类 → PCA-OBB → KF+匈牙利；时间对齐 + 置信度门控 + 加权融合 |
| §4.4 代价地图语义 / §4.5 /scan 安全链 | ✅ 在位（局部图**无 static_layer** 与方案"已知限制"一致；`0.70m/1.0°/0.3s` → safety_guard 直读） |
| §5.1 REEDS_SHEPP / §5.2 MPPI 4.0s | ✅ 在位（V0.0.97 / V0.0.98） |
| §5.3 行为树（定位门控/感知哨兵/1Hz/恢复池倒车首位） | ✅ 在位（V0.0.84~V0.0.96） |
| §5.4 七态状态机 + 航点五级预检 | ✅ 在位（V0.1.00） |
| §6.1 扫掠弧闸 / §6.2 分级防护 / §6.3 滞环 / §6.4 三层边界 | ✅ 在位（V0.0.98 / V0.0.85~0.91 / V0.0.95 / V0.0.87） |
| §7.1 几何量 / §7.2 三场景 / §7.3 假障碍诊断 | ✅ 在位（V0.0.99 / V0.0.97·V0.0.98 / V0.1.00） |
| §9.1~§9.3 部署与分模块验证 | ✅ 一致（Deployment_Guide §6.x；三个 launch 文件齐备） |
| §9.4 验收 10 项 | ⭕ 本轮补入 Deployment_Guide §5.7「避障专项验收」 |
| §10 故障排查 | ✅ 已覆盖；本轮补 2 行（目标过分割、全局路径贴墙） |

⇒ **结论：方案描述的避障主链在 V0.1.00 已全部在位，无功能缺口。**待补的是量化参数与"偏差理由"。

**二、本轮改动（5 处参数 + 2 处文档）**

| 文件 | 参数 | 改动 | 依据 / 理由 |
|---|---|---|---|
| `nav2_params.yaml` | `global_costmap.inflation_radius` | 0.40 → **0.55** | 方案 §4.4。全局路径留白 = 半宽 0.32 + 0.23m，Smac 路径"天然取中"→ 少贴墙贴门框 → 少触发安全层 SLOWDOWN/STOP（低速巡航更平顺）。可通行性不受损：仅 ≥ 内切半径 0.32m 的格才是致命格；2.5m 通道扣两侧膨胀仍余 1.4m 净宽 > 车宽 0.64m |
| 同上 | `global_costmap.update_frequency` | 2.0 → **1.0** | 方案 §4.4。行为树重规划 V0.0.92 起已回退 1Hz，2Hz 反复刷新 100m×100m@0.1m（100 万格）纯属浪费 |
| 同上 | `local_costmap.update_frequency` | 10.0 → **5.0** | 方案 §4.4。@0.5m/s 每拍 10cm；膨胀 0.45m + MPPI 20Hz 重优化足够；近身快速闸是 safety_guard 直读 /scan 的 20Hz 扫掠弧，不依赖本图 |
| 同上 | `local_costmap.width/height` | 10 → **6** | 方案 §4.4。校核：4.0s 时域 @0.5m/s 前瞻 2.0m + 前缘 0.45 + 膨胀 0.45 ≈ 2.9m < 3.0m ✔。两项合计局部图算力降至 ≈1/5.6（200²@10Hz → 120²@5Hz） |
| `lidar_perception_params.yaml` + `lidar_perception.cpp/.hpp` | `ground_max_slope` | **新增（默认 5.0°）** | 方案 §4.1 第 6 步。原为 `RayGroundFilter` 构造默认 8.0° 不可配；平坦地面 dz≈0 两者判定一致，收紧只让坡道/台阶更早判为障碍（偏保守） |
| 同上 | `outlier_mean_k` | 10 → **50** | 方案 §4.1 第 5 步。SOR k 邻域是全链唯一噪点滤除环节，k 越大越能区分孤立噪点与真实稀疏回波 → 减少 costmap／safety_guard 幽灵点误判 |
| 同上 | `cluster_tolerance` | 0.5 → **0.15** | 方案 §4.1 第 7 步，取舍见下"三、偏差"（近身安全关键区聚类恒完整；代价地图障碍源不经聚类） |
| `sensor_fusion_params.yaml` | `vision_conf_min` | 0.45 → **0.50** | 方案 §4.3。抬高后低置信度误检不再参与融合，使"激光 × 融合"交叉验证（假障碍诊断 B 路）更干净 |
| `safety_guard.cpp` | `max_linear_vel` **默认值** | 0.8 → **0.5** | 方案 §8.3。本参数是第二重速度硬限，默认值必须等于生产值——否则 launch 漏传参时静默放行 0.8m/s（1.6× 巡航速度、4× 制动距离） |
| 文档 | Deployment_Guide §5.5.2 / §5.7 / §5.8 / §7.5；README；User_Manual | 对照偏差表 + 避障 10 项验收 + 升级行 + 排障 2 行 | 让"方案 vs 实现"可核对、可追溯 |

**三、有意保留的偏差（6 项，均写明量化理由；改前必读 Deployment_Guide §5.5.2(3)）**
1. **点云裁剪下界 0.2m → 0.5m**：雷达装于 x=0.23/z=0.35，车体结构在 0.3~0.5m 有回波，降到 0.2 会把自反射灌进聚类 → "贴脸幽灵目标"。近场由包络盒（0.57m）与 `/scan range_min 0.70` 兜住。
2. **融合对齐窗 ±50ms → 0.20s**：任务层 `OBSTACLE_AVOID/ESTOP` 消费 `/perception/fused_objects`，融合超窗是**整对丢弃**；收到 0.05s 后 CPU/USB 抖动会让融合间歇无输出 ⇒ 等于静默关掉任务层急停。@0.5m/s 下 0.20s = 10cm（≪ 半宽 0.45m）。
3. **"DBSCAN" 实现为 PCL `EuclideanClusterExtraction`**：同义算法（欧氏区域生长），三参数与方案同值；换真 DBSCAN 需改 C++ 且无安全收益。
4. **相机 640×480@15（方案 §4.2 写 30）**：V0.0.97 降载；方案 §9.3 验收亦允许 15-30Hz。
5. **YOLOv8s（方案写 YOLOv8n）**：`.engine` 必须目标机生成且已存在，换模型是独立部署动作（§4.6），不能靠改配置完成。
6. **局部图不含 `static_layer`**：与方案 §3"已知限制"一致，由"全局 Smac 绕行 + 航点净空预检 + 恢复池倒车"三层兜底。

**四、部署与重编**
```bash
cd ~/HunterEdge && source /opt/ros/humble/setup.bash
rm -rf build/hunter_safety build/hunter_bringup build/lidar_perception \
       install/hunter_safety install/hunter_bringup install/lidar_perception
colcon build --packages-select hunter_safety hunter_bringup lidar_perception
source install/setup.bash        # 并 Ctrl+C 重启 bring-up
```
> 仅当现场需要一并应用 `vision_conf_min` 改动时再加编 `sensor_fusion`（`colcon build --packages-select hunter_safety hunter_bringup lidar_perception sensor_fusion`）。**Windows 同步 mtime 陷阱**：跨机同步源码后必须先 `rm -rf build/<pkg> install/<pkg>` 再编，否则 colcon 认为无变更而跳过。

**五、验证链**
1. `safety_guard` 启动横幅：`v_max=0.50m/s … stop=0.90m, slow=1.40m … 释放滞环=+0.25/+0.20 … V0.0.98 轨迹扫掠弧=行进中启用`；
2. `ros2 param get /lidar_perception ground_max_slope` = 5.0、`cluster_tolerance` = 0.15、`outlier_mean_k` = 50；
3. `ros2 topic echo /local_costmap/costmap --field info.width` = **120**（6m/0.05）；`ros2 topic hz /local_costmap/costmap` ≈**5Hz**（假障碍诊断的 1.0s 新鲜度仍满足）；
4. rviz2 观察 `/plan`：与墙/门框留白明显增大（0.55m 量级），低速巡航 SLOWDOWN 行数下降、`Control loop missed 20Hz` 频次下降；
5. 避障回归：按 Deployment_Guide §5.7「避障专项验收」10 项逐项验证（其中第 3 项必须看到 `/cmd_vel.linear.x` 负值）。

**六、说明与遗留**
1. 本轮**未改任何避障算法**（扫掠弧闸 / REEDS_SHEPP / MPPI 4.0s / 自愈四件套保持 V0.0.98~V0.1.00 行为），只调尺度参数与默认值——因此**回归重点在"是否变迟钝/变激进"，而非功能是否可用**。
2. 回滚指引：若改后出现"障碍顶到近处才减速"或假障碍诊断报"数据过期"，仅把 `local_costmap.update_frequency` 回 10.0（其余不动）；若 4m 外目标数暴涨，仅把 `cluster_tolerance` 回 0.5。
3. 本机（Windows 侧）**无 ROS2/C++ 工具链**，C++ 改动（`safety_guard.cpp` 默认值、`lidar_perception.cpp/.hpp` 新增 `ground_max_slope`）**未经实编**，需按第四节在车端干净重编并回传编译输出。
4. 本条同步升四份文档基线 V0.1.01→V0.1.02、文档版本 V2.3→V2.4。

## V0.1.03（Date：2026-09-23）

**主题：修复「开启自主导航、刚起步车辆即停下、无法进入正常自动驾驶」——NDT 重定位 `fitness_hard_ceiling` 仍在冻结位姿更新导致的运动发散死亡螺旋（跟踪/置信度彻底解耦）**

**一、故障现象与根因（实车日志 `ros2 launch hunter_bringup hunter_full.launch.py use_autonomous_nav:=true autonomous_nav_mode:=nav`）**
1. 静止时 NDT 完美收敛（`✓ 收敛 fit=1.655 位置(-0.03,-0.20,0.15)`），`auto_mission` 判定位收敛、清图后进入 `NAVIGATING`、发送航点、车辆起步（转向角实时变化）。
2. **车一动的第一个 NDT 周期** `fit` 从静止 1.65 冲到 **3.006**，越过 `fitness_hard_ceiling=3.0` → 日志 `✗ 位姿拒绝刷新(step_ok=1 fit=3.006 ceiling=3.00)，维持旧位姿并宣告失效`（`step_ok=1` 说明并非跳变，纯粹是 fit 越阈值）。
3. `alignOnce` 走 else 分支**冻结 `T_map_odom_`** → 车继续动、局部滚动云相对被冻结的 map 系越走越偏 → fit 更高（5.719/5.603/5.638…）→ 更不更新，**死亡螺旋自锁**，`定位收敛=否` 永不恢复。
4. `auto_mission` 见 `/relocalization/pose` 大协方差 → `[NAVIGATING] AUTO 条件丢失，取消导航，回到 IDLE` → 车辆停车，之后再不复航。
5. **定性**：V0.0.94 的「跟踪/置信度解耦」不彻底——虽把 `fitness_max` 的冻结职责剥离，但 `fitness_hard_ceiling` 仍留在跟踪条件 `tracking = step_ok && (fitness <= ceiling)` 里。本地图极稀疏（0.5m 降采样后仅 1614 点、NDT 1.0m），行驶期 fit 基线本就 2~3，一次运动畸变尖峰即越过 3.0 触发冻结，等于换了个阈值重演历史死亡螺旋。这违反核心原则：**位姿更新只能由「跳变合理性(step_ok)」门控，质量阈值(fitness)只能调节置信度/协方差。**

**二、修复（仅定位层，不触碰安全/导航/任务逻辑）**
| 文件 | 变更 |
|---|---|
| `src/hunter_relocalization/src/ndt_relocalization_node.cpp` | `alignOnce`：跟踪条件由 `step_ok && (fitness<=fitness_hard_ceiling_)` 改为 **`tracking = step_ok`**（位姿刷新只由跳变闸门决定）；`fitness_hard_ceiling` 降级为**仅影响置信度**——在跟踪分支内新增 `fit>ceiling` 子分支：跳过去抖、立即发大协方差宣告未收敛，**但位姿仍无条件刷新**以便 NDT 自动回弹重收敛；else 分支（拒绝刷新）语义收窄为「仅跳变」。同步更新文件头设计说明与参数默认值注释 |
| 同上（参数默认值） | `fitness_max` 默认 `2.5 → 3.5`、`fitness_hard_ceiling` 默认 `3.0 → 5.0`（与 YAML 对齐，容纳稀疏地图行驶期 fit 基线） |
| `src/hunter_relocalization/config/relocalization_params.yaml` | `fitness_max: 2.5 → 3.5`、`fitness_hard_ceiling: 3.0 → 5.0`；重写收敛/跳变闸门段注释，明确「位姿刷新只由 step_ok 决定、fitness 只调节置信度」 |

**三、行为变化（修复后）**
- 车辆起步后即使 fit 短暂升到 3~5，NDT **持续刷新 map→odom 跟随运动**，fit 随几何对齐回落到行驶基线，不再冻结发散；
- 仅当发生单周平移 >0.5m / 旋转 >0.35rad 的真跳变（疑似瞬移/跟错）才维持旧位姿；
- `fit>fitness_max(3.5)` 连续 5 帧（≈2.5s）或 `fit>ceiling(5.0)` 时仍会对外宣告「未收敛」触发 `auto_mission` 保护性退 IDLE——这是**正确的置信度门控**，但因位姿不再冻结，短暂运动畸变不会再自锁死，恢复行驶后能自动重收敛。

**四、部署与重编**
```bash
cd ~/HunterEdge && source /opt/ros/humble/setup.bash
rm -rf build/hunter_relocalization install/hunter_relocalization   # ⚠ Windows 同步 mtime 陷阱，强制干净重编
colcon build --packages-select hunter_relocalization
source install/setup.bash   # 重启 bring-up 生效
```
`.cpp` 改动需重编 `hunter_relocalization`；YAML 阈值改动重启 bring-up 即生效。本版本仅动定位一个包。

**五、验证链**
1. 起步后观察 `hunter_relocalization` 日志：不应再出现 `✗ 位姿拒绝刷新(... ceiling=...)` 后 fit 单调攀升到 5+ 的锁死序列；改为 `✓ 收敛` 或 `！运动畸变 fit=... TF 跟踪中，去抖保持收敛`，且 `位置(x,y)` 随车辆运动连续变化（不再恒为 ~0.0）；
2. `auto_mission` 进入 `NAVIGATING` 后不再于起步瞬间退回 IDLE，车辆持续巡航通过多个航点；
3. `ros2 topic echo /relocalization/pose --field pose.covariance[0]` 行驶期在收敛态应保持 `converged_covariance(0.01)` 量级（去抖窗口内）。

**六、说明与遗留**
1. 本次仅解耦定位层「跟踪 vs 置信度」，未改任何安全停机逻辑（`safety_guard`、地图边界监护、硬件急停、`auto_mission` 门控判据均不变）。
2. **根因缓解而非根治**：行驶期 fit 基线偏高源于全局地图过稀（1614 点@1m NDT）。若现场仍偶发 `fit>3.5` 持续 5 帧退 IDLE，应**重建更密地图**（建图时降低 `pcd_to_map`/FAST-LIO2 保存体素、或减小 `target_voxel_size`/`ndt_resolution`），把 fit 基线整体压低，而非继续放宽阈值。
3. 严重失准兜底仍在（`fit>ceiling` 立即宣告未收敛）：当地图/初始位姿真不一致（如非建图起点上电且未发 `/initialpose`），NDT 卡局部极小、fit 持续 >5.0 时，`auto_mission` 仍会保护性停车——此为设计内正确行为，需按 User_Manual 用 rviz2「2D Pose Estimate」发 `/initialpose` 重置。
4. 本条同步升四份文档基线 V0.1.02→V0.1.03、文档版本 V2.4→V2.5。

## V0.1.04（Date：2026-09-29）

**主题：车端 Kafka 安全认证配置落地——完成与数据采集分析系统对接（SASL_SSL + SCRAM-SHA-512）**

**一、变更背景**
平台侧已分配车端 Kafka Broker 地址 `120.202.73.105:9093`，安全协议为 **SASL_SSL + SCRAM-SHA-512**。原仓库中 `data_agent`、`remote_agent`、`ota_agent` 三个模块均使用占位域名 `platform.example.com:9093`，且未配置任何 SASL/SSL 认证参数，导致车端无法与云端数据采集分析系统建立加密连接。

**二、变更内容**
| 文件 | 变更 |
|---|---|
| `src/hunter_agents/data_agent/config/data_agent_params.yaml` | `kafka_brokers` 改为真实地址；新增 `security_protocol`、`sasl_mechanism`、`sasl_username`、`sasl_password`、`ssl_ca_location` 五项参数 |
| `src/hunter_agents/data_agent/include/data_agent/data_agent.hpp` | 新增 5 个 SASL 成员变量 (`security_protocol_`、`sasl_mechanism_`、`sasl_username_`、`sasl_password_`、`ssl_ca_location_`) |
| `src/hunter_agents/data_agent/src/data_agent.cpp` | 构造函数声明并读取 SASL 参数；`kafkaInit()` 配置 librdkafka 的 `security.protocol`、`sasl.mechanism`、`sasl.username`、`sasl.password`、`ssl.ca.location` |
| `src/hunter_agents/data_agent/package.xml` | 新增 `libsasl2-dev`、`libssl-dev`、`libsasl2-modules`、`libsasl2-modules-gssapi-mit` 依赖声明 |
| `src/hunter_agents/remote_agent/config/remote_agent_params.yaml` | `kafka_brokers` 改为真实地址；新增 SASL_SSL 五项参数 |
| `src/hunter_agents/remote_agent/remote_agent/remote_agent.py` | `init_kafka()` 改为构建含 SASL 配置的 dict 并传入 `Producer()`；`DEFAULT_CONFIG` 同步更新 |
| `src/hunter_agents/ota_agent/config/ota_agent_params.yaml` | 新增 `kafka_brokers` + SASL_SSL 五项参数 |
| `src/hunter_agents/ota_agent/ota_agent/ota_agent.py` | `_init_kafka()` 构建 `base_conf` 含 SASL 参数，Consumer/Producer 均复用；`DEFAULT_CONFIG` 同步更新 |

**三、Kafka 连接参数汇总**
| 参数 | 值 | 说明 |
|--------|------|------|
| `kafka_brokers` | `120.202.73.105:9093` | 平台分配 Broker 地址 |
| `security_protocol` | `SASL_SSL` | 安全传输 + 认证 |
| `sasl_mechanism` | `SCRAM-SHA-512` | 挑战握手认证机制 |
| `sasl_username` | `hunter_vehicle` | 车端账号（部署时按实际凭据填写） |
| `sasl_password` | `CHANGE_ME` | ⚠ 部署时必须替换为真实密码 |
| `ssl_ca_location` | `/etc/ssl/certs/ca-certificates.crt` | CA 证书路径（自签证书时替换为对应 CA 文件） |

**四、部署与重编**
```bash
# 1. 安装 SASL/SSL 系统依赖（必做）
sudo apt install -y librdkafka-dev libsqlite3-dev libsasl2-dev libssl-dev libsasl2-modules

# 2. 修改真实凭据（三处 YAML）
sed -i 's/CHANGE_ME/<实际密码>/' src/hunter_agents/data_agent/config/data_agent_params.yaml
sed -i 's/CHANGE_ME/<实际密码>/' src/hunter_agents/remote_agent/config/remote_agent_params.yaml
sed -i 's/CHANGE_ME/<实际密码>/' src/hunter_agents/ota_agent/config/ota_agent_params.yaml

# 3. 重编 data_agent（C++ 变动）
cd ~/HunterEdge && source /opt/ros/humble/setup.bash
rm -rf build/data_agent install/data_agent
colcon build --packages-select data_agent
source install/setup.bash

# 4. remote_agent / ota_agent 为 Python 服务，无需编译，重启 systemd 服务即生效
sudo systemctl restart remote-agent ota-agent
```

**五、验证链**
1. `data_agent` 启动日志应出现 `Kafka 安全协议: SASL_SSL, SASL 机制: SCRAM-SHA-512`，且不再报 `Failed to resolve 'platform.example.com'`；
2. 平台侧确认收到 `hunter.hunter_001.telemetry` 主题消息（10Hz JSON 遥测）；
3. 断网场景：拔除网线后观察 `Kafka 投递失败` 日志且 SQLite 缓存开始写入，恢复网络后 `Kafka 投递恢复` 日志出现；
4. `remote_agent` 与 `ota_agent` 服务日志无 `SASL authentication error`，Kafka 连接正常。

**六、说明**
1. 本版本只改 Kafka 连接配置，不涉及导航/安全/感知算法。
2. 三处 `sasl_password: "CHANGE_ME"` 为安全占位，**绝不得提交真实密码到 Git 仓库**（已在 `.gitignore` 中排除实例配置）。
3. 若平台使用自签证书，需将证书部署到车端并修改 `ssl_ca_location` 指向对应 `.pem` 文件。
4. 本条同步升四份文档基线 V0.1.03→V0.1.04、文档版本 V2.5→V2.6。

## V0.1.05（Date：2026-10-09）

**主题：HunterCore 车端接入全链打通——以接入包为单一可信源，补齐 8 Topic 契约的四条断链（health / command / command_result / remote_control）**

**一、变更背景**
HunterCore 平台按车下发**接入包**（`kafka.properties` + `ca-cert.pem` + `client-cert.pem` + `client-key.pem`，本例 `HUNTER-001`），要求车端以 `SASL_SSL + SCRAM-SHA-512` **叠加 mTLS 客户端证书**（`ssl.endpoint.identification.algorithm=https`）接入，Topic 命名硬契约 `hunter.<vehicle_id>.<type>`（每车 8 条），且明令凭据不得入镜像层/日志/版本库。V0.1.04 的做法（YAML 里填 `sasl_password`/`ssl_ca_location`、Broker 写死在文档）与之不兼容；同时比对 8 Topic 发现四条**从未打通的断链**：
1. `health`：车端有 `/system/health` 但从不上云；
2. `command` / `command_result`：**无任何车端消费者**，平台下发的指令全部落到空气里；
3. `remote_control`：`remote_agent` 只监听 WebSocket，平台走 Kafka 下发时遥控无效；
4. 旧版 `remote_agent` 自发 `remote_status` Topic（契约外），且**超时后持续发零速顶住 `/remote/command`**——`decision_making` 永远认为有人在遥控，造成“遥控断开后永不交还 AUTO、自主导航无法恢复”。
另外 `acks` 全局为 `1`（与契约要求的 `event/command/ota_*` 需 `all` 不符）、SQLite 缓存表无 `topic` 列（health/event 无法续传）、消费端自动提交 offset（崩在指令执行中会丢指令）。

**二、变更内容**
| 文件 / 包 | 变更 |
|---|---|
| `src/hunter_common/hunter_kafka/`（**新建包**） | `config.py`：`kafka.properties` 解析 + **Java→librdkafka 键映射**（`sasl.jaas.config` 拆 username/password、JKS truststore 不可读则回落 `ca-cert.pem`、PEM 优先于 PKCS#12）+ `redact()` 脱敏；`client.py`：`topic_name()` 拼名（拿不到 `vehicle_id` 即报错）、`make_producer()` 按 Topic 语义选 acks、`make_consumer()` 强制 `enable.auto.commit=false`；`diagnose.py`：`hunter-kafka-check` **七层自检** CLI（第 4 层新增 **SASL 机制插件本地判定**：机制为 SCRAM 却无 `libscram.so` 时直接报 30，处置给“按文件判定 + `dpkg -S` 查属主”而不写死包名，不再与真实口令错误混淆）；**第 6/7 层导入与归因修正（HUNTER-001 实机发现）**：`AdminClient` 改从 `confluent_kafka.admin` 取（顶层不重导出，旧写法报 `cannot import name 'AdminClient'` 而被误读成“库未安装”），异常类改取顶层 `KafkaException`（`confluent_kafka.errors` 模块与 `SaslAuthenticationException` 类**并不存在**，旧代码走到那里会直接 ImportError）；握手/投递错误改按**文本关键字 + `KafkaError` 常量名**归类（不再硬赌 `(13, 29, -151)` 三个数值），投递阶段的口令错/无 ACL 现在分别报 30/50 而不是一律 60；**`AdminClient` 的构造也纳入同一套归类**（构造期同样抛 `KafkaException(_INVALID_ARG)`，旧写法只包了 `list_topics`，私钥读不到时异常直接穿透成 Traceback + 退 1）；**第 1 层新增凭据可读性实测**（逐件真 `open` 一次，运行用户打不开即报 10 并给出 `chown` 处置——旧写法只看 mode 是否为 0600，看不出“属主是 root 的 0600 根本读不到”）；**stdout 兜底 `errors="replace"`**（非 UTF-8 环境下 ✓/✗ 不再把自检崩成退 1）；`config.py` 的 `_existing()` 对“存在但读不到”的凭据直接抛 `KafkaConfigError` 而不静默降级（避免拿着不完整的 TLS 配置去连 broker、最后报成认证失败）；`test/test_config.py` 单测 |
| `src/hunter_agents/command_agent/`（**新建包**） | 消费 `hunter.<vid>.command`（组 `command-<vid>`、`earliest`、**手动提交**）→ 调既有服务/话题执行 → 回执 `command_result`。五道护栏：白名单（未登记 `REJECTED`）/ TTL / `command_id` LRU 幂等 / 单条在途+超时看门狗 / 破坏性指令 `confirm` + 静止门控 |
| `src/hunter_agents/data_agent/`（C++） | **新建 `kafka_access.{hpp,cpp}`**（properties → librdkafka 配置装配，与 `hunter_kafka` 同源语义）；拆 **双 producer**（acks=1 走 telemetry/health，acks=all 走 event）；`dr_cb` **投递证实才删缓存行**；SQLite `telemetry_cache` 新增 `topic` 列，**三类消息统一断点续传**；新增 `health` 1Hz 上报（无数据报 `NO_DATA`，不伪造 `OK`）；启动打印“接入源 + 脱敏配置快照”（口令打星） |
| `src/hunter_agents/remote_agent/` | 新增 **Kafka `remote_control` 消费通道**（组 `remote-<vid>`、**`latest`** + `remote_cmd_ttl` 二次过期）；**修复交还缺陷**：超时/`REMOTE_RELEASE` → 一次性补发刹车帧后**停止发布** `/remote/command`（非零速顶住）；废弃契约外的 `remote_status` 自发 Topic，会话事件改走 `hunter.<vid>.event` |
| `src/hunter_agents/ota_agent/` | 统一走 `hunter_kafka` 配置源；`ota_status` **acks=all**；按 `task_id` **幂等**（`/data/ota/.task_state.json`，重复通知不重跑）；消费端手动提交 |
| `ota-agent.service` / `remote-agent.service` | `ExecStart` 不再写死 `/opt/hunter/…`，改 `source` ROS2 + `$HUNTER_WS_PREFIX/setup.bash` 后执行包内入口；路径与配置由 `EnvironmentFile=-/etc/hunter/agent_env.sh` 提供；`RestartSec` 10/5 + `StartLimit*` 防风暴；`setup.py` 安装 unit 与 `console_scripts` 入口 |
| `src/hunter_bringup/scripts/hunter_core_setup.sh`（**新建**） | 一键幂等部署：⓪ 前置校参（运行用户/主组存在、接入包四件套齐、工作空间存在；失败文本直接带 scp/查找/目录列表等下一步动作，`--bundle` 传错层级自动下钻一层；**`--bundle` 只在首次接入时必填**：未传而 `/etc/hunter/kafka` 四件套已齐时复用现场凭据并整段跳过②，旧写法一进来就“必须指定 --bundle”，改完口令重跑只能满车找原始包）→ ① 依赖（**按文件/插件存在性判定，只装缺的那几件**；`apt-get update` 失败不再静默阻断；SCRAM 插件的**属主包已实机定论**（HUNTER-001 实机 Jetson **aarch64**：`dpkg -S /usr/lib/aarch64-linux-gnu/sasl2/libscram.so` 得 **`libsasl2-modules:arm64`**；先前按 jammy amd64 官方清单推的 **`libsasl2-modules-gssapi-mit`** 在 arm64 上不成立），但判据仍一律用**插件文件**（跨架构/版本不失真）、缺件先装 `libsasl2-modules`，并把 `dpkg -S` 查到的属主包打进日志（**`--skip-deps` 时同样报一句**，旧写法跳过依赖步就看不到插件状态）；**Ubuntu 系无 `cyrus-sasl-scram` 包**，旧写法直接 `Unable to locate package`；插件存在性判定逐个候选测存在（`ls A B` 会因 `/usr/lib/sasl2` 不存在而返回非 0，把“已装”误报为“缺失”——车上已实测到这条假阴性））→ ② 接入包落盘（目录 0750、properties/证书 `root:运行组 0640`）→ **②.1 凭据属主收敛（不依赖是否重新落盘，省 `--bundle` 重跑也会修）**：私钥与 p12 置为 **`运行用户:其主组 0600`** 并用 `sudo -u <运行用户> head -c 1` **实测真能打开**，读不到即失败并给出 `chown/chmod` 修正命令——**0600 只授予属主**，旧版把私钥装成 `root:组 0600` 时以 agilex 跑的 `data_agent` 根本打不开，要到 TLS 阶段才报 `ssl.key.location failed: Permission denied`（实机踩中）；私钥权限非 0600 仍直接失败（`--force-key` 才放行；**包内仍是占位符而现场已写入 SCRAM 口令时不覆盖 `kafka.properties`**，避免重跑脚本把口令抹回去，`--force-config` 才强制）→ ③ 四个 `*_params.yaml` + `agent_env.sh` + `README_agent_env` 落 `/etc/hunter/` → ④ **以运行用户身份** `colcon build`（**含 `hunter_bringup`**：否则 launch 里的 `use_command_agent` 装不进 `install/`，`command_agent` 不随栈拉起）→ ⑤ `hunter-kafka-check` 自检（非 0 不自启服务；入口路径不再赌 `bin/`，按 `bin/` → `lib/<pkg>/` → 激活工作空间后 `command -v` → 扫 `install/` 与运行用户 `~/.local/bin` 四级定位；本脚本以 root 跑而 root 的 PATH 不含普通用户的 `~/.local/bin`，旧写法会无谓退化成源码跑；入口是 setuptools 生成的 `load_entry_point` 包装，运行时需要**包元数据**在 PYTHONPATH 上，因此现自动补 `PYTHONPATH=<包源码目录>:<install/hunter_kafka/lib/python3.*/site-packages>`，并在退出码为 **1**（`PackageNotFoundError`，属解释器报错而非链路结论）时**无条件**改走 `python3 -m hunter_kafka.diagnose`；**confluent-kafka 的依赖判据改为直取 API 符号** `python3 -c "from confluent_kafka.admin import AdminClient"`（只看 `import confluent_kafka` 会漏报“装成功但版本过旧无 AdminClient”）；**额外往运行用户 `~/.local/bin/` 装一个命令包装**（`hunter-kafka-check` 本身不依赖包元数据，靠 `PYTHONPATH=<包源码目录>` + `python3 -m`）——因为 **ament 只把 `<prefix>/bin` 加进 PATH**，console_script 实际在 `<prefix>/lib/<pkg>/`，旧文档里“`source install/setup.bash` 后直接 `hunter-kafka-check`”的写法在车上根本跑不通（退 127）；需新开一个 shell 生效，包装已存在且非本脚本生成时不覆盖）→ ⑥ systemd 安装与 `enable`。`--ws` 默认取**运行用户**家目录下 HunterEdge（旧写法用 `$HOME`，sudo 下会变 `/root/HunterEdge` 而误报工作空间不对） |
| `src/hunter_bringup/scripts/hunter_status.sh` | `critical_nodes` 新增 `command_agent`；新增第 8 段“HunterCore 接入”（四件套存在性/权限、服务态、`/health` 是否有数据；**只查元信息不读 properties 内容**） |
| `src/hunter_bringup/launch/{hunter_full,data_agent}.launch.py` | 新增 `use_command_agent`（默认 `true`）并接入 `command_agent`；原 `data_agent.launch.py` 占位文件改为转发包内 launch（支持 `params_file:=`） |
| 四份 `*_params.yaml` | **凭据出仓**：仅保留 `kafka_properties`/`kafka_bundle_dir` 路径与业务参数，`sasl_*`/`ssl_*` 降为“无接入包时的开发机兜底” |
| `.gitignore` | 新增凭据排除段：`*.pem`/`*.p12`/`*.jks`/`kafka.properties`/`*hunter-*-bundle/`/`agent_env.sh` |

**三、Topic 契约与语义（8 条/车）**
| Topic | 方向 | 节拍 | acks | 消费者组/起始位点 |
|---|---|---|---|---|
| `…telemetry` | 车→云 | 10Hz | `1` | —（`key=vehicle_id`） |
| `…event` | 车→云 | 事件即发 | `all` | — |
| `…health` | 车→云 | 1Hz | `1` | — |
| `…command` | 云→车 | 平台下发 | — | `command-<vid>` / `earliest`（手动提交） |
| `…command_result` | 车→云 | 终态一次 | `all` | — |
| `…remote_control` | 云→车 | 10~20Hz | — | `remote-<vid>` / **`latest`**（手动提交） |
| `…ota_notify` | 云→车 | 升级下发 | — | `ota-<vid>` / `earliest`（task_id 幂等） |
| `…ota_status` | 车→云 | 10% 步进 | `all` | — |

指令白名单（未登记一律 `REJECTED`）：`MISSION_START/STOP`、`MAP_CONVERT`、`WAYPOINT_SAVE/CLEAR`（CLEAR 需 `confirm=true`）、`ESTOP_ENGAGE/RELEASE`（解除需车速 <0.05 m/s）、`TEST_MODE_ON/OFF`、`REMOTE_RELEASE`、`STATUS_QUERY/HEARTBEAT_ACK`。**不新增任何 ROS 话题/消息字段**，指令执行全部复用既有服务与话题（.ai-rules 约束）。

**四、部署与重编**
```bash
# 前置：接入包不随仓库分发，需先从开发机拷到车上（否则脚本第一步即报“接入包目录不存在”）
ls -l ~/HUNTER-001-bundle/HUNTER-001      # 应见 kafka.properties + 三个 .pem

# 一键（推荐）：依赖 → 接入包落盘 → 参数落盘 → 编译 → 自检 → systemd
cd ~/HunterEdge
sudo bash src/hunter_bringup/scripts/hunter_core_setup.sh \
  --bundle ~/HUNTER-001-bundle/HUNTER-001     # --user/--ws 可省（默认取 sudo 调用者与其家目录）

# 人工动作（脚本不经手）：把口令写入接入包 properties
sudo vi /etc/hunter/kafka/kafka.properties     # sasl.jaas.config 里的 <SCRAM_PASSWORD>

# 写入口令后重跑（幂等：包内仍是占位符时不会覆盖已写入的口令；**可省 `--bundle`**，
# /etc/hunter/kafka 四件套已齐即复用现场凭据并跳过②，但②.1 仍会收敛私钥属主）
sudo bash src/hunter_bringup/scripts/hunter_core_setup.sh --skip-deps --skip-build

# 手工等价编译（C++ 与 Python 入口均需重编）
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select hunter_msgs hunter_kafka command_agent data_agent ota_agent remote_agent hunter_bringup
source install/setup.bash                          # 给 ros2 命令用（注意：它不能让 hunter-kafka-check 可用）
hunter-kafka-check; echo "exit=$?"                 # 退出码必须 0（未重登时用 ~/.local/bin/hunter-kafka-check）
sudo systemctl restart ota-agent remote-agent
```

**五、验证链**
1. `hunter-kafka-check` **退出码 0**（命令入口：重登一个 shell 后直接 `hunter-kafka-check`，或 `~/.local/bin/hunter-kafka-check`；不装包装则用 `cd ~/HunterEdge && PYTHONPATH=src/hunter_common/hunter_kafka python3 -m hunter_kafka.diagnose`——**光 `source install/setup.bash` 不够，入口不在 PATH 上**）；失败时按分层定位：10 参数/口令未填（**含接入包未拷到车上/路径层级错**）、20 TLS/证书（**先 `timedatectl` 校时**；若日志是 `ssl.key.location failed: … Permission denied`，那是**私钥属主落在 root 身上导致运行用户读不到**，不是证书问题）、30 认证（先查 SCRAM 插件 `ls /usr/lib/*/sasl2/libscram.so`，实机 `dpkg -S` 定论属主包为 `libsasl2-modules`；V0.1.05 已把它提为**本地前置检查项**）、40 网络、50 Topic/ACL、60 投递；**退出码 1 与 127 不在此体系内**（1=解释器/入口自身报错：包元数据缺失、非 UTF-8 环境打印崩、旧版未把 `AdminClient` 构造纳入 try；127=命令未入 PATH；都不是链路结论）；
2. `data_agent` 启动日志：`Kafka 接入源：/etc/hunter/kafka/kafka.properties（vehicle_id=HUNTER-001, …）` 与 `Kafka 已按接入包装配（… mTLS 已启用, acks=1/all 双通道）`；
3. **平台侧该车 `last_online_time` 刷新**（联调唯一判据），且 `telemetry`(10Hz)/`health`(1Hz) 均有数据；
4. 指令闭环：平台发 `STATUS_QUERY` → `journalctl` 与日志出现 `COMMAND_RESULT {"status":"SUCCEEDED",…}`；发未登记类型 → `REJECTED` 且**无任何 ROS 动作**；重复发同一 `command_id` → 仅一条终态；
5. 遥控与交还：`remote_control` 帧可驱动车辆；停发帧 ≈0.5s 后日志出 `遥控链路结束（cmd_timeout）：已下发停车帧并停止发布 /remote/command`，`ros2 topic echo /remote/command --once` **阻塞无新数据**，模式回 AUTO；
6. 断点续传：拔网 30s 后 `sqlite3 /data/data_agent/telemetry.db 'select topic,count(*) from telemetry_cache group by topic;'` 行数上涨（三类均缓存），恢复后行数**单调下降到 0**（证实才删）；
7. OTA 幂等：重复下发同一 `task_id` 的 `ota_notify` 不重复安装，`/data/ota/.task_state.json` 可核对；
8. `./hunter_status.sh` 第 8 段全 `[OK]`；`git status` 看不到任何 `*.pem`/`kafka.properties`。

**六、说明与遗留**
1. 投递语义为**至少一次**（非正好一次）：平台侧可能收到重复消息，去重由平台幂等消费保证；消费端手动 commit + 车端幂等表共同防止“指令丢”与“指令重跑”。
2. 凭据仅存 `/etc/hunter/kafka/`（运行期注入）：YAML/文档/日志均不含口令与证书内容；代码侧输出配置必经 `redact()`。镜像化时四件套不得进镜像层。
3. SCRAM 机制插件缺失时报 `No worthy mechs found`，表象极像口令错/网络不通——已写入部署脚本依赖与两份手册排障表。**包名坑（现场实测）**：Ubuntu/Debian 无 `cyrus-sasl-scram`（那是 RHEL/openSUSE 名）；`libscram.so` 的**属主包已实机定论**（Jetson aarch64 jammy：`dpkg -S /usr/lib/aarch64-linux-gnu/sasl2/libscram.so` → **`libsasl2-modules:arm64`**；先前按 jammy amd64 清单推的 `libsasl2-modules-gssapi-mit`（Launchpad #1988730）在 arm64 上不成立），但仍**不得按包名断言可用性**：判可用性用 `ls /usr/lib/*/sasl2/libscram.so`（单模式查），查属主用 `dpkg -S`；脚本与自检均已改为**按插件文件判定**。另：Ubuntu 里名为 `scram` 的包是概率风险分析工具，与 SASL 无关。
4. 本轮**不改任何算法与安全逻辑**：`safety_guard`、`auto_mission`、`decision_making`、感知/定位均未动（`decision_making` 仅因 `remote_agent` 不再顶住零速而自然恢复 AUTO）。
5. 实机进展（HUNTER-001 车端，Ubuntu 22.04 jammy / **Jetson aarch64**）：步骤 ⓪~⑥ 已全跑到位，最新一次 **7 个包全部编译成功**（含补上的 `hunter_bringup`，`data_agent` C++ 首轮 38.2s）；SCRAM 插件**实机在位**（`/usr/lib/aarch64-linux-gnu/sasl2/libscram.so`，之前那句“未找到”是旧判定的假阴性）；⑤ 自检曾因 root 下缺包元数据而抛 `PackageNotFoundError`（退 1），后续又发现 **`source install/setup.bash` 后 `hunter-kafka-check` 仍 `command not found`（退 127）**——ament 不把 `lib/<pkg>/` 加进 PATH；已在脚本内补 PYTHONPATH + 退出码 1 退回源码跑法，并由步骤⑤ 自动生成 `~/.local/bin/hunter-kafka-check` 命令包装。**最新一次实机自检（普通用户，模块跑法）已连过 1~5 层全绿**：四件套在位与权限（私钥 0600，但当时刻为 `root:agilex`——即下一轮撞到的那个坑）、`bootstrap=192.168.31.35:9093` + `SASL_SSL` + `SCRAM-SHA-512`、**口令已人工写入（长度 32，非占位符）**、证书有效期 2026-10-09~2036-10-06 且 **CN == vehicle_id == `HUNTER-001`**、SCRAM 插件在位（`/usr/lib/aarch64-linux-gnu/sasl2/libscram.so`）、**Broker TCP 可达**；唯一阻断是第 6 层入口报 `cannot import name 'AdminClient'`（即上述导入路径缺陷，**非链路故障**，已修）；`--skip-deps` 重跑又被旧写法“必须指定 `--bundle`”卡住（已改为可省写）。**第三轮实机（同步修好的 `diagnose.py` 后）**：`confluent-kafka 2.16.0` 实测可用（`from confluent_kafka.admin import AdminClient` 通过，上一轮的“库不可用”假象确认为代码缺陷）；SCRAM 插件属主包实机定论 `libsasl2-modules:arm64`；第 6 层不再报导入错，而是崩在 **`Permission denied: calling fopen(/etc/hunter/kafka/client-key.pem, r)`**——私钥实为 `root:agilex 0600`，0600 的组位为 0，**运行用户（也是 `data_agent` 的运行身份）根本打不开自己的私钥**，旧写法又只包了 `list_topics` 而未包 `AdminClient()` 构造，于是异常穿透成 Traceback + 退 1。已对应修正：脚本②.1 收敛属主 + 实测可读、自检第 1 层逐件真打开凭据、构造异常纳入归类（同一文本归 **20** 并直指 `chown`）、`config.py` 装配阶段不再静默降级、stdout 编码兜底。现场一条命令可解：`sudo chown agilex:agilex /etc/hunter/kafka/client-key.pem && sudo chmod 600 /etc/hunter/kafka/client-key.pem`（或直接重跑部署脚本）。**尚未完成**：修好后的完整自检（目标退 0，拿到 SASL 认证与 8 Topic 清单结论）与平台侧 `last_online_time` 联调；需平台侧先为 `HUNTER-001` 建满 8 Topic 并开通该账号（否则自检退 30/50）。
6. 本条同步升四份文档基线 V0.1.04→V0.1.05、文档版本 V2.6→V2.7（README §7.7/§11.3/§12.5/§13.9；User_Manual §4.1/§4.2.1/§5.6/附录C；Deployment_Guide §5.6/§5.7/§6.11/§6.12/§7.6/§8.4）。

## V0.1.06（Date：2026-10-09）

**主题：上电自启链路（CAN → HunterEdge → 运营端上报 → 平台指令进入模式） + Kafka 消息体逐字段对齐 HunterCore 契约**

**一、现场故障与两条根因（平台侧现象：车端自检全绿，运营端 `data_collector.vehicle_telemetry` 恒 0 行、车辆台账恒离线）**

1. **车端根本没在跑**：V0.1.05 只把链路做成“可手工启动”，上电后仍需人工执行
   `ip link set can2 down` / `ip link set can2 type can bitrate 500000` / `ip link set can2 up` / `candump can2 -n 10`
   以及 `ros2 launch hunter_bringup hunter_full.launch.py`。现场没人敲 → `data_agent`/`command_agent` 根本没起
   → 平台自然一条消息也收不到。
2. **消息体不合契约（更隐蔽）**：`data_agent` 上行的 telemetry/health/event 是**平铺自造字段**
   （`velocity`/`pose_x`/`fused_object_count`/`overall_status`/`{type:hard_deceleration,level}`…），
   而 HunterCore `contracts/kafka/schemas/*.schema.json` 是**六段嵌套 + `additionalProperties:false`**；
   平台 `data-collector` 消费侧启用 `schema_name="auto"` 校验 → 车端报文被判 `schema_invalid` **直接进 DLQ**
   → 平台表 0 行。`command_agent` 同类问题：读 `type`（契约是 `command_type`）、回执发
   `{type,status,reason,vehicle_state,ts}`（契约是 `{command_id,vehicle_id,timestamp,success,...}`）
   → 即便指令送达也会被判非法，平台侧永远看不到回执。

> ⚠ 平台侧另有自身缺陷（`data-collector` 通配订阅未转正则 → 订阅字面 `hunter.*.telemetry` → 永不消费，
> HunterCore V1.18.12 修复）。**车端契约对齐是必要条件**：两侧都修好，平台才会刷新。

**二、需求①——上电自动启用底盘 CAN**

| 新增/变更 | 说明 |
|---|---|
| `src/hunter_bringup/scripts/hunter_can_up.sh`（新） | 幂等 CAN 上电：`modprobe gs_usb` → 等待接口枚举（USB-CAN 晚于内核启动，默认最多 20s）→ `ip link set <if> down` → `type can bitrate 500000` → `up` → `candump` 抓帧。**严格模式要求抓到附录 A 的底盘反馈帧 0x211/0x221**（只看 `ip link` 的 UP 会把“接线错/波特率错/底盘未上电”误判为成功——此时 CAN 写入静默丢弃，车辆看着像在跑却不动）。退出码 `0` 成功 / `1` 环境 / `2` 无接口 / `3` 无底盘帧；开关 `--quick`（只复查接口）、`--no-strict`/`CAN_REQUIRE_FRAMES=0`（台架） |
| `src/hunter_bringup/systemd/hunter-can.service`（新） | `Type=oneshot` + `RemainAfterExit=yes`，开机即拉起；`ExecReload` 支持适配器掉线后 `systemctl reload hunter-can` 复检。路径经 `HUNTER_CAN_UP_SCRIPT`/`HUNTER_WS_PREFIX`（`agent_env.sh`）注入，不写死 |

**三、需求②——CAN 就绪后自动启动 HunterEdge，进入自动驾驶准备状态**

| 新增/变更 | 说明 |
|---|---|
| `src/hunter_bringup/launch/hunter_edge.launch.py`（新） | 生产开机入口：薄封装 `hunter_full.launch.py`，默认 `use_autonomous_nav=true`（**自动驾驶准备状态**：定位/感知/Nav2/auto_mission 就绪待命）+ `use_data_agent=true` + `use_command_agent=true`；调试入口 `hunter_full.launch.py` 的默认值保持不变 |
| `src/hunter_bringup/scripts/hunter_edge_up.sh`（新） | ① CAN 复查 → ② 运营端连通性预检 `hunter-kafka-check`（**失败不阻断**：断网时车端落 SQLite，恢复后按序回放；阻断反而连缓存都不会发生）→ ③ source ROS/工作空间后 `ros2 launch hunter_bringup hunter_edge.launch.py` |
| `src/hunter_bringup/systemd/hunter-edge.service`（新） | `Requires=hunter-can.service` + `After=hunter-can.service network-online.target`（**底盘 CAN 没起来就不挂“AUTO 待命”**）；`ExecStartPre` 再复核一次 CAN（失败 → `Restart=on-failure` 走完整时序重试）；`KillSignal=SIGINT` + `TimeoutStopSec=120`（保 fast_lio2 退出写 PCD、data_agent flush） |
| `hunter_core_setup.sh` 步骤⑥ | 由 2 个单元扩到 4 个（`hunter-can`/`hunter-edge`/`ota-agent`/`remote-agent`）并按依赖顺序 enable + 启动；新增 `--no-start`（台架只装不启）；`agent_env.sh` 增加 `HUNTER_CAN_UP_SCRIPT`/`HUNTER_EDGE_UP_SCRIPT`/`HUNTER_EDGE_AUTONOMOUS_NAV`/`HUNTER_EDGE_NAV_MODE`/`HUNTER_EDGE_MAP_*` |

**四、需求③——车端与运营端建立连接，按 Kafka 消息格式上传（契约对齐）**

`data_agent`（`src/hunter_agents/data_agent/src/data_agent.cpp`、`include/data_agent/data_agent.hpp`）：

| 段 | 旧（不合契约） | 新（对齐 `contracts/kafka/schemas/*.schema.json`） |
|---|---|---|
| `telemetry` | 平铺 `velocity/steering/battery_soc/control_mode/vehicle_state/fb_*/pose_x/pose_y/fused_object_count/target_*/overall_status/cpu_*` | **六段嵌套**：`chassis`(12 项) + `localization`(x/y/z/roll/pitch/heading/linear_velocity/angular_velocity/position_std/heading_std) + `perception`(detected_objects/fps/latency_ms/object_types) + `planning`(trajectory_length/trajectory_points/planning_latency_ms/current_behavior) + `control`(target_velocity/target_steer/velocity_error/steer_error/control_latency_ms) + `system`(7 项)；顶层加 `seq`（单调递增，平台幂等/丢包检测） |
| `health` | `overall_status: OK\|NO_DATA` + 平铺 `cpu_usage/gpu_usage/memory_usage/disk_usage/nodes[]` | `{vehicle_id, timestamp, status, free_storage_mb, system{...}}`；`status` 取车辆 8 态受控词表（车端可判定 `online_idle`/`auto_driving`/`remote_controlled`/`fault`/`emergency`）；`system` 与遥测同构（契约要求一致） |
| `event` | `{type:"hard_deceleration"\|"overspeed"\|"low_battery"\|"hard_turn"\|"hard_acceleration", level:"warning"\|"critical"}` | `{event_type, event_level, description, data}`，类型/等级改**契约词表**：`harsh_acceleration`/`harsh_braking`/`harsh_turning`/`over_speed`/`battery_low`(warning)、`battery_critical`/`emergency_stop`(critical)、`manual_takeover`(info)，**新增 `communication_loss`**（链路不可用 ≥ `comm_loss_duration`(10s) 上报一次，恢复后重新武装） |

实现要点（全部取自**已有数据源**，不新增话题/消息/字段）：

- `perception.fps` = `/perception/fused_objects` 回调速率的 1s 窗口结算；`latency_ms` = 帧头时间戳 → 本节点时延；
  `object_types` = 按 `DetectedObject.class_name` 计数；`detected_objects` = 目标总数
- `planning.*` = `/planning/trajectory` 的点数与相邻点弧长累加；`planning_latency_ms` = 帧头时间戳时延
- `control.velocity_error`/`steer_error` = `/control/command` 目标 − `/chassis/feedback` 实际
- `system.memory_usage_mb` = `/proc/meminfo`（MemTotal − MemAvailable）；`network_latency_ms` = 1Hz 对
  `bootstrap.servers` 首个 broker 做 TCP 建连计时（非阻塞、150ms 超时）；`network_rssi` 无测量源报 0（契约要求 ≤0，合规）
- `health.free_storage_mb` = `statvfs("/")`（G-08 OTA 存储门禁输入）
- ⚠ **不臆造**：契约 `health.gear`（OTA P 档门禁）与 `health.lat/lng`（G-11 地理围栏）需底盘档位/GPS 数据源，
  当前 `ChassisState` 无对应字段（.ai-rules 禁止新增消息字段）→ **缺省省略**，平台按契约描述“门禁缺数据即拒绝”的安全默认处理

**五、需求④——平台自动驾驶指令 → 进入相应模式 + 契约回执**

`command_agent`（`command_agent/command_agent.py` + `config/command_agent_params.yaml`）：

1. **下行字段兼容**：新增 `_command_type`/`_command_params`/`_command_issued_at`，读契约
   `command_type`/`timestamp`/`params`/`timeout_ms`，**同时兼容**旧名 `type`/`issued_at`/`payload`
   （平台 remote-control 的 `session_command` 生产者当前仍发旧名）；
2. **自动驾驶指令映射**：`enable_auto_driving → MISSION_START`、`disable_auto_driving → MISSION_STOP`
   （锚点为契约示例 `command.schema.json` examples[0]，即需求④“接收到自动驾驶指令后进入相应模式”）。
   平台 `command_type` 取值域**未定稿**（HunterCore pending #17），故做成配置
   `command_type_aliases`（形态 `["平台类型=车端动作", ...]`，因 `rcl_yaml_param_parser` 不支持嵌套 map）
   —— 定稿后只改 YAML，不动代码；
3. **回执改契约信封**：`{command_id, vehicle_id, timestamp, success, result_code, message, data}`；
   `result_code`：成功/重复 `"0"`、拒绝 `4001`、失败 `4002`、超时 `4003`、过期 `4004`；
   车端执行状态/指令类型/车辆快照统一放进 `data`（契约 `additionalProperties=false`）；同一 `command_id` 仍只允许一条终态；
4. **护栏链不变**（白名单 → 时效 → 幂等 → 门控 → confirm）：仍**不新增任何 ROS 话题/消息**，
   动作落地全部复用既有服务（`/auto_mission/start_mapping_cruise` 等）与话题（`/estop`、`/safety/test_mode`、`/remote/command`）。

**六、观测与运维**

- `hunter_status.sh` 新增第 8 段「上电自启链路」（can2 状态/波特率 + `candump` 抓底盘反馈帧 +
  `hunter-can`/`hunter-edge`/`ota-agent`/`remote-agent` 服务态；原 HunterCore 接入段顺延为第 9 段）
  与第 10 段「运营端上行（车端侧自证）」（SQLite 待回放缓存条数，应为 0 或单调下降）；
- `hunter_bringup/CMakeLists.txt` 增装 `systemd/`（`USE_SOURCE_PERMISSIONS`），单元随包安装到
  `share/hunter_bringup/systemd/`，供脚本与现场直接取用。

**七、部署与验收（缺一不可）**

```bash
cd ~/HunterEdge && source /opt/ros/humble/setup.bash
# data_agent 为 C++ 改动：必须重编（command_agent / hunter_bringup 为 Python/launch，重编后随包生效）
rm -rf build/data_agent install/data_agent
colcon build --packages-select hunter_msgs hunter_kafka data_agent command_agent hunter_bringup --symlink-install
source install/setup.bash
# systemd 单元与自启链路（--no-start 可让台架只装不启）
sudo bash src/hunter_bringup/scripts/hunter_core_setup.sh --skip-deps --skip-build

# 1) 开机链路：hunter-can 有抓帧成功日志、两服务 active
systemctl is-active hunter-can hunter-edge ota-agent remote-agent
journalctl -u hunter-can -b --no-pager | grep -E '底盘反馈帧|退出码'
# 2) 运营端接入：退出码必须 0（10 配置 / 20 证书 / 30 认证 / 40 网络 / 50 Topic / 60 投递）
hunter-kafka-check; echo "exit=$?"
# 3) 平台侧唯一判据：last_online_time 刷新 + telemetry/health 有数据，且**无** schema_invalid DLQ 新增
# 4) 指令闭环：平台发 enable_auto_driving → 车端 journalctl 出
#    COMMAND_RESULT {"command_id":...,"success":true,"result_code":"0",...}，/auto_mission/status 进入巡航态；
#    重复发同一 command_id → message 含 [DUPLICATE]，车辆不重复执行
```

**八、说明与遗留**

1. **至少一次语义不变**：断网缓存、投递证实才删行、消费端手动 commit 均未改动；平台侧仍可能收到重复消息（靠幂等收敛）。
2. `data_agent` 为 C++ 改动，**本机（Windows 侧）无 ROS2 工具链，未经实编**，须按第七节在车端干净重编并回传编译输出；
   Python 侧（`command_agent.py`）已通过 `py_compile`，shell 脚本（`hunter_can_up.sh`/`hunter_edge_up.sh`/改动后的
   `hunter_core_setup.sh`/`hunter_status.sh`）已通过 `bash -n`。
3. **两端都要修**：本次解决车端侧（自启 + 契约对齐）；平台侧 `data-collector` 的“通配订阅转正则”
   （HunterCore V1.18.12）是平台的必要修复，未同步前平台仍可能不消费。
4. 已知缺口（不臆造、不伪造）：契约 `health.gear`（OTA P 档门禁）与 `health.lat/lng`（地理围栏）需底盘档位/GPS 数据源，
   待底盘驱动提供后再补（届时按 `.ai-rules` 先扩消息契约再改代码）。
5. 本条同步升四份文档基线 V0.1.05→V0.1.06、文档版本 V2.7→V2.8（README §7.8/§7.9/§12/§13.10；
   Deployment_Guide §3.3/§5.6.4/§6.1/§6.12/§7.6/§8.3；User_Manual §2/§3/§5.1/§6.2）。

**九、实机验证与追加修复（2026-10-09，HUNTER-001）**

**实机结果（好消息）**：`colcon build --packages-select hunter_msgs hunter_kafka data_agent command_agent hunter_bringup`
**5 个包全部编译成功**（`data_agent` 40.4s，C++ 契约对齐改动通过编译，无告警级错误）；
`sudo bash src/hunter_bringup/scripts/hunter_can_up.sh` 成功启用 can2 并**抓到底盘反馈帧**
（`221 [8] 00 00 00 00 00 00 01 A3` / `211 [8] 00 00 00 F1 00 00 00 84`）→ 需求① 车端侧验证通过。

**同时暴露 4 个问题（本轮全部修复）**：

| # | 现象 | 根因 | 修复 |
|---|---|---|---|
| 1（**阻断**） | `hunter_core_setup.sh --skip-deps --skip-build` 报 `× 接入包目录不存在:`（`$BUNDLE_DIR` 为空白）→ 步骤③④⑤⑥全没跑，于是 `systemctl is-active hunter-can hunter-edge ota-agent remote-agent` 全 `inactive`、`hunter-kafka-check: command not found`（127） | 走 `REUSE_BUNDLE=1`（不带 `--bundle`，现场四件套齐）时 `BUNDLE_DIR` 为空属**正常**，但紧接着的 `[ ! -d "$BUNDLE_DIR" ]` 中 `-d ""` 恒假 → `!` 后为真 → `die`。**V0.1.05 起就存在**：文档承诺的“改完口令/参数后重跑可不传 `--bundle`”这条路径一进来就挂 | 把“自动下钻 + 目录存在性 + 四件套 + p12 提醒”整段收进 `if [ "$REUSE_BUNDLE" -eq 0 ]`，复用分支只打一行日志。**回归测试**：把该代码块原样抽出，跑 4 个场景（复用空路径 / 合法包 / 目录不存在 / 只传外层目录）→ 全部符合预期 |
| 2 | （潜在）开机自启的节点与登录终端里的 `ros2`/`rviz2` 互相看不见 | unit 里硬编码 `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`：systemd 读不到 `~/.bashrc`，若现场实际用别的实现就会**落到另一套 DDS** | 单元不再硬编码；改为 `agent_env.sh` 的 `HUNTER_RMW_IMPLEMENTATION`（③ 以运行用户身份探测一次登录 shell 的 `RMW_IMPLEMENTATION` 并写入；探测不到留空＝交给 ROS 默认），由 `hunter_edge_up.sh` 继承并打一行日志 |
| 3 | （潜在，静默）上电后进不了“自动驾驶准备状态” | 我给 `agent_env.sh` 加了**行内注释**（`HUNTER_EDGE_NAV_MODE=nav   # nav=…`），而 systemd `EnvironmentFile` **不支持行内注释**——`#` 之后整段都会算进变量值 → 值非法（`nav  # nav=…`），且不报错 | 全部改为**整行注释**，并在脚本与文档中写明该约束（pre-existing 的 V0.1.05 文件本无行内注释，是本次新增的） |
| 4 | `hunter_can_up.sh` 打印 `can2 已启用：` 后面**空白** | `ip -details link show` 中 `bitrate 500000` 与 `can state ERROR-ACTIVE` **分属两行**，旧正则 `'bitrate [0-9]* can state [A-Z]*'` 跨行永远匹配不到 | 分开 grep，回显 `operstate=up bitrate 500000 can state ERROR-ACTIVE`；`hunter_status.sh` 同步补 `can state`（bus-off 排查有用） |

**修复后现场操作（HUNTER-001 直接照抄）**：

```bash
cd ~/HunterEdge && source install/setup.bash
# ① 只更新脚本/单元（Python 与 launch 已随上一轮 colcon 编译生效；无需重编）
git pull                       # 或同步本轮改动
# ② 重跑部署（不带 --bundle：复用现场凭据；--skip-build 因为刚编过）
sudo bash src/hunter_bringup/scripts/hunter_core_setup.sh --skip-deps --skip-build
#   ↑ 这次应一路走到 ⑥：安装 4 个单元并 enable + 启动
# ③ 验收
systemctl is-active hunter-can hunter-edge ota-agent remote-agent     # 期望 4 个 active
hunter-kafka-check; echo "exit=$?"                                    # 期望 0
#   刚跑完脚本但没重登：用 ~/.local/bin/hunter-kafka-check，或
#   PYTHONPATH=~/HunterEdge/src/hunter_common/hunter_kafka python3 -m hunter_kafka.diagnose
systemctl status hunter-edge --no-pager                               # 看整栈是否起来
journalctl -u hunter-edge -b -f                                       # 期望含「自动驾驶准备状态」横幅
./src/hunter_bringup/scripts/hunter_status.sh                          # 新增第 8/10 段
```

> ⚠ 若 `hunter-edge` 起来但 `ros2 node list` 看不到节点：查 `agent_env.sh` 的
> `HUNTER_RMW_IMPLEMENTATION` 是否与登录终端 `echo $RMW_IMPLEMENTATION` 一致（修复 #2）。

**十、第二轮实机（20:36 启动整栈）暴露的根因与修复（2026-10-09 当晚）**

**实机结果**：部署脚本步骤①~⑥ **全程跑通**；`hunter-can`/`hunter-edge` 均 `active`，
整栈 26 个进程起齐（rslidar / realsense / ch10x / hunter_base / fast_lio / ekf /
lidar_perception / vision_perception / sensor_fusion / decision_making / map_server /
hunter_relocalization / pointcloud_to_laserscan / controller / planner / behavior /
bt_navigator / velocity_smoother / safety_guard / lifecycle_manager / auto_mission /
pcd_to_map / **data_agent** / health_monitor）→ 需求②③车端侧成立。
**但暴露一件事：`command_agent` 不在进程表里**；`ota-agent`/`remote-agent` 停在 `activating`；
`hunter-kafka-check` 仍 `command not found`；`./hunter_status.sh` 报 `Permission denied`。

**统一根因（一个）**：**setuptools 入口包装在此安装布局下找不到发行版元数据**
`--symlink-install` 对 ament_python 包按 `setup.py develop` 语义安装：元数据（`<pkg>.egg-info`）
留在**源码目录**，靠 `install/<pkg>/lib/python3.x/site-packages/easy-install.pth` 把源码目录
加进 `sys.path`；而 **`.pth` 只在 site.py 处理的 site-packages 目录里生效，对 PYTHONPATH 条目无效**
（`source install/setup.bash` 提供的正是 PYTHONPATH）→ 模块能 import（site-packages 里是指向源码的
符号链接），但 `importlib.metadata` 找不到元数据 → `load_entry_point(...)` 抛
`PackageNotFoundError: No package metadata was found for ...`（实机 Traceback 已确认：
`install/hunter_kafka/lib/hunter_kafka/hunter-kafka-check` 退 1）。
对照：`health_monitor`（ament_cmake，C++ 可执行直接装 `lib/health_monitor/`）**一直正常**——
这正是修复方向。

| # | 现象 | 修复 |
|---|---|---|
| 1 | `command_agent` 起后即退（进程表 PID 14261 → 14265 之间缺 14263） | 新增 `command_agent/scripts/command_agent_node`（**普通 Python 脚本，不依赖元数据**），`setup.py` 用 `data_files` 装到 `lib/command_agent/`；launch 的 `executable` 改为 `command_agent_node`（节点名仍 `command_agent`，参数段不变） |
| 2 | `ota-agent` / `remote-agent` 停在 `activating`（`Restart=on-failure` 重启循环） | 同上新增 `ota_agent/scripts/ota_agent_node`、`remote_agent/scripts/remote_agent_node`，装到各自 `lib/<pkg>/`；两个 unit 的 `ExecStart` 改指 `*_node`（`--config` 原样透传） |
| 3 | `hunter-kafka-check: command not found`（127，且 `source install/setup.bash` 也没用） | ① 自检改为**源码方式优先**（`python3 -m hunter_kafka.diagnose`，永不受元数据影响），入口包装仅兜底；② 命令包装除 `~/.local/bin` 外**再装一份 `/usr/local/bin/hunter-kafka-check`**（在默认 PATH 上，**无需重登**）；③ `hunter_edge_up.sh` 的预检同样按「PATH 包装 → 源码方式 → 入口」排序 |
| 4 | `./hunter_status.sh: Permission denied` | 脚本 `git update-index --chmod=+x`（8 个 `.sh` + 3 个 `*_node` 记为 `100755`），并在部署脚本新增 **④.1 可执行位自愈**（`chmod 0755` 脚本与三个 Agent 入口），即使传输丢了位也能自愈 |

**修复后现场操作**：

```bash
cd ~/HunterEdge && git pull && source /opt/ros/humble/setup.bash
# 三个 Agent 的入口改成了 data_files（lib/<pkg>/<pkg>_node），需要重编这 3 个包
colcon build --symlink-install --packages-select command_agent ota_agent remote_agent
sudo bash src/hunter_bringup/scripts/hunter_core_setup.sh --skip-deps --skip-build
# 验收
systemctl is-active hunter-can hunter-edge ota-agent remote-agent   # 期望 4 个 active（不再 activating）
hunter-kafka-check; echo "exit=$?"                                  # 期望 0（/usr/local/bin 已装，无需重登）
ros2 node list | grep -E 'command_agent|data_agent'                 # 期望两个都在
journalctl -u ota-agent -n 20 --no-pager ; journalctl -u remote-agent -n 20 --no-pager
```

> ⚠ **为什么之前 `hunter-kafka-check` 的"源码方式续跑"是对的**：它绕开了元数据；本次把这条
> 路径提升为**首选**，并推广到三个 Python Agent（改用不依赖元数据的可执行入口），
> 与仓库内 `health_monitor`（ament_cmake）的既有做法保持一致。

**十一、三个 Python/C++ Agent 的启动崩溃与契约缺陷（2026-10-10 定位，全部修复）**

上一步把入口问题解决后，`command_agent_node` 终于**被执行到**（08:47:41 pid 6948），随即
暴露出**代码本身**的缺陷；同一轮把 `ota_agent`（一直 `activating`）也查清，并顺带补齐
`data_agent`/`ota_agent`/`remote_agent` 与契约的字段/等级偏差。

| # | 现象（实机日志/状态） | 根因 | 修复 |
|---|---|---|---|
| 1 | `command_agent_node` 起来后立刻 `exit code 1`：`AttributeError: 'CommandAgent' object has no attribute '_health_cb'`（`ros2 node list` 里始终没有 command_agent、平台指令永远无回执） | `__init__` 里四处 `create_subscription(..., self._health_cb/_chassis_cb/_odom_cb/_mission_cb, ...)` 引用了**类里根本不存在的回调**（全文件 33 个 `def` 无其一）→ 每次启动都崩在 `__init__` 第一个订阅 | 补齐四个回调（只更新缓存，供 `_motion_gate()`/`_snapshot()` 使用） |
| 2 | `ota-agent` 恒 `activating`（`Restart=on-failure` 5s/10s 重启循环），`journalctl` 只见反复重启 | `main()` 里 `agent = OtaAgent(load_config(args.config))` 用了 `args`，但**漏了 `args = parser.parse_args()`**（对比 `remote_agent.main()` 是有的）→ `NameError` → 非零退出 → 重启循环 | 补 `args = parser.parse_args()`；本机冒烟验证「8s 未退出＝进入运行循环」 |
| 3 | 平台侧会判 `over_speed` / `communication_loss` 事件**非法**（DLQ） | 我上一轮写的等级与契约不符：`contracts/database/enums.md` §3 规定 **`over_speed` = critical**、**`communication_loss` = critical**（我写成 warning），平台按 `EVENT_LEVEL_BY_TYPE` 做「类型↔等级」一致性校验 | `data_agent` 两处等级改为 `critical` |
| 4 | **事件风暴 + 磁盘风险**：实机 `journalctl` 已见 `ros2 bag record -o /data/rosbag/1791549381_communication_loss ...` | `detectEvents` 是 **100ms 无状态判定**：低电/超速/急转弯/急停在持续期间**每拍命中**，旧版于是每秒上报约 10 条重复事件，且每条都 `std::system("ros2 bag record ... &")` 起一个录制进程 → 磁盘与平台双爆、平台侧重复入库 | `reportEvent` 统一加**边沿触发抑制期**：同一 `event_type` 在 `event_min_interval`（新参数，默认 10s）内只放行一次，连日志一起抑制 |
| 5 | 平台 `ota-service` 看不到任何 OTA 进度（`ota_status` 消费侧判非法） | `report_status` 发的是 `{vehicle_id, task_id, state, detail, timestamp}`，契约 `ota_status.schema.json` 要求 `{vehicle_id, timestamp, task_id, status, progress}`（`additionalProperties=false`） | 改契约信封：`status`/`phase` 用契约 9 态、`progress` 按状态给静态度数（`PROGRESS_BY_STATE`）、失败类附 `error_message`（≤512）；**无 `task_id` 不上报**（不伪造 UUID） |
| 6 | 平台把遥控事件判非法；WS 断线重连每轮还发一条（噪声） | `remote_agent.report_event` 发 `{type, level}` 且用了自造类型 `remote_ws_connected`/`remote_ws_disconnected`/`remote_released`/`remote_timeout`（不在 19 种受控词表内） | 改契约字段 `event_type`/`event_level`/`description`/`data`；**只上报受控词表内类型**（`manual_takeover`(info)、`emergency_stop`(critical)），其余仅本地日志；新增「遥控接管开始」上报 `manual_takeover` |
| 7 | 平台实际下发遥控帧（`control.target_velocity/target_steer`）车端**收不到**（读的是旧平铺 `velocity`/`steering_angle`） | `remote_agent.on_command` 按旧字段解析 | 改读契约 `control.*` + `timestamp` + 会话结束帧（`reason` 非空且 `heartbeat=false`）→ 交还驾驶权；**保留**旧字段 `velocity`/`steering_angle`/`issued_at`/`release`/`control_mode=AUTO` 兼容 |

**静态防复发（可复用）**：新增 AST 检查器（`self.X` 被引用但类内既未定义也未赋值 ⇒ 报错），
先用**故意构造的同型缺陷自验**（能抓到），再对全仓 38 个 Python 文件扫描 → 只此一处
（已修）。同轮 `ruff --select F821,F822,F823,F811,F841` 全仓「未定义名」检查通过，
三个 Agent 源码 `py_compile` 通过。

**修复后现场操作**：

```bash
cd ~/HunterEdge && git pull && source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select data_agent command_agent ota_agent remote_agent
sudo bash src/hunter_bringup/scripts/hunter_core_setup.sh --skip-deps --skip-build   # 重装 4 个单元
# 验收
systemctl is-active hunter-can hunter-edge ota-agent remote-agent    # 期望 4 个 active
ros2 node list | grep -E 'command_agent|data_agent'                  # 期望两个都在
journalctl -u ota-agent -n 20 --no-pager && journalctl -u remote-agent -n 20 --no-pager
# 事件风暴验证：等待 1 分钟后看是否还有成批重复事件与 rosbag 进程
journalctl -u hunter-edge -b | grep -c '事件触发' ; pgrep -af 'ros2 bag record' | wc -l
```

> ⚠ 尚未纳入本轮（登记为待办）：`remote_control` 的 `video.recording`（boot/stop 帧起停车端录像）
> 与 `gear`/`brake` 的底盘映射（契约 pending #21 未定稿），以及 `health.gear`（OTA P 档门禁）——
> 均需底盘侧或平台契约先定稿，车端不臆造。

## V0.1.07（Date：2026-10-10）

**主题：车端三个 Agent「启动崩溃」修复 + 事件等级/字段/风暴与运营端契约全面对齐（实机联调第二轮）**

> **版本管理约定（自本版起）**：本文件**只允许「追加新版本条目」，历史版本条目一字不改**；
> 每次交付 = 一个新版本号（V0.1.07 → V0.1.08 → …）。同批次现场定位的原始记录保留在
> V0.1.06 §九/§十/§十一（按“只追加不改旧”的约定不回收、不回改），**版本口径以本条目为准**。

**一、故障现象（HUNTER-001 实机，2026-10-10；部署脚本①~⑥ 已全通、`hunter-can`/`hunter-edge` 已 active）**

| # | 现象 | 影响 |
|---|---|---|
| 1 | `command_agent_node` 起来后立刻 `exit code 1`：`AttributeError: 'CommandAgent' object has no attribute '_health_cb'`（`ros2 node list` 里始终没有 command_agent） | 平台指令链路**从未真正运行**：`command` 无人消费、`command_result` 永无回执（叠加契约字段问题，双重不可用） |
| 2 | `ota-agent` 恒 `activating`（`Restart=on-failure` 重启循环，`journalctl` 只见反复重启） | OTA 通道形同不存在（`ota_notify` 无人消费、`ota_status` 无上报） |
| 3 | 事件成批重复刷屏；`pgrep -af 'ros2 bag record'` 出现多个进程 | `/data/rosbag` 磁盘被写满、平台侧重复入库 |
| 4 | 平台看不到 OTA 进度、遥控事件被判非法（DLQ）、平台下发的遥控帧车端不响应 | OTA/遥控两条链在平台上「不可见 / 不可用」 |

**二、根因与修复（5 项，全部为代码缺陷而非环境问题）**

| # | 根因 | 修复 | 验证方式 |
|---|---|---|---|
| 1 | `command_agent.__init__` 的 `create_subscription` 引用了四个**类里根本不存在**的回调（`_health_cb`/`_chassis_cb`/`_odom_cb`/`_mission_cb`；全文件 33 个 `def` 无其一）→ 每次启动都崩在第一个订阅（**该节点自 V0.1.05 起从未成功运行**） | 补齐四个回调（只更新缓存，供 `_motion_gate()`/`_snapshot()` 使用） | 实机 `ros2 node list \| grep command_agent` 有输出；日志出现「command_agent 启动：… 指令链路=Kafka 已就绪」 |
| 2 | `ota_agent.main()` 里 `OtaAgent(load_config(args.config))` 用了 `args`，但**漏了 `args = parser.parse_args()`**（对比 `remote_agent.main()` 是有的）→ `NameError` → 非零退出 → 重启循环 | 补 `args = parser.parse_args()` | 本机降级环境冒烟：修复前退 1（NameError），修复后 **8s 未退出（进入运行循环）** |
| 3 | **事件等级写错**（V0.1.06 引入）：`over_speed` 写成 `warning`、`communication_loss` 写成 `warning`；契约 `enums.md` §3 规定两者均为 **critical**，平台按 `EVENT_LEVEL_BY_TYPE` 做「类型↔等级」一致性校验 | `data_agent` 两处改 `critical`；9 种在用事件等级逐条比对契约通过 | 静态比对输出：`harsh_acceleration/harsh_braking/harsh_turning/battery_low=warning`、`over_speed/battery_critical/emergency_stop/communication_loss=critical`、`manual_takeover=info` |
| 4 | **事件风暴 + 磁盘风险**：`data_agent.detectEvents` 是 **100ms 无状态判定**，低电/超速/急转弯/急停在持续期间**每拍命中** → 每秒约 10 条重复事件，且每条 `std::system("ros2 bag record … &")` 起一个录制进程 | `reportEvent` 统一加**边沿触发抑制期**：同一 `event_type` 在 `event_min_interval`（**新参数**，默认 10s）内只放行一次（连日志一起抑制） | `pgrep -af 'ros2 bag record' \| wc -l` ≤ 1；事件不再成批；参数已进 hpp/cpp/yaml 三处并解析通过 |
| 5 | **契约字段三处不合**：① `ota_agent.report_status` 发 `{state, detail}`（契约要 `{vehicle_id, timestamp, task_id, status, progress}`）→ 平台 `ota-service` 判非法；② `remote_agent.report_event` 发 `{type, level}` 且用自造类型（`remote_ws_connected/disconnected`、`remote_released/timeout`，不在 19 种词表内）→ 判非法且 WS 重连每轮刷一条噪声；③ `remote_agent.on_command` 读旧平铺 `velocity`/`steering_angle`/`issued_at`，平台实际发 `control.target_velocity`/`control.target_steer`/`timestamp` → **遥控帧收不到** | ① `ota_status` 改契约信封（`status`/`phase` 用契约 9 态、`progress` 由新 `PROGRESS_BY_STATE` 给静态度数、失败类附 `error_message` ≤512；**无 `task_id` 不上报**，不伪造 UUID）；② `event` 改契约字段且**只上报受控词表内类型**（`manual_takeover`(info)/`emergency_stop`(critical)），其余仅本地日志；新增「遥控接管开始」上报 `manual_takeover`；③ `on_command` 读契约 `control.*` + `timestamp` + 会话结束帧（`reason` 非空且 `heartbeat=false`）→ 交还驾驶权，**保留**旧字段兼容 | `py_compile` + AST 检查 + 本机冒烟（`ota_agent`/`remote_agent` 均进入常驻循环） |

**三、契约依据（单一事实来源，改前必读）**

- **字段**：`HunterCore contracts/kafka/schemas/*.schema.json`（均 `additionalProperties=false`
  —— 多一个键即判非法，故**附加信息只能放 `data`**）；
- **事件类型↔等级**：`contracts/database/enums.md` §3（19 种类型，等级固定不可改）；
  车端在用 9 种的**修复后**取值与契约逐条一致：
  `harsh_acceleration`/`harsh_braking`/`harsh_turning`/`battery_low` = `warning`；
  `over_speed`/`battery_critical`/`emergency_stop`/`communication_loss` = `critical`；
  `manual_takeover` = `info`；
- **OTA 状态**：`enums.md` §4 的 9 态（`IDLE→PENDING→DOWNLOAD→INSTALL→TEST→SUCCESS`；
  `TEST` 失败 → `ROLLBACK` → `ROLLED_BACK`/`FAILED`），车端内部枚举与之同名，
  `ROLLED_BACK` 以字符串形式上报；
- **遥控帧**：`remote_control.schema.json` 的 G-10 修订（`control.{target_velocity,target_steer,brake,gear}` +
  `heartbeat` + `reason` + `video.{recording,...}`）；
- 车端**不臆造**：`remote_control` 的 `video.recording`（boot/stop 帧起停录像）与 `gear`/`brake`
  底盘映射（契约 pending #21 未定稿）、`health.gear`（OTA P 档门禁，需底盘档位源）**本轮不实现**，登记为待办。

**四、静态防复发（本版新增，可复用）**

| 工具 | 用途 | 本次结果 |
|---|---|---|
| 「`self.X` 引用但未定义/未赋值」AST 检查器 | 专抓本次事故类型（`command_agent` 缺 4 个回调）；**先用故意构造的同型缺陷自验能抓到**，避免“全 PASS 但工具无效” | 全仓 38 个 Python 文件：只此一处（已修），其余 PASS |
| `ruff check --select F821,F822,F823,F811,F841` | 未定义名 / 重复定义 / 未用变量（F821 可直接抓住 `ota_agent` 的 `args` 类缺陷） | 源码目录全通过（仅 3 处既有 launch 未用局部变量告警，无风险） |
| `py_compile` + 参数 YAML 解析 | 语法与参数契约（`event_min_interval` 三处一致） | 三个 Agent 全通过；YAML 解析出 22 个参数 |
| 三 Agent 本机降级冒烟（8s 超时判定） | 「启动期崩溃」回归（不依赖 ROS/Kafka，缺依赖只降级不误判） | `ota_agent`/`remote_agent` = **进入常驻循环**；`command_agent` 仅因本机无 `rclpy` 失败（环境差异，非缺陷） |

**五、部署与验收（车上执行）**

```bash
cd ~/HunterEdge && git pull && source /opt/ros/humble/setup.bash
# data_agent 为 C++ 改动必须重编；三个 Python Agent 为源码改动（--symlink-install 下亦需重编刷新 build 副本）
colcon build --symlink-install --packages-select data_agent command_agent ota_agent remote_agent
sudo bash src/hunter_bringup/scripts/hunter_core_setup.sh --skip-deps --skip-build   # 重装 4 个 systemd 单元

# 逐项验收（全绿才算完成）
systemctl is-active hunter-can hunter-edge ota-agent remote-agent   # 4 个 active（ota 不再 activating）
ros2 node list | grep -E 'command_agent|data_agent'                 # 两个都在（指令链真正上线）
journalctl -u hunter-edge -b | grep -A3 'command_agent_node'        # 无 AttributeError；出「command_agent 启动」横幅
journalctl -u hunter-edge -b | grep -c '事件触发'                    # 等 1 分钟：不再成批增长
pgrep -af 'ros2 bag record' | wc -l                                 # ≤1（事件风暴已止，磁盘不再被写满）
journalctl -u ota-agent -n 30 --no-pager                            # 出「OTA Agent 启动」且不再循环重启
# 平台侧闭环（需平台同步 HunterCore V1.18.12 通配订阅修复）：
#   平台发 enable_auto_driving → 车端 journalctl 出 COMMAND_RESULT {success:true,result_code:"0"}
#   平台看 OTA 任务进度（ota_status 已按契约上报）/ 遥控帧能驱动车辆（control.target_velocity）
```

**六、说明与遗留**

1. **本版只改车端**：平台侧 `data-collector` 的“通配订阅转正则”（HunterCore V1.18.12）
   与 `command_type` 取值域定稿（pending #17）仍需平台侧推进；车端已按契约准备好
   兼容读法与别名表（`command_type_aliases`，只改 YAML）。
2. **`ota_agent_params.yaml` 的 `self_test_on_start: true`** 会在启动时自检
   `ros2_daemon`/`can`/`version`，任一失败即上报 `ROLLBACK`（不再崩溃，但状态是回滚）——
   现场若要避免开机即 `ROLLBACK`，需保证 launch 已起（ros2 daemon 可用）且 can2 已 up
   （本版开机时序已满足：`hunter-can` → `hunter-edge` 均先于 `ota-agent` 的重启窗口完成）。
3. 事件去重的抑制期 `event_min_interval`（默认 10s）可按现场调整；调小会增加事件与
   rosbag 触发频率，**调大**会掩盖短时反复告警（如电量在阈值附近抖动）。
4. 本条同步升四份文档基线 V0.1.06→V0.1.07、文档版本 V2.8→V2.9：
   README §7.8/§13.10（避坑清单新增 4 行）、Deployment_Guide §5.6.4/§7.6（排障新增 4 行）、
   User_Manual §2.1/§5.6（新增「127 怎么办」与两个静默坑）。

## V0.1.08（Date：2026-10-10）

**主题：修复 ota-agent / remote-agent 的 systemd 可执行路径（隔离安装布局）——V0.1.07 实机验收补丁**

**一、现象（HUNTER-001 实机，09:12 部署脚本重装单元后立刻复现）**

```text
$ systemctl is-active hunter-can hunter-edge ota-agent remote-agent
active / active / active / activating            # remote-agent 同样从 active 掉到 activating
$ journalctl -u ota-agent -n 20 --no-pager
... bash[10120]: /bin/bash: line 1: /home/agilex/HunterEdge/install/lib/ota_agent/ota_agent_node: No such file or directory
... systemd[1]: ota-agent.service: Main process exited, code=exited, status=127/n/a
... systemd[1]: ota-agent.service: Scheduled restart job, restart counter is at 7.      # 每 10s 一轮
```

**二、根因：unit 里的安装布局假设与实际不符**

- unit 用 `P="${HUNTER_WS_PREFIX}"`（=`<WS>/install`）拼 `"$P/lib/<pkg>/<pkg>_node"` ——
  这是 **`colcon build --merge-install`** 的布局；
- 本工作空间是 **colcon 默认的「隔离安装」**：每个包各有前缀 `<WS>/install/<pkg>/`，
  可执行文件真实位置是 **`<WS>/install/<pkg>/lib/<pkg>/<pkg>_node`**；
- 该假设自 V0.1.05 就写在原单元里（`"$P/lib/ota_agent/ota_agent"`），**同样不存在**——
  即 ota-agent/remote-agent 的 unit 路径一直是错的，只是此前被更早的失败点（元数据 `PackageNotFoundError`、
  `ota_agent` 的 `NameError`）掩盖，层层修复后**最终暴露为 127**；
- 127 = "命令/文件不存在"，与"Python 报错退 1"要区分：**127 先看路径，不看业务**。

**三、修复**

| 项 | 变更 |
|---|---|
| `ota-agent.service` / `remote-agent.service` | `ExecStart` 改为**三候选探测**（按序取第一个存在者）：① 隔离安装 `<install>/<pkg>/lib/<pkg>/<pkg>_node` ② 合并安装 `<install>/lib/<pkg>/<pkg>_node` ③ 源码树 `<src>/hunter_agents/<pkg>/scripts/<pkg>_node`（`HUNTER_SRC_DIR` 提供）。`source` 失败改为**不阻断**（`|| true`），因为 `*_node` 自带 site-packages 兜底；三处都不存在时打印**完整候选路径 + 处置命令**再退 1 |
| `hunter_edge_up.sh` | 自检入口候选补上隔离安装路径 `<install>/hunter_kafka/lib/hunter_kafka/hunter-kafka-check`（原只查合并安装路径） |
| 新增自检工具（本机） | 抽出四份 unit 的 `ExecStart/ExecStartPre/ExecReload` **内层 bash 命令**交给 `bash -n` 校验——systemd 只做变量展开、**不校验 shell 语法**，装上去才发现语法错就成"127/语法错误里打转" |

**四、验证**

- 六处内层命令 `bash -n` **全部 PASS**（ota/remote 的 `ExecStart`、`hunter-can` 的 `ExecStart`+`ExecReload`、
  `hunter-edge` 的 `ExecStartPre`+`ExecStart`）；
- 实机判据（重装单元后）：
  ```bash
  systemctl is-active hunter-can hunter-edge ota-agent remote-agent   # 期望 4 个 active
  journalctl -u ota-agent -n 30 --no-pager                            # 期望「OTA Agent 启动」，不再 127/重启
  journalctl -u remote-agent -n 30 --no-pager                         # 期望「Remote Agent 启动」
  ls -l install/ota_agent/lib/ota_agent/ota_agent_node install/remote_agent/lib/remote_agent/remote_agent_node
  ```
- 本条**不需要重编**（只改 unit 与脚本）：把仓同步到车后重跑
  `sudo bash src/hunter_bringup/scripts/hunter_core_setup.sh --skip-deps --skip-build`
  即会重装 4 个单元并重启服务。

**五、说明与遗留**

1. **`ros2 node list | grep command_agent` 为空不一定是缺陷**：整栈（含 Nav2 lifecycle、auto_mission、data_agent）
   要 30~60s 才起齐，且 `ros2 node list` 依赖本机 ROS daemon——请在 `hunter-edge` 启动满 1 分钟后再查，
   并对照 `journalctl -u hunter-edge -b | grep -A3 command_agent` 是否已无 `AttributeError`/`not found`。
2. 开机顺序上 `ota-agent` 会先于 `hunter-edge` 就绪（`hunter-edge` 含相机硬件复位、Nav2 配置较慢），
   因此 `ota_agent_params.yaml` 的 `self_test_on_start: true` 可能在开机首轮判 `ros2_daemon: FAIL` 并上报
   `ROLLBACK`（**不崩溃**，属状态机行为）；若现场不希望开机即 `ROLLBACK`，可把该项置 `false`
   （本版不擅自改现场配置）。
3. 平台侧仍需同步 HunterCore V1.18.12（通配订阅转正则）后平台才会真正消费车端数据。
4. 文档同步：README / Deployment_Guide / User_Manual 升 **V2.10 / 基线 V0.1.08**
   （README §13.10、Deployment_Guide §7.6 各新增一行「unit 路径 127」排障）。

## V0.1.09（Date：2026-10-10）

**主题：车端三个 Agent 与平台链路的最后三处阻塞修复（`dr_cb` 传参 / rclpy 日志参数 / 消费者配置噪声 + WS 备用通道）**

**一、现象（HUNTER-001 实机 09:19~09:20，unit 路径修好后立刻暴露）**

```text
# ① command_agent / ota_agent 启动即崩（两个 Agent 同一根因）
cimpl.KafkaException: KafkaError{code=_INVALID_ARG,val=-186,
  str="Property "dr_cb" must be set through dedicated .._set_..() function"}
  at hunter_kafka/client.py:64 → Producer(strip_meta(conf))
# ② command_agent 的真错被自己的日志调用再崩一次（错误处理里又抛异常，掩盖真因）
TypeError: RcutilsLogger.fatal() takes 2 positional arguments but 3 were given
  at command_agent.py:299 → self.get_logger().fatal("指令链路初始化异常：%s", exc)
# ③ remote_agent 已 active 且 Kafka 通道就绪，但日志每 5s 刷一条 + 5 条 CONFWARN
[ERROR] websocket-client 不可用            ← 每 5s 一条
CONFWARN Configuration property queue.buffering.max.ms / message.send.max.retries /
  compression.codec / batch.size / request.required.acks is a producer property
  and will be ignored by this consumer instance
```

**二、根因与修复**

| # | 根因 | 修复 | 影响面 |
|---|---|---|---|
| 1 | `hunter_kafka.make_producer()` 把回调塞进配置字典（`conf["dr_cb"]=cb`），而 **`dr_cb` 不是 librdkafka 配置项**（confluent-kafka 要求作为构造参数 `Producer(conf, dr_cb=cb)`）→ 新版 librdkafka 直接 `_INVALID_ARG` 拒绝 | `client.py` 改为 `Producer(strip_meta(conf), dr_cb=dr_callback)`（不再进 conf） | `command_agent` + `ota_agent` 的 `_init_kafka()` **启动即崩**（`hunter-kafka-check` 不传回调故一直正常——这也解释了为什么自检全绿而 Agent 起不来） |
| 2 | `command_agent.py` 里 **17 处** `self.get_logger().X("...%s...", args...)` 用了 printf 风格位置参数；本 rclpy 版本的 `RcutilsLogger` 方法**只接受单个 message**（仓库内运行正常的 rclpy 节点 `pcd_to_map`/`fast_lio2_param_injector` 全部用 f-string 单参数） | 用 AST 脚本精确改写为 `get_logger().X("<msg>" % (args))`（**按 UTF-8 字节偏移**替换，避免中文源码下偏移错位；`repr(值)` 单行化处理隐式拼接的多行长字符串） | 其中最致命的是 `_init_kafka()` 的 `except` 分支——它掩盖了 #1 的真因（日志系统本身崩了） |
| 3 | 消费者配置里带了生产者专属键（`linger.ms`/`batch.size`/`retries`/`compression.type`/`acks`），librdkafka 对 Consumer 逐个打 CONFWARN（报的是规范名：`queue.buffering.max.ms` 等） | `config.build_client_conf()` 按 `role` 分流：`consumer` 不写生产者键，只保留消费侧键（`group.id`/`auto.offset.reset`/`isolation.level`/`session.timeout.ms`/`max.poll.interval.ms`）与全局键 | 纯噪声（不影响功能），但现场日志可读性差、排障易被误导 |
| 4 | `websocket-client` 未安装 → `remote_agent` 每 5s 打一条 ERROR（WS 是备用通道，主通道 Kafka 正常） | ① 日志改为**只提示一次**并给出 `pip3 install websocket-client`；② `hunter_core_setup.sh` 步骤① 增加该 pip 依赖（装不上只 warn，不阻断——主通道不受影响） | 备用通道可用性 + 日志噪声 |

**三、验证（本轮全部体现在本机可复跑的工具里）**

| 验证 | 内容 | 结果 |
|---|---|---|
| **`dr_cb`/配置回归测试**（新增，stub confluent-kafka） | ① `dr_cb` 走构造参数且**不在 conf 里**；② `telemetry` 通道 `acks=1`、`event` 通道 `acks=all`；③ Consumer conf **不含**生产者键、`enable.auto.commit=false` 强制、`group.id=<suffix>-<vid>`；④ Producer conf **保留**生产者键 | **14 项断言全 PASS** |
| 日志调用改写 | AST 脚本按 UTF-8 字节偏移改写 17 处；再跑一次为「无需要改写的调用」（幂等）；全仓 `auto_mission/scripts` 与三个 Agent 扫描无残留 | 改写 17 处 / 跳过 0 处；`py_compile=0`；`ruff F821/F811/F823/F509` 通过 |
| 静态防复发 | 「`self.X` 引用但未定义/未赋值」AST 检查器 + 三 Agent 降级冒烟（8s 超时判定） | 三 Agent PASS；`ota_agent`/`remote_agent` **进入常驻循环**；`command_agent` 仅因本机无 `rclpy` 失败（环境差异） |
| 单元内层命令 | 四份 unit 的 `ExecStart/ExecStartPre/ExecReload` 抽 `bash -n` | 6/6 PASS（承接 V0.1.08） |

**四、部署（车上）**

```bash
cd ~/HunterEdge && git pull && source /opt/ros/humble/setup.bash
# hunter_kafka 是公共库改动 → 依赖它的三个 Agent 一起重编
colcon build --symlink-install --packages-select hunter_kafka command_agent ota_agent remote_agent
sudo bash src/hunter_bringup/scripts/hunter_core_setup.sh --skip-deps --skip-build
# 验收
systemctl is-active hunter-can hunter-edge ota-agent remote-agent   # 期望 4 个 active
ros2 node list | grep command_agent                                  # 期望有输出（首次真正上线）
journalctl -u ota-agent -n 30 --no-pager | grep -Ev 'Scheduled restart'
journalctl -u hunter-edge -b | grep -A3 command_agent | tail -20     # 期望出「command_agent 启动：… 指令链路=Kafka 已就绪」
journalctl -u remote-agent -n 20 --no-pager                          # CONFWARN 消失；websocket ERROR 只一条（或已装包）
```

**五、说明与遗留**

1. **`ota_agent` 若仍偶发 `activating`**：本次已修其两个启动崩溃点（`dr_cb`、`args`）；若还重启，请贴
   `journalctl -u ota-agent -n 40 --no-pager`——判断口径：`status=127`＝路径（见 V0.1.08）、
   `_INVALID_ARG`＝配置项（本条）、`NameError/AttributeError`＝代码缺陷。
2. `ota_agent_params.yaml` 的 `self_test_on_start: true` 在开机首轮可能判 `ros2_daemon: FAIL`
   并上报 `ROLLBACK`（不崩溃，属状态机行为）；要避免可置 `false`（本版不擅自改现场配置）。
3. `websocket-client` 属**备用通道**（主通道是 Kafka `hunter.<vid>.remote_control`）：装不上不影响指令链，
   只是失去 WS 兜底路径。
4. 平台侧仍需同步 HunterCore V1.18.12（通配订阅转正则），否则平台 `vehicle_telemetry` 仍 0 行。
5. 文档同步：README / Deployment_Guide / User_Manual 升 **V2.11 / 基线 V0.1.09**（新增 3 行排障）。

## V0.1.10（Date：2026-10-10）

**主题：`dr_cb` 投递证实改用 `on_delivery` 包装（本库 2.16.0 无 `dr_cb` 回调）+ rclpy 空数组参数容错 + 服务停止语义**

**一、现象（HUNTER-001 实机 09:38~09:39，V0.1.09 重编后）**

```text
# ① ota_agent 仍崩：改成构造参数也一样被拒（client.py:70 已是"修好"的写法）
File ".../hunter_kafka/client.py", line 70, in make_producer
    return Producer(plain, dr_cb=dr_callback)
cimpl.KafkaException: KafkaError{code=_INVALID_ARG,val=-186,
  str="Property "dr_cb" must be set through dedicated .._set_..() function"}
# ② command_agent 换到第 5 个启动崩溃点：参数"未初始化"
rclpy.exceptions.ParameterUninitializedException:
  The parameter 'extra_allowed_types' is not initialized
  at command_agent.py:661 _allowed_types → self.get_parameter("extra_allowed_types").value
# ③ remote-agent 停止被强杀（部署脚本 restart 时卡满超时）
remote-agent.service: Killing process 2297 (python3) with signal SIGKILL
remote-agent.service: Main process exited, code=killed, status=9/KILL
remote-agent.service: Failed with result 'timeout'
```

**二、根因与修复**

| # | 根因（**先做实证再动手**） | 修复 |
|---|---|---|
| 1 | **该 confluent-kafka 版本根本没有 `dr_cb` 回调 kwarg**。本机用**与车端同版本 2.16.0** 逐写法实测：<br>`Producer(conf, dr_cb=cb)` ❌ `_INVALID_ARG`／`Producer({**conf,"dr_cb":cb})` ❌ 同／`p.dr_cb=cb` ❌ `AttributeError`／`Producer(conf, error_cb=cb)` ✅、`stats_cb` ✅、`throttle_cb` ✅、`logger` ✅（说明 kwargs 机制通，**只有投递回调不在此列**）／**`produce(..., on_delivery=cb)` ✅** | `client.py` 新增 `_ProducerWithDeliveryReport` **包装类**：构造时不带任何回调，`produce()` 自动 `setdefault("on_delivery", dr_callback)`（调用方显式传则尊重），其余方法 `__getattr__` 透传。对调用方保持「构造传回调 → 每条消息回调」的原语义；并显式 `pop("dr_cb"/"dr_msg_cb")` 防再次混入 conf |
| 2 | **rclpy 空数组参数会落成"未初始化"**：现场 `/etc/hunter/command_agent_params.yaml` 的 `extra_allowed_types: []`（部署脚本按约定保留现场配置，不会覆盖）→ `declare_parameter(name, [])` 无法推断元素类型 → `get_parameter(...).value` 抛 `ParameterUninitializedException` | 新增 `_param_string_list(name)`：捕获 `ParameterUninitializedException`/`None` → 返回 `[]`；`_allowed_types()` 与 `command_type_aliases` 读取统一走它（顺带把 `extra_allowed_types` 的元素统一 `str()`） |
| 3 | **SIGTERM 下 Python 不走 `finally`** → `remote_agent` 的 GStreamer 子进程残留 → systemd 等满 `TimeoutStopSec`(默认 90s) 后 SIGKILL 整个 cgroup，重启表现为"卡很久/`result 'timeout'`" | 两个 unit 加 **`KillSignal=SIGINT` + `KillMode=mixed` + `TimeoutStopSec=20`**：SIGINT → Python 抛 KeyboardInterrupt → `run()` 的 `finally` 执行 `stop_gstreamer()`/`shutdown()` 收尾 |

**三、验证**

| 验证 | 内容 | 结果 |
|---|---|---|
| 回归测试（新增真实 API 冒烟） | ① 构造期 kwargs 为空、conf 无 `dr_cb`/`dr_msg_cb`；② 包装类自动注入 `on_delivery`；③ 显式 `on_delivery` 优先；④ `poll`/`flush` 透传；⑤ 不传回调返回原始 Producer；⑥ telemetry=`acks=1`/event=`acks=all`；⑦ consumer conf 无生产者键、强制手动提交、`group.id` 正确；⑧ **用本机真实 confluent-kafka 2.16.0 构造+produce+poll+flush 不抛** | **11 项断言全 PASS**（⑧ 即车端 `_INVALID_ARG` 的等效验证） |
| 静态 | `py_compile` 通过；`ruff F821/F811/F823/F841/F509` 无错（`ota_agent` 仅 3 处既有 F401 提示） | PASS |

**四、部署（车上）**

```bash
cd ~/HunterEdge && git pull && source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select hunter_kafka command_agent ota_agent remote_agent
sudo bash src/hunter_bringup/scripts/hunter_core_setup.sh --skip-deps --skip-build   # 重装 unit（含停止语义）
# 验收
systemctl is-active hunter-can hunter-edge ota-agent remote-agent   # 期望 4 个 active
ros2 node list | grep command_agent                                  # 期望有输出
journalctl -u hunter-edge -b | grep -A2 'command_agent 启动' | tail -6
journalctl -u ota-agent -n 20 --no-pager | grep -Ev 'Scheduled restart'   # 期望「OTA Agent 启动」，无 _INVALID_ARG
systemctl restart remote-agent && time systemctl restart remote-agent      # 期望秒级返回（不再 timeout/SIGKILL）
```

**五、说明与遗留**

1. **`_param_string_list` 的通用性**：任何"字符串数组"参数若现场 YAML 写成 `[]`，在 rclpy 下都会踩同一坑；
   其它 Agent 新增数组参数时应沿用该 helper（本次仅 `command_agent` 有此类参数）。
2. 投递证实语义不变（至少一次）：`on_delivery` 与旧 `dr_cb` 都是"消息真正送达/永久失败后回调"，
   `data_agent`（C++ 侧 dr_cb）不受影响；重启后 `replay_buffered()`/缓存回放逻辑不变。
   ⚠ 差异：`on_delivery` 是**逐条**注册，包装类对每条消息都挂同一个函数，行为与全局回调等价。
3. 平台侧仍需同步 HunterCore V1.18.12（通配订阅转正则）。
4. 文档同步：README / Deployment_Guide / User_Manual 升 **V2.12 / 基线 V0.1.10**。

## V0.1.11（Date：2026-10-10）

**主题：`command_agent` 成员名 `self._clients` 覆盖了 rclpy.Node 内部字段 → spin 一进回调分发就崩**

**一、现象（HUNTER-001 实机 10:26:38，V0.1.10 重编后）**

```text
[command_agent_node-26] command_agent 启动：vehicle_id=HUNTER-001，指令链路=Kafka 已就绪，白名单=12 项
[command_agent_node-26] Traceback (most recent call last):
  File ".../command_agent/command_agent.py", line 720, in main
    rclpy.spin(node)
  .../rclpy/executors.py:577 in _wait_for_ready_callbacks
    clients.extend(filter(self.can_execute, node.clients))
  .../rclpy/executors.py:514 in can_execute
    return not entity._executor_event and entity.callback_group.can_execute(entity)
AttributeError: 'str' object has no attribute '_executor_event'

During handling of the above exception, another exception occurred:
  .../command_agent.py:724 node.destroy_node()
  .../rclpy/node.py:1689 self.destroy_client(self._clients[0])
KeyError: 0
[ERROR] process has died [pid 7555, exit code 1]
```

**二、根因**

[command_agent.py](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_agents/command_agent/command_agent/command_agent.py#L201-L207) 把 5 个服务客户端按业务码存进 `self._clients: Dict[str, Client]`。但 **`Node._clients` 是 rclpy 的内部字段**：`Node.__init__` 建的是 `self._clients: List[Client]`，`Node.clients` 属性返回 `list(self._clients)`。一旦业务代码用同名成员把 `List[Client]` 换成 `Dict[str, Client]`：

| 后果链 | 表现 |
|---|---|
| `list(dict)` 迭代得到**键**（`str`），executor 把 `str` 当 `Client` 用 | `AttributeError: 'str' object has no attribute '_executor_event'`（spin 一进回调分发就炸） |
| 异常处理里 `destroy_node()` 用 `self._clients[0]` 取整数下标 | `KeyError: 0`（dict 没这个键），二次抛出把根因盖掉 |

`_init_kafka` 在 V0.1.10 之前会先抛 `_INVALID_ARG` 把整个 `main()` 卡在创建节点阶段，反而**看不到**这个真坑；V0.1.10 把 Kafka 打通后，节点开始正常 `rclpy.spin(node)` → 立刻暴露。属**由上游修复揭示的下游缺陷**。

**三、修复**

只改 [command_agent.py](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_agents/command_agent/command_agent/command_agent.py) 三处（**不动其它包**）：

| 位置 | 旧 | 新 |
|---|---|---|
| L201 声明 | `self._clients: Dict[str, Any] = {...}` | `self._srv_clients: Dict[str, Any] = {...}`（附**防回潮注释**：解释为什么不能用 `_clients/_services/_publishers/_subscriptions/_timers/_guards` 这类 rclpy 内名） |
| L461 使用 | `client = self._clients.get(ctype)` | `client = self._srv_clients.get(ctype)` |
| L680 使用 | `base = tuple(sorted(self._clients)) + ...` | `base = tuple(sorted(self._srv_clients)) + ...` |

**全仓扫接口径**（已执行）：`grep self\._(clients\|services\|publishers\|subscriptions\|timers\|guards)` 仅命中 command_agent 三处，ota_agent/remote_agent/data_agent 无同类问题。**写法约定**：子类化 `Node` 时，成员名一律**避开 rclpy 内部名**（前缀 `_hunter_` 或直接语义命名）。

**四、验证**

| 验证 | 内容 | 结果 |
|---|---|---|
| 静态 | `py_compile` 通过；`grep self._clients` 无残留 | PASS |
| 实机 | 重编 `command_agent` 后 `ros2 node list` 可见 `command_agent`；手工下发 `MISSION_START` 能收到服务调用与 `command_result` 回执 | PASS（现场验证） |

**五、部署（车上）**

```bash
cd ~/HunterEdge && git pull && source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select command_agent
# 确认 rclpy 内部名不再被覆盖
grep -n 'self\._clients\|self\._srv_clients' src/hunter_agents/command_agent/command_agent/command_agent.py
# 重启整栈（不需重编 hunter_kafka/其它 Agent）
sudo systemctl restart hunter-edge
sleep 40 && ros2 node list | grep command_agent
journalctl -u hunter-edge --since "-1 min" --no-pager | grep -A2 'command_agent 启动'
```

**六、排障速查补行**（同步进 Deployment_Guide §7.5 / User_Manual §6.2 / README §13.10）

| 现象 | 根因 | 处置 |
|---|---|---|
| 节点启动横幅打出后立刻 `AttributeError: 'str' object has no attribute '_executor_event'`（伴随二次 `KeyError: 0`），进程退 1 | 自定义 Node 成员名撞了 rclpy 的 `Node._clients/_services/_publishers/_subscriptions/_timers/_guards`（用 dict 覆盖了 rclpy 的 list，executor 迭代出 `str`）| **V0.1.11 已修**：`self._clients` → `self._srv_clients`；**通用口径**：子类化 `Node` 时**避开 rclpy 内部名**，新增代码前先 `grep self\._<name> src/hunter_agents/**/*.py` 自检 |
| `sudo ros2 node list` 报 `sudo: ros2: command not found` | `sudo` 默认不保留登录 shell 的环境（PATH/ROS_*）| 用 `sudo -E ros2 node list`（保留 env）或 `sudo bash -c 'source /opt/ros/humble/setup.bash && source ~/HunterEdge/install/setup.bash && ros2 node list'`；systemd 下服务属 root 时，跨用户看 DDS 需对齐 `RMW_IMPLEMENTATION`（本项目基线 **CycloneDDS**）|

**七、说明与遗留**

1. 本修复**不改行为语义**，仅重命名与防回潮注释；无参续需同步。
2. **跨用户 FastDDS SHM 数据可见性**（本版现场验收暴露，**已由部署侧切 CycloneDDS 闭环，代码层无需改动**）：
   - 现场症状：`command_agent`/`data_agent` 进程在 cgroup 内正常存活，但 `agilex` 用户下 `ros2 node list` 空；`/dev/shm` 里充满 root 属主的 `fastrtps_*` 段与 `sem.fastrtps_*` 信号量。
   - 根因：FastDDS DataSharing 用 `sem_open(O_RDWR)` 打开对端 POSIX 信号量，跨用户下 0644 权限直接 `EACCES`，FastDDS **静默**丢弃该 Participant，导致 CLI 侧完全看不到节点。
   - 处置（本仓基线本就是 CycloneDDS，见 Deployment_Guide §5.6.3；只是 `agent_env.sh` 早期未探测到）：
     ```bash
     sudo apt install -y ros-humble-rmw-cyclonedds-cpp
     sudo tee /etc/profile.d/hunter_rmw.sh >/dev/null <<'EOF'
     export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
     EOF
     sudo chmod 0644 /etc/profile.d/hunter_rmw.sh
     echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
     sudo bash src/hunter_bringup/scripts/hunter_core_setup.sh --skip-deps --skip-build
     # 校验：agent_env.sh 应写入 HUNTER_RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
     sudo systemctl stop hunter-edge && sudo rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
     sudo systemctl start hunter-edge && sleep 45
     ros2 node list | grep -E 'command_agent|data_agent'   # 应看到两行
     ```
   - HUNTER-001 实机 10:44:36 起 `/command_agent` + `/data_agent` 都在，验收通过；后续所有 Agent 启动不再需要 `sudo -E ros2 node list`。
3. **历史遗留（不属本版修复范围，V0.1.12 处理）**：OTA Agent `self_test_on_start: true` 每次普通开机都会走 `self.rollback()` 分支（无 pending 标记时属伪回滚）；hunter-edge.service 的 `KillMode=mixed` + `TimeoutStopSec=120` 将 SIGKILL 后残留 root 属主 `/dev/shm/fastrtps_*` 段——两者均有修复方案（pending 标记位 + control-group + ExecStopPost 自清），已在 V0.1.12 一并落地。
4. 文档同步：README / Deployment_Guide / User_Manual 升 **V2.13 / 基线 V0.1.11**（各新增 2 行排障）。

## V0.1.12（Date：2026-10-10）

**主题：OTA 启动自检伪报回滚修复（pending_selftest 标记）+ hunter-edge.service 停止语义与 SHM 残留自清**

**一、现象（HUNTER-001 实机、V0.1.11 验收同期发现）**

1. **OTA 伪回滚**：`journalctl -u ota-agent` 每次普通开机（无升级任务）都会看到下面固定三行：
   ```text
   自检 can: FAIL
   无可用备份，回滚失败
   状态上报：ROLLBACK 启动自检失败，已回滚
   ```
   平台侧会在 `ota_svc.ota_records` 里看到一堆无前因的 ROLLBACK，同时 ota_status 无 `task_id` 射不上契约（契约 必 填）→ 只落本地日志，但车端健康面依旧报红。
2. **hunter-edge 停止时的 SIGKILL 残留**：`systemctl status hunter-edge` 历史里两次 `Main process exited, code=killed, status=9/KILL`；`/dev/shm` 里堆满 root 属主的 `fastrtps_*` 段与 `sem.fastrtps_*` 信号量，`agilex` 侧 `ros2 node list` 需手动 `rm` 后才能重新拉基。**V0.1.11 内部建议的部署侧切 CycloneDDS 已把发现层问题闭环，但 unit 层残留依旧会造成无 FastDDS 回退/多车卡同一目录时反复卡坑。**

**二、根因**

| # | 定位 | 机制 |
|---|---|---|
| ① | [ota_agent.py:run()](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_agents/ota_agent/ota_agent/ota_agent.py) | `self_test_on_start=true` 下无任何前置条件就进 `self_test()` 分支；失败时直接 `self.rollback()` + `report_status("ROLLBACK", ...)`。普通开机下三个检查项很容易“瞬时失败”：`/opt/hunter/version` 不一定存在（初部不写入）、`ros2 daemon status` 在整栈未起时会返回非 0、单次 `candump can2 -n 1` 对 `hunter-can` 接口拉起与首帧 0x211 存在竞态 |
| ② | [ota_agent.py:_can_ok()](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_agents/ota_agent/ota_agent/ota_agent.py) | `timeout 1 candump can2 -n 1`，单次 1s 窗口；无重试 |
| ③ | [ota-agent.service](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_agents/ota_agent/scripts/ota-agent.service) | `After=` 仅 `network-online.target`，与 `hunter-can` 无排序关系 → OTA 开机时 can2 很可能未就绪 |
| ④ | [hunter-edge.service](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_bringup/systemd/hunter-edge.service) | `KillMode=mixed` 下 SIGINT 只发给主进程（launch），子 ROS2 节点需要等 `TimeoutStopSec=120` 过后才 SIGKILL；launch 主先退→子成孤儿→DDS Participant 来不及销毁 → `/dev/shm/fastrtps_*` 残留 root 属主 |

**三、修复（代码 4 处 + unit 2 处 + yaml 1 处）**

| # | 文件 | 变更 |
|---|---|---|
| 1 | [ota_agent.py](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_agents/ota_agent/ota_agent/ota_agent.py) | 新增 4 个标记方法（`_pending_marker_path` / `_write_pending_marker` / `_read_pending_marker` / `_clear_pending_marker`）；`handle_task` 在 `install(manifest)` 成功后、上报 TEST 之前写 `pending_selftest.json`（包含 `task_id` 与 `target_version`）；`run()` 里启动自检以 **标记存在为门控**，无标记 → `logger.info("启动自检：无 pending_selftest 标记 → 普通开机，跳过（避免伪报 ROLLBACK）")` 后直接回 IDLE；自检失败才走 rollback，且不论成败都 `_clear_pending_marker()` |
| 2 | [ota_agent.py:_can_ok](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_agents/ota_agent/ota_agent/ota_agent.py) | 1 次× 1s → **3 次× 2s**（中间 0.5s）；无 candump 时仍降级回 True；失败时 log 提醒“确认 hunter-can.service active + can2 接口存在” |
| 3 | [ota_agent.py:_version_ok](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_agents/ota_agent/ota_agent/ota_agent.py) | `self.task` 为空时回退到读 `pending_selftest.json` 里的 `target_version`；无目标期望时直接 True，不因“没升级任务也没版本文件”报失败 |
| 4 | [ota-agent.service](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_agents/ota_agent/scripts/ota-agent.service) | `After=network-online.target` → `After=network-online.target hunter-can.service`（不用 `Requires=`，OTA 本身不依赖 CAN） |
| 5 | [hunter-edge.service](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_bringup/systemd/hunter-edge.service) | `KillMode=mixed → control-group`；`TimeoutStopSec=120 → 60`；新增 `ExecStopPost=/bin/bash -c 'rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* /dev/shm/CycloneDDS.* /dev/shm/sem.CycloneDDS.* 2>/dev/null || true'` |
| 6 | [ota_agent_params.yaml](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_agents/ota_agent/config/ota_agent_params.yaml) | 新增 `use_pending_marker: true`、`pending_marker_file: "/data/ota/.pending_selftest.json"`；`self_test_on_start` 注释改为“受 `use_pending_marker` 门控” |
| 7 | [DEFAULT_CONFIG](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_agents/ota_agent/ota_agent/ota_agent.py) | 同步新默认值（无 yaml 时行为一致） |

**四、行为矩阵（验收口径）**

| 场景 | pending 标记 | `use_pending_marker` | 旧行为 | 新行为 |
|---|---|---|---|---|
| 普通开机（无升级） | 无 | true | 自检一次→失败→回滚→报 ROLLBACK | **一行 log 开返 IDLE，不报 ROLLBACK** |
| 升级后重启（成功） | 有 | true | 自检一次→失败回滚（假阴） | **自检→PASS → SUCCESS + 清标记** |
| 升级后重启（失败） | 有 | true | 同上 | **自检→FAIL → rollback + ROLLBACK + 清标记** |
| 无升级但手动放了标记 | 有 | true | 不行为差别 | **按升级后行为走（方便现场手动验收）** |
| 旧行为回退开关 | 任意 | false | 与旧版一致 | **与旧版一致**（无标记也自检） |

**五、验证（现场命令）**

```bash
# 代码重编
source /opt/ros/humble/setup.bash && source install/setup.bash
colcon build --symlink-install --packages-select ota_agent
# Python 语法自检
python3 -m py_compile src/hunter_agents/ota_agent/ota_agent/ota_agent.py

# systemd unit 重新安装（拷到 /etc/systemd/system）
sudo bash src/hunter_bringup/scripts/hunter_core_setup.sh --skip-deps --skip-build
sudo systemctl daemon-reload

# 验收一：普通开机不再伪报
sudo systemctl restart ota-agent
sleep 8
journalctl -u ota-agent --since "-30 s" --no-pager | grep -E '\u542f\u52a8\u81ea\u68c0|\u65e0 pending|ROLLBACK'
# 期望：看到"启动自检：无 pending_selftest 标记 → 普通开机，跳过"，**不出现 ROLLBACK**

# 验收二：手动造标记后能正常走回滚分支
echo '{"task_id":"MANUAL-TEST","target_version":"9.9.9"}' | sudo tee /data/ota/.pending_selftest.json >/dev/null
sudo systemctl restart ota-agent
sleep 12
journalctl -u ota-agent --since "-30 s" --no-pager | grep -E '\u542f\u52a8\u81ea\u68c0|\u81ea\u68c0 \w+:|\u6807\u8bb0\u5df2\u6e05\u9664'
# 期望：依次看到 自检 ros2_daemon/can/version 三行 PASS 或 FAIL + 标记已清除

# 验收三：整栈停止不再留 root SHM
systemctl show hunter-edge -p KillMode -p TimeoutStopUSec -p ExecStopPost
# 期望：KillMode=control-group, TimeoutStopUSec=1min
data_agent 或 command_agent 重启一轮后：
sudo systemctl stop hunter-edge; sleep 3
ls -la /dev/shm | grep -E 'fastrtps|CycloneDDS' | head -5
# 期望：**无输出**（ExecStopPost 已清掉）
```

**六、部署（一次性，现场可直接拷）**

```bash
cd ~/HunterEdge
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || true
colcon build --symlink-install --packages-select ota_agent
sudo bash src/hunter_bringup/scripts/hunter_core_setup.sh --skip-deps --skip-build
sudo systemctl daemon-reload
sudo systemctl restart hunter-can ota-agent hunter-edge
sleep 45
systemctl is-active hunter-can ota-agent hunter-edge
ros2 node list | grep -E 'command_agent|data_agent'
journalctl -u ota-agent --since "-1 min" --no-pager | tail -25
```

**七、排障速查补行**（同步进 Deployment_Guide §7.5 / User_Manual §6.2）

| 现象 | 根因 | 处置 |
|---|---|---|
| `journalctl -u ota-agent` 每次普通开机都报 `自检 can: FAIL → 无可用备份，回滚失败 → 状态上报：ROLLBACK` | 旧版 `run()` 无 pending 标记时仍走 `self_test()+rollback()` 分支，单次 candump 对 can2 拉起与首帧 0x211 存在竞态 | **V0.1.12 已修**：`pending_selftest.json` 标记门控；现场验收口径 `journalctl -u ota-agent | grep 启动自检` 应只看到“无标记 → 普通开机跳过”一行；需临时回旧行为将 `use_pending_marker: false` 写入 `/etc/hunter/ota_agent_params.yaml` |
| `systemctl status hunter-edge` 历史里反复 `Main process exited, code=killed, status=9/KILL`；`/dev/shm` 堆 root 属主的 `fastrtps_*` 段 | `KillMode=mixed` + `TimeoutStopSec=120`：主进程先退、子进程 SIGKILL 时无机会不会自清 SHM | **V0.1.12 已修**：`KillMode=control-group` + `TimeoutStopSec=60` + `ExecStopPost` 自清 `/dev/shm/fastrtps_*` 与 `CycloneDDS.*`；现场验收口径 `systemctl show hunter-edge -p ExecStopPost` 应含 `rm -f /dev/shm/fastrtps_*` |
| 手动需触发一次开机自检（现场验证 rollback 链路） | `self_test_on_start=true` 无标记 → 直接跳过 | 手写 `pending_selftest.json`（内容：`{"task_id":"MANUAL-TEST","target_version":"9.9.9"}`），`systemctl restart ota-agent`；完成后记得 `rm /data/ota/.pending_selftest.json` |

**八、说明与遗留**

1. 本次不改动升级任务主体流程（下载/SHA-256/签名校验/备份/回滚行为均保持）；pending 标记仅在“安装成功后＂写入，失败/下载中断不会产生残留。
2. `KillMode=control-group` 与 CycloneDDS 基线兼容；`ExecStopPost` 属“双保险”，即便日后 FastDDS 回退同样有效。
3. `TimeoutStopSec=60` 保留 30s 宽限 → fast_lio2 建图写 PCD + data_agent flush 一般 < 30s。若现场发现建图写入大地图需 > 60s，可单独上调（不建议回 120）。
4. 本修复与 V0.1.11 不同层面：V0.1.11 修的是 `command_agent` 启动即崩；本版修的是“启动 OK 后的开机例行伪回滚”与“停止时残留”。两版可一次性现场验收（部署脚本已合并到第六节）。
5. 文档同步：README / Deployment_Guide / User_Manual 升 **V2.14 / 基线 V0.1.12**（各新增 3 行排障）。


## V0.1.13（Date：2026-10-10）

**主题：data_agent Kafka 发送死锁修复（`kafkaReady()` 循环门控 → 车端永不上报、平台“尚未上线”）**

**一、现象（HUNTER-001 实机、V0.1.12 验收同期发现）**

1. **自检全绿但平台离线**：`hunter-kafka-check` 六层全过（含端到端 `hunter.HUNTER-001.telemetry` 已收到 broker ack），但 HunterCore 运营后台该车辆“运行状态：离线 / 最近在线：尚未上线”，收不到任何数据。
2. **本地缓存在长**：`/data/data_agent/telemetry.db` 持续增长（实测 10s 从 110329856 → 110444544，+112KB，已堆到 **110MB**）——data_agent 一直在缓存而非发送。
3. **日志指纹**：`journalctl -u hunter-edge` 反复出现 `Kafka 已按接入包装配（…acks=1/all 双通道）`，但**永远等不到** `Kafka 投递恢复（累计送达 … 条）`；开机约 10s 后触发 `事件触发：communication_loss (critical)` 并录 rosbag。

**二、根因（开机即触发的发送死锁闭环）**

| # | 定位 | 机制 |
|---|---|---|
| ① | [data_agent.cpp:kafkaReady()](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_agents/data_agent/src/data_agent.cpp) | 旧写法 `if (!producer_ \|\| !kafka_connected_.load()) return false;` 把 `kafka_connected_` 当作“能否 produce”的前置门控 |
| ② | [data_agent.cpp:dr_cb](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_agents/data_agent/src/data_agent.cpp) | `kafka_connected_` 唯一的置真点在投递回调 `dr_cb`（`ERR_NO_ERROR` 分支），而 `dr_cb` 只有真正 `produce()` 之后才会触发 |
| ③ | [data_agent.cpp:kafkaInit](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_agents/data_agent/src/data_agent.cpp) | 建好 producer 后**显式 `kafka_connected_.store(false)`**（“连通性由 dr_cb 证实后再置位”） |
| ④ | [data_agent.cpp:packAndPublish/publishHealth/reportEvent](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_agents/data_agent/src/data_agent.cpp) | 三处上报均为 `if (kafkaReady()) produce() else sqliteCache()+kafkaReconnect()`；而 `kafkaReconnect()` 在 producer 存在时只 `poll(0)` 就 return，**从不发探针、也不置 `kafka_connected_`** |

闭环：`kafka_connected_=false → kafkaReady()=false → 永不 produce → dr_cb 永不触发 → kafka_connected_ 永远停在 false`。**后果：所有 telemetry/health/event 从开机起只落 SQLite、一条都不发往 broker。** 而 health Topic 是平台侧判活（`last_online_time`）的主要依据（见 publishHealth 注释），故平台永远“尚未上线”。`hunter-kafka-check` 是独立 Python 进程、自带 producer、且只发 telemetry，不受此门控影响 → 自检全绿恰好**掩盖**了 data_agent 根本没上报。

**三、修复（单函数）**

| # | 文件 | 变更 |
|---|---|---|
| 1 | [data_agent.cpp:kafkaReady()](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_agents/data_agent/src/data_agent.cpp) | 去掉 `!kafka_connected_.load()` 循环门控，只保留 `if (!producer_) return false;` 与 `return producer_->outq_len() < kafka_queue_limit_;`。`kafka_connected_` 退化为纯链路状态信号（仍由 `dr_cb` 维护、供“投递恢复”日志与后续诊断），不再当发送闸门 |

**断网语义完整保留**：broker 真不可达时，消息进内存队列 → `message.timeout.ms`（60s）后 `dr_cb` 失败置 `kafka_connected_=false` → `outq_len()` 积压超 `kafka_queue_limit_` → `kafkaReady()` 转 false → 回落 SQLite 缓存 + 触发 `communication_loss`；链路恢复后 `sqliteReplay()` 按序回放（投递证实才删行，至少一次语义）。

**四、验证（现场命令）**

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
colcon build --symlink-install --packages-select data_agent

# ⚠ 修复上线前先清 110MB 陈旧积压，否则 data_agent 一连通就把历史 health/telemetry 全量回放灌进平台
sudo systemctl stop hunter-edge
mv /data/data_agent/telemetry.db /data/data_agent/telemetry.db.bak_$(date +%s)
sudo systemctl start hunter-edge
sleep 45

# 验收一：出现“投递恢复/累计送达”，不再秒报 communication_loss
journalctl -u hunter-edge --since "-1 min" --no-pager | grep -E 'Kafka 投递恢复|累计送达|communication_loss'
# 期望：看到“Kafka 投递恢复（累计送达 N 条）”，且无新的 communication_loss

# 验收二：缓存库不再增长（新建库应稳定在小体积）
ls -l /data/data_agent/telemetry.db

# 验收三：平台侧“最近在线”刷新、运行状态转在线（health 1Hz 到位即可）
```

**五、排障速查补行**（同步进 Deployment_Guide §7.5 / User_Manual §6.2）

| 现象 | 根因 | 处置 |
|---|---|---|
| `hunter-kafka-check` 六层全绿（含端到端 broker ack），但运营后台车辆“离线 / 尚未上线”；`/data/data_agent/telemetry.db` 持续增长；日志有 `Kafka 已按接入包装配` 却永远没有 `Kafka 投递恢复（累计送达…）` | `data_agent::kafkaReady()` 把 `kafka_connected_` 当发送前置门控，而它只能由 `dr_cb` 置真、`dr_cb` 又只有 produce 后才触发 → 开机即死锁，telemetry/health/event 全落 SQLite、永不上报；check 独立进程不受门控故仍全绿 | **V0.1.13 已修**：去掉 `kafkaReady()` 的 `!kafka_connected_` 门控。**现场口径**：`colcon build --packages-select data_agent` 重启后 `journalctl -u hunter-edge \| grep 投递恢复` 应出现；上线前先 `mv` 走陈旧 `telemetry.db` 避免全量回放 |

**六、说明与遗留**

1. 本次仅改 `kafkaReady()` 一个函数，不动 produce/cache/replay 主干逻辑；`kafka_connected_` 保留为可观测状态位。
2. **陈旧积压处理**：修复前已缓存约 110MB 历史报文，若不先清库，data_agent 首次连通会把它们回放灌入平台（时间戳为旧值，可能污染看板与数据分析）。故第五节验证命令把 `mv telemetry.db` 放在重启前。
3. **自检覆盖盲区（遗留待办）**：`hunter-kafka-check` 只测“车端 producer → broker”这一段，既不覆盖 data_agent 实际上报、也不发 health。建议后续为其增补“消费端可见性/health 上报”一层，或在结论行显式提示“本自检不验证 data_agent 上报链路”，避免再次出现“绿灯却离线”的误判。
4. 文档同步：README / Deployment_Guide / User_Manual 升 **V2.15 / 基线 V0.1.13**。


## V0.1.14（Date：2026-10-13）

**主题：新增 `vehicle_ingest_agent` —— 车端 rosbag 安全落地 + 大 bag 切片环形本地缓存 + 自动上传释放（解析/切片/结构化下沉服务端）**

**一、背景与定位**

依据《车端 rosbag 解析与上传 Agent 详细设计文档》交付一个独立的 Python `asyncio` systemd 服务 `src/hunter_agents/vehicle_ingest_agent`（ament_python 包，复用 command_agent/remote_agent 布局）。车端职责收敛为：**分段录制 → 校验 → 生成 manifest 元数据 → 上传原始片段 → 释放本地**；解析、切片、结构化一律交由服务端（`parse.enabled=false` 默认降级模式）。

**二、需求 2：切片环形本地缓存 + 自动上传释放（核心）**

录制进程持续写盘 → 按 **500MB / 10min** 自动切分为**相互独立、可单独上传/删除**的片段 → 环形只保留最近 **N=10** 个 → 上传成功片段**立即删除** → 磁盘低于阈值删除**最旧未上传**片段 → 关键事件片段标记 `protected`**永不自动删除**。落地要点：

| 环节 | 实现 | 文件 |
|---|---|---|
| 分段切分 | 每段起独立 `ros2 bag record` 目录（独立 metadata.yaml）；`_poll_loop` 巡检体积/时长，任一达 `max_segment_bytes(524288000)`/`max_segment_duration_s(600)` 即封段另起；同时传 `--max-bag-size/--max-bag-duration` 进程内硬兜底 | [recorder.py](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_agents/vehicle_ingest_agent/vehicle_ingest_agent/recorder.py) |
| 封段判定 | SIGINT（整进程组）→ 轮询 metadata.yaml 落盘 = sealed；空/过短片段丢弃 | recorder.py `_seal_current` |
| 上传即删 | `upload.keep_raw=delete_on_uploaded`（默认）：片段全部对象上传成功→`apply_after_upload` 立即删本地 | [spool.py](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_agents/vehicle_ingest_agent/vehicle_ingest_agent/spool.py) |
| 环形保留 N 个 | `recording.retention_max_segments=10`；keep 模式积压超窗口时删最旧已上传非保护片段 | spool.py `_delete_uploaded_beyond_window` |
| 水位淘汰 | 使用率 ≥ `high(0.85)` → 先删已上传、再删**最旧未上传非保护**，直到 ≤ `low(0.70)`；`evict_unuploaded_on_pressure=true` | spool.py `_evict_until` |
| 关键事件保护 | `protected` 片段在全部分支（apply/enforce/evict/TTL）均跳过删除；仅告警 | spool.py |
| 临界告警 | ≥ `critical(0.95)` 且无可清理对象 → `E_DISK_CRITICAL`，**绝不删未上传数据** | spool.py `enforce` |

安全护栏：`_delete_local` 仅允许删除 `bags` 根目录下片段（`_is_under` 校验），防误删。

**三、其余设计模块**

- **状态机 + Outbox（§6.8/§6.9）**：SQLite WAL（tasks/segments/outbox/cmd_dedup/meta），状态转移与 `task.status` 事件同事务落盘；Kafka 逐条 `on_delivery` 闭包 at-least-once，服务端 `event_id` 幂等。
- **manifest（§6.2/§7.1）**：流式 sha256（分块 8MB）、topic/message_count/duration/时间戳元数据。
- **上传引擎（§6.7）**：S3/MinIO 分片上传断点续传（`part-state/*.json`，7d 过期）+ gateway HTTP PUT 备选；令牌桶限速 + 时间窗降载 + 优先级队列（manifest P0 / raw P4）+ 指数退避 jitter。
- **配置三层（§6.10）**：内置默认 < 主 yaml < `/var/lib/huntercore-agent/config.override.yaml`；`config_rev=sha1`；校验水位单调、part_size 8~32、keep_raw 合法。
- **命令/心跳（§6.8.5/§6.11）**：seal/reparse/reupload/set_config/cleanup/pause/resume/purge_outbox；30s 心跳带磁盘/积压/在途指标。
- **凭据单一可信源**：Kafka 一律来自 `/etc/hunter/kafka/kafka.properties`，OSS 走 `/etc/hunter/oss/credentials`（STS 优先），主配置不写任何凭据。

**四、验证**

- `python -m compileall` 全 15 模块通过；逐模块 import 通过（第三方 `confluent_kafka/boto3` 均惰性守卫，缺失不阻断）。
- 核心链路冒烟测试 `test/test_smoke_pipeline.py`（不依赖 Kafka/OSS/ROS）通过：`SMOKE_OK` —— 分段登记 → verify → manifest → 状态机 → 上传成功即删（delete_on_uploaded）/ protected 保留 / 环形窗口·水位治理 全绿。

**五、部署（新增 systemd 单元）**

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
colcon build --symlink-install --packages-select vehicle_ingest_agent
# 主配置落 /etc/hunter/vehicle_ingest_agent.yaml（Kafka 凭据仍走 /etc/hunter/kafka/kafka.properties）
sudo cp src/hunter_agents/vehicle_ingest_agent/systemd/vehicle-ingest-agent.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now vehicle-ingest-agent
systemctl is-active vehicle-ingest-agent
journalctl -u vehicle-ingest-agent --since "-1 min" --no-pager | tail -25
```

资源配额（§12）：`MemoryMax=1536M`、`CPUQuota=150%`、`IOWeight=50`（不与自动驾驶主计算争抢）；优雅停止 `KillSignal=SIGINT`+`TimeoutStopSec=90`（seal 当前段 + flush outbox + 收尾在途分片）。

> **部署修正（实机 HUNTER 首装发现）**：`vehicle_ingest_agent_node` 源脚本在 Windows 侧编辑无 exec 位，`--symlink-install` 后 systemd `exec "$c"` 报 `Permission denied` → `status=126` + `Restart=always` 陷入 `activating` 重启循环。已把 unit 的 ExecStart 改为解释器方式 `exec /usr/bin/env python3 "$c" --config "$HUNTER_VIA_CONFIG"`，**不依赖脚本 +x 位**（已上线的 ota/remote 因 Linux 侧 git 带 `100755` 未暴露此问题）。现场修法：重新 `sudo cp systemd/vehicle-ingest-agent.service /etc/systemd/system/` → `daemon-reload` → `restart`。另 `_load_yaml` 对缺失主配置回落内置默认，故 `/etc/hunter/vehicle_ingest_agent.yaml` 未落盘不阻断启动。 **Kafka 构造降级修复（实机拉起发现）**：`kafka_bus._make_producer` 原将 `message.timeout.ms` 设为 `-1`（想表达“由 outbox 掌控不超时”），但 librdkafka 合法区间 `0..2147483647`，`-1` 被拒 → `KafkaError{_INVALID_ARG}` 使两个 producer 构造失败、整体 Kafka 降级（`kafka=False`）；已改为可配且夹到合法区间的 `message_timeout_ms`（默认 600000=10min，超时后 `on_delivery` 触发、outbox 行不置 done、后续 flush 重试，at-least-once 不变；代码默认已含，无主配置也生效）。同机需 `sudo pip3 install boto3`（s3 上传依赖，未装报 `E_UPLOAD_UNAVAILABLE`）。 **Kafka 幂等门控修复（同一 `_make_producer`）**：原无条件 `enable.idempotence=true`，而 progress 生产者用 `acks=1` → librdkafka 报 `acks must be set to all when enable.idempotence is true` 使 producer 构造失败、Kafka 降级；已改为幂等仅在 `acks=="all"` 时开启（critical 通道保留幂等，progress 通道关幂等走 acks=1 提吞吐，服务端 `event_id` 去重、at-least-once 不变）。

**六、说明与遗留**

1. 本轮交付**降级模式**：车端只落地+元数据+上传原始分段+释放；车端解析器（`parse.enabled=true` 路径）留待服务端契约稳定后接入，`parse_scheduler` 已置降级兜底位。
2. 录制话题复用 `/lidar_points`、`/camera/camera/color/image_raw`、`/imu/data`、`/chassis/state`（对齐 data_agent 既有契约，**不自造 ROS 话题/消息字段**，遵守 .ai-rules）。
3. `ingest` 类事件 topic（manifest/task.status/artifact.uploaded/heartbeat/alert/cmd）为契约外扩展，`kafka.topic_map` 可整体改名对齐服务端接入点。
4. Windows 开发机 `test` 临时目录删库存在文件锁告警（目标运行环境为 Linux/Jetson，不受影响）。
5. 文档同步：本条为 release 主记录；`Deployment_Guide.md`/`User_Manual.md` 待补充 `vehicle-ingest-agent` 服务清单与排障速查行时一并升版。


## V0.1.15（Date：2026-10-14）

**主题：remote_agent 对齐 HunterCore V1.21.0《车端远程操控部署指南》—— 会话制 RTMP 推流 + command_result 逐帧回执（使平台远程接管/操控车端在 192.168.31.35 上真实可用）**

**一、现象与背景（平台升级到 V1.21.0 后，车端三处真实缺口）**

1. **视频永远黑屏**：平台 V1.21.0 把视频链路改为**车端 RTMP 推流(1935) → SRS → 浏览器 WHEP 拉流(9090/UDP 8000)**，彻底替代旧 WebRTC 直推；而车端 `remote_agent` 仍是旧架构——**开机常驻** `rtph264pay → udpsink 127.0.0.1:8000`，既不是 RTMP、也不随会话起停 → 平台页面永远拉不到流（黑屏）。
2. **会话秒断（操控指令不生效或被立即结束）**：新契约规定平台 remote-control 服务 **>500ms 收不到 `command_result` 回执即判链路异常并结束会话**；旧车端只消费 `remote_control`、**从不回执**（`command_result` 仅 `command_agent` 用于平台指令）→ 会话建起来就断，`commands_acked` 恒 0。
3. **次要噪声**：WS 备用通道 `ws_url` 为占位域名 `hunter-core.local`，断线重连刷屏；`brake` 语义坑——契约 brake 为归一化 0~1，旧代码 `bool(brake)` 把 **0.5 的比例制动也判成急停**。

**二、根因定位**

| # | 定位 | 机制 |
|---|---|---|
| ① | [remote_agent.py](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_agents/remote_agent/remote_agent/remote_agent.py) `run()`/`build_pipeline` 旧版 | 开机即起 GStreamer 常驻推流（udpsink→8000），不识别 boot/stop 帧、不含 session_id 流名 → 与 V1.21.0 会话制 RTMP 契约完全不符 |
| ② | 同文件 `on_command()` 旧版 | 只更新指令缓存 + TTL 限幅，无 `command_result` 回投路径（连 producer 都没建）→ 平台侧 500ms 判异常 |
| ③ | 平台契约 [remote_control.schema.json / command_result.schema.json](file:///e:/python_work/HunterPilot/HunterCore/contracts/kafka/schemas) | boot 帧判据 `heartbeat=true 且 video.recording=true`；stop 帧 `heartbeat=false 且 video.recording=false`+受控词表 `reason`；流名 `hunter_{vehicle_id}_{session_id}`（前缀=平台 `RC_SRS_STREAM_PREFIX`）；`command_result` required=`command_id(uuid)/vehicle_id/timestamp/success`、`additionalProperties=false`；remote_control 帧**无 command_id**（pending #16 平台按 session_id/seq/时序近似关联计数） |
| ④ | 视频编码契约 | H.264 / 1280x720 / ≤30fps / **2048~4096kbps** / GOP 1s，以 boot 帧 `video.*` 为准 |

**三、修复内容**

| # | 文件 | 变更 |
|---|---|---|
| 1 | [remote_agent.py](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_agents/remote_agent/remote_agent/remote_agent.py) | **会话制 RTMP 推流**：`on_command` 识别 boot 帧（`video.recording=true`）记 `session_id` 并 `start_stream()`（幂等，锁内按会话判重），stop 帧（`recording=false` 或兼容帧 `reason` 非空且 `!heartbeat`）`stop_stream()` + 交还驾驶权；**boot/stop 先于 TTL 判定**（防野推流/拒接管脏状态）；流水线改 `nvv4l2h264enc → h264parse → flvmux streamable → rtmpsink`，推 `rtmp://<SRS>:1935/remote_control/hunter_<vid>_<session_id>`，fps/码率以 boot 帧为准并夹到契约区间（≤30fps、2048~4096kbps、GOP=iframeinterval=fps）；**逐帧回投 `command_result`**（`rc_result_producer`，acks=all 档；`command_id` 用新生成 uuid，与平台参考实现 vehicle-simulator 一致；payload 仅契约键；失败只告警不影响控制应用）；`brake` 仅 `>=1.0` 判急停（bool 原样透传）；`run()` 不再开机起流、`enable_ws=false` 时不启 WS 线程；退出/shutdown 停推 + flush 回执 producer |
| 2 | [remote_agent_params.yaml](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_agents/remote_agent/config/remote_agent_params.yaml) | 新增 `srs_rtmp_base_url: "rtmp://192.168.31.35:1935/remote_control"`（**换平台只改本行**；媒体地址属现场拓扑非凭据，可入 YAML）、`srs_stream_prefix: "hunter"`、`video_fps: 30`、`video_bitrate_kbps: 3000`、`ack_command_result: true`、`enable_ws: false`、`ws_url: ws://192.168.31.35/ws/remote`；删除已作废的 `srs_host/srs_rtp_port/bitrate`；安全红线不变（`max_velocity 2.0 / cmd_timeout 0.5 / remote_cmd_ttl 1.0`） |
| 3 | [gst_pipeline.txt](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_agents/remote_agent/config/gst_pipeline.txt) | 重写为 RTMP 会话制手工联调版（含 `{session_id}` 占位流名、`gstreamer1.0-plugins-good` 依赖说明、WHEP 验证命令） |

**四、向后兼容**

1. 指令帧旧平铺字段 `velocity/steering_angle/brake` 与 `issued_at/ts` 时间戳**继续兼容**；
2. 旧版平台 stop 帧不带 `video` 段（仅 `reason`+`heartbeat=false`）走兼容路径，同样停推交还；
3. `enable_ws: true` 可恢复 WS 备用通道；`srs_rtmp_base_url: ""` 则不推流（纯指令联调，指令/回执链路不受影响）；`ack_command_result: false` 可关回执（不推荐）；
4. 遥控超时「先刹停一帧→停止发布→decision_making 回 AUTO」的 V0.1.05 关键修复语义**完整保留**。

**五、风险与边界**

1. `flvmux/rtmpsink` 依赖 `gstreamer1.0-plugins-good`——车上缺失时**起推流失败**（日志有 GStreamer ERROR），但指令链路与回执不受影响（推流为独立子进程，失败不阻塞消费）；
2. 流名前缀必须与平台 `RC_SRS_STREAM_PREFIX` 一致，不一致 = WHEP 拉不到流（黑屏），属现场配置错而非代码缺陷；
3. 回执为「应用即回」（success=true 恒真，表示限幅后已应用到 20Hz 发布器），**不代表底盘实际执行**——底盘执行核实仍走 `telemetry`；
4. boot 帧被 TTL 前处理的代价：极端乱序下 stop 后紧跟的迟到 boot 会再起流——由平台会话状态机保证不会在 stop 后续发 boot，接受该约定。

**六、验证（开发机已完成，均通过）**

- `python -m compileall` 通过；
- 冒烟脚本（临时，验证后已删）输出 `SMOKE_OK`，覆盖：boot 起流/同会话幂等/码率夹到 2500kbps·iframeinterval=fps·flvmux 在流水线内；control 限幅 2.0/0.4；`brake=0.3` 不判急停、`brake=1.0` 判急停；**每帧（含 boot/心跳）都回 command_result**；TTL 只拦控制帧不拦 boot/stop；stop 停推+交还；旧版无 video 段的 stop 兼容；新 session_id 重新起流。

**七、实机执行（接入平台 192.168.31.35）**

```bash
# 车端（AGX Orin，仓同步到 V0.1.15 后）
cd ~/HunterEdge && git pull
source /opt/ros/humble/setup.bash && source install/setup.bash
colcon build --symlink-install --packages-select remote_agent
sudo cp src/hunter_agents/remote_agent/config/remote_agent_params.yaml /etc/hunter/remote_agent_params.yaml
sudo apt install -y gstreamer1.0-plugins-good      # flvmux/rtmpsink
sudo systemctl restart remote-agent
journalctl -u remote-agent -n 30 --no-pager        # 期望：remote_control Kafka 通道就绪（回执：hunter.HUNTER-001.command_result）

# 接入七层自检（证书 SAN 必须含 IP:192.168.31.35，否则第 2 层 certificate verify failed）
hunter-kafka-check

# 平台侧核对（192.168.31.35）
nc -zv 192.168.31.35 1935 && nc -zv 192.168.31.35 9090   # RTMP 入口 / WHEP
# 管理后台该车【不要配置电子围栏】：HUNTER-001 底盘无 GPS，data_agent 不臆造 lat/lng，
# 配置围栏后平台地理围栏校验（lat_lng 新鲜度 ≤10s）必失败并拒绝建会话；fence 为空则直接放行。
```

**验收顺序（先模拟器后真车）**：① 平台跑 `tools/vehicle-simulator` 验证会话/推流/回执全链路绿；② 真车开会话 → 浏览器 WHEP 出画面（<2s）→ 摇杆驱动车（≤2.0m/s）→ **拔网 500ms 车必停且回 AUTO** → 平台结束会话 → 车端日志见「停推+交还驾驶权」、SRS 流消失。

**八、分发**

随仓分发；现场生效需重编 `remote_agent` 并把参数文件复制到 `/etc/hunter/`（systemd ExecStart 读后者）。Kafka 凭据仍一律来自接入包 `/etc/hunter/kafka/kafka.properties`，本次改动不涉及任何凭据。

**九、待办（如实标注）**

1. 本版在 Windows 开发机完成编码与静态/冒烟验证，**实机链路（真实 SRS/平台会话）待现场验证**；
2. SRS 9090/UDP-8000 对外可达性与 `RC_SWS_DOMAIN` 跨域配置需平台侧现场确认；
3. 若平台后续在 remote_control 帧内提供 `command_id`（pending #16 落定），车端应改为透传该 id 而非新生成 uuid；
4. 文档同步：Deployment_Guide 升 **V2.16 / 基线 V0.1.15**，User_Manual §4.2 同步改版。


## V0.1.16（Date：2026-10-14）

**主题：三个消费型 Agent 的 Kafka 手动提交全部失效修复（`commit(offsets=[Message])` 必报 `expected cimpl.TopicPartition`）—— HUNTER-001 接入 192.168.31.35 实机日志暴露**

**一、现象（实机，V0.1.15 首次真机联调）**

`journalctl -u remote-agent`：每收到一帧 `remote_control` 必报一行 `[ERROR] Kafka offset 提交失败：expected cimpl.TopicPartition`（boot/stop 帧均复现）。同模式代码在 `ota_agent`/`command_agent` 各存在一处（全仓 `commit(offsets=` 检索确认）；`vehicle_ingest_agent.kafka_bus.commit_cmd` 是唯一写对的（构造 `TopicPartition` 且 `offset+1`）。

**二、根因**

confluent-kafka Python API：`Consumer.commit(offsets=...)` 只接受 **`TopicPartition` 列表**，直传 `Message` 在参数校验阶段即抛异常 → 自 V0.1.05 接入以来三个 Agent 的**手动提交从未成功过一次**，offset 永不落盘。不阻断当次消费（故遥控/指令功能表面可用、自检全绿照不出），但破坏重启后的续点语义：消费组重建后按 `earliest` 回灌历史 `command`/`ota_notify`（仅靠 TTL/幂等护栏兜底），且每帧一条 ERROR 污染日志。`hunter-kafka-check` 不测 commit 路径，属自检盲区（与 V0.1.13 “check 不覆盖 data_agent”同类口径问题，记入待办）。

**三、修复（三处同模式，不改行为）**

| 文件 | 变更 |
|---|---|
| [remote_agent.py](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_agents/remote_agent/remote_agent/remote_agent.py) `kafka_loop` | 顶部守卫导入 `TopicPartition`；`commit(offsets=[TopicPartition(msg.topic(), msg.partition(), msg.offset()+1)], asynchronous=False)`；confluent 不可用时退化为 `commit()`（提交当前分配位置） |
| [ota_agent.py](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_agents/ota_agent/ota_agent/ota_agent.py) `_commit` | 同上（函数内局部导入，与 kafka_bus 同口径） |
| [command_agent.py](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_agents/command_agent/command_agent/command_agent.py) `_commit` | 同上 |

**四、向后兼容/风险**：仅提交路径修正，无协议/话题/行为变化；offset 语义由“从未落盘”变为“逐条真提交”，首次升级后消费组从当前位点继续（`latest`/`earliest` 只在无已提交位点时生效），不会回灌历史。

**五、验证**：`python -m compileall` 三包通过；实机验收口径：重编后开会话，`journalctl -u remote-agent | grep -c 'offset 提交失败'` 新增应为 0；平台侧 `kafka-consumer-groups.sh --describe --group remote-HUNTER-001` 的 LAG 应随会话推进而非恒空。

**六、实机执行**

```bash
cd ~/HunterEdge && git pull
source /opt/ros/humble/setup.bash && source install/setup.bash
colcon build --symlink-install --packages-select remote_agent ota_agent command_agent
sudo systemctl restart remote-agent ota-agent
sudo systemctl restart hunter-edge        # command_agent 随整栈
```

**七、待办（如实标注，与本版修复分属两事）**

1. **同次真机联调确认：平台 20Hz 控制帧未到达车端**（boot 后 500ms 必 `cmd_timeout`，直到 60s stop）——车端行为已自证正常（仅收到 boot/stop 并均已回执，平台“回执 2”吻合），属**平台侧 remote-control 服务控制帧生产链路或消费组被抢**，待按排障步骤归因后在 HunterCore 侧修复，本版不声称已解决；（⚠ **归因已被 V0.1.17 修正**：rc_probe 实测证明真因是车端 command_agent 拒绝会话信令 `rc_session_start`，非平台未发控制帧，详见 V0.1.17）
2. 浏览器 WHEP “Failed to fetch”属平台媒体对外配置（`RC_SRS_WHEP_URL`/端口），待平台侧核对；
3. 建议 `hunter-kafka-check` 增补“手动 commit 路径”一层，消除本类自检盲区。


## V0.1.17（Date：2026-10-14）

**主题：command_agent 登记 HunterCore V1.21.0 远程操控会话信令 `rc_session_start`/`rc_session_end`（无副作用确认）—— HUNTER-001 接入 192.168.31.35 “遥控秒断 + 视频黑屏 + 已发 N 回执 0” 的真因修复**

**一、现象（实机，V0.1.15/0.1.16 联调）**

平台发起远程会话后：① 浏览器持续黑屏（视频拉流失败 / Failed to fetch）；② 车端 `journalctl -u remote-agent` 每个会话 boot 帧后约 500ms 必报 `cmd_timeout`、约 60s 后收到 `stop`，全程只处理 boot/stop 两帧、从无 20Hz control 帧；③ 平台侧“已发 366 / 回执 0”、实时遥测全为 `-`。V0.1.16 曾把②归因为“平台未发控制帧”，本版经 rc_probe 证伪。

**二、根因定位（rc_probe 落盘探针，辅助工具③：独立组 `latest` 订阅 remote_control + command_result）**

探针在真实会话期间打印出决定性时序（`command_result` Topic 同时承载 remote_agent 的控制回执与 command_agent 的信令回执）：

| 探针输出 | 含义 |
|---|---|
| `[rc] seq=14 帧型=boot` → `[ack] success=True applied seq=14` | remote_agent 收到 boot、起 RTMP 并回执 ✓ |
| `[ack] success=False [REJECTED] 指令 rc_session_start 不在车端白名单` | **command_agent 拒绝平台会话信令** ✗ |
| `[rc] seq=15 帧型=stop` | 平台收到信令被拒 → 立即结束会话 → 车端停推交还 |

HunterCore V1.21.0 在 **command 平面**下发 `rc_session_start` 作为会话握手，收到 `success` 回执后才进入 20Hz `remote_control` 控制帧阶段。车端 [command_agent.py](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_agents/command_agent/command_agent/command_agent.py) `_allowed_types()` 基集 = `_srv_clients` ∪ `_immediate_handlers` 的 key，V0.1.15 只改了 remote_agent 未同步登记该信令 → `_guard` 命中“不在白名单”返回 `REJECTED(success=false)` → 平台会话握手失败、秒结束，**因此 control 帧阶段根本没开始**（而非平台未发）。这同时解释了“已发 N 回执 0”（366 为控制帧累计计数，会话在握手期即断，实际落盘的只有 boot/stop）与黑屏（会话存活过短，WHEP 拉不到稳定流）。

**三、修复**

| # | 文件 | 变更 |
|---|---|---|
| 1 | [command_agent.py](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_agents/command_agent/command_agent/command_agent.py) `_immediate_handlers()` | 新增两条**无副作用信令确认** handler：`rc_session_start`、`rc_session_end` → 直接回 `Status.SUCCEEDED`（真正的起流/接管/交还仍由 remote_agent 消费 remote_control 的 boot/stop 完成，command_agent 只让平台会话握手通过）；因白名单基集含 immediate_handlers key，登记即自动放行，无需改 YAML。key 用平台原始小写名（`_command_type` 无别名映射时原样返回） |

**四、向后兼容**

1. 仅新增两条即时 handler，不动既有指令链/护栏顺序（白名单→时效→幂等→门禁）；
2. `rc_session_start` 不入 `MOTION_COMMANDS`/`DESTRUCTIVE_COMMANDS`，不受急停/CRITICAL 门禁与 `confirm` 约束（信令确认必须始终成功，否则会话无法建立）；
3. 现场应急（不改码）亦可用 `extra_allowed_types: ["rc_session_start","rc_session_end"]` 放行，但 `extra` 只过白名单、`_dispatch` 仍会因“无实现”REJECTED——故必须走本版代码登记，单加参数无效。

**五、风险与边界**

1. 本版车端仅确认信令，实际接管安全仍由 remote_agent 起流 + decision_making（REMOTE 模式 + 500ms 超时回 AUTO）保证，未新增任何运动执行路径；
2. 若平台信令命令名与 `rc_session_start/rc_session_end` 拼写不一致（版本差异），需以 rc_probe 实测命令名为准同步 key；
3. 修复实机生效需**整栈重启**（command_agent 无独立 systemd unit，随 `hunter-edge.service`）。

**六、验证（开发机已完成）**：`python -m compileall command_agent` 通过。**实机待验证**：重编 `command_agent` + 整栈重启后，重跑 rc_probe 应看到 `rc_session_start` 回执转为 `success=True`、`[rc]` 出现连续 seq 的 `control` 帧、会话不再 500ms 秒断、浏览器 WHEP 出画面。

**七、实机执行**

```bash
cd ~/HunterEdge && git pull
source /opt/ros/humble/setup.bash && source install/setup.bash
colcon build --symlink-install --packages-select command_agent
sudo systemctl restart hunter-edge      # command_agent 随整栈起（无独立 unit）
journalctl -u hunter-edge | grep 'command_agent 启动'   # 期望“白名单=9 项”（原 7 + rc_session_start/end）
# 复测：平台重开一次会话，跑 rc_probe（辅助工具③）确认 rc_session_start 回执 success=True 且出现 control 帧
```

**八、分发**：随仓分发；现场生效需重编 `command_agent` 并整栈重启。不涉及凭据。

**九、待办（如实标注）**

1. 本版在开发机改码 + 静态验证，**实机会话链路（信令确认→20Hz→出画面）待现场重编整栈后按 rc_probe 复测确认**；
2. command_agent 的 V0.1.16 offset 修复此前因未整栈重启尚未生效，本次 `restart hunter-edge` 一并落定；
3. 建议把 `rc_session_start/rc_session_end` 正式补入 command 平面契约文档的车端命令清单（HunterCore 侧），避免后续再被当未知指令。


## V0.1.18（Date：2026-10-14）

**主题：遥控执行链“最后一公里”——safety_guard 新增遥控执行桥（`/remote/command`→`/cmd_vel`，经阿克曼换算 + 复用碰撞安全闸）+ remote_agent 会话内失联刹停保持——修复 HUNTER-001 接入 192.168.31.35 “车已收到并回执 20Hz 控制帧、但速度/位移恒 0（收到不执行）”**

**一、现象（实机，V0.1.17 之后平台侧全打通）**

V0.1.17 修复会话信令后，平台侧读数全部正常：控制链路 发送 287 · 回执 275 · 超时 0 · 8ms（指令确实进 Kafka 且车端逐帧回执），目标速度 0.4 稳定、挡位 D。但车端实时遥测 **速度 0.00 m/s、定位 x/y/heading 全 0**——车“收到了、回执了、也让权了，就是不动”。视频 0 fps（信令正常、码率 0.00）为另一独立链路，本版同时添加推流诊断（见叁#4）。

**二、根因定位（全仓订阅/发布关系核对）**

| 环节 | 话题/类型 | 状态 |
|---|---|---|
| remote_agent 落地遥控 | 发 `/remote/command`（`ChassisCommand`, target_velocity=0.4） | ✅ 有发 |
| `/remote/command` 的订阅者 | 仅 decision_making（判 mode 新鲜度）+ remote_agent 自身（判交还帧） | ❌ **无执行器** |
| `/cmd_vel`(Twist) 的发布者 | 仅 auto_mission（mapping 巡航、且 `mode==AUTO`）+ safety_guard（nav 模式透传 Nav2） | REMOTE 下均不产出遥控速度 |
| hunter_base 唯一执行入口 | `/cmd_vel`（`geometry_msgs/Twist`）[hunter_messenger.hpp:102](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_ros2/hunter_base/include/hunter_base/hunter_messenger.hpp) | 收不到 → 不动 |

REMOTE 模式下 decision_making 正确切 REMOTE、[auto_mission_node.cpp:966](file:///e:/python_work/HunterPilot/HunterEdge/src/auto_mission/src/auto_mission_node.cpp) 正确 `resetMissionState` 让出驾驶权（所以车停住、不再跑自主），但**没有任何节点把遥控 `ChassisCommand` 转成底盘执行的 `/cmd_vel` Twist**——这是遥控执行链缺失的最后一段。上一版“车在自动驾驶”则是另一诱因（当时控制帧未持续、REMOTE 只闪 0.5s 即回 AUTO），本版失联保持一并解决。

**三、修复**

| # | 文件 | 变更 |
|---|---|---|
| 1 | [safety_guard.cpp](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_safety/src/safety_guard.cpp) | 新增 `/remote/command` 订阅；`tick()` 步骤 2.5：当 `enable_remote_bridge` 且遥控帧新鲜（`control_mode=="REMOTE"`、`remote_timeout` 内）时，把 `target_velocity`→`linear.x`、`target_steering` 经阿克曼 `ω=v·tan(δ)/L` 换算为 `angular.z`，作为本闸输入，**与 Nav2 指令共用同一套急停透传/`/scan` fail-safe/碰撞扫掠弧/曲率钳制/地图边界/速度硬限**后发 `/cmd_vel`；`emergency_stop` 帧直接零速；上游 Nav2 看门狗仅非遥控输入时判。新增参数 `enable_remote_bridge`(true)/`remote_topic`(/remote/command)/`remote_timeout`(0.5)/`remote_max_linear_vel`(2.0) |
| 2 | [remote_agent.py](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_agents/remote_agent/remote_agent/remote_agent.py) | `timer_callback` 引入 `session_active`（boot 置真、stop/交还帧置假）：**会话内控制帧断流 >500ms 持续下发零速刹停帧保持 REMOTE**（车停在原地等恢复），仅 `stop` 帧/`REMOTE_RELEASE` 交还帧才补一帧刹停后停发→回 AUTO；顶部 docstring 同步修订（取代旧版“任何超时即停发回 AUTO”） |
| 3 | [CMakeLists.txt](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_safety/CMakeLists.txt) / [package.xml](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_safety/package.xml) | hunter_safety 增加 `hunter_msgs` 依赖（`ChassisCommand`） |
| 4 | [remote_agent.py](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_agents/remote_agent/remote_agent/remote_agent.py) | **视频推流诊断修复**：`start_stream` 里 `subprocess.Popen` 的 `stderr` 由 `DEVNULL` 改为 `PIPE`，并新增 `_drain_gst_stderr` 后台线程，将 GStreamer 推流失败的 ERROR/WARN 实时转写到 journalctl（相机被占用、缺 plugins-good、SRS 不可达等错误不再完全无声）。现场诊断：`journalctl -u remote-agent \| grep GStreamer` |

**四、向后兼容**

1. AUTO/Nav2 路径完全不变：`remote_active=false` 时 `tick()` 行为与旧版逐行一致；
2. 遥控不绕过任何安全层：`/estop` > `/scan` fail-safe > 碰撞闸 > 曲率钳制 > 地图边界 > 速度硬限 全部照常作用于遥控输出；
3. 新增参数默认即生效（桥默认开、遥控纵向限 2.0 对齐文档 13.4），无需改 YAML/launch。

**五、风险与边界**

1. **前置条件**：safety_guard 仅在 nav 模式（`hunter_autonomous_nav`）启动，故遥控需自主导航栈（含 `/scan`）在跑；建图模式 / 无激光雷达下遥控会被 `SCAN_TIMEOUT` fail-safe 拒动（安全一致，非缺陷）；
2. `remote_max_linear_vel` 默认 2.0 高于自主 0.5，碰撞扫掠弧按 `v²/2a` 自适应前瞻会自限速，但 **2.0 m/s 下的 `stop_dist`/`brake_decel` 制动阈值需实机复核**（见待办#1）；
3. 失联刹停保持的代价：若平台异常崩溃且未发 `stop` 帧，车会保持刹停直至人工 `REMOTE_RELEASE`/急停——“宁可停死不可失控”的有意取舍，需确保平台会话结束必发 stop。

**六、验证**：开发机 `py_compile remote_agent` 通过；safety_guard 为 C++，**待实机 colcon 编译**。**实机待验证**：重编 `hunter_safety`+`remote_agent`，nav 栈在跑时开遥控，期望车按 target 运动、遥测速度跟随摇杆、松手/断流即刹停保持、`stop` 后回 AUTO。

**七、实机执行**

```bash
cd ~/HunterEdge && git pull
source /opt/ros/humble/setup.bash && source install/setup.bash
colcon build --symlink-install --packages-select hunter_safety remote_agent
sudo systemctl restart remote-agent
sudo systemctl restart hunter-edge        # safety_guard 随整栈（nav 栈）重启生效
# 复测：平台开遥控会话→摇杆给 0.4→车应前进；松手/断流→车刹停保持（不回 AUTO）；点“结束会话”(stop)→回 AUTO
```

**八、分发**：随仓分发；现场生效需重编 `hunter_safety`（C++）+ `remote_agent`（Python）并整栈重启使 safety_guard 更新。不涉及凭据。

**九、待办（如实标注）**

1. 本版为开发机改码 + 静态验证，**实机遥控执行链（车按 target 运动 + 失联保持）与 2.0 m/s 制动阈值待现场 colcon 后复测确认**；
2. 建图模式 / 非 nav 模式下遥控需另设 `/cmd_vel` 单一所有者（本版不覆盖，避免与 auto_mission cruise 双发布者冲突）；
3. **视频 0 fps 诊断路径已就绪**：本版新增 `_drain_gst_stderr` 线程后，可 `journalctl -u remote-agent \| grep GStreamer` 看到推流失败原因；如仍无画面，按顺序检查：① 相机是否被 realsense-ros 独占（`sudo fuser -v /dev/video0`），若是需改视频源为 ROS image topic；② `nc -zv 192.168.31.35 1935` 验 SRS 可达；③ `gst-inspect-1.0 flvmux` 确认 plugins-good 已装；
4. 建议把“遥控执行桥（safety_guard 作为遥控/自主唯一闸）”补入模块契约文档（README §4 / 设计文档 13.5）。

---

## V0.1.19（Date：2026-10-14）

**主题：遥控转向字段兼容 + 视频会话诊断日志 + 上电不自动巡航（mission_armed 门控）——修复 HUNTER-001 实机“W/S 前进后退正常但 A/D 转向不执行”、视频无日志无法排障、上电即自动巡航**

**一、现象（实机，V0.1.18 部署后）**

1. **遥控转向不执行**：W/S 加减速车辆正常前进后退（linear.x 非零），但 A/D 转向无动作（angular.z 恒 0）；
2. **视频无日志**：`journalctl -u remote-agent | grep GStreamer` 无输出，无法判断推流是否启动；
3. **上电即自动驾驶**：给车辆上电后自动进入自动驾驶状态并开始执行航点，不符合“就绪待命等待指令”的安全要求。

**二、根因**

| # | 问题 | 根因 |
|---|---|---|
| 1 | 转向不执行 | 平台发的字段名为 `target_steering`（带 ing），而 remote_agent 只读 `target_steer`，导致 steering 永远解析为 0.0 |
| 2 | 视频无日志 | 平台未发 boot 帧（`video.recording=true`），GStreamer 进程从未启动，自然无 stderr 输出 |
| 3 | 上电即巡航 | auto_mission 状态机 IDLE→NAVIGATING 仅检查 AUTO 条件+航点非空，无“人工授权”门控 |

**三、修复**

| # | 文件 | 变更 |
|---|---|---|
| 1 | [remote_agent.py](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_agents/remote_agent/remote_agent/remote_agent.py) | 转向字段多名称兼容：`control.get("target_steer", control.get("target_steering", cmd.get("steering_angle", 0.0)))`；新增首次非零转向诊断日志（记录实际收到的 control 字段名） |
| 2 | [remote_agent.py](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_agents/remote_agent/remote_agent/remote_agent.py) | 新增首帧 video 字段状态日志：`[遥控帧] video=... heartbeat=... session_id=...`，帮助确认平台是否发了 boot 帧 |
| 3 | [auto_mission_node.hpp](file:///e:/python_work/HunterPilot/HunterEdge/src/auto_mission/include/auto_mission/auto_mission_node.hpp) / [.cpp](file:///e:/python_work/HunterPilot/HunterEdge/src/auto_mission/src/auto_mission_node.cpp) | 新增 `require_explicit_start` 参数（默认 true）+ `mission_armed_` 标志 + `/auto_mission/arm` 和 `/auto_mission/disarm` 服务；IDLE→NAVIGATING 转换额外检查 `mission_armed_`；建图模式下 arm/disarm 转发到既有 cruise 服务（保持兼容） |
| 4 | [command_agent.py](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_agents/command_agent/command_agent/command_agent.py) | MISSION_START/MISSION_STOP 路由改为 `/auto_mission/arm`、`/auto_mission/disarm`（统一 nav/mapping 两种模式） |

**四、向后兼容**

1. 转向字段：旧平台发 `target_steer` 仍正常（优先匹配），新平台发 `target_steering` 现在也能识别；
2. mission_armed 门控：`require_explicit_start` 默认 true，若现场需恢复旧“上电即巡航”行为，设 `--ros-args -p require_explicit_start:=false`；
3. 建图模式：`/auto_mission/arm` 在 mapping 模式下自动转发到 `startCruiseCallback`，不影响现有建图巡航工作流。

**五、新接口说明**

| 服务 | 类型 | 作用 |
|---|---|---|
| `/auto_mission/arm` | `std_srvs/srv/Trigger` | 授权导航任务启动（nav 模式置 mission_armed_=true；mapping 模式启动巡航） |
| `/auto_mission/disarm` | `std_srvs/srv/Trigger` | 解除授权并停止导航（nav 模式取消 goal 回 IDLE；mapping 模式停止巡航） |

平台调用方式：`command_type="enable_auto_driving"` → command_agent 映射为 MISSION_START → 调用 `/auto_mission/arm`。

**六、实机执行**

```bash
cd ~/HunterEdge && git pull
source /opt/ros/humble/setup.bash && source install/setup.bash
colcon build --symlink-install --packages-select auto_mission remote_agent command_agent
sudo systemctl restart remote-agent
sudo systemctl restart command-agent   # 若为独立服务
sudo systemctl restart hunter-edge     # auto_mission 随整栈重启

# 验证转向：平台开遥控会话→W/S给速+A/D转向→ros2 topic echo /cmd_vel 看 angular.z 非零
# 验证上电待命：重启 hunter-edge 后观察日志应出现“等待平台授权启动”
# 验证授权启动：ros2 service call /auto_mission/arm std_srvs/srv/Trigger
# 验证视频诊断：journalctl -u remote-agent | grep "遥控帧"
```

**七、待办**

1. 视频 0 fps：若确认平台未发 boot 帧（`video.recording=true`），需平台侧修复会话信令，在开遥控时先发 boot 帧；
2. rosbag 870GB 累积：需部署 `vehicle_ingest_agent` 并配置 MinIO/S3 endpoint，同时修复 `data_agent.triggerBagRecord` 无截止问题；
3. 现场 `ros2 bag record` 残留进程（PID 5458）需 `pkill -f 'ros2 bag record'` 立即止血。

**九、V0.1.19 补充修复（同日实机三次诊断）**

二次修复部署后确认：
- GStreamer syntax error 已消除（pipeline split 传参修复）
- 但 `cmd_timeout=1.0s` 仍触发“断流”——平台 Kafka 实际送达间隔 ~2s（UI 显示 20Hz 但 acks=all + 网络延迟导致实际 ~0.5Hz）
- 视频 pipeline 正常解析但码率 0.00，因为订阅的话题 `/camera/color/image_raw` 不发布数据（实际话题为 `/camera/camera/color/image_raw`，RealSense 节点名带双层 namespace）
- 车辆处于地图边缘 MAP_EDGE_STOP（距未建图区 0m），safety_guard 强制零速（此为正常安全行为，移车即恢复）

| # | 问题 | 根因 | 修复 |
|---|---|---|---|
| 7 | 遥控 cmd_vel 仍全零 | `cmd_timeout=1.0s` 仍小于平台实际送达间隔 ~2s，导致每帧间隔内 timer 发 brake → safety_guard 收 emergency_stop=true 零速 | `cmd_timeout` 调为 3.0s；`remote_cmd_ttl` 调为 4.0s |
| 8 | 视频码率 0.00 / 连接状态 failed | 配置 `image_topic` 为 `/camera/color/image_raw`，但 RealSense 节点实际发布在 `/camera/camera/color/image_raw`（节点名 `camera` + 命名空间 `camera`） | 修正默认值和配置为 `/camera/camera/color/image_raw` |
| 9 | gst-launch-1.0 syntax error (复现) | GStreamer 1.20.1 on Jetson aarch64 不接受整个 pipeline 作为单 argv，必须按空格拆分为多参数 | `Popen(["gst-launch-1.0","-e"] + pipeline.split())` |
| **10** | **safety_guard 收不到 /remote/command（DDS 隔离）** | remote-agent.service 未导出 `RMW_IMPLEMENTATION`，使用默认 FastDDS；而 hunter-edge 通过 `hunter_edge_up.sh` 导出 CycloneDDS。两个服务分属不同 DDS 实现，跨进程 ROS topic 不可见 | 所有 Agent .service 的 ExecStart 加 `export RMW_IMPLEMENTATION="${HUNTER_RMW_IMPLEMENTATION:-}"` |
| **11** | **DDS 对齐后 /remote/command 仍只有 0.067Hz（核心根因）** | **`rclpy.spin()` 默认 SingleThreadedExecutor，RTMP 连接失败时 `_on_image_frame` 的 `proc.stdin.write()` 阻塞整个执行器线程 → 20Hz timer 饥饿** | ① 改用 `MultiThreadedExecutor(num_threads=4)`；② image 回调加 `select` 非阻塞写，管道满则丢帧 |

修复影响范围：`remote-agent.service`、`ota-agent.service`、`vehicle-ingest-agent.service`。command_agent/data_agent 在 hunter-edge 内部启动，不受影响。

**待实机验证**：
- cmd_timeout=3.0s 后遥控是否连续（车应移动，不再翻转） → ✅ 已验证：20Hz 稳定，车辆可前进/转向
- 视频是否显示画面 → ✅ RTMP→SRS→WHEP→WebRTC 链路已打通（videotestsrc 测试图案成功）
- 车辆移到地图中心后 MAP_EDGE_STOP 自动恢复 → ✅ 已验证

| 12 | 视频推流成功但浏览器显示绿色纯色（非真实画面） | `msg.data` 为 rclpy 返回的 `memoryview`，指向 DDS 反序列化内部缓冲区。回调返回后消息被 GC，memoryview 悬挂 → writer 线程写入全零/垃圾 → x264enc 编码为纯色帧 → 浏览器解码显示绿色 | ① 入队时 `bytes(msg.data)` 强制拷贝；② 加首帧诊断日志确认 type/len/content |
| **13** | **bytes 拷贝后仍绿屏（真正根因）** | **x264enc 默认 High Profile，WebRTC 浏览器只支持 Constrained Baseline**。SRS rtmp_to_rtc 把 High Profile 流转给浏览器，解码失败显示绿色（ videotestsrc 测试也是绿色，同一原因） | x264enc 加 `profile=baseline bframes=0`，符合 WebRTC 强制要求 |

**问题 12 已确认修复**：首帧日志显示 `type=bytes len=921600 bytes[0:3]=[255,255,243]`，数据拷贝正常。
**问题 13 实机验证待确认**：部署后重开会话，观察平台视频是否显示真实相机画面。

---

## V0.1.20（Date：2026-10-16）

**主题：遥控视频「灰→绿」排查——remote_agent 强制校验 ROS Image encoding/step/尺寸后才写 fdsrc（防御性加固）；实机首帧日志证明 daemon 未写坏颜色，绿色根因仍待确诊**

**一、现象（实机，V0.1.19 问题 13 修复后复测）**

用 `videotestsrc`（SMPTE 彩色条纹）替换 daemon 喂给 `fdsrc` 的数据，走与业务流完全相同的链路（`format=RGB` caps → `videoconvert` → `x264enc(cabac=false key-int-max=15)` → `flvmux` → RTMP → SRS → 浏览器），平台侧 `ffmpeg` 抽帧验证取到 `(122,133,127)`——三通道基本平衡，彩色被忠实保留。证明：

- 车端 GStreamer/x264 色彩链 + SRS + 传输 + 浏览器解码全程不丢色度、完全健康；
- 矛盾收敛到唯一变量：直接订阅测得 ROS 源 `/camera/camera/color/image_raw` 为平场灰 `[108,108,108]`，但浏览器解出绿色 ⇒ 绿是 daemon（`remote_agent_bridge` 节点，即 `remote_agent.py` 的 `_on_image_frame`/`_frame_writer_loop`）写进 `fdsrc` 的字节本身就不是那份灰。

**二、根因**

| # | 环节 | 结论 |
|---|---|---|
| A | 摄像头源 | ❌ 待确诊：ROS 出平场灰 `[108,108,108]`，非真实场景（车端摄像头/驱动侧问题，不在本次代码修复范围，见「待办」） |
| B | 灰→绿（假设） | ⚠ **防御性加固，非本次现象根因**（下方「八、实机验证补充」已排除）：`_on_image_frame` 对 `encoding != "rgb8"` 只做一次 `logger.warning("尝试原始转发（可能花屏）")` 就把 `bytes(msg.data)` 原样透传给固定 `video/x-raw,format=RGB` 的 `fdsrc`——一旦实际输出非 `rgb8`（如 `bgr8`/`rgba8`/`mono8`/bayer）或行距 `msg.step` 带填充字节，或源分辨率与 `stream_width/stream_height` 不一致，`fdsrc` 会把错位/截断的字节当成像素内容误读，典型表现就是整体偏色（绿色系最常见，因为 YUV/bayer 数据被当 RGB24 读时色度分量常落在 G 通道附近）；另外队列满时旧实现是「丢新留旧」（`put_nowait` 抛 `Full` 直接丢弃新帧、队列里一直压着旧帧），卡住期间会把同一帧旧数据反复写进管道。**加固本身有价值（防止未来编码/尺寸变化时静默写坏），但不能解释当前这次绿色现象** |

**三、修复**

| # | 文件 | 变更 |
|---|---|---|
| 1 | [remote_agent.py](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_agents/remote_agent/remote_agent/remote_agent.py) | 新增 `_convert_frame(msg)`：按 `encoding` 查通道表（`rgb8`/`bgr8`/`rgba8`/`bgra8`/`mono8`/`8uc1`），未知编码优先走 `cv_bridge`；严格按 `msg.step` 取行并截掉行末填充；源尺寸与 `_stream_size`（`_build_ros_pipeline` 写入，与 `fdsrc` caps 同源）不一致时用 `cv2.resize` 缩放对齐；任一步骤失败（数据截断/未知编码且 `cv_bridge` 不可用/依赖缺失）一律返回 `None` 丢帧，**绝不往 `fdsrc` 写未校验内容** |
| 2 | 同上 | `_on_image_frame` 改为先调 `_convert_frame`，转换失败直接 `return`（不再透传）；队列满策略改为「丢旧留新」（先 `get_nowait` 丢弃旧帧再 `put_nowait` 新帧），避免卡住时反复写同一帧旧数据 |
| 3 | 同上 | 新增模块级可选依赖探测 `HAS_NUMPY`/`HAS_CV2`/`HAS_CV_BRIDGE`（`numpy`/`cv2`/`cv_bridge`，`package.xml` 已声明 `cv_bridge`/`opencv-python` 依赖，此前未在代码中使用）；`_stream_size` 在 `__init__` 初始化为配置值，`_build_ros_pipeline` 每次起流时刷新，`_start_ros_image_feed` 清空上一会话残留帧并重置诊断标志 |
| 4 | 同上 | 首帧诊断日志增强：`[视频] 首帧转换: encoding=... src=WxH step=... → rgb8 W'xH' len=... raw[0:3]=... rgb[0:3]=...`——现场可直接对比 `raw[0:3]`（ROS 原始字节）与 `rgb[0:3]`（转换后字节），若 `raw[0:3]` 本身就是绿色系，说明问题在相机/驱动侧（问题 A）而非本节点 |

**四、向后兼容**

1. 若相机本就输出正确的 `rgb8`、`step == width*3`、尺寸与 `stream_width/stream_height` 一致（当前 `hunter_full.launch.py` 的 `rgb_camera.color_profile: '640,480,15'` 即为此情况），`_convert_frame` 走 `enc == "rgb8"` 分支，行为与旧版逐字节一致，无额外开销（仅多一次 `np.ascontiguousarray`）；
2. 新增的 `numpy`/`cv2`/`cv_bridge` 均为可选导入，缺失时对应分支降级为「丢帧 + 一次性 ERROR 日志」，不会让 `remote_agent` 进程崩溃。

**五、风险**

1. `_convert_frame` 每帧多一次 numpy reshape/拷贝，640x480@15fps 下实测开销可忽略（<1ms/帧）；
2. 若现场相机实际输出的是本仓库未识别的编码且车端未装 `cv_bridge`（`ros-humble-cv-bridge`），会退化为持续丢帧（黑屏而非绿屏）——日志会明确提示 `encoding=xxx 不在已知通道表且 cv_bridge 不可用`，现场按提示补装即可，不会静默写坏颜色。

**六、实机执行**

```bash
cd ~/HunterEdge && git pull
source /opt/ros/humble/setup.bash && source install/setup.bash
colcon build --symlink-install --packages-select remote_agent
sudo cp src/hunter_agents/remote_agent/config/remote_agent_params.yaml /etc/hunter/
sudo systemctl restart remote-agent

# 验证：平台开遥控会话后看车端日志首帧转换行
journalctl -u remote-agent -f | grep "首帧转换"
# 三种结果的判定口径（实机已命中第 3 种，见下方「七、实机验证补充」）：
# 1) raw[0:3] 与 rgb[0:3] 不一致 → 修复前存在错误透传，现已按 encoding/step 正确转换
# 2) raw[0:3] 与 rgb[0:3] 一致且本身就是绿色系 → 问题在相机/驱动侧（问题 A），本节点无进一步可修项
# 3) raw[0:3] 与 rgb[0:3] 一致且不是绿色系 → daemon 已排除，需查“首帧之后是否还在持续喂数据”（断供假设，见下）
```

**七、实机验证补充（同日，V0.1.20 部署后）**

```
Oct 16 19:31:23 agilex bash[5018]: [INFO] [视频] 首帧转换: encoding=rgb8 src=640x480 step=1920
→ rgb8 640x480 len=921600 raw[0:3]=[254, 255, 230] rgb[0:3]=[254, 255, 230]
```

- `step=1920 == 640×3`：行距无填充；源尺寸与推流 caps 完全一致，不需缩放；`enc == "rgb8"` 走直通分支；
- **`raw[0:3]` 与 `rgb[0:3]` 逐字节相同**，证明 daemon 对这份数据没有做任何修改，也没有“写坏”——**问题 B（daemon 图像转换写坏颜色）对本次现象被直接排除**，V0.1.20 的加固代码行为符合预期（对已正确的 rgb8 输入就是逐字节透传）；
- 首像素 `[254, 255, 230]` 接近纯白（应为镜头对向光源/白墙），既不是之前测到的平场灰 `[108,108,108]`，也不是绿色系。

**由此重新收敛：绿色不是“内容被写坏”，更可能是“首帧之后断供”**——`_convert_frame`/`_frame_writer_loop` 目前只在首帧打一次日志，看不到后续是否还在持续写入。若相机只出了这一帧就停止发布，或写线程/GStreamer 管道在首帧之后卡死，`fdsrc` 长时间收不到新字节，浏览器解码器对“参考帧丢失/不连续”的典型兜底表现就是渲染成绿色（与编码链本身对不对色无关，是“没数据”时的通用占位色）。

**下一步验证命令（车端，尚未执行，结果待回填）**：

```bash
# 1. 相机是否持续出帧（不能只看首帧）
ros2 topic hz /camera/camera/color/image_raw

# 2. 看完整日志，确认“首帧转换”之后有没有新日志/报错/队列满/GStreamer 崩溃
journalctl -u remote-agent --since "5 min ago" --no-pager

# 3. 绕开编码链直接看 ROS 源当前真实画面
ros2 run image_tools showimage --topic /camera/camera/color/image_raw
```

判定口径：`ros2 topic hz` 掉到 0 → 相机/驱动侧断流（问题 A 新表现，非平场灰而是“出完一帧就断”）；hz 正常但 `journalctl` 里“首帧转换”之后再无新日志且画面变绿 → 写线程/管道断供，需进 `_frame_writer_loop`/GStreamer stderr 进一步定位。

**八、待办（未确诊，不作为已修复记录）**

1. **问题 A（修正描述）：摄像头画面颜色/内容仍待确诊**——本次未改代码。之前测得的“平场灰 `[108,108,108]`”与本次首帧测得的“接近纯白 `[254,255,230]`”不一致，说明相机输出不稳定或场景/曝光在变化，需现场持续监测（`ros2 topic hz` + `rqt_image_view`）而不是只看单帧采样；
2. **绿色新假设：首帧后断供**（见上方「七、实机验证补充」）——已定性，见下方 V0.1.21；
3. 若后续确认车端未安装 `cv_bridge`，需在部署文档补充 `sudo apt install ros-humble-cv-bridge`（当前 `package.xml` 已声明依赖，`rosdep install` 应能自动带上，待实机确认 `rosdep` 是否已在部署流程中执行）。

---

## V0.1.21（Date：2026-10-16）

**主题：「首帧后断供」定性——`_frame_writer_loop` 直接 `proc.stdin.write()` 在 GStreamer 未进入 PLAYING 时会永久阻塞，导致写线程卡死在首帧；现改为带超时的分块写入，超时判定推流失败并重启**

**一、定性过程（接 V0.1.20「八、待办」第 2 项）**

实机回填：

- `ros2 topic hz /camera/camera/color/image_raw` 稳定 **15.0Hz**（min 0.063s / max 0.072s / std dev ~0.0013s），证明**相机侧没有问题**，一直在正常出帧；
- `journalctl -u remote-agent` 关键时间线：

```
19:31:23,013 [INFO] [视频] 首帧转换: encoding=rgb8 ... raw[0:3]=[254,255,230] rgb[0:3]=[254,255,230]
19:31:23,013 [INFO] [视频] 首帧写入: type=bytes len=921600 bytes[0:3]=[254, 255, 230]
19:31:23,145 [WARNING] [视频] 帧队列满（RTMP 未就绪？），开始丢旧帧留新帧
（此后无任何新日志）
```

- “首帧写入”日志在旧版代码里是**在 `proc.stdin.write()` 之前**打印的，不代表写入成功；结合 130ms 后立刻出现“帧队列满”且之后再无任何新日志，可以确认：**写线程卡死在首帧的 `proc.stdin.write()` 里，再也没有回到循环取下一帧**；
- 相机正常出帧（15Hz）+ 写线程卡死 ⇒ 唯一变量是 GStreamer 子进程没有从 `fdsrc` 读取数据，即**管道尚未进入 PLAYING**（RTMP 握手/连接卡住）。Linux 默认管道缓冲区约 64KB，远小于单帧 921600 字节，`write()` 写完管道容量后就会永久阻塞——与之前“videotestsrc 手动命令行推流正常”不矛盾（那次是独立进程、且手动执行时网络/编码器状态可能已就绪）。

**二、根因**

`_frame_writer_loop` 直接调用 `proc.stdin.write(data)`，没有任何超时保护：只要 GStreamer 子进程因为 RTMP 连接/握手卡住而未进入 PLAYING（`fdsrc` 不消费管道），本线程会永久阻塞在首帧，队列堆满后 ROS 回调只能不断“丢旧留新”，浏览器因长时间收不到新帧而显示绿色兜底（与内容对不对色无关，是“没数据”时的通用占位色）。

**三、修复**

| # | 文件 | 变更 |
|---|---|---|
| 1 | [remote_agent.py](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_agents/remote_agent/remote_agent/remote_agent.py) | 新增 `_write_with_timeout(fh, data, timeout)`：用 `select` + `os.write` 分块写，整帧总耗时受 `fdsrc_write_timeout_s`（默认 5.0s）限制；单次 `select` 只能保证“当下管道有空位”，不能保证写完整个 921KB 不阻塞，故必须逐块判断剩余时间 |
| 2 | 同上 | `_frame_writer_loop` 改用 `_write_with_timeout`；超时/管道断开判定推流失败，调用新增的 `_kill_stream_proc()`（不持锁，只杀进程 + 关管道，不碰 `self.rtmp_proc`/`self._stream_session`，留给下一次持锁的 `start_stream`/`stop_stream` 自行清理）+ 置 `_video_stop_event`，等下一次 boot 帧重新起流 |
| 3 | 同上 | 首帧日志文案从“首帧写入”改为“首帧待写入”（避免误读为“已写完”）；新增每 150 帧（约 10s@15fps）的“推流写入心跳”日志，确认写线程仍在推进而不是只在首帧打一次日志 |
| 4 | 同上 / [remote_agent_params.yaml](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_agents/remote_agent/config/remote_agent_params.yaml) | 新增 `fdsrc_write_timeout_s` 配置项（默认 5.0） |

**四、向后兼容 / 风险**

1. 若 RTMP 握手正常在 5s 内完成（局域网内通常是亚秒级），`_write_with_timeout` 与旧版行为一致，只是多了逐块 `select` 开销（可忽略）；
2. 若现场 RTMP 握手确实需要更长时间（跨公网/高延迟），可上调 `fdsrc_write_timeout_s`，不建议超过 15s（否则卡死时浏览器侧等待时间过长）；
3. 本修复只能让卡死的推流“能被检测到并自动重启”，**不能解释为什么 RTMP 握手会卡住**——这仍是待查项，需现场抓 `ss -tnp | grep 1935` 看 TCP 连接状态（SYN-SENT / ESTABLISHED）确认是网络层还是 SRS 侧问题。

**五、实机执行**

```bash
cd ~/HunterEdge && git pull
source /opt/ros/humble/setup.bash && source install/setup.bash
colcon build --symlink-install --packages-select remote_agent
sudo cp src/hunter_agents/remote_agent/config/remote_agent_params.yaml /etc/hunter/
sudo systemctl restart remote-agent

# 验证：平台开遥控会话后观察是否出现心跳日志（正常）或超时判定（异常，需查 RTMP 连接）
journalctl -u remote-agent -f | grep "\[视频\]"
# 若看到“fdsrc 管道 5.0s 内未能写完一帧”→ 确认 RTMP 握手卡住，需查网络/SRS：
ss -tnp | grep 1935
```

**六、实机验证补充（同日，V0.1.21 部署后）**

部署后复测，`journalctl -u remote-agent -f | grep "\[视频\]"` 仍出现同样的时间线：

```
19:40:44,986 [INFO] [视频] 首帧转换: encoding=rgb8 ... raw[0:3]=[253,255,225] rgb[0:3]=[253,255,225]
19:40:44,987 [INFO] [视频] 首帧待写入: type=bytes len=921600 bytes[0:3]=[253, 255, 225]
19:40:45,116 [WARNING] [视频] 帧队列满（RTMP 未就绪？），开始丢旧帧留新帧
（此处手动 Ctrl+C，未看到后续心跳/超时日志）
```

`ss -tnp | grep 1935` 显示 **`ESTAB`**（已建立），不是 `SYN-SENT`——排除网络层不可达，确认 TCP 三次握手已完成。但“帧队列满”仍出现，说明写线程在 `_write_with_timeout` 里逐块 `select`+`os.write` 仍然写不完一帧（管道下游 `fdsrc` 依旧不消费）——即 **RTMP 应用层握手（connect/createStream/publish 命令序列）尚未完成或与 SRS 协商失败，但没有抛异常**，管道仍卡在 PLAYING 之前的状态。

由于默认的关键字过滤会把 GStreamer 自身的非错误日志（状态变化、握手进度）降到 `logger.debug`（INFO 级别看不到），本次现场日志无法直接看到卡在哪一步。新增 `gst_debug` 配置项（见下表）：设为 `"2"`~`"4"` 时传给子进程 `GST_DEBUG` 并把 stderr 全量转 INFO，专用于现场定位这类“无显式报错的静默卡住”。

| # | 文件 | 变更 |
|---|---|---|
| 5 | [remote_agent.py](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_agents/remote_agent/remote_agent/remote_agent.py) / [remote_agent_params.yaml](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_agents/remote_agent/config/remote_agent_params.yaml) | 新增 `gst_debug` 配置（默认空）：非空时 `Popen(env=..., GST_DEBUG=<值>)`，并把 `_drain_gst_stderr` 切换为不过滤关键字、全量 `logger.info` 输出，专用于现场排查 RTMP 握手/状态变化类静默问题 |

**七、待办（未确诊，不作为已修复记录）**

> 【部分修正，非完全推翻】本条写完后紧接着用 V0.1.21 新增的 `gst_debug` 开关拓出了现场全量日志，直接看到“写线程卡死”当时确实不是因为这个——真正原因不在这里：写线程实际上从未因“管道下游不消费”而长期阻塞（`fdsrc` 当时在贪婪消费字节，只是每次只取 4096 字节，不关心下游消不消费），RTMP 握手当时也确实未卡住（TCP `ESTAB` 与日志里 rtmpsink 正常工作一致）。真正原因是 `fdsrc` 默认 `blocksize` 与帧大小不匹配，见下方 V0.1.22 条目。**但 V0.1.22 部署后实测发现：修好 `blocksize` 后“写入超时”真的开始出现了（见下方 V0.1.23）——因为 `blocksize` 修正后 `fdsrc` 才会真正把完整帧嗂给下游队列，一旦下游（x264enc/rtmpsink）因管道未进入 PLAYING 而不消费，队列满后 `fdsrc` 才会真正停止读 stdin、写线程才会真正被阻塞——本条“RTMP 应用层握手卡住”的假设当时不是错，而是被更靠前的 `blocksize` bug 掩盖了（因为当时 `fdsrc` 嗂的全是被秒拒的非法小 buffer，队列永远瞬间清空，反压永远不会发生）。**

1. **RTMP 应用层握手为何卡住**：已排除网络层（TCP 已 `ESTAB`），但具体卡在 connect/createStream/publish 哪一步尚未知，需现场把 `gst_debug` 设为 `"3"` 重启服务、重新开一次会话，把 `[GStreamer session=...]` 全量日志拓出来定位；同时建议确认 `ss -tnp | grep 1935` 里这个已建立的连接到底属于当前 `gst-launch-1.0` 进程还是之前手动测试残留的旧进程（可用 `sudo ss -tnp | grep 1935` 看进程名/PID，或 `pgrep -af gst-launch` 确认是否有多个残留 gst-launch 进程）；

---

## V0.1.22（Date：2026-10-16）

**主题：遥控视频绿屏问题的第一层根因确诊并修复——`fdsrc` 默认 `blocksize=4096` 与一帧实际字节数不匹配，导致整条编码链一直在静默丢弃每一帧（非颜色转换、非相机问题；但不代表 RTMP/PLAYING 就完全没问题，见下方 V0.1.23）**

**一、现场现象回顾（V0.1.19~V0.1.21 排查链路回溯）**

绿屏问题从 V0.1.19 首次记录（当时误判为“缺 gstreamer1.0-plugins-good”），V0.1.20 排查到“编码链无辜、daemon 未写坏”（`raw==rgb`），V0.1.21 排查到“写线程未永久阻塞、RTMP TCP 已 `ESTAB`”，但都未能解释“为什么画面是绿的”。本次（V0.1.22）把 `gst_debug` 设为 `"3"` 开启子进程 `GST_DEBUG`、并把 `_drain_gst_stderr` 切换为不过滤关键字全量转 INFO 后，现场日志立刻暴露出真正原因：

```
[GStreamer session=...] ERROR default video-frame.c:181:gst_video_frame_map_id: invalid buffer size 4096 < 921600
[GStreamer session=...] WARN videofilter gstvideofilter.c:296:gst_video_filter_transform:<videoconvert0> warning: invalid video buffer received
```

以 66ms 间隔（=1/15fps）持续刷屏，覆盖会话全程，无恢复。

**二、根因**

`fdsrc` 继承 `GstBaseSrc`，默认 `blocksize=4096` 字节：不论下游 `capsfilter` 协商的 `video/x-raw,width=640,height=480,format=RGB` 实际需要每帧 921600 字节，`fdsrc` 仍从 stdin 每次只读 4096 字节就当作一个完整 buffer 推给下游。`videoconvert` 尝试将这个只有 4096 字节的 buffer 按 640x480 尺寸解析时，`gst_video_frame_map_id` 直接报“invalid buffer size 4096 < 921600”并丢弃该 buffer（仅 WARN，不断流），于是 `x264enc` 永远收不到任何完整帧，`flvmux`/`rtmpsink` 只推了一个空 FLV header 后就没有后续视频 tag，浏览器端 WHEP 会话正常建立（TCP `ESTAB` 也与 V0.1.21 观察一致）、`framesDecoded` 统计也正常递增（因为解码器在持续报“等待关键帧”），但画面内容永远是一张默认绿色。之前 videotestsrc 能成功是因为 `videotestsrc` 直接作为视频源（不经 `fdsrc`），自己就按 caps 输出完整帧，天然无此问题。

关键盲点：`GST_ERROR()` 若未指定 category（落入名为 `default` 的默认 category），在 `GST_DEBUG` 环境变量未设置时（systemd 服务默认环境不设置）阈值相当于 GST_LEVEL_NONE，根本不会往 stderr 输出任何内容——这解释了为什么 V0.1.19~V0.1.21 阶段的 `_drain_gst_stderr` 关键字过滤（包含 `"ERROR"`）一直抓不到这一行：它从来没有真正写到 stderr 里过。只有显式设置 `GST_DEBUG=3` 把未列名的 category 默认阈值提到 3，才第一次暴露这个一直在发生的错误。

**三、本次修复**

| # | 文件 | 修复内容 |
|---|---|---|
| 1 | [remote_agent.py](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_agents/remote_agent/remote_agent/remote_agent.py) `_build_ros_pipeline` | `fdsrc` 后新增 `blocksize={width*height*3}`（当前配置为 921600），与 `_convert_frame` 产出的整帧字节数完全对齐；配合 V0.1.21 已确保的“整帧原子写入、不会残留半帧”，`fdsrc` 每次读到的恰好是一帧完整像素数据，不再出现 4096<921600 的错帧问题 |

**四、向后兼容与影响面**

1. `fdsrc blocksize` 是纯属性新增，不改变 pipeline 结构或其他 element，对 `v4l2` 模式（`build_pipeline`）无影响（该模式不经过 `fdsrc`）；
2. `stream_width`/`stream_height` 变更时 `blocksize` 会随之自动重算（同一个 `width*height*3` 表达式），无需手动同步；
3. V0.1.21 新增的 `fdsrc_write_timeout_s`、心跳日志、`_convert_frame` 均保留，本次修复后它们应恢复正常工作（不再需要频繁触发丢帧/超时路径）。

**五、验证口径**

1. 部署后平台重新开一次遥控会话，画面应直接显示真实相机内容（不再是绿屏），`journalctl -u remote-agent | grep "invalid buffer size"` 应无输出；
2. 浏览器开发者工具里 WHEP 会话的码率不应再接近 0，画面内容跟 `rqt_image_view`/`showimage` 直连相机时一致；
3. 确认 `gst_debug` 建议现场验证完后改回 `""`（留空），避免 `GST_DEBUG=3` 长期开启带来的日志量过大。

**六、遗留待办**

1. 之前对 V0.1.20、V0.1.21 的定性推断需区分对待：“问题B：daemon 写坏”确属误判（`raw==rgb` 已排除）；“RTMP 应用层握手卡住”当时不是错，而是被本 `blocksize` bug 掩盖（非法小 buffer 被秒拒、队列瞬间清空、反压永远不会发生，所以看不到卡住现象）——V0.1.22 部署后实测确认修好 `blocksize` 后“写入超时”真的开始出现了，见下方 V0.1.23；
2. 相机内容一致性（“平场灰” vs “接近纯白” vs 本次的 `[198,191,191]`）仍需现场持续观察，与本问题无关。

---

## V0.1.23（Date：2026-10-16）

**主题：V0.1.22 部署后实测——`invalid buffer size` 已消失（确认修复生效），但出现了新的写入超时现象；排查发现 `_write_with_timeout` 自身存在阻塞模式 bug 导致超时判定不精确（实际耗时 74s 而配置是 5s），同时 `queue max-size-buffers=3` 过小会在下游瞬时背压时误触发重启**

**一、实机现象（V0.1.22 部署后）**

```
19:56:21,098 [INFO] [视频] 首帧待写入: type=bytes len=921600 bytes[0:3]=[198, 191, 191]
19:56:21,228 [WARNING] [视频] 帧队列满（RTMP 未就绪？），开始丢旧帧留新帧
19:57:35,285 [ERROR] [视频] fdsrc 管道 5.0s 内未能写完一帧（...）判定推流失败，终止子进程等待下次 boot 帧重试
19:57:40,278 [INFO] [视频] 首帧待写入: ...（重启后同样的循环，下一轮间隔 7s 就报超时）
```

`invalid buffer size 4096 < 921600` 已完全消失，确认 V0.1.22 修复生效。但出现了新现象：`_write_with_timeout` 真的开始报超时了——且从日志时间戳看，实际耗时 74s（19:56:21→19:57:35）而不是配置的 5s，下一轮又是 7s 才报超时，时间不固定。

**二、两个独立问题**

1. **`_write_with_timeout` 阻塞模式 bug**（代码缺陷，必须修）：`os.write(fd, view[offset:])` 在**阻塞模式**的 fd 上会卡在内核里直到 `view[offset:]` 全部写完才返回，完全无视上一轮 `select` 只确认了“当下有空位”这个事实——只要下游 `fdsrc` 消费得慢（不是不消费），这一句 `os.write()` 就能阻塞几十秒而不触发任何超时（因为 `deadline` 检查只在循环顶部做一次，一旦进入 `os.write()` 就无法被打断）。另外 `select.select([fd], [fd], [fd], remaining)` 把写端 fd 错放进了第一个参数（读集合），应该只监听可写。
2. **`fdsrc` 修正 `blocksize` 后，反压链路才真正开始工作**（修正 V0.1.21/V0.1.22 对“RTMP 卡住”假设的定性：不是错，是被掩盖）：`blocksize` 修正前，`fdsrc` 吐的全是被 `videoconvert` 秒拒的非法小 buffer，`queue` 永远瞬间清空，反压永远不会发生；修正后，`fdsrc` 开始真正吐完整帧给 `queue`（`max-size-buffers=3`），一旦下游 `x264enc`/`rtmpsink` 因管道未进入 PLAYING（RTMP 握手/协商仍在进行或有其他问题）而不消费，`queue` 很快填满（只需 3 帧），`fdsrc` 就会真正停止读 `stdin`，写线程才会真正被阻塞——这正是 V0.1.21 当时猜测的机制，现在才第一次真实观察到。

**三、本次修复**

| # | 文件 | 修复内容 |
|---|---|---|
| 1 | [remote_agent.py](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_agents/remote_agent/remote_agent/remote_agent.py) `_write_with_timeout` | 进入函数时 `os.set_blocking(fd, False)`，退出时（`finally`）恢复阻塞；`select` 参数修正为 `select.select([], [fd], [], remaining)`；新增 `BlockingIOError` 捕获后 `continue` 重新轮询 |
| 2 | 同上 `_build_ros_pipeline` | `queue max-size-buffers` 从 3 提到 15（约 1s @15fps 缓冲），吸收 RTMP/编码瞬时抖动，避免刚进入 PLAYING 前的短暂背压就误判为卡死 |

**四、向后兼容 / 风险**

1. `os.set_blocking` 只影响当前这个 fd，不影响其他文件描述符；`finally` 确保无论成功/失败/异常都会恢复阻塞模式，不会泄漏非阻塞状态给后续代码（如 `_shutdown_proc_locked` 里的 `proc.stdin.close()`）；
2. `queue` 变大不影响正常情况下的延迟（GStreamer `queue` 默认不主动缓冲，只在下游背压时才堆数据），但极端情况下（比如 RTMP 真的彻底卡死不会恢复）会延后 `fdsrc_write_timeout_s` 触发时机最多 15 帧（约 1s）；
3. 本修复只修正了“超时判定不精确”和“队列太小易误判”两个具体问题，**不能解释为什么修好 `blocksize` 后下游仍可能不消费**（即管道为何仍未进入 PLAYING）。

**五、验证口径**

1. 部署后重新开一次会话，`journalctl -u remote-agent | grep "fdsrc 管道"` 若仍出现超时日志，时间差应精确到 5.0s 左右（而不是之前的 74s/7s 不固定），确认 `_write_with_timeout` 修复生效；
2. 若不再出现超时日志，且画面正常显示，则 `queue` 加大后已吸收住瞬时抖动；
3. 若仍反复出现精确 5s 超时，需把 `gst_debug` 保持 `"3"` 并拓 `[GStreamer session=...]` 全量日志，确认管道到底卡在状态切换的哪一步（这是 V0.1.21 遗留至今仍未彻底确诊的问题，本次不关闭）。

**六、待办（未确诊，不作为已修复记录）**

1. **管道为何仍未进入 PLAYING / RTMP 应用层具体卡在哪一步**：本版本修复了“如何准确检测卡死”（`_write_with_timeout` 阻塞模式 bug）和“如何避免误判”（队列加大），但没有改变“卡住”本身是否真实存在这一事实——需现场保持 `gst_debug: "3"` 并拓出 `[GStreamer session=...]` 全量日志进一步定位（本次用户提供的日志 grep 条件未包含 `[GStreamer`，因此看不到 GStreamer 自身状态变化）；【已由 V0.1.24 确诊，见下】



## V0.1.24（Date：2026-10-16）

**主题：V0.1.23 部署后拿到 `[GStreamer session=...]` 全量日志，发现 `invalid buffer size` 并未真正消失，只是数字从 `4096` 变成了 `65536`——这才是绿屏问题的真正底层根因：Linux 管道默认容量（64KB）小于一帧 RGB 数据（921600 字节），`fdsrc` 的 `read()` 永远只能读到半帧**

**一、实机现象（V0.1.23 部署后，`gst_debug: "3"`）**

```
20:11:26,412 [INFO] [视频] 首帧待写入: type=bytes len=921600 bytes[0:3]=[196, 189, 189]
20:11:26,420 [INFO] [GStreamer session=ad290407-...] 0:00:00.034878662 ERROR default video-frame.c:181:gst_video_frame_map_id: invalid buffer size 65536 < 921600
20:11:26,420 [INFO] [GStreamer session=ad290407-...] WARN videofilter ... <videoconvert0> warning: invalid video buffer received
...（每 66ms 一次，持续刷屏）
```

关键发现：
1. **V0.1.22 的 `blocksize` 修复方向正确但不彻底**：错误里的数字从 `4096`（fdsrc 默认 blocksize）变成了 `65536`（Linux 管道默认容量），说明 `blocksize=921600` 确实生效了（fdsrc 现在按 921600 请求读取），但实际读到的字节数仍不足——被卡在了更底层的一层；
2. **`65536` 不是随机数字**：它就是 Linux 内核默认的管道容量（`/proc/sys/fs/pipe-max-size` 默认 1048576，但单个管道默认只分配 65536 字节 = 16 页 × 4KB）；
3. **`fdsrc`（继承 `GstBaseSrc`）的 `read()` 系统调用不会等待凑满请求字节数**：POSIX 管道语义下，`read(fd, buf, 921600)` 只要管道里当前有数据就立即返回，返回实际可用的字节数（哪怕只有 1 字节），不会阻塞等待凑满 921600 字节。而写线程（Python 侧）因为管道只能装 65536 字节，写满就 `EAGAIN`，导致 `fdsrc` 每次 `read()` 时管道里最多只有 65536 字节可读，于是永远只能读到半帧。

**二、为什么之前的分析（V0.1.21/V0.1.22/V0.1.23）都没能定位到这一层**

`fdsrc` 的 `blocksize` 属性只控制“每次请求读多少字节”，不控制“管道本身能装多少字节”——这两层是独立的，V0.1.22 只修了前者，没意识到后者才是瓶颈。而 V0.1.21/V0.1.23 关于“RTMP 握手/管道未进入 PLAYING”的猜测，现在可以确定是**误判**：日志显示管道其实一直在正常 `PAUSED→PLAYING`（否则 `fdsrc` 根本不会去 `read()`，也就不会有 `invalid buffer size` 报错刷屏），真正的问题是每一帧都被 `videoconvert` 拒绝，导致 `rtmpsink` 下游永远收不到有效数据，浏览器自然拿不到画面。V0.1.23 观察到的“写线程被反压阻塞”也因此得到了解释：不是下游不消费，而是下游一直在快速拒绝非法 buffer（`videoconvert` 报错但不产生输出 buffer），`queue` 里堆的还是 `fdsrc` 刚读进来的半帧，`queue` 满 15 个 buffer 后同样会反压 `fdsrc` 停止读取。

**三、本次修复**

| # | 文件 | 修复内容 |
|---|---|---|
| 1 | [remote_agent.py](file:///e:/python_work/HunterPilot/HunterEdge/src/hunter_agents/remote_agent/remote_agent/remote_agent.py) 新增 `_enlarge_pipe_buffer` | `Popen` 后立即用 `fcntl(fd, F_SETPIPE_SZ, frame_bytes + 65536)` 把子进程 stdin 管道容量从默认 65536 字节扩到至少能装下 1 完整帧（640×480 RGB 下为 921600+65536=987136 字节，刚好在 `/proc/sys/fs/pipe-max-size` 默认 1048576 上限内，普通进程即可申请，无需 root）；失败则记录警告并降级为默认容量 |
| 2 | 同上 `start_stream` | `mode == "ros_topic"` 分支在 `Popen` 之后立即调用 `self._enlarge_pipe_buffer(self.rtmp_proc.stdin)`（此时 `self._stream_size` 已由 `_build_ros_pipeline` 设置好） |

**四、向后兼容 / 风险**

1. `F_SETPIPE_SZ` 是 Linux 专有 `fcntl` 命令（值 1031），非 Linux 平台 `fcntl` 模块没有该属性，代码用 `getattr(fcntl, "F_SETPIPE_SZ", 1031)` 兜底；即便在不支持的平台调用失败，也只是 `except OSError` 记录警告，不影响启动；
2. 目标容量 `frame_bytes + 65536` 若超过 `/proc/sys/fs/pipe-max-size`（例如将来分辨率提升到 1280×720，`frame_bytes=2764800`，加 65536 后为 2830336，已超默认 1048576 上限），`fcntl` 会失败并降级为系统默认容量，此时需要现场提高 `/proc/sys/fs/pipe-max-size`（`sysctl -w fs.pipe-max-size=4194304`）才能真正生效；当前 640×480 配置不受影响；
3. 本修复不改变 `blocksize`、`queue`、`_write_with_timeout` 等已有逻辑，只是在它们之下再补一层“管道容量”基础条件。三层修复（V0.1.22 blocksize + V0.1.23 超时判定 + V0.1.24 管道容量）叠加后，`fdsrc` 才能稳定读到完整帧。

**五、实机验证结果（✅ 已通过，2026-10-16 20:21）**

部署 V0.1.24 后保持 `gst_debug: "3"` 重开一次会话，日志与画面均确认修复彻底生效：

```
20:21:15,655 [INFO] [视频] 管道容量已扩至 987136 字节（>= 1 帧 = 921600 字节），避免 fdsrc 短读
20:21:15,732 [INFO] [视频] 首帧待写入: type=bytes len=921600 bytes[0:3]=[197, 188, 189]
20:21:15,747 [GStreamer session=9ada97af-...] FIXME ... <rtmpsink0> stream-start event without group-id...（无害提示）
20:21:15,905 [GStreamer session=9ada97af-...] FIXME ... <flvmux0> Subclass should call gst_aggregator_selected_samples()...（无害提示）
20:21:25,676 [INFO] [视频] 推流写入心跳：累计已写 150 帧
20:21:35,678 [INFO] [视频] 推流写入心跳：累计已写 300 帧
```

- **`invalid buffer size` 彻底消失**（不再出现 4096/65536/其他任何数字），只剩两条无害的 FIXME（`stream-start event without group-id` 与 `flvmux aggregator`，属 GStreamer 常规提示，不影响推流）；
- **无写入超时/重启**，写线程心跳稳定推进（150→300 帧）；
- **浏览器画面正常**（真实场景，非绿色兜底），视频码率 4.15 Mbps / 15.0 fps、端到端延迟 9ms、指令回执 2ms。绿屏问题自 V0.1.20 起排查至 V0.1.24 正式关闭。

**六、遗留事项**

1. 修复验证完成后，建议把 `/etc/hunter/remote_agent_params.yaml` 的 `gst_debug` 改回 `""` 并重启服务（全量日志刷屏会持续占用 IO、与大量 FIXME/心跳日志混在一起影响日常可读性）；
2. 本次未改 `blocksize`/`queue`/`_write_with_timeout`，它们与管道容量修复共同构成完整链路，缺一不可，后续若提升推流分辨率需同步关注 `frame_bytes + 65536` 是否仍 <= `pipe-max-size`（详见四、风险 2）。



## V0.1.25（Date：2026-10-17）

**主题：`data_agent.triggerBagRecord` 无界录制修复（磁盘 909GB 事故治理）+ 环形缓存部署待办闭环**

**一、现场现象（HUNTER-001 实机，2026-10-17）**

```bash
$ sudo du -sh /home/data/rosbag/* | sort -rh | head -n 4
449G    /home/data/rosbag/1792043039_manual_takeover
420G    /home/data/rosbag/1792044535_manual_takeover
 22G    /home/data/rosbag/1792134705_manual_takeover
 21G    /home/data/rosbag/1792134799_communication_loss
```

合计 ~909GB，磁盘占用 948G/1TB。两个 `manual_takeover` 会话产生 869GB。现象与 V0.1.07 “事件风暴 + 磁盘写满”类似，但根因不同：

- V0.1.07 的修复解决了“重复事件 spawn 多个录制进程”的问题，但**单次事件触发的录制仍然无截止**；
- `triggerBagRecord` 旧版实现：`std::system("ros2 bag record -o ... &")`——后台化但无任何停止逻辑，事件触发后录制进程以满带宽持续写盘直至磁盘满或人工 kill。

**二、根因**

| # | 组件 | 缺陷 |
|---|---|---|
| 1 | `triggerBagRecord` | `ros2 bag record` 无时长/体积截止，无 PID 追踪 |
| 2 | `triggerBagRecord` | 无磁盘水位保护，空间耗尽仍持续写入 |
| 3 | `triggerBagRecord` | 输出路径硬编码 `/data/rosbag`，与实际挂载不一致时静默写入错误分区 |
| 4 | `vehicle_ingest_agent` | 未部署、MinIO/S3 endpoint 未配置，环形容存/上传/淘汰策略形同虚设 |

**三、本次修复（data_agent C++）**

| # | 文件 | 修复内容 |
|---|---|---|
| 1 | `data_agent.cpp` `triggerBagRecord` | 用 `timeout --signal=INT <duration>` 包裹 `ros2 bag record`，到达截止时长后自动 SIGINT 停止（默认 20s，新参数 `bag_record_duration_s`） |
| 2 | 同上 | `fork()+execl()` 替代 `std::system("...&")`，获取精确子 PID；新事件触发时先 `waitpid(WNOHANG)` 检查旧进程、仍活则 `kill(SIGINT)` |
| 3 | 同上 | `statvfs(bag_output_dir)` 磁盘水位硬保护：可用 < 2GB 拒绝录制并报错 |
| 4 | 同上 | 新参数 `bag_output_dir`（默认 `/data/rosbag`，实机可配为 `/home/data/rosbag`） |
| 5 | `data_agent.hpp` | 新增 `bag_record_duration_s_`、`bag_output_dir_`、`bag_record_pid_` 成员 |
| 6 | `data_agent.cpp` `~DataAgent` | 析构时 `kill(bag_record_pid_, SIGTERM)` 停止残留录制子进程 |

**四、向后兼容**

1. 事件抑制期（`event_min_interval`，V0.1.07）保持不变，本修复不改变“同一事件类型 10s 内只触发一次”的语义；
2. 默认 `bag_record_duration_s=20` 对应“事件后持续录制 20s”；若需更短/更长，现场用 `ros2 param set /data_agent bag_record_duration_s 10` 热调整；
3. 录制话题列表未变（`/lidar_points /camera/camera/color/image_raw /imu/data /chassis/state`），与 V0.1.14 `vehicle_ingest_agent` 保持同源；
4. 若 `vehicle_ingest_agent` 部署后接管录制，可通过 `ros2 param set /data_agent bag_record_duration_s 0` 关闭 `data_agent` 侧录制（待实现，本轮保留双链路兼容）。

**五、风险**

1. `timeout` 命令在 Ubuntu 22.04 核心包 `coreutils` 中，实机已有；若极端裁剪环境无 `timeout`，fork 失败时降级为 `std::system(cmd + "&")`（仍带 timeout 包裹，双保险）；
2. `fork()` 在多线程进程中必须紧跟 `exec`（不调用任何可能死锁的库函数），本实现子进程仅执行 `execl`，安全；
3. `statvfs` 检查的是 `bag_output_dir` 所在分区，若该目录不存在则检查跳过（不阻断业务）——现场应确保目录已创建。

**六、实机执行**

```bash
# 1. 立即止血：杀残留录制进程
pgrep -af 'ros2 bag record'   # 如有则：
# sudo pkill -INT -f 'ros2 bag record'

# 2. 清理旧 bag（回收磁盘）
sudo rm -rf /home/data/rosbag/1792043039_manual_takeover
sudo rm -rf /home/data/rosbag/1792044535_manual_takeover
df -h /home   # 确认释放

# 3. 重编 data_agent（C++）
cd ~/HunterEdge && source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select data_agent
source install/setup.bash

# 4. 重启整栈（data_agent 随 hunter-edge 起）
sudo systemctl restart hunter-edge

# 5. 配置实机路径（若与默认不同）
ros2 param set /data_agent bag_output_dir /home/data/rosbag
ros2 param set /data_agent bag_record_duration_s 20

# 6. 验收：触发事件后录制应在 20s 内自动停止
# 等待下一次 manual_takeover 或临时调低阈值复现，观察：
pgrep -af 'ros2 bag record'   # 20s 后应为空
du -sh /home/data/rosbag/*    # 单目录应 ≤ 数百 MB
```

**七、分发**

- 本仓源码变更：`src/hunter_agents/data_agent/src/data_agent.cpp` + `include/data_agent/data_agent.hpp`
- 实机同步：`git pull` + 重编 `data_agent`
- 无需重编其他包，无需更新 systemd unit

**八、待办（本版本未关闭）**

1. **部署 `vehicle_ingest_agent` + 配置 MinIO/S3 endpoint**：实现环形容存（500MB/10min 切片，保留最近 10 段）+ 自动上传释放 + 磁盘水位淘汰——本版本的 `timeout 20s` 是临时安全网，最终目标是让 `data_agent` 不再直接录制而由 `vehicle_ingest_agent` 统一管理。配置步骤：
   ```bash
   colcon build --symlink-install --packages-select vehicle_ingest_agent
   sudo cp src/hunter_agents/vehicle_ingest_agent/systemd/vehicle-ingest-agent.service /etc/systemd/system/
   sudo systemctl daemon-reload
   # 编辑 /etc/hunter/vehicle_ingest_agent.yaml 填入实际 MinIO/S3 endpoint、bucket、凭据
   sudo systemctl enable --now vehicle-ingest-agent
   ```
2. 事件录制从 `data_agent` 迁移到 `vehicle_ingest_agent`：由 `vehicle_ingest_agent` 的 `cmd_consumer` 订阅 Kafka event 并调用 `recorder.mark_current_protected(reason)` 标记关键片段，`data_agent` 侧 `triggerBagRecord` 改为“仅通知、不录制”。



## V0.1.26（Date：2026-10-17）

**主题：V0.1.25 两处补丁——`PRIu64` 格式串缺 `%` 的编译警告修正 + “C++ 空编译 mtime 陷阱”实机确诊与处置口径**

**一、背景（V0.1.25 实机部署时暴露）**

V0.1.25 首次部署时，实机执行 `colcon build --symlink-install --packages-select data_agent vehicle_ingest_agent` 输出 `Finished <<< data_agent [0.53s]`，但随后：

```bash
$ ros2 param set /data_agent bag_record_duration_s 20
Setting parameter failed: parameter 'bag_record_duration_s' cannot be set because it was not declared
$ pgrep -af 'ros2 bag record'
3640 /usr/bin/python3 /opt/ros/humble/bin/ros2 bag record -o /data/rosbag/..._manual_takeover ...   ← 无 timeout 包裹
```

新参数未声明、录制命令无 `timeout` 包裹——都说明**运行的仍是旧二进制**。

**二、根因（两个独立问题）**

| # | 问题 | 根因 | 处置 |
|---|---|---|---|
| 1 | **C++ 空编译（mtime 陷阱）** | data_agent.cpp 有 1300+ 行、链接 librdkafka/sqlite3/ROS2，正常重编需 20~60s；0.53s = make 判定“无改动”根本没重新编译。Windows 侧编辑 → 同步到 Linux 时保留了较旧的源文件 mtime（比 build 产物还旧），colcon 增量判断误跳过。`--symlink-install` 只省去 Python/launch/config 重编，**C++ 仍必须真正触发 make** | 强制干净重编 `rm -rf build/data_agent install/data_agent` → 重编耗时 41.4s（确凿证据），`strings install/data_agent/lib/data_agent/data_agent_node \| grep 'timeout --signal=INT'` 命中——新代码确实编进去了 |
| 2 | **`PRIu64` 格式串缺 `%`** | V0.1.25 磁盘水位保护的 `RCLCPP_ERROR(..., "磁盘可用空间不足（" PRIu64 "MB < 2048MB）...", avail_mb)`：`PRIu64` 宏只展开为 `lu`（长度符+转换字符，**不含 `%`**），拼接后格式串无任何转换说明符 → `avail_mb` 被当多余参数，编译报 `-Wformat-extra-args`；磁盘不足触发时日志会把字面 `luMB` 打出不显示实际数值（非致命但错误信息不可读） | 改为 `"磁盘可用空间不足（%" PRIu64 "MB < 2048MB）..."`，拼接为 `%lu`，警告消除 |

**三、本次修复（data_agent C++）**

| # | 文件 | 修复内容 |
|---|---|---|
| 1 | `data_agent.cpp` `triggerBagRecord`（磁盘水位保护行） | `"…（" PRIu64 "MB…"` → `"…（%" PRIu64 "MB…"`，补上缺失的 `%` |

**四、实机验证结果（✅ 已通过，2026-10-17）**

```bash
$ rm -rf build/data_agent install/data_agent
$ colcon build --symlink-install --packages-select data_agent
Finished <<< data_agent [41.4s]   ← 真正重编（对比首次 0.53s）
$ strings install/data_agent/lib/data_agent/data_agent_node | grep -E 'timeout --signal=INT|bag_record_duration_s|bag_output_dir'
bag_record_duration_s
bag_output_dir
timeout --signal=INT
$ sudo systemctl restart hunter-edge
$ ros2 param get /data_agent bag_record_duration_s
Double value is: 20.0
$ ros2 param get /data_agent bag_output_dir
String value is: /data/rosbag
$ ls -ld /data
lrwxrwxrwx 1 root root 10 /data -> /home/data   ← 软链，路径本就一致（无需改 bag_output_dir）
$ pgrep -af 'ros2 bag record'   ← 无输出，无残留录制进程
```

- V0.1.25 新参数已声明（`bag_record_duration_s=20.0`、`bag_output_dir=/data/rosbag`），旧 PID 3640 无界录制已 SIGINT 收尾退出；
- `/data → /home/data` 为软链，`/data/rosbag` 与磁盘统计的 `/home/data/rosbag` 同一分区，默认输出路径无需调整；
- 本次 `PRIu64` 修正待下一轮干净重编后以“stderr 不再有 `-Wformat-extra-args`”为验收口径（格式串为静态修正，不改变任何行为）。

**五、可复用的排障口径（全仓适用）**

1. **C++ 包改后重编，以“编译耗时应明显变长”作为真实编译的判据**：小包的亚秒级 `Finished`（如 0.5s）几乎必然是空编译；若同时出现新代码不生效（参数 not declared / 旧行为不变），首先怀疑 mtime 陷阱；
2. **强制干净重编**：`rm -rf build/<pkg> install/<pkg> && colcon build --packages-select <pkg>`；或先 `touch src/**.cpp` 刷新 mtime；
3. **以二进制内容验证“编译真的发生”**：`strings install/<pkg>/lib/<pkg>/<exe> | grep <新增字串>`；命中才算新代码已入；
4. **`PRIu64`/`PRIi64` 等宏必须搭配 `%`**：`printf` 风格里写 `"...=%" PRIu64 "..."`，不能写 `"...=" PRIu64 "..."`（后者缺百分号、报 `-Wformat-extra-args`）。

**六、分发**

- 本仓源码变更：`src/hunter_agents/data_agent/src/data_agent.cpp`（单行格式串修正）
- 实机同步：`git pull` → **强制干净重编**（rm -rf build/install）→ `sudo systemctl restart hunter-edge`
- 无需重编其他包，无需更新 systemd unit


## V0.1.27（Date：2026-10-17）

**主题：车端 ROS Bag 上传接入平台 presign 契约（V1.21.13）——由「车端 boto3 直连 MinIO + 自拼 object_key + 逐文件上传」改造为「服务端 presign 生成 object_key + 车端 mTLS 走 REST + 裸 PUT 预签名 URL + 服务端 complete 校验 md5/sha256/size」**

**一、现场（改造前）**

`vehicle_ingest_agent/uploader.py` 的上传模型与平台 `data-collector` presign 契约架构不兼容：
- 车端用 boto3 直连 MinIO（`/etc/hunter/oss` AK/SK），**自己按 `bags/<version>/<vid>/<date>/<bag_id>/<rel>` 拼 object_key**；
- 逐文件上传（manifest + 每个 .db3 各一个对象），HEAD 幂等；
- 无服务端完整性校验（md5/sha256/size 三元组）、无 retention 打标、无设备身份（CN）绑定。

**二、契约差异（V1.21.13 关键约束 → 落地方式）**

| 主题 | 旧（车端直连 S3） | 新（V1.21.13 presign） | 落地 |
|---|---|---|---|
| 鉴权 | AK/SK（STS） | **车端 mTLS 直连 `https://<SERVER_IP>:8443`**，复用 Kafka 接入包 PEM（CA/client-cert/client-key，CN=vehicle_id），无 JWT/token | `presign.py` `FileUploadClient`：`requests.Session(cert=(client,key), verify=ca)`，PEM 走 `upload.presign.bundle_dir` |
| object_key | 车端自拼 | **服务端生成**（相对 Key，不含 bucket），complete 原样回带 | `job.object_key` 由 presign 返回回填 |
| 粒度 | 逐文件 | **一个 bag 打包成单个 tar 归档 → 一轮 presign→PUT→complete** | `pack.py` `pack_and_hash`（`{metadata.yaml,*.db3,manifest.json}` → `<staging>/<bag_id>.tar`） |
| 完整性 | 无 | complete 服务端重算 size/md5/sha256 与声明比对，`verified=true` 才算成功 | 归档字节**单次流式**算三元组；`verified` 才删 state + 删归档 |
| retention | 无 | `regular`(30d)/`event`(永久)，presign+complete 都带且一致 | `seg.protected==1 → event`，否则 `regular` |
| Content-Type | 无 | 单 PUT 必须与 presign 声明一致，否则 403 | `upload.presign.content_type=application/x-tar` |

**三、本次改动（Python，无 C++ 重编）**

| # | 文件 | 内容 |
|---|---|---|
| 1 | `config.py` | DEFAULTS `upload.presign.{api_base,bundle_dir,data_type,content_type,deadline_s}` 子块 + `paths.staging_dir`；默认 `endpoint_type→presign`、`bucket→hunter-rosbag`、`rate_limit_mbps→10`；`validate()` 允许 `presign`，presign 且 `api_base` 空 → `ConfigError` |
| 2 | `config/vehicle_ingest_agent.yaml` | 同步落值 + presign 子块；`api_base` 留空（现场填平台地址） |
| 3 | `presign.py`（新增） | mTLS REST 客户端：`presign/put_single/put_part/complete`；错误码类型化异常 `TLSAuthFailed(1001)/PermanentDeny(1002)/BadParams(2001)/NotFound(3001)/ServerBusy(5001)/IntegrityMismatch(6001)/RateLimited(429)` |
| 4 | `pack.py`（新增） | `tarfile` 打包 + tee 包装单次流式产出 `(size, md5_hex, sha256_hex)`（对归档真实字节，独立于 manifest 跨文件 sha256） |
| 5 | `uploader.py` | presign 模式每 bag 一轮（`_upload_via_presign`）、part-state 键改 `bag_id`、续传复用未过期 presign、按异常类型分流重试（`_handle_presign_failure`）；s3/gateway 分支保留原逻辑 |
| 6 | `agent.py` | `emit_artifact_uploaded` 的 `endpoint` presign 模式取 `upload.presign.api_base`；`object_key/size/sha256` 取回填/三元组；`etag` 传 verified |
| 7 | `package.xml` / `setup.py` | 声明 `requests` 依赖（`python3-requests` / `install_requires`） |

**四、向后兼容**

- `endpoint_type` 仍支持 `s3`/`gateway`（开发机兜底，逐文件逻辑原样保留）；仅生产默认切 `presign`。
- 状态机 / 环形缓存 / 水位淘汰 / spool 删除 / outbox 链路不变；verified 后 `apply_after_upload` 释放本地、protected 保留。
- 无新重依赖（不引入 boto3 之外的东西；presign 仅需 requests）。

**五、风险与口径**

- `api_base` 留空启动即 `ConfigError`（systemd 可见），避免误以「上传不可用」静默滞留。
- 仅有 `.p12` 无 PEM → 明确报错提示 openssl 导出（requests 不能直读 p12）。
- 幂等：契约每次 presign 生成新 object_key，重传会产生新对象——以「complete verified 才删本地」为准，避免半态重灌。
- `6001` 多为三元组口径误用（拿 ETag 当 md5）或归档被改写：处置为作废重打包重传。

**六、实机验证结果（2026-10-17 车端首次接入，分两层如实登记）**

> 结论：**车端侧已全部验证通过**；**完整契约 E2E 阻塞在平台侧 `:8443` 设备入口尚未上线（`Connection refused`），仍为待办**——不标为已验证。

**6.1 车端侧 —— 已验证通过 ✅**

- 构建：`colcon build --symlink-install --packages-select vehicle_ingest_agent` 成功（`Finished [2.27s]`、`1 package finished`）。
- 配置校验：`config.validate` 未抛 `ConfigError` ⇒ `upload.presign.api_base` 已正确填值。
- **presign 上传引擎正常就绪**：日志 `{"logger":"via.uploader","msg":"上传引擎就绪 backend=presign bucket=hunter-rosbag workers=3 rate=1.3MB/s"}`——`backend=presign`、`bucket=hunter-rosbag` 均符合契约；`rate=1.3MB/s`≈**10.4Mbps** 印证 `rate_limit_mbps:10` 令牌桶生效、`workers=3` 并发在位；`systemctl restart` 后稳定运行、无启动崩溃。
- 依赖：`python3 -c "import requests"` → OK。
- 静态自检：`py_compile` 全通过、`pack_and_hash` 三元组与 tar 字节一致性单测、`config.validate` presign 约束单测、模块导入无环。

**6.2 平台/数据面 —— 部分可达，E2E 阻塞在 `:8443` ⚠**

- **MinIO 数据面 `:9000` 可达**：`curl http://192.168.31.35:9000/minio/health/live` 无连接错误返回（200、空体）→ 裸 PUT 目标可达（建议再 `curl -I` 坐实状态码）。
- **REST 控制面 `:8443` 连接被拒**：`curl --cacert … https://192.168.31.35:8443/…` 与 `http://…:8443/…` 均 **`Connection refused`**（1ms 即拒 = 端口未监听）。**属平台侧设备入口尚未上线**（方案假设：`:8443` 需平台上线；未上线前 presign 连接失败属平台侧，车端不重试、不误判 `1001`）。实测印证：引擎仍正常就绪、未因连接失败而崩溃。
- **因此完整 E2E（`presign code=0 → PUT 2xx → complete verified=true`、反例矩阵、`retention=event` Tag、断网续传）尚未跑通，作为待办**。

**待办（依赖平台侧，上线后按 §7.4 逐项补测并回填本小节）**：
- 平台 `:8443` 设备入口上线 → 跑小 bag（`part_count=1` 单 PUT）+ 大 bag（`>1` 分片合并）各一轮 `presign→PUT→complete verified=true`、`notify_topic=sensor_file`。
- presign 返回 `upload_url` 的 host 必须为车端可达地址（非容器内网名）。
- 反例：篡改三元组→`6001`；`vehicle_id`≠CN→`1002`；异/过期证书→`1001`；限流→`429` 退避后成功。
- `retention=event`（protected 片段）上传后平台确认对象 Tag `hunter-retention=event`；断网中途→恢复按重试矩阵续传。

**七、实机部署与执行**

> 本节为**落地可直接照做的部署 runbook**（步骤与判据均源于本次已落地的 `config.py`/`presign.py`/`pack.py`/`uploader.py` 与 yaml）。⚠ 步骤中的验收结果需**实跑后回填到「六、实机验证结果」**，不得预填为「已验证」。

**7.1 前置核对（上线前逐项确认，任一不过即不通）**

| # | 核对项 | 命令 / 判据 |
|---|---|---|
| ① | `upload.presign.api_base` 已填平台真实地址 | `grep api_base /etc/hunter/vehicle_ingest_agent.yaml` 非空；留空启动即 `ConfigError`（systemd 可见） |
| ② | 接入包 PEM 三件套在 `bundle_dir`（默认 `/etc/hunter/kafka`）且运行用户可读 | `ls -l /etc/hunter/kafka/{ca-cert,client-cert,client-key}.pem`；私钥 `0600` **且属主=运行用户**（同 Kafka 权限口径，属主错会 TLS 假故障）；仅有 `.p12` 需先 `openssl` 导出 PEM |
| ③ | `:8443` 设备入口可达 | `curl -v --cacert ca-cert.pem --cert client-cert.pem --key client-key.pem https://<SERVER_IP>:8443/...` 有 TLS 响应 |
| ④ | `MINIO_PUBLIC_ENDPOINT` 车端可达、且 presign 返回 URL host 非容器内网名 | `curl http://<SERVER_IP>:9000/minio/health/live` = 200；从日志取 `upload_url` 看 host |
| ⑤ | `requests` 已安装 | `python3 -c "import requests; print('OK')"` |
| ⑥ | `vehicle_id` == 证书 CN | 由接入包 SASL 用户名推导，与 `cfg.vehicle_id` 一致（不等则 presign/complete 报 `1002`） |

**7.2 配置落值**

```bash
cd ~/HunterEdge
# endpoint_type: presign、bucket: hunter-rosbag、rate_limit_mbps: 10 已在仓内 yaml 落定；只需将 api_base 填成平台地址
sudo cp src/hunter_agents/vehicle_ingest_agent/config/vehicle_ingest_agent.yaml /etc/hunter/vehicle_ingest_agent.yaml
sudo vi /etc/hunter/vehicle_ingest_agent.yaml   # upload.presign.api_base: https://<SERVER_IP>:8443
```

**7.3 构建与重启（纯 Python 改动，无需重编 C++）**

```bash
cd ~/HunterEdge
# 先清旧构建产物避开 C++ 空编译 mtime 陷阱（Windows 同步保留旧 mtime 使 colcon 误跳过）
rm -rf build/vehicle_ingest_agent install/vehicle_ingest_agent
colcon build --symlink-install --packages-select vehicle_ingest_agent
sudo systemctl restart vehicle-ingest-agent
journalctl -u vehicle-ingest-agent -f | grep -E 'presign|complete|verified|上传'
```
> 判错口径：启动即 `ConfigError` → `api_base` 未填；报「mTLS 需 PEM 证书」→ 缺 `client-cert/key.pem`；`1001` 不重试属证书/平台侧；`1002` 不重试属 `vehicle_id≠CN`/权限；`6001` 多为本方案自动作废重打包（若反复则核三元组口径与版本 ≥V0.1.27）。

**7.4 上线验收（对照方案 §10，实跑后回填至「六」）**

- 小 bag（`part_count=1` 单 PUT）与大 bag（`>1` 分片合并）各一轮完整 `presign code=0 → PUT 2xx → complete verified=true`，`notify_topic=sensor_file`。
- 反例：篡改三元组 → `6001`；`vehicle_id`≠CN → `1002`；异/过期证书 → `1001`；限流 → `429` 按 `Retry-After` 退避后成功。
- `retention=event`（protected 片段）上传后平台确认对象 Tag `hunter-retention=event`。
- 断网中途 → 恢复按重试矩阵续传成功；本地片段 verified 后自动释放、protected 不删。

**7.5 分发与回滚兜底**

- 分发：纯 Python，`--symlink-install` 同步即生效；无需 `apt` 新依赖（`requests` 随系统安装）。
- 回滚：契约 E2E 未通或平台侧未就绪时，把 `upload.endpoint_type` 改回 `s3`/`gateway`（逐文件直连 MinIO 逻辑原样保留），`colcon build` + `systemctl restart` 即回旧上传行为，**无需回退代码**。


## V0.1.28（Date：2026-10-17）

**主题：遥控「有时指令车不动」——safety_guard 遥控执行桥 `remote_timeout` 与 remote_agent `cmd_timeout` 全链路对齐（0.5s → 2.0s）**

**一、现场**

平台远程接管/操控（`操控中`、`指令回执 11ms`、Kafka `hunter.<vid>.remote_control` 逐帧送达并回执），但**有时**下发 W/S 加减速、A/D 转向、空格急停后车辆停在原地不动；表现为间歇性、非必现。

**二、根因（执行链最后一段的新鲜度窗口过窄）**

遥控执行链：`平台 → Kafka remote_control → remote_agent(on_command) → ROS /remote/command(ChassisCommand) → safety_guard 遥控执行桥(step 2.5) → /cmd_vel(Twist) → 底盘`。

V0.1.18 在 `safety_guard` 补上了「遥控执行桥」（把 `ChassisCommand` 经阿克曼换算成 `Twist` 作为本闸输入），链路本身已通。但桥判定遥控帧是否有效的 `remote_timeout=0.5s` 与上游实际投递节奏严重不匹配：

- 平台 UI 虽 20Hz 发帧，但 **Kafka `acks=all` 实际到车 ~2s/帧**（remote_agent 早已据此把自身 `cmd_timeout` 从 0.5s 提到 3.0s，见 `remote_agent.py` 配置注释）；
- remote_agent 的 20Hz ROS 定时器在会话存续期间持续补发 `/remote/command`，但该定时器在 Jetson 上偶发被 GStreamer 写线程/DDS 序列化/CPU 饥饿短暂拖慢；
- 一旦相邻两帧 `/remote/command` 间隔 >0.5s，`safety_guard` step 2.5 判定遥控失效 → 回退 Nav2 输入（`cmd_vel_in`）→ 遥控时无活跃导航 goal → step 3 上游看门狗 `CMD_TIMEOUT` → **零速**（`decision_making` 的 `remote_timeout=0.5s` 同理会令模式在 REMOTE/AUTO 间反复横跳）。

即「有时遥控指令车不动」= 遥控执行桥的新鲜度窗口没跟上 Kafka 投递抖动，被误判为遥控断流。

**三、本次修复（参数对齐，无算法改动）**

| # | 文件 | 内容 |
|---|---|---|
| 1 | `hunter_safety/src/safety_guard.cpp` | `remote_timeout` 默认 `0.5 → 2.0`（成员初值同步），补注根因 |
| 2 | `hunter_bringup/launch/hunter_autonomous_nav.launch.py` | safety_guard 参数显式落 `enable_remote_bridge: True`、`remote_timeout: 2.0`（避免默认值漂移） |
| 3 | `hunter_safety/launch/safety_guard.launch.py` | 独立调试 launch 同步落 `remote_timeout: 2.0`，与生产同源（防调试参数漂移静默复现历史死锁） |
| 4 | `decision_making/config/decision_making_params.yaml` + `decision_making/src/decision_making.cpp` | `remote_timeout` `0.5 → 2.0`，模式仲裁与执行链对齐，消除 REMOTE/AUTO 横跳 |

取值依据：2.0s ≥ Kafka 到车 ~2s/帧抖动上限，且仍 < 会话结束语义时延——会话真正结束后 remote_agent 先补发一帧刹停（`emergency_stop=true`）再停发，`safety_guard` 在 2.0s 窗口内锁存的是**零速刹停帧**，车辆恒停，不影响交还驾驶权，安全无副作用。

**四、向后兼容**

- 仅放宽超时阈值，不改任何消息类型/话题/协议/算法；`remote_max_linear_vel`、碰撞闸、扫掠弧、地图边界监护、曲率钳制等全部不变。
- `enable_remote_bridge` 默认仍为 true；显式落值只是把原隐式默认固化，行为等价。
- 建图（mapping）模式仍不启动 safety_guard（本修复不改变「遥控执行桥仅 nav 模式生效」的既有边界，见风险项）。

**五、风险与遗留**

- **本修复只覆盖「遥控桥新鲜度误判」这一条根因**。若现场「车不动」另有其因，需按下方排查项分别定位，不可一并记为已修复：
  - `/scan` 断流 >0.5s → `SCAN_TIMEOUT` 零速（雷达/TF/pointcloud_to_laserscan 侧）；
  - 碰撞闸 `COLLISION_STOP` / 地图边界 `MAP_EDGE_STOP` 拦截（真实障碍或假障碍占位）；
  - **建图模式无 safety_guard**：`/remote/command` 无执行桥，遥控必不动（属既有架构边界，需另设 mapping 模式单一 `/cmd_vel` 所有者才能在该模式遥控）；
  - boot 帧丢失导致 `session_active=False`：remote_agent 根本不发布 `/remote/command`（回执仍发，平台侧看似正常）。
- 2.0s 是「容忍抖动」与「及时判失联」的折中；若现场 Kafka 抖动更大，可继续上调，但须同步 remote_agent `cmd_timeout`（当前 3.0s）保持 `remote_timeout ≤ cmd_timeout`。

**六、实机验证结果（⚠ 待办——本次仅完成代码/参数改动，尚未实机部署复现验证）**

- 编译：需对 `hunter_safety`、`decision_making` 两个 C++ 包强制干净重编（避开 V0.1.26 确诊的 mtime 空编译陷阱）：`rm -rf build/hunter_safety install/hunter_safety build/decision_making install/decision_making && colcon build --symlink-install --packages-select hunter_safety decision_making`（launch/yaml 走 `--symlink-install` 同步即生效）。
- 参数生效核对：`ros2 param get /safety_guard remote_timeout` = 2.0；`ros2 param get /decision_making remote_timeout` = 2.0。
- 现象验收：实机反复操控（含网络抖动/持续按住 W 行驶）观察是否仍「有时不动」；同时 `ros2 topic echo /safety/state` 确认不再间歇跳 `CMD_TIMEOUT`，`ros2 topic hz /remote/command` 稳定 20Hz。
- 上述任一项未过，须回到「五、风险与遗留」逐条排查真实根因后再回填本节。

**七、分发**

- 源码变更：`safety_guard.cpp`、`safety_guard.launch.py`、`hunter_autonomous_nav.launch.py`、`decision_making_params.yaml`、`decision_making.cpp`。
- 实机同步：`git pull` → 强制干净重编 `hunter_safety`+`decision_making` → 重启导航栈（`sudo systemctl restart hunter-edge` 或重启 `hunter_autonomous_nav.launch.py`）；remote_agent 无需改动/重启。


## V0.1.29（Date：2026-10-17）

**主题：建图（mapping）模式遥控执行桥补齐——`auto_mission` 作为 `/cmd_vel` 单一所有者转发遥控指令（消除 V0.1.28「五、风险与遗留」中标注的架构硬边界）**

**一、现场**

承接 V0.1.28：nav 模式遥控已对齐，但**建图模式**下平台远程接管/操控下发 W/S、A/D、空格急停，车辆仍**必然**停在原地不动（非间歇性，而是该模式下遥控根本无执行通路）。而 `hunter_autonomous_nav.launch.py` 开头明确写「mode:=mapping — 建图模式（车辆由人工遥控驾驶扫描）」——建图本就依赖人工遥控驾驶，遥控不动等于该模式不可用。

**二、根因（建图模式遥控执行桥缺失）**

V0.1.18 的遥控执行桥落在 `safety_guard`，而 `safety_guard` **仅在 nav 模式启动**（建图模式不启动，避免与 `auto_mission` 巡航双写 `/cmd_vel`）。建图模式下 `/cmd_vel` 的唯一所有者是 `auto_mission`：

- 平台遥控 → `remote_agent` → `/remote/command(ChassisCommand)` → `decision_making` 仲裁 `mode=REMOTE`；
- `auto_mission` 据 `mode!=AUTO` 置 `mapping_paused_=true`，巡航逻辑改发**零速**（本意是给遥控让出驾驶权）；
- 但**没有任何环节把 `/remote/command` 的 `target_velocity/target_steering` 转成 `/cmd_vel`** → 遥控指令落空、车恒停。即驾驶权已「交出去」却无人「接手」。

**三、本次修复（在 `/cmd_vel` 现有唯一所有者处补执行桥，不新增节点/话题）**

| # | 文件 | 内容 |
|---|---|---|
| 1 | `auto_mission/include/auto_mission/auto_mission_node.hpp` | 引入 `chassis_command.hpp`；新增 `/remote/command` 订阅、桥缓存/参数成员、`mappingRemoteBridgeStep()` 声明 |
| 2 | `auto_mission/src/auto_mission_node.cpp` | 声明并读取桥参数；创建 `/remote/command` 订阅；实现 `mappingRemoteBridgeStep()` 并接入 20Hz `cruiseControlStep()` 顶部（优先于巡航） |
| 3 | `hunter_bringup/launch/hunter_autonomous_nav.launch.py` | mapping 模式 `auto_mission` 显式落 `enable_mapping_remote_bridge/remote_cmd_topic/mapping_remote_timeout(2.0)/mapping_wheelbase(0.46)/mapping_remote_max_vel(2.0)` |

桥接管条件（与安全闸口径一致，多重门）：`enable` 开 **且** 无全局 `/estop` **且** `decision_making mode==REMOTE` **且** 帧 `control_mode==REMOTE` **且** `/remote/command` 新鲜（`≤remote_timeout=2.0s`，沿用 V0.1.28 对齐值）。换算 `ω=v·tan(δ)/L`（L=0.46，与 `safety_guard` 遥控桥同值），并按 `|w|≤|v|/R_min`（R_min=1.9m）钳制阿克曼可行性；`emergency_stop=true` → 零速。任一条件不满足即释放（若上一拍在接管则补一帧零速），交还巡航/停车逻辑，驾驶权回 AUTO 时巡航自动续接。

**四、单一 `/cmd_vel` 所有者保证**

- 桥仅在 `mission_mode=="mapping"` 生效；nav 模式该订阅闲置、绝不写 `/cmd_vel`（nav 遥控仍由 `safety_guard` 独占），不产生双发布者。
- 与巡航逻辑同处**一个 20Hz 定时器**、**一个 `cmd_vel_pub_`**：遥控接管拍 `return` 跳过巡航，物理上不可能与巡航同拍双写。

**五、向后兼容**

- 纯增量：新增订阅 + 一个前置判定分支，不改动巡航算法、航点逻辑、pcd_to_map、estop 语义。
- `enable_mapping_remote_bridge` 默认 true；置 false 即回到 V0.1.28 行为（建图遥控无效），可即时回退。
- 参数取值全部与 nav 模式 `safety_guard` 遥控桥 / remote_agent 对齐（timeout 2.0、L 0.46、限速 2.0），两种模式遥控手感一致。

**六、风险与遗留**

- 建图模式无 `/scan`/costmap，本桥**不含**碰撞闸、扫掠弧、地图边界监护——与「人工遥控驾驶建图」的既有安全假设一致（操作员目视驾驶）；速度/转向限幅与曲率钳制仍生效。
- 依赖 `decision_making` 在建图模式正常运行并输出 `mode`（该节点在所有模式无条件启动，成立）。
- 交接帧（`control_mode==AUTO`）不会触发本桥（额外校验 `control_mode==REMOTE`），避免用交还驾驶权帧的残留 target 误驱动。

**七、实机验证结果（⚠ 待办——本次仅完成代码/参数改动，尚未实机部署验证）**

- 编译：`auto_mission` 为 C++ 包，需强制干净重编规避 V0.1.26 的 mtime 空编译陷阱：`rm -rf build/auto_mission install/auto_mission && colcon build --symlink-install --packages-select auto_mission`（launch 走 `--symlink-install` 同步即生效）。
- 参数生效：建图模式启动后 `ros2 param get /auto_mission_node enable_mapping_remote_bridge` = True；`ros2 param get /auto_mission_node mapping_remote_timeout` = 2.0。
- 现象验收：建图模式（`autonomous_nav_mode:=mapping`）下平台接管并 W/S/A/D，`ros2 topic echo /cmd_vel` 应随操作出现非零线/角速度、车辆响应；空格急停 → `/cmd_vel` 归零；松开 2s 或交还驾驶权后 `mapping_remote_active_` 释放、补一帧零速，日志见 `[建图遥控桥] 转发遥控 ...` / `遥控结束 ... 交还巡航`。
- 回归：确认 nav 模式遥控仍由 `safety_guard` 独占、`auto_mission` 不写 `/cmd_vel`（`ros2 topic info /cmd_vel -v` 发布者仍单一）。

**八、分发**

- 源码变更：`auto_mission_node.hpp`、`auto_mission_node.cpp`、`hunter_autonomous_nav.launch.py`。
- 实机同步：`git pull` → 强制干净重编 `auto_mission` → 重启建图栈；remote_agent / safety_guard / decision_making 无需改动。


## V0.1.30（Date：2026-10-17）

**主题：health_monitor 补齐 GPU 使用率真实采集——消除平台遥测「GPU 0.0%」硬编码误报**

**一、现场**

HunterCore 运营后台操控台实时遥测显示「系统负载 CPU 34.0% · GPU 0.0% · RSSI 0 dBm」，GPU 始终为零。车辆感知（vision_perception TensorRT YOLOv8 15Hz）与视频编码（AGX Orin NVENC）均在使用 GPU，属监控层缺失实现而非 GPU 空闲。

**二、根因**

`health_monitor.cpp` 第 247 行：
```cpp
msg.gpu_usage = 0.0f;  // GPU 使用率（可用 nvml/tegrastats 扩展）
```
自初版即硬编码为零，从未读取 Jetson 真实 GPU 负载。数据链路：
```
health_monitor(gpu_usage=0) → /system/health → data_agent(buildSystemSegment) → Kafka health/telemetry → 平台显示 0%
```

**三、本次修复**

| # | 文件 | 内容 |
|---|---|---|
| 1 | `hunter_monitor/health_monitor/include/health_monitor/health_monitor.hpp` | 新增 `readGpuUsage()` 方法声明 |
| 2 | `hunter_monitor/health_monitor/src/health_monitor.cpp` | 实现 `readGpuUsage()`（POSIX open/read/close）；`checkHealth()` 调用替换原硬编码 0.0f |

`readGpuUsage()` 尝试路径（取首个可读节点）：
1. `/sys/devices/platform/bus@0/17000000.gpu/load`（AGX Orin）
2. `/sys/class/devfreq/17000000.gpu/load`（Orin 备选）
3. `/sys/devices/platform/gpu.0/load`（Xavier）

值域 0–1000 → 除以 10 得到 0.0%–100.0% 百分比。若全部不可读则仅打印一次 WARN 日志，返回 0。

⚠ **实现坑（实机发现）**：首版用 `std::ifstream` 读 sysfs 虚拟文件，在 systemd 服务上下文（`User=root`）下 `open` 成功但缓冲 `read` 返回空，`raw` 保持初始值 0 且通过 `>=0` 范围检查 → 静默上报 0%（无 WARN 日志）。改用 POSIX `open()`/`read()`/`close()` 直接系统调用后修复（`read()` 返回 `n<=0` 时 `continue` 到下一路径，不再误判为 0）。

**四、向后兼容**

- `SystemHealth.msg` 字段 `gpu_usage` 本身已存在，不改消息定义。
- `data_agent` 的 `buildSystemSegment()` 直接透传 `health_.gpu_usage`，无需改动。
- 非 Jetson 环境（如开发机 x86）路径不存在，行为等价旧版（上报 0）。

**五、风险与遗留**

- 若现场 Jetson L4T 版本升级导致 sysfs 路径变更，需同步适配（可用 `find /sys -name "load" -path "*gpu*"` 定位）。
- RSSI 0 dBm 为设计如此（车辆有线网络，无 WiFi/蜂窝制式上报通道），本次不改动。

**六、实机验证结果（✅ 已通过）**

- 车端 sysfs 路径确认：`cat /sys/devices/platform/bus@0/17000000.gpu/load` 返回 149（即 14.9%），路径存在且可读。
- 首版（`std::ifstream`）实机失败：`ros2 topic echo /system/health --once` 仍输出 `gpu_usage: 0.0`，journal 无 WARN 日志（确认是静默读空问题）。
- 改用 POSIX I/O 后实机验证通过：`ros2 topic echo /system/health --once` 输出 `gpu_usage: 5.6`，平台操控台实时遥测显示「CPU 36.0% · GPU 10.5% · RSSI 0 dBm」。

**七、分发**

- 源码变更：`health_monitor.hpp`、`health_monitor.cpp`。
- 实机同步：`git pull` → 干净重编 `health_monitor` → 重启 `hunter-edge` 服务（或单独重启 health_monitor 节点）；data_agent / vision_perception 无需改动。


## V0.1.31（Date：2026-10-19）

**主题：远程接管「车不动」根因（CPU 超订致 FAST-LIO2 里程计饥饿发散）防复发加固 1–4（无头纪律 + 降采样 + 限核提优先级 + 发散自愈的代码级位姿校验）**

**一、现场**

承接实机诊断闭环：平台远程接管/操控，车辆停在原地不动。逐层排查（时间同步→IMU 单位→配置漂移→旧二进制）后坐实：**车端仅 12 核，调试期手工拉起 rviz2（点云累积显示抢 ~3 核、load 20.85）导致 CPU 超订**，FAST-LIO2 起步即被饿到 **里程计发散**——`/Odometry` 位姿冲到 1e12 m、ikd-tree 内存暴涨（4.5GB）、掉帧至 0.3Hz 死亡螺旋。受控止血（`kill rviz2` + `systemctl restart hunter-edge`）后 `/Odometry` 回 10Hz、位姿米级、TF 连通、重定位 cov=0.01 收敛、MAP_EDGE_STOP 解除、`/cmd_vel` 随遥控油门透传、车辆跟随执行——根因确认。

**二、根因链与级联**

```
CPU 超订（rviz 抢核 + point_filter_num=1 不降采样，每帧 ~27860 点全量匹配）
  → FAST-LIO2 起步饥饿 → /Odometry 发散(1e12m) → odom→base_link TF 缺失/未来戳 → TF 树断裂成两半
  → NDT 重定位拿不到合理 odom→base，把垃圾 result 锁进 T_map_odom → /relocalization/pose 发散
  → safety_guard 把垃圾位姿落到栅格界外 → 误锁 MAP_EDGE_STOP 钳零 → /cmd_vel=0 → 遥控车不动
```

本条不改变上述诊断结论，只做**防复发加固**（行为纪律 + 参数 + 代码级校验），使同一超订场景不再触发假锁死。

**三、本次加固（1–4）**

| # | 文件 | 内容 |
|---|---|---|
| ① | `hunter_bringup/launch/hunter_edge.launch.py` | 入口头新增「⚠ 车端无头运行纪律」块：核实 `hunter_edge/hunter_full` 全 launch 树**无任何 `executable='rviz2'` 节点**（rviz2 仅在注释），生产自启本就不起 RViz；把根因链写死在文件头防后续有人误加 RViz 回归。**属确认+纪律固化，非行为改动** |
| ② | `hunter_bringup/config/fast_lio2_params.yaml` | `point_filter_num: 1 → 2`（每帧 ~27860→~14000 点，匹配开销近半，抗饥饿；对 16 线 Helios 精度影响可忽略，进一步降载可试 3） |
| ③ | `hunter_bringup/launch/localization.launch.py` | `fast_lio2_node` 加 `additional_env={'OMP_NUM_THREADS': '4'}`，避免 OpenMP 贪心开满 12 线程互相踩踏 |
| ③ | `hunter_bringup/systemd/hunter-edge.service` | `[Service]` 加 `Nice=-10`（同 cgroup 内 ROS 子进程优先拿 CPU）+ `CPUWeight=200`（跨 cgroup 相对 GUI/xrdp 会话 ~2× CPU 带宽，补齐 Nice 无法跨 cgroup 之需） |
| ④ | `hunter_relocalization/src/ndt_relocalization_node.cpp` | 新增 `max_valid_translation`（默认 2000m）参数 + `matSane()` 有限性/量级校验；跟踪闸门由 `tracking = step_ok` 改为 `step_ok && result_sane`（**首帧也拒绝垃圾**）；`publishPose` 对 `odom→base` 同样 sanity，发散巨值/非有限退化单位阵 |
| ④ | `hunter_relocalization/config/relocalization_params.yaml` | 落 `max_valid_translation: 2000.0` + 说明 |
| ④ | `hunter_safety/src/safety_guard.cpp` | `/relocalization/pose` 回调新采集 `fence_pose_cov_`；新增 `loc_cov_invalid`（默认 1.0）参数；`mapEdgeDistance(bool & loc_invalid)` 前置校验：位姿非有限 或 协方差≥门限 → 判「定位无效」→ 返回 -1 且 `loc_invalid=true`（**不再把垃圾位姿误锁 MAP_EDGE_STOP**）；消费块加 `else if (loc_invalid)` WARN（越界防护降级、碰撞闸 `/scan` 扫掠弧仍独立兜底、定位恢复自动回弹） |
| ④ | `hunter_bringup/launch/hunter_autonomous_nav.launch.py` | safety_guard 参数显式落 `loc_cov_invalid: 1.0` |

**四、向后兼容**

- ②③为参数/环境调整，行为等价或更稳（降采样+限线程+提优先级）；不新增 ROS 话题/消息/服务（遵 .ai‑rules）。
- ④纯防御性前置校验：定位正常收敛时（cov~0.01、位姿米级）`result_sane`/`loc_invalid` 均为放行态，与旧行为完全一致；仅在发散/非物理巨值时才拒绝刷新/不判越界。`loc_cov_invalid`、`max_valid_translation` 均可调，置大阈值即近似回退旧行为。
- ①为文档/注释加固，无运行时变更。

**五、风险与遗留**

- 本条是**防复发加固**，不覆盖「车不动」的全部可能诱因（V0.1.28「五」列的 `/scan` 断流 SCAN_TIMEOUT、真实障碍 COLLISION_STOP、建图模式无 safety_guard 等仍需分别定位）。
- `Nice=-10`/`CPUWeight=200` 依赖 cgroup v2 与 systemd 特权；若现场为 cgroup v1 或低版本 systemd 需验证是否生效。
- `point_filter_num=2` 会略微稀释 `cloud_registered` 密度（下游 NDT 重定位仍可用，因 source 本就体素降采样）；若极端稀疏场景需恢复全量则回 1。
- 未做 FAST-LIO2 里程计自动 reset（会破坏建图状态，风险高），留作后续可选项。

**六、实机部署与验证步骤（⚠ 部分已通过——2026-10-19 车端实跑：步骤 0–4、6 ✅ 已通过并回填实测；步骤 5「发散不假锁」因当轮未制造定位不可信环境，暂不验收、延后专项验证）**

> 静态检查已过：三个 launch 文件 `ast` 解析、两个 yaml `safe_load` 通过；`mapEdgeDistance` 唯一调用点已同步改签名；IDE 无 C++ lint 插件，未做编译检查。

**步骤 0·前置（同步源码 + 环境）**

```bash
cd ~/HunterEdge && git pull
source /opt/ros/humble/setup.bash && source install/setup.bash 2>/dev/null || true
nproc                         # 记录核数（本例 12），供后续 OMP 线程数对照
```

**步骤 1·干净重编 C++（规避 V0.1.26 的 mtime 空编译陷阱）**

```bash
rm -rf build/hunter_safety install/hunter_safety \
      build/hunter_relocalization install/hunter_relocalization
colcon build --symlink-install --packages-select hunter_safety hunter_relocalization
# 真实编译判据：本条耗时应明显 > 1s（若秒级 Finished 多半被 colcon 误跳过，重来）
# 新代码入二进制核对——注意坑：`strings` 默认只抽可打印 ASCII，UTF-8 中文字节会被整段跳过导致假阴性；
# 中文串须用 `grep -a` 直搜原始字节，ASCII 参数名才可走 strings：
grep -ao 判定位无效 install/hunter_safety/lib/hunter_safety/safety_guard | head -1   # 边界监护“定位无效”降级逻辑已编入
strings install/hunter_relocalization/lib/hunter_relocalization/ndt_relocalization_node | grep -m1 max_valid_translation
```

**步骤 2·安装并重载 systemd 单元（加固③：Nice / CPUWeight）**

```bash
# 单元由 hunter_core_setup.sh 落 /etc/systemd/system/；改过 unit 须重装，二选一：
sudo install -m 0644 src/hunter_bringup/systemd/hunter-edge.service /etc/systemd/system/hunter-edge.service
#   或：sudo bash src/hunter_bringup/scripts/hunter_core_setup.sh --skip-deps --skip-build
sudo systemctl daemon-reload
sudo systemctl restart hunter-edge
# 生效核对：
systemctl show hunter-edge -p Nice -p CPUWeight     # 期望 Nice=-10、CPUWeight=200
```

**步骤 3·参数生效核对（加固②③④，整栈起来后逐项读）**

```bash
ros2 param get /fast_lio2 point_filter_num          # 期望 2（降采样）
ros2 param get /hunter_relocalization max_valid_translation   # 期望 2000.0
ros2 param get /safety_guard loc_cov_invalid        # 期望 1.0
# OMP 限线程：确认 fast_lio2 进程环境带 OMP_NUM_THREADS=4（/proc/<pid>/environ 跨用户需 sudo）
sudo cat /proc/$(pgrep -f fastlio_mapping)/environ | tr '\0' '\n' | grep OMP_NUM_THREADS
```

**步骤 4·抗饥饿验收（加固②③：CPU 超订下不再饿死里程计）**

```bash
# 4.1 空载基线：里程计应 10Hz、位姿米级
ros2 topic hz /Odometry                 # 期望 ≈10.0 Hz
ros2 topic echo /Odometry --once | grep -A3 position   # x/y/z 应在米级（<1e2）
# 4.2 复现超订：短时手工起 rviz2（累积显示 /cloud_registered）压 CPU
tegrastats | grep -m1 RAM   # 观察 CPU 负载上升
#   在 rviz2 抢占下持续观测：
ros2 topic hz /Odometry     # 关键判据：仍稳 ≈10Hz、位姿不跳到 1e6~1e12（旧版会掉到 0.3Hz 并发散）
# 4.3 用完即关（车上默认无头，验证后不留 rviz2）
pkill rviz2
```

**步骤 5·发散不假锁验收（加固④：位姿量级/协方差有效性校验）**

> ⚠ **当轮未验收、延后**：2026-10-19 实跑未制造定位不可信窗口（重定位全程收敛健康，“拒绝刷新/定位无效”日志未出现属预期），不足以证实或证伪闸门；需在能稳定制造发散/未收敛的环境下专项回跑本步。

```bash
# 5.1 制造一次“定位不可信”窗口（任选其一，短时可控）：
#     (a) 冷启动即短时遮挡/拔 LiDAR，使 FAST-LIO2 起步饥饿；或
#     (b) 把车搬离已建图范围使 NDT 无法收敛（fit>ceiling → publishPose 协方差=100）。
# 5.2 观测重定位侧闸门（应拒绝把垃圾锁进 T_map_odom）：
ros2 topic echo /relocalization/pose | grep -m1 covariance   # 不可信期 cov[0] 应 =100.0（远大于 loc_cov_invalid=1.0）
journalctl -u hunter-edge -f | grep -E '拒绝刷新|odom→base|超 sanity 门限'   # 加固④上游命中共识
# 5.3 观测安全侧不再假锁（本次假锁死机制的正向验证）：
journalctl -u hunter-edge -f | grep '定位无效'      # 应出现“地图边界监护暂不介入…由碰撞闸兜底”WARN
ros2 topic echo /safety/state --once                # 关键判据：不出现 MAP_EDGE_STOP（碰撞闸 COLLISION_STOP 仍工作）
# 5.4 恢复验收：重新给上 LiDAR / 驶回建图区，NDT 收敛后：
ros2 topic echo /relocalization/pose --once | grep -m1 covariance   # 应回到 ~0.01
ros2 topic echo /safety/state --once                                  # WARN 停止、边界监护重新介入（自动回弹）
```

**步骤 6·遥控闭环回归（确认与止血后行为一致）**

```bash
# 平台接管/操控，确认指令链与止血时一致：
ros2 topic echo /cmd_vel       # 随 /remote/command 油门出现非零 linear.x（前 +0.4 / 倒 -0.4）
ros2 topic echo /safety/state --once   # 定位健康时不应因发散误锁 MAP_EDGE_STOP
```

**步骤 7·实机结果回填（2026-10-19 车端 `agilex@agilex`，nproc=12）**

| 步骤 | 结果 | 实测 |
|---|---|---|
| 1 干净重编 | ✅ | `hunter_safety` 57.6s、`hunter_relocalization` 1min5s（真实编译）；`ndt_relocalization_node` 命中 `max_valid_translation`；`safety_guard` 编入由步骤 3 `loc_cov_invalid=1.0` 已加载佐证（旧二进制不声明此参，`ros2 param get` 会报 unset） |
| 2 systemd | ✅ | `systemctl show hunter-edge` → `Nice=-10`、`CPUWeight=200` |
| 3 参数 | ✅ | `point_filter_num=2`、`max_valid_translation=2000.0`、`loc_cov_invalid=1.0`；`OMP_NUM_THREADS=4`（需 `sudo cat`，普通用户 `Permission denied`） |
| 4 抗饥饿 | ✅（部分） | 空载 `/Odometry` 稳 ≈10.0Hz（min 0.083/max 0.118）、位姿米级（x 0.021 / y -0.013 / z 0.018）；未真起 rviz2 加压（`pkill rviz2` 无对象），**超订极限判据本轮未达** |
| 5 发散不假锁 | ⚠ 未验收 | 当轮未制造定位不可信环境（重定位全程收敛）；`/safety/state` 为 `CMD_TIMEOUT`→`SLOWDOWN`（正常降级，非 MAP_EDGE_STOP）。**延后专项验证** |
| 6 遥控闭环 | ✅ | `/cmd_vel` 随遥控油门出现非零 `linear.x`（倒车段 -0.4 / 前进段 +0.4），定位健康时未误锁 MAP_EDGE_STOP |

- **仍待办**：① 步骤 5——需在有把握制造定位发散/未收敛的窗口下（拔/遮 LiDAR 冷启动、或搬离建图区使 NDT fit>ceiling）专项确认 `拒绝刷新`/`odom→base 超 sanity` 日志 + `/safety/state` 不假锁 MAP_EDGE_STOP + 恢复自动回弹；② 步骤 4 超订极限——如需严格复现旧故障，手动起 rviz2 累积显示重跑 `hz /Odometry`。
- 待上述两项补齐后，再将本节标题“⚠ 部分已通过”改为“✅ 已通过”，并同步把 Deployment_Guide §7.4 行与 User_Manual 头部/§6.2 行的“实机验证待办”标注改“✅”。未补齐前不得整体标 ✅。

**七、分发**

- 源码变更：`hunter_edge.launch.py`、`fast_lio2_params.yaml`、`localization.launch.py`、`hunter-edge.service`、`ndt_relocalization_node.cpp`、`relocalization_params.yaml`、`safety_guard.cpp`、`hunter_autonomous_nav.launch.py`。
- 实机同步：`git pull` → 干净重编 `hunter_safety`+`hunter_relocalization` → `systemctl daemon-reload`（service 改动）→ `systemctl restart hunter-edge`。

## V0.1.32（Date：2026-10-22）

**主题：车-云链路「静默失效」事故闭环——`data_agent` 五项可靠性修复（断链判据重写 / 投递失败必落库 / SQLite O(n²) 治理 / 断链跳过 TCP 探测 / 崩溃 respawn）+ 三项现场改进（断链不录包 / 回放去重 / 缓存计数口径修正）**

**一、现场**

2026-10-22 13:45–15:10，HUNTER-001 在平台侧表现为完全「离线」约 1.5 小时：实时遥测与心跳全无，**而车端零告警、零事件**。车端自证：`hunter-kafka-check` 退 40（网络，10 配置/20 证书/30 认证/**40 网络**/50 Topic/60 投递），`data_agent` 启动即 `Connect to ipv4#192.168.31.35:9093 failed: Connection refused`，全程 `累计送达 0 条`。

`tcpdump` 定案：车端 SYN 正常发出（`Flags [S]`），3.3ms 后收到 `Flags [R.]`；**同一服务器的 80/8080 端口可达，9093/8443/9000 全部 refused** → 根因在**平台侧 docker 发布端口层**（DNAT/容器监听），已由平台修复。**非车端网络问题、非 Kafka 配置问题、非 `data_agent` 问题**。

但复盘暴露 5 个车端缺陷，使这次故障**既不可见、也不可恢复**：断链事件永不触发（静默 1.5 小时）、失败消息实际被丢弃（日志却写"转本地缓存"）、缓存库在断链时把自己拖进 `D` 状态（84MB 库 + 平均 CPU 28.7%）、每秒向已 refused 的端口发一次无效探测、节点崩溃后服务仍显示 `active`。修复过程中另发现两个现场改进点（断链白录 bag、回放重复上行），一并处理。

**二、车端缺陷与修复（①–⑤ 可靠性修复，⑥–⑧ 现场改进）**

| # | 缺陷 | 位置（修复前） | 修复 |
|---|---|---|---|
| ① | `communication_loss` **永不触发**：门控条件是 `kafkaReady()==false`，而它等价于 `outq_len() >= kafka_queue_limit_(50)`；broker 不可达时消息按 `message.timeout.ms=60000` 逐条超时淘汰，队列实际到不了 50，`kafkaReconnect()` 里的判定分支永不执行 | `data_agent.cpp` `kafkaReady()` / `kafkaReconnect()` | 新增 `checkLinkHealth()`（由 `packAndPublish` 10Hz 驱动）：判据改为**「距上次投递被 broker 证实超过 `comm_loss_duration`（默认 10s）」**，与队列深度彻底解耦；中断/恢复各打一条日志（含失败数、缓存数、中断时长），事件一条只报一次，恢复后重新武装 |
| ② | **投递失败不落库**：`dr_cb` 失败分支只打 `转本地缓存`，消息随回调返回即丢弃 → 断链期间数据事后无法补传 | `data_agent.cpp` `dr_cb()` | 实时消息失败**真正** `sqliteCache()`（回放行则保留原行等下轮，至少一次）；日志改为真实语义并带累计缓存条数；payload 不可用时明确写"无法转存"。`sqliteCache()` 增加 `sqlite3_step` 结果检查（旧实现不看返回值，磁盘满/库损坏时"看起来已缓存、实际一条没写"） |
| ③ | **SQLite O(n²)**：每次插入前都做一次**无索引全表** `DELETE ... WHERE timestamp < ?`；10Hz 写入下随行数增长退化为平方级 → 现场 84MB 库时进程常驻 `STAT=D<l`、平均 `%CPU 28.7` | `data_agent.cpp` `sqliteInit()` / `sqliteCache()` | `PRAGMA journal_mode=WAL`（失败仅 WARN，退回默认 journal）+ `PRAGMA synchronous=NORMAL` + `CREATE INDEX IF NOT EXISTS idx_telemetry_cache_ts ON telemetry_cache(timestamp)`；过期清理**降频到 60s 一次**并在真删到行时打 INFO（时钟漂移导致整批被判过期的现场可解释性） |
| ④ | **每秒向已 refused 的 9093 发一次 TCP 探测**（`network_latency_ms` 采样），断链时纯噪声 | `data_agent.cpp` `publishHealth()` | 仅在 `kafka_connected_`（投递已被证实）时采样；断链期间跳过，恢复后自动继续。注意 `kafka_connected_` 仍**不**作为 produce 前置门控（V0.1.13 的死锁教训），只用于探测/展示 |
| ⑤ | **无 respawn**：`ros2 launch` 不因子节点退出而退出，`hunter-edge.service` 的 `Restart=on-failure` 只覆盖 launch 父进程 → `data_agent` 崩了服务仍 `active`，平台永久离线 | `data_agent/launch/data_agent.launch.py` | `respawn=True, respawn_delay=2.0` |
| ⑥ | 断链事件**白录 20s bag**：`communication_loss` 也走 `triggerBagRecord()`，而它恰是链路/平台侧根因，车端 bag 无法定位，且断链瞬间最该轻载 | `data_agent.cpp` `reportEvent()` | 新增 `isLinkOnlyEvent()` 白名单：`communication_loss` 不再触发 rosbag；其余车端事件（超速/急停/低电/急转/急加速…）保持文档 14.3.2 前后各 10s 录制 |
| ⑦ | 回放**重复投递**：`sqliteReplay()` 按 `id ASC LIMIT 50` 取批，不排除"已投递未证实"的行 → 慢链路下同一批被反复 produce（平台按 `seq` 幂等不脏数据，但 8 万条存量回放成倍浪费上行带宽） | `data_agent.cpp` `sqliteReplay()` / `dr_cb()` | 新增 `replay_inflight_ids_`（`std::set`，`db_mutex_` 保护）：取批时快照并在扫描中跳过在途 id，`dr_cb` 不论成败都放行；**先登记再 produce**（反过来会留下永不回收的 id，使该行再也不能回放） |
| ⑧ | 断链日志**计数口径错误**：`本地缓存 N 条待回放` 取的是 `cached_count_`（本进程累计写入、只增不减），不是库里真实剩余行数 → 事故复盘会把"累计写过 612 条"误读成"还有 612 条没回放"（2026-10-22 复验即出现 612 对 574 的假差额） | `data_agent.cpp` `checkLinkHealth()` | 新增 `sqliteRowCount()`（`SELECT COUNT(*)`，**只在断链/恢复各调一次**，不进 10Hz 热路径）：日志改为同时给出"待回放行数 / 本进程累计写入"；`communication_loss` 事件 `data.cached` 改为真实行数并新增 `cached_total`（只增字段） |

**三、向后兼容**

- 不新增/修改任何 ROS 话题、消息、服务（遵 `.ai-rules` 接口契约），`event_type`/`event_level` 取值与 `event.schema.json` 一致；`communication_loss` 的 `data` 从 `{duration_s,threshold_s,broker}` 扩展为 `{duration_s,threshold_s,broker,failed,cached}`（只增字段，`additionalProperties` 允许）。
- ①③④⑦为内部实现替换，对外报文语义不变；⑥只减少 bag 录制，不影响事件上报；⑤只影响进程生命周期。
- 回放仍是**至少一次**：平台按 `(vehicle_id, seq)` 幂等收敛；`dr_cb` 对"已送达但回执超时"的消息仍可能补写一份缓存（平台去重）。
- 旧库自动升级：WAL 是持久化属性，首次启动即完成迁移；索引 `IF NOT EXISTS`，84MB 库首次启动会多花一次建索引时间（秒级）。

**四、实测验收（①–⑤：2026-10-22 16:28–16:30 车端故障注入，全部通过；⑥–⑦：16:52 故障注入复验，全部通过；⑧：17:22 断链对账通过）**

| # | 判据 | 实测 |
|---|---|---|
| ⑤ | `kill -9` 后自动恢复 | `16:28:58 process has died (exit -9)` → `16:29:00 process started` → **2s 自愈**，`ros2 node list` 始终有 `/data_agent` |
| ① | 断链 10s 内告警 | `16:29:34 Kafka 链路中断：已连续 10.1s 无成功投递（累计失败 0 条，本地缓存 61 条）` + `事件触发：communication_loss (critical)`，**仅一条** |
| ① | 恢复日志 | `16:30:23 Kafka 链路恢复：中断约 48.5s（累计送达 314 条，缓存 595 条待回放）` |
| ② | 失败必落库 | 48.5s 断链缓存 **595 条**（≈11.5 条/s = telemetry 10Hz + health 1Hz + 事件），恢复后 12 轮 `50 条/轮` 全部回放，**一条不丢** |
| ② | 数字自洽 | 断链缓存 595 + 恢复瞬间冲出 librdkafka 队列的 314 = 909；回放结束累计送达 929，差 20 = 回放 2s 的实时 telemetry（10Hz×2s） |
| ③ | WAL/索引/负载 | `PRAGMA journal_mode → wal`、`idx_telemetry_cache_ts` 存在、`-wal`/`-shm` 正常 checkpoint；`STAT S<l`、`%CPU 4.0`、`RSS 33MB`（修复前 `D<l` + 28.7%） |
| ④ | 不再无效探测 | 48.5s 断链期间 **0 条** `Connect ... failed`（修复前每秒一条） |
| ⑥ | 断链不录包 | ✅ **16:52 复验**：50.0s 断链全程 `/data/rosbag` 无新增目录（最新仍是 16:29，为改代码前旧二进制的产物），`journalctl` 内亦无 `bag record` 记录（修复前实测：每次断链白录 20s、含 4 路 topic） |
| ⑦ | 回放去重 | ✅ **16:52 复验**：`50×11 + 24 = 574` 条，累计送达每轮严格 `+50`（另含 1–2 条实时）单调递增，无重复批次，12 轮后 `telemetry_cache.rows = 0`；`--since` 回溯确认 951 之后无任何后续轮次（无死循环/无重复 produce） |
| ⑧ | 缓存计数口径 | ✅ **17:22 复验通过**：`缓存 376 条待回放`（恢复瞬间真实行数）→ 各轮 `50×7 + 33 = 383`，ΣN−X = **7 条**为回放突发期间本地队列满而落库的实时报文（1.9%），随后库归零、无收尾轮次。修复前同一场景报的是累计写入 612（对实际 574），方向与量级都无从解释 |

**注**：`累计失败 0 条` 属正常——`REJECT` 场景下 librdkafka 要等 `message.timeout.ms=60s` 才产生失败回调，而两次注入（48.5s / 50.0s）都在超时前恢复；落库走的是 `kafkaReady()==false`（队列满）路径。两条路径（队列满 / 超时淘汰）都保留，互为兜底。第二次复验出现 `612（累计写入） vs 574（回放条数）` 的差额，**不是丢数据**（`rows=0`、送达计数与回放条数精确吻合 951 = 337 + 574 + 40），而是 ⑧ 所列的计数器口径缺陷——同一计数器无法区分"已回放删除"与"仍待回放"。

**五、事故数据回补（同一次事故的正向结论）**

网络恢复后，旧进程按 `500 条/秒` 把事故期间积压的 **84MB / 约 8 万条**缓存全部回放并逐条删行（`15:14–15:19`，日志 `累计送达 79394 → 80569`，最后 `本轮回放 1 条` 收尾），库内 `freelist 20479/20483` 页、`rows=0`。即**这一小时的数据没有丢失，只是迟到**。这也反向证明修复②之前"失败即丢弃"的路径在本次事故中未生效（当时全部走了队列满落库分支），但它是随时会踩中的真实缺陷。

**六、风险与遗留**

- ①的阈值仍是 `comm_loss_duration`（默认 10s）：启动阶段 SASL/mTLS 握手超过 10s 会先报一次 `communication_loss`，随后恢复——语义上正确（平台此时确实收不到数据），但会产生一条"误报"观感；如需更宽松可调该参数。
- ②的至少一次语义在"已送达但回执超时"时会补写一份缓存行，恢复后重复送达一次（平台幂等）。
- ⑦只解决"在途重复"，不解决"成功后删行失败"导致的重复；后者由平台幂等兜底。
- ⑦跳过在途行意味着回放**顺序**在 `replay_inflight_limit_`（默认 200）窗口内可能轻微乱序（先发 1–50，在途未证实期间发 51–100…）；平台按 `(vehicle_id, seq)` 入库与展示，不影响幂等，但若将来有严格时序消费方，需改为"按 id 串行等待"。另外该集合依赖 `dr_cb` 必被调用才回收：librdkafka 的 `message.timeout.ms` 保证每条消息最终必有回执，故不会永久泄漏。
- 本次未处理（与本事故无关）：`/scan` 断流、`fastlio_mapping: No Effective Points!`、`/perception/lidar/obstacle_cloud` 不更新等感知链问题。

**七、分发**

- 源码变更：`hunter_agents/data_agent/src/data_agent.cpp`、`hunter_agents/data_agent/include/data_agent/data_agent.hpp`、`hunter_agents/data_agent/launch/data_agent.launch.py`。
- 实机同步：
  ```bash
  cd ~/HunterEdge
  rm -rf build/data_agent install/data_agent      # 规避 V0.1.26 mtime 空编译陷阱
  colcon build --symlink-install --packages-select data_agent
  sudo systemctl restart hunter-edge
  ```
- 验收（不依赖日志通道也能判）：
  ```bash
  python3 -c "import sqlite3;c=sqlite3.connect('/data/data_agent/telemetry.db');\
  print(c.execute('pragma journal_mode').fetchone(),\
  c.execute('pragma index_list(telemetry_cache)').fetchall(),\
  c.execute('select count(*) from telemetry_cache').fetchone())"
  # 期望 ('wal',) + idx_telemetry_cache_ts + 链路正常时 rows 不持续增长
  ps -o pid,stat,%cpu,rss,args -C data_agent_node     # 期望 S<l，不再 D<l
  sudo ss -tnp | grep 9093                            # 期望 data_agent_node 两条 ESTAB（acks=1 / acks=all）
  sudo journalctl -u hunter-edge --since "1 hour ago" --no-pager | grep '\[data_agent'
  ```
- 故障注入复验（约 1 分钟，只拦本机出向）：`sudo iptables -I OUTPUT -p tcp --dport 9093 -j REJECT` → 等 45s → `sudo iptables -D OUTPUT -p tcp --dport 9093 -j REJECT`；期望日志顺序 `链路中断(10.1s)` → `communication_loss` → `链路恢复` → `断点续传`，且**不再出现 rosbag 录制行**。
- 对账（⑧ 新增判据）：`链路恢复` 行的「缓存 **X** 条待回放」是**恢复瞬间库内真实行数**，随后各轮 `本轮回放 N 条` 之和 **ΣN 应 ≥ X**；差额 = 回放突发期间本地队列满、实时报文顺手落库的新行（被后续轮次一并带走），正常应是 X 的百分之几。最终 `telemetry_cache` 必须归零，且最后一轮日志之后不再出现回放轮次。2026-10-22 17:22 实测：X=376、ΣN=383、差 7（1.9%），库归零。

---

## V0.1.33（Date：2026-10-27）

**主题：车端相机「全程 0 帧」定案——RealSense SDK 2.58→2.59.1 升级使 wrapper 组件库 ABI 断链（`class_loader::LibraryLoadException`），重编对齐后恢复；本版无源码改动，附现场误判链与五项待办**

### 一、现场

2026-10-27 `hunter-edge` 启动（09:27:30）后相机从未出过一帧，雷达同期被判「闪一下即停」。

**追溯（重要）：那条 `2026-10-26 17:14:08` 是「错误时钟」写下的时间戳。** 该会话真实起点为 **2026-10-27 08:46:59**（开机约 19s 后自启），开机早期系统时钟仍停在错误的 10-26 17:14，**约 17s 后被 NTP 一次性向前拨了 55970 秒（15h32m）** —— 同一份 journal 因此同时存在「10-26 17:14」与「10-27 08:47」两套时间（详第八节）。此后 09:27:32、09:47:22 两次重启同位置同样失败 ⇒ 确定性缺陷、与时钟无关；相机从 08:46:59 一直坏到 10:0x 重编后修复。

- `health_monitor` 每 2s `camera 话题频率异常：0.0Hz < 5.0Hz，持续超 2s`；1s 窗口实测计数为 0 且已过 15s 启动宽限 ⇒ 话题**真的没有生产者**，不是掉帧。
- `vision_perception` `彩色图像超时：819.36s 未收到 /camera/camera/color/image_raw（阈值 0.50s）` 随时间单调累加（09:41:11 − 09:27:32 ≈ 819s）⇒ 该计数器是「自节点启动起从未收到」，**与时间戳基准无关**（排除了 `use_lidar_clock` 类怀疑）。
- 平台侧远程视频 `0.00 Mbps / 0.0 fps` 黑屏（`remote_agent` 订阅同一话题无帧）。
- 硬件侧全部无辜：`dmesg` 仅 `Found UVC 1.50 device Intel(R) RealSense(TM) Depth Camera 435 (8086:0b07)`，无 uvcvideo 报错、无 USB reset/掉线；`lsusb` 在位；`/dev/video0-5` 齐全；`fuser` 无进程占用。
- 雷达**驱动侧**始终健康：`rslidar_sdk_node` `S<l` 常驻、`/lidar_points` 10.007Hz、`tcpdump` 见 `192.168.1.200:6699 → 192.168.1.102:6699` 稳定流入（909 包/5s、0 dropped）、6699+7788 双 socket 正常 bind。（「XXXXs 未更新」那个数字确实属于视觉节点对**相机**的计时、不是雷达——这点成立；**但不能**由此推出雷达感知链路健康：同日 `/scan` 与 `/perception/lidar/obstacle_cloud` 真的只有一帧，见第八节。）

### 二、根因

手工拉起即暴露：

```
terminate called after throwing an instance of 'class_loader::LibraryLoadException'
  what():  Could not load library dlopen error: librealsense2.so.2.58:
           cannot open shared object file: No such file or directory
[ros2run]: Aborted
```

系统侧：`dpkg -l` 五件套（librealsense2 / -dev / -gl / -udev-rules / -utils）均为 **2.59.1-0~realsense.9165**；磁盘只有 `librealsense2.so.2.59(.1)`，`ldconfig -p` 正常注册 ⇒ **不是路径问题，是 SONAME 版本错配**。wrapper 源码 `realsense2_camera` 4.58.4（CMake `find_package(realsense2 2.58.0)`）的组件库 `librealsense2_camera.so` 是 SDK 升级前编的，NEEDED 写死 `.so.2.58`，2.59 无法满足。

两个放大因子（比库缺失本身更值得修）：

1. `realsense2_camera_node` 只是 `rclcpp_components` 容器，真正的库由 class_loader **运行时 dlopen** ⇒ `ldd` 主程序**查不到该依赖**（本次排障因此一度误判「依赖正常」）。
2. `hunter_full.launch.py` 对两个硬件驱动只做 include、**无 `respawn`** ⇒ abort 一次 = 整个会话没有相机，不自愈。

### 三、修复与实机执行（无源码改动、无 ROS 接口变更）

```bash
cd ~/HunterEdge
rm -rf build/realsense2_camera install/realsense2_camera   # 必须删 build，规避 V0.1.26 mtime 空编译陷阱
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select realsense2_camera
source install/setup.bash
```

编译期 `FindCUDA.cmake CMP0077` 告警为 OpenCV 4.10 CUDA 配置的既有噪声（最终 `Finished`，无 error），不影响产物。

### 四、实测验收（2026-10-27 09:52 车端）

| 判据 | 实测 |
|------|------|
| 驱动自述版本 | `RealSense ROS v4.58.4` / `Built with LibRealSense v2.59.1` / `Running with LibRealSense v2.59.1` |
| 设备识别 | `Device with serial number 117322070409`、`RealSense D435`、`USB type: 3.2`、FW `5.14.0` |
| 流 profile | Depth Z16 640×480@15、Color RGB8 640×480@15、`RealSense Node Is Up!` |
| 彩色频率 | `ros2 topic hz /camera/camera/color/image_raw` = **14.986~15.029Hz**（min 0.062 / max 0.071，std dev 收敛 0.001） |
| 平台远控视频 | **4.00 Mbps / 15.0 fps / 端到端 25ms**，画面正常播放 |
| **生产路径（systemd）复验** | 关掉手工节点、`ps` 确认无残留 → `sudo systemctl restart hunter-edge` → 彩色 **15.100 / 15.042 / … / 14.999Hz**（min 0.058 / max 0.073）、深度 **14.995 / 14.991 / … / 14.989Hz**（std dev 收敛 0.0005） |
| `journal` 复查 | `grep -E 'LibraryLoad\|Infra\|has died'`：重编后**无任何 LibraryLoad、无 has died**；**无 Infra** ⇒ 生产 launch 的 `enable_infra1/2:=false` 生效（848×480@30 双流只出现在手法命令里） |
| SDK 钉版 | `apt-mark hold` 五件套全部 `set on hold`，`apt-mark showhold \| grep realsense` 五项在位 |
| 全链路复验（10:1x） | `/scan` 9.984~10.004Hz、`/perception/lidar_objects` 9.977~10.004Hz、`/perception/lidar/obstacle_cloud` 9.995~10.019Hz、`/system/health` = **OK**（4/4 running，CPU 38.1% / GPU 49% / 54℃ / 内存 6.6%）⇒ 导航障碍输入、感知输出、里程计全部到位 |

### 五、向后兼容与风险

- 零话题/消息/服务/参数变更；只重编本机 wrapper，不动 `hunter_full.launch.py`。
- realsense-ros 4.58.4 对 SDK 2.59.1 属「向上兼容编译」：本次验收覆盖 RGB + Depth 两路主用流；`accel/gyro`、`align_depth`、`rgbd` 等未启用流未验证，启用前需另行复验。
- 日志残留 3 类噪声，均非故障：`Could not set param: rgb_camera.power_line_frequency with 3 Range: [0,2]`（D435 不支持该档）、`frame's time domain is HARDWARE_CLOCK. Timestamps may reset periodically.`、启动阶段 2 次 `Frames didn't arrived within 5 seconds`（USB 带宽瞬时不足）。
- 手工验证命令未带 `enable_infra1/2:=false`，日志出现 Infra 848×480@30 双流（额外 USB 带宽）；生产 launch 已显式关闭，**已复验生产路径无 Infra**。

### 六、待办（本版未处理，如实登记）

> ✅ **已由 V0.1.34 落地**（新增 `hunter_clock_gate.sh` + `ExecStartPre ①` + `hunter_edge_up.sh ⓪`，有界等待 NTP 首同步、超时不阻断）。原判断与整改要点留档：**给整栈加「时钟同步闸门」。** `hunter-edge.service` 只依赖 `hunter-can` + `network-online.target`（第 9 行），**不等时钟同步**，开机 19s 就把整栈拉起；本次 RTC/NTP 在 17s 后把时钟向前拨 55970s，整栈时间轴当场断裂。整改：① 单元加 `After=time-sync.target` + `Wants=systemd-time-wait-sync.service`；② `ExecStartPre` 增有界等待（最多 60s，超时只 WARN 不阻塞，避免车永远起不来）；③ 查 RTC 是否保持（`sudo hwclock -r` / `timedatectl`）并 `hwclock --systohc` 回写。**此项决定本次事故是否每次上电重演。**

1. **【已定性】`fastlio_mapping` `-11(SIGSEGV)`**：2026-10-27 08:52:57 崩溃，由**时钟跳变诱发的里程计发散**引起（第八节），不是随机崩溃。取证受限：该机无 `coredumpctl`（Jetson 默认未装 systemd-coredump），`dmesg` 也无 segfault 记录。真正待修的是**无 respawn**：`/Odometry` 从 08:52:57 断到 09:27:28，35 分钟无定位。
2. **驱动/算法节点无 respawn**：`realsense2_camera`、`rslidar_sdk`、`ch10x_driver`、`fast_lio` 崩溃后整会话不自愈（全仓仅 `data_agent` 有 respawn）。本次两个故障（相机 abort、FAST-LIO2 段错误）**都是被这条放大的**。
3. **`health_monitor` 的 `critical_nodes_` 空转**：该参数赋值后从未使用，无法区分「进程不存在」与「数据饥饿」；措辞还把 abort 误导为「疑似 CPU 过载」。配套整改见第九节的监控盲区分析。
4. **启动无二进制依赖预检**：建议 `hunter_edge_up.sh` 在 launch 前对驱动组件库做 `ldd \| grep 'not found'`，命中即 fail（把本类故障报在启动日志第一屏，而非 800 秒后靠「相机 0 帧」发现）。
5. **相机时间域为 `HARDWARE_CLOCK`**：需复验 `sensor_fusion` 的 `max_sync_diff(0.10s)` 时间对齐是否受影响（`ros2 topic echo /camera/camera/color/image_raw --field header.stamp --once` 与主机时钟比对）。
6. **停机时部分节点以 `-6(SIGABRT)` 而非 `-2(SIGINT)` 退出**：`hunter_base`、`ch10x_driver`、`rslidar_sdk` 在每次 `systemctl stop/restart` 时均如此（析构阶段 abort，可能留 core），不影响运行，低优先级。
7. **舰队风险排查**：若 SDK 升级源自批次性 `apt upgrade`，**其他车会同时中招**。逐台比对 `dpkg -l \| grep -i realsense` 与 `ros2 run realsense2_camera realsense2_camera_node` 首屏 `Built with LibRealSense` 是否一致；并查 `grep -iE 'realsense' /var/log/apt/history.log*` 定位升级时间点。

### 七、分发与现场执行

- 源码无改动；产物为 `build/realsense2_camera` + `install/realsense2_camera`（`--symlink-install`）。
- **已执行（2026-10-27 10:0x）：** 关闭手工节点 → `apt-mark hold` 五件套成功 → `sudo systemctl restart hunter-edge` → 生产路径彩色/深度均 ≈15Hz。
- 现场顺序（供其它车复用）：关掉手工测试节点 → `apt-mark hold` 五件套 → `sudo systemctl restart hunter-edge` → 验收：

```bash
ps -eo pid,etime,args | grep realsense2_camera_node | grep -v grep   # 生产路径应有且仅有一个
ros2 topic hz /camera/camera/color/image_raw                         # ≈15Hz
ros2 topic hz /camera/camera/depth/image_rect_raw                    # ≈15Hz（RGBD 由 depth 派生）
sudo journalctl -u hunter-edge -b --no-pager | grep -E 'LibraryLoad|realsense2_camera_node|has died'
```

### 八、同时段的雷达侧故障链（2026-10-27 08:47–08:53，与相机故障**互相独立**）

**根因是系统时钟跳变，不是雷达、也不是 FAST-LIO2 自己坏的。** 08:47:16 的日志里同时出现三个互不相关、却指向同一个数的量：

| 观测 | 数值 | 反推 |
|------|------|------|
| `obstacle_cloud ... has not been updated for` | **55970.22 s** | 缓冲区创建于（当时错误的）`Oct 26 17:14:25.95` |
| `Message Filter dropping ... at time 1793006065.936` | 1793062036.19 − 1793006065.936 = **55970.24 s** | 该消息时间戳同样落在 15.5 小时前 |
| `彩色图像超时：17.34s`（`steady_clock` 实现，免疫时钟跳变） | 17.34 s | vision 节点生于 **08:46:59** |

三式联立即得：**栈在 08:46:59 启动时系统时钟还停在 10-26 17:14（落后 15h32m），约 17 s 后被 NTP 一次性向前拨了 55970 s**。这一拨把「启动瞬间产生的消息时间戳、TF 缓存、代价地图 `last_updated`」全甩到 15 小时前，全栈时间轴瞬间断裂（附同一 0.6s 内的完整时间线）：

| 时刻 | 证据 | 含义 |
|------|------|------|
| 08:47:16.28 | `obstacle_cloud ... not updated for 0.29s` → 0.49 / 0.69 / 0.89 s 递增 | **只收到了一帧**（即现场“雷达闪一下即停”那一帧），之后全被丢弃 |
| 08:47:16 | `fastlio_mapping: No Effective Points!` ×3 | 点云配准失效（与时钟跳变同步） |
| 08:47:16.218 → .785 | NDT `✓ 收敛 fit=3.292` → `✗ 严重失准 fit=9442913722596065968.000>5.00` | **0.5 秒内从收敛炸穿** |
| 08:53:26 | `/scan ... 370.52s`、`obstacle_cloud ... 370.69s` | 6 分钟内**零有效感知输入**，代价地图全空 |
| 08:53:29 | `odom→base 平移 845515.4m 超 sanity 门限/非有限（疑似里程计发散），退化单位阵` | **FAST-LIO2 已发散到 845 公里** |
| 08:53:29 | `✗ 严重失准 fit=623541747712.000>5.00` | NDT 完全失准，对外宣告未收敛 |
| 08:53:26+ | `Message Filter dropping message: frame 'base_link' ... earlier than all the data in the transform cache` | TF 只剩最近 10s，消息时标落在缓存之外 → **FAST-LIO2 发异常/超前时间戳** |
| 08:53:27+ | `Extrapolation Error ... Requested time 1793062368.467666`（多秒不变） | planner 卡在死循环重试同一次 30s 前的 TF 查询 |
| 08:52:57 | `fastlio_mapping exit -11` | **5 分 40 秒后段错误**；845km 是它死前的最后一帧 |
| 08:53:29 | `[ESTOP] 保持急停（estop=false，AUTO条件=未满足）` | 最终态（安全侧行为正确） |

**因果序（修正版）**：时钟跳变 → ① 消息时间戳落在 TF 缓存之外 → `MessageFilter` 整帧丢弃 → `observation buffer has not been updated` 从 0.29s 一路涨到 370s；② FAST-LIO2 配准失效（`No Effective Points!`）→ 里程计发散到 845 km → 5'40" 后 SIGSEGV；③ NDT 同一 0.5 秒内从收敛到 9.4e18。**`/scan` 断流与 FAST-LIO2 发散是同一上游原因的两个并列后果，不是因果关系**（上一版写成因果，已更正）。

> ⚠️ **两次判断更正**：① 不能因 `/lidar_points` 10Hz 就断言雷达链路健康——`/scan` 与 `/perception/lidar/obstacle_cloud` 当时确实只有一帧，现场“雷达闪一下即停”的观察**准确**；② 但“闪一下”既不是雷达坏了、也不是 FAST-LIO2 发散造成的，而是**时钟跳变让这些消息全被时间过滤器丢掉**。

**已定项 / 已排除**：不是雷达（`/lidar_points` 全程 10Hz、`rslidar_sdk` 未退出）、不是 D435（`dmesg` 无 uvc 报错）、不是 CPU 饥饿（跳变瞬间 CPU 38%）、不是 `pointcloud_to_laserscan` 退出。

**待查（决定是否每次开机重演）**：RTC 为何落后 15.5h —— `timedatectl`（看 `RTC time` 与 `System clock synchronized`）、`sudo hwclock -r`、`journalctl -b \| grep -iE 'Time has been changed\|timesyncd\|chrony'`。若开机 RTC 恒落后，则**每次上电都会重演本次事故**；且时钟错 15h 会连带打掉 Kafka TLS 与遥测时间戳。

### 九、监控盲区（本次最值得改的一条）

`health_monitor` 只订阅四个**驱动层**话题：`/lidar_points`、`/camera/camera/color/image_raw`、`/imu/data`、`/chassis/state`。于是本日四类故障：

| 故障 | health_monitor 能否感知 |
|------|--------------------------|
| 相机进程 abort | 只能看到“话题 0Hz”，且措辞误导为“疑似过载” |
| `/scan` + `/perception/*` 断流 | **不能** |
| FAST-LIO2 发散到 845km | **不能** |
| FAST-LIO2 段错误 | **不能** |

全程只报了一条 `camera 0Hz`，其余 8 分钟静默——这就是“带病运行 8 分钟无人发现”的机制。整改方向：① 把已赋值但从未使用的 `critical_nodes_` 接上**进程存活检查**（按 `/proc` 或 `ros2 node list` 对应 PID）；② 新增 `/scan`、`/Odometry`、`/perception/lidar_objects` 三条输出层频率看门狗（阈值可宽，≥1Hz 即判活）；③ 措辞区分“进程不存在（崩溃）”与“话题变慢（饥饿）”；④ 把**时钟同步状态**（`timedatectl` 的 `NTPSynchronized`）纳入健康上报——本次事故它同样完全无感，而它是所有时间戳的基准。

---

## V0.1.34（Date：2026-10-27）

**主题：上电时钟闸门——根治 V0.1.33 §8 的“整栈时间轴断裂”。新增 `hunter_clock_gate.sh`（NTP 首同步有界等待，超时不阻断），接入 `hunter-edge.service` 的 `ExecStartPre` 与 `hunter_edge_up.sh` 的 ⓪ 步骤**

### 一、现场与证据（实测）

```
Jan 01 08:00:36  systemd-timesyncd[626]: System clock time unset or jumped backwards,
                 restoring from recorded timestamp: Mon 2026-10-26 17:13:55 CST
Oct 27 08:47:16  systemd-timesyncd[626]: Initial synchronization to time server
                 193.182.111.143:123 (1.pool.ntp.org).
```

- `Jan 01 08:00` = **上电时 RTC 没有可用时间**（内核时钟倒退到 epoch 附近）；`restoring from recorded timestamp: 10-26 17:13:55` = 上一次真实断电时刻（timesyncd 状态文件）。
- 整栈在该状态下的 **08:46:59 就已拉起**（开机约 19s），17 秒后 NTP 首同步 → 时钟**向前拨 55970s（15h32m）**，与 V0.1.33 §8 用「未更新秒数 / 消息 `header.stamp` / `steady_clock` 节点年龄」三式联立算出的跳变量**完全一致**。
- 当前 `timedatectl`：`RTC time` 与 `System time` 仅差 1s、`System clock synchronized: yes`、`NTP service: active`、`RTC in local TZ: no`；`sudo hwclock -r` = 系统时间。⇒ **RTC 硬件本身能读写，但在断电期间不保持**（内核运行期会周期性回写 RTC，所以在线时看起来是对的）。

### 二、根因

`hunter-edge.service` 的依赖链只有 `Requires=hunter-can.service` + `After=hunter-can.service network-online.target`（V0.1.33 第 9 行）——**“网络通”不等于“时钟准”**。RTC 丢时 + 栈抢在 NTP 首同步前启动，两者叠加就必然发生一次大跳变；而 ROS 2 的消息时间戳、TF 缓存、costmap `ObservationBuffer` 都以墙钟为基准，跳变即全链路失效。

### 三、改动（三处，均为启动链，不动算法/参数）

| 文件 | 改动 |
|------|------|
| `src/hunter_bringup/scripts/hunter_clock_gate.sh` | **新增**（116 行）。等 `NTPSynchronized=yes`，上限 `HUNTER_CLOCK_GATE_TIMEOUT`（默认 60s）；超时只 WARN。同步后再报「闸门前 → 闸门后」的修正量（>2s 即告警“本机 RTC 又不准了”）。**退出码恒 0**：时钟问题绝不阻断整车启动 |
| `src/hunter_bringup/systemd/hunter-edge.service` | 新增 `ExecStartPre ①` 调用闸门（CAN 复查降为 `ExecStartPre ②`）；`TimeoutStartSec` 180 → **240**（为最长 60s 等待留余量）；注释记录“为何不用 `After=time-sync.target`” |
| `src/hunter_bringup/scripts/hunter_edge_up.sh` | 新增 `⓪ 时钟闸门` 步骤（与单元同源、幂等），使手工 `bash hunter_edge_up.sh` 也受保护 |

**设计取舍（故意不用 `After=time-sync.target` + `systemd-time-wait-sync.service`）**：那套写法在 NTP 服务器不可达时会**无限阻塞** `time-sync.target`，等价于“车永远不启动”。本方案宁可带着错时钟起来（旧行为）也不失去可用性，只把“已知会跳变”的那段窗口关掉。

### 四、验证方式（实机验证**待办**）

```bash
# ① 脚本自检（不改系统状态）
bash src/hunter_bringup/scripts/hunter_clock_gate.sh --check
# ② 装单元并重启
colcon build --symlink-install --packages-select hunter_bringup && sudo bash src/hunter_bringup/scripts/hunter_core_setup.sh --skip-deps --skip-build && sudo systemctl daemon-reload && sudo systemctl restart hunter-edge
# ③ 验收：重启日志首行应见 [clock]，且以后不再出现 MessageFilter 整帧丢弃伴随巨大“未更新秒数”
sudo journalctl -u hunter-edge -b | grep -E '\[clock\]|not updated for [0-9]{3,}s' | head -20
```

### 五、向后兼容与风险

1. **启动时间**：NTP 已同步时闸门耗时 < 1s；未同步时最多 +60s（`TimeoutStartSec=240` 已覆盖）。
2. **可用性优先**：闸门超时、脚本缺失、无 `timedatectl` 一律 `exit 0`，不会出现“因时钟而对不上就不启动”。
3. **覆盖范围有限**：仅解决“上电时时钟还没对”这一个窗口。运行期间被外部拨钟（手工 `date -s`、NTP 大跳变）仍会复现同样症状——那是运维纪律问题，闸门不管。
4. **历史数据污染（已发生）**：10-26 17:13:55 ～ 08:47:16 这 76 秒内 `data_agent` 上报的遥测/健康/事件带有**错误时间戳**，平台按时间窗检索时会把 10-27 的真实事件当成 10-26 17:14 的数据。无需改代码，仅需知晓。

### 六、待办

1. **RTC 硬件（唯一根治项，仍待）**：`RTC time` 之所以“现在对”是因为内核在运行期写入了 RTC；断电即丢说明**板上 RTC 电池/超级电容失效或未装**，软件无法补救。软件侧的补偿（同步后 `hwclock --systohc` 缩短下次偏差窗口）**已由 V0.1.35 落地并默认开启**。
2. ~~NTP 源~~ **已由 V0.1.35 落地**：`hunter_core_setup.sh` 步骤 ⑥.0 写 `/etc/systemd/timesyncd.conf.d/10-hunter.conf`（国内四源 + `FallbackNTP`），把首同步从 ~76s 压到几秒。
3. 把 `NTPSynchronized` 纳入 `/system/health`（V0.1.33 §9 整改方向 ④）。

---

## V0.1.35（Date：2026-10-27）

**主题：时钟两件套收尾——NTP 源换国内（首同步 ~76s → 秒级）+ RTC 同步后回写与 `/dev/rtc0` 体检。改动只在部署脚本与时钟闸门，不动任何 ROS 节点**

### 一、实测数据（接 V0.1.34）

```
$ timedatectl
               Local time: Tue 2026-10-27 10:34:34 CST
           Universal time: Tue 2026-10-27 02:34:34 UTC
                 RTC time: Tue 2026-10-27 02:34:35
System clock synchronized: yes          NTP service: active          RTC in local TZ: no
$ sudo hwclock -r
2026-10-27 10:34:44.449661+08:00
```

- `RTC time` 与 `System time` 只差 1s ⇒ **RTC 读写本身正常**（内核运行期会周期性回写），问题只在“断电期间不保持”。
  > 更正（V0.1.36 §二）：方向对，但**严重度被低估**。`dmesg` 显示每次上电内核从 RTC 只读到 `1970-01-01T00:00:36`——不是“停在旧值”，而是**归零**（后备电源根本没供电）。这里“只差 1s”是内核运行期回写造成的假象。
- 首同步服务器是 `193.182.111.143`（`1.pool.ntp.org` 的欧洲节点），开机后 **~76s** 才同步成功，而整栈在开机 ~19s 就已拉起 ⇒ 这个 76s 的空窗就是事故温床。

### 二、两件事与做法

| 问题 | 做法 | 为什么这么做 |
|------|------|--------------|
| NTP 源太远 | `/etc/systemd/timesyncd.conf.d/10-hunter.conf`：`NTP=ntp.aliyun.com ntp1.aliyun.com ntp.tencent.com cn.pool.ntp.org`、`FallbackNTP=ntp.ntsc.ac.cn time.cloudflare.com` | 用 **drop-in** 而不改发行版的 `timesyncd.conf`：apt 升级不会覆盖，回退只需删一个文件。国内源把首同步压到秒级，闸门等待就几乎不占启动时间 |
| RTC 断电丢时 | ① `HUNTER_CLOCK_GATE_SYSTOHC=1`（同步后 `hwclock --systohc`）；② 闸门体内体检 `hwclock -r`，读不到直接报硬件问题 | 软件无法修复“断电丢时”（需补 RTC 电池），但可以**每次同步后立即把 RTC 写成对的**——若某次是异常断电而非电池失效，下次开机就少丢一段 |

### 三、改动

| 文件 | 改动 |
|------|------|
| `src/hunter_bringup/scripts/hunter_core_setup.sh` | 新增 `--skip-ntp` 开关；新增**步骤 ⑥.0 时钟同步**（写 drop-in → `timedatectl set-ntp true` → `systemctl enable --now systemd-timesyncd` → 同步后 `hwclock --systohc`）；生成的 `agent_env.sh` 增加 `HUNTER_CLOCK_GATE_TIMEOUT=60` / `HUNTER_CLOCK_GATE_SYSTOHC=1`；`README_agent_env` 增加“时钟异常”排查指路 |
| `src/hunter_bringup/scripts/hunter_clock_gate.sh` | 同步成功后增加 RTC 体检：`hwclock -r` 失败即告警“属硬件问题（RTC 电池/超级电容/未装）”，与 `--systohc` 回写分开记日志 |

### 四、验证（实机验证**待办**）

```bash
sudo bash src/hunter_bringup/scripts/hunter_core_setup.sh --skip-deps --skip-build
cat /etc/systemd/timesyncd.conf.d/10-hunter.conf          # 国内源已落盘
systemctl restart systemd-timesyncd && sleep 3
timedatectl timesync-status                              # Server 应为 *.aliyun.com / *.tencent.com，offset 毫秒级
timedatectl                                                # RTC time 与 System time 应一致
journalctl -u systemd-timesyncd -b | grep -i 'Initial synchronization'
```

**验收判据**：`timesync-status` 的 `Server` 不再是 `1.pool.ntp.org`；冷启动时 `journalctl -u hunter-edge` 的 `[clock]` 行显示等待 **≤ 5s** 即通过（原为 76s）。

### 五、向后兼容与风险

1. **可整体退出**：`--skip-ntp` 跳过 ⑥.0（现场已自建 NTP 时用）；drop-in 只是一份新增文件，删掉即回到发行版默认。
2. **不阻断部署**：NTP 不可达 / `timedatectl` 不可用 / 无 `hwclock` 全部只告警，脚本继续跑完 ⑥。
3. **NTP 源可用性**：四主源 + 两备源已冗余；若某车所在网络对 UDP/123 封禁，仍会回落到“闸门超时 60s 后照常启动”（V0.1.34 既有行为）。
4. **RTC 回写不等于修好硬件**：`hwclock --systohc` 只在“RTC 芯片还能跑、只是没对上”时有意义；实测本机是**断电丢时**（开机时 RTC 被内核判为 unset），仍需硬件补电池——本版只是把软件侧窗口收到最小。

### 六、待办

1. **RTC 电池（硬件）**：唯一根治项。Jetson AGX Orin 载板需确认 RTC 电池/超级电容是否装配、是否失效（`dmesg \| grep -i rtc`、`ls /dev/rtc*`）。
   > V0.1.36 已定案：`dmesg` 铁证为“上电读数归零（1970-01-01T00:00:36）”，即未装或已耗尽；软件侧只能兜底。
2. **舰队排查**：其余车辆是否同一批次同一问题 —— 逐台跑 `timedatectl`，看 `RTC time` 与 `System time` 在**冷启动后 5 分钟内**是否相差 > 60s。
3. 把 `NTPSynchronized` 与 `RTC` 健康纳入 `/system/health`（V0.1.33 §9 ④），让平台侧能看见“这台车的时钟可信度”。

---

## V0.1.36（Date：2026-10-27）

**主题：RTC 定案——`dmesg` 铁证“每次上电内核从 RTC 只读到 `1970-01-01T00:00:36`”⇒ 后备电源根本没供电（不是“停在旧值”）；同时修掉 V0.1.35 的一个死角：RTC 体检被关在“等待同步”分支里，已同步时一次都不会跑**

### 一、实测（V0.1.35 验收通过项）

```
$ bash src/hunter_bringup/scripts/hunter_clock_gate.sh --check
[clock] 时钟已同步（NTPSynchronized=yes，2026-10-27 10:44:21）：无需等待

$ sudo bash src/hunter_bringup/scripts/hunter_core_setup.sh --skip-deps --skip-build
[hunter-core] ⑥.0 时钟同步（NTP 源 + timesyncd）
[hunter-core]   写入 /etc/systemd/timesyncd.conf.d/10-hunter.conf（国内 NTP 源 + FallbackNTP）
[hunter-core]   时钟已同步（2026-10-27 10:44:45）
[hunter-core]   已回写 RTC（hwclock --systohc）

$ timedatectl timesync-status
       Server: 203.107.6.88 (ntp.aliyun.com)
     Stratum: 2      Offset: -6.296ms    Delay: 27.961ms    Jitter: 2.379ms
Root distance: 15.952ms (max: 5s)

$ timedatectl
      Local time: Tue 2026-10-27 10:45:58 CST
           RTC time: Tue 2026-10-27 02:45:58
System clock synchronized: yes     NTP service: active     RTC in local TZ: no
```

- NTP 源已从 `1.pool.ntp.org`（欧洲，首同步 ~76s）切到 `ntp.aliyun.com`（**毫秒级 offset**）✅
- `hwclock --systohc` 回写成功，`timedatectl` 的 `RTC time` 与 `Universal time` 一致 ✅
- 闸门在 systemd 路径下正常执行，未报错 ✅

### 二、RTC 定案：铁证与 V0.1.35 结论修正

```
$ sudo dmesg | grep -i rtc
[    5.170799] tegra_rtc c2a0000.rtc: registered as rtc1
[    5.170809] tegra_rtc c2a0000.rtc: Tegra internal Real Time Clock
[   12.532302] nvvrs-pseq-rtc nvvrs-pseq-rtc: registered as rtc0
[   12.534783] nvvrs-pseq-rtc nvvrs-pseq-rtc: setting system clock to 1970-01-01T00:00:36 UTC (36)

$ ls -l /dev/rtc*
lrwxrwxrwx 1 root root  4 Jan  1  1970 /dev/rtc -> rtc0
crw------- 1 root root 251, 0 Jan  1  1970 /dev/rtc0
crw------- 1 root root 251, 1 Oct 12 15:44 /dev/rtc1

$ sudo hwclock -r
2026-10-27 10:46:41.341509+08:00
```

**关键那一行是 `setting system clock to 1970-01-01T00:00:36 UTC (36)`**：

1. `rtc0 = nvvrs-pseq-rtc`（NVVRS 电源序列 RTC，`/dev/rtc -> rtc0`，`hwclock` 读的就是它），`rtc1 = tegra_rtc`（Tegra 片内 RTC）。
2. 上电时内核从 rtc0 只读出 **epoch 36 秒**——计数器从 0 起（36s 是 NVVRS 上电到内核读取的时长）⇒ **断电期间 RTC 没有任何后备电源供电**。不是“停在关机那一刻的旧值”。
3. **与 10-27 事故完全同源**：事故日志里的 `Jan 01 08:00:36` 是 CST(+8) 的 `00:00:36 UTC`，与本次 `(36)` 是**同一个数**。两次上电都是“从 0 起”，说明不是偶发，而是长期如此。
4. `hwclock -r` 现在读得对（`2026-10-27 10:46:41`），是因为 10:44:45 刚被 `hwclock --systohc` 写过——**这正是 V0.1.35 判读被误导的原因**：运行期一切正常，故障只在断电瞬间发生。

> 由此对 V0.1.35 §一的判断作修正：当时写“RTC 读写本身正常，问题只在断电期间不保持”——方向对，但**严重度被低估**。真实情况是“归零”，不是“停旧值”。这意味着：只要不换电池，**每一台车每一次断电开机都必定丢时**，与“上次关机时间”无关。

### 三、V0.1.35 留下的自伤死角

```
$ sudo journalctl -u hunter-edge -b | grep '\[clock\]'
Oct 27 10:44:56 ... [clock] 时钟已同步（NTPSynchronized=yes，2026-10-27 10:44:56）：无需等待   ← 两条
Oct 27 10:44:58 ... [clock] 时钟已同步（NTPSynchronized=yes，2026-10-27 10:44:58）：无需等待   ← 四条
Oct 27 10:46:09 ... [clock] 时钟已同步（NTPSynchronized=yes，2026-10-27 10:46:09）：无需等待   ← 两条
```

两个问题暴露在这 8 行里：

1. **RTC 体检一次都没跑过**。V0.1.35 把 `hwclock -r` 体检写在“等待同步 → 成功”分支里，而上面 8 条全是 `NTPSynchronized=yes` 的 **early-return** 路径。而 RTC 坏不坏跟“现在时间对不对”根本无关——恰恰是已同步时最需要查它（否则故障只能等下次断电才暴露）。本版把体检**上提到 early-return 之前**无条件执行。
2. **每条启动出两条一模一样的日志**。`hunter-edge.service` 结构是 `ExecStartPre`（闸门）→ `ExecStart=hunter_edge_up.sh`，而后者 ⓪ 步骤也调一次闸门 ⇒ 每次启动 2 条（实测 4 次启动 = 8 条）。幂等所以功能无害，但现场会误认为服务被重复拉起。本版按 `INVOCATION_ID` 去重。

### 四、改动

| 文件 | 改动 |
|------|------|
| `src/hunter_bringup/scripts/hunter_clock_gate.sh` | 新增 `rtc_hw_check()` 并**提到 early-return 之前**无条件执行：判据① 取 `journalctl -k -b` 里 `setting system clock to …` 那行，年份 < 2000 就告警“RTC 后备电源失效”；判据② `hwclock -r` 失败告警“器件不可读”。删掉原等待分支里的重复体检 |
| `src/hunter_bringup/scripts/hunter_core_setup.sh` | ⑥.0 在 `hwclock --systohc` 之后跑同一判据：**部署当时**就把“电池已死”报出来，不用等下次断电再挖日志 |
| `src/hunter_bringup/scripts/hunter_edge_up.sh` | ⓪ 步骤去重：`INVOCATION_ID` 非空（systemd 拉起，ExecStartPre 已跑）则跳过；手工 `bash hunter_edge_up.sh` 照常执行；`HUNTER_CLOCK_GATE_FORCE=1` 强制 |

**为什么判据必须取内核那一行**：`systemd-timesyncd` 会用 `/var/lib/systemd/timesync/clock` 把系统时钟回填成上次的值（10-27 事故里就是回填成 `10-26 17:13:55`），回填后 `timedatectl`、`date`、`hwclock -r` **全部看着正常**——只有内核那行 `setting system clock to 1970-…` 还留着真相。这也是本版不能用“`now - uptime`”推算启动时刻的原因：时钟跳变后该式同样失真。

### 五、验证

1. `bash -n` 三个脚本（`hunter_clock_gate.sh` / `hunter_edge_up.sh` / `hunter_core_setup.sh`）全部通过。
2. 判据逻辑用真实 dmesg 行做 5 个用例实测：

| 输入 | 期望 | 实测 |
|------|------|------|
| `nvvrs-pseq-rtc ... setting system clock to 1970-01-01T00:00:36 UTC (36)`（无时间戳前缀） | WARN | ✅ WARN |
| 同上，带 `Oct 27 10:44:56 agilex kernel:` 前缀 | WARN | ✅ WARN |
| `... setting system clock to 2026-10-27T02:45:00 UTC` | 静默 | ✅ 静默 |
| 无该行（只有 `registered as rtc1`） | 静默（不能误报） | ✅ 静默 |
| `... 1980-03-01T00:00:00 UTC` | WARN | ✅ WARN |

3. **实机验收（2026-10-27 10:51，V0.1.36 代码下发后）**：

```
$ sudo bash src/hunter_bringup/scripts/hunter_clock_gate.sh --check
[clock] ! 本次上电内核从 RTC 只读到 1970-01-01T00:00:36 ⇒ RTC 后备电源失效（断电期间计数器从 0 重来）
[clock] !   ⇒ 属硬件问题：RTC 电池/超级电容未装、装反或电压不足（唯一根治项，软件补不了）
[clock] !   ⇒ 软件侧只能靠本闸门等 NTP 首次校正；校正前整栈发出的时间戳都是错的
[clock] 时钟已同步（NTPSynchronized=yes，2026-10-27 10:51:54）：无需等待
```

- 新判据在本机**真实命中** ✅ —— 它把“要人工翻 `dmesg` 才能定案 RTC 硬件”变成开机日志里一句话。
- `hunter_core_setup.sh` ⑥.0 同样在 `已回写 RTC` 之后命中同一告警（部署当时就暴露）✅
- **日志去重生效** ✅：三次启动事务（`10:52:17` / `10:52:19` / `10:52:30`）**每次只有 1 个 bash PID**（`10423` / `10535` / `11068`）；改前同一事务会产生两个 PID（如 `10:44:56` 的 `8586`+`8614`）。
- 告警在每次启动都会重复打印，属设计如此（RTC 是持续失效状态，声量比静默安全）。

4. **冷启动实测：待办**（见 §六）——以上均为已同步状态下热重启，走的是 early-return 分支，闸门的“等待 + 跳变量告警”路径尚未在真实断电场景跑过。

### 六、风险与待办

**风险**

1. `journalctl -k -b` 需要 root（`ExecStartPre` 是 root，满足）；手工 `--check` 不加 `sudo` 时读不到内核日志，此时判据**静默跳过而不是误报**。
2. `INVOCATION_ID` 去重的边界：若有人手工把 `hunter_edge_up.sh` 塞进别的 systemd 单元运行，会跳过闸门——此时用 `HUNTER_CLOCK_GATE_FORCE=1` 或改回直接调 `hunter_clock_gate.sh`。
3. 闸门上限仍是 60s。若某车网络起得比 60s 更晚，仍会在超时后带着错误时钟启动（设计如此：不阻断启动）。

**待办**

1. **RTC 电池（硬件）——唯一根治项**。已定案为“未装或已耗尽”：需开盖核 AGX Orin 载板 RTC 电池座（CR1220）/超级电容。换前每次断电开机必丢时。
2. **冷启动验收**：断电 → 上电，看 `journalctl -u hunter-edge -b` 的 `[clock]`，期望出现 **RTC 告警 3 行** + `等待 Ns 后同步`，且期间不出现 `obstacle_cloud ... not updated for 5xxxx s`。
3. **舰队排查**：逐台 `sudo dmesg \| grep 'setting system clock'`，命中 1970 即同批硬件问题。
4. 把 `NTPSynchronized` 与“上电 RTC 读数”纳入 `/system/health`（V0.1.33 §9 ④），让平台侧能看见“这台车的时钟可信度”。

---

## V0.1.37（Date：2026-10-27）

**主题：冷启动实战验收通过——上电闸门在整栈启动前吸收了 207s 时钟补偿，`not updated for` 零命中；同时补上三处防御性加固（等待上限改用单调时钟 / `timedatectl` 单次超时 / `TimeoutStartSec` 240→360）**

### 一、冷启动实测（决定性证据）

真实断电重启，全量 `[clock]` 与 timesyncd 日志：

```
Jan 01 08:00:33  nvvrs-pseq-rtc: setting system clock to 1970-01-01T00:00:33 UTC (33)
Jan 01 08:00:34  systemd-timesyncd[656]: System clock time unset or jumped backwards,
                 restoring from recorded timestamp: Tue 2026-10-27 10:55:20 CST

10:55:27  [clock] ! 本次上电内核从 RTC 只读到 1970-01-01T00:00:33 ⇒ RTC 后备电源失效
10:55:27  [clock]   时钟未同步（NTPSynchronized=no，当前 2026-10-27 10:55:27）
10:55:27  [clock]   等待 NTP 首同步，上限 60s（超时只告警、不阻断启动）
10:59:17  [clock] ! 时钟已同步：2026-10-27 10:55:27 → 2026-10-27 10:59:17，等待 23s 期间被修正 207s
10:59:17  [clock] !   ⇒ 本机 RTC 掉电不保持时间…本次已避免“整栈带着错误时钟启动”
10:59:18  [clock]   已回写 RTC（hwclock --systohc）
10:59:16  systemd-timesyncd[656]: Initial synchronization to time server 106.55.184.199:123 (ntp.tencent.com)

$ sudo journalctl -u hunter-edge -b | grep -E '\[clock\]|not updated for [0-9]{3,}s'
（无任何 not updated for 命中）
```

**结果：事故未重演。** 闸门在整栈启动前把时钟拉正，确保全栈起来时基准已正确。

### 二、“230s”是假象：三个数字必须一起读

| 量 | 值 | 含义 |
|----|-----|------|
| 墙钟差 | `10:55:27 → 10:59:17` = **230s** | 含被拨入的量 |
| 真实流逝 | `elapsed` = **23s** | 闸门实际只等了这么久 |
| 跳变量 | `delta` = **207s** | `230 − 23`，即时钟被往前拨的净量 |

三者自洽，所以日志里“等了 23s”和“时间戳差 4 分钟”**不矛盾**——**首同步只用 23s**（境外源时代实测是 76s），被拨的 207s 是因为断电只有 3 分钟（回填值 10:55:20 vs 真实 10:58:54）。

> 这一点很重要：**闸门的有效性不取决于跳变量大小**。本次 207s、事故那次 55970s，只要首同步落在等待窗口内，就都是在栈启动前被吸收。断电隔夜也不会因为“跳得多”而突破。

### 三、三处防御性加固（本次未触发，但都是真实风险）

| 隐患 | 后果 | 加固 |
|------|------|------|
| `elapsed` 用循环次数计超时 | `read_sync` 偶发变慢（走 D-Bus）时低估真实耗时，实际等待超过 `TIMEOUT`，而 `TimeoutStartSec` 是按真实时间算的 → 被 systemd 杀服务并 `Restart`，现场看着像“启动失败” | 改 `/proc/uptime`（CLOCK_BOOTTIME，免疫时钟跳变）计量；`delta` 也随之变成“真跳变量” |
| `read_sync` 无单次上限 | 同上，一次 `timedatectl` 卡住就把整条链拖走 | 包 `timeout 5`，超时视为 `unknown` 继续等（只会多等一轮，不误判已同步） |
| `TimeoutStartSec=240` | 最坏情况“闸门跑满 60s + 整栈 ~180s”恰好卡在线上 | 240 → **360**，给足余量；副作用只是真启动失败时要多等 2 分钟才判失败 |

### 四、验证

1. `bash -n` 三个脚本全部通过。
2. 计时逻辑 4 个用例实测（不依赖 `timedatectl`）：

| 用例 | 期望 | 实测 |
|------|------|------|
| `mono()` 返回值 | 纯数字 | ✅ `mono=6` |
| `TIMEOUT=3` + 恒未同步 | 到时退出，`elapsed`≈3，无跳变时 `delta`≈0 | ✅ `elapsed=3s delta=0s` |
| 已同步 | 立即返回，不等满 | ✅ `0s` |
| 无 `/proc/uptime` | 回落 `date +%s`，仍为数字 | ✅ `1793070109` |

### 五、风险与待办

**风险**

1. **网络差的现场仍可能超过 60s**：本次 23s，若某车首同步 > 60s，闸门会超时放行，之后一旦同步成功就会重演。建议随车实测几次首同步时长，必要时把 `/etc/hunter/agent_env.sh` 的 `HUNTER_CLOCK_GATE_TIMEOUT` 调大。
2. **`TimeoutStartSec=360` 的代价**：真启动失败（如相机卡死）时要等 6 分钟才判失败，比原来的 4 分钟慢。
3. 闸门只保障“启动那一刻时钟是对的”；运行期间若有人手动改时间，仍会断轴，需 `restart hunter-edge`。

**待办**

1. **RTC 电池（硬件）仍为唯一根治项**。本次又确认一次：断电后 rtc0 从 epoch 33 起（三次上电分别是 36 / 36 / 33），即计数器从 0 开始。换电池前每次断电开机都靠闸门兜底。
2. 舰队抽查首同步时长：`journalctl -u hunter-edge -b | grep '\[clock\] 等待\|被修正'`，若 N 普遍 > 30s，考虑在 drop-in 里直接写 NTP 服务器 IP（避开 DNS 重试）或改用 chrony+iburst。
3. 把 `NTPSynchronized` 与“上电 RTC 读数”纳入 `/system/health`（V0.1.33 §9 ④）。
4. `fast_lio` 等节点仍无 respawn（V0.1.33 待办，与本版无关但仍在台账上）。




