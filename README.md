# HunterEdge 自动驾驶车载系统 — 开发指南

> **项目**：HunterEdge 自动驾驶车载系统
> **文档版本**：V1.1（开发指南）
> **编制依据**：《自动驾驶车辆系统详细设计文档 V2.0》（下称"设计文档"）
> **面向对象**：开发人员 / 测试与现场运维人员

---

## 目录

- [1. 项目简介](#1-项目简介)
- [2. 硬件清单](#2-硬件清单)
- [3. 软件环境与版本](#3-软件环境与版本)
- [4. 项目目录结构](#4-项目目录结构)
- [5. 依赖安装](#5-依赖安装)
- [6. 编译步骤](#6-编译步骤)
- [7. 快速启动](#7-快速启动)
- [8. ROS2 关键话题总览](#8-ros2-关键话题总览)
- [9. 系统控制模式](#9-系统控制模式)
- [10. 自主导航模块](#10-自主导航模块)
- [11. 开发指引](#11-开发指引)
- [12. 运维脚本说明](#12-运维脚本说明)
- [13. 已知限制与注意事项](#13-已知限制与注意事项)
- [14. 文档索引](#14-文档索引)

---

## 1. 项目简介

**HunterEdge** 是 HUNTER 自动驾驶平台的**车辆端（边缘智能端）**软件工程。系统运行于基于 **HUNTER SE 阿克曼 UGV 底盘**与 **EDU Pro Kit 传感器套件**构建的自动驾驶车辆上，核心计算平台为 **NVIDIA Jetson AGX Xavier 32GB**（详见设计文档 §1.5、§3.2）。

根据设计文档 §1.2，系统定位为 HUNTER 平台的**边缘智能端**，承担以下核心职能：

1. **环境感知**：基于激光雷达、深度相机、IMU 等传感器感知周围环境；
2. **多传感器融合**：融合多源传感器数据，输出统一环境表示；
3. **高精度定位**：融合 IMU、轮速计、LiDAR 里程计，输出车辆位姿；
4. **行为决策**：根据环境和任务决定驾驶行为；
5. **轨迹规划**：生成安全、平滑、可执行的运动轨迹；
6. **运动控制**：将轨迹转化为 CAN 控制指令，驱动 HUNTER SE 底盘；
7. **OTA 升级**：接收平台软件升级并安全完成更新；
8. **远程操控**：支持远程视频回传和人工接管；
9. **数据上传**：将实时遥测、事件和传感器数据上传至平台。

**软件架构**：基于 ROS2 Humble 的分布式节点架构（设计文档 §2.2），由应用层（感知/融合/定位/规划/控制/Agent/监控）、中间件层（rclcpp/rclpy/tf2/pluginlib）、驱动层（LiDAR/Camera/IMU/CAN 驱动）与系统层（Ubuntu+JetPack）构成。

> 💡 **【开发者视角】** 本仓库是车辆端全部算法与集成代码的载体；构建、启动、联调均在此工作空间完成。

---

## 2. 硬件清单

### 2.1 底盘 — HUNTER SE（设计文档 §3.1）

| 参数 | 规格 |
|------|------|
| 车型 | 阿克曼转向 UGV |
| 外形尺寸 | 820 × 640 × 310 mm |
| 轴距 | 460 mm |
| 轮距 | 550 mm |
| 整备质量 | 42 kg |
| 最大负载 | 50 kg |
| 驱动方式 | 前轮双电机驱动 |
| 驱动电机 | 2 × 350W 直流无刷电机 |
| 转向电机 | 1 × 150W 直流无刷电机 |
| 最高行驶速度 | 4.8 m/s（17.28 km/h） |
| 最大爬坡角度 | 20° |
| 最小转弯半径 | 1.9 m |
| 电池规格 | 24V / 30Ah 锂电池 |
| 通信接口 | CAN 2.0B（500Kbps），DB9 接口 |
| 工作温度 | -10℃ ~ 50℃ |
| 防护等级 | IPX4（底盘主体） |

### 2.2 计算平台 — NVIDIA Jetson AGX Xavier 32GB（设计文档 §3.2）

| 参数 | 规格 |
|------|------|
| CPU | 8核 NVIDIA Carmel ARM v8.2 @ 2.26GHz |
| GPU | 512-core Volta GPU + 64 Tensor Cores |
| AI 算力 | 32 TOPS（INT8） |
| 内存 | 32GB 256-bit LPDDR4x |
| 存储 | 32GB eMMC 5.1（可扩展 NVMe SSD） |
| 接口 | 千兆以太网、USB 3.1×4、HDMI/DP、CSI、UART、CAN×2、PCIe Gen4 x8 |
| 功耗模式 | 10W / 15W / 30W / 50W（默认 MODE_15W，§3.5；附录 B 推荐 MODE_30W） |
| 操作系统 | Ubuntu 22.04.5 LTS + JetPack 6.1.2 |

### 2.3 传感器（EDU Pro Kit，设计文档 §3.3）

| 传感器 | 型号 | 关键规格 |
|--------|------|----------|
| 激光雷达 | RoboSense RS-Helios-16P | 16线，0.2~150m，360°水平，-15°~+15°垂直，10Hz(600rpm)，±2cm |
| 深度相机 | Intel RealSense D435 | 深度 1280×720@90fps，RGB 1920×1080@30fps，0.1~10m |
| IMU | CH10X | ±16g / ±2000°/s，100Hz，含 EKF 融合，UART 输出 |
| 路由器 | GL.iNet GL-AR750S | 双频 WiFi + 4G Dongle + 千兆网，车-云通信 |

> ⚠️ **【现场运维视角】** 传感器安装位置与精确外参以设计文档 §20.2 标定流程与附录 D 为准；未标定的外参可能导致感知/定位偏差。

---

## 3. 软件环境与版本

完整版本清单（设计文档 §4.1）：

| 组件 | 版本 | 说明 |
|------|------|------|
| 操作系统 | Ubuntu 22.04.5 LTS（Jammy Jellyfish） | JetPack 6.1.2 自带 |
| Linux 内核 | 5.15.136-tegra | NVIDIA L4T R36.4.4 |
| CUDA | 12.6.68 | GPU 计算 |
| cuDNN | 9.3.0 | 深度学习加速 |
| TensorRT | 10.3.0 | 模型推理优化 |
| OpenCV | 4.5.4（with CUDA） | 图像处理 |
| ROS2 | Humble Hawksbill | 机器人中间件 |
| Python | 3.10.12 | 系统默认 |
| CMake | 3.22.1 | 构建工具 |
| GCC | 11.4.0 | C++ 编译器 |

> 💡 **【开发者视角】** 视觉感知依赖 TensorRT + OpenCV CUDA，仅 NVIDIA Jetson 环境可用；CPU 部分算法可跨平台编译，但完整系统需在车载 Jetson 上运行。

---

## 4. 项目目录结构

基于设计文档 §4.2 的工作空间 `~/HunterEdge/src` 结构如下：

### 4.1 自定义功能包

| 包 | 用途 | 设计文档章节 |
|----|------|--------------|
| `hunter_bringup` | 全系统/模块化启动 launch、参数 config、行为树、URDF | §4.2 / §4.4 |
| `hunter_drivers/ch10x_driver` | CH10X IMU 驱动 | §3.3.3 |
| `hunter_perception/lidar_perception` | 激光感知（地面分割+欧式聚类+跟踪） | §5.1 |
| `hunter_perception/vision_perception` | 视觉感知（YOLOv8+TensorRT） | §5.2 |
| `hunter_agents/data_agent` | 数据采集上传（Kafka+MinIO） | §14 |
| `hunter_agents/ota_agent` | OTA 升级（systemd 服务） | §12 |
| `hunter_agents/remote_agent` | 远程操控（WebRTC，systemd 服务） | §13 |
| `hunter_common/hunter_msgs` | 自定义消息（DetectedObject/ChassisState/Trajectory/HunterStatus 等，含 AgileX 底盘消息） | §4.3 |
| `hunter_common/hunter_utils` | 公共工具函数库 | §4.2 |
| `hunter_perception/sensor_fusion` | 多传感器目标级数据融合（匈牙利关联 + 加权融合 + 可行驶区域） | §6 |
| `hunter_monitor/health_monitor` | 系统监控与健康管理 | §15 |
| `decision_making` | 模式仲裁决策（AUTO/REMOTE/ESTOP 状态机） | §13.5 |
| `auto_mission` | AUTO 模式自主任务调度（航点巡航/安全守护/建图控制） | 自主导航扩展 |

### 4.2 第三方依赖包（外部依赖，需按 §5 安装）

> 以下为设计文档 §4.2 定义的 vendor 外部依赖，**当前 `src/` 源码目录中不包含**，需按 §5 安装后随工作空间一起编译。

| 包 | 用途 |
|----|------|
| `ugv_sdk` | AgileX 官方 UGV SDK（底盘通信） |
| `hunter_ros2` | AgileX 官方 HUNTER ROS2 驱动 |
| `rslidar_sdk` | RoboSense LiDAR 驱动 |
| `realsense-ros` | Intel D435 相机驱动（ROS2） |
| `fast_lio2` | FAST-LIO2 雷达惯导里程计 |
| `robot_localization` | EKF 传感器融合 |
| `navigation2` | Nav2 导航栈（SmacPlanner、RPP、MPPI） |
| `yolo_trt_ros` | YOLO TensorRT 推理 ROS2 封装（可选） |

---

## 5. 依赖安装

在 Ubuntu 车载环境执行 `bash ./src/hunter_bringup/scripts/import_vendor.sh`，自动 git clone 全部第三方包。脚本会自动：

- **Nav2 跳过**：检测环境中是否已 apt 安装 `ros-humble-navigation2`，若已装则跳过 `navigation2` 源码编译（避免与二进制包同名冲突）；
- **yolo_trt_ros 去重**：仅保留 `src/jetson` 子包，其余同名 ROS 包自动禁用（重命名 `package.xml`）。

### 5.1 AgileX 底盘驱动（设计文档 §11.6）

```bash
cd ~/HunterEdge/src

# 安装 ugv_sdk
git clone https://github.com/agilexrobotics/ugv_sdk.git
cd ugv_sdk && mkdir build && cd build && cmake .. && make -j4

# 安装 hunter_ros2（humble 分支）
cd ~/HunterEdge/src
git clone -b humble https://github.com/agilexrobotics/hunter_ros2.git
```

### 5.2 其他第三方包

以下包可用 `apt` 安装二进制版本（更快更稳）：

```bash
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install -y ros-humble-robot-localization
sudo apt install -y ros-humble-realsense2-camera
sudo apt install -y librdkafka-dev libsqlite3-dev   # data_agent（Kafka + SQLite 缓存）
```

以下包需源码编译（vendor 源码）：

```bash
cd ~/HunterEdge/src
git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git                 # LiDAR 驱动
git clone https://github.com/wyf-yfw/TensorRT_YOLO_ROS2.git yolo_trt_ros     # YOLO TensorRT（可选，仅保留 src/jetson）
```

> ⚠️ **【运维视角】** 上述第三方仓库地址/分支以各项目官方文档为准；`ugv_sdk`、`hunter_ros2` 安装见 §5.1（源自设计文档 §11.6）。`navigation2`、`robot_localization`、`realsense2_camera` 建议用上文 `apt` 安装。

---

## 6. 编译步骤

```bash
cd ~/HunterEdge
colcon build --symlink-install
source install/setup.bash
```

**编译指定包**（可加速联调）：

```bash
# 仅编译自定义功能包
colcon build --packages-select hunter_msgs hunter_bringup lidar_perception
```

> 💡 **【开发者视角】** `--symlink-install` 使 Python 脚本与 launch 文件在源码修改后无需重新编译；若修改了 `.msg` 或 C++ 头文件，则需重新 `colcon build` 该包。

---

## 7. 快速启动

先加载环境（每次新终端都需要）：

```bash
source ~/HunterEdge/install/setup.bash
```

### 7.1 全系统启动（设计文档 §4.4）

```bash
ros2 launch hunter_bringup hunter_full.launch.py
```

启动顺序（按设计文档 §4.4）：传感器驱动 → CAN 驱动 → 定位 → 感知 → 融合 → 决策 → 规划 → 控制 → Agent → 监控。

**可选参数开关**：

| 参数 | 默认 | 说明 |
|------|------|------|
| `use_perception` | `true` | 是否启动感知模块 |
| `use_navigation` | `true` | 是否启动 Nav2（`use_autonomous_nav=true` 时自动禁用） |
| `use_data_agent` | `true` | 是否启动数据采集 Agent |
| `use_autonomous_nav` | `false` | 是否启动自主导航全栈（建图/巡航） |
| `autonomous_nav_mode` | `nav` | `nav`=导航巡航模式，`mapping`=建图模式 |
| `map_yaml_path` | `/home/agilex/HunterEdge/maps/hunter_map.yaml` | 导航模式地图 YAML 路径 |
| `map_file_path` | `/home/agilex/HunterEdge/maps/hunter_map.pcd` | 建图模式 PCD 保存路径 |

### 7.2 仅启动感知

```bash
ros2 launch hunter_bringup perception.launch.py
```

### 7.3 仅启动定位 + 导航

```bash
ros2 launch hunter_bringup localization.launch.py
ros2 launch hunter_bringup navigation.launch.py
```

### 7.4 启动数据采集 Agent

```bash
ros2 launch hunter_bringup data_agent.launch.py
```

### 7.5 自主导航模式启动

详见 [§10 自主导航模块](#10-自主导航模块)，快速命令：

```bash
# 全系统 + 在线建图（边建图边记录航点）
ros2 launch hunter_bringup hunter_full.launch.py \
  use_autonomous_nav:=true \
  autonomous_nav_mode:=mapping \
  map_file_path:=/home/agilex/HunterEdge/maps/hunter_map.pcd

# 全系统 + 自主航点巡航（加载已有地图）
ros2 launch hunter_bringup hunter_full.launch.py \
  use_autonomous_nav:=true \
  autonomous_nav_mode:=nav \
  map_yaml_path:=/home/agilex/HunterEdge/maps/hunter_map.yaml
```

> 💡 **【开发者视角】** 模块化启动便于逐模块联调；`hunter_full.launch.py` 的参数开关见上表（设计文档 §4.4）。

---

## 8. ROS2 关键话题总览

（依据设计文档 §17.1 内部 ROS 接口汇总）

| 话题名 | 消息类型 | 频率 | 用途 |
|--------|----------|------|------|
| `/lidar_points` | `sensor_msgs/PointCloud2` | 10Hz | 激光雷达点云（rslidar_sdk 发布） |
| `/camera/color/image_raw` | `sensor_msgs/Image` | 30Hz | D435 RGB 图像 |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image` | 30Hz | D435 深度图 |
| `/imu/data` | `sensor_msgs/Imu` | 100Hz | IMU 数据（CH10X） |
| `/chassis/state` | `hunter_msgs/ChassisState` | 10Hz | 底盘状态（hunter_ros2 发布） |
| `/chassis/feedback` | `hunter_msgs/ChassisState` | 50Hz | 底盘运动反馈 |
| `/perception/lidar_objects` | `hunter_msgs/DetectedObjectArray` | 10Hz | 激光检测结果 |
| `/perception/vision_objects` | `hunter_msgs/DetectedObjectArray` | 15Hz | 视觉检测结果 |
| `/perception/fused_objects` | `hunter_msgs/DetectedObjectArray` | 10Hz | 融合目标 |
| `/perception/freespace` | `nav_msgs/OccupancyGrid` | 10Hz | 可行驶区域 |
| `/localization/odom` | `nav_msgs/Odometry` | 50Hz | 融合定位结果 |
| `/planning/behavior_state` | `hunter_msgs/BehaviorState` | 10Hz | 决策行为状态 |
| `/planning/trajectory` | `hunter_msgs/Trajectory` | 10Hz | 规划轨迹 |
| `/control/command` | `hunter_msgs/ChassisCommand` | 50Hz | 最终控制指令 |
| `/remote/command` | `hunter_msgs/ChassisCommand` | 20Hz | 远程控制指令 |
| `/system/health` | `hunter_msgs/SystemHealth` | 1Hz | 系统健康状态 |
| `/auto_mission/status` | `std_msgs/String` | 10Hz | 自主任务状态（IDLE/MAPPING/NAVIGATING/OBSTACLE_AVOID/ESTOP） |
| `/auto_mission/current_waypoint` | `std_msgs/Int32` | 事件 | 当前执行的航点索引 |
| `/pcd_to_map/status` | `std_msgs/String` | 事件 | PCD→地图转换状态（IDLE/CONVERTING/DONE/ERROR） |
| `/navigate_to_pose` (action) | `nav2_msgs/NavigateToPose` | — | Nav2 单点导航 action 接口（方式B 外部下发） |

> 自定义消息定义见设计文档 §4.3（`hunter_msgs`）。
>
> ⚠️ **实际运行说明**：话题表为设计文档 §17.1 定义的设计接口。实际实现中：`/control/command` 由 `decision_making` 模式仲裁后发布（而非 §17.1 所述 nav2_controller）；`/planning/behavior_state` 由 `decision_making` 发布（实际约 50Hz）；若使用 Nav2，其全局路径输出为 `/plan`（`nav_msgs/Path`），自定义 `/planning/trajectory`（`hunter_msgs/Trajectory`）需由规划模块按需发布。

---

## 9. 系统控制模式

车辆支持三种控制模式，由**模式仲裁**节点管理（设计文档 §13.5），**切换优先级：`ESTOP > REMOTE > AUTO`**。

| 模式 | 控制源 | 说明 |
|------|--------|------|
| AUTO | 规划模块输出（Nav2） | 默认；远程未接管时生效 |
| REMOTE | Remote Agent 远程指令 | 平台发起远程操控，操作员接管 |
| ESTOP | 急停（v=0） | 急停按钮/碰撞风险/系统故障时触发 |

**关键行为**：

- **ESTOP 优先**：急停信号、底盘故障、CAN 通信丢失均立即切换到 ESTOP，输出零速度 + 紧急制动（设计文档 §16.2）。
- **REMOTE 接管**：远程指令覆盖自动驾驶输出（设计文档 §13.4），最高限速 2.0 m/s，指令超时 500ms 自动停车。
- **AUTO 默认**：无急停、无远程接管时，使用 Nav2 规划输出的控制指令。

---

## 10. 自主导航模块

`auto_mission` 包实现 AUTO 模式下的完整自主导航能力，包含**建图模式**与**定位导航模式**，以及三个自动化工具节点，消除建图、转换、航点采集三项手动操作。

### 10.1 模块架构

```
hunter_full.launch.py (use_autonomous_nav:=true)
└── hunter_autonomous_nav.launch.py
    ├── [mapping 模式]
    │   ├── fast_lio2_param_injector  ← 自动注入 pcd_save_en=true + 路径
    │   ├── auto_mission_node         ← 状态机（MAPPING 状态，不下发 goal）
    │   ├── pcd_to_map                ← 建图结束自动 PCD→PGM+YAML 转换
    │   └── waypoint_recorder         ← /clicked_point 自动写入航点 yaml
    └── [nav 模式]
        ├── fast_lio2_param_injector  ← 自动注入 pcd_save_en=false
        ├── map_server                ← 加载静态 PGM 地图
        ├── Nav2 全栈                 ← 使用 autonomous_navigate.xml 扩展行为树
        └── auto_mission_node         ← 状态机（航点巡航/安全守护）
```

### 10.2 AUTO 进入条件（全部满足才允许导航）

| 条件 | 检测方式 |
|------|----------|
| `decision_making` 输出 `mode == "AUTO"` | 订阅 `/planning/behavior_state` |
| 无急停信号 | 订阅 `/estop` |
| `SystemHealth` 非 `CRITICAL` | 订阅 `/system/health` |
| 定位协方差迹 ≤ 0.5（可配） | 订阅 `/localization/odom` 协方差对角元素 |
| 感知数据新鲜度 ≤ 2s（可配） | 订阅 `/perception/fused_objects` 时间戳 |

### 10.3 任务状态机

```
IDLE ──[AUTO条件满足]──→ WAITING_LOCALIZE ──[收敛]──→ NAVIGATING
IDLE ──[mapping模式]──→ MAPPING
NAVIGATING ──[障碍物 < warn_dist]──→ OBSTACLE_AVOID ──[路清]──→ NAVIGATING
NAVIGATING ──[障碍物 < stop_dist]──→ ESTOP
任意状态 ──[非AUTO/急停]──→ IDLE / ESTOP
```

### 10.4 安全约束参数（`autonomous_nav_params.yaml`）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `warn_obstacle_dist` | 2.0 m | 障碍物减速警告距离 |
| `stop_obstacle_dist` | 0.8 m | 障碍物急停距离 |
| `obstacle_fov_deg` | 120° | 前向检测扇区 |
| `localize_cov_threshold` | 0.5 | 定位协方差迹收敛阈值 |
| `localize_wait_timeout` | 10 s | 等待定位收敛超时 |
| `perception_timeout` | 2 s | 感知数据超时阈值 |
| `max_velocity` | 1.5 m/s | 巡航速度（≤ 2.0 m/s 合规限制） |
| `loop_waypoints` | `true` | 完成所有航点后是否循环 |
| `goal_timeout` | 60 s | 单点导航超时 |
| `obstacle_wait_timeout` | 30 s | 障碍物等待超时后触发 ESTOP |

### 10.5 自动化工具节点

| 节点 | 可执行文件 | 解决的手动操作 |
|------|-----------|---------------|
| `fast_lio2_param_injector` | 同名 | 自动向 `fast_lio2` 注入 `pcd_save_en` + `map_file_path` |
| `pcd_to_map` | 同名 | 建图结束自动将 PCD 三维点云转换为 Nav2 栅格地图（.pgm + .yaml） |
| `waypoint_recorder` | 同名 | 建图时 rviz2 点击即自动追加写入 `autonomous_nav_params.yaml` |

### 10.6 扩展行为树

`autonomous_navigate.xml` 在原 `navigate_to_pose.xml` 基础上新增：

- **定位门控**：`TransformAvailable(map→base_link)` 前置检查，定位不可用时立即阻断导航；
- **感知保鲜**：`TimeExpired(2s)` 哨兵，感知超时时清除局部代价地图并等待恢复；
- **动态减速**：`SpeedController(0.4~2.0 m/s)`，障碍物距离 < 3m 时按比例降速；
- **阿克曼后退**：`BackUp(0.3m)` 替代 `Spin`（原地旋转），适合阿克曼底盘脱困。

### 10.7 新增/修改文件清单

```
src/
├── auto_mission/                               ← 新增包
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── config/
│   │   └── autonomous_nav_params.yaml          ← 全部参数配置（含航点列表）
│   ├── include/auto_mission/
│   │   └── auto_mission_node.hpp
│   ├── src/
│   │   ├── auto_mission_node.cpp               ← 6态状态机 C++ 实现
│   │   └── main.cpp
│   └── scripts/
│       ├── waypoint_recorder.py                ← 航点自动采集
│       ├── pcd_to_map.py                       ← PCD→PGM 自动转换
│       └── fast_lio2_param_injector.py         ← pcd_save 参数自动注入
└── hunter_bringup/
    ├── behavior_trees/
    │   └── autonomous_navigate.xml             ← 扩展行为树（新增）
    └── launch/
        ├── hunter_full.launch.py               ← 修改：新增 4 个参数开关
        └── hunter_autonomous_nav.launch.py     ← 新增一体化 launch
```

---

## 11. 开发指引

> 💡 **【开发者视角】**

### 11.1 分模块 AI 任务清单

本项目按功能模块拆分为独立的开发/联调任务，每个模块均对应设计文档章节，便于分模块开发与验收：

| 任务编号 | 功能模块 | 设计文档章节 |
|----------|----------|--------------|
| 任务 00 | 项目骨架与工作空间 | §4.2 |
| 任务 01 | 自定义消息 `hunter_msgs` | §4.3 |
| 任务 02 | 车辆 URDF/Xacro 与静态 TF | §7.4 / 附录D |
| 任务 03 | 启动配置 yaml 集合 | §4.2 / 附录C |
| 任务 04 | CH10X IMU 驱动 | §3.3.3 |
| 任务 05 | 激光感知节点 | §5.1 |
| 任务 06 | 视觉感知节点 | §5.2 |
| 任务 07 | 数据融合节点 | §6 |
| 任务 08 | 定位启动配置 | §7 |
| 任务 09 | Nav2 导航栈启动配置 | §8 / §9 / §10 |
| 任务 10 | 模式仲裁决策节点 | §13.5 |
| 任务 11 | 系统监控健康管理 | §15 |
| 任务 12 | 数据采集 Agent | §14 |
| 任务 13 | OTA Agent（systemd 服务） | §12 |
| 任务 14 | 远程操控 Agent（systemd 服务） | §13 |
| 任务 15 | 全系统总启动 launch | §4.4 |
| 任务 16 | 运维 Shell 工具集 | §20.3 |
| 任务 17 | 自主导航模块（auto_mission + 行为树扩展 + 三工具节点） | 自主导航扩展 |

### 11.2 接口契约驱动开发原则

- 移动端模块之间以 `hunter_msgs` 自定义消息（设计文档 §4.3）为**接口契约**通过 ROS2 话题/服务通信；
- 开发/修改模块时，**先对齐接口契约**（消息字段、话题名、频率、坐标系），再实现内部逻辑；
- 新增或修改消息字段时，需保持与设计文档 §4.3 一致，**不随意增删字段**，以免破坏下游消费者（如 `decision_making`、`data_agent`）。

---

## 12. 运维脚本说明

> 🔧 **【现场运维视角】** 脚本位于 `hunter_bringup/scripts/`，需先赋予执行权限：

```bash
chmod +x ~/HunterEdge/src/hunter_bringup/scripts/*.sh
```

| 脚本 | 用途 |
|------|------|
| `hunter_status.sh` | 查看系统状态、节点存活、资源占用 |
| `hunter_log.sh` | 查看/导出系统日志 |
| `hunter_can_test.sh` | CAN 通信测试 |
| `hunter_bag.sh` | ROS Bag 录制/回放 |

### 12.1 系统状态

```bash
./hunter_status.sh          # 节点列表 + 关键节点存活 + CPU/内存/磁盘/温度
```

### 12.2 日志查看/导出

```bash
./hunter_log.sh                       # 查看最新日志
./hunter_log.sh /tmp/logs_export      # 导出并打包 tar.gz
```

### 12.3 CAN 通信测试

```bash
./hunter_can_test.sh up      # 配置 can2 @ 500Kbps
./hunter_can_test.sh test    # 检测 0x211/0x221 底盘反馈报文
```

### 12.4 Bag 录制/回放

```bash
./hunter_bag.sh record                 # 录制默认话题
./hunter_bag.sh play /data/rosbag/xxx  # 回放
./hunter_bag.sh info /data/rosbag/xxx  # 查看信息
```

---

## 13. 已知限制与注意事项

> ⚠️ **【现场运维视角】** 联调与部署时需关注以下约束（均源自设计文档）。

### 13.1 性能约束（设计文档 §18）

| 指标 | 设计值 |
|------|--------|
| 感知→控制端到端延迟 | < 200ms |
| 系统 CPU 占用 | ~87%（8 核总占比） |
| 系统 GPU 占用 | ~55%（Volta GPU） |
| 内存使用 | ~10GB / 32GB |

> 视觉感知（TensorRT + OpenCV CUDA）为 GPU 密集模块；高负载下注意散热与降频。

### 13.2 默认限速与安全（设计文档 §19.3 / §10.5）

- **默认最高速度 2.0 m/s**，可通过平台配置调整（最高不超过底盘 4.8 m/s）；
- `/cmd_vel` 超时 > 500ms 自动停车；CAN 通信丢失 > 100ms 底盘自动制动；
- 自动驾驶运行时需有安全员监控，可随时急停。

### 13.3 传感器标定要求（设计文档 §20.2）

- 需完成 **LiDAR-Camera 外参标定、LiDAR-IMU 外参标定、车辆运动学标定**；
- 未标定或标定误差会直接影响感知融合与定位精度；关键配置（外参、控制参数）需校验后生效。

### 13.4 散热与功耗模式（设计文档 §3.5 / 附录 B）

- AGX Xavier 默认功耗模式 **MODE_15W**（15W TDP），附录 B 推荐 **MODE_30W**（平衡性能与散热）；
- 温度 > 85℃ 降频告警，> 95℃ 触发保护性降载；依据场景选定功耗模式。

### 13.5 CAN 通信（设计文档 §11.3 / 附录 A）

启动底盘通信前需配置 CAN 接口（can2 @ 500Kbps）。拓扑与说明：
- AGX Xavier 与 Hunter SE 底盘通过 USB-CAN 适配器相连，适配器枚举为 **can2**（实测 candump 可见 0x211/0x221/0x231/0x241/0x251 等底盘反馈帧）；
- can0 为 Jetson 板载 mttcan 控制器（`parentdev c310000.mttcan`），未接底盘线束，candump 静默属正常；
- 接口 UP 状态下改波特率会报 `Device or resource busy`，需先 `sudo ip link set can2 down`；
- 建议加总线故障自动恢复：`sudo ip link set can2 type can restart-ms 100`。

```bash
sudo ip link set can2 type can bitrate 500000   # 若已为 500Kbps 可跳过
sudo ip link set can2 up
candump can2 -n 5                                # 期待 0x211/0x221/0x241 等底盘反馈帧
```

核心报文：`0x111` 运动控制、`0x211` 系统状态、`0x221` 运动反馈。

### 13.6 systemd 服务与非 ROS 进程（设计文档 §12 / §13）

- **OTA Agent** 与 **Remote Agent** 为独立 systemd 服务（非 ROS 节点），需单独部署；
- 两者通过 Kafka / WebSocket 与平台交互，Remote Agent 通过 rclpy 桥接发布 `/remote/command`。

### 13.7 容器化可选（设计文档 §20.5）

平台支持 Docker 镜像部署（`nvidia` 运行时 + host 网络 + 设备直通 + `--ipc=host`），与 OTA 升级（镜像拉取替换）协同；原生 colcon 工作空间部署仍为默认方式。

### 13.8 常见故障排查（设计文档 §20.4）

| 故障现象 | 可能原因 | 排查步骤 |
|----------|----------|----------|
| CAN 无数据 | 接线 / 波特率 / 驱动 | 检查 CAN 线、`ip link show can2`、`candump can2` |
| LiDAR 无点云 | 网络 / 电源 / IP 配置 | `ping` LiDAR IP、检查供电、`rosnode list` |
| 相机无图像 | USB 连接 / 权限 | 检查 USB、`ls /dev/video*`、权限配置 |
| 定位漂移大 | IMU 标定 / 轮速 / 外参 | 检查 IMU 数据、外参文件、EKF 参数 |
| 控制抖动 | 控制参数 / 延迟 | 调整 RPP 参数、检查控制频率 |
| 系统卡顿 | GPU / CPU / 温度 | `tegrastats` 查看资源、降温、降频 |
| 无法连平台 | 网络 / 证书 / Kafka | 检查 4G/WiFi、证书有效期、Kafka 配置 |

---

## 14. 文档索引

关联文档如下：

| 文档 | 说明 |
|------|------|
| 《自动驾驶车辆系统详细设计文档 V2.0》 | 本项目的设计基准；本文档全部参数、话题、CAN 协议、坐标系均可追溯至其对应章节 |
| AI 编码任务清单 | 分模块开发任务（任务 00 ~ 任务 17），指导按模块开发与验收 |
| `User_Manual.md` | 面向现场运维人员的用户手册（独立文档，含详细部署/联调/故障排查/自主导航操作流程） |
| `release.md` | 版本历史（V0.0.1 ~ 当前），记录每版主要功能与修复 |

> **追溯原则**：本 README 中所有硬件参数（§2）、软件版本（§3）、话题（§8）、控制模式（§9）、限制（§13）均源自《自动驾驶车辆系统详细设计文档 V2.0》，未虚构功能。自主导航模块（§10）为在设计文档框架内的扩展实现。

---

*HunterEdge 开发指南 · 文档版本 V1.1 · 编制依据《自动驾驶车辆系统详细设计文档 V2.0》*

