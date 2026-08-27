# HunterEdge

HUNTER 自动驾驶平台（NVIDIA AGX Xavier + ROS2 Humble）车载端工作空间。

基于《自动驾驶车辆系统详细设计文档》实现的车载端完整软件栈，覆盖感知、融合、定位、决策、规划、控制、数据采集、OTA 升级、远程操控与系统监控全链路。

## 系统架构

### 自定义功能包

| 模块 | 包 | 说明 |
|------|-----|------|
| 启动配置 | `hunter_bringup` | 全系统/模块化 launch、参数 yaml、URDF、行为树、运维脚本 |
| 消息定义 | `hunter_common/hunter_msgs` | DetectedObject/ChassisState/ChassisCommand/Trajectory/BehaviorState/SystemHealth |
| 工具库 | `hunter_common/hunter_utils` | 公共工具函数 |
| IMU 驱动 | `hunter_drivers/ch10x_driver` | CH10X IMU 驱动（UART，100Hz，生命周期节点） |
| 激光感知 | `hunter_perception/lidar_perception` | 地面分割 + 欧式聚类 + OBB + 卡尔曼跟踪 |
| 视觉感知 | `hunter_perception/vision_perception` | YOLOv8n TensorRT FP16 + 深度 2D→3D 投影 |
| 数据融合 | `hunter_perception/sensor_fusion` | 匈牙利关联 + 加权融合 + 可行驶区域 |
| 模式仲裁 | `decision_making` | AUTO/REMOTE/ESTOP 状态机（ESTOP > REMOTE > AUTO） |
| 系统监控 | `hunter_monitor/health_monitor` | 节点/传感器频率/资源监控、故障降级 |
| 数据采集 | `hunter_agents/data_agent` | Kafka 遥测+事件上报、SQLite 缓存 |
| OTA 升级 | `hunter_agents/ota_agent` | systemd 服务（非 ROS，Python） |
| 远程操控 | `hunter_agents/remote_agent` | systemd 服务（非 ROS，Python，WebRTC） |

### 第三方依赖包

| 包 | 用途 |
|----|------|
| `hunter_ros2` / `ugv_sdk` | AgileX HUNTER SE 底盘驱动 |
| `rslidar_sdk` | RS-Helios-16P 激光雷达驱动 |
| `realsense-ros` | Intel RealSense D435 相机驱动 |
| `fast_lio2` | 紧耦合雷达惯导里程计 |
| `robot_localization` | EKF 多传感器融合定位 |
| `navigation2` | Nav2 导航栈（SmacPlannerHybrid + RPP） |
| `yolo_trt_ros` | YOLO TensorRT 推理封装（可选） |

> **导入方式**：在 Ubuntu 车载环境执行 `./src/hunter_bringup/scripts/import_vendor.sh`，自动 git clone 全部第三方包；navigation2 / robot_localization / realsense2_camera 也可用 `apt` 安装（`ros-humble-*` 二进制包）。

## 目录结构

```
HunterEdge/
├── src/
│   ├── hunter_bringup/          # 启动配置（launch/config/urdf/behavior_trees/scripts）
│   ├── hunter_common/           # 公共库（hunter_msgs / hunter_utils）
│   ├── hunter_drivers/          # 传感器驱动（ch10x_driver）
│   ├── hunter_perception/       # 感知（lidar_perception / vision_perception / sensor_fusion）
│   ├── hunter_monitor/          # 系统监控（health_monitor）
│   ├── hunter_agents/           # 平台交互（data_agent / ota_agent / remote_agent）
│   ├── decision_making/         # 模式仲裁决策
│   └── (第三方依赖包...)
├── README.md
├── release.md                   # 版本更新文档 + 用户手册
├── LICENSE                      # Apache 2.0
└── .gitignore
```

## 构建

```bash
cd ~/HunterEdge
colcon build --symlink-install
source install/setup.bash
```

## 启动

```bash
# 全系统启动（严格按启动顺序）
ros2 launch hunter_bringup hunter_full.launch.py

# 模块化启动（参数开关）
ros2 launch hunter_bringup hunter_full.launch.py use_perception:=false   # 关闭感知
ros2 launch hunter_bringup hunter_full.launch.py use_navigation:=false   # 关闭导航
ros2 launch hunter_bringup hunter_full.launch.py use_data_agent:=false   # 关闭数据采集

# 独立模块启动
ros2 launch hunter_bringup localization.launch.py   # 定位
ros2 launch hunter_bringup perception.launch.py     # 感知
ros2 launch hunter_bringup navigation.launch.py     # 导航
```

## 运维工具

| 脚本 | 功能 |
|------|------|
| `hunter_status.sh` | 节点存活、资源占用 |
| `hunter_log.sh` | 日志查看/导出 |
| `hunter_can_test.sh` | CAN 通信调试 |
| `hunter_bag.sh` | Bag 录制/回放 |

## systemd 服务

| 服务 | 单元文件路径 |
|------|-------------|
| OTA Agent | `hunter_agents/ota_agent/scripts/ota-agent.service` |
| Remote Agent | `hunter_agents/remote_agent/scripts/remote-agent.service` |

## 许可

[Apache License 2.0](LICENSE)
