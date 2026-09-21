# HunterEdge 自动驾驶车载系统 — 开发指南

> **项目**：HunterEdge 自动驾驶车载系统
> **文档版本**：V1.2（开发指南）
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

**HunterEdge** 是 HUNTER 自动驾驶平台的**车辆端（边缘智能端）**软件工程。系统运行于基于 **HUNTER SE 阿克曼 UGV 底盘**与 **EDU Pro Kit 传感器套件**构建的自动驾驶车辆上，核心计算平台为 **NVIDIA Jetson AGX Orin Developer Kit 64GB**（设计文档基线为 AGX Xavier 32GB，现场实机为 AGX Orin，以实机为准，详见 §2.2；设计文档 §1.5、§3.2）。

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

### 2.2 计算平台 — NVIDIA Jetson AGX Orin Developer Kit（实机；设计文档 §3.2 基线为 AGX Xavier 32GB）

> ⚠️ **【设备差异警示（V0.0.68/69 现场教训）】** 实机为 **AGX Orin Developer Kit 64GB**（`cat /proc/device-tree/model` → `NVIDIA Jetson AGX Orin Developer Kit`），**不是**设计文档基线的 AGX Xavier。两者 GPU 架构与 CUDA 计算能力不同（**Orin = 8.7 / Xavier = 7.2**），部署 GPU 组件（OpenCV CUDA 源码编译、TensorRT engine）时**勿照抄教程或设计文档中的 Xavier 参数**，必须先按实机确认。

| 参数 | 规格（实机 AGX Orin：官方规格 + 现场实测） |
|------|------|
| CPU | 12 核 NVIDIA Cortex-A78AE @ 2.2GHz |
| GPU | 2048-core Ampere GPU + 64 Tensor Cores（**计算能力 8.7**；trtexec 实测 16 SMs） |
| AI 算力 | 275 TOPS（INT8） |
| 内存 | 64GB 256-bit LPDDR5（trtexec 实测约 62GB 可用） |
| 设备识别 | `/proc/device-tree/model`；`trtexec` 输出 `Device 0: "Orin"`、`Compute Capability: 8.7` |
| 功耗模式 | 以 `sudo nvpmodel -q` 实际档位为准（Orin 档位与 Xavier 不同，勿按 Xavier 档位表执行） |
| 操作系统 | Ubuntu 22.04.5 LTS + JetPack 6（实测内核 5.15.148-tegra / CUDA 12.6 / TensorRT 10.3.0） |

> 设计文档基线 AGX Xavier 32GB（8 核 Carmel / 512-core Volta / 32 TOPS / 10W-15W-30W-50W）仅作历史追溯保留，不作为部署依据。

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

完整版本清单（设计文档 §4.1 + 现场实测修正）：

| 组件 | 版本 | 说明 |
|------|------|------|
| 操作系统 | Ubuntu 22.04.5 LTS（Jammy Jellyfish） | JetPack 6 自带 |
| Linux 内核 | 5.15.148-tegra | 实测值；JetPack 6 / L4T R36.x（Orin 特征） |
| CUDA | 12.6 | GPU 计算（实机 Orin：计算能力 8.7） |
| cuDNN | 9.3.0 | 深度学习加速 |
| TensorRT | 10.3.0 | 模型推理优化；`.engine` 与 GPU 绑定，**须在目标机生成** |
| OpenCV | **4.10.0（源码编译 with CUDA，/usr/local，CUDA_ARCH_BIN=8.7）** | 视觉节点专用（编译方法见 §5.3）；apt 4.5.4 仅供 cv_bridge 等旧组件，不得与视觉节点混链 |
| ROS2 | Humble Hawksbill | 机器人中间件 |
| Python | 3.10.12 | 系统默认 |
| CMake | 3.22.1 | 构建工具 |
| GCC | 11.4.0 | C++ 编译器 |

> ⚠️ **【单一 OpenCV 铁律（V0.0.67/68 现场教训）】** 同一进程绝不允许混链两套 OpenCV（4.10 与 4.5.4 并存会破坏 `cv::Mat` 不变量，每帧必现 `setSize` 断言崩溃）。`vision_perception` 已**解耦 cv_bridge**（自实现 `imageMsgToMat()` 完成消息转换），其 CMake 强制 `OpenCV_DIR=/usr/local/lib/cmake/opencv4`；configure 日志必须包含指纹 `vision_perception: OpenCV 4.10.0 选自 /usr/local/lib/cmake/opencv4`，`ldd` 检查只允许 `so.410`、无 `4.5d`、无 `libcv_bridge`（验收命令见 §6）。

> 💡 **【开发者视角】** 视觉感知依赖 TensorRT + OpenCV CUDA，仅 NVIDIA Jetson 环境可用；CPU 部分算法可跨平台编译，但完整系统需在车载 Jetson 上运行。TensorRT `.engine` 与目标 GPU 绑定，**必须在目标机上用 trtexec 生成**（命令见 §6）。

---

## 4. 项目目录结构

基于设计文档 §4.2 的工作空间 `~/HunterEdge/src` 结构如下：

### 4.1 自定义功能包

| 包 | 用途 | 设计文档章节 |
|----|------|--------------|
| `hunter_bringup` | 全系统/模块化启动 launch、参数 config、行为树、URDF | §4.2 / §4.4 |
| `hunter_drivers/ch10x_driver` | CH10X IMU 驱动 | §3.3.3 |
| `hunter_perception/lidar_perception` | 激光感知（地面分割+欧式聚类+跟踪） | §5.1 |
| `hunter_perception/vision_perception` | 视觉感知（YOLOv8+TensorRT；已解耦 cv_bridge，强制链接 /usr/local OpenCV 4.10.0 CUDA，GPU/CPU 自适应预处理） | §5.2 |
| `hunter_agents/data_agent` | 数据采集上传（Kafka+MinIO） | §14 |
| `hunter_agents/ota_agent` | OTA 升级（systemd 服务） | §12 |
| `hunter_agents/remote_agent` | 远程操控（WebRTC，systemd 服务） | §13 |
| `hunter_common/hunter_msgs` | 自定义消息（DetectedObject/ChassisState/Trajectory/HunterStatus 等，含 AgileX 底盘消息） | §4.3 |
| `hunter_common/hunter_utils` | 公共工具函数库 | §4.2 |
| `hunter_perception/sensor_fusion` | 多传感器目标级数据融合（时间对齐 + 置信度门控 + 匈牙利关联 + 加权融合 + 可行驶区域） | §6 |
| `hunter_monitor/health_monitor` | 系统监控与健康管理 | §15 |
| `decision_making` | 模式仲裁决策（AUTO/REMOTE/ESTOP 状态机） | §13.5 |
| `auto_mission` | AUTO 模式自主任务调度（航点巡航/安全守护/建图控制） | 自主导航扩展 |
  | `hunter_safety/safety_guard` | 碰撞防护与运动学安全约束（scan 碰撞闸/阿克曼曲率钳制/分级预警/测试模式，速度链末级） | §10.8 |

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

### 5.3 车载环境 OpenCV 4.10.0 CUDA 源码编译（必做；设计文档未覆盖，V0.0.69 实测定案）

`vision_perception` 的 GPU 预处理（`cv::cuda::GpuMat` 上下传 / `cuda::resize` / `cuda::cvtColor`）依赖带 CUDA 模块的 OpenCV 4.10.0，安装于 `/usr/local`（源码目录示例 `~/opencv_build`）。**编译 ARCH 必须匹配实机 GPU**（Orin=8.7 / Xavier=7.2），先确认设备再选参数：

```bash
# ① 确认设备（决定 CUDA_ARCH_BIN，勿照抄教程）
cat /proc/device-tree/model        # → NVIDIA Jetson AGX Orin Developer Kit ⇒ ARCH=8.7

# ② 配置（Orin 全量 CUDA 编译约 1~2 小时；FFMPEG/GTK3/Python 绑定均需保留）
cd ~/opencv_build/opencv-4.10.0/build
cmake -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
      -DCUDA_ARCH_BIN=8.7 -DCUDA_ARCH_PTX=8.7 \
      -DWITH_CUDA=ON -DWITH_FFMPEG=ON -DWITH_GTK=ON \
      -DBUILD_opencv_python3=ON \
      -DOPENCV_EXTRA_MODULES_PATH=~/opencv_build/opencv_contrib-4.10.0/modules ..
# ③ 编译安装；若此前 sudo 操作残留 root 属主文件导致 configure 报 Permission denied，先修复属主
sudo chown -R $USER:$USER ~/opencv_build
make -j12 && sudo make install
# ④ 验证：CUDA 内核须包含 sm_87（旧库曾误编 sm_72，GPU 调用报 -217 no kernel image）
cuobjdump --list-elf /usr/local/lib/libopencv_cudawarping.so.410   # 应见 sm_87.cubin
python3 -c "import cv2; print(cv2.__version__)"                    # 应为 4.10.0
```

> ⚠️ **【教训】** ARCH 与设备不符时，GPU 内核运行报 `error (-217) no kernel image is available for execution on the device`；节点代码已内置一次性降级 CPU 兜底（感知不断流，见 §13.8），但**环境必须按上表重编修正**，重启节点后自动恢复 GPU。`sudo make install` 在旧 build 目录执行可完整保留既有配置（FFMPEG/GTK3/Python 绑定）；`CUDA_ARCH_PTX=8.7` 保留 JIT 回退能力。

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

**vision_perception 专项（V0.0.68 起）**：

- CMake 强制 `OpenCV_DIR=/usr/local/lib/cmake/opencv4`（EXISTS 守卫 + CACHE 写法），configure 日志必须出现 `vision_perception: OpenCV 4.10.0 选自 /usr/local/lib/cmake/opencv4` 与 `GPU preprocessing enabled`（无 CUDA OpenCV 时自动走 CPU 路径并打印 `CPU preprocessing`，不算错误）；
- symlink-install 工作区重编该包须先清净：`rm -rf build/vision_perception install/vision_perception && colcon build --packages-select vision_perception`；
- 编后自检：`ldd install/vision_perception/lib/vision_perception/vision_perception_node | grep opencv` → 全部 `so.410`、零 `4.5d`、无 `libcv_bridge`（命中即为混链根因修复态）。

**TensorRT 引擎（`.engine` 与 GPU/设备绑定）**：换机、重装 JetPack 或重编 OpenCV 后需在目标机重新生成：

```bash
trtexec --onnx=yolov8s.onnx --saveEngine=/data/models/yolov8s.engine --fp16
```

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

### 7.6 坐标系与 TF 树（V0.0.70 修订）

系统 TF 由两类发布者构成，**相机驱动自身 TF 已关闭**，不存在双父：

| TF 段 | 发布者 | 说明 |
|-------|--------|------|
| `base_link → rslidar` / `camera_color_optical_frame` / `imu` | `robot_state_publisher`（URDF 静态外参，文档 7.4/附录D） | 相机外参唯一来源（`camera_color_joint`：xyz 0.40/0/0.30，rpy 0/-π/2/π/2） |
| `odom → base_link` | EKF（`ekf_params.yaml` 中 `publish_tf: true`） | 定位输出 |
| `map → odom` | 后续全局定位模块（当前未接入） | — |

- `realsense2_camera` 驱动在 `hunter_full.launch.py` 中固定传入 **`publish_tf: 'false'`**：驱动自建 `camera_link` 树与 URDF 对 `camera_color_optical_frame` 构成同 frame 双父，TF 树分裂为 `base_link` / `camera_link` 两棵，sensor_fusion `lookupTransform` 必败（报 `TF unconnected trees`，V0.0.70 现场问题）；驱动 TF 无任何消费者（相机外参唯一来源是 URDF；`align_depth` 在驱动内部完成不依赖 ROS TF），关闭无副作用，仅 RViz 少显示 realsense 原生 TF 视角；
- 验证：`ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame` 应输出 translation (0.40, 0.00, 0.30)、rotation 对应 rpy (0, −π/2, π/2)；调试可用 `ros2 run tf2_tools view_frames` 导出 frames.pdf 确认全树单棵连通。

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
| `/safety/state` | `std_msgs/String` | 2Hz | safety_guard 分级预警心跳（状态|原因；OK/SLOWDOWN/COLLISION_STOP/SCAN_TIMEOUT/CMD_TIMEOUT/ESTOP_PASS/TEST_ABORTED/MAP_EDGE_SLOWDOWN/MAP_EDGE_STOP） |
| `/safety/test_mode` | `std_msgs/Bool` | 事件 | 自动驾驶测试模式开关（true 开启 0.1m/s 限速+严阈值+异常自动中止） |
| `/cmd_vel_nav` | `geometry_msgs/Twist` | 20Hz | controller_server 原始速度指令（safety_guard 测试模式监控其断流） |

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
    │   │                               （V0.0.88：Ctrl+C 退出时派生独立会话后台
    │   │                                进程兜底，等 FAST-LIO2 把 PCD 写完整后转换，
    │   │                                日志 maps/pcd_to_map_final.log）
    │   └── waypoint_recorder         ← /clicked_point 自动写入航点 yaml
    └── [nav 模式]
        ├── fast_lio2_param_injector  ← 自动注入 pcd_save_en=false
        ├── map_server                ← 加载静态 PGM 地图
        ├── Nav2 全栈                 ← 使用 autonomous_navigate.xml 扩展行为树
        ├── auto_mission_node         ← 状态机（航点巡航/安全守护）
        └── safety_guard              ← 碰撞闸（V0.0.85/86：scan 急停/限速 + 阿克曼
                                          曲率钳制 + 速度硬限 + 测试模式，速度链末级，
                                          发布 /cmd_vel 至底盘）

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
| `max_velocity` | 0.8 m/s | 巡航速度（V0.0.85 全链降速，与 RPP desired_linear_vel 一致） |
| `loop_waypoints` | `true` | 完成所有航点后是否循环 |
| `goal_timeout` | 60 s | 单点导航超时 |
| `obstacle_wait_timeout` | 30 s | 障碍物等待超时后触发 ESTOP |

### 10.5 自动化工具节点

| 节点 | 可执行文件 | 解决的手动操作 |
|------|-----------|---------------|
| `fast_lio2_param_injector` | 同名 | 自动向 `fast_lio2` 注入 `pcd_save_en` + `map_file_path` |
| `pcd_to_map` | 同名 | 建图结束自动将 PCD 三维点云转换为 Nav2 栅格地图（.pgm + .yaml）；V0.0.88 起 Ctrl+C 退出时另派生**独立会话**后台转换进程兜底（等 FAST-LIO2 写完 PCD 后转换，日志 `maps/pcd_to_map_final.log`），并支持 `--finalize` 离线转换（无需 ROS） |
| `waypoint_recorder` | 同名 | 建图时 rviz2 点击即自动追加写入 `autonomous_nav_params.yaml` |

### 10.6 扩展行为树

`autonomous_navigate.xml` 在原 `navigate_to_pose.xml` 基础上新增：

- **定位门控**：`TransformAvailable(map→base_link)` 前置检查，定位不可用时立即阻断导航；
- **感知保鲜**：`TimeExpired(2s)` 哨兵，感知超时时清除局部代价地图并等待恢复；
- ~~动态减速~~（V0.0.82 移除 `SpeedController`：本 fork 该节点为按平滑速度调子树 tick 周期的装饰器，无"障碍物距离→限速"语义）；障碍物减速由 RPP `use_cost_regulated_linear_velocity_scaling`（近障碍自动降速）+ approach 减速承担；
- **阿克曼后退**：`BackUp(0.3m)` 替代 `Spin`（原地旋转），适合阿克曼底盘脱困。
- **重规划提速**（V0.0.85）：RateController 1.0→2.0Hz——RPP 为纯路径跟随器、
  无局部避障语义，动态障碍全靠全局重规划绕行，1Hz×高速=数米级盲区；配合全链降速 0.8m/s。

### 10.8 碰撞防护与安全约束（V0.0.85 新增 hunter_safety/safety_guard）

速度指令链最后一级物理安全闸，串联于 `velocity_smoother` 与底盘之间
（launch 重映射 cmd_vel_smoothed→cmd_vel_pre_safety，safety_guard 发布 /cmd_vel），
数据源为 /scan（pointcloud_to_laserscan 直出），**不依赖感知融合链**，与决策层
（auto_mission OBSTACLE_AVOID/ESTOP）、模式仲裁（decision_making）互为冗余：

| 能力 | 触发条件 | 动作 |
|------|----------|------|
| 碰撞急停 | 行进方向 ±60° 扇区最近障碍 < stop_dist(0.6m) | 立即零速 COLLISION_STOP |
| 碰撞限速 | 最近障碍 < slow_dist(1.2m) | 线性限速至 max×(d−stop)/(slow−stop)，SLOWDOWN |
| 感知 fail-safe | /scan 超时 0.5s 或未到达 | 零速（宁可停车不盲走）SCAN_TIMEOUT |
| 指令看门狗 | 上游速度指令断流 >0.5s | 零速心跳 CMD_TIMEOUT |
| 急停透传 | /estop=true | 零速（弥补 Nav2 goal 取消延迟窗口）ESTOP_PASS |
| 阿克曼曲率钳制 | 恒生效 | \|w\| ≤ \|v\|/1.9（δ≤0.33rad，杜绝打满转向） |
| 速度硬限 | 恒生效 | \|v\| ≤ 0.8m/s（第二重限速） |
| 测试模式碰撞急停 | 测试模式开启时同扇区 < test_stop_dist(1.0m) | 立即零速 COLLISION_STOP【测试模式】 |
| 测试模式减速 | 测试模式开启时同扇区 < test_slow_dist(2.0m) | 限速 ≤0.1m/s（test_max_linear_vel） |
| 测试模式异常中止 | 疑似碰撞卡死（指令>0.05m/s 而反馈≈0 持续 1s）/ goal ABORTED / goal 活跃但 /cmd_vel_nav 断流 >2s | 零速锁存 TEST_ABORTED + /estop=true + 取消全部导航目标 |
| 地图边界减速（V0.0.87） | 车辆距未建图(unknown)/界外栅格 < map_edge_slow_dist(1.5m) | 线性限速至 max×(d−stop)/(slow−stop)，MAP_EDGE_SLOWDOWN |
| 地图边界停车（V0.0.87） | 车辆距未建图(unknown)/界外栅格 < map_edge_stop_dist(0.5m) | 立即零速 MAP_EDGE_STOP（回到已建图区域自动恢复）；测试模式下升级为中止锁存 TEST_ABORTED |

> **地图边界约束（V0.0.87 三层防线）**：保证车辆行驶范围、预设航点、导航点与
> 规划路径均在已采集地图区域内——① Nav2 规划层：`global_costmap
> track_unknown_space: true` + Smac `allow_unknown: false`，全局规划路径不穿越
> 未采集区域；② 任务层：auto_mission 发送航点前校验（矩形边界+0.5m 边距+
> 非 unknown 栅格），越界航点自动跳过；③ 执行层：safety_guard 地图边界监护
> （/map + /amcl_pose 距离场，上表最后两行），行驶中越界零速兜底。建图模式
> 无 /map 与 AMCL，③ 自动不介入。

分级预警：/safety/state（std_msgs/String）2Hz 心跳，格式 状态|原因；
仅导航模式启动（mapping 模式 auto_mission cruise 直发 /cmd_vel，避免双发布者）。

**自动驾驶测试模式（V0.0.86）**：低速实车联调专用。运行时开关：

```bash
ros2 topic pub --once /safety/test_mode std_msgs/msg/Bool "{data: true}"   # 开启
ros2 topic pub --once /safety/test_mode std_msgs/msg/Bool "{data: false}"  # 关闭（恢复常规阈值）
```

开启后 0.1m/s 限速巡航、±60° 扇区 <1.0m 急停/<2.0m 减速（较常规 0.6/1.2m 更早介入），
并自动监控四类异常——疑似碰撞卡死、控制器断流、Nav2 goal ABORTED、
**地图越界（V0.0.87，距未建图/界外栅格 <0.5m）**——任一发生立即
零速锁存（TEST_ABORTED）并发布 /estop=true（auto_mission 取消全部导航任务）；
锁存后即使外部把 /estop 清回 false 也保持零速，必须重新发布 true 才能解除
（视为人工确认现场安全）。

地图边界监护（V0.0.87）运行时参数（launch 注入，默认全开）：
`enable_map_fence`（开关）、`map_edge_stop_dist`（0.5m 零速阈值）、
`map_edge_slow_dist`（1.5m 减速阈值）；监护在 /map 与 /amcl_pose 均就绪后生效，
启动日志输出 `[边界监护] 已就绪：栅格 WxH @0.050m/cell ...`。

### 10.9 新增/修改文件清单

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
  ├── hunter_safety/                              ← 新增包（V0.0.85 碰撞防护）
  │   ├── CMakeLists.txt / package.xml
  │   ├── launch/safety_guard.launch.py           ← 独立调试启动
  │   └── src/safety_guard.cpp                    ← scan 碰撞闸/曲率钳制/预警实现
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
| 系统 GPU 占用 | ~55%（Orin Ampere GPU） |
| 内存使用 | ~10GB / 32GB |

> 视觉感知（TensorRT + OpenCV CUDA）为 GPU 密集模块；高负载下注意散热与降频。

### 13.2 默认限速与安全（设计文档 §19.3 / §10.5）

- **默认最高速度 2.0 m/s**，可通过平台配置调整（最高不超过底盘 4.8 m/s）；
- `/cmd_vel` 超时 > 500ms 自动停车；CAN 通信丢失 > 100ms 底盘自动制动；
- **地图边界约束（V0.0.87）**：自主巡航行驶范围不得超过已采集地图区域——
  Nav2 规划层不穿越未建图区域（track_unknown_space + allow_unknown=false）、
  航点发送前校验、safety_guard 边界监护行驶中限速/零速兜底（§10.8）；
- 自动驾驶运行时需有安全员监控，可随时急停。

### 13.3 传感器标定要求（设计文档 §20.2）

- 需完成 **LiDAR-Camera 外参标定、LiDAR-IMU 外参标定、车辆运动学标定**；
- 未标定或标定误差会直接影响感知融合与定位精度；关键配置（外参、控制参数）需校验后生效。

### 13.4 散热与功耗模式（设计文档 §3.5 / 附录 B）

- 实机为 **AGX Orin**，功耗档位以 `sudo nvpmodel -q` 实际输出为准（设计文档的 Xavier 档位表仅作历史基线，勿直接执行）；
- GPU 密集任务（TensorRT + OpenCV CUDA 预处理）建议使用高性能档并确认散热正常；
- 温度 > 85℃ 降频告警，> 95℃ 触发保护性降载；依据场景选定功耗模式。

### 13.5 CAN 通信（设计文档 §11.3 / 附录 A）

启动底盘通信前需配置 CAN 接口（can2 @ 500Kbps）。拓扑与说明：
- AGX Orin 与 Hunter SE 底盘通过 USB-CAN 适配器相连，适配器枚举为 **can2**（实测 candump 可见 0x211/0x221/0x231/0x241/0x251 等底盘反馈帧）；
- can0 为 Jetson 板载 mttcan 控制器（`parentdev c310000.mttcan`），未接底盘线束，candump 静默属正常；
- 接口 UP 状态下改波特率会报 `Device or resource busy`，需先 `sudo ip link set can2 down`；
- 建议加总线故障自动恢复：`sudo ip link set can2 type can restart-ms 100`。

```bash
sudo ip link set can2 type can bitrate 500000   # 若已为 500Kbps 可跳过
sudo ip link set can2 up
candump can2 -n 5                                # 期待 0x211/0x221/0x241 等底盘反馈帧
```

核心报文：`0x111` 运动控制、`0x211` 系统状态、`0x221` 运动反馈。

> ⚠️ **【已知待办】** can2 配置（bitrate / restart-ms）**重启后不保留**：每次开机后需重新执行上行配置命令；持久化方案（启动脚本或 `/etc/network/interfaces.d`）尚未落地。

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
| 视觉节点每帧崩溃（`setSize` 断言） | OpenCV 4.10 / 4.5.4 **混链**（同进程两套 OpenCV，破坏 `cv::Mat` 不变量） | `ldd vision_perception_node \| grep opencv`：只允许 `so.410`、无 `4.5d`、无 `libcv_bridge`；异常时按 §6 专项重编（禁止在视觉进程重新引入 cv_bridge） |
| 视觉 0Hz，日志 `resize.cu:175 error (-217) no kernel image` | OpenCV CUDA 编译 ARCH 与实机 GPU 不符（如 sm_72 用于 Orin） | `cuobjdump --list-elf /usr/local/lib/libopencv_cudawarping.so.410` 应见 `sm_87`；按 §5.3 以 `CUDA_ARCH_BIN=8.7` 重编后重启节点即恢复 GPU（期间节点自动降级 CPU，感知不断流） |
| sensor_fusion 报 `TF unconnected trees` | 相机 frame 双父（驱动 TF + URDF 并存，TF 树分裂） | 确认 `hunter_full.launch.py` 相机驱动为 `publish_tf: 'false'`；`ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame` 验证外参；必要时 `tf2_tools view_frames` 看全树 |
| 启动时一次性 `彩色图像超时 x.x s` | 启动竞态（视觉节点激活早于彩色流就绪） | 仅出现一次属良性，可忽略；反复出现才按"相机无图像"排查 |
| 一次性 `[TensorRT] Using an engine plan file across different models of devices` | `.engine` 非本机/本设备型号生成（换机或文件被旧引擎覆盖） | 不阻塞运行（话题 15Hz 正常）；目标机重生成：`trtexec --onnx=<绝对路径>/yolov8s.onnx --saveEngine=/data/models/yolov8s.engine --fp16` 后重启视觉节点 |
| Ctrl+C 后 `maps/` 只有 `.pcd`，`.pgm/.yaml` 未生成（V0.0.88 前必现） | FAST-LIO2 在 `main()` 于 `spin` 返回**后**才写 PCD（20.7M 点 ≈ 664MB 需数秒至数十秒），而 Ctrl+C 同时终止 `pcd_to_map`，运行期 `MAPPING→非MAPPING` 跳变不会发生 → 原自动转换从不启动 | V0.0.88 起 `pcd_to_map` 退出时派生独立会话后台转换进程兜底：`tail -f maps/pcd_to_map_final.log`（应见 `PCD 已写完整 → 转换成功`），数十秒内 `ls -lh maps/` 应齐 `.pcd/.pgm/.yaml`；仍缺时手动兜底 `python3 ~/HunterEdge/install/auto_mission/lib/auto_mission/pcd_to_map --finalize --pcd-file ~/HunterEdge/maps/hunter_map.pcd --force` |

---

## 14. 文档索引

关联文档如下：

| 文档 | 说明 |
|------|------|
| 《自动驾驶车辆系统详细设计文档 V2.0》 | 本项目的设计基准；本文档全部参数、话题、CAN 协议、坐标系均可追溯至其对应章节 |
| AI 编码任务清单 | 分模块开发任务（任务 00 ~ 任务 17），指导按模块开发与验收 |
| `User_Manual.md` | 面向现场运维人员的用户手册（独立文档，含详细部署/联调/故障排查/自主导航操作流程） |
| `Deployment_Guide.md` | 部署操作文档 V1.0（环境要求/环境配置/环境安装/源码部署/功能操作步骤/异常处理全流程，对应软件基线 V0.0.72） |
| `release.md` | 版本历史（V0.0.1 ~ 当前），记录每版主要功能与修复 |

> **追溯原则**：本 README 中所有硬件参数（§2）、软件版本（§3）、话题（§8）、控制模式（§9）、限制（§13）均源自《自动驾驶车辆系统详细设计文档 V2.0》，未虚构功能。自主导航模块（§10）为在设计文档框架内的扩展实现。

---

*HunterEdge 开发指南 · 文档版本 V1.2 · 编制依据《自动驾驶车辆系统详细设计文档 V2.0》，并含 V0.0.67~V0.0.70 现场实测修正*

