# HunterEdge 自动驾驶车载系统 — 开发指南

> **项目**：HunterEdge 自动驾驶车载系统
> **文档版本**：V2.0（开发指南，对应软件基线 V0.0.98：V0.0.97（几何死锁修复：`stop_dist 0.90`/`slow_dist 1.40`/`range_min 0.70` + MPPI 放开倒车 + Smac `REEDS_SHEPP` + goal 世代号）经实车验证已消除“满舵死磕”，但车辆**仍**绕不开障碍物——本版修复第二层根因：① **安全层与避障层互斥（主因）**：旧碰撞闸把 (v,w) 指令只按行进方向**直线投影**求净空，绕障弧起点必落在走廊内 → MPPI 每生成绕障轨迹执行 0.1~0.3s 即被按停（“能规划、能出弧、就是走不出去”）——**V0.0.98 safety_guard 轨迹扫掠弧（swept-arc）碰撞闸**：行进中按指令积分真实阿克曼轨迹（前 `reaction_lag 0.4s` 直线、其后圆弧 |w|≤v/1.9、弧长等步长 5cm）+ 车体包络盒扫掠，输出与走廊同量纲的纵向等价净空，“绕得开就放行、真撞才拦”（新参 `reaction_lag 0.4`/`brake_decel 1.5`/`vel_trust_eps 0.03`）；② **MPPI 预测时域/critic 重标定**：时域 1.5s→**4.0s**（`50×0.08`，旧值 @0.25m/s 仅前瞻 0.38m 看不到绕障弧）、`batch_size 700`、`CostCritic 3→6`、`PathAlign 14→8`+`occupancy_ratio 0.05→0.15`（旧值使合理离径被饱和惩罚）；③ **取消竞态残留**：`Goal was canceled` 后 2ms 即发新 goal → 被旧 BT 失败状态波及 ABORTED → 新增 **`goal_cancel_settle_time 2.0s` 静置门控**；④ /scan 链降载（`angle_increment 1.0°` 360 束、`transform_tolerance 0.3`、`expected_update_rate 0.25`）。历史：V0.0.96（航点净空校验、恢复池倒车）、V0.0.97（绕障几何 + 世代号）修复均已实车验证生效）
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

> ⚠️ **【巡航前置检查（V0.0.95 实车教训）】** 切 AUTO 前确认：① 车头正前方 **≥2m** 净空
> （V0.0.94 实车即因起点正前方 ~1m 处障碍 + 航点[0]与车位重合，出现原地抖动不前进）；
> ② `waypoints` 首点与车辆起始位姿距离 **>0.30m**（否则该航点被判“已到达”跳过；全部航点
> 都在 0.30m 内会直接 FAULT 锁存）；③ 相机彩色流正常（`ros2 topic hz /camera/camera/color/image_raw`），
> 否则避障退化为单雷达源（见 §13.8 相机全程 0Hz 条目）。

> 💡 **【开发者视角】** 模块化启动便于逐模块联调；`hunter_full.launch.py` 的参数开关见上表（设计文档 §4.4）。

### 7.6 坐标系与 TF 树（V0.0.93 方案A 修订）

每条 TF 边严格**单一发布者**（打架是 V0.0.93 之前“无法自主巡航”根因）：

| TF 段 | 发布者 | 说明 |
|-------|--------|------|
| `base_link → rslidar` / `camera_color_optical_frame` / `imu` | `robot_state_publisher`（URDF 静态外参，文档 7.4/附录D） | 相机外参唯一来源（`camera_color_joint`：xyz 0.40/0/0.30，rpy 0/-π/2/π/2） |
| `odom → base_link` | **FAST-LIO2**（`laserMapping.cpp` 唯一广播；底盘 `hunter_base` 与 EKF 均 `publish_tf=false`） | LIO 紧耦合里程计；帧名已归一（原 camera_init/body） |
| `map → odom` | **`hunter_relocalization`**（NDT 点云配准，V0.0.93 新增；V0.0.94 将 TF 广播与 NDT 解耦） | 将实时点云对齐先验全局 .pcd；替代 AMCL。**V0.0.94**：NDT 低频（2Hz）精炼 `T_map_odom_`，另用 **20Hz 定时器持续广播 `map→odom`**（取最新缓存值）避免下游外推失败；位姿刷新只由跳变闸门 `step_ok`+`fit<=fitness_hard_ceiling` 决定（运动时不冻结、可回弹），`fit<=fitness_max` 仅决定对外协方差置信度（带 `diverge_tolerance_cycles` 去抖） |

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
| `/safety/test_mode` | `std_msgs/Bool` | 事件 | 自动驾驶测试模式开关（true 开启 0.3m/s 限速+严阈值+异常自动中止，V0.0.89） |
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
| **全局重定位(NDT) x+y 方差和 ≤ 0.10（可配，V0.0.93）** | 订阅 `/relocalization/pose`（hunter_relocalization 收敛时小协方差、未收敛时大协方差。**V0.0.94**：NDT 位姿刷新与对外置信度解耦——`fit<=fitness_max` 仅决定协方差且带 `diverge_tolerance_cycles` 连续失败去抖，运动畸变帧仍持续跟踪不冻结，避免虚假未收敛致秒退 IDLE） |
| 感知数据新鲜度 ≤ 2s（可配） | 订阅 `/perception/fused_objects` 时间戳 |

### 10.3 任务状态机

```
IDLE ──[AUTO条件满足]──→ WAITING_LOCALIZE ──[收敛]──→ NAVIGATING
IDLE ──[mapping模式]──→ MAPPING
NAVIGATING ──[障碍物 < warn_dist]──→ OBSTACLE_AVOID ──[路清]──→ NAVIGATING
NAVIGATING ──[障碍物 < stop_dist]──→ ESTOP
NAVIGATING ──[航点受阻：stall_detect_time 内位移 < stall_move_eps]──→ 取消 goal 换点
                                                          （连续达 max_wp_failures → FAULT）
NAVIGATING ──[航点已在 already_reached_dist 内 / 越界]──→ 跳过该航点（不发 goal）
NAVIGATING ──[连续失败达 max_wp_failures]──→ FAULT（V0.0.91 锁存：不发 goal、不重发，
                                                  需模式开关离开 AUTO 再切回）
任意状态 ──[非AUTO/急停]──→ IDLE / ESTOP
```

> **V0.0.95 任务层两条新防线**（对应“遇到障碍物无法绕开障碍物自动驾驶”）：
> ① **航点“已到达”预检**：航点与 `/relocalization/pose` 位置重合（≤ `already_reached_dist` 0.30m）
>    时视为已完成并跳过——goal 与自身位姿重合时 Smac 路径≈0 且要求终止朝向，阿克曼无法
>    原地转向，MPPI 会持续打满转向（日志 `set steering angle: ±0.386428 rad` 即曲率钳制上限
>    对应的最大内轮转角）而纵向零进挪，ProgressChecker(0.1m/10s) 必判 `Failed to make progress`；
> ② **受阻（stall）检测**：goal 在途且朝目标推进量 < `stall_move_eps`(0.15m) 持续 `stall_detect_time`(25s)
>    → 明确判“前方障碍无法绕行/航点无可达路径”，取消本 goal 并换点（连续 3 次 → FAULT 锁存），
>    日志给出可判读结论，避免“原地抖动 90s 后静默换点”。（V0.0.97 补：累计行程 ≥ `stall_path_allow_m` 1.0m
>    视为绕障机动中不判受阻；V0.0.98 补：换点重发经 `goal_cancel_settle_time` 静置门控，不再取消后 2ms 立即重发）
>
> **V0.0.96 任务层第三条防线 + 到达语义修正**：
> ③ **航点占据栅格 + 净空校验**（`waypoint_clearance_m` 0.50m）：发送前拒绝“落在静态地图障碍上/其
>    膨胀区内”的航点（现场 (5.0,0.0) 即此类，车开到该点后 Smac 持续抛 `Starting point in lethal space!`）；
> ④ **“ABORT 但已在到达半径内 ⇒ 判为完成”**：避免“车已到 0.07m 却因规划失败被整树 ABORT”被计成
>    航点失败而快速累积到 FAULT。

### 10.4 安全约束参数（`autonomous_nav_params.yaml`）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `warn_obstacle_dist` | 1.5 m | 障碍物减速警告距离（V0.0.89 窄小测试场地档） |
| `stop_obstacle_dist` | 0.6 m | 障碍物急停距离（V0.0.89，略大于 footprint 前缘 0.45） |
| `obstacle_fov_deg` | 120° | 前向检测扇区 |
| `localize_cov_threshold` | 0.5 | 里程计（odom→base_link）定位协方差迹收敛阈值 |
| `amcl_cov_threshold` | 0.10 | 全局重定位(NDT)（map→base_link）x+y 方差和收敛阈值（**V0.0.93 语义由 AMCL 改为 NDT**；成员名沿用 amcl_*。hunter_relocalization 收敛时发布 covariance[0]=[7]=0.01（和=0.02）放行，未收敛时=100（和=200）拦截） |
| `localize_wait_timeout` | 15 s | 等待定位收敛超时（NDT 首帧即可收敛，保留 15s 供大场景首帧充分扫到特征） |
| `perception_timeout` | 2 s | 感知数据超时阈值 |
| `max_velocity` | 0.5 m/s | 巡航速度（V0.0.89 窄小测试场地低速档，与 MPPI vx_max/velocity_smoother/safety_guard 一致） |
| `loop_waypoints` | `true` | 完成所有航点后是否循环 |
| `goal_timeout` | 45 s | 单点导航超时（**V0.0.95 90→45**：受阻由 `stall_detect_time` 25s 内提前定性，本超时只兜底“缓慢但可达”的航点） |
| `obstacle_wait_timeout` | 30 s | 障碍物等待超时后触发 ESTOP |
| `already_reached_dist` | 0.35 m | **V0.0.95 新增；V0.0.97 由 0.30 上调**：航点“已到达”判定半径。航点与当前 map 系位姿距离 ≤ 此值时视为已完成并**跳过不发 goal**（修“目标=自身”退化 goal 死锁；0.304m 退化目标实例）。须 > 阿克曼停车精度且 ≥ 2× Nav2 `xy_goal_tolerance`(0.10m)，不宜 >0.4m。全部航点均在此半径内 → FAULT 锁存 |
| `stall_path_allow_m` | 1.0 m | **V0.0.97 新增**：绕障机动宽容——自本 goal 发出起**累计行程** ≥ 此值即视为“多点掉头/倒车绕障中”，即使净推进 <`stall_move_eps` 也不判受阻（放开倒车后正常绕障是“倒 0.4m→进 0.5m→再倒”多段机动，净推进可长期偏小）；兜底 `goal_timeout` 45s |
| `goal_cancel_settle_time` | 2.0 s | **V0.0.98 新增**：取消静置门控。本节点主动 cancel 旧 goal 后换点重发前等待旧结果收敛（CANCELED/ABORTED 先到即提前放行，超时兜底）——消除日志实证的“取消后 2ms 即发新 goal → 新 goal 被旧 BT 失败状态波及 ABORTED → fail_count 误累积→ FAULT”；<0.5 强制回 2.0，不建议 >5s；配套：`ABORTED + 本节点主动取消` 不计失败不换点 |
| `stall_detect_time` | 25 s | **V0.0.95 新增**：受阻判定时长。goal 在途且朝目标推进停滞 → 判“前方障碍无法绕行/航点无可达路径”，取消本 goal 并换点。须 > 一轮 Nav2 恢复周期（ProgressChecker 10s + 清图 + 受限倒车脱困） |
| `stall_move_eps` | 0.15 m | **V0.0.95 新增**：受阻判定位移下限（>2× 定位抖动）。**V0.0.96 判据改“朝目标推进量”**（= 发 goal 时到航点距离 − 当前距离）——倒车脱困会增大到目标距离（推进量为负）仍判受阻，位移标量会被“后退”骗过 |
| `waypoint_clearance_m` | 0.50 m | **V0.0.96 新增**：航点距最近**占据栅格**的最小净空。航点为占据栅格 / 净空不足 → 发送前拒绝并跳过（日志给实测值），全部被拒 → FAULT。必须 > Nav2 内切半径（footprint 半宽 0.32m）；必要性：滚动局部代价地图不含 static_layer，MPPI 不会避开仅存在于静态地图中的障碍，会把车开到该点，随后 Smac 持续报 `Starting point in lethal space!`（清图无效——`StaticLayer::reset()` 只置 `has_updated_data_` 并重新盖章） |

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
- ~~动态减速~~（V0.0.82 移除 `SpeedController`：本 fork 该节点为按平滑速度调子树 tick 周期的装饰器，无"障碍物距离→限速"语义）；障碍物减速由 **MPPI** `CostCritic`/`PathAlignCritic`（近障碍自动降速，V0.0.93）+ approach 减速承担；
- ~~阿克曼后退~~（V0.0.89 移除，**V0.0.95 有条件恢复**）：`BackUp` 在 `FollowPath` 失败恢复序列中恢复为**受限倒车**（`backup_dist=0.45m`、`backup_speed=0.10m/s`、`time_allowance=10s`）。V0.0.89 移除的顾虑是“倒车把车倒进更差的致命栅格 + AMCL 位姿跳变”；V0.0.93 起全局定位改为 NDT 点云重配准（不依赖倒车运动模型），且 `safety_guard` 在指令为负时自动切换**后方走廊**判据（`footprint_rear+self_margin` 起算，后净空 < `stop_dist` 即零速），外加地图边界监护兜底；实车证明“纯非运动恢复”在“阿克曼 + 车头正对障碍”时**永远无法脱困**（V0.0.94 日志：车原地抖动 56s 零位移）。**配套改动**：`velocity_smoother.min_velocity[0]` 由 `0.0` → `-0.20`（该节点对全部速度源做绝对值钳制，置 0 会把倒车指令直接钳成 0 → “恢复行为报成功但车不倒”）；控制器/规划层的禁倒车由 MPPI `vx_min=0` + Smac `allow_reversing=false` 继续保证。**V0.0.97 升级——倒车能力下沉到控制器/规划器**：MPPI `vx_min 0.0→-0.20`（`PathAngleCritic` 据此自动由 `Reversing not allowed` 变 `Reversing allowed`）、`PreferForwardCritic.cost_weight 5.0→1.5`、`PathAngleCritic.reverse_penalty_weight 1.0→0.5`、新增 `vy_max: 0.0`；Smac `motion_model_for_search DUBIN→REEDS_SHEPP`（配 `reverse_penalty 1.5`/`change_penalty 0.3`）。原因：R_min 1.9m 的阿克曼绕开正前方障碍需**提前转向距离** `s ≥ √(2R·c) ≈ 1.33m`，车距障碍 <1.33m 时纯前进无解，必须“先退再转”或多点掉头（旧 `DUBIN` + `vx_min=0` 使正常路径不含倒车段，只能靠恢复行为倒 0.45m 后又被 1.0m 碰撞闸按停 → 满舵死磕至 FAULT）。⚠ 本 fork **`allow_reversing` 参数并不存在**（已核对 `nav2_smac_planner/src/smac_planner_hybrid.cpp` 只读 `motion_model_for_search/reverse_penalty/change_penalty`），倒车自由度只由 `motion_model_for_search` 决定。`Spin`（原地旋转）保持移除——阿克曼不能原地转向。
- ~~重规划提速~~（V0.0.85 1.0→2.0Hz，**V0.0.92 回退至 1.0Hz**）：实车复盘发现，2Hz 重规划在代价地图残留假障碍时会与脏图更新同频共振，导致控制器转向角全幅振荡（蛇形行驶）；1Hz + 起步清图同步门控（`clearCostmapsOnStart()`，V0.0.92）已足够覆盖动态障碍响应（safety_guard 物理碰撞闸 + MPPI 近障碍降速兜底），且不再放大感知噪声。**V0.0.94**：清图客户端类型由 `std_srvs/Empty` 修正为 Nav2 原生主类型 `nav2_msgs/ClearEntireCostmap`——CycloneDDS 下 `service_is_ready()` 的 graph 匹配只认主类型（`std_srvs/Empty` 仅序列化兼容副类型），误用 Empty 会导致清图门控永久 `global=PEND local=PEND`、goal 永不下发。

### 10.8 碰撞防护与安全约束（V0.0.85 新增 hunter_safety/safety_guard）

速度指令链最后一级物理安全闸，串联于 `velocity_smoother` 与底盘之间
（launch 重映射 cmd_vel_smoothed→cmd_vel_pre_safety，safety_guard 发布 /cmd_vel），
数据源为 /scan（pointcloud_to_laserscan 直出），**不依赖感知融合链**，与决策层
（auto_mission OBSTACLE_AVOID/ESTOP）、模式仲裁（decision_making）互为冗余：

| 能力 | 触发条件 | 动作 |
|------|----------|------|
| 碰撞急停（**V0.0.98 双判据**） | 行进中（\|v\|>0.03m/s）按**轨迹扫掠弧净空**、静止/蠕行按**直线走廊净空** < stop_dist(0.90m，**V0.0.97 由 1.0 重标定**，依据见 Deployment_Guide §5.5.1：必须 < 阿克曼绕障“提前转向距离”1.33m)；**V0.0.95 起带释放滞环**（恢复到 stop+0.25m 才放行） | 立即零速 COLLISION_STOP（日志标注净空来源「扫掠弧/直线走廊」） |
| 碰撞限速（**V0.0.98 双判据**） | 同上判据 < slow_dist(1.40m，**V0.0.97 由 1.8 重标定**，取“转向提前量 1.33m”量级，进入可转向区即已限速到 ≈0.2~0.3m/s——恰为绕障带速)；**V0.0.95 起带释放滞环**（恢复到 slow+0.20m 才全速） | 线性限速至 max×(d−stop)/(slow−stop)，SLOWDOWN |
| **轨迹扫掠弧碰撞闸（V0.0.98 核心）** | 行进中把指令 (v,w) 按阿克曼运动学积分真实轨迹（前 `reaction_lag` 0.4s 直线、其后圆弧 \|w\|≤\|v\|/1.9，弧长等步长 5cm、前瞻 ≤3m），车用包络盒（半宽 0.44m）逐步扫掠求首次接触，输出与走廊**同量纲**的纵向等价净空 | **绕得开就放行、真撞才拦**——修 V0.0.97 后“MPPI 能出绕障弧但被执行 0.1~0.3s 就被直线走廊闸按停”的安全/避障互斥死锁；⚠ 不能改回“弧/走廊取小”（取小即回到 V0.0.97 死锁）；静止/蠕行（≤`vel_trust_eps` 0.03）仍用走廊防抖，/scan 断流 fail-safe 回退走廊+SCAN_TIMEOUT |
| **阈值抖振抑制（V0.0.95）** | 净空在阈值附近微抖（实测 ±6mm：0.991↔1.006m） | 释放滞环使状态**不再每 0.1~0.2s 往返切换**（旧实现 56s 内 47 次 SLOWDOWN↔COLLISION_STOP，速度被反复归零 → 车“抖动不前进”+ 日志刷屏）；状态转移日志附带“滞环保持（释放阈值 X.XXm）” |
| **幽灵点门控（V0.0.95；V0.0.98 同弧生效）** | 判据源（走廊/弧）内最近回波纵向 ±`obstacle_cluster_span`(0.25m) 内回波点数 < `min_obstacle_points`(3)；弧判据下为接触后 0.2s×speed 弧长窗口内累计接触点 <3 | 判为孤立噪点（单束噪声/玻璃反光/雨雾），不作为刹车依据，2s 节流 WARN 输出诊断；真障碍必有数点以上回波（可调 2，设 1 即恢复旧行为） |
| 盲区一致性强制（V0.0.91；**V0.0.97 参数重标定**） | 配置的 stop_dist ≤ /scan `range_min`+0.15（急停区落在感知盲区内） | 运行时强制抬升到 range_min+0.15 并一次性 ERROR 告警修参数（撞墙事故根因：range_min 0.8 > stop_dist 0.5）。**V0.0.97：`range_min 0.8→0.70`、`stop_dist 1.0→0.90`（0.70+0.15=0.85 < 0.90 ✔）；⚠ `range_min` 不得 < 0.65——V0.0.88 实测车体自反射可达 ~0.6m，再降自反射会变成“永久障碍”把车钉住** |
| 感知 fail-safe | /scan 超时 0.5s 或未到达 | 零速（宁可停车不盲走）SCAN_TIMEOUT |
| 指令看门狗 | 上游速度指令断流 >0.5s | 零速心跳 CMD_TIMEOUT |
| 急停透传 | /estop=true | 零速（弥补 Nav2 goal 取消延迟窗口）ESTOP_PASS |
| 阿克曼曲率钳制 | 恒生效 | \|w\| ≤ \|v\|/1.9（δ≤0.24rad，杜绝打满转向） |
| 速度硬限 | 恒生效 | \|v\| ≤ 0.5m/s（第二重限速，V0.0.89） |
| 测试模式碰撞急停 | 测试模式开启时同走廊净空 < test_stop_dist(0.90m，**V0.0.97 与正式模式同源**，原 1.0) | 立即零速 COLLISION_STOP【测试模式】 |
| 测试模式减速 | 同走廊净空 < test_slow_dist(1.40m，**V0.0.97 与正式模式同源**，原 1.8) | 限速 ≤0.3m/s（test_max_linear_vel，V0.0.89） |
| 测试模式异常中止 | 疑似碰撞卡死（指令>0.05m/s 而反馈≈0 持续 1s）/ **连续** `test_max_goal_aborts`(3) 次 goal ABORTED（EXECUTING 会清零，V0.0.89）/ goal 活跃但 /cmd_vel_nav 断流 >`plan_fail_timeout`(10s) | 零速锁存 TEST_ABORTED + /estop=true + 取消全部导航目标 |
| 地图边界减速（V0.0.87） | 车辆距未建图(unknown)/界外栅格 < map_edge_slow_dist(1.0m，V0.0.89 原 1.5) | 线性限速至 max×(d−stop)/(slow−stop)，MAP_EDGE_SLOWDOWN |
| 地图边界停车（V0.0.87） | 车辆距未建图(unknown)/界外栅格 < map_edge_stop_dist(0.4m，V0.0.89 原 0.5) | 立即零速 MAP_EDGE_STOP（回到已建图区域自动恢复）；测试模式下升级为中止锁存 TEST_ABORTED |

> **地图边界约束（V0.0.87 三层防线）**：保证车辆行驶范围、预设航点、导航点与
> 规划路径均在已采集地图区域内——① Nav2 规划层：`global_costmap
> track_unknown_space: true` + Smac `allow_unknown: false`，全局规划路径不穿越
> 未采集区域；② 任务层：auto_mission 发送航点前校验（矩形边界+0.5m 边距+
> 非 unknown 栅格+**V0.0.95 已到达预检**），越界/已到达航点自动跳过；③ 执行层：safety_guard 地图边界监护
> （/map + /relocalization/pose 距离场，上表最后两行），行驶中越界零速兜底。建图模式
> 无 /map 与重定位位姿，③ 自动不介入。
>
> **V0.0.96 绕障/脱困能力补强（第二轮，对应“行驶一小段路立即停下、不再漫游”）**：
> ① **任务层三道预检**：已到达跳过 → 越界（边界/未建图）拒绝 → **V0.0.96 占据栅格 + 净空拒绝**
> （`waypoint_clearance_m` 0.50m，修“把车开进静态地图障碍的膨胀区”）；
> ② **行为层两条失败路径都可脱困**：`FollowPath` 失败序列（清局部图 + BackUp 0.45m + Wait）与
> **外层恢复池首位 BackUp 0.45m**（V0.0.96 新增，专治“起点格致命”类规划失败——清图/等待都无效，
> 唯一出路是物理驶离膨胀区）；
> ③ **安全层兜底不变**：倒车按后方走廊判据（后净空 <1.0m 零速）+ 地图边界监护；
> ④ **任务层不再误判**：ABORT 时若车已在 `already_reached_dist`(0.30m) 内 → 判为“已到达”，
> 不计失败（现场：车已到距目标 0.07m，却因“必须先规划成功才跑 FollowPath”被整树 ABORT）；受阻判据
> 改用“朝目标推进量”，倒车脱困不再被误判为“有进展”。
>
> ⚠ **架构已知限制（V0.0.96 记录）**：Nav2 局部代价地图为滚动窗口（odom 系）且插件仅
> `obstacle_layer + inflation_layer`，**不含 static_layer** ⇒ MPPI 只能避开“实时感知到”的障碍，
> 对“仅存在于静态 PGM 地图中”的障碍（建图期存在的物体/被遮挡结构/幽灵障碍）**不减速、不绕行**，
> 全局规划（Smac + 静态层）才是绕障权威。因此**航点必须落在净空 ≥0.5m 的自由区**（任务层已强制校验）。

分级预警：/safety/state（std_msgs/String）2Hz 心跳，格式 状态|原因；
仅导航模式启动（mapping 模式 auto_mission cruise 直发 /cmd_vel，避免双发布者）。

**自动驾驶测试模式（V0.0.86）**：低速实车联调专用。运行时开关：

```bash
ros2 topic pub --once /safety/test_mode std_msgs/msg/Bool "{data: true}"   # 开启
ros2 topic pub --once /safety/test_mode std_msgs/msg/Bool "{data: false}"  # 关闭（恢复常规阈值）
```

开启后 0.3m/s 限速巡航（V0.0.89，原 0.1 易导致“基本不动”）、前方安全走廊净空 <0.90m 急停 / <1.40m 减速（**V0.0.97 由 1.0/1.8 重标定**，见 Deployment_Guide §5.5.1）；
并自动监控四类异常——疑似碰撞卡死、控制器断流（>`plan_fail_timeout` 10s，V0.0.89 原 2s）、Nav2 goal ABORTED（V0.0.89 起**连续**达 `test_max_goal_aborts`(3) 次才触发，EXECUTING 清零，避免起步期瞬时 ABORT 一票否决）、
**地图越界（V0.0.87，距未建图/界外栅格 <0.4m）**——任一发生立即
零速锁存（TEST_ABORTED）并发布 /estop=true（auto_mission 取消全部导航任务）；
锁存后即使外部把 /estop 清回 false 也保持零速，必须重新发布 true 才能解除
（视为人工确认现场安全）。

地图边界监护（V0.0.87）运行时参数（launch 注入，默认全开；V0.0.89 窄场地适配）：
`enable_map_fence`（开关）、`map_edge_stop_dist`（0.4m 零速阈值）、
`map_edge_slow_dist`（1.0m 减速阈值）；监护在 /map 与 /relocalization/pose 均就绪后生效，
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
| 控制抖动 | 控制参数 / 延迟 | 调整 MPPI 参数（critics 权重/时间步）、检查控制频率 |
| 系统卡顿 | GPU / CPU / 温度 | `tegrastats` 查看资源、降温、降频 |
| 无法连平台 | 网络 / 证书 / Kafka | 检查 4G/WiFi、证书有效期、Kafka 配置 |
| 视觉节点每帧崩溃（`setSize` 断言） | OpenCV 4.10 / 4.5.4 **混链**（同进程两套 OpenCV，破坏 `cv::Mat` 不变量） | `ldd vision_perception_node \| grep opencv`：只允许 `so.410`、无 `4.5d`、无 `libcv_bridge`；异常时按 §6 专项重编（禁止在视觉进程重新引入 cv_bridge） |
| 视觉 0Hz，日志 `resize.cu:175 error (-217) no kernel image` | OpenCV CUDA 编译 ARCH 与实机 GPU 不符（如 sm_72 用于 Orin） | `cuobjdump --list-elf /usr/local/lib/libopencv_cudawarping.so.410` 应见 `sm_87`；按 §5.3 以 `CUDA_ARCH_BIN=8.7` 重编后重启节点即恢复 GPU（期间节点自动降级 CPU，感知不断流） |
| sensor_fusion 报 `TF unconnected trees` | 相机 frame 双父（驱动 TF + URDF 并存，TF 树分裂） | 确认 `hunter_full.launch.py` 相机驱动为 `publish_tf: 'false'`；`ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame` 验证外参；必要时 `tf2_tools view_frames` 看全树 |
| 启动时一次性 `彩色图像超时 x.x s` | 启动竞态（视觉节点激活早于彩色流就绪） | 仅出现一次属良性，可忽略；反复出现才按"相机无图像"排查 |
| **相机全程 0Hz**：`xioctl(VIDIOC_QBUF) failed: No such device` + `Failed to resolve the request: Z16 848x480`，`彩色图像超时` 持续递增、`health_monitor` 报 `camera 话题频率异常 0.0Hz`（V0.0.95 已修） | D435 驱动启动后先落默认 profile（depth/infra 848x480x30）再"停传感器→重开"，**重开瞬间 USB 设备节点消失**（ENODEV）→ 整机相机 0Hz，vision/fusion 退化为单雷达源 | ① V0.0.95 起 launch 已显式下发 `640,480,30`（避免默认 profile 触发的 stop/start 重配）并关闭 infra；② 仍复现按硬件链排查：`lsusb`、`dmesg \| grep -iE 'usb\|uvc\|xhci'`（找 disconnect/reset）、D435 直连 USB3 口勿经 HUB、关闭 USB 自动挂起；③ 设备枚举异常（`/dev/video*` 消失）时把 `hunter_full.launch.py` 相机 `initial_reset` 改 `'true'` 重启；④ 验证 `ros2 topic hz /camera/camera/color/image_raw`（应 ~30Hz） |
| **自主巡航车辆原地抖动不前进**：`/safety/state` 在 `SLOWDOWN↔COLLISION_STOP` 间高频往返、`set steering angle` 恒为 ±0.386428、最终 `Failed to make progress`（V0.0.95 已修） | ① 航点与车辆当前位姿重合（"目标=自身"退化 goal，阿克曼无法原地转向）；② 走廊净空贴着急停阈值 ±6mm 抖振，速度被反复归零；③ BT 恢复池只有"清图+Wait"非运动手段，无法脱困 | V0.0.95 已分层修复（航点"已到达"预检 + 阈值释放滞环 + 受限倒车脱困 + 受阻检测）；现场仍复现时：① 确认车头前方 ≥2m 无障碍（`rviz2` 看 /scan 与 costmap，分清真实障碍/幽灵点）；② 看 `safety_guard` 启动日志确认 `释放滞环=+0.25/+0.20m`、`幽灵点门控=≥3 点` 已注入；③ 确认 `velocity_smoother min_velocity[0]=-0.20`（=0 会把倒车脱困钳成 0）；④ `waypoints` 首点不得与车位重合（见 `autonomous_nav_params.yaml` 航点布置约束） |
| **自主巡航报 `[NAVIGATING] 航点[i] 受阻` 或 FAULT 锁存** | 前方真实障碍无法绕行 / 航点在障碍后无可达路径 / 全部航点与车位重合 | 先看 `/safety/state` 的走廊净空与 `幽灵点抑制` 告警区分真实障碍与噪点；移除障碍或人工把车移到空旷处；用 rviz2 重新标定航点；FAULT 需把模式开关离开 AUTO 再切回解除 |
| **倒车脱困"报成功但车不倒"** | `velocity_smoother` 对全部速度源做绝对值钳制，`min_velocity[0]=0.0` 把负线速度钳成 0（V0.0.94 及以前默认） | `nav2_params.yaml` `velocity_smoother.min_velocity` 应为 `[-0.20, 0.0, -0.8]`（V0.0.95）；禁倒车由 MPPI `vx_min=0` + Smac `allow_reversing=false` 保证，不在平滑器上设 0 |
| **行驶一小段后停下，`planner_server` 持续报 `Starting point in lethal space! Cannot create feasible plan..`，清图/等待均无效，3 次后 `FAULT`（V0.0.96 已修）** | 车辆停在**静态地图障碍的膨胀区**内（起点格代价 LETHAL 254 / INSCRIBED 253）→ Smac 的 `areInputsValid()` 判起点无效；清图无效（本 fork `StaticLayer::reset()` 只置 `has_updated_data_`，静态障碍会重新盖章）；**滚动局部代价地图不含 static_layer**，MPPI 不会避开静态地图障碍，因而会把车一路开到那里 | V0.0.96 三层修复：① 任务层航点净空校验（`waypoint_clearance_m` 0.50m，占据栅格 + 净空双判，发送前拒绝并打印实测净空）；② 行为树恢复池**首位 BackUp 0.45m**（规划失败也能物理驶离膨胀区，1~2s 内生效）；③ 任务层“ABORT 但已在到达半径内 ⇒ 判为已到达”。**现场恢复手段**：`ros2 run teleop_twist_keyboard` 人工把车倒出膨胀区，或 rviz2 确认航点位置后重标 `waypoints` |
| **自主巡航报"距静态地图障碍仅 x.xxm < 净空要求 0.50m（处于 Nav2 膨胀/致命区内）"并跳过该航点** | 航点标定在障碍旁/障碍上（仓库示例航点 (0,0)/(5,0)/(5,3)/(0,3) 为占位值，实测 (5.0,0.0) 不满足净空） | 设计行为（防止把车开进死局）：在 rviz2 中确认目标点四周 ≥0.5m 无占据（黑色）栅格，按 `autonomous_nav_params.yaml` 的标定步骤重标；全部航点被拒会 FAULT 锁存（模式开关离开 AUTO 再切回解锁） |
| **相机仍 0Hz（V0.0.96 已将 `initial_reset` 置 true 仍复现）** | 属 USB 链路级故障（供电/带宽/接触/枚举异常），非驱动参数问题 | 按 §13.8 相机条目硬件排查：D435 直连 USB3 口（勿经 HUB）、`dmesg \| grep -iE 'usb\|uvc'` 查掉线、关 USB 自动挂起、必要时更换线缆/接口。导航不受阻（`health=WARNING` 不拦 AUTO 门控），但视觉避障退化为单雷达源 |
| **日志被 `set steering angle: x` 刷屏（20~50Hz）** | 底盘驱动（`hunter_ros2/hunter_base`，vendor 目录）在每个 `/cmd_vel` 回调 `std::cout` 打印转向角，未节流 | 分析时过滤：`grep -v 'set steering angle' /tmp/hunt7.log`；或 `scripts/hunter_log.sh` 导出后离线过滤。驱动属 vendor 代码（git-ignored），不建议直接改 |
| 一次性 `[TensorRT] Using an engine plan file across different models of devices` | `.engine` 非本机/本设备型号生成（换机或文件被旧引擎覆盖） | 不阻塞运行（话题 15Hz 正常）；目标机重生成：`trtexec --onnx=<绝对路径>/yolov8s.onnx --saveEngine=/data/models/yolov8s.engine --fp16` 后重启视觉节点 |
| **已升 V0.0.97 后仍绕不开障碍：不再满舵死磕，但带转向的绕障轨迹每执行 0.1~0.3s 即被 `COLLISION_STOP（行进方向走廊净空 …）` 清零，车在 stop/slow 间往复抖振（V0.0.98 已修）** | 旧碰撞闸把 (v,w) 只按行进方向直线投影求净空，忽略 w 的横移避让分量——绕障弧起点必落在走廊内，安全层反过来封死避障层 | 升级 `hunter_safety`（轨迹扫掠弧碰撞闸，README §10.8 / Deployment_Guide §5.5.1(5)）+ `hunter_bringup`（MPPI 4.0s 时域）；验收：启动日志 `V0.0.98 轨迹扫掠弧=行进中启用`、行进中拦停措辞「扫掠弧净空」；配套 `auto_mission` 取消静置门控消除换点误判 FAULT |
| Ctrl+C 后 `maps/` 只有 `.pcd`，`.pgm/.yaml` 未生成（V0.0.88 前必现） | FAST-LIO2 在 `main()` 于 `spin` 返回**后**才写 PCD（20.7M 点 ≈ 664MB 需数秒至数十秒），而 Ctrl+C 同时终止 `pcd_to_map`，运行期 `MAPPING→非MAPPING` 跳变不会发生 → 原自动转换从不启动 | V0.0.88 起 `pcd_to_map` 退出时派生独立会话后台转换进程兜底：`tail -f maps/pcd_to_map_final.log`（应见 `PCD 已写完整 → 转换成功`），数十秒内 `ls -lh maps/` 应齐 `.pcd/.pgm/.yaml`；仍缺时手动兜底 `python3 ~/HunterEdge/install/auto_mission/lib/auto_mission/pcd_to_map --finalize --pcd-file ~/HunterEdge/maps/hunter_map.pcd --force` |

---

## 14. 文档索引

关联文档如下：

| 文档 | 说明 |
|------|------|
| 《自动驾驶车辆系统详细设计文档 V2.0》 | 本项目的设计基准；本文档全部参数、话题、CAN 协议、坐标系均可追溯至其对应章节 |
| AI 编码任务清单 | 分模块开发任务（任务 00 ~ 任务 17），指导按模块开发与验收 |
| `User_Manual.md` | 面向现场运维人员的用户手册（独立文档，含详细部署/联调/故障排查/自主导航操作流程，**V2.1** 对应软件基线 V0.0.99） |
| `Deployment_Guide.md` | 部署操作文档 V2.1（环境要求/环境配置/环境安装/源码部署/功能操作步骤/异常处理全流程，对应软件基线 V0.0.99） |
| `release.md` | 版本历史（V0.0.1 ~ 当前），记录每版主要功能与修复 |

> **追溯原则**：本 README 中所有硬件参数（§2）、软件版本（§3）、话题（§8）、控制模式（§9）、限制（§13）均源自《自动驾驶车辆系统详细设计文档 V2.0》，未虚构功能。自主导航模块（§10）为在设计文档框架内的扩展实现。

---

*HunterEdge 开发指南 · 文档版本 V2.1 · 编制依据《自动驾驶车辆系统详细设计文档 V2.0》，并含 V0.0.67~V0.0.99 现场实测修正（V0.0.93：方案A 定位架构重构；V0.0.94：“原地不动”残余故障链修复；V0.0.95：“无法绕开障碍物”分层修复；V0.0.96：“行驶一小段立即停下”修复；V0.0.97：阿克曼绕障几何死锁 + 过期 goal 修复；V0.0.98：“V0.0.97 后仍无法绕开障碍物”第二层根因修复——safety_guard 轨迹扫掠弧碰撞闸、MPPI 4.0s 预测时域与 critic 重标定、auto_mission 取消静置门控、/scan 链降载；**V0.0.99：阿克曼几何参数一致性修正——safety_guard 轴距 0.65→0.46（与 README 硬件表/decision_making 统一，δ_max 18.9°→13.6°，非功能性）+ 独立调试启动 safety_guard.launch.py 与生产同步（修正滞留的 stop_dist=1.0 死锁值）**；重编 `hunter_safety hunter_bringup`）*

