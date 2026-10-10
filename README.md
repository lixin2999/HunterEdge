# HunterEdge 自动驾驶车载系统 — 开发指南

> **项目**：HunterEdge 自动驾驶车载系统
> **文档版本**：V2.14（开发指南，对应软件基线 **V0.1.12**：**V0.1.12 OTA 启动自检伪报回滚修复（pending_selftest 标记）+ hunter-edge.service 停止语义与 SHM 残留自清**——① `ota_agent.py run()` 旧行为下无论是否有升级任务只要 `self_test_on_start=true` 就会进自检，失败直接 `self.rollback()` + 上报 `ROLLBACK`；普通开机时因 `/opt/hunter/version` 未写入 / ROS daemon 未起 / 单次 candump 对 can2 接口拉起与首帧 0x211 存在竞态而“瞬时失败”，现场日志里反复出现 `自检 can: FAIL → 无可用备份、回滚失败 → 状态上报：ROLLBACK`。新行为：`handle_task` 安装成功后写 `/data/ota/.pending_selftest.json`（含 `task_id`/`target_version`），`run()` 以标记存在为门控才走 self_test+rollback；无标记⇒**一行 log 即返 IDLE**，不报 ROLLBACK。`_can_ok` 1×1s → **3×2s**（中间 0.5s），无 candump 仍降级 True；`_version_ok` 优先取 `self.task.version`，次优取 pending 标记里的 `target_version`，两者均无时直接 True。② `ota-agent.service` `After=network-online.target hunter-can.service`（不使 `Requires`）；`hunter-edge.service` `KillMode=mixed → control-group`，`TimeoutStopSec=120 → 60`，新增 `ExecStopPost` 自清 `/dev/shm/fastrtps_*` `sem.fastrtps_*` `CycloneDDS.*` `sem.CycloneDDS.*`（双保险、兼容 FastDDS 回退）。③ 新参数 `use_pending_marker: true` / `pending_marker_file: "/data/ota/.pending_selftest.json"` 写入 `ota_agent_params.yaml` 与 DEFAULT_CONFIG；`false` 可临时回旧行为（不推荐）。验收口径：`journalctl -u ota-agent | grep 启动自检` 应只看到“无 pending_selftest 标记 → 普通开机，跳过”；`systemctl show hunter-edge -p ExecStopPost` 应含 `rm -f /dev/shm/fastrtps_*`；上一版 V0.1.11 `command_agent` 成员名 `self._clients` 撞 rclpy.Node 内部字段——`Node.__init__` 建的 `self._clients: List[Client]` 被业务代码覆盖成 `Dict[str, Client]`，executor 走 `node.clients` 时 `list(dict)` 迭代得到键（str），对 str 调 `entity._executor_event` → `AttributeError`；`destroy_node()` 又 `self._clients[0]` 抛 `KeyError: 0`。修复仅重命名→`self._srv_clients`（声明/get/sorted 三处），**不改行为**，只重编 `command_agent`。写法约定：子类化 `Node` 时一律避开 rclpy 内部名 `_clients/_services/_publishers/_subscriptions/_timers/_guards`；上一版 V0.1.10 投递证实改 `on_delivery` 包装 + rclpy 空数组参数容错 + 服务停止语义**——① 本库 confluent-kafka **2.16.0 没有 `dr_cb` 回调 kwarg**（本机同版本逐写法实测：`Producer(conf, dr_cb=cb)`、`{**conf,"dr_cb":cb}`、属性赋值全部失败；只有 `error_cb`/`stats_cb`/`throttle_cb`/`logger` 可用）→ 改用 `_ProducerWithDeliveryReport` 包装类在 `produce()` 上挂 `on_delivery`，`ota_agent`/`command_agent` 不再 `_INVALID_ARG`；② 现场 YAML 的 `extra_allowed_types: []` 在 rclpy 下会落成"未初始化"（`ParameterUninitializedException: ... is not initialized`）→ 新增 `_param_string_list()` 容错；③ 两个 unit 加 `KillSignal=SIGINT`+`KillMode=mixed`+`TimeoutStopSec=20`，消除 `remote-agent` 停止超时被 SIGKILL（`Failed with result 'timeout'`）。附 11 项回归断言（含**真实 2.16.0 API 冒烟**）；上一版 V0.1.09：**V0.1.09 三处阻塞修复（`dr_cb` 传参 / rclpy 日志参数 / 消费者配置噪声）**——① `hunter_kafka.make_producer()` 把 `dr_cb` 塞进配置字典（librdkafka **不认**，要求构造参数）→ `command_agent`/`ota_agent` 的 `_init_kafka()` 启动即崩 `_INVALID_ARG`；本版改为 `Producer(conf, dr_cb=cb)`（`hunter-kafka-check` 不传回调，故自检全绿也照不出这个坑）；② `command_agent.py` **17 处** `get_logger().fatal("...%s", exc)` 用了 printf 风格位置参数，而本 rclpy 版本 `RcutilsLogger` **只接受单 message** → `TypeError`（最致命的是 `except` 分支自身的日志把 `dr_cb` 真因盖掉）；本版全部改为 `%` 预格式化；③ 消费者 conf 混入生产者键导致 librdkafka 逐条 CONFWARN，按 `role` 分流去除；④ `websocket-client` 缺失时的每 5s ERROR 改为只提示一次并补进部署脚本依赖。新增 `dr_cb`/配置回归测试（14 项断言）；上一版 V0.1.08：**V0.1.08 systemd 可执行路径修复（隔离安装布局）**——`ota-agent`/`remote-agent` 两个 unit 按 `--merge-install` 布局拼路径（`<install>/lib/<pkg>/<pkg>_node`），而本工作空间是 colcon 默认的**隔离安装**（`<install>/<pkg>/lib/<pkg>/<pkg>_node`）→ 实机 `status=127 No such file or directory`、每 10s 重启一轮（`is-active` 显示 `activating`）；本版改为「隔离安装 → 合并安装 → 源码树」三候选探测 + 失败时打印候选路径与处置命令，并补 `hunter_edge_up.sh` 的自检入口候选；新增"抽取 unit 内层 bash 命令跑 `bash -n`"自检（6 处全 PASS）。**排障要点：127 先看路径，不看业务**（详见 §13.10 与 release.md V0.1.08）；上一版 V0.1.07：**V0.1.07 车端 Agent「启动崩溃」修复 + 事件等级/字段/风暴与运营端契约全面对齐**——① `command_agent` **自 V0.1.05 起从未成功运行**（`__init__` 的 `create_subscription` 引用了四个类里不存在的回调 → 启动即 `AttributeError`），本版补齐，平台指令链首次真正上线；② `ota_agent` 恒 `activating`（`main()` 漏 `args = parser.parse_args()` → `NameError` → 重启循环），本版补齐；③ 事件等级按契约 `enums.md` §3 校正（`over_speed`/`communication_loss` 必须为 `critical`）；④ **事件风暴 + 磁盘风险**修复（`detectEvents` 100ms 无状态判定 → 每秒约 10 条重复事件且每条拉起一个 `ros2 bag record`；本版加边沿触发抑制期 `event_min_interval`，默认 10s）；⑤ 契约字段三处对齐（`ota_status` 改 `{status, phase, progress}` 且无 `task_id` 不上报；遥控事件改 `{event_type, event_level}` 且只上报受控词表类型；遥控帧改读 `control.target_velocity/target_steer` + `reason`/`heartbeat` 会话结束帧）；新增 AST/ruff/降级冒烟三重静态防复发（§13.10、release.md V0.1.07）；上一版 V0.1.06：**V0.1.06 上电自启链路 + Kafka 消息体契约对齐**——① **上电自动启用底盘 CAN**（新增 `hunter_can_up.sh` + `hunter-can.service`：等待 USB-CAN 枚举 → `can2 down / type can bitrate 500000 / up` → `candump` 抓底盘反馈帧 0x211/0x221 才算通过；退出码 2=无接口、3=无底盘帧）；② **CAN 就绪后自动启动 HunterEdge 并进入自动驾驶准备状态**（新增 `hunter_edge_up.sh` + `hunter-edge.service` + `hunter_edge.launch.py`，`Requires/After=hunter-can` 严格时序，生产默认 `use_autonomous_nav=true`）；③ **车端与运营端建立连接并按契约上传数据**——`data_agent` 上行 JSON **逐字段对齐** HunterCore `contracts/kafka/schemas/*.schema.json`（telemetry 六段嵌套 + `seq`；health 8 态 `status` + `system{}` + `free_storage_mb`；event 契约 `event_type`/`event_level`/`description`/`data`），新增 `communication_loss` 中断事件与 `system.network_latency_ms` 实测（旧版平铺字段会被平台 `schema_name="auto"` 判非法 → DLQ，这正是**运营端 0 行**的直接原因）；④ **平台自动驾驶指令 → 进入相应模式**——`command_agent` 改读契约 `command_type/timestamp/params/timeout_ms`（兼容旧 `type/issued_at/payload`）、映射 `enable_auto_driving → MISSION_START`（别名表可配），回执改为契约 `{command_id, vehicle_id, timestamp, success, result_code, message, data}`；`hunter_core_setup.sh` 步骤⑥ 扩到 4 个 systemd 单元（新增 `--no-start`）；`hunter_status.sh` 新增「上电自启链路」与「运营端上行（车端侧自证）」两段，详见 §7.8 / §7.9 / §13.10；上一版 V0.1.05：**V0.1.05 HunterCore 车端接入全链打通**——把接入方式从“自填 YAML 参数”改为“**以平台下发的接入包为唯一可信源**”（`/etc/hunter/kafka/kafka.properties` + CA/客户端证书，`SASL_SSL + SCRAM-SHA-512` 上叠加 **mTLS**，主机名校验 `https`）；补齐 8 Topic 契约（`hunter.<vehicle_id>.<type>`）中此前沿未打通的四条断链：`health`（1Hz）、`command`/`command_result`（**新增 `command_agent` 包**）、`remote_control`（`remote_agent` 改为 Kafka 消费驱动，并修复“遥控超时后永不交还 AUTO”缺陷）；`acks` 按 Topic 分档（telemetry/health=1，event/command_result/ota_status=all）、SQLite 缓存加 `topic` 列做**三类消息统一断点续传**（投递证实才删行）、消费端**手动提交 offset**、OTA 按 `task_id` **幂等**；四份 `*_params.yaml` **凭据出仓**；新增 `hunter_kafka` 公共库（properties → librdkafka 映射）+ `hunter-kafka-check` 六层自检 CLI + `hunter_core_setup.sh` 一键部署（详见 §11.3、§7.7 与 Deployment_Guide §5.6）；上一版 V0.1.04**：V0.1.04 车端 Kafka 安全认证配置落地**——完成与数据采集分析系统对接，Broker `120.202.73.105:9093`（SASL_SSL + SCRAM-SHA-512），统一配置到 data_agent/remote_agent/ota_agent 三个模块；上一版 V0.1.03**：V0.1.03 修复「开启自主导航、刚起步车辆即停下、无法进入正常自动驾驶」**——NDT 重定位「跟踪/置信度」彻底解耦：位姿刷新只由跳变闸门 `step_ok` 决定，`fitness_hard_ceiling` 降级为仅调节对外置信度、不再冻结位姿（旧逻辑车一动 fit 越过 ceiling 即冻结 map→odom → 越走越偏 → fit 发散锁死的死亡螺旋），并将 `fitness_max 2.5→3.5`、`fitness_hard_ceiling 3.0→5.0` 容纳稀疏地图行驶期 fit 基线，仅重编 `hunter_relocalization`（根因缓解：行驶期 fit 基线偏高源于全局地图过稀，必要时重建更密地图）；上一版 V0.1.02**：《HUNTER SE 低速自动驾驶避障解决方案》逐条对表落地**——对表审计确认方案的避障主链（扫掠弧闸/REEDS_SHEPP/MPPI 4.0s/自愈四件套/BT/五级预检）**已全部在位**，本轮只补齐 5 处量化参数：① 代价地图尺度（全局 `inflation_radius 0.40→0.55`、全局 `update_frequency 2.0→1.0`、局部 `update_frequency 10.0→5.0` + 窗口 `10m→6m`，局部图算力降至 ≈1/5.6）；② 感知（`lidar_perception` 新增可配 `ground_max_slope 5.0°`、`outlier_mean_k 10→50`、`cluster_tolerance 0.5→0.15`；`sensor_fusion.vision_conf_min 0.45→0.50`）；③ `safety_guard.max_linear_vel` 默认值 `0.8→0.5`；并把 6 项**有意保留的偏差**写明理由（对照全表见 Deployment_Guide §5.5.2、避障 10 项验收见 §5.7、本节 §10.10）。上一版 V0.1.01：**health_monitor「相机崩溃/重启风暴」误报修复**——频率看门狗把 15Hz 标称帧率在系统过载下的正常抖动误判为进程崩溃（`checkNodes` 不 respawn 任何进程，仅按相机话题频率做异常-恢复边沿计数）：`camera_min_rate` 10→5Hz + 措辞去误导（“疑似崩溃/重启”→“频率异常/恢复”，逻辑与对外语义不变），重编 `hunter_monitor`，详见 release.md V0.1.01。上一版 V0.1.00：实车日志「`[FAULT] 任务已锁存（单航点导航超时连续达上限）……处置后将模式开关离开 AUTO 再切回以重启任务`」的彻底修复——`auto_mission` 任务层「**自愈四件套**」：① **逐航点失败隔离 + 自动轮转**（失败计数由「跨航点总量」下沉为「单个航点」，超时/受阻 → 放弃该点 + 清两张代价地图（重新规划）+ 改发下一个可用航点，隔离冷却 120s 后自动重试；任务级 `max_consec_failures`(12) 才兜底进 FAULT）；② **车身四周/脚底「假障碍占位」自动诊断**（`/local_costmap/costmap` × `/perception/lidar_objects` × `/perception/fused_objects`（含仅相机确认的目标）三路交叉：代价地图在车身框±35cm 内占位而两路传感器均无实体 ⇒ 判【假障碍】并自动清图，不再需要人拿 rviz2 看图）；③ **航点可达性按地图范围判定**（5/7 chamfer 净空场 + 8 邻域可通行连通域 BFS，与车位不在同一连通域 ⇒ 永久隔离该点并继续跑其他航点，按冷却周期自动复判）；④ **FAULT 由「永久锁存等人工解锁」改为自愈态**（静置 20s → 诊断 + 清图 + 全航点复判 + 三重门控 → 自动回 IDLE 重启；**遥控接管后切回 AUTO 即自动复驶**）。历史：V0.0.99（阿克曼几何参数一致性修正）、V0.0.98（safety_guard 轨迹扫掠弧碰撞闸 + MPPI 4.0s 时域 + 取消静置门控）、V0.0.97（绕障几何 + goal 世代号）、V0.0.96（航点净空校验、恢复池倒车）修复均已实车验证生效）
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
| `hunter_agents/data_agent` | 数据采集上传：Kafka `telemetry`(10Hz)/`event`/`health`(1Hz) + SQLite 断点续传 + MinIO | §14 |
| `hunter_agents/command_agent` | **V0.1.05 新增**：消费平台指令 `hunter.<vid>.command` → 复用既有 ROS 服务/话题执行 → 回执 `command_result`（白名单/TTL/幂等/超时护栏） | §14（扩展） |
| `hunter_agents/ota_agent` | OTA 升级（systemd 服务，消费 `ota_notify`、上报 `ota_status`） | §12 |
| `hunter_agents/remote_agent` | 远程操控（WebRTC + Kafka `remote_control` 消费驱动，systemd 服务） | §13 |
| `hunter_common/hunter_kafka` | **V0.1.05 新增**：HunterCore Kafka 接入公共库——`/etc/hunter/kafka/kafka.properties` 单一可信源 → librdkafka 配置映射（SASL_SSL + SCRAM-SHA-512 + mTLS）、Topic 命名、acks 分档、手动提交 Consumer、`hunter-kafka-check` 自检 CLI | §12/§13/§14 |
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
sudo apt install -y librdkafka-dev librdkafka++1 libsqlite3-dev libsasl2-dev libssl-dev libsasl2-modules   # data_agent/command_agent（Kafka SASL_SSL + mTLS + SQLite 缓存；libsasl2-modules 含 SCRAM 插件，实测属主包）
pip3 install confluent-kafka            # Python Agent（ota_agent / remote_agent / command_agent / hunter_kafka）
```

> ⚠️ **【SCRAM 机制必装（V0.1.05）】** 接入口要求的 `SCRAM-SHA-512` 由 Cyrus SASL 插件 `libscram.so` 提供，**缺它必报 `No worthy mechs found`**，表象极像口令错或网络不通，是现场最易误判的一条（详见 §7.7、Deployment_Guide §5.6.3）。
> **包名坑（jammy 实测）**：Ubuntu/Debian **没有** `cyrus-sasl-scram` 这个包（那是 RHEL/openSUSE 的名字，照它装会直接 `Unable to locate package`）。
> **属主包（HUNTER-001 实机定论）**：`dpkg -S /usr/lib/aarch64-linux-gnu/sasl2/libscram.so` → **`libsasl2-modules:arm64`**（不是先前根据官方 amd64 清单推的 `libsasl2-modules-gssapi-mit`）；但判据一律以**插件文件在不在**为准，包名只当处置提示。
> **查法与陷阱**：判可用性一律用 `ls /usr/lib/*/sasl2/libscram.so`（**单模式查**：写成 `ls A B` 时任一操作数不存在就返回非 0，会把“已装”误判为“缺失”），属主包用 `dpkg -S <那个文件>` 现查（`dpkg -L` 与 `dpkg -S` 可能不一致，以 `dpkg -S` 为准）。**同名陷阱**：Ubuntu 里有个叫 `scram` 的包，它是概率风险分析工具，与 SASL 无关，别拿它当替代方案。上述依赖与证书落盘可由 `scripts/hunter_core_setup.sh` 一次性完成（它按插件文件在不在来决定装什么，并把实机的属主包直接打到日志里）。

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
| `use_data_agent` | `true` | 是否启动数据采集 Agent（`telemetry`/`event`/`health` 上云） |
| `use_command_agent` | `true` | 是否启动平台指令接入 Agent（消费 `command`、回执 `command_result`；无接入包时会告警降级，不影响其余模块） |
| `use_autonomous_nav` | `false` | 是否启动自主导航全栈（建图/巡航） |
| `autonomous_nav_mode` | `nav` | `nav`=导航巡航模式，`mapping`=建图模式 |
| `map_yaml_path` | `/home/agilex/HunterEdge/maps/hunter_map.yaml` | 导航模式地图 YAML 路径 |
| `map_file_path` | `/home/agilex/HunterEdge/maps/hunter_map.pcd` | 建图模式 PCD 保存路径 |

### 7.8 上电自动启动（生产运行方式，V0.1.06）→ 详见 §13.10

生产现场**不再手工敲命令**：上电后由 systemd 按依赖链自动拉起，四个需求一次落地：

```bash
# 上电即自动执行（hunter-can.service → hunter-edge.service）
#   ① 底盘 CAN：ip link set can2 down/type can bitrate 500000/up + candump 抓帧验证
#   ② HunterEdge 整栈启动 → 车辆进入自动驾驶准备状态（定位/感知/Nav2/auto_mission 待命）
#   ③ 车端与运营端建立连接：telemetry 10Hz / health 1Hz / event 事件即发
#   ④ 收到平台自动驾驶指令 → 进入相应模式，并按 Kafka 消息格式回传 command_result

systemctl status hunter-can hunter-edge        # 开机链路状态
journalctl -u hunter-can -u hunter-edge -b -f  # 开机时序日志

# 现场手工走同一条路径（排查用；等价于开机自启）
bash src/hunter_bringup/scripts/hunter_edge_up.sh
# 仅复跑 CAN 上电与抓帧验证（退出码 0/1/2/3）
sudo bash src/hunter_bringup/scripts/hunter_can_up.sh

# 建图模式上电自启：改 /etc/hunter/agent_env.sh 的 HUNTER_EDGE_NAV_MODE=mapping
sudo systemctl restart hunter-edge
```

> ⚠ 顺序即安全：`hunter-edge.service` 用 `Requires=hunter-can.service` +
> `After=hunter-can.service network-online.target` 声明依赖——**底盘 CAN 通信没起来，
> 整栈不会启动**（避免把车挂到“AUTO 待命”却连不上底盘）。CAN 服务为 `Type=oneshot`
> +`RemainAfterExit`，上电后适配器掉线可用 `sudo systemctl reload hunter-can` 重新拉起。

### 7.9 平台指令接入（需求④：接收自动驾驶指令并进入相应模式，V0.1.06）

平台下发到 `hunter.<vehicle_id>.command` 的指令由 `command_agent` 消费，落地仍复用既有
ROS 服务/话题（**不新增任何话题/消息**），并按契约回执 `command_result`：

| 平台 `command_type` | 车端动作（内部名） | 说明 |
|---|---|---|
| `enable_auto_driving` | `MISSION_START` | 契约示例（`command.schema.json` examples[0]）→ 启动自主任务，进入自动驾驶 |
| `disable_auto_driving` | `MISSION_STOP` | 退出自动驾驶（停止自主任务） |
| `MISSION_START` / `MISSION_STOP` / `MAP_CONVERT` / `WAYPOINT_SAVE` / `WAYPOINT_CLEAR` / `ESTOP_ENGAGE` / `ESTOP_RELEASE` / `TEST_MODE_ON` / `TEST_MODE_OFF` / `REMOTE_RELEASE` / `STATUS_QUERY` / `HEARTBEAT_ACK` | 同名 | 车端内部动作名，保持可用 |

> ⚠ 平台侧 `command_type` 取值域**尚未定稿**（HunterCore 契约标 pending #17），
> 因此映射表做成**配置**（`command_agent_params.yaml` 的 `command_type_aliases`，
> 形态 `["平台类型=车端动作", ...]`）：定稿后只改 YAML、不动代码。

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

### 7.7 HunterCore 车端接入部署与自检（V0.1.05）

车端与 HunterCore 平台的**全部**数据交互以平台按车下发的**接入包**为唯一可信源（`kafka.properties` + `ca-cert.pem` + `client-cert.pem` + `client-key.pem`），**仓内 YAML 不得出现任何凭据**。接入前提：§5.2 依赖已装齐（尤其 SCRAM 插件 `libscram.so`——以 `ls /usr/lib/*/sasl2/libscram.so` 为准，属主包随架构不同）、车-平台网络可达、系统**已校时**（时间漂移会让 TLS 必失败）。

```bash
# 前置：接入包不在仓库内，需先从开发机拷到车上（scp / U 盘），否则第一步就报“接入包目录不存在”
ls -l ~/HUNTER-001-bundle/HUNTER-001      # 应见 kafka.properties + 三个 .pem

# 一键部署：依赖 → 接入包落盘(/etc/hunter/kafka) → 参数落盘(/etc/hunter) → 编译 → 自检 → systemd
cd ~/HunterEdge
sudo bash src/hunter_bringup/scripts/hunter_core_setup.sh \
  --bundle ~/HUNTER-001-bundle/HUNTER-001
# --user / --ws 一般不写：默认取 sudo 调用者与其家目录下的 HunterEdge（显式写则必须是车上真实存在的用户）
# 云端不可达时先只查本地配置/证书：追加 --offline
# 常用开关：--skip-deps / --skip-build / --no-systemd / --force-config / --force-key

# 改完 SCRAM 口令或 /etc/hunter/*.yaml 后的重跑：**不需要再传 --bundle**
# （/etc/hunter/kafka 四件套齐时脚本自动复用现场凭据并跳过步骤②，不碰已部署的凭据）
sudo bash src/hunter_bringup/scripts/hunter_core_setup.sh --skip-deps
```

> ⚠️ **【口令写入是人工动作】** 接入包模板里 `sasl.jaas.config` 的 `password="<SCRAM_PASSWORD>"` 需由运营/现场负责人**手工**换为平台分配的真实口令；脚本与日志均不经手该值。未替换时 `hunter-kafka-check` 会直接失败（退出码 10），**不要当网络问题排**。**写入口令后可放心重跑部署脚本**：包内仍是占位符时，步骤② 保留现场已写入真值的 `kafka.properties`（仅用 `grep -q` 判占位串，不输出内容），要强制用包内容覆盖才加 `--force-config`。

部署完成后，`ros2 launch hunter_bringup hunter_full.launch.py` 会随栈拉起 `data_agent` + `command_agent`；`ota-agent` / `remote-agent` 为 systemd 服务（由脚本步骤⑥ 安装并 `enable`）。日常核验：

```bash
source ~/HunterEdge/install/setup.bash
hunter-kafka-check                # 全量七层自检（含 SASL 机制插件层与端到端投递证实）
hunter-kafka-check --offline      # 仅本地配置/证书/机制层
./src/hunter_bringup/scripts/hunter_status.sh   # 第 8 段输出「HunterCore 接入」状态
```

> ⚠️ **【`hunter-kafka-check: command not found`（退 127）是正常的，直到你跑过一次部署脚本】** ament **只把 `<prefix>/bin` 加进 PATH**，而 ament_python 的 `console_script` 实际装在 `<prefix>/lib/<pkg>/` 下（本仓实测：`install/hunter_kafka/lib/hunter_kafka/hunter-kafka-check`），所以**光 `source install/setup.bash` 并不会让这个命令可用**。两种解法：
> ① 跑一次 `hunter_core_setup.sh`（步骤⑤ 会往运行用户的 `~/.local/bin/` 装一个不依赖包元数据的命令包装，**新开一个 shell** 后 `hunter-kafka-check` 直接可用；未重登则用 `~/.local/bin/hunter-kafka-check`）；
> ② 不依赖任何安装、现在就能跑的等价命令：`cd ~/HunterEdge && PYTHONPATH=src/hunter_common/hunter_kafka python3 -m hunter_kafka.diagnose`（可自行 `alias` 成短名）。

> ⚠️ **【报 `PackageNotFoundError: No package metadata was found for hunter-kafka`】** 这是直接拿**绝对路径**跑 setuptools 生成的入口脚本所致（`load_entry_point` 包装需要包元数据在 `PYTHONPATH` 上；`sudo` 下的 root 环境尤其不带）。它是解释器报错（退出码 1），**不是链路结论**，也不在 10~60 这套退出码里：改用上面的 `python3 -m hunter_kafka.diagnose`（或部署脚本装的 `~/.local/bin/hunter-kafka-check` 包装），它们不依赖包元数据（部署脚本 ⑤ 已自动补 `PYTHONPATH`，并在遇到退出码 1 时自动改走源码跑法）。

> ⚠️ **【报 `cannot import name 'AdminClient' from 'confluent_kafka'`】** **不是没装库**：`AdminClient` 属于 `confluent_kafka.admin` 子模块，顶层不重导出（早期代码写成 `from confluent_kafka import AdminClient`，已在本版修正）。因此车上仍见到这条 = 跑的是**旧版代码**，先把仓同步到车上（`--symlink-install` 下 `src/` 里的 `.py` 同步即生效，无需重编）；同步后还报，才是库本身过旧：`sudo pip3 install -U confluent-kafka`（部署脚本 ① 的依赖判据已改为直取 `admin.AdminClient`，不只看 `import confluent_kafka` 成不成功）。

> ⚠️ **【报 `ssl.key.location failed: … Permission denied`（常伴 Traceback + 退 1）】** **不是 TLS 故障，也不是权限“不够大”，而是属主不对**：私钥按契约给 0600，而 0600 的**组位与其他位都是 0**，所以装成 `root:<组> 0600` 时，以 `agilex` 跑的进程（自检、`data_agent`）**根本打不开自己的私钥**——前 5 层全绿、到建客户端才崩就是这一类。处置（一条命令）：
> `sudo chown agilex:agilex /etc/hunter/kafka/client-key.pem /etc/hunter/kafka/kafka-client.p12 && sudo chmod 600 /etc/hunter/kafka/client-key.pem`，
> 或直接重跑 `hunter_core_setup.sh`（步骤②.1 会收敛属主并以运行用户身份**实测能真打开**）。本版自检已在**第 1 层**逐件真打开凭据并直接给结论，不会等到第 6 层才报 OpenSSL 原文。

**退出码与处置**（逐层递进，前一层失败即短路）：

| 退出码 | 含义 | 首要处置 |
|---|---|---|
| `0` | 全部通过 | 看平台侧该车 `last_online_time` 是否刷新（**联调唯一判据**） |
| `10` | 参数/文件 | 四件套是否齐全、SCRAM 口令是否仍为占位符、`vehicle_id` 能否推出；**接入包未拷到车上/路径层级传错**也归本类 |
| `20` | TLS/证书 | 先 `timedatectl` 校时；再查证书有效期、CN 是否等于 `vehicle_id`、私钥是否 **0600 且属主为运行用户**（报 `ssl.key.location failed: … Permission denied` 就是这一条） |
| `30` | 认证 | SASL 用户名/口令、账号是否已在平台开通、是否缺 SCRAM 插件（`ls /usr/lib/*/sasl2/libscram.so`；无输出则 `sudo apt install -y libsasl2-modules`——**实机 `dpkg -S` 确认的属主包**） |
| `40` | 网络 | `bootstrap.servers` 的 IP:端口需与 broker `advertised.listeners` 一致 |
| `50` | Topic/ACL | 平台未按该车建满 8 个 Topic，或账号无 describe/write 权限 |
| `60` | 投递 | broker 可达但 leader 异常；看 `data_agent` 是否已转 SQLite 缓存 |

> 📌 **退出码按“真因”而非“发在哪一层”**：握手（第 6 层）与投递（第 7 层）的错误都会先按**错误文本关键字**、再按**错误码常量名**归类（常量数值随 librdkafka 版本变，不硬编码数值）——所以投递阶段碰上 `SASL authentication failed` 会报 **30**、`Topic authorization failed` 会报 **50**、`No worthy mechs found` 会报 **30** 并直指缺 SCRAM 插件，而不是一律归到 60（旧写法会把“口令错”伪装成“投递故障”）。

完整 Topic 契约、凭据红线与服务部署细节见 [§11.3](#113-huntercore-车端接入契约v0105) 与 `Deployment_Guide.md` §5.6。

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
| `/auto_mission/status` | `std_msgs/String` | 10Hz | 自主任务状态（IDLE/MAPPING/WAITING_LOCALIZE/NAVIGATING/OBSTACLE_AVOID/ESTOP/FAULT） |
| `/auto_mission/current_waypoint` | `std_msgs/Int32` | 事件 | 当前执行的航点索引 |
| `/pcd_to_map/status` | `std_msgs/String` | 事件 | PCD→地图转换状态（IDLE/CONVERTING/DONE/ERROR） |
| `/navigate_to_pose` (action) | `nav2_msgs/NavigateToPose` | — | Nav2 单点导航 action 接口（方式B 外部下发） |
| `/safety/state` | `std_msgs/String` | 2Hz | safety_guard 分级预警心跳（状态\|原因；OK/SLOWDOWN/COLLISION_STOP/SCAN_TIMEOUT/CMD_TIMEOUT/ESTOP_PASS/TEST_ABORTED/MAP_EDGE_SLOWDOWN/MAP_EDGE_STOP） |
| `/safety/test_mode` | `std_msgs/Bool` | 事件 | 自动驾驶测试模式开关（true 开启 0.3m/s 限速+严阈值+异常自动中止，V0.0.89） |
| `/cmd_vel_nav` | `geometry_msgs/Twist` | 20Hz | controller_server 原始速度指令（safety_guard 测试模式监控其断流） |
| `/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | 5Hz（= `publish_frequency`） | 局部代价地图整图（帧 `odom`）；**V0.1.00 起被 `auto_mission` 订阅用作“车身近身假障碍占位”诊断**，需 `local_costmap.always_send_full_costmap: true` 才会在几何不变时持续发整图 |

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
NAVIGATING ──[航点受阻：stall_detect_time 内净推进 < stall_move_eps
            或 单航点超时 goal_timeout]──→ 取消 goal → 清图重规划 → 换下一个可用航点
            └─ V0.1.00：失败计入【该航点】，达 max_wp_failures(3) 仅隔离该点
                        （冷却 wp_isolation_cooldown=120s 后自动重试）
NAVIGATING ──[航点已在 already_reached_dist 内 / 越界 / 占据栅格 / 净空不足
            / 不在可通行连通域（V0.1.00）]──→ 跳过并隔离该航点（不发 goal）
NAVIGATING ──[任务级连续失败达 max_consec_failures(12)、全部航点均被隔离、
            或 goal 连续被拒收]──→ FAULT
FAULT ──[静置 fault_hold_time(20s) 后周期自愈：诊断+清图+航点复判+三重门控通过]──→ IDLE
                                                （V0.1.00：自动复驶，无需人工解锁）
FAULT ──[模式离开 AUTO（遥控接管/降级）]──→ IDLE（全量复位，含清除地图不可达永久隔离）
任意状态 ──[非AUTO/急停]──→ IDLE / ESTOP
```

> **V0.1.00 任务层「自愈四件套」**（对应实车日志「任务已锁存（单航点导航超时连续达上限）…需将模式开关离开 AUTO 再切回」的彻底修复）：
> ① **逐航点隔离与轮转**：`WaypointRuntime{fail_count, isolated, permanent, isolated_at, last_reason}` 数组 `wp_rt_` 取代旧的跨航点总量 `wp_fail_count_`；`handleWaypointFailure()` 统一处置“放弃当前航点 → 逐点计数 → 即时诊断 → 清图重规划 → `advanceToNextAvailableWaypoint()`”。A/B/C 三个不同航点各失败一次不再拖垮整条任务。
> ② **近身假障碍自动诊断** `diagnoseSelfSurroundings()`：代价地图（车身框 + 外扩 `self_check_margin`）判“Nav2 认为近身有东西”× 激光/融合目标判“传感器确实看到东西”→ A∧¬B 即【假障碍占位】，连判 `self_check_confirm_count` 次后自动清图；结论直接写进 `[航点失败]`/`[FAULT]` 日志。⚠ 硬依赖 `nav2_params.yaml` `local_costmap.always_send_full_costmap: true`。
> ③ **地图可达性** `ensureReachabilityCache()` + `waypointReachabilityDetail()`：静态地图上“非占据、非未建图且净空 ≥ `reach_clearance`”的 8 邻域连通域（斜向需两侧正交格可通行）；航点不在车位连通域 → 永久隔离（不逐圈重复失败）；信息不足/预算截断一律“不可判定→放行”（宁误放勿误杀）。
> ④ **FAULT 自愈态** `faultRecoveryStep()`：静置 → 假障碍则清图 → 遍历航点复判（隔离到期推进 + 地图校验 + 可达性）→ 三重门控（AUTO 条件&Nav2 ACTIVE ∧ 可用航点≥1 ∧ 近身非真障碍包围）→ `resetMissionState()` 回 IDLE 自动重启；失败则按 `fault_retry_interval × 轮次`（上限 180s）退避重试，**永不锁死**。遥控接管后**切回 AUTO 即自动复驶**（`mainLoop()` 检测离开 AUTO 的下降沿做全量复位，回 IDLE 后由 IDLE 分支自动重启）。
>
> **V0.0.95 任务层两条旧防线**（仍有效）：
> ① **航点“已到达”预检**：航点与 `/relocalization/pose` 位置重合（≤ `already_reached_dist` 0.35m）
>    时视为已完成并跳过——goal 与自身位姿重合时 Smac 路径≈0 且要求终止朝向，阿克曼无法
>    原地转向，MPPI 会持续打满转向（日志 `set steering angle: ±0.386428 rad` 即曲率钳制上限
>    对应的最大内轮转角）而纵向零进挪，ProgressChecker(0.1m/10s) 必判 `Failed to make progress`；
> ② **受阻（stall）检测**：goal 在途且朝目标推进量 < `stall_move_eps`(0.15m) 持续 `stall_detect_time`(25s)
>    → 明确判“前方障碍无法绕行/航点无可达路径”，取消本 goal 并换点（**V0.1.00 起：失败计入该航点，不再直接 FAULT**），
>    日志给出可判读结论，避免“原地抖动 90s 后静默换点”。（V0.0.97 补：累计行程 ≥ `stall_path_allow_m` 1.0m
>    视为绕障机动中不判受阻；V0.0.98 补：换点重发经 `goal_cancel_settle_time` 静置门控，不再取消后 2ms 立即重发）
>
> **V0.0.96 任务层第三条防线 + 到达语义修正**：
> ③ **航点占据栅格 + 净空校验**（`waypoint_clearance_m` 0.50m）：发送前拒绝“落在静态地图障碍上/其
>    膨胀区内”的航点（现场 (5.0,0.0) 即此类，车开到该点后 Smac 持续抛 `Starting point in lethal space!`）；
> ④ **“ABORT 但已在到达半径内 ⇒ 判为完成”**：避免“车已到 0.07m 却因规划失败被整树 ABORT”被计成
>    航点失败而快速累积到 FAULT。
>
> **V0.1.00 发送前预检扩为三级**（`sendNextWaypoint()`）：隔离态（含到期复判）→ 地图四级校验 **+ 可达性**（不通过＝**永久隔离**）→ 已到达。全部被筛掉才 `enterFault`。

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
| `max_wp_failures` | 3 | **V0.1.00 语义变更**：由「跨航点累计」改为【单个航点】连续失败上限——达上限只**隔离该航点**（冷却后自动重试），不再锁存整条巡航线；跳航点的任务级兜底由 `max_consec_failures` 负责。`<1` 强制回 1 |
| `goal_timeout` | 45 s | 单点导航超时（**V0.0.95 90→45**：受阻由 `stall_detect_time` 25s 内提前定性，本超时只兜底“缓慢但可达”的航点） |
| `obstacle_wait_timeout` | 30 s | 障碍物等待超时后触发 ESTOP |
| `already_reached_dist` | 0.35 m | **V0.0.95 新增；V0.0.97 由 0.30 上调**：航点“已到达”判定半径。航点与当前 map 系位姿距离 ≤ 此值时视为已完成并**跳过不发 goal**（修“目标=自身”退化 goal 死锁；0.304m 退化目标实例）。须 > 阿克曼停车精度且 ≥ 2× Nav2 `xy_goal_tolerance`(0.10m)，不宜 >0.4m。全部航点均在此半径内 → FAULT 锁存 |
| `stall_path_allow_m` | 1.0 m | **V0.0.97 新增**：绕障机动宽容——自本 goal 发出起**累计行程** ≥ 此值即视为“多点掉头/倒车绕障中”，即使净推进 <`stall_move_eps` 也不判受阻（放开倒车后正常绕障是“倒 0.4m→进 0.5m→再倒”多段机动，净推进可长期偏小）；兜底 `goal_timeout` 45s |
| `goal_cancel_settle_time` | 2.0 s | **V0.0.98 新增**：取消静置门控。本节点主动 cancel 旧 goal 后换点重发前等待旧结果收敛（CANCELED/ABORTED 先到即提前放行，超时兜底）——消除日志实证的“取消后 2ms 即发新 goal → 新 goal 被旧 BT 失败状态波及 ABORTED → fail_count 误累积→ FAULT”；<0.5 强制回 2.0，不建议 >5s；配套：`ABORTED + 本节点主动取消` 不计失败不换点 |
| `stall_detect_time` | 25 s | **V0.0.95 新增**：受阻判定时长。goal 在途且朝目标推进停滞 → 判“前方障碍无法绕行/航点无可达路径”，取消本 goal 并换点。须 > 一轮 Nav2 恢复周期（ProgressChecker 10s + 清图 + 受限倒车脱困） |
| `stall_move_eps` | 0.15 m | **V0.0.95 新增**：受阻判定位移下限（>2× 定位抖动）。**V0.0.96 判据改“朝目标推进量”**（= 发 goal 时到航点距离 − 当前距离）——倒车脱困会增大到目标距离（推进量为负）仍判受阻，位移标量会被“后退”骗过 |
| `waypoint_clearance_m` | 0.50 m | **V0.0.96 新增**：航点距最近**占据栅格**的最小净空。航点为占据栅格 / 净空不足 → 发送前拒绝并跳过（日志给实测值），全部被拒 → FAULT。必须 > Nav2 内切半径（footprint 半宽 0.32m）；必要性：滚动局部代价地图不含 static_layer，MPPI 不会避开仅存在于静态地图中的障碍，会把车开到该点，随后 Smac 持续报 `Starting point in lethal space!`（清图无效——`StaticLayer::reset()` 只置 `has_updated_data_` 并重新盖章） |
| **V0.1.00 需求①——逐航点隔离与轮转** | | |
| `wp_isolation_cooldown` | 120 s | 临时隔离（导航失败累计）冷却期满自动解除并清零该点失败数；【永久隔离】（地图不可达）也按本周期**复判**（重跑地图校验 + 可达性，通过即解除）——避免永久隔离成为新的死锁。`<10s` 强制回 10s |
| `max_consec_failures` | 12 | 任务级兜底：连续失败期间**一次都没成功到达**过，达此上限才进 FAULT（自愈态）。必须 > `max_wp_failures`（内部校验：不足则自动抬到 `max_wp_failures+2`，否则来不及轮转就锁存） |
| **V0.1.00 需求②——车身四周/脚底假障碍自动诊断（激光 + 相机交叉验证）** | | |
| `self_check_enable` | `true` | 总开关；关闭后诊断降级为“不可判定”，不再自动清图 |
| `self_check_interval` | 5.0 s | `NAVIGATING` 中周期巡检间隔（除周期巡检外，每次航点失败当下、每次 FAULT 自愈检查、进 FAULT 即刻均会跑一次） |
| `self_check_margin` | 0.35 m | 车身框外扩监视边距（“车身四周”）；置 0 则只查“脚底”（原始车身框内） |
| `self_check_box_front / _rear / _half_width` | 0.45 / 0.37 / 0.32 m | 车身框尺寸（base_link 系），**必须与 `nav2_params.yaml` 的 footprint 一致**；存在非正值则回退默认 0.45/0.37/0.32 |
| `self_check_cost_min` | 99 | 代价地图“占据”阈值（Nav2 `cost_translation_table`：0=自由、99=内切、100=致命、-1=未知）；clamp 到 [1,100] |
| `self_check_confirm_count` | 3 | 防抖：连判 N 次“假障碍”才触发自动清图（单帧噪声不致误动作） |
| `self_check_data_timeout` | 1.0 s | 代价地图 / 里程计 / 激光目标新鲜度阈值；超时则结论为“不可判定”（绝不误判、绝不误清图） |
| `local_costmap_topic` | `/local_costmap/costmap` | 诊断数据源 A（帧 `odom`）；⚠ **硬依赖 `nav2_params.yaml` `local_costmap.always_send_full_costmap: true`**——否则 `Costmap2DPublisher` 只在窗口几何变化时发整图，车辆停驻（正是需要诊断的时刻）时收到的是过期图 |
| `lidar_objects_topic` / `vision_objects_topic` | `/perception/lidar_objects` / `/perception/vision_objects` | 诊断数据源 B（激光 base_link 直接参与几何判定；相机原始帧为 `camera_color_optical_frame`，本节点**不引 TF 依赖**，只取其新鲜度/目标数参与结论描述；相机确认实体的几何判定由 `/perception/fused_objects` 承载） |
| **V0.1.00 需求③——航点可达性按地图范围判定** | | |
| `reachability_enable` | `true` | 旧四级校验（矩形界/未建图/占据栅格/净空）之上的**第五级**：可通行连通域 |
| `reach_clearance` | 0.35 m | 可通行格最小净空（≈内切半径 0.32 + 余量）；`<0.32` 自动抬到 0.35（过小会把实际过不去的窄缝判成连通） |
| `reach_recompute_dist` | 1.0 m | 车位移动超此距离才重建连通域（节流）；地图修订号 `map_revision_` 变化则必重建 |
| `reach_bfs_max_cells` | 400000 | BFS 预算（格数）；被截断时“不可达”结论不可靠 → **自动放行**（宁误放勿误杀）；`<10000` 强制回 10000 |
| **V0.1.00 需求④——FAULT 自愈** | | |
| `fault_auto_recover` | `true` | 置 `false` 可退回旧行为（只停驻等人工解锁） |
| `fault_hold_time` | 20 s | 进 FAULT 后首次自愈检查前的静置时长（留给障碍离开/人工移车）；`<5s` 强制回 5s |
| `fault_retry_interval` / `fault_retry_max_interval` | 30 s / 180 s | 自愈重试间隔随轮次递增（`interval × 轮次`，上限 `max_interval`）——防刷屏与慢死循环，但**永不锁死** |
| `max_mission_recoveries` | 5 | 超过此轮次后仅提高告警强度并给人工建议（不锁死） |

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
│   │   ├── auto_mission_node.cpp               ← 7 态状态机 C++ 实现（V0.1.00 含 FAULT 自愈与逐航点隔离）
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

### 10.10 V0.1.02 避障参数（《HUNTER SE 低速自动驾驶避障解决方案》对表）

对表审计的完整方法见 Deployment_Guide **§5.5.2**（"方案章节 → 实现锚点"表 + "有意保留偏差"表）。本节只列**本轮被改动的值**，便于代码审查与回归定位：

| 文件 | 参数 | 改动 | 依据 / 关键理由 |
|---|---|---|---|
| `hunter_bringup/config/nav2_params.yaml` | `global_costmap.inflation_radius` | 0.40 → **0.55** | 方案 §4.4。全局图是 **Smac** 的规划依据：留白 = 半宽 0.32 + 0.23m，路径"天然取中"→ 少贴墙贴门框 → 少触发安全层 SLOWDOWN/STOP（低速更平顺）。仅 ≥0.32m（内切半径）才是致命格，可通行性不受损 |
| 同上 | `global_costmap.update_frequency` | 2.0 → **1.0** | 方案 §4.4。BT 重规划 V0.0.92 起已回退 1Hz；2Hz 反复刷 100 万格纯属浪费 |
| 同上 | `local_costmap.update_frequency` | 10.0 → **5.0** | 方案 §4.4。@0.5m/s 每拍 10cm；膨胀 0.45m + MPPI 20Hz 重优化足够；近身快闸是 safety_guard 直读 /scan 的 20Hz 扫掠弧 |
| 同上 | `local_costmap.width` / `height` | 10 → **6** | 方案 §4.4。校核：4.0s 时域 @0.5m/s 前瞻 2.0m + 前缘 0.45 + 膨胀 0.45 ≈ **2.9m < 3.0m** ✔；两项合计局部图算力降至 ≈1/5.6（200²@10Hz → 120²@5Hz） |
| `hunter_perception/lidar_perception/config/lidar_perception_params.yaml` + `src/lidar_perception.cpp` + `include/lidar_perception/lidar_perception.hpp` | `ground_max_slope` | **新增（默认 5.0°）** | 方案 §4.1 第 6 步。原为 `RayGroundFilter` 构造默认 8.0°，硬编码不可配；平坦地面 dz≈0 两者判定一致，收紧只让坡道/台阶更早判为障碍 |
| 同上 | `outlier_mean_k` | 10 → **50** | 方案 §4.1 第 5 步。SOR 是全链唯一噪点滤除环节，k 越大越能区分孤立噪点与真实稀疏回波 → 减少 costmap/safety_guard 幽灵点误判 |
| 同上 | `cluster_tolerance` | 0.5 → **0.15** | 方案 §4.1 第 7 步。⚠ 16 线垂直 2°/线 ⇒ 线间距 1m→3.5cm、3m→10.5cm、4.3m→15cm：**近身聚类恒完整，4.3m 外逐环线断开**。只影响 `/perception/lidar_objects` 粒度与跟踪，**不影响避障**（代价地图障碍源是未聚类原始点云与 `/scan`） |
| `hunter_perception/sensor_fusion/config/sensor_fusion_params.yaml` | `vision_conf_min` | 0.45 → **0.50** | 方案 §4.3。低置信度误检不再参与融合，"激光 × 融合"交叉验证（假障碍诊断 B 路）更干净 |
| `hunter_safety/src/safety_guard.cpp` | `max_linear_vel` **默认值** | 0.8 → **0.5** | 方案 §8.3。本参数是第二重速度硬限，默认值必须等于生产值——否则 launch 漏传参时静默放行 1.6× 巡航速度 |

**有意保留的偏差（6 项，理由见 Deployment_Guide §5.5.2(3)，改前必读）**：点云裁剪下界 `0.5m`（方案 0.2，防自反射幽灵目标）、融合对齐窗 `0.20s`（方案 ±50ms，保任务层 ESTOP 可用）、欧式聚类实现（方案称 DBSCAN，本仓为 PCL `EuclideanClusterExtraction`）、相机 `15fps`（方案 §4.2 写 30，§9.3 验收允许 15-30）、`YOLOv8s`（方案写 n，engine 须目标机生成）、局部图无 `static_layer`（方案亦列为"已知限制"）。

**重编清单**：`rm -rf build/{hunter_safety,hunter_bringup,lidar_perception} install/{hunter_safety,hunter_bringup,lidar_perception}` → `colcon build --packages-select hunter_safety hunter_bringup lidar_perception`（若同时应用融合参数再加 `sensor_fusion`），随后 **Ctrl+C 重启 bring-up**。

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

### 11.3 HunterCore 车端接入契约（V0.1.05）

**身份三合一**：`vehicle_id` 是接入的唯一锚点，必须同时等于 `kafka.properties` 的 SASL 用户名 **与** 客户端证书 CN（本例 `HUNTER-001`）。Topic 命名硬约束 `hunter.<vehicle_id>.<type>`，每车 8 条；拼名时拿不到 `vehicle_id` 则**拒启**（不生成 `hunter..telemetry` 这类非法名，避免“静默发往不存在 Topic”这种最难查的故障）。凭据优先级：`/etc/hunter/kafka/kafka.properties` > YAML（YAML 里只放非凭据参数）。

| Topic | 方向 | 节拍/触发 | acks | 车端实现 |
|---|---|---|---|---|
| `hunter.<vid>.telemetry` | 车→云 | 10Hz | `1` | `data_agent`（C++）：底盘/位姿/感知/任务快照，`key=vehicle_id` 保证同车分区内有序 |
| `hunter.<vid>.event` | 车→云 | 事件即发 | `all` | `data_agent`：急停/碰撞/低电/超速/通信中断/遥控会话/OTA 关键节点，**不可丢** |
| `hunter.<vid>.health` | 车→云 | 1Hz | `1` | `data_agent`：`/system/health`（`SystemHealth`）转 JSON；**无数据时报 `NO_DATA`，不伪造 `OK`** |
| `hunter.<vid>.command` | 云→车 | 平台下发 | — | `command_agent` 消费：组 `command-<vid>`、`earliest`、**手动提交 offset** |
| `hunter.<vid>.command_result` | 车→云 | 每条指令终态一次 | `all` | `SUCCEEDED/REJECTED/TIMEOUT/FAILED` + 状态快照；同一 `command_id` 只允许一条终态 |
| `hunter.<vid>.remote_control` | 云→车 | 摇杆帧 10~20Hz | — | `remote_agent` 消费：组 `remote-<vid>`、**`latest`**（旧帧无意义、防回灌）+ TTL 二次过期 |
| `hunter.<vid>.ota_notify` | 云→车 | 升级下发 | — | `ota_agent` 消费：组 `ota-<vid>`、`earliest`；按 `task_id` **幂等** |
| `hunter.<vid>.ota_status` | 车→云 | 进度 10% 步进 + 终态 | `all` | `DOWNLOADING/INSTALLING/SUCCESS/FAILED/ROLLBACK` |

**指令白名单**（`command_agent` 当前实现；**未登记类型一律 `REJECTED`**，绝不“未知即执行”）：

| 类型 | 车端动作（全部复用既有服务/话题） | 前置条件 |
|---|---|---|
| `MISSION_START` / `MISSION_STOP` | `/auto_mission/start_mapping_cruise` / `stop_mapping_cruise` | 任务层门控（模式/定位收敛） |
| `MAP_CONVERT` | `/pcd_to_map/convert` | PCD 存在 |
| `WAYPOINT_SAVE` / `WAYPOINT_CLEAR` | `/waypoint_recorder/save` / `clear` | **CLEAR 需 `params.confirm=true`**（破坏性） |
| `ESTOP_ENGAGE` / `ESTOP_RELEASE` | 锁存发布 `/estop` | 解除要求车速 < 0.05 m/s（静止） |
| `TEST_MODE_ON` / `TEST_MODE_OFF` | 发布 `/safety/test_mode` | 仅低速联调 |
| `REMOTE_RELEASE` | 发一帧 `control_mode="AUTO"` 的 `/remote/command` 交还驾驶权 | — |
| `STATUS_QUERY` / `HEARTBEAT_ACK` | 直接回执状态快照 | — |

> 🔒 **五条护栏不可绕过**（白名单 / TTL 过期丢弃 / `command_id` LRU 幂等 / 单条在途 + 超时看门狗 / 破坏性指令 confirm + 静止门控）。消费端**手动 commit**，保证“没真正执行完不丢指令”；回执丢失重发时由幂等表抑制重复终态。

> ⚠ **不新增 ROS 话题/消息字段**（.ai-rules）：指令执行一律落在既有接口上；`command_result` 的车端留痕以 INFO 级日志 `COMMAND_RESULT <json>` 输出（`journalctl` 可核对“平台下发过什么、车端回了什么”）。

**凭据红线**（`.gitignore` 已排除，不得口头约定）：`*.pem` / `*.p12` / `*.jks` / `kafka.properties` / `*hunter-*-bundle/` **一律不得入仓、入镜像层、入日志**；私钥 `client-key.pem` 必须 `0600` **且属主为运行 Agent 的用户**（0600 只授予属主；`hunter_core_setup.sh` 步骤②.1 会收敛属主并实测运行用户真能打开，否则直接失败）；SCRAM 口令只存 `/etc/hunter/kafka/kafka.properties`，**不得写进任何 `*_params.yaml`**；代码侧输出配置快照时必须走 `redact()` 打星（`data_agent` 的 `kafka_access`、`hunter_kafka.config.redact`）。

> ⚠ **模式交还硬约束**：`remote_agent` 的 Kafka 消费超时后必须**停止发布** `/remote/command`（而非发零速顶住），`decision_making` 按该话题 0.5s 新鲜度判定 REMOTE，不发即自动回 AUTO；否则会出现“遥控断开后车辆永不交还自主”缺陷（V0.1.05 已修）。仲裁优先级：ESTOP > REMOTE > AUTO。

---

## 12. 运维脚本说明

> 🔧 **【现场运维视角】** 脚本位于 `hunter_bringup/scripts/`，需先赋予执行权限：

```bash
chmod +x ~/HunterEdge/src/hunter_bringup/scripts/*.sh
```

| 脚本 | 用途 |
|------|------|
| `hunter_status.sh` | 查看系统状态、节点存活、资源占用；**V0.1.05 新增第 8 段「HunterCore 接入」**（四件套存在性与私钥权限、systemd 服务态、`/health` 是否有数据；只查元信息不读 `kafka.properties` 内容，避免口令进日志）；**V0.1.06 新增第 8 段「上电自启链路」**（can2 状态/波特率 + `candump` 抓底盘反馈帧、`hunter-can`/`hunter-edge`/`ota-agent`/`remote-agent` 服务态）与第 10 段「运营端上行（车端侧自证）」（SQLite 待回放缓存条数） |
| `hunter_core_setup.sh` | **V0.1.05 新增**：HunterCore 一键接入部署（依赖 → 接入包落盘 → 参数落盘 → 编译 → 自检 → systemd），六步幂等，可重跑（见 §7.7）；**V0.1.06**：步骤⑥ 由 2 个单元扩到 4 个（`hunter-can`/`hunter-edge`/`ota-agent`/`remote-agent`）并按依赖顺序 enable+启动，新增 `--no-start`（台架只装不启） |
| `hunter_can_up.sh` | **V0.1.06 新增**：上电自动启用底盘 CAN（等待 USB-CAN 枚举 → `ip link set can2 down/type can bitrate 500000/up` → `candump` 抓帧验证底盘反馈帧 0x211/0x221）。幂等；退出码 `0` 成功 / `1` 环境 / `2` 无接口 / `3` 无底盘帧；`--quick` 只复查接口、`--no-strict` 只要求接口 up |
| `hunter_edge_up.sh` | **V0.1.06 新增**：HunterEdge 整栈启动包装（① CAN 复查 → ② 运营端连通性预检 `hunter-kafka-check`（失败不阻断，落 SQLite 待回放）→ ③ `ros2 launch hunter_bringup hunter_edge.launch.py` 进入自动驾驶准备状态）。供 `hunter-edge.service` 调用，也可现场手工执行 |
| `hunter_log.sh` | 查看/导出系统日志 |
| `hunter_can_test.sh` | CAN 通信测试（手动 `up/down/dump/send/test`；上电自启请用 `hunter_can_up.sh`） |
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

### 12.5 HunterCore 接入部署与状态查看（V0.1.05）

```bash
# 首次接入 / 换车 / 接入包重新下发（需 sudo，可重复执行；接入包需先拷到车上）
sudo bash ./hunter_core_setup.sh --bundle ~/HUNTER-001-bundle/HUNTER-001

# 只是改完口令/参数后重跑（无需再传 --bundle：/etc/hunter/kafka 四件套齐就自动复用现场凭据、跳过步骤②）
sudo bash ./hunter_core_setup.sh --skip-deps

# 接入链路自检（退出码含义与命令入口见 §7.7）
~/.local/bin/hunter-kafka-check          # 由部署脚本⑤ 安装；重登一个 shell 后可直接敲 hunter-kafka-check
PYTHONPATH=~/HunterEdge/src/hunter_common/hunter_kafka python3 -m hunter_kafka.diagnose   # 不依赖任何安装的等价跑法

# 一键确认“接入到底通不通”（含凭据落盘/服务态/health 数据）
./hunter_status.sh

# 平台侧下发过什么指令、车端回了什么（command_result 留痕）
journalctl -u remote-agent -o cat | tail -50
ros2 topic echo /system/health --once        # health 数据源
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

- **OTA Agent** 与 **Remote Agent** 为独立 systemd 服务（非 ROS 节点），需单独部署（`data_agent`/`command_agent` 仍由 `hunter_full.launch.py` 拉起，不注册服务）；
- 两者通过 Kafka / WebSocket 与平台交互，Remote Agent 通过 rclpy 桥接发布 `/remote/command`；**V0.1.05 起遥控链路改为 Kafka `remote_control` 消费驱动**；
- 单元文件的 `ExecStart` **不写死 `/opt/hunter/...`**，而是 source `/etc/hunter/agent_env.sh` 取工作空间前缀后执行包内入口——**换目录/换机后重跑 `hunter_core_setup.sh` 即可，不必改 unit**；现场参数文件取 `/etc/hunter/<agent>_params.yaml`（`HUNTER_OTA_CONFIG` / `HUNTER_REMOTE_CONFIG`）。

### 13.7 容器化可选（设计文档 §20.5）

平台支持 Docker 镜像部署（`nvidia` 运行时 + host 网络 + 设备直通 + `--ipc=host`），与 OTA 升级（镜像拉取替换）协同；原生 colcon 工作空间部署仍为默认方式。

> ⚠ **镜像化时凭据不得进镜像层**：`/etc/hunter/kafka/` 四件套属于**运行期注入**（挂载卷 / 下发机制），不得 `COPY` 进镜像；否则同一镜像多人传播等于批量泄露客户端证书（接入包 README 硬约束）。

### 13.8 常见故障排查（设计文档 §20.4）

| 故障现象 | 可能原因 | 排查步骤 |
|----------|----------|----------|
| CAN 无数据 | 接线 / 波特率 / 驱动 | 检查 CAN 线、`ip link show can2`、`candump can2` |
| LiDAR 无点云 | 网络 / 电源 / IP 配置 | `ping` LiDAR IP、检查供电、`rosnode list` |
| 相机无图像 | USB 连接 / 权限 | 检查 USB、`ls /dev/video*`、权限配置 |
| 定位漂移大 | IMU 标定 / 轮速 / 外参 | 检查 IMU 数据、外参文件、EKF 参数 |
| 控制抖动 | 控制参数 / 延迟 | 调整 MPPI 参数（critics 权重/时间步）、检查控制频率 |
| 系统卡顿 | GPU / CPU / 温度 | `tegrastats` 查看资源、降温、降频 |
| 无法连平台 | 网络 / 证书 / Kafka SASL 配置 | **先跑 `hunter-kafka-check` 拿退出码定位到哪一层**（§7.7）：10=配置/口令未填、20=证书/未校时、30=认证或缺 SCRAM 插件、40=网络、50=Topic/ACL；不再靠翻 `kafka_brokers`/`sasl_password` YAML（V0.1.05 起凭据不在 YAML 里） |
| 平台下发指令无反应 / 一直 `REJECTED` | 指令类型不在白名单、接入包未部署（`command_agent` 降级）、旧 `command_id` 被幂等表拦下、TTL 过期 | `ros2 launch … use_command_agent:=false` 可单独排除该节点；看 `command_agent` 日志里 `REJECT …` 原因字串；`ros2 topic echo /system/health --once` 确认 ROS 侧接口在位 |
| 视觉节点每帧崩溃（`setSize` 断言） | OpenCV 4.10 / 4.5.4 **混链**（同进程两套 OpenCV，破坏 `cv::Mat` 不变量） | `ldd vision_perception_node \| grep opencv`：只允许 `so.410`、无 `4.5d`、无 `libcv_bridge`；异常时按 §6 专项重编（禁止在视觉进程重新引入 cv_bridge） |
| 视觉 0Hz，日志 `resize.cu:175 error (-217) no kernel image` | OpenCV CUDA 编译 ARCH 与实机 GPU 不符（如 sm_72 用于 Orin） | `cuobjdump --list-elf /usr/local/lib/libopencv_cudawarping.so.410` 应见 `sm_87`；按 §5.3 以 `CUDA_ARCH_BIN=8.7` 重编后重启节点即恢复 GPU（期间节点自动降级 CPU，感知不断流） |
| sensor_fusion 报 `TF unconnected trees` | 相机 frame 双父（驱动 TF + URDF 并存，TF 树分裂） | 确认 `hunter_full.launch.py` 相机驱动为 `publish_tf: 'false'`；`ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame` 验证外参；必要时 `tf2_tools view_frames` 看全树 |
| 启动时一次性 `彩色图像超时 x.x s` | 启动竞态（视觉节点激活早于彩色流就绪） | 仅出现一次属良性，可忽略；反复出现才按"相机无图像"排查 |
| **相机全程 0Hz**：`xioctl(VIDIOC_QBUF) failed: No such device` + `Failed to resolve the request: Z16 848x480`，`彩色图像超时` 持续递增、`health_monitor` 报 `camera 话题频率异常 0.0Hz`（V0.0.95 已修） | D435 驱动启动后先落默认 profile（depth/infra 848x480x30）再"停传感器→重开"，**重开瞬间 USB 设备节点消失**（ENODEV）→ 整机相机 0Hz，vision/fusion 退化为单雷达源 | ① V0.0.95 起 launch 已显式下发 `640,480,30`（避免默认 profile 触发的 stop/start 重配）并关闭 infra；② 仍复现按硬件链排查：`lsusb`、`dmesg \| grep -iE 'usb\|uvc\|xhci'`（找 disconnect/reset）、D435 直连 USB3 口勿经 HUB、关闭 USB 自动挂起；③ 设备枚举异常（`/dev/video*` 消失）时把 `hunter_full.launch.py` 相机 `initial_reset` 改 `'true'` 重启；④ 验证 `ros2 topic hz /camera/camera/color/image_raw`（应 ~30Hz） |
| **自主巡航车辆原地抖动不前进**：`/safety/state` 在 `SLOWDOWN↔COLLISION_STOP` 间高频往返、`set steering angle` 恒为 ±0.386428、最终 `Failed to make progress`（V0.0.95 已修） | ① 航点与车辆当前位姿重合（"目标=自身"退化 goal，阿克曼无法原地转向）；② 走廊净空贴着急停阈值 ±6mm 抖振，速度被反复归零；③ BT 恢复池只有"清图+Wait"非运动手段，无法脱困 | V0.0.95 已分层修复（航点"已到达"预检 + 阈值释放滞环 + 受限倒车脱困 + 受阻检测）；现场仍复现时：① 确认车头前方 ≥2m 无障碍（`rviz2` 看 /scan 与 costmap，分清真实障碍/幽灵点）；② 看 `safety_guard` 启动日志确认 `释放滞环=+0.25/+0.20m`、`幽灵点门控=≥3 点` 已注入；③ 确认 `velocity_smoother min_velocity[0]=-0.20`（=0 会把倒车脱困钳成 0）；④ `waypoints` 首点不得与车位重合（见 `autonomous_nav_params.yaml` 航点布置约束） |
| **自主巡航报 `[NAVIGATING] 航点[i] 受阻` 或 FAULT** | 前方真实障碍无法绕行 / 航点在障碍后无可达路径 / 全部航点与车位重合 | **V0.1.00 起不再需要人工解锁**：受阻/超时 → 放弃该点 + 清图重规划 + 轮转下一个可用航点；日志直接给自动诊断结论（近身真/假障碍 + 航点可达性）；进 FAULT 后按静置→复检自动复驶，或遥控接管后切回 AUTO 立即复位重启；若仍不动，看 `[FAULT] 自愈未通过：…` 里列的具体门控（Nav2 未就绪 / 无可用航点 / 近身确有实体） |
| **倒车脱困"报成功但车不倒"** | `velocity_smoother` 对全部速度源做绝对值钳制，`min_velocity[0]=0.0` 把负线速度钳成 0（V0.0.94 及以前默认） | `nav2_params.yaml` `velocity_smoother.min_velocity` 应为 `[-0.20, 0.0, -0.8]`（V0.0.95）；禁倒车由 MPPI `vx_min=0` + Smac `allow_reversing=false` 保证，不在平滑器上设 0 |
| **行驶一小段后停下，`planner_server` 持续报 `Starting point in lethal space! Cannot create feasible plan..`，清图/等待均无效，3 次后 `FAULT`（V0.0.96 已修）** | 车辆停在**静态地图障碍的膨胀区**内（起点格代价 LETHAL 254 / INSCRIBED 253）→ Smac 的 `areInputsValid()` 判起点无效；清图无效（本 fork `StaticLayer::reset()` 只置 `has_updated_data_`，静态障碍会重新盖章）；**滚动局部代价地图不含 static_layer**，MPPI 不会避开静态地图障碍，因而会把车一路开到那里 | V0.0.96 三层修复：① 任务层航点净空校验（`waypoint_clearance_m` 0.50m，占据栅格 + 净空双判，发送前拒绝并打印实测净空）；② 行为树恢复池**首位 BackUp 0.45m**（规划失败也能物理驶离膨胀区，1~2s 内生效）；③ 任务层“ABORT 但已在到达半径内 ⇒ 判为已到达”。**现场恢复手段**：`ros2 run teleop_twist_keyboard` 人工把车倒出膨胀区，或 rviz2 确认航点位置后重标 `waypoints` |
| **自主巡航报"距静态地图障碍仅 x.xxm < 净空要求 0.50m（处于 Nav2 膨胀/致命区内）"并跳过该航点** | 航点标定在障碍旁/障碍上（仓库示例航点 (0,0)/(5,0)/(5,3)/(0,3) 为占位值，实测 (5.0,0.0) 不满足净空） | 设计行为（防止把车开进死局）：**V0.1.00 起该航点被永久隔离，巡航线继续跑其他点**，并在日志区分“地图校验”与“地图可达性”两类原因；在 rviz2 中确认目标点四周 ≥0.5m 无占据（黑色）栅格后按 `autonomous_nav_params.yaml` 的标定步骤重标（或遥控接管后切回 AUTO 触发全量复位重判），隔离按 `wp_isolation_cooldown`(120s) 周期自动复判 |
| **`[FAULT] …→ 第 N 轮自愈` 循环不前进（车辆停驻不复驶）** | 三重自愈门控有一项未过：`AUTO 条件 & bt_navigator ACTIVE` / `可用航点≥1` / `近身非真实障碍包围` | 看 `[FAULT] 自愈检查（第 N 轮）：…` 一行即可定位：`AUTO/Nav2就绪=否`→查定位收敛/感知新鲜度/health/Nav2 激活；`可用航点=0/n`→全部航点被隔离，核实地图或重标航点；`近身真障碍=是`→激光/相机确实看到车身四周有东西，需遥控移车或清障（此类**不允许自动复位**，防碰撞）；超 `max_mission_recoveries`(5) 轮后告警会给人工建议，但仍不锁死 |
| **近身假障碍诊断总给“数据不足（代价地图/里程计/激光目标超时），本次不可判定”** | `local_costmap.always_send_full_costmap` 未开（整图只在几何变化时发）、或 `odom→base_link` EKF 断流、或 `lidar_perception` 未发 `/perception/lidar_objects` | `ros2 topic hz /local_costmap/costmap` 应≈`publish_frequency`(5Hz)且**车辆静止时仍有数据**；`ros2 topic hz /localization/odom`（50Hz）、`/perception/lidar_objects`（10Hz，**无目标也发空数组**）；退化到“不可判定”是**安全侧行为**（宁可不自动清图也不误判），不影响需求①③④ |
| **相机仍 0Hz（V0.0.96 已将 `initial_reset` 置 true 仍复现）** | 属 USB 链路级故障（供电/带宽/接触/枚举异常），非驱动参数问题 | 按 §13.8 相机条目硬件排查：D435 直连 USB3 口（勿经 HUB）、`dmesg \| grep -iE 'usb\|uvc'` 查掉线、关 USB 自动挂起、必要时更换线缆/接口。导航不受阻（`health=WARNING` 不拦 AUTO 门控），但视觉避障退化为单雷达源 |
| **日志被 `set steering angle: x` 刷屏（20~50Hz）** | 底盘驱动（`hunter_ros2/hunter_base`，vendor 目录）在每个 `/cmd_vel` 回调 `std::cout` 打印转向角，未节流 | 分析时过滤：`grep -v 'set steering angle' /tmp/hunt7.log`；或 `scripts/hunter_log.sh` 导出后离线过滤。驱动属 vendor 代码（git-ignored），不建议直接改 |
| 一次性 `[TensorRT] Using an engine plan file across different models of devices` | `.engine` 非本机/本设备型号生成（换机或文件被旧引擎覆盖） | 不阻塞运行（话题 15Hz 正常）；目标机重生成：`trtexec --onnx=<绝对路径>/yolov8s.onnx --saveEngine=/data/models/yolov8s.engine --fp16` 后重启视觉节点 |
| **已升 V0.0.97 后仍绕不开障碍：不再满舵死磕，但带转向的绕障轨迹每执行 0.1~0.3s 即被 `COLLISION_STOP（行进方向走廊净空 …）` 清零，车在 stop/slow 间往复抖振（V0.0.98 已修）** | 旧碰撞闸把 (v,w) 只按行进方向直线投影求净空，忽略 w 的横移避让分量——绕障弧起点必落在走廊内，安全层反过来封死避障层 | 升级 `hunter_safety`（轨迹扫掠弧碰撞闸，README §10.8 / Deployment_Guide §5.5.1(5)）+ `hunter_bringup`（MPPI 4.0s 时域）；验收：启动日志 `V0.0.98 轨迹扫掠弧=行进中启用`、行进中拦停措辞「扫掠弧净空」；配套 `auto_mission` 取消静置门控消除换点误判 FAULT |
| Ctrl+C 后 `maps/` 只有 `.pcd`，`.pgm/.yaml` 未生成（V0.0.88 前必现） | FAST-LIO2 在 `main()` 于 `spin` 返回**后**才写 PCD（20.7M 点 ≈ 664MB 需数秒至数十秒），而 Ctrl+C 同时终止 `pcd_to_map`，运行期 `MAPPING→非MAPPING` 跳变不会发生 → 原自动转换从不启动 | V0.0.88 起 `pcd_to_map` 退出时派生独立会话后台转换进程兜底：`tail -f maps/pcd_to_map_final.log`（应见 `PCD 已写完整 → 转换成功`），数十秒内 `ls -lh maps/` 应齐 `.pcd/.pgm/.yaml`；仍缺时手动兜底 `python3 ~/HunterEdge/install/auto_mission/lib/auto_mission/pcd_to_map --finalize --pcd-file ~/HunterEdge/maps/hunter_map.pcd --force` |

### 13.9 HunterCore 接入约束与红线（V0.1.05）

> ⚠️ **【现场运维视角】** 以下四条均为**硬约束**，违反其一就会在现场出现“看着在跑、平台没数据”或“指令重复执行”类难查故障。

1. **凭据位置唯一**：Broker/账号/口令/证书只认 `/etc/hunter/kafka/kafka.properties`，`*_params.yaml` 里的同名字段仅为**开发机兜底**（生产留空）；不得入仓、不得进镜像层、不得写日志（`*.pem`/`*.p12`/`*.jks`/`kafka.properties`/`*hunter-*-bundle/` 已在 `.gitignore` 排除）。
2. **至少一次投递，不是正好一次**：断网期间 `telemetry`/`health`/`event` 写入 SQLite（`/data/data_agent/telemetry.db`，`telemetry_cache` 表），**仅当 `dr_cb` 投递证实才删行**；因此平台侧可能收到重复消息，去重由平台幂等消费保证。缓存上限 `cache_max_hours`(24h) 逐旧淘汰，恢复后按 `replay_batch_size`(50)/轮限流回放（防瞬时冲击）；若长时间断网，需人工确认库大小与丢弃量。
3. **指令只能走白名单，遥控超时必须交还**：未知指令一律 `REJECTED`；破坏性指令需 `confirm=true`；`remote_control` 断开后 `remote_agent` **停止发布** `/remote/command`（而非持续发零速），否则车辆不会回到 AUTO（见 §11.3）。
4. **联调判据只看平台侧 `last_online_time`**：车端日志“发送成功”不等于平台收到；`hunter-kafka-check` 退出码 0 且平台在线时刷新才算接入完成（§7.7）。首次接入需确认已**校时**（`timedatectl`）——证书校验对系统时间极为敏感。

### 13.10 上电自动启动链路（V0.1.06）

> ⚠️ **【现场运维视角】** 这一节回答一个具体故障：**“车端自检全绿、平台侧却永远收不到数据”**。
> 实测根因往往不是链路，而是两件事：① **车端根本没在跑**（上电后没人手工执行 `ip link` 与
> `ros2 launch`）；② **消息体不符 HunterCore 契约**（平台消费侧 `schema_name="auto"` 校验不通过，
> 直接进 DLQ，平台表 0 行）。本轮两条一起修。

**一、时序（systemd 依赖链，无需人工干预）**

```
上电
 └─ hunter-can.service        Type=oneshot + RemainAfterExit
      · modprobe gs_usb → 等待 can2 枚举（USB-CAN 晚于内核启动）
      · ip link set can2 down / type can bitrate 500000 / up
      · candump can2 抓帧：必须看到底盘反馈帧 0x211/0x221 才算“底盘通信建立”
      · 退出码：0 成功 / 2 接口始终没出现 / 3 接口 up 但无底盘帧（CAN_REQUIRE_FRAMES=1）
 └─ hunter-edge.service     Requires=hunter-can.service，After=hunter-can.service network-online.target
      · ExecStartPre：hunter_can_up.sh --quick（CAN 幂等复查，失败即整服务失败并重试）
      · ExecStart：hunter_edge_up.sh
         ① CAN 复查 → ② hunter-kafka-check（运营端预检，失败不阻断、落 SQLite 待回放）
         → ③ ros2 launch hunter_bringup hunter_edge.launch.py
              = 传感器/CAN 驱动 → 定位 → 感知 → Nav2/auto_mission（**自动驾驶准备状态**）
              + data_agent（telemetry 10Hz / health 1Hz / event）
              + command_agent（消费 command → 进入相应模式 → 回执 command_result）
 └─ ota-agent.service / remote-agent.service   平台侧交互（与 CAN 无依赖）
```

**二、关键设计取舍（改前必读）**

| 决策 | 原因 |
|---|---|
| CAN 复检放在**两个地方**（`hunter-can.service` + `hunter-edge.ExecStartPre`） | 上电瞬间 CAN 可能尚未枚举，单靠开机一次性 oneshot 容易“第一次就失败”；整栈启动前再复核一次，失败即由 `Restart=on-failure` 走完整时序重试 |
| 严格模式要求**抓到底盘反馈帧** | 只看 `ip link` 的 UP 标志会把“接线错/波特率错/底盘未上电”判成成功——此时 CAN 写入静默丢弃，车辆看着像在跑却不动（最难查的一类现场故障）。台架可用 `--no-strict` / `CAN_REQUIRE_FRAMES=0` |
| 运营端预检失败**不阻断**启动 | 断网时 `data_agent` 会把三类报文落 SQLite（`/data/data_agent/telemetry.db`），链路恢复后按序回放（至少一次）；若阻断启动，则连缓存都不会发生 |
| 整栈启动走 `hunter_edge.launch.py`（新文件）而非直接 `hunter_full.launch.py` | 生产默认 `use_autonomous_nav=true`（自动驾驶准备状态），调试默认是 `false`；薄封装避免改动既有调试入口 |
| `KillSignal=SIGINT` + `TimeoutStopSec=120` | ros2 launch / fast_lio2 需在退出瞬间写 PCD（建图模式）、data_agent 需 `flush` 残留消息；SIGTERM 截断会写出半截 PCD |
| 不新增任何 ROS 话题/消息 | `.ai-rules` 第 3 条；指令落地全部复用既有服务与话题 |

**三、消息体契约对齐（V0.1.06 修复；“平台 0 行”的直接原因）**

`data_agent` 与 `command_agent` 的输出**逐字段对齐** HunterCore 契约
（`contracts/kafka/schemas/*.schema.json`，`additionalProperties=false`）：

| Topic | 旧（不合契约） | 新（契约） |
|---|---|---|
| `telemetry` | 平铺 `velocity/angle/pose_x/fused_object_count/...` | 六段嵌套 `chassis / localization / perception / planning / control / system` + `seq` |
| `health` | `overall_status: OK\|NO_DATA` + 平铺 `cpu_usage` | `status`（8 态受控词表）+ `system{}` + `free_storage_mb` |
| `event` | `{type: hard_deceleration\|overspeed\|low_battery, level}` | `{event_type: harsh_acceleration\|harsh_braking\|harsh_turning\|over_speed\|battery_low\|battery_critical\|emergency_stop\|manual_takeover\|communication_loss, event_level: info\|warning\|critical, description, data}` |
| `command`（入向） | 只读 `type` / `issued_at` / `payload` | 读契约 `command_type` / `timestamp` / `params` / `timeout_ms`，**同时兼容**旧名 |
| `command_result`（出向） | `{type,status,reason,vehicle_state,ts}` | `{command_id, vehicle_id, timestamp, success, result_code, message, data}` |

> ⚠ 平台侧同一份契约也存在自身缺陷（`data-collector` 通配订阅未转正则 → 订阅到字面
> `hunter.*.telemetry` 而永远无消息；见 HunterCore V1.18.12 修复）。**车端契约对齐是必要条件，
> 不是充分条件**：两侧都修好，平台 `last_online_time` 与 `data_collector.vehicle_telemetry` 才会刷新。
>
> ⚠ 已知缺口（不臆造、不伪造）：契约 `health.gear`（OTA P 档门禁）与 `health.lat/lng`（地理围栏）
> 需要底盘档位与 GPS 数据源，当前 `ChassisState` 消息无对应字段（`.ai-rules` 禁止新增消息字段），
> 故**上报中缺省省略**；平台按契约描述“门禁缺数据即拒绝”的安全默认处理。

**四、验收判据（缺一不可）**

```bash
# 1) 开机链路：两个服务都是 active，且 hunter-can 有抓帧成功日志
systemctl is-active hunter-can hunter-edge ota-agent remote-agent
journalctl -u hunter-can -b --no-pager | grep -E '底盘反馈帧|退出码'

# 2) 运营端接入：退出码必须为 0（10 配置 / 20 证书 / 30 认证 / 40 网络 / 50 Topic / 60 投递）
hunter-kafka-check; echo "exit=$?"

# 3) 平台侧（唯一判据）：该车 last_online_time 刷新 + telemetry/health 有数据
#    HunterCore 侧：
#    docker exec hunter-postgres psql -U hunter -d hunter_core -c \
#      "SELECT count(*), max(time) FROM data_collector.vehicle_telemetry WHERE vehicle_id='HUNTER-001';"

# 4) 指令闭环：平台发 enable_auto_driving → 车端 journalctl 出 COMMAND_RESULT {... success:true}
#    且 /auto_mission/status 进入巡航态；重复发同一 command_id → message 含 [DUPLICATE]（不重复执行）
```

**五、实机避坑清单（V0.1.06 首轮实机反馈，改前必读）**

| 坑 | 现象 | 结论/处置 |
|---|---|---|
| `hunter_core_setup.sh` **不带 `--bundle` 重跑必挂**（V0.1.05 起就存在） | `× 接入包目录不存在:` 且 `$BUNDLE_DIR` 为空白 → 步骤③④⑤⑥全没跑（服务全 `inactive`、`hunter-kafka-check` 127） | **V0.1.06 已修**（校验块收进 `REUSE_BUNDLE=0`）；同步到车后重跑即可。根因：复用分支 `BUNDLE_DIR=""`，而 `[ ! -d "" ]` 恒真 |
| `agent_env.sh` **不能写行内注释** | 值变成 `nav   # nav=…` → `hunter-edge` 起来但**静默没进自动驾驶准备状态**（launch 参数非法） | **V0.1.06 已修**；该文件是 systemd `EnvironmentFile`，注释只能**独占一行** |
| `RMW_IMPLEMENTATION` 在 unit 里硬编码 | 开机自启的节点与登录终端里的 `ros2 node list`/`rviz2` **互相看不见**（两套 DDS） | **V0.1.06 已修**：单元不硬编码，改由 `agent_env.sh` 的 `HUNTER_RMW_IMPLEMENTATION` 继承（③ 自动探测运行用户登录 shell 的值；探测不到＝ROS 默认） |
| `ip -details link show` 的状态回显 | `can2 已启用：` 后面空白 | **V0.1.06 已修**：`bitrate` 与 `can state` 在输出里**分属两行**，必须分开 grep（`hunter_status.sh` 同步补 `can state`，bus-off 一眼可见） |
| **setuptools 入口包装找不到发行版元数据**（本轮最关键） | `command_agent` 起后即退（进程表里没有）；`ota-agent`/`remote-agent` 停在 `activating` 重启循环；`hunter-kafka-check` 退 1 带 `PackageNotFoundError`；`source install/setup.bash` 也不解决 | **V0.1.06 已修**。根因：`--symlink-install` 下 ament_python 走 `setup.py develop`，元数据留在源码目录并靠 `easy-install.pth` 注入——而 **`.pth` 对 PYTHONPATH 条目不生效**（setup.bash 提供的正是 PYTHONPATH）→ 模块能 import、元数据找不到。修法：三个 Python Agent 改走 `lib/<pkg>/<pkg>_node` **普通脚本**入口（与 `health_monitor` 的 ament_cmake 做法一致）；自检改为**源码方式优先**；命令包装加装 `/usr/local/bin`（免重登）；`hunter_bringup/scripts/*.sh` 与三个 `*_node` 在 git 里记为 `100755` 并新增 ④.1 可执行位自愈 |
| 脚本可执行位 | `./hunter_status.sh: Permission denied` | **V0.1.06 已修**（git `chmod=+x` + 部署脚本 ④.1 自愈）；临时也可 `bash <脚本路径>` 直接跑 |
| **`command_agent` 起来后立刻退出**（`exit code 1`，`AttributeError: 'CommandAgent' object has no attribute '_health_cb'`） | `__init__` 里 `create_subscription` 引用的四个回调（`_health_cb`/`_chassis_cb`/`_odom_cb`/`_mission_cb`）**类里不存在** → 每次启动都崩在第一个订阅 | **V0.1.07 已修**（补齐回调）。判据：`ros2 node list \| grep command_agent` 有输出；日志出现「command_agent 启动：… 指令链路=Kafka 已就绪」 |
| **`ota-agent` 恒 `activating`**（重启循环，`journalctl` 只见反复重启、无 ERROR 级信息） | `ota_agent.main()` 漏了 `args = parser.parse_args()` → `NameError: name 'args' is not defined` → 非零退出 → `Restart=on-failure` 重启循环（`is-active` 显示 `activating`） | **V0.1.07 已修**。判据：`systemctl is-active ota-agent` = `active`；`journalctl -u ota-agent -n 20` 出现「OTA Agent 启动」 |
| **事件风暴 / 磁盘被 rosbag 写满**（`pgrep -af 'ros2 bag record'` 出现多个进程；同型事件刷屏） | `data_agent.detectEvents` 为 100ms 无状态判定：低电/超速/急转弯/急停在持续期间**每拍命中** → 每秒约 10 条重复事件，且每条拉起一个 `ros2 bag record` | **V0.1.07 已修**：`reportEvent` 加边沿触发抑制期（新参数 `event_min_interval`，默认 10s）。验证：`pgrep -af 'ros2 bag record' \| wc -l` 应≤1，事件不再成批 |
| 平台看不到 OTA 进度 / 遥控事件被判非法 | `ota_status` 发 `{state, detail}`、`event` 发 `{type, level}` + 自造事件类型 → 均不合契约（`additionalProperties=false`） | **V0.1.07 已修**：改契约字段（`status`/`phase`/`progress`；`event_type`/`event_level`），遥控只上报受控词表内类型（`manual_takeover`/`emergency_stop`），其余仅本地日志 |
| **`ota-agent`/`remote-agent` 报 `status=127`**（`/bin/bash: …/install/lib/<pkg>/<pkg>_node: No such file or directory`，`is-active` 显示 `activating`，每 10s 重启一轮） | unit 按 **`--merge-install`** 布局拼路径 `<install>/lib/<pkg>/`，而本工作空间是 colcon **默认的隔离安装**（真实路径 `<install>/<pkg>/lib/<pkg>/<pkg>_node`） | **V0.1.08 已修**：unit 改「隔离安装 → 合并安装 → 源码树」**三候选探测**，找不到时打印候选路径 + 处置命令；**127 一律先看路径，别看业务**。判据：`systemctl is-active ota-agent remote-agent` = `active`；`ls install/<pkg>/lib/<pkg>/<pkg>_node` 存在 |
| **`command_agent`/`ota_agent` 崩在 `_init_kafka`**：`KafkaError{code=_INVALID_ARG, str="Property "dr_cb" must be set through dedicated .._set_..() function"}` | `hunter_kafka.make_producer()` 把回调塞进配置字典（`conf["dr_cb"]=cb`），而 **本库 confluent-kafka 2.16.0 根本没有 `dr_cb` 回调 kwarg**——本机同版本实测：`Producer(conf, dr_cb=cb)`／`{**conf,"dr_cb":cb}`／`p.dr_cb=cb` **三种写法全部失败**，可用回调只有 `error_cb`/`stats_cb`/`throttle_cb`/`logger`，**投递证实唯一入口是 `produce(..., on_delivery=cb)`** | **V0.1.10 已修**：`_ProducerWithDeliveryReport` 包装类在 `produce()` 上自动挂 `on_delivery`（调用方显式传则尊重），对外语义仍是"构造传回调 → 每条消息回调"。⚠ **`hunter-kafka-check` 不传回调，所以自检全绿也照不出这个坑**：Agent 起不来而自检 0 时优先怀疑这里。判据：`journalctl -u ota-agent`/`-u hunter-edge` 不再出现 `_INVALID_ARG` |
| **`ParameterUninitializedException: The parameter 'extra_allowed_types' is not initialized`**（command_agent 启动即崩） | 现场 YAML 写的是 `extra_allowed_types: []`（部署脚本按约定保留现场配置、不覆盖）→ `declare_parameter(name, [])` 在 rclpy 下**无法推断元素类型 → 空数组落成"未初始化"**，`get_parameter(...).value` 直接抛 | **V0.1.10 已修**：新增 `_param_string_list(name)` 捕获该异常/`None` → 返回 `[]`；`extra_allowed_types` 与 `command_type_aliases` 统一走它。**通用口径：字符串数组参数一律用该 helper 读取** |
| `systemctl restart remote-agent` 卡很久且 `Failed with result 'timeout'`（日志一串 `Killing process … SIGKILL`） | 默认 `KillSignal=SIGTERM` 让 Python **直接终止、不走 `finally`** → GStreamer 子进程残留 → systemd 等满 `TimeoutStopSec`(90s) 再强杀整个 cgroup | **V0.1.10 已修**：两个 unit 加 `KillSignal=SIGINT`（Python 抛 KeyboardInterrupt → `finally` 里 `stop_gstreamer()`/`shutdown()`）+ `KillMode=mixed` + `TimeoutStopSec=20`。判据：`systemctl restart remote-agent` **秒级返回** |
| **`TypeError: RcutilsLogger.fatal() takes 2 positional arguments but 3 were given`**（且它出现在 `except` 分支里，**把真错盖掉**） | 本 rclpy 版本的 `RcutilsLogger` 方法**只接受单个 message**，不支持 printf 风格位置参数；`command_agent.py` 有 17 处 `get_logger().X("...%s", args)`（仓库内运行正常的 rclpy 节点 `pcd_to_map`/`fast_lio2_param_injector` 全部用 f-string 单参数） | **V0.1.09 已修**：17 处改为 `%` 预格式化。写作口径：`get_logger().info("...%s" % (x,))` 或 f-string，**不要**再写 `info("...%s", x)` |
| 日志刷 `CONFWARN … is a producer property and will be ignored by this consumer instance` / `[ERROR] websocket-client 不可用` 每 5s | 消费者 conf 混入生产者键（librdkafka 报**规范名**：`queue.buffering.max.ms`/`message.send.max.retries`/`compression.codec`/`batch.size`/`request.required.acks`）；`websocket-client` 未安装（WS 只是备用通道） | **V0.1.09 已修**：`build_client_conf()` 按 `role` 分流（consumer 不写生产者键）；WS 缺包只提示一次并给出补装命令，`hunter_core_setup.sh` 步骤① 自动 `pip3 install websocket-client` |

---

## 14. 文档索引

关联文档如下：

| 文档 | 说明 |
|------|------|
| 《自动驾驶车辆系统详细设计文档 V2.0》 | 本项目的设计基准；本文档全部参数、话题、CAN 协议、坐标系均可追溯至其对应章节 |
| AI 编码任务清单 | 分模块开发任务（任务 00 ~ 任务 17），指导按模块开发与验收 |
| `User_Manual.md` | 面向现场运维人员的用户手册（独立文档，含详细部署/联调/故障排查/自主导航操作流程，**V2.7** 对应软件基线 V0.1.05） |
| `Deployment_Guide.md` | 部署操作文档 **V2.7**（环境要求/环境配置/环境安装/源码部署/功能操作步骤/异常处理全流程，对应软件基线 V0.1.05；HunterCore 接入全流程见 §5.6） |
| `release.md` | 版本历史（V0.0.1 ~ 当前），记录每版主要功能与修复 |

> **追溯原则**：本 README 中所有硬件参数（§2）、软件版本（§3）、话题（§8）、控制模式（§9）、限制（§13）均源自《自动驾驶车辆系统详细设计文档 V2.0》，未虚构功能。自主导航模块（§10）为在设计文档框架内的扩展实现。

---

*HunterEdge 开发指南 · 文档版本 V2.12 · 编制依据《自动驾驶车辆系统详细设计文档 V2.0》，并含 V0.0.67~V0.1.10 现场实测修正（**V0.1.10：投递证实改 `on_delivery` 包装 + rclpy 空数组参数容错 + 服务停止语义**——本库 confluent-kafka 2.16.0 **无 `dr_cb` 回调 kwarg**（本机同版本实测四种写法：构造 kwarg／conf 键／属性赋值全失败，`error_cb`/`stats_cb`/`throttle_cb`/`logger` 可用，仅 `produce(on_delivery=)` 可用）→ `_ProducerWithDeliveryReport` 包装类；rclpy 下 `extra_allowed_types: []` 落成"未初始化" → `_param_string_list()` 容错；unit 加 `KillSignal=SIGINT`+`TimeoutStopSec=20` 消除停止超时被 SIGKILL。附 11 项回归断言（含真实 API 冒烟），详见 §13.10 与 release.md V0.1.10；**V0.1.09：三处阻塞修复（`dr_cb` 传参 / rclpy 日志参数 / 消费者配置噪声）**——`dr_cb` 不是 librdkafka 配置项（必须 `Producer(conf, dr_cb=cb)`），旧写法致 `command_agent`/`ota_agent` 启动即崩 `_INVALID_ARG`；`command_agent` 17 处 `get_logger().X("...%s", args)` 属 printf 风格（本 rclpy 版本只收单 message）致 `TypeError`，且把真因盖在错误处理里；消费者 conf 混入生产者键致 CONFWARN；`websocket-client` 缺失的每 5s ERROR 改一次性提示并补进部署依赖。附 `dr_cb`/配置回归测试 14 项断言（§13.10、release.md V0.1.09）；**V0.1.08：systemd 可执行路径修复（隔离安装布局）**——`ota-agent`/`remote-agent` 的 unit 旧按 `--merge-install` 拼路径 → 实机 `status=127 No such file or directory` 且每 10s 重启；改为「隔离安装 → 合并安装 → 源码树」三候选探测，并新增"抽取 unit 内层 bash 跑 `bash -n`"自检（6 处全 PASS）。**排障口径：127 先看路径，不看业务**（§13.10、release.md V0.1.08）；**V0.1.07：车端 Agent「启动崩溃」修复 + 契约等级/字段/风暴对齐**——`command_agent` 四个订阅回调缺失致启动即崩（**自 V0.1.05 起从未成功运行**）、`ota_agent.main()` 漏 `args = parser.parse_args()` 致恒 `activating` 重启循环、事件等级 `over_speed`/`communication_loss` 校正为 `critical`、事件风暴与 `ros2 bag record` 泛滥加 `event_min_interval`（10s）抑制、`ota_status`/遥控事件/遥控帧三处契约字段对齐；新增 AST/ruff/降级冒烟三重防复发，详见 §13.10 与 release.md V0.1.07；**V0.1.06：上电自启链路 + Kafka 消息体契约对齐**——新增 `hunter_can_up.sh`/`hunter_edge_up.sh`/`hunter_edge.launch.py` 与 `hunter-can`/`hunter-edge` 两个 systemd 单元，实现“上电 → 底盘 CAN（抓帧验证）→ 整栈启动 → 自动驾驶准备状态 → 运营端上报 → 平台指令进入模式并回执”的全自动链路；`data_agent` 上行 telemetry/health/event 与 `command_agent` 上下行**逐字段对齐** HunterCore `contracts/kafka/schemas/*.schema.json`（旧版平铺字段被平台判非法 → DLQ，是运营端 0 行的直接原因）；详见 §7.8 / §7.9 / §13.10 与 release.md V0.1.06；V0.0.93：方案A 定位架构重构；V0.0.94：“原地不动”残余故障链修复；V0.0.95：“无法绕开障碍物”分层修复；V0.0.96：“行驶一小段立即停下”修复；V0.0.97：阿克曼绕障几何死锁 + 过期 goal 修复；V0.0.98：safety_guard 轨迹扫掠弧碰撞闸、MPPI 4.0s 预测时域与 critic 重标定、auto_mission 取消静置门控、/scan 链降载；V0.0.99：阿克曼几何参数一致性修正（轴距 0.65→0.46、调试 launch 与生产同步）；**V0.1.00：`auto_mission` 任务层「自愈四件套」——逐航点失败隔离与自动轮转（单点超时不再锁存整条任务）、车身四周/脚底假障碍自动诊断（局部代价地图 × 激光 × 相机）、航点可达性按静态地图可通行连通域判定、FAULT 由永久锁存改自愈态（接管后切回 AUTO 即自动复驶）；新增 23 个参数，`nav2_params.yaml` `local_costmap.always_send_full_costmap: true`；不新增任何话题/消息/服务；重编 `auto_mission hunter_bringup`）；**V0.1.01：health_monitor「相机崩溃/重启风暴」误报修复——频率看门狗将 15Hz 标称帧率在系统过载下的正常抖动误判为进程崩溃（`checkNodes` 不 respawn），`camera_min_rate` 10→5Hz + `checkNodes` 措辞去误导（“疑似崩溃/重启”→“频率异常/恢复”，逻辑与对外语义不变），仅重编 `hunter_monitor`）；**V0.1.02：按《HUNTER SE 低速自动驾驶避障解决方案》逐条对表落地——审计确认避障主链（扫掠弧闸/REEDS_SHEPP/MPPI 4.0s/自愈四件套/BT/五级预检）已全部在位，本轮只补 5 处量化参数：全局 `inflation_radius 0.55` + 全局 `update_frequency 1.0` + 局部 `6m×6m @5Hz`（局部图算力 ≈1/5.6）、`lidar_perception` 新增可配 `ground_max_slope 5.0°` 与 `outlier_mean_k 50`、`cluster_tolerance 0.15`、`sensor_fusion.vision_conf_min 0.50`、`safety_guard.max_linear_vel` 默认 `0.5`；6 项有意保留偏差写明理由（§10.10 与 Deployment_Guide §5.5.2）、避障 10 项验收见 Deployment_Guide §5.7；重编 `hunter_safety hunter_bringup lidar_perception`）；**V0.1.03：修复「自主导航刚起步即停下」——NDT 重定位 `alignOnce` 跟踪/置信度彻底解耦，位姿刷新只由跳变闸门 `step_ok` 决定、`fitness_hard_ceiling` 不再冻结位姿（仅调节置信度），`fitness_max 2.5→3.5`、`fitness_hard_ceiling 3.0→5.0`，仅重编 `hunter_relocalization`）；**V0.1.04：车端 Kafka 安全认证配置落地**——Broker `120.202.73.105:9093`（`SASL_SSL + SCRAM-SHA-512`），统一配置到 data_agent/remote_agent/ota_agent 三个模块（该版凭据仍写在 YAML，**V0.1.05 已改为接入包单一可信源**）；**V0.1.05：HunterCore 车端接入全链打通**——以平台下发的接入包为唯一可信源（`/etc/hunter/kafka/kafka.properties` + CA/客户端证书，SASL_SSL 上叠加 **mTLS**、主机名校验 `https`），补齐 8 Topic 契约中此前沿未打通的四条断链：`health`(1Hz)、`command`/`command_result`（**新增 `command_agent` 包**，白名单/TTL/幂等/超时/confirm 五道护栏）、`remote_control`（`remote_agent` 改为 Kafka 消费驱动，并修复“遥控超时后永不交还 AUTO”缺陷）；`acks` 按 Topic 分档（telemetry/health=1，event/command_result/ota_status=all）、SQLite 加 `topic` 列做三类消息统一断点续传（**投递证实才删行**）、消费端**手动提交 offset**、OTA 按 `task_id` **幂等**；四份 `*_params.yaml` **凭据出仓**；新增 `hunter_kafka` 公共库 + `hunter-kafka-check` 六层自检 CLI + `hunter_core_setup.sh` 一键部署（见 §7.7 / §11.3 / §13.9 与 Deployment_Guide §5.6）；重编 `hunter_msgs hunter_kafka command_agent data_agent ota_agent remote_agent hunter_bringup`）*

