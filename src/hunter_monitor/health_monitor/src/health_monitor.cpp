// Copyright 2026 HUNTER Development Team
// 系统监控健康管理实现（文档第 15 章）
#include "health_monitor/health_monitor.hpp"

#include <sys/statvfs.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>

namespace health_monitor
{

namespace
{
// 传感器话题频率监控（文档 15.3：低于阈值持续 duration 秒判定异常）
void checkTopicFrequency(
  TopicMonitor & mon, const rclcpp::Time & now, rclcpp::Logger logger)
{
  const double dt = (now - mon.window_start).seconds();
  if (dt < 1.0) {
    return;
  }
  const double freq = mon.msg_count / dt;
  mon.msg_count = 0;
  mon.window_start = now;

  if (freq < mon.min_rate) {
    if (!mon.anomaly) {
      mon.anomaly = true;
      mon.anomaly_since = now.seconds();
    } else if ((now.seconds() - mon.anomaly_since) > mon.duration) {
      RCLCPP_WARN_THROTTLE(
        logger, now, 3000, "%s 话题频率异常：%.1fHz < %.1fHz，持续超 %.0fs",
        mon.name.c_str(), freq, mon.min_rate, mon.duration);
    }
  } else {
    if (mon.anomaly) {
      RCLCPP_INFO(logger, "%s 话题恢复正常（%.1fHz）", mon.name.c_str(), freq);
    }
    mon.anomaly = false;
    mon.anomaly_since = -1.0;
  }
}
}  // namespace

HealthMonitor::HealthMonitor(const rclcpp::NodeOptions & options)
: rclcpp::Node("health_monitor", options),
  prev_cpu_total_(0),
  prev_cpu_idle_(0),
  cpu_sampled_(false)
{
  // 参数（文档 15.2/15.3/3.5）
  declare_parameter("critical_nodes", std::vector<std::string>{
    "lidar_perception", "vision_perception", "fast_lio2", "ekf_filter_node",
    "hunter_ros2", "decision_making"});
  declare_parameter("node_check_interval", 2.0);
  declare_parameter("report_interval", 1.0);
  declare_parameter("restart_limit", 3);        // 文档 15.2
  declare_parameter("restart_window", 300.0);   // 5 分钟
  declare_parameter("warning_temp", 85.0);      // 文档 3.5
  declare_parameter("critical_temp", 95.0);     // 文档 3.5
  // 传感器频率阈值（文档 15.3）
  declare_parameter("lidar_min_rate", 8.0);
  declare_parameter("lidar_duration", 2.0);
  declare_parameter("camera_min_rate", 10.0);
  declare_parameter("camera_duration", 2.0);
  declare_parameter("imu_min_rate", 50.0);
  declare_parameter("imu_duration", 1.0);
  declare_parameter("can_min_rate", 5.0);
  declare_parameter("can_duration", 1.0);

  critical_nodes_ = get_parameter("critical_nodes").as_string_array();
  node_check_interval_ = get_parameter("node_check_interval").as_double();
  report_interval_ = get_parameter("report_interval").as_double();
  restart_limit_ = static_cast<int>(get_parameter("restart_limit").as_int());
  restart_window_ = get_parameter("restart_window").as_double();
  warning_temp_ = get_parameter("warning_temp").as_double();
  critical_temp_ = get_parameter("critical_temp").as_double();

  const rclcpp::Time now = this->now();
  // 传感器频率监控（文档 15.3 阈值：LiDAR<8Hz/2s、Camera<10Hz/2s、IMU<50Hz/1s、CAN<5Hz/1s）
  lidar_mon_ = {"lidar",
    get_parameter("lidar_min_rate").as_double(),
    get_parameter("lidar_duration").as_double(), 0, now, false, -1.0};
  camera_mon_ = {"camera",
    get_parameter("camera_min_rate").as_double(),
    get_parameter("camera_duration").as_double(), 0, now, false, -1.0};
  imu_mon_ = {"imu",
    get_parameter("imu_min_rate").as_double(),
    get_parameter("imu_duration").as_double(), 0, now, false, -1.0};
  can_mon_ = {"can",
    get_parameter("can_min_rate").as_double(),
    get_parameter("can_duration").as_double(), 0, now, false, -1.0};

  // 订阅传感器话题（频率监控）
  lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "/lidar_points", rclcpp::SensorDataQoS(),
    std::bind(&HealthMonitor::lidarCallback, this, std::placeholders::_1));
  camera_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "/camera/color/image_raw", rclcpp::SensorDataQoS(),
    std::bind(&HealthMonitor::cameraCallback, this, std::placeholders::_1));
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data", rclcpp::SensorDataQoS(),
    std::bind(&HealthMonitor::imuCallback, this, std::placeholders::_1));
  chassis_sub_ = create_subscription<hunter_msgs::msg::ChassisState>(
    "/chassis/state", rclcpp::SensorDataQoS(),
    std::bind(&HealthMonitor::chassisCallback, this, std::placeholders::_1));

  // 发布
  health_pub_ =
    create_publisher<hunter_msgs::msg::SystemHealth>("/system/health", 10);
  estop_pub_ = create_publisher<std_msgs::msg::Bool>("/estop", 10);

  // 定时器
  health_timer_ = create_wall_timer(
    std::chrono::duration<double>(report_interval_),
    std::bind(&HealthMonitor::checkHealth, this));
  node_timer_ = create_wall_timer(
    std::chrono::duration<double>(node_check_interval_),
    std::bind(&HealthMonitor::checkNodes, this));

  RCLCPP_INFO(get_logger(), "health_monitor 启动：上报 1Hz，节点检查 %.0fs",
    node_check_interval_);
}

void HealthMonitor::lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr)
{
  lidar_mon_.msg_count++;
}

void HealthMonitor::cameraCallback(const sensor_msgs::msg::Image::SharedPtr)
{
  camera_mon_.msg_count++;
}

void HealthMonitor::imuCallback(const sensor_msgs::msg::Imu::SharedPtr)
{
  imu_mon_.msg_count++;
}

void HealthMonitor::chassisCallback(const hunter_msgs::msg::ChassisState::SharedPtr)
{
  can_mon_.msg_count++;
}

void HealthMonitor::checkHealth()
{
  const rclcpp::Time now = this->now();

  // 1. 传感器频率监控（文档 15.3）
  checkTopicFrequency(lidar_mon_, now, get_logger());
  checkTopicFrequency(camera_mon_, now, get_logger());
  checkTopicFrequency(imu_mon_, now, get_logger());
  checkTopicFrequency(can_mon_, now, get_logger());

  // 2. 资源采集
  const double cpu = readCpuUsage();
  const double mem = readMemoryUsage();
  const double disk = readDiskUsage();
  const double cpu_temp = readTemp("/sys/class/thermal/thermal_zone0/temp");
  const double gpu_temp = readTemp("/sys/class/thermal/thermal_zone1/temp");

  // 3. 降级策略（文档 15.4）
  std::string overall = "OK";
  // CAN 通信中断 → 严重告警 + 安全停车
  if (can_mon_.anomaly) {
    overall = "CRITICAL";
    std_msgs::msg::Bool estop;
    estop.data = true;
    estop_pub_->publish(estop);
    RCLCPP_WARN_THROTTLE(get_logger(), now, 3000, "CAN 通信中断，触发安全停车");
  }
  // 传感器异常 → 告警降级
  if (lidar_mon_.anomaly || camera_mon_.anomaly || imu_mon_.anomaly) {
    if (overall != "CRITICAL") {
      overall = "WARNING";
    }
  }
  // 温度：超过 95℃ 降载，超过 85℃ 告警（文档 3.5）
  if (cpu_temp > critical_temp_ || gpu_temp > critical_temp_) {
    overall = "CRITICAL";
  } else if (cpu_temp > warning_temp_ || gpu_temp > warning_temp_) {
    if (overall != "CRITICAL") {
      overall = "WARNING";
    }
  }

  // 4. 发布 /system/health（1Hz）
  hunter_msgs::msg::SystemHealth msg;
  msg.header.stamp = now;
  msg.cpu_usage = static_cast<float>(cpu);
  msg.gpu_usage = 0.0f;  // GPU 使用率（可用 nvml/tegrastats 扩展）
  msg.memory_usage = static_cast<float>(mem);
  msg.disk_usage = static_cast<float>(disk);
  msg.cpu_temp = static_cast<float>(cpu_temp);
  msg.gpu_temp = static_cast<float>(gpu_temp);
  msg.overall_status = overall;
  msg.node_names = {"lidar", "camera", "imu", "can"};
  msg.node_states = {
    lidar_mon_.anomaly ? "fault" : "running",
    camera_mon_.anomaly ? "fault" : "running",
    imu_mon_.anomaly ? "fault" : "running",
    can_mon_.anomaly ? "fault" : "running"};
  health_pub_->publish(msg);
}

void HealthMonitor::checkNodes()
{
  // 节点存活监控 + 重启计数（文档 15.2）
  // 通过传感器话题活跃度推断节点存活状态
  const std::vector<std::pair<std::string, bool>> node_status = {
    {"lidar", !lidar_mon_.anomaly},
    {"camera", !camera_mon_.anomaly},
    {"imu", !imu_mon_.anomaly},
    {"can", !can_mon_.anomaly},
  };

  for (const auto & [name, alive] : node_status) {
    auto it = node_was_alive_.find(name);
    const bool was_alive = (it != node_was_alive_.end()) ? it->second : true;

    if (!alive && was_alive) {
      RCLCPP_WARN(get_logger(), "%s 节点/话题异常（疑似崩溃）", name.c_str());
    } else if (alive && !was_alive) {
      // 异常恢复 → 视为一次重启
      restart_count_[name]++;
      RCLCPP_WARN(
        get_logger(), "%s 节点重启（第 %d 次，阈值 %d 次）",
        name.c_str(), restart_count_[name], restart_limit_);
      if (restart_count_[name] > restart_limit_) {
        // 文档 15.2：5 分钟内重启超过 3 次 → 标记故障，不再重启，上报
        RCLCPP_ERROR(
          get_logger(), "%s 节点在 %.0fs 内重启超过 %d 次，标记为故障",
          name.c_str(), restart_window_, restart_limit_);
      }
    }
    node_was_alive_[name] = alive;
  }

  // 关键节点故障触发安全停车（文档 15.2：hunter_ros2/navigation 故障）
  if (can_mon_.anomaly) {
    std_msgs::msg::Bool estop;
    estop.data = true;
    estop_pub_->publish(estop);
  }
}

double HealthMonitor::readCpuUsage()
{
  std::ifstream file("/proc/stat");
  if (!file.good()) {
    return 0.0;
  }
  std::string line;
  std::getline(file, line);
  std::istringstream iss(line);
  std::string label;
  unsigned long long user = 0, nice = 0, system = 0, idle = 0, iowait = 0;
  unsigned long long irq = 0, softirq = 0, steal = 0;
  iss >> label >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal;

  const unsigned long long total =
    user + nice + system + idle + iowait + irq + softirq + steal;
  const unsigned long long idle_total = idle + iowait;

  if (!cpu_sampled_) {
    prev_cpu_total_ = total;
    prev_cpu_idle_ = idle_total;
    cpu_sampled_ = true;
    return 0.0;  // 首次采样，无增量
  }

  const double total_delta = static_cast<double>(total - prev_cpu_total_);
  const double idle_delta = static_cast<double>(idle_total - prev_cpu_idle_);
  prev_cpu_total_ = total;
  prev_cpu_idle_ = idle_total;

  if (total_delta <= 0.0) {
    return 0.0;
  }
  return (1.0 - idle_delta / total_delta) * 100.0;
}

double HealthMonitor::readMemoryUsage()
{
  std::ifstream file("/proc/meminfo");
  if (!file.good()) {
    return 0.0;
  }
  std::string key;
  unsigned long long value = 0;
  std::string unit;
  unsigned long long total = 0;
  unsigned long long available = 0;
  std::string line;
  while (std::getline(file, line)) {
    std::istringstream iss(line);
    iss >> key >> value >> unit;
    if (key == "MemTotal:") {
      total = value;
    } else if (key == "MemAvailable:") {
      available = value;
    }
  }
  if (total == 0) {
    return 0.0;
  }
  return (1.0 - static_cast<double>(available) / total) * 100.0;
}

double HealthMonitor::readDiskUsage()
{
  struct statvfs st;
  if (statvfs("/", &st) != 0) {
    return 0.0;
  }
  const double total = static_cast<double>(st.f_blocks * st.f_frsize);
  const double free = static_cast<double>(st.f_bfree * st.f_frsize);
  if (total <= 0.0) {
    return 0.0;
  }
  return (1.0 - free / total) * 100.0;
}

double HealthMonitor::readTemp(const std::string & zone)
{
  std::ifstream file(zone);
  if (!file.good()) {
    return 0.0;
  }
  int temp_milli = 0;
  file >> temp_milli;
  return temp_milli / 1000.0;  // 毫摄氏度 → 摄氏度
}

}  // namespace health_monitor


