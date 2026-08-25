// Copyright 2026 HUNTER Development Team
// 多传感器目标级数据融合实现（文档第 6 章）
#include "sensor_fusion/sensor_fusion.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace sensor_fusion
{

namespace
{
// 类别一致性：相同，或任一方为通用 "obstacle"（激光粗分类）
bool classConsistent(const std::string & a, const std::string & b)
{
  return a == b || a == "obstacle" || b == "obstacle";
}
}  // namespace

SensorFusion::SensorFusion(const rclcpp::NodeOptions & options)
: rclcpp::Node("sensor_fusion", options),
  lidar_received_(false),
  vision_received_(false),
  last_lidar_time_(std::chrono::steady_clock::now()),
  last_vision_time_(std::chrono::steady_clock::now())
{
  // 参数（文档 6.2/6.3/6.5）
  declare_parameter("lidar_topic", "/perception/lidar_objects");
  declare_parameter("vision_topic", "/perception/vision_objects");
  declare_parameter("fused_topic", "/perception/fused_objects");
  declare_parameter("freespace_topic", "/perception/freespace");
  declare_parameter("match_distance", 2.0);       // 文档 6.2：匹配阈值
  declare_parameter("pos_lidar_weight", 0.7);     // 文档 6.3：位置激光权重
  declare_parameter("pos_vision_weight", 0.3);
  declare_parameter("size_lidar_weight", 0.8);    // 文档 6.3：尺寸激光权重
  declare_parameter("size_vision_weight", 0.2);
  declare_parameter("freespace_resolution", 0.2); // 文档 6.5
  declare_parameter("freespace_length", 40.0);    // 车前 40m
  declare_parameter("freespace_width", 30.0);     // 左右各 15m
  declare_parameter("inflation_radius", 0.3);     // 膨胀 0.3m
  declare_parameter("lidar_timeout", 0.3);
  declare_parameter("vision_timeout", 0.5);

  lidar_topic_ = get_parameter("lidar_topic").as_string();
  vision_topic_ = get_parameter("vision_topic").as_string();
  fused_topic_ = get_parameter("fused_topic").as_string();
  freespace_topic_ = get_parameter("freespace_topic").as_string();
  match_distance_ = get_parameter("match_distance").as_double();
  pos_lidar_weight_ = get_parameter("pos_lidar_weight").as_double();
  pos_vision_weight_ = get_parameter("pos_vision_weight").as_double();
  size_lidar_weight_ = get_parameter("size_lidar_weight").as_double();
  size_vision_weight_ = get_parameter("size_vision_weight").as_double();
  freespace_resolution_ = get_parameter("freespace_resolution").as_double();
  freespace_length_ = get_parameter("freespace_length").as_double();
  freespace_width_ = get_parameter("freespace_width").as_double();
  inflation_radius_ = get_parameter("inflation_radius").as_double();
  lidar_timeout_ = get_parameter("lidar_timeout").as_double();
  vision_timeout_ = get_parameter("vision_timeout").as_double();

  lidar_sub_ = create_subscription<hunter_msgs::msg::DetectedObjectArray>(
    lidar_topic_, rclcpp::SensorDataQoS(),
    std::bind(&SensorFusion::lidarCallback, this, std::placeholders::_1));
  vision_sub_ = create_subscription<hunter_msgs::msg::DetectedObjectArray>(
    vision_topic_, rclcpp::SensorDataQoS(),
    std::bind(&SensorFusion::visionCallback, this, std::placeholders::_1));

  fused_pub_ =
    create_publisher<hunter_msgs::msg::DetectedObjectArray>(fused_topic_, 10);
  freespace_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(freespace_topic_, 10);

  timeout_timer_ = create_wall_timer(
    std::chrono::milliseconds(1000),
    std::bind(&SensorFusion::checkTimeout, this));

  RCLCPP_INFO(get_logger(), "sensor_fusion 启动：匹配阈值=%.1fm, 权重=%.1f/%.1f/%.1f/%.1f",
    match_distance_, pos_lidar_weight_, pos_vision_weight_,
    size_lidar_weight_, size_vision_weight_);
}

void SensorFusion::lidarCallback(
  const hunter_msgs::msg::DetectedObjectArray::SharedPtr msg)
{
  lidar_cache_ = *msg;
  lidar_stamp_ = msg->header.stamp;
  lidar_received_ = true;
  last_lidar_time_ = std::chrono::steady_clock::now();
  fuseAndPublish();  // 激光帧驱动融合（10Hz）
}

void SensorFusion::visionCallback(
  const hunter_msgs::msg::DetectedObjectArray::SharedPtr msg)
{
  vision_cache_ = *msg;
  vision_stamp_ = msg->header.stamp;
  vision_received_ = true;
  last_vision_time_ = std::chrono::steady_clock::now();
}

void SensorFusion::checkTimeout()
{
  auto now = std::chrono::steady_clock::now();
  if (lidar_received_) {
    const double elapsed =
      std::chrono::duration<double>(now - last_lidar_time_).count();
    if (elapsed > lidar_timeout_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "激光目标超时：%.2fs 未收到 %s，降级为纯视觉", elapsed, lidar_topic_.c_str());
    }
  }
  if (vision_received_) {
    const double elapsed =
      std::chrono::duration<double>(now - last_vision_time_).count();
    if (elapsed > vision_timeout_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "视觉目标超时：%.2fs 未收到 %s，降级为纯激光", elapsed, vision_topic_.c_str());
    }
  }
}

namespace
{
// 标准匈牙利算法（最小代价指派，O(n³)）
std::vector<int> hungarian(const std::vector<std::vector<double>> & cost)
{
  const int n = static_cast<int>(cost.size());
  if (n == 0) {
    return {};
  }
  const double INF = std::numeric_limits<double>::max();
  std::vector<double> u(n + 1, 0.0), v(n + 1, 0.0);
  std::vector<int> p(n + 1, 0), way(n + 1, 0);

  for (int i = 1; i <= n; ++i) {
    p[0] = i;
    int j0 = 0;
    std::vector<double> minv(n + 1, INF);
    std::vector<bool> used(n + 1, false);
    do {
      used[j0] = true;
      const int i0 = p[j0];
      int j1 = 0;
      double delta = INF;
      for (int j = 1; j <= n; ++j) {
        if (!used[j]) {
          const double cur = cost[i0 - 1][j - 1] - u[i0] - v[j];
          if (cur < minv[j]) {
            minv[j] = cur;
            way[j] = j0;
          }
          if (minv[j] < delta) {
            delta = minv[j];
            j1 = j;
          }
        }
      }
      for (int j = 0; j <= n; ++j) {
        if (used[j]) {
          u[p[j]] += delta;
          v[j] -= delta;
        } else {
          minv[j] -= delta;
        }
      }
      j0 = j1;
    } while (p[j0] != 0);

    do {
      const int j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0 != 0);
  }

  std::vector<int> assignment(n, -1);
  for (int j = 1; j <= n; ++j) {
    if (p[j] > 0) {
      assignment[p[j] - 1] = j - 1;
    }
  }
  return assignment;
}
}  // namespace

std::vector<int> SensorFusion::associate(
  const std::vector<hunter_msgs::msg::DetectedObject> & lidar,
  const std::vector<hunter_msgs::msg::DetectedObject> & vision)
{
  const int n = static_cast<int>(lidar.size());
  const int m = static_cast<int>(vision.size());
  const int N = std::max(n, m);
  if (N == 0) {
    return {};
  }

  // 关联代价：位置距离 < 2.0m 且类别一致（文档 6.2）
  std::vector<std::vector<double>> cost(N, std::vector<double>(N, 1e6));
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < m; ++j) {
      const double dx = lidar[i].pose.position.x - vision[j].pose.position.x;
      const double dy = lidar[i].pose.position.y - vision[j].pose.position.y;
      const double d = std::sqrt(dx * dx + dy * dy);
      if (d < match_distance_ && classConsistent(lidar[i].class_name, vision[j].class_name)) {
        cost[i][j] = d;
      }
    }
  }
  return hungarian(cost);
}

void SensorFusion::fuseAndPublish()
{
  const rclcpp::Time now = this->now();
  const bool lidar_valid =
    lidar_received_ && (now - lidar_stamp_).seconds() < lidar_timeout_;
  const bool vision_valid =
    vision_received_ && (now - vision_stamp_).seconds() < vision_timeout_;

  if (!lidar_valid && !vision_valid) {
    return;  // 双输入丢失
  }

  std::vector<FusionObject> fused;

  if (lidar_valid && vision_valid) {
    // 完整融合：匈牙利关联 + 加权融合（文档 6.2/6.3）
    const auto assignment = associate(lidar_cache_.objects, vision_cache_.objects);
    std::vector<bool> vision_matched(vision_cache_.objects.size(), false);
    for (size_t i = 0; i < lidar_cache_.objects.size(); ++i) {
      const int j = (i < assignment.size()) ? assignment[i] : -1;
      if (j >= 0 && j < static_cast<int>(vision_cache_.objects.size())) {
        fused.push_back({fusePair(lidar_cache_.objects[i], vision_cache_.objects[j]),
          Source::FUSED, true, true});
        vision_matched[j] = true;
      } else {
        fused.push_back({lidar_cache_.objects[i], Source::LASER, true, false});
      }
    }
    for (size_t j = 0; j < vision_cache_.objects.size(); ++j) {
      if (!vision_matched[j]) {
        fused.push_back({vision_cache_.objects[j], Source::VISION, false, true});
      }
    }
  } else if (lidar_valid) {
    // 视觉丢失 → 纯激光（laser_only）
    for (const auto & obj : lidar_cache_.objects) {
      fused.push_back({obj, Source::LASER, true, false});
    }
  } else {
    // 激光丢失 → 纯视觉（vision_only）
    for (const auto & obj : vision_cache_.objects) {
      fused.push_back({obj, Source::VISION, false, true});
    }
  }

  // 发布融合目标（10Hz）
  hunter_msgs::msg::DetectedObjectArray arr;
  arr.header.stamp = now;
  arr.header.frame_id = "base_link";
  arr.objects.reserve(fused.size());
  for (const auto & fo : fused) {
    arr.objects.push_back(fo.obj);
  }
  fused_pub_->publish(arr);

  // 构建可行驶区域（需激光，文档 6.5）
  if (lidar_valid) {
    nav_msgs::msg::OccupancyGrid grid;
    buildFreespace(fused, grid);
    grid.header.stamp = now;
    freespace_pub_->publish(grid);
  }
}

hunter_msgs::msg::DetectedObject SensorFusion::fusePair(
  const hunter_msgs::msg::DetectedObject & lidar,
  const hunter_msgs::msg::DetectedObject & vision)
{
  hunter_msgs::msg::DetectedObject out = lidar;  // 基础用激光（含速度）

  // 位置加权融合（文档 6.3：激光 0.7，视觉 0.3）
  out.pose.position.x =
    pos_lidar_weight_ * lidar.pose.position.x + pos_vision_weight_ * vision.pose.position.x;
  out.pose.position.y =
    pos_lidar_weight_ * lidar.pose.position.y + pos_vision_weight_ * vision.pose.position.y;
  out.pose.position.z =
    pos_lidar_weight_ * lidar.pose.position.z + pos_vision_weight_ * vision.pose.position.z;

  // 尺寸加权融合（文档 6.3：激光 0.8，视觉 0.2）
  out.dimensions.x =
    size_lidar_weight_ * lidar.dimensions.x + size_vision_weight_ * vision.dimensions.x;
  out.dimensions.y =
    size_lidar_weight_ * lidar.dimensions.y + size_vision_weight_ * vision.dimensions.y;
  out.dimensions.z =
    size_lidar_weight_ * lidar.dimensions.z + size_vision_weight_ * vision.dimensions.z;

  // 类别：取置信度更高的类别（文档 6.3）
  if (vision.confidence > lidar.confidence) {
    out.class_name = vision.class_name;
  }
  // 置信度：取更高者
  out.confidence = std::max(lidar.confidence, vision.confidence);
  // 速度：使用激光跟踪速度（文档 6.3：视觉无直接速度，out 已保留 lidar.velocity）

  return out;
}

void SensorFusion::buildFreespace(
  const std::vector<FusionObject> & objects, nav_msgs::msg::OccupancyGrid & grid)
{
  const int width = static_cast<int>(freespace_length_ / freespace_resolution_);
  const int height = static_cast<int>(freespace_width_ / freespace_resolution_);

  grid.header.frame_id = "base_link";
  grid.info.resolution = freespace_resolution_;
  grid.info.width = width;
  grid.info.height = height;
  grid.info.origin.position.x = 0.0;
  grid.info.origin.position.y = -freespace_width_ / 2.0;
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.w = 1.0;

  // 初始化：自由空间 0（文档 6.5）
  grid.data.assign(static_cast<size_t>(width) * height, 0);

  // 障碍物标记为占用 100，并向外膨胀（文档 6.5：0.3m 安全余量）
  const double y_off = freespace_width_ / 2.0;
  for (const auto & fo : objects) {
    const double cx = fo.obj.pose.position.x;
    const double cy = fo.obj.pose.position.y;
    const double half_l = fo.obj.dimensions.x / 2.0 + inflation_radius_;
    const double half_w = fo.obj.dimensions.y / 2.0 + inflation_radius_;

    const int gx_min = std::max(0, static_cast<int>((cx - half_l) / freespace_resolution_));
    const int gx_max =
      std::min(width - 1, static_cast<int>((cx + half_l) / freespace_resolution_));
    const int gy_min = std::max(
      0, static_cast<int>((cy - half_w + y_off) / freespace_resolution_));
    const int gy_max = std::min(
      height - 1, static_cast<int>((cy + half_w + y_off) / freespace_resolution_));

    for (int gx = gx_min; gx <= gx_max; ++gx) {
      for (int gy = gy_min; gy <= gy_max; ++gy) {
        grid.data[static_cast<size_t>(gy) * width + gx] = 100;
      }
    }
  }
}

}  // namespace sensor_fusion


