// Copyright 2026 HUNTER Development Team
// 激光感知节点实现（文档 5.1 完整处理链路）
#include "lidar_perception/lidar_perception.hpp"

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

#include <algorithm>
#include <cmath>
#include <utility>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace lidar_perception
{

LidarPerception::LidarPerception(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("lidar_perception", options),
  ground_filter_(),
  tracker_(),
  last_cloud_time_(std::chrono::steady_clock::now()),
  cloud_received_(false)
{
  // 参数（文档 5.1.2 阈值）
  declare_parameter("cloud_topic", "/lidar_points");
  declare_parameter("imu_topic", "/imu/data");
  declare_parameter("target_frame", "base_link");
  declare_parameter("roi_max_range", 50.0);
  declare_parameter("roi_min_range", 0.5);
  declare_parameter("roi_horizontal_fov", 120.0);
  declare_parameter("ground_height_threshold", 0.15);
  declare_parameter("ground_ray_resolution", 0.1);
  declare_parameter("cluster_tolerance", 0.5);
  declare_parameter("cluster_min_points", 10);
  declare_parameter("cluster_max_points", 5000);
  declare_parameter("association_distance", 1.0);
  declare_parameter("timeout", 0.3);
  declare_parameter("outlier_mean_k", 10);
  declare_parameter("outlier_std_dev", 1.0);
  // 修正：将硬编码参数提取为可配置参数
  declare_parameter("scan_period", 0.1);        // 扫描周期，默认 0.1s (10Hz)
  declare_parameter("marker_lifetime", 0.2);    // 可视化标记生命周期，默认 0.2s
  declare_parameter("marker_alpha", 0.7);       // 可视化标记透明度，默认 0.7
}

LidarPerception::CallbackReturn
LidarPerception::on_configure(const rclcpp_lifecycle::State &)
{
  cloud_topic_ = get_parameter("cloud_topic").as_string();
  imu_topic_ = get_parameter("imu_topic").as_string();
  target_frame_ = get_parameter("target_frame").as_string();
  roi_max_range_ = get_parameter("roi_max_range").as_double();
  roi_min_range_ = get_parameter("roi_min_range").as_double();
  roi_hfov_deg_ = get_parameter("roi_horizontal_fov").as_double();
  ground_height_threshold_ = get_parameter("ground_height_threshold").as_double();
  ground_ray_resolution_deg_ = get_parameter("ground_ray_resolution").as_double();
  cluster_tolerance_ = get_parameter("cluster_tolerance").as_double();
  cluster_min_points_ = static_cast<int>(get_parameter("cluster_min_points").as_int());
  cluster_max_points_ = static_cast<int>(get_parameter("cluster_max_points").as_int());
  association_distance_ = get_parameter("association_distance").as_double();
  timeout_ = get_parameter("timeout").as_double();
  outlier_mean_k_ = static_cast<int>(get_parameter("outlier_mean_k").as_int());
  outlier_std_dev_ = get_parameter("outlier_std_dev").as_double();
  // 修正：读取新增的参数
  scan_period_ = get_parameter("scan_period").as_double();
  marker_lifetime_ = get_parameter("marker_lifetime").as_double();
  marker_alpha_ = get_parameter("marker_alpha").as_double();

  ground_filter_ = RayGroundFilter(ground_height_threshold_, ground_ray_resolution_deg_);
  tracker_ = MultiObjectTracker(association_distance_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  // 发布器（文档 5.1.3 输出话题）
  objects_pub_ =
    create_publisher<hunter_msgs::msg::DetectedObjectArray>("/perception/lidar_objects", 10);
  ground_pub_ =
    create_publisher<sensor_msgs::msg::PointCloud2>("/perception/lidar/ground_cloud", 10);
  obstacle_pub_ =
    create_publisher<sensor_msgs::msg::PointCloud2>("/perception/lidar/obstacle_cloud", 10);
  marker_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>(
    "/perception/lidar/cluster_markers", 10);

  // 订阅（activate 后才真正接收）
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic_, rclcpp::SensorDataQoS(),
    std::bind(&LidarPerception::cloudCallback, this, std::placeholders::_1));
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_, rclcpp::SensorDataQoS(),
    std::bind(&LidarPerception::imuCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "lidar_perception 已配置：目标帧=%s, ROI=%.1fm/%.0f°",
    target_frame_.c_str(), roi_max_range_, roi_hfov_deg_);
  return CallbackReturn::SUCCESS;
}

LidarPerception::CallbackReturn
LidarPerception::on_activate(const rclcpp_lifecycle::State &)
{
  objects_pub_->on_activate();
  ground_pub_->on_activate();
  obstacle_pub_->on_activate();
  marker_pub_->on_activate();

  timeout_timer_ = create_wall_timer(
    std::chrono::milliseconds(1000),
    std::bind(&LidarPerception::checkTimeout, this));

  RCLCPP_INFO(get_logger(), "lidar_perception 已激活，发布 /perception/lidar_objects @ 10Hz");
  return CallbackReturn::SUCCESS;
}

LidarPerception::CallbackReturn
LidarPerception::on_deactivate(const rclcpp_lifecycle::State &)
{
  timeout_timer_.reset();
  objects_pub_->on_deactivate();
  ground_pub_->on_deactivate();
  obstacle_pub_->on_deactivate();
  marker_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

LidarPerception::CallbackReturn
LidarPerception::on_cleanup(const rclcpp_lifecycle::State &)
{
  cloud_sub_.reset();
  imu_sub_.reset();
  objects_pub_.reset();
  ground_pub_.reset();
  obstacle_pub_.reset();
  marker_pub_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  imu_queue_.clear();
  return CallbackReturn::SUCCESS;
}

LidarPerception::CallbackReturn
LidarPerception::on_shutdown(const rclcpp_lifecycle::State &)
{
  timeout_timer_.reset();
  return CallbackReturn::SUCCESS;
}

void LidarPerception::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  ImuSample s;
  s.stamp = msg->header.stamp;
  s.ang_vel_z = msg->angular_velocity.z;
  imu_queue_.push_back(s);
  // 保留最近 1 秒
  while (!imu_queue_.empty() &&
    (rclcpp::Time(msg->header.stamp) - rclcpp::Time(imu_queue_.front().stamp)).seconds() > 1.0)
  {
    imu_queue_.pop_front();
  }
}

void LidarPerception::checkTimeout()
{
  if (!cloud_received_) {
    return;
  }
  const double elapsed =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - last_cloud_time_).count();
  if (elapsed > timeout_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "点云数据超时：%.2fs 未收到 /lidar_points（阈值 %.2fs），感知降级", elapsed, timeout_);
  }
}

void LidarPerception::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  last_cloud_time_ = std::chrono::steady_clock::now();
  cloud_received_ = true;

  // 1. PointCloud2 → PCL
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(*msg, *cloud);

  // 2. 运动去畸变（IMU 补偿，文档 5.1.1）
  deskew(cloud, msg->header.stamp, cloud);

  // 3. 坐标系转换 lidar → base_link
  transformToBaseLink(cloud, cloud);

  // 4. ROI 裁剪（车前 120°、50m，文档 5.1.1）
  roiFilter(cloud, cloud);

  // 5. 离群点去除（PCL StatisticalOutlierRemoval）
  outlierRemoval(cloud, cloud);

  // 6. 射线坡度法地面分割
  auto ground = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  auto non_ground = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  ground_filter_.filter(cloud, ground, non_ground);

  // 7. 欧式聚类 + OBB 提取
  std::vector<Detection> detections;
  visualization_msgs::msg::MarkerArray markers;
  extractClusters(non_ground, detections, markers);

  // 8. 多目标跟踪（卡尔曼 + 匈牙利，10Hz 帧周期）
  const auto tracks = tracker_.update(detections, 0.1);

  // 9. 发布结果
  publishObjects(tracks, msg->header.stamp);

  // 发布调试点云/marker
  sensor_msgs::msg::PointCloud2 ground_msg, obstacle_msg;
  pcl::toROSMsg(*ground, ground_msg);
  pcl::toROSMsg(*non_ground, obstacle_msg);
  ground_msg.header = msg->header;
  ground_msg.header.frame_id = target_frame_;
  obstacle_msg.header = ground_msg.header;
  ground_pub_->publish(ground_msg);
  obstacle_pub_->publish(obstacle_msg);
  marker_pub_->publish(markers);
}

void LidarPerception::transformToBaseLink(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & in,
  pcl::PointCloud<pcl::PointXYZ>::Ptr & out)
{
  if (!tf_buffer_ || in->empty()) {
    *out = *in;
    return;
  }
  try {
    const auto tf = tf_buffer_->lookupTransform(
      target_frame_, in->header.frame_id, tf2::TimePointZero, tf2::durationFromSec(0.1));
    pcl_ros::transformPointCloud(*in, *out, tf);
  } catch (const tf2::TransformException & e) {
    // TF 不可用，降级：保留原坐标系
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "TF 变换失败（%s），保留原始坐标系", e.what());
    *out = *in;
  }
}

void LidarPerception::deskew(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & in,
  const rclcpp::Time &,
  pcl::PointCloud<pcl::PointXYZ>::Ptr & out)
{
  if (in->empty() || imu_queue_.empty()) {
    *out = *in;
    return;
  }
  // 简化运动去畸变：基于 IMU 横摆角速度（z）的旋转补偿
  const double wz = imu_queue_.back().ang_vel_z;
  // 修正：使用参数 scan_period_ 替代硬编码的 kScanPeriod
  const double scan_period = scan_period_;
  out->clear();
  out->header = in->header;
  out->reserve(in->size());

  for (const auto & p : in->points) {
    const double angle = std::atan2(p.y, p.x);  // [-pi, pi]
    // 机械雷达：角度 -pi..pi 对应扫描周期内 0..scan_period
    const double t = (angle + M_PI) / (2.0 * M_PI) * scan_period;
    const double rot = -wz * t;  // 反向补偿
    const double c = std::cos(rot);
    const double s = std::sin(rot);
    pcl::PointXYZ q;
    q.x = static_cast<float>(c * p.x - s * p.y);
    q.y = static_cast<float>(s * p.x + c * p.y);
    q.z = p.z;
    out->points.push_back(q);
  }
}

void LidarPerception::roiFilter(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & in,
  pcl::PointCloud<pcl::PointXYZ>::Ptr & out)
{
  const double half_fov = roi_hfov_deg_ * M_PI / 180.0 / 2.0;
  out->clear();
  out->header = in->header;
  for (const auto & p : in->points) {
    const double r = std::sqrt(p.x * p.x + p.y * p.y);
    if (r < roi_min_range_ || r > roi_max_range_) {
      continue;
    }
    const double angle = std::atan2(p.y, p.x);
    if (std::fabs(angle) > half_fov) {
      continue;  // 仅保留车前扇形
    }
    out->points.push_back(p);
  }
}

void LidarPerception::outlierRemoval(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & in,
  pcl::PointCloud<pcl::PointXYZ>::Ptr & out)
{
  if (in->size() < 3) {
    *out = *in;
    return;
  }
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(in);
  sor.setMeanK(outlier_mean_k_);
  sor.setStddevMulThresh(outlier_std_dev_);
  sor.filter(*out);
}

void LidarPerception::extractClusters(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  std::vector<Detection> & detections,
  visualization_msgs::msg::MarkerArray & markers)
{
  detections.clear();
  markers.markers.clear();
  if (cloud->empty()) {
    return;
  }

  // KD-Tree + 欧式聚类（文档 5.1.2：距离 0.5m，最小 10 点，最大 5000 点）
  auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
  tree->setInputCloud(cloud);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(static_cast<float>(cluster_tolerance_));
  ec.setMinClusterSize(cluster_min_points_);
  ec.setMaxClusterSize(cluster_max_points_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  int marker_id = 0;
  for (const auto & indices : cluster_indices) {
    auto cluster = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for (const auto idx : indices.indices) {
      cluster->points.push_back(cloud->points[idx]);
    }
    cluster->width = cluster->points.size();
    cluster->height = 1;
    cluster->is_dense = true;

    // OBB 包围盒提取（文档 5.1.1：位置 + 尺寸 + 朝向）
    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> moi;
    moi.setInputCloud(cluster);
    moi.compute();
    pcl::PointXYZ min_pt, max_pt, center;
    Eigen::Matrix3f rot;
    moi.getOBB(min_pt, max_pt, center, rot);

    Detection d;
    d.x = center.x;
    d.y = center.y;
    d.z = center.z;
    d.length = max_pt.x - min_pt.x;
    d.width = max_pt.y - min_pt.y;
    d.height = max_pt.z - min_pt.z;
    d.yaw = std::atan2(rot(1, 0), rot(0, 0));  // 主方向
    detections.push_back(d);

    // 聚类可视化 marker（立方体）
    visualization_msgs::msg::Marker m;
    m.header.frame_id = target_frame_;
    m.header.stamp = this->now();
    m.ns = "clusters";
    m.id = marker_id++;
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = center.x;
    m.pose.position.y = center.y;
    m.pose.position.z = center.z;
    m.pose.orientation.w = std::cos(d.yaw / 2.0);
    m.pose.orientation.z = std::sin(d.yaw / 2.0);
    m.scale.x = std::max(d.length, 0.1f);
    m.scale.y = std::max(d.width, 0.1f);
    m.scale.z = std::max(d.height, 0.1f);
    // 修正：使用参数 marker_alpha_ 替代硬编码的透明度
    m.color.a = static_cast<float>(marker_alpha_);
    m.color.r = 1.0f;
    m.color.g = 0.0f;
    m.color.b = 0.0f;
    // 修正：使用参数 marker_lifetime_ 替代硬编码的生命周期
    const int64_t lifetime_ns = static_cast<int64_t>(marker_lifetime_ * 1e9);
    m.lifetime = rclcpp::Duration(0, lifetime_ns);
    markers.markers.push_back(m);
  }
}

void LidarPerception::publishObjects(
  const std::vector<Track> & tracks, const rclcpp::Time & stamp)
{
  hunter_msgs::msg::DetectedObjectArray arr;
  arr.header.stamp = stamp;
  arr.header.frame_id = target_frame_;

  for (const auto & t : tracks) {
    hunter_msgs::msg::DetectedObject obj;
    obj.header = arr.header;
    obj.id = t.id;
    obj.track_id = t.id;
    obj.class_name = "obstacle";  // 粗分类（文档 5.1.1：行人/车辆/未知）
    obj.confidence = 0.8f;
    obj.pose.position.x = t.x;
    obj.pose.position.y = t.y;
    obj.pose.position.z = t.last_det.z;
    const double yaw = t.last_det.yaw;
    obj.pose.orientation.w = std::cos(yaw / 2.0);
    obj.pose.orientation.z = std::sin(yaw / 2.0);
    obj.dimensions.x = t.last_det.length;
    obj.dimensions.y = t.last_det.width;
    obj.dimensions.z = t.last_det.height;
    obj.velocity.linear.x = t.vx;
    obj.velocity.linear.y = t.vy;
    obj.tracking_age = static_cast<float>(t.age) * static_cast<float>(scan_period_);  // 帧数 × 扫描周期
    arr.objects.push_back(obj);
  }

  objects_pub_->publish(arr);
}

}  // namespace lidar_perception



