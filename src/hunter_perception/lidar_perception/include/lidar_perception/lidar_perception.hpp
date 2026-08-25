// Copyright 2026 HUNTER Development Team
// 激光感知节点声明（文档 5.1）
#ifndef LIDAR_PERCEPTION__LIDAR_PERCEPTION_HPP_
#define LIDAR_PERCEPTION__LIDAR_PERCEPTION_HPP_

#include <chrono>
#include <deque>
#include <memory>
#include <string>

#include "lidar_perception/ground_filter.hpp"
#include "lidar_perception/tracker.hpp"

#include "hunter_msgs/msg/detected_object_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_perception
{

// IMU 采样（用于运动去畸变）
struct ImuSample
{
  rclcpp::Time stamp;
  double ang_vel_z;
};

class LidarPerception : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit LidarPerception(const rclcpp::NodeOptions & options);

  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void checkTimeout();

  // 处理链路（文档 5.1.1）
  void transformToBaseLink(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & in,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & out);
  void deskew(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & in,
    const rclcpp::Time & stamp,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & out);
  void roiFilter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & in,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & out);
  void outlierRemoval(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & in,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & out);
  void extractClusters(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    std::vector<Detection> & detections,
    visualization_msgs::msg::MarkerArray & markers);
  void publishObjects(
    const std::vector<Track> & tracks, const rclcpp::Time & stamp);

  // 参数
  std::string cloud_topic_, imu_topic_, target_frame_;
  double roi_max_range_, roi_min_range_, roi_hfov_deg_;
  double ground_height_threshold_, ground_ray_resolution_deg_;
  double cluster_tolerance_;
  int cluster_min_points_, cluster_max_points_;
  double association_distance_;
  double timeout_;
  int outlier_mean_k_;
  double outlier_std_dev_;

  // 订阅/发布
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp_lifecycle::LifecyclePublisher<hunter_msgs::msg::DetectedObjectArray>::SharedPtr
    objects_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    marker_pub_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // 算法
  RayGroundFilter ground_filter_;
  MultiObjectTracker tracker_;

  // 状态
  std::deque<ImuSample> imu_queue_;
  std::chrono::steady_clock::time_point last_cloud_time_;
  bool cloud_received_;
};

}  // namespace lidar_perception

#endif  // LIDAR_PERCEPTION__LIDAR_PERCEPTION_HPP_
