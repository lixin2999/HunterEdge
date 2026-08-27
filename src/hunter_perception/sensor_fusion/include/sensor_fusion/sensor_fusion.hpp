// Copyright 2026 HUNTER Development Team
// 多传感器目标级数据融合节点声明（文档第 6 章）
#ifndef SENSOR_FUSION__SENSOR_FUSION_HPP_
#define SENSOR_FUSION__SENSOR_FUSION_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "hunter_msgs/msg/detected_object.hpp"
#include "hunter_msgs/msg/detected_object_array.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace sensor_fusion
{

// 目标来源标记（文档 6.3：laser_only / vision_only / fused）
enum class Source
{
  LASER,
  VISION,
  FUSED
};

// 融合目标（内部结构）
struct FusionObject
{
  hunter_msgs::msg::DetectedObject obj;
  Source source;
  bool has_laser;
  bool has_vision;
};

class SensorFusion : public rclcpp::Node
{
public:
  explicit SensorFusion(const rclcpp::NodeOptions & options);

private:
  void lidarCallback(const hunter_msgs::msg::DetectedObjectArray::SharedPtr msg);
  void visionCallback(const hunter_msgs::msg::DetectedObjectArray::SharedPtr msg);
  void checkTimeout();

  // 核心融合（由激光帧驱动，10Hz）
  void fuseAndPublish();
  // 匈牙利关联匹配（文档 6.2）
  std::vector<int> associate(
    const std::vector<hunter_msgs::msg::DetectedObject> & lidar,
    const std::vector<hunter_msgs::msg::DetectedObject> & vision);
  // 加权状态融合（文档 6.3）
  hunter_msgs::msg::DetectedObject fusePair(
    const hunter_msgs::msg::DetectedObject & lidar,
    const hunter_msgs::msg::DetectedObject & vision);
  // 构建可行驶区域 OccupancyGrid（文档 6.5）
  void buildFreespace(
    const std::vector<FusionObject> & objects, nav_msgs::msg::OccupancyGrid & grid);

  // 参数
  std::string lidar_topic_, vision_topic_, fused_topic_, freespace_topic_;
  double match_distance_;           // 匹配阈值 2.0m
  double pos_lidar_weight_;         // 位置激光权重 0.7
  double pos_vision_weight_;        // 位置视觉权重 0.3
  double size_lidar_weight_;        // 尺寸激光权重 0.8
  double size_vision_weight_;       // 尺寸视觉权重 0.2
  double freespace_resolution_;     // 0.2m
  double freespace_length_;         // 车前 40m
  double freespace_width_;          // 左右各 15m（总 30m）
  double inflation_radius_;         // 膨胀 0.3m
  double lidar_timeout_;            // 激光超时
  double vision_timeout_;           // 视觉超时

  // 订阅/发布
  rclcpp::Subscription<hunter_msgs::msg::DetectedObjectArray>::SharedPtr lidar_sub_;
  rclcpp::Subscription<hunter_msgs::msg::DetectedObjectArray>::SharedPtr vision_sub_;
  rclcpp::Publisher<hunter_msgs::msg::DetectedObjectArray>::SharedPtr fused_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr freespace_pub_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;

  // 缓存
  hunter_msgs::msg::DetectedObjectArray lidar_cache_;
  hunter_msgs::msg::DetectedObjectArray vision_cache_;
  rclcpp::Time lidar_stamp_;
  rclcpp::Time vision_stamp_;
  bool lidar_received_;
  bool vision_received_;
  std::chrono::steady_clock::time_point last_lidar_time_;
  std::chrono::steady_clock::time_point last_vision_time_;

  // 修正：TF2 坐标转换
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace sensor_fusion

#endif  // SENSOR_FUSION__SENSOR_FUSION_HPP_
