// Copyright 2026 HUNTER Development Team
// 视觉感知节点声明（文档 5.2）
#ifndef VISION_PERCEPTION__VISION_PERCEPTION_HPP_
#define VISION_PERCEPTION__VISION_PERCEPTION_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include "vision_perception/tensorrt_inference.hpp"

#include "hunter_msgs/msg/detected_object_array.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <opencv2/opencv.hpp>

namespace vision_perception
{

// 3D 目标（2D 框 + 深度投影结果）
struct Detection3D
{
  float x, y, z;              // 相机光学坐标系 3D 位置
  float length, width, height;
  float confidence;
  int class_id;
};

class VisionPerception : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit VisionPerception(const rclcpp::NodeOptions & options);

  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void colorCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void checkTimeout();
  // 帧处理主流程（由 colorCallback 调用；异常在回调处兜底，避免节点 abort）
  void processColorFrame(
    const sensor_msgs::msg::Image::SharedPtr msg, const rclcpp::Time & now);

  // 2D→3D 投影（文档 5.2.1：相机内参 + 深度图）
  bool projectTo3D(const BBox2D & box, const cv::Mat & depth, Detection3D & det);
  void publishObjects(
    const std::vector<Detection3D> & dets, const rclcpp::Time & stamp);
  // 构建可行驶区域（文档 5.2.1 输出 /perception/freespace）
  void buildFreespace(
    const std::vector<Detection3D> & dets, nav_msgs::msg::OccupancyGrid & grid);

  // 参数
  std::string color_topic_, depth_topic_, target_frame_, freespace_topic_;
  std::string engine_path_;
  int input_width_, input_height_;
  float conf_threshold_, nms_threshold_;
  double fx_, fy_, cx_, cy_;          // 相机内参
  double publish_rate_;               // 输出频率（默认 15Hz）
  double timeout_;                    // 图像超时阈值
  double min_depth_, max_depth_;      // 深度有效范围
  double freespace_resolution_;       // 可行驶区域分辨率（文档 6.5：0.2m）
  double freespace_length_;           // 车前长度（40m）
  double freespace_width_;            // 总宽度（左右各 15m）
  double free_depth_;                 // 可行驶深度阈值（m）

  // 订阅/发布
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp_lifecycle::LifecyclePublisher<hunter_msgs::msg::DetectedObjectArray>::SharedPtr
    objects_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
    freespace_pub_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;

  // 推理
  TensorRTInference trt_;

  // 状态
  std::mutex depth_mutex_;
  cv::Mat depth_img_;
  bool depth_received_;
  std::chrono::steady_clock::time_point last_color_time_;
  rclcpp::Time last_publish_time_;
};

}  // namespace vision_perception

#endif  // VISION_PERCEPTION__VISION_PERCEPTION_HPP_
