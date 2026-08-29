// Copyright 2026 HUNTER Development Team
// 视觉感知节点实现（文档 5.2：OpenCV-CUDA 预处理 + TensorRT 推理 + 深度投影）
#include "vision_perception/vision_perception.hpp"

#include <cv_bridge/cv_bridge.h>

#include <algorithm>
#include <mutex>
#include <utility>

namespace vision_perception
{

namespace
{
// COCO 类名 → 文档 4.3.1 粗分类
std::string classifyName(int class_id)
{
  switch (class_id) {
    case 0: return "pedestrian";            // person
    case 2: case 5: case 7: return "car";   // car, bus, truck
    case 1: case 3: return "cyclist";       // bicycle, motorcycle
    default: return "obstacle";
  }
}
}  // namespace

VisionPerception::VisionPerception(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("vision_perception", options),
  depth_received_(false),
  last_color_time_(std::chrono::steady_clock::now()),
  last_publish_time_(0, 0, RCL_ROS_TIME)
{
  // 参数（模型路径、推理参数全部 yaml 化，文档 5.2.2）
  declare_parameter("color_topic", "/camera/color/image_raw");
  declare_parameter("depth_topic", "/camera/depth/image_rect_raw");
  declare_parameter("target_frame", "camera_color_optical_frame");
  declare_parameter("engine_path", "/data/models/yolov8n.engine");
  declare_parameter("input_width", 640);
  declare_parameter("input_height", 640);
  declare_parameter("conf_threshold", 0.25);
  declare_parameter("nms_threshold", 0.45);
  declare_parameter("fx", 615.0);   // D435 RGB 内参（默认，需标定）
  declare_parameter("fy", 615.0);
  declare_parameter("cx", 320.0);
  declare_parameter("cy", 240.0);
  declare_parameter("publish_rate", 15.0);
  declare_parameter("timeout", 0.5);
  declare_parameter("min_depth", 0.1);
  declare_parameter("max_depth", 10.0);
  declare_parameter("freespace_topic", "/perception/vision_freespace");
  declare_parameter("freespace_resolution", 0.2);
  declare_parameter("freespace_length", 40.0);
  declare_parameter("freespace_width", 30.0);
  declare_parameter("free_depth", 5.0);
}

VisionPerception::CallbackReturn
VisionPerception::on_configure(const rclcpp_lifecycle::State &)
{
  color_topic_ = get_parameter("color_topic").as_string();
  depth_topic_ = get_parameter("depth_topic").as_string();
  target_frame_ = get_parameter("target_frame").as_string();
  engine_path_ = get_parameter("engine_path").as_string();
  input_width_ = static_cast<int>(get_parameter("input_width").as_int());
  input_height_ = static_cast<int>(get_parameter("input_height").as_int());
  conf_threshold_ = static_cast<float>(get_parameter("conf_threshold").as_double());
  nms_threshold_ = static_cast<float>(get_parameter("nms_threshold").as_double());
  fx_ = get_parameter("fx").as_double();
  fy_ = get_parameter("fy").as_double();
  cx_ = get_parameter("cx").as_double();
  cy_ = get_parameter("cy").as_double();
  publish_rate_ = get_parameter("publish_rate").as_double();
  timeout_ = get_parameter("timeout").as_double();
  min_depth_ = get_parameter("min_depth").as_double();
  max_depth_ = get_parameter("max_depth").as_double();
  freespace_topic_ = get_parameter("freespace_topic").as_string();
  freespace_resolution_ = get_parameter("freespace_resolution").as_double();
  freespace_length_ = get_parameter("freespace_length").as_double();
  freespace_width_ = get_parameter("freespace_width").as_double();
  free_depth_ = get_parameter("free_depth").as_double();

  trt_.setThresholds(conf_threshold_, nms_threshold_);

  // 加载 TensorRT 引擎（文档 5.2.3：FP16）
  if (!trt_.loadEngine(engine_path_)) {
    RCLCPP_ERROR(get_logger(), "TensorRT 引擎加载失败: %s", engine_path_.c_str());
    return CallbackReturn::FAILURE;
  }

  objects_pub_ =
    create_publisher<hunter_msgs::msg::DetectedObjectArray>("/perception/vision_objects", 10);
  freespace_pub_ =
    create_publisher<nav_msgs::msg::OccupancyGrid>(freespace_topic_, 10);

  color_sub_ = create_subscription<sensor_msgs::msg::Image>(
    color_topic_, rclcpp::SensorDataQoS(),
    std::bind(&VisionPerception::colorCallback, this, std::placeholders::_1));
  depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
    depth_topic_, rclcpp::SensorDataQoS(),
    std::bind(&VisionPerception::depthCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "vision_perception 已配置：引擎=%s, 阈值=%.2f/%.2f",
    engine_path_.c_str(), conf_threshold_, nms_threshold_);
  return CallbackReturn::SUCCESS;
}

VisionPerception::CallbackReturn
VisionPerception::on_activate(const rclcpp_lifecycle::State &)
{
  objects_pub_->on_activate();
  freespace_pub_->on_activate();
  timeout_timer_ = create_wall_timer(
    std::chrono::milliseconds(1000),
    std::bind(&VisionPerception::checkTimeout, this));
  RCLCPP_INFO(get_logger(), "vision_perception 已激活，发布 /perception/vision_objects @ 15Hz");
  return CallbackReturn::SUCCESS;
}

VisionPerception::CallbackReturn
VisionPerception::on_deactivate(const rclcpp_lifecycle::State &)
{
  timeout_timer_.reset();
  objects_pub_->on_deactivate();
  freespace_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

VisionPerception::CallbackReturn
VisionPerception::on_cleanup(const rclcpp_lifecycle::State &)
{
  color_sub_.reset();
  depth_sub_.reset();
  objects_pub_.reset();
  freespace_pub_.reset();
  return CallbackReturn::SUCCESS;
}

VisionPerception::CallbackReturn
VisionPerception::on_shutdown(const rclcpp_lifecycle::State &)
{
  timeout_timer_.reset();
  return CallbackReturn::SUCCESS;
}

void VisionPerception::depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(depth_mutex_);
  try {
    depth_img_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
    depth_received_ = true;
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "深度图转换失败: %s", e.what());
  }
}

void VisionPerception::checkTimeout()
{
  const double elapsed =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - last_color_time_).count();
  if (elapsed > timeout_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "彩色图像超时：%.2fs 未收到 %s（阈值 %.2fs）", elapsed, color_topic_.c_str(), timeout_);
  }
}

void VisionPerception::colorCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  last_color_time_ = std::chrono::steady_clock::now();

  // 降采样到 publish_rate（彩色 30Hz → 15Hz）
  const rclcpp::Time now = msg->header.stamp;
  if ((now - last_publish_time_).seconds() < (1.0 / publish_rate_)) {
    return;
  }

  // 1. Image → cv::Mat
  cv::Mat color;
  try {
    color = cv_bridge::toCvCopy(msg, "bgr8")->image;
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "彩色图像转换失败: %s", e.what());
    return;
  }

  // 2. TensorRT YOLOv8 推理（含预处理）
  std::vector<BBox2D> boxes;
  if (!trt_.infer(color, boxes)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "TensorRT 推理失败");
    return;
  }

  // 3. 取深度图（线程安全）
  cv::Mat depth;
  {
    std::lock_guard<std::mutex> lock(depth_mutex_);
    if (!depth_received_ || depth_img_.empty()) {
      // 深度失效：降级告警，跳过本帧
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "深度图未就绪/失效，跳过本帧");
      return;
    }
    depth = depth_img_;
  }

  // 4. 2D→3D 投影（相机内参 + 深度图）
  std::vector<Detection3D> dets;
  dets.reserve(boxes.size());
  for (const auto & box : boxes) {
    Detection3D det;
    if (projectTo3D(box, depth, det)) {
      dets.push_back(det);
    }
  }

  // 5. 发布结果
  publishObjects(dets, msg->header.stamp);

  // 6. 发布视觉可行驶区域（文档 5.2.1 输出 freespace）
  nav_msgs::msg::OccupancyGrid grid;
  buildFreespace(dets, grid);
  grid.header.stamp = msg->header.stamp;
  freespace_pub_->publish(grid);

  last_publish_time_ = now;
}

bool VisionPerception::projectTo3D(
  const BBox2D & box, const cv::Mat & depth, Detection3D & det)
{
  // 框中心像素坐标
  const float u = (box.x1 + box.x2) / 2.0f;
  const float v = (box.y1 + box.y2) / 2.0f;

  // 边界检查
  if (u < 0 || u >= depth.cols || v < 0 || v >= depth.rows) {
    return false;
  }

  // 取深度（mm → m），16UC1
  const float d = static_cast<float>(depth.at<uint16_t>(
    static_cast<int>(v), static_cast<int>(u))) / 1000.0f;

  // 深度失效（超出有效范围）
  if (d < min_depth_ || d > max_depth_) {
    return false;
  }

  // 2D→3D 投影（针孔模型，光学坐标系：X右/Y下/Z前）
  det.x = static_cast<float>((u - cx_) * d / fx_);
  det.y = static_cast<float>((v - cy_) * d / fy_);
  det.z = d;

  // 尺寸近似（基于框像素尺寸 + 深度）
  const float w_pix = box.x2 - box.x1;
  const float h_pix = box.y2 - box.y1;
  det.width = static_cast<float>(w_pix * d / fx_);
  det.height = static_cast<float>(h_pix * d / fy_);
  det.length = det.width;  // 近似

  det.confidence = box.confidence;
  det.class_id = box.class_id;
  return true;
}

void VisionPerception::publishObjects(
  const std::vector<Detection3D> & dets, const rclcpp::Time & stamp)
{
  hunter_msgs::msg::DetectedObjectArray arr;
  arr.header.stamp = stamp;
  arr.header.frame_id = target_frame_;  // camera_color_optical_frame

  for (const auto & det : dets) {
    hunter_msgs::msg::DetectedObject obj;
    obj.header = arr.header;
    obj.class_name = classifyName(det.class_id);
    obj.confidence = det.confidence;
    obj.pose.position.x = det.x;
    obj.pose.position.y = det.y;
    obj.pose.position.z = det.z;
    obj.dimensions.x = det.length;
    obj.dimensions.y = det.width;
    obj.dimensions.z = det.height;
    arr.objects.push_back(obj);
  }

  objects_pub_->publish(arr);
}

void VisionPerception::buildFreespace(
  const std::vector<Detection3D> & dets, nav_msgs::msg::OccupancyGrid & grid)
{
  // 视觉可行驶区域（光学坐标系：Z前/X右），车前 z∈[0,length]，左右 x∈[-width/2, width/2]
  const int width = static_cast<int>(freespace_length_ / freespace_resolution_);
  const int height = static_cast<int>(freespace_width_ / freespace_resolution_);

  grid.header.frame_id = target_frame_;  // camera_color_optical_frame
  grid.info.resolution = freespace_resolution_;
  grid.info.width = width;
  grid.info.height = height;
  grid.info.origin.position.x = -freespace_width_ / 2.0;
  grid.info.origin.position.y = 0.0;
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.w = 1.0;

  // 初始化：自由空间 0（文档 6.5）
  grid.data.assign(static_cast<size_t>(width) * height, 0);

  // 检测目标投影为占用 100（膨胀由参数 free_depth 示意，此处以目标包围盒标记）
  for (const auto & det : dets) {
    if (det.z <= 0.0f || det.z > free_depth_) {
      continue;
    }
    const double half_l = det.length / 2.0;
    const double half_w = det.width / 2.0;
    const int gz_min = std::max(0, static_cast<int>((det.z - half_l) / freespace_resolution_));
    const int gz_max =
      std::min(width - 1, static_cast<int>((det.z + half_l) / freespace_resolution_));
    const int gx_min = std::max(0, static_cast<int>(
      (det.x - half_w + freespace_width_ / 2.0) / freespace_resolution_));
    const int gx_max = std::min(height - 1, static_cast<int>(
      (det.x + half_w + freespace_width_ / 2.0) / freespace_resolution_));
    for (int gx = gx_min; gx <= gx_max; ++gx) {
      for (int gz = gz_min; gz <= gz_max; ++gz) {
        grid.data[static_cast<size_t>(gx) * width + gz] = 100;
      }
    }
  }
}

}  // namespace vision_perception


