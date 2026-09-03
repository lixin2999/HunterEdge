// Copyright 2026 HUNTER Development Team
// TensorRT YOLOv8 推理封装（文档 5.2.3：TensorRT FP16）
#ifndef VISION_PERCEPTION__TENSORRT_INFERENCE_HPP_
#define VISION_PERCEPTION__TENSORRT_INFERENCE_HPP_

#include <NvInfer.h>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

namespace vision_perception
{

// TensorRT 日志器
class Logger : public nvinfer1::ILogger
{
public:
  void log(Severity severity, const char * msg) noexcept override
  {
    if (severity <= Severity::kWARNING) {
      std::cerr << "[TensorRT] " << msg << std::endl;
    }
  }
};

// TensorRT >= 8.5 废弃了 destroy()，改用 delete（通过自定义 deleter 的智能指针管理）
struct TRTDeleter
{
  template <typename T>
  void operator()(T * obj) const { delete obj; }
};

template <typename T>
using TRTUniquePtr = std::unique_ptr<T, TRTDeleter>;

// YOLOv8 2D 检测框（像素坐标）
struct BBox2D
{
  float x1, y1, x2, y2;
  float confidence;
  int class_id;
};

class TensorRTInference
{
public:
  TensorRTInference()
  : input_w_(640), input_h_(640), conf_threshold_(0.25f), nms_threshold_(0.45f) {}
  ~TensorRTInference() = default;

  // 加载 TensorRT 引擎（.engine）
  bool loadEngine(const std::string & engine_path);

  void setThresholds(float conf, float nms)
  {
    conf_threshold_ = conf;
    nms_threshold_ = nms;
  }

  // 推理：输入 BGR 图像，输出 2D 检测框
  bool infer(const cv::Mat & image, std::vector<BBox2D> & boxes);

private:
  // YOLOv8 后处理：输出 (1, 4+num_classes, num_boxes) → 解码 + NMS
  void postprocess(
    const float * output, int num_classes, int num_boxes,
    float scale_x, float scale_y, std::vector<BBox2D> & boxes);

  static Logger logger_;

  // TensorRT >= 8.5：用智能指针替代手动 destroy()
  TRTUniquePtr<nvinfer1::IRuntime> runtime_;
  TRTUniquePtr<nvinfer1::ICudaEngine> engine_;
  TRTUniquePtr<nvinfer1::IExecutionContext> context_;

  // 引擎 I/O 张量名称（TRT >= 8.5 用名称查询维度）
  std::string input_name_;
  std::string output_name_;

  int input_w_, input_h_;
  int num_classes_, num_boxes_;
  std::vector<float> input_buffer_;
  std::vector<float> output_buffer_;

  float conf_threshold_;
  float nms_threshold_;
};

}  // namespace vision_perception

#endif  // VISION_PERCEPTION__TENSORRT_INFERENCE_HPP_
