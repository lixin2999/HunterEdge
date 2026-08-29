// Copyright 2026 HUNTER Development Team
// TensorRT YOLOv8 推理实现（引擎加载 + 前向推理 + 后处理）
#include "vision_perception/tensorrt_inference.hpp"

#include <cuda_runtime.h>

#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>

namespace vision_perception
{

Logger TensorRTInference::logger_;

TensorRTInference::~TensorRTInference()
{
  if (context_) {
    context_->destroy();
    context_ = nullptr;
  }
  if (engine_) {
    engine_->destroy();
    engine_ = nullptr;
  }
  if (runtime_) {
    runtime_->destroy();
    runtime_ = nullptr;
  }
}

bool TensorRTInference::loadEngine(const std::string & engine_path)
{
  std::ifstream file(engine_path, std::ios::binary);
  if (!file.good()) {
    std::cerr << "[TensorRT] 无法打开引擎文件: " << engine_path << std::endl;
    return false;
  }
  file.seekg(0, std::ios::end);
  const size_t size = static_cast<size_t>(file.tellg());
  file.seekg(0, std::ios::beg);
  std::vector<char> data(size);
  file.read(data.data(), static_cast<std::streamsize>(size));
  file.close();

  runtime_ = nvinfer1::createInferRuntime(logger_);
  if (!runtime_) {
    return false;
  }
  engine_ = runtime_->deserializeCudaEngine(data.data(), size, nullptr);
  if (!engine_) {
    return false;
  }
  context_ = engine_->createExecutionContext();
  if (!context_) {
    return false;
  }

  // 输入维度 (1, 3, H, W)
  const auto input_dims = engine_->getBindingDimensions(0);
  input_h_ = input_dims.d[2];
  input_w_ = input_dims.d[3];
  input_buffer_.resize(3 * input_h_ * input_w_);

  // 输出维度 (1, 4+num_classes, num_boxes)
  const auto output_dims = engine_->getBindingDimensions(1);
  num_classes_ = output_dims.d[1] - 4;
  num_boxes_ = output_dims.d[2];
  output_buffer_.resize((4 + num_classes_) * num_boxes_);

  return true;
}

bool TensorRTInference::infer(const cv::Mat & image, std::vector<BBox2D> & boxes)
{
  boxes.clear();
  if (!context_) {
    return false;
  }

  // 预处理（OpenCV CUDA 硬件加速：resize + BGR→RGB，文档 5.2.3）
  cv::Mat rgb;
  {
    cv::cuda::GpuMat gpu_in(image), gpu_resized, gpu_rgb;
    cv::cuda::resize(gpu_in, gpu_resized, cv::Size(input_w_, input_h_));
    cv::cuda::cvtColor(gpu_resized, gpu_rgb, cv::COLOR_BGR2RGB);
    gpu_rgb.download(rgb);
  }
  rgb.convertTo(rgb, CV_32FC3, 1.0f / 255.0f);

  const int h = input_h_;
  const int w = input_w_;
  for (int c = 0; c < 3; ++c) {
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        input_buffer_[c * h * w + y * w + x] = rgb.at<cv::Vec3f>(y, x)[c];
      }
    }
  }

  // 拷贝输入到 GPU + 推理
  void * buffers[2] = {nullptr, nullptr};
  cudaMalloc(&buffers[0], input_buffer_.size() * sizeof(float));
  cudaMalloc(&buffers[1], output_buffer_.size() * sizeof(float));
  cudaMemcpy(
    buffers[0], input_buffer_.data(), input_buffer_.size() * sizeof(float),
    cudaMemcpyHostToDevice);

  cudaStream_t stream = nullptr;
  if (!context_->enqueueV2(buffers, stream, nullptr)) {
    cudaFree(buffers[0]);
    cudaFree(buffers[1]);
    return false;
  }
  cudaMemcpy(
    output_buffer_.data(), buffers[1], output_buffer_.size() * sizeof(float),
    cudaMemcpyDeviceToHost);

  cudaFree(buffers[0]);
  cudaFree(buffers[1]);

  // 后处理（缩放回原图尺寸）
  const float scale_x = static_cast<float>(image.cols) / input_w_;
  const float scale_y = static_cast<float>(image.rows) / input_h_;
  postprocess(output_buffer_.data(), num_classes_, num_boxes_, scale_x, scale_y, boxes);
  return true;
}

void TensorRTInference::postprocess(
  const float * output, int num_classes, int num_boxes,
  float scale_x, float scale_y, std::vector<BBox2D> & boxes)
{
  // output: (4+num_classes, num_boxes)，通道主序
  std::vector<BBox2D> candidates;
  for (int j = 0; j < num_boxes; ++j) {
    // 找最大类别分数
    float max_score = 0.0f;
    int max_cls = -1;
    for (int c = 0; c < num_classes; ++c) {
      const float s = output[(4 + c) * num_boxes + j];
      if (s > max_score) {
        max_score = s;
        max_cls = c;
      }
    }
    if (max_score < conf_threshold_) {
      continue;
    }

    // 归一化坐标（0-1，相对输入尺寸）
    const float cx = output[0 * num_boxes + j];
    const float cy = output[1 * num_boxes + j];
    const float w = output[2 * num_boxes + j];
    const float h = output[3 * num_boxes + j];

    BBox2D b;
    b.x1 = (cx - w / 2.0f) * input_w_ * scale_x;
    b.y1 = (cy - h / 2.0f) * input_h_ * scale_y;
    b.x2 = (cx + w / 2.0f) * input_w_ * scale_x;
    b.y2 = (cy + h / 2.0f) * input_h_ * scale_y;
    b.confidence = max_score;
    b.class_id = max_cls;
    candidates.push_back(b);
  }

  // NMS（按置信度降序 + IoU 抑制）
  std::sort(
    candidates.begin(), candidates.end(),
    [](const BBox2D & a, const BBox2D & b) { return a.confidence > b.confidence; });

  std::vector<bool> keep(candidates.size(), true);
  for (size_t i = 0; i < candidates.size(); ++i) {
    if (!keep[i]) {
      continue;
    }
    for (size_t j = i + 1; j < candidates.size(); ++j) {
      if (!keep[j]) {
        continue;
      }
      const float ix1 = std::max(candidates[i].x1, candidates[j].x1);
      const float iy1 = std::max(candidates[i].y1, candidates[j].y1);
      const float ix2 = std::min(candidates[i].x2, candidates[j].x2);
      const float iy2 = std::min(candidates[i].y2, candidates[j].y2);
      const float iw = std::max(0.0f, ix2 - ix1);
      const float ih = std::max(0.0f, iy2 - iy1);
      const float inter = iw * ih;
      const float area_i =
        (candidates[i].x2 - candidates[i].x1) * (candidates[i].y2 - candidates[i].y1);
      const float area_j =
        (candidates[j].x2 - candidates[j].x1) * (candidates[j].y2 - candidates[j].y1);
      const float iou = inter / (area_i + area_j - inter + 1e-6f);
      if (iou > nms_threshold_) {
        keep[j] = false;
      }
    }
  }

  for (size_t i = 0; i < candidates.size(); ++i) {
    if (keep[i]) {
      boxes.push_back(candidates[i]);
    }
  }
}

}  // namespace vision_perception
