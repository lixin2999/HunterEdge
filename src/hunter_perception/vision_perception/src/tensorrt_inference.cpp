// Copyright 2026 HUNTER Development Team
// TensorRT YOLOv8 推理实现（引擎加载 + 前向推理 + 后处理）
#include "vision_perception/tensorrt_inference.hpp"

#include <cuda_runtime.h>

// OpenCV CUDA 模块仅在编译时检测到 opencv_cudaimgproc / opencv_cudawarping 时启用
// 无 CUDA OpenCV 的环境（如仅安装 CPU-only OpenCV）会自动降级到 CPU 预处理路径
#ifdef HAVE_OPENCV_CUDA
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#endif

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>

namespace vision_perception
{

Logger TensorRTInference::logger_;

// TensorRT >= 8.5：IRuntime/ICudaEngine/IExecutionContext 已改为通过 delete 释放
// 析构由智能指针（TRTUniquePtr）自动完成，此处无需手动 destroy()

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

  runtime_.reset(nvinfer1::createInferRuntime(logger_));
  if (!runtime_) {
    return false;
  }
  // TRT >= 8.5：deserializeCudaEngine 只接受 (blob, size)，第三个参数已移除
  engine_.reset(runtime_->deserializeCudaEngine(data.data(), size));
  if (!engine_) {
    return false;
  }
  context_.reset(engine_->createExecutionContext());
  if (!context_) {
    return false;
  }

  // TRT >= 8.5：用张量名称查询维度，废弃了基于绑定索引的 getBindingDimensions()
  // 约定：引擎第 0 个 I/O 为输入，第 1 个为输出（YOLOv8 导出的默认命名）
  const int num_io = engine_->getNbIOTensors();
  if (num_io < 2) {
    std::cerr << "[TensorRT] 引擎 I/O 张量数量不足（期望 >= 2，实际 " << num_io << "）" << std::endl;
    return false;
  }
  input_name_ = engine_->getIOTensorName(0);
  output_name_ = engine_->getIOTensorName(1);

  // 输入维度 (1, 3, H, W)
  const auto input_dims = engine_->getTensorShape(input_name_.c_str());
  input_h_ = input_dims.d[2];
  input_w_ = input_dims.d[3];
  if (input_h_ <= 0 || input_w_ <= 0) {
    std::cerr << "[TensorRT] 引擎输入维度无效：H=" << input_h_
              << " W=" << input_w_ << "，引擎文件可能损坏或未正确转换。"
              << " 请重新执行：trtexec --onnx=yolov8s.onnx"
              << " --saveEngine=/data/models/yolov8s.engine --fp16" << std::endl;
    context_.reset();
    engine_.reset();
    runtime_.reset();
    return false;
  }
  input_buffer_.resize(3 * input_h_ * input_w_);

  // 输出维度 (1, 4+num_classes, num_boxes)
  const auto output_dims = engine_->getTensorShape(output_name_.c_str());
  num_classes_ = output_dims.d[1] - 4;
  num_boxes_ = output_dims.d[2];
  if (num_classes_ <= 0 || num_boxes_ <= 0) {
    std::cerr << "[TensorRT] 引擎输出维度无效：num_classes=" << num_classes_
              << " num_boxes=" << num_boxes_ << "，引擎文件可能损坏。" << std::endl;
    context_.reset();
    engine_.reset();
    runtime_.reset();
    return false;
  }
  output_buffer_.resize((4 + num_classes_) * num_boxes_);

  return true;
}

bool TensorRTInference::infer(const cv::Mat & image, std::vector<BBox2D> & boxes)
{
  boxes.clear();
  if (!context_) {
    return false;
  }
  if (input_w_ <= 0 || input_h_ <= 0) {
    return false;
  }

  // 预处理（resize + BGR→RGB）
  // 有 CUDA OpenCV 时走 GPU 路径（延迟更低），否则降级为 CPU 路径
  cv::Mat rgb;
#ifdef HAVE_OPENCV_CUDA
  {
    cv::cuda::GpuMat gpu_in(image), gpu_resized, gpu_rgb;
    cv::cuda::resize(gpu_in, gpu_resized, cv::Size(input_w_, input_h_));
    cv::cuda::cvtColor(gpu_resized, gpu_rgb, cv::COLOR_BGR2RGB);
    gpu_rgb.download(rgb);
  }
#else
  {
    cv::Mat resized;
    cv::resize(image, resized, cv::Size(input_w_, input_h_));
    cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
  }
#endif
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

  // TRT >= 8.5：用张量名称绑定地址，通过 executeV2() 同步执行
  // （enqueueV2 已废弃，enqueueV3 不再接受 bindings 数组，改用 setTensorAddress）
  context_->setTensorAddress(input_name_.c_str(), buffers[0]);
  context_->setTensorAddress(output_name_.c_str(), buffers[1]);
  if (!context_->executeV2(buffers)) {
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
