// Copyright 2026 HUNTER Development Team
// 多目标跟踪：卡尔曼滤波 + 匈牙利匹配（文档 5.1.2：状态 [x,y,vx,vy]，关联距离 1.0m）
#ifndef LIDAR_PERCEPTION__TRACKER_HPP_
#define LIDAR_PERCEPTION__TRACKER_HPP_

#include <cstdint>
#include <vector>

namespace lidar_perception
{

// 检测目标（聚类 + OBB 提取结果）
struct Detection
{
  float x, y, z;
  float length, width, height;
  float yaw;
};

// 跟踪目标
struct Track
{
  int32_t id;
  float x, y, vx, vy;
  int age;      // 总跟踪帧数
  int missed;   // 连续丢失帧数
  bool confirmed;
  Detection last_det;
};

class MultiObjectTracker
{
public:
  explicit MultiObjectTracker(double association_distance = 1.0)
  : association_distance_(association_distance), next_id_(0) {}

  // 更新：输入当前帧检测（base_link 系），输出跟踪目标
  std::vector<Track> update(const std::vector<Detection> & detections, double dt);

private:
  void predict(double dt);
  void updateTrack(Track & t, const Detection & d, double dt);

  double association_distance_;
  int32_t next_id_;
  std::vector<Track> tracks_;
};

}  // namespace lidar_perception

#endif  // LIDAR_PERCEPTION__TRACKER_HPP_
