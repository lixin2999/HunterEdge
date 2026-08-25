// Copyright 2026 HUNTER Development Team
// 射线坡度法地面分割（文档 5.1.2：高度阈值 0.15m，射线分辨率 0.1°）
#ifndef LIDAR_PERCEPTION__GROUND_FILTER_HPP_
#define LIDAR_PERCEPTION__GROUND_FILTER_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <utility>
#include <vector>

namespace lidar_perception
{

class RayGroundFilter
{
public:
  explicit RayGroundFilter(double height_threshold = 0.15, double ray_resolution_deg = 0.1,
    double max_slope_deg = 8.0)
  : height_threshold_(height_threshold),
    ray_resolution_(ray_resolution_deg * M_PI / 180.0),
    max_slope_(max_slope_deg * M_PI / 180.0)
  {
  }

  // 分割：输入点云 → 地面点云 + 非地面点云
  void filter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & input,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & ground,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & non_ground)
  {
    ground->clear();
    non_ground->clear();
    if (!input || input->empty()) {
      return;
    }

    // 1. 按水平角度分桶（射线，分辨率 ray_resolution_）
    std::map<int, std::vector<std::pair<float, pcl::PointXYZ>>> bins;
    for (const auto & p : input->points) {
      const float r = std::sqrt(p.x * p.x + p.y * p.y);
      if (r < 0.5f) {
        continue;  // 忽略近处盲区
      }
      const float angle = std::atan2(p.y, p.x);  // [-pi, pi]
      const int bin = static_cast<int>(std::floor(angle / ray_resolution_));
      bins[bin].emplace_back(r, p);
    }

    // 2. 每条射线按距离排序，坡度判断
    for (auto & kv : bins) {
      auto & pts = kv.second;
      std::sort(
        pts.begin(), pts.end(),
        [](const auto & a, const auto & b) { return a.first < b.first; });

      float ref_z = pts.front().second.z;
      float ref_r = pts.front().first;
      ground->points.push_back(pts.front().second);

      for (size_t i = 1; i < pts.size(); ++i) {
        const float r = pts[i].first;
        const float z = pts[i].second.z;
        const float dr = r - ref_r;
        const float dz = z - ref_z;
        const float slope = (dr > 1e-4f) ? std::atan2(dz, dr) : 0.0f;

        if (std::fabs(slope) < max_slope_ && std::fabs(dz) < height_threshold_) {
          ground->points.push_back(pts[i].second);
          ref_z = z;  // 更新地面参考
          ref_r = r;
        } else {
          non_ground->points.push_back(pts[i].second);
          // 不更新参考：障碍物之后的点继续用原地面参考判断
        }
      }
    }
  }

private:
  double height_threshold_;
  double ray_resolution_;
  double max_slope_;
};

}  // namespace lidar_perception

#endif  // LIDAR_PERCEPTION__GROUND_FILTER_HPP_
