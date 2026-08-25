// Copyright 2026 HUNTER Development Team
// 多目标跟踪实现：匈牙利匹配 + 卡尔曼(α-β 稳态)滤波
#include "lidar_perception/tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace lidar_perception
{

// 标准匈牙利算法（最小代价指派，O(n³)）
// cost: N×N 方阵；返回 assignment[i] = 匹配的列 j（-1 表示未匹配）
static std::vector<int> hungarian(const std::vector<std::vector<double>> & cost)
{
  const int n = static_cast<int>(cost.size());
  if (n == 0) {
    return {};
  }
  const double INF = std::numeric_limits<double>::max();

  std::vector<double> u(n + 1, 0.0), v(n + 1, 0.0);
  std::vector<int> p(n + 1, 0), way(n + 1, 0);

  for (int i = 1; i <= n; ++i) {
    p[0] = i;
    int j0 = 0;
    std::vector<double> minv(n + 1, INF);
    std::vector<bool> used(n + 1, false);
    do {
      used[j0] = true;
      const int i0 = p[j0];
      int j1 = 0;
      double delta = INF;
      for (int j = 1; j <= n; ++j) {
        if (!used[j]) {
          const double cur = cost[i0 - 1][j - 1] - u[i0] - v[j];
          if (cur < minv[j]) {
            minv[j] = cur;
            way[j] = j0;
          }
          if (minv[j] < delta) {
            delta = minv[j];
            j1 = j;
          }
        }
      }
      for (int j = 0; j <= n; ++j) {
        if (used[j]) {
          u[p[j]] += delta;
          v[j] -= delta;
        } else {
          minv[j] -= delta;
        }
      }
      j0 = j1;
    } while (p[j0] != 0);

    do {
      const int j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0 != 0);
  }

  std::vector<int> assignment(n, -1);
  for (int j = 1; j <= n; ++j) {
    if (p[j] > 0) {
      assignment[p[j] - 1] = j - 1;
    }
  }
  return assignment;
}

void MultiObjectTracker::predict(double dt)
{
  for (auto & t : tracks_) {
    // 匀速运动模型
    t.x += t.vx * dt;
    t.y += t.vy * dt;
    t.age++;
    t.missed++;
  }
}

void MultiObjectTracker::updateTrack(Track & t, const Detection & d, double dt)
{
  // 卡尔曼（α-β 稳态）滤波：状态 [x, y, vx, vy]，观测 [x, y]
  const double alpha = 0.5;
  const double beta = 0.1;
  const double rx = d.x - t.x;
  const double ry = d.y - t.y;
  t.x += alpha * rx;
  t.y += alpha * ry;
  if (dt > 1e-6) {
    t.vx += beta * rx / dt;
    t.vy += beta * ry / dt;
  }
  t.missed = 0;
  t.confirmed = true;
  t.last_det = d;
}

std::vector<Track> MultiObjectTracker::update(
  const std::vector<Detection> & detections, double dt)
{
  if (dt <= 0.0) {
    dt = 0.1;
  }

  predict(dt);

  const int n = static_cast<int>(tracks_.size());
  const int m = static_cast<int>(detections.size());
  const int N = std::max(n, m);

  // 构建距离矩阵（N×N）
  std::vector<std::vector<double>> cost(N, std::vector<double>(N, 1e6));
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < m; ++j) {
      const double dx = tracks_[i].x - detections[j].x;
      const double dy = tracks_[i].y - detections[j].y;
      const double d = std::sqrt(dx * dx + dy * dy);
      if (d <= association_distance_) {
        cost[i][j] = d;
      } else {
        cost[i][j] = 1e6;  // 超过关联距离阈值，视为不可匹配
      }
    }
  }

  const auto assignment = hungarian(cost);

  // 应用匹配结果
  std::vector<bool> matched_det(m, false);
  for (int i = 0; i < n; ++i) {
    const int j = assignment[i];
    if (j >= 0 && j < m && cost[i][j] < 1e6) {
      updateTrack(tracks_[i], detections[j], dt);
      matched_det[j] = true;
    }
  }

  // 未匹配的检测 → 新生目标
  for (int j = 0; j < m; ++j) {
    if (!matched_det[j]) {
      Track t;
      t.id = next_id_++;
      t.x = detections[j].x;
      t.y = detections[j].y;
      t.vx = 0.0;
      t.vy = 0.0;
      t.age = 0;
      t.missed = 0;
      t.confirmed = false;
      t.last_det = detections[j];
      tracks_.push_back(t);
    }
  }

  // 移除长时间丢失的目标（生命周期：消失）
  tracks_.erase(
    std::remove_if(
      tracks_.begin(), tracks_.end(),
      [](const Track & t) { return t.missed > 5; }),
    tracks_.end());

  return tracks_;
}

}  // namespace lidar_perception
