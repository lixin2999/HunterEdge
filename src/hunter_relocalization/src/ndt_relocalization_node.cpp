// ndt_relocalization_node.cpp
// 方案A（V0.0.93）全局重定位：PCL NDT 将 FAST-LIO2 实时点云（odom 系）配准到
// 先验全局点云地图（.pcd，map 系），估计/持续修正 map→odom TF。
//
// 设计要点：
//  - map→odom 是本节点唯一广播的 TF 边；odom→base_link 归 FAST-LIO2；底盘/EKF 均 publish_tf=false。
//  - 初值 T_map_odom 默认单位阵（假设上电≈建图起点），可用 /initialpose 重设。
//  - 收敛判据：NDT hasConverged() 且 fitness<=fitness_max，且单周跳变在闸门内；否则维持旧值
//    并发布大协方差（auto_mission 门控据此判断“定位未收敛”）。
//  - 大数据安全：全局 .pcd 体素降采样为 NDT target；滚动时间窗累积局部子图并降采样为 source。
#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/common/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>

using namespace std::chrono_literals;

class NdtRelocalization : public rclcpp::Node
{
public:
  NdtRelocalization()
  : Node("hunter_relocalization")
  {
    // ---- 参数 ----
    global_map_path_    = declare_parameter<std::string>("global_map_path", "");
    world_frame_        = declare_parameter<std::string>("world_frame", "map");   // NDT target 系
    odom_frame_         = declare_parameter<std::string>("odom_frame", "odom");    // NDT source 系（实时云）
    cloud_topic_        = declare_parameter<std::string>("cloud_topic", "/cloud_registered");
    publish_rate_       = declare_parameter<double>("publish_rate", 2.0);          // Hz，NDT 较重，低频足够
    accumulate_window_  = declare_parameter<double>("accumulate_window", 2.0);     // s，滚动子图时间窗
    // 全局地图目标体素（NDT 分辨率量级）；局部源点云体素
    target_voxel_       = declare_parameter<double>("target_voxel_size", 0.5);
    source_voxel_       = declare_parameter<double>("source_voxel_size", 0.25);
    // NDT 超参
    ndt_resolution_     = declare_parameter<double>("ndt_resolution", 1.0);
    ndt_step_size_      = declare_parameter<double>("ndt_step_size", 0.1);
    ndt_epsilon_        = declare_parameter<double>("ndt_epsilon", 0.01);
    ndt_max_iters_      = declare_parameter<int>("ndt_max_iterations", 30);
    // 收敛/跳变闸门
    fitness_max_        = declare_parameter<double>("fitness_max", 1.0);
    max_step_trans_     = declare_parameter<double>("max_step_translation", 0.5);  // m / 周期
    max_step_rot_       = declare_parameter<double>("max_step_rotation", 0.35);    // rad / 周期
    converged_cov_      = declare_parameter<double>("converged_covariance", 0.01); // 收敛时发布的小协方差

    // ---- 加载全局地图 ----
    if (!loadGlobalMap()) {
      RCLCPP_ERROR(get_logger(),
        "[relocalization] 全局地图加载失败：'%s' 不存在或为空。map→odom 将不可用。",
        global_map_path_.c_str());
    }

    // ---- TF 监听（把异帧点云归一到 odom；/initialpose 反解用 odom→base） ----
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // ---- 订阅实时点云 ----
    auto qoss = rclcpp::SensorDataQoS();
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, qoss,
      std::bind(&NdtRelocalization::cloudCallback, this, std::placeholders::_1));

    // ---- 订阅初始位姿（rviz2 "2D/先验估计" 或上层发布） ----
    initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", rclcpp::QoS(1),
      std::bind(&NdtRelocalization::initialPoseCallback, this, std::placeholders::_1));

    // ---- 发布重定位位姿（供 auto_mission / safety_guard 判定 map 系位姿与收敛性） ----
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/relocalization/pose", rclcpp::QoS(1));

    // ---- 主循环定时器 ----
    const double dt = (publish_rate_ > 1e-6) ? (1.0 / publish_rate_) : 0.5;
    timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(dt * 1000)),
      std::bind(&NdtRelocalization::alignOnce, this));

    RCLCPP_INFO(get_logger(),
      "[relocalization] 启动：map=%s odom=%s cloud=%s rate=%.1fHz map='%s'",
      world_frame_.c_str(), odom_frame_.c_str(), cloud_topic_.c_str(),
      publish_rate_, global_map_path_.c_str());
  }

private:
  // ---------- 全局地图 ----------
  bool loadGlobalMap()
  {
    if (global_map_path_.empty()) return false;
    pcl::PointCloud<pcl::PointXYZI>::Ptr raw(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(global_map_path_, *raw) == -1) {
      RCLCPP_ERROR(get_logger(), "[relocalization] loadPCDFile 失败：%s", global_map_path_.c_str());
      return false;
    }
    if (raw->empty()) {
      RCLCPP_ERROR(get_logger(), "[relocalization] 全局地图为空：%s", global_map_path_.c_str());
      return false;
    }
    // 体素降采样（大图 NDT 目标必须稀疏化，否则内存/耗时爆炸）
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(raw);
    vg.setLeafSize(target_voxel_, target_voxel_, target_voxel_);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ds(new pcl::PointCloud<pcl::PointXYZI>);
    vg.filter(*ds);

    ndt_->setInputTarget(ds);
    ndt_->setResolution(ndt_resolution_);
    ndt_->setStepSize(ndt_step_size_);
    ndt_->setTransformationEpsilon(ndt_epsilon_);
    ndt_->setMaximumIterations(ndt_max_iters_);
    target_ = ds;

    RCLCPP_INFO(get_logger(),
      "[relocalization] 全局地图就绪：原始 %zu 点 → 降采样(%.2fm) %zu 点，NDT 分辨率 %.2fm",
      raw->size(), target_voxel_, ds->size(), ndt_resolution_);
    return true;
  }

  // ---------- 实时点云回调：滚动时间窗累积 ----------
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    auto cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PCLPointCloud2 proto;
    pcl_conversions::toPCL(*msg, proto);
    pcl::fromPCLPointCloud2(proto, *cloud);
    if (cloud->empty()) return;

    // 若点云不在 odom 系，尝试经 TF 变换到 odom（正常 FAST-LIO2 输出即 odom，变换为恒等）
    if (!msg->header.frame_id.empty() && msg->header.frame_id != odom_frame_) {
      geometry_msgs::msg::TransformStamped ts;
      try {
        ts = tf_buffer_->lookupTransform(odom_frame_, msg->header.frame_id,
                                         tf2::TimePointZero, tf2::durationFromSec(0.05));
        tf2::Transform tf;
        tf2::fromMsg(ts.transform, tf);
        const Eigen::Matrix4f aff = toMat4(tf);
        pcl::transformPointCloud(*cloud, *cloud, aff);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "[relocalization] 点云帧 %s→%s TF 不可用：%s（按原帧使用）",
          msg->header.frame_id.c_str(), odom_frame_.c_str(), ex.what());
      }
    }

    const double stamp = rclcpp::Time(msg->header.stamp).seconds();
    std::lock_guard<std::mutex> lk(src_mtx_);
    src_window_.push_back({stamp, cloud});
    // 丢弃窗口外的旧帧
    while (!src_window_.empty() && (stamp - src_window_.front().stamp) > accumulate_window_) {
      src_window_.pop_front();
    }
  }

  // ---------- 初始位姿：反解 T_map_odom 作为 NDT 初值 ----------
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    // 期望：机器人在 map 系的位姿。结合当前 odom→base TF 求 T_map_odom。
    geometry_msgs::msg::TransformStamped ts;
    try {
      ts = tf_buffer_->lookupTransform(odom_frame_, "base_link", tf2::TimePointZero,
                                       tf2::durationFromSec(0.2));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(),
        "[relocalization] /initialpose 需要 %s→base_link TF：%s，暂忽略",
        odom_frame_.c_str(), ex.what());
      return;
    }
    tf2::Transform t_odom_base; tf2::fromMsg(ts.transform, t_odom_base);
    tf2::Transform t_map_base;  tf2::fromMsg(msg->pose.pose, t_map_base);
    // T_map_odom = T_map_base * inverse(T_odom_base)
    Eigen::Matrix4f T = toMat4(t_map_base * t_odom_base.inverse());
    std::lock_guard<std::mutex> lk(t_mtx_);
    T_map_odom_ = T;
    has_prev_   = true;
    RCLCPP_INFO(get_logger(), "[relocalization] 已用 /initialpose 重置 map→odom 初值。");
  }

  // ---------- 每个周期：拼子图 → NDT → 闸门 → 广播 ----------
  void alignOnce()
  {
    if (!ndt_ || target_ == nullptr) {
      publishPose(1e8, 100.0, rclcpp::Time(now()));  // 无地图：大协方差=未收敛
      return;
    }

    // 1) 组装局部源点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr src(new pcl::PointCloud<pcl::PointXYZI>);
    {
      std::lock_guard<std::mutex> lk(src_mtx_);
      for (auto & fr : src_window_) *src += *fr.cloud;
    }
    if (src->size() < 50) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "[relocalization] 局部点云过少(%zu)，等待传感器...", src->size());
      publishPose(1e8, 100.0, rclcpp::Time(now()));
      return;
    }
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(src);
    vg.setLeafSize(source_voxel_, source_voxel_, source_voxel_);
    pcl::PointCloud<pcl::PointXYZI>::Ptr src_ds(new pcl::PointCloud<pcl::PointXYZI>);
    vg.filter(*src_ds);

    // 2) NDT 配准（初值 = 上次 T_map_odom）
    ndt_->setInputSource(src_ds);
    Eigen::Matrix4f guess;
    { std::lock_guard<std::mutex> lk(t_mtx_); guess = T_map_odom_; }
    pcl::PointCloud<pcl::PointXYZI>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZI>);
    ndt_->align(*aligned, guess);
    Eigen::Matrix4f result = ndt_->getFinalTransformation();
    const float fitness = ndt_->getFitnessScore();
    const bool converged = ndt_->hasConverged() && (fitness <= fitness_max_);

    // 3) 跳变闸门
    bool step_ok = true;
    if (has_prev_) {
      Eigen::Matrix4f delta = guess.inverse() * result;
      double dtrans = delta.block<3,1>(0,3).norm();
      double drot   = std::acos(std::max(-1.0, std::min(1.0,
                    (delta.block<3,3>(0,3).trace() - 1.0) * 0.5)));
      step_ok = (dtrans <= max_step_trans_) && (drot <= max_step_rot_);
    }

    rclcpp::Time stamp(now());
    if (converged && step_ok) {
      std::lock_guard<std::mutex> lk(t_mtx_);
      T_map_odom_ = result;
      has_prev_   = true;
      sendTf(result, stamp);
      publishPose(fitness, converged_cov_, stamp);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 4000,
        "[relocalization] ✓ 收敛 fit=%.3f 位置(%.2f,%.2f,%.2f)",
        fitness, result(0,3), result(1,3), result(2,3));
    } else {
      // 维持旧 TF，发布大协方差告知门控“未收敛”
      Eigen::Matrix4f keep;
      { std::lock_guard<std::mutex> lk(t_mtx_); keep = T_map_odom_; }
      sendTf(keep, stamp);
      publishPose(fitness, 100.0, stamp);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "[relocalization] ✗ 未收敛(converged=%d fit=%.3f<=%.2f step_ok=%d)，保持上次位姿",
        static_cast<int>(ndt_->hasConverged()), fitness, fitness_max_, static_cast<int>(step_ok));
    }
  }

  // ---------- 广播 map→odom TF ----------
  void sendTf(const Eigen::Matrix4f & T, const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::TransformStamped ts;
    ts.header.stamp    = stamp;
    ts.header.frame_id = world_frame_;
    ts.child_frame_id  = odom_frame_;
    ts.transform.translation.x = T(0,3);
    ts.transform.translation.y = T(1,3);
    ts.transform.translation.z = T(2,3);
    Eigen::Quaternionf q(T.block<3,3>(0,3));
    q.normalize();
    ts.transform.rotation.x = q.x();
    ts.transform.rotation.y = q.y();
    ts.transform.rotation.z = q.z();
    ts.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(ts);
  }

  // ---------- 发布 map 系机器人位姿 + 收敛性（cov[0]=cov[7]=cov_val） ----------
  void publishPose(float fitness, double cov_val, const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped p;
    p.header.stamp    = stamp;
    p.header.frame_id = world_frame_;
    Eigen::Matrix4f T;
    { std::lock_guard<std::mutex> lk(t_mtx_); T = T_map_odom_; }
    // 机器人在 map 的位姿 = T_map_odom * (odom→base)。若无 odom→base 则退化为 T_map_odom 平移。
    Eigen::Matrix4f T_odom_base = Eigen::Matrix4f::Identity();
    try {
      auto ts = tf_buffer_->lookupTransform(odom_frame_, "base_link",
                                            tf2::TimePointZero, tf2::durationFromSec(0.05));
      tf2::Transform tb; tf2::fromMsg(ts.transform, tb);
      T_odom_base = toMat4(tb);
    } catch (const tf2::TransformException &) { /* 保持单位，退化可用 */ }

    Eigen::Matrix4f T_map_base = T * T_odom_base;
    p.pose.pose.position.x = T_map_base(0,3);
    p.pose.pose.position.y = T_map_base(1,3);
    p.pose.pose.position.z = T_map_base(2,3);
    Eigen::Quaternionf q(T_map_base.block<3,3>(0,3)); q.normalize();
    p.pose.pose.orientation.w = q.w();
    p.pose.pose.orientation.x = q.x();
    p.pose.pose.orientation.y = q.y();
    p.pose.pose.orientation.z = q.z();
    for (int i = 0; i < 36; ++i) p.pose.covariance[i] = 0.0;
    p.pose.covariance[0]  = cov_val;   // x
    p.pose.covariance[7]  = cov_val;   // y
    p.pose.covariance[35] = cov_val;   // yaw
    pose_pub_->publish(p);
    (void)fitness;
  }

  static Eigen::Matrix4f toMat4(const tf2::Transform & t)
  {
    tf2::Matrix3x3 R = t.getBasis();
    Eigen::Matrix4f M = Eigen::Matrix4f::Identity();
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) M(i,j) = R.getRow(i)[j];
    M(0,3) = t.getOrigin().x();
    M(1,3) = t.getOrigin().y();
    M(2,3) = t.getOrigin().z();
    return M;
  }

  // ---------- 成员 ----------
  std::string global_map_path_, world_frame_, odom_frame_, cloud_topic_;
  double publish_rate_, accumulate_window_, target_voxel_, source_voxel_;
  double ndt_resolution_, ndt_step_size_, ndt_epsilon_, fitness_max_;
  double max_step_trans_, max_step_rot_, converged_cov_;
  int ndt_max_iters_;

  pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt_
    = std::make_shared<pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>>();
  pcl::PointCloud<pcl::PointXYZI>::Ptr target_{nullptr};

  struct Frame { double stamp; pcl::PointCloud<pcl::PointXYZI>::Ptr cloud; };
  std::deque<Frame> src_window_;
  std::mutex src_mtx_;

  Eigen::Matrix4f T_map_odom_ = Eigen::Matrix4f::Identity();
  bool has_prev_ = false;
  std::mutex t_mtx_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NdtRelocalization>());
  rclcpp::shutdown();
  return 0;
}
