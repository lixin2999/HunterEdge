// Copyright 2026 HUNTER Development Team
// CH10X IMU 驱动节点（文档 3.3.3）
#ifndef CH10X_DRIVER__CH10X_DRIVER_HPP_
#define CH10X_DRIVER__CH10X_DRIVER_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace ch10x_driver
{

// CH10X 协议帧常量（文档 3.3.3：UART 自定义协议，含四元数/欧拉角/角速度/加速度）
// 帧结构：0xAA 0x55 | TYPE(1B) | LEN(2B,小端) | PAYLOAD | CHECKSUM(1B)
constexpr uint8_t FRAME_HEAD_1 = 0xAA;
constexpr uint8_t FRAME_HEAD_2 = 0x55;
constexpr uint8_t FRAME_TYPE_IMU = 0x01;
// PAYLOAD = timestamp(4) + quaternion(4*4) + euler(3*4) + gyro(3*4) + accel(3*4)
constexpr uint16_t PAYLOAD_LEN = 56;
constexpr size_t FRAME_OVERHEAD = 5;   // 帧头2 + 类型1 + 长度2
constexpr size_t FRAME_CHECKSUM = 1;   // 校验 1 字节

// IMU 数据帧（协议解析结果）
struct ImuData
{
  uint32_t timestamp_ms;
  float quaternion[4];          // w, x, y, z
  float euler[3];               // roll, pitch, yaw (rad)
  float angular_velocity[3];    // wx, wy, wz (rad/s)
  float linear_acceleration[3]; // ax, ay, az (m/s²)
};

class Ch10xDriver : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit Ch10xDriver(const rclcpp::NodeOptions & options);

  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  // 串口
  bool openSerial();
  void closeSerial();
  // 读取线程
  void readLoop();
  // 协议解析：返回 1=成功, 0=数据不足, -1=帧错误(已丢弃1字节重新同步)
  int parseFrame(std::vector<uint8_t> & buffer, ImuData & data);
  // 发布与超时
  void publishImu(const ImuData & data);
  void checkTimeout();

  // 参数（on_configure 声明）
  std::string port_;
  int baud_rate_;
  std::string frame_id_;
  double timeout_;

  // 串口与线程
  int fd_;
  std::atomic<bool> running_;
  std::atomic<bool> serial_ok_;
  std::thread read_thread_;
  std::vector<uint8_t> rx_buffer_;
  std::chrono::steady_clock::time_point last_data_time_;

  // 发布器与定时器
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
};

}  // namespace ch10x_driver

#endif  // CH10X_DRIVER__CH10X_DRIVER_HPP_
