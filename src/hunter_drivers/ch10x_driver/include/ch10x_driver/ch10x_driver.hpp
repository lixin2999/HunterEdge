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

// CH10X 串口协议（按厂商手册实测确认，帧长 82 字节 @100Hz）：
//   5A A5 | LEN(2B 小端) | CRC16(2B 小端) | PAYLOAD(LEN 字节)
//   LEN 即 0x91 数据域长度 = 76
//   CRC16/XMODEM：初值 0x0000，先对 buf[0..4)（帧头+长度）更新，
//   再接着对 buf[6..6+LEN)（数据域）更新，结果 == buf[4] | (buf[5]<<8)
// 0x91 数据域布局（相对数据域起始 buf[6]）：
//   +0  tag=0x91   +1  id(1B)     +2..3 reserved
//   +4  pressure(float32, Pa)     +8 timestamp(uint32, ms)
//   +12 acc[3](float32, 单位 G)   +24 gyr[3](float32, 单位 °/s)
//   +36 mag[3](float32, 单位 uT)  +48 eul[3](float32, 单位 °)
//   +60 quat[4](float32, w,x,y,z)
constexpr uint8_t FRAME_HEAD_1 = 0x5A;
constexpr uint8_t FRAME_HEAD_2 = 0xA5;
constexpr uint8_t PACKET_TAG_91 = 0x91;
constexpr size_t FRAME_HEADER_LEN = 6;   // 帧头2 + 长度2 + CRC16 2
constexpr size_t PAYLOAD_LEN_91 = 76;    // 0x91 数据包的数据域长度

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
