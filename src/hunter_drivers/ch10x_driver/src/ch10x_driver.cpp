// Copyright 2026 HUNTER Development Team
// CH10X IMU 驱动实现（UART 读取 + 协议解析 + 生命周期 + 异常处理）
#include "ch10x_driver/ch10x_driver.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <utility>

namespace ch10x_driver
{

namespace
{
// CRC16/XMODEM：poly=0x1021, init=0x0000, 无反射/无异或（厂商 crc16_update）
uint16_t crc16XmodemUpdate(uint16_t crc, const uint8_t * data, size_t n)
{
  for (size_t i = 0; i < n; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int j = 0; j < 8; ++j) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

// 单位换算常量：加速度 G→m/s²，角速度 °/s→rad/s
constexpr float kGravity = 9.80665f;
constexpr float kDegToRad = 0.017453292519943295f;
}  // namespace

Ch10xDriver::Ch10xDriver(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("ch10x_driver", options),
  fd_(-1),
  running_(false),
  serial_ok_(false),
  last_data_time_(std::chrono::steady_clock::now())
{
  // 声明参数（文档 3.3.3：UART、100Hz；文档 7.4：frame_id = "imu"）
  declare_parameter("port", "/dev/ttyUSB0");
  declare_parameter("baud_rate", 115200);
  declare_parameter("frame_id", "imu");
  declare_parameter("timeout", 0.2);  // 数据超时阈值（秒）
}

Ch10xDriver::CallbackReturn
Ch10xDriver::on_configure(const rclcpp_lifecycle::State &)
{
  port_ = get_parameter("port").as_string();
  baud_rate_ = static_cast<int>(get_parameter("baud_rate").as_int());
  frame_id_ = get_parameter("frame_id").as_string();
  timeout_ = get_parameter("timeout").as_double();

  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data",
    rclcpp::SensorDataQoS(rclcpp::KeepLast(500)));

  if (!openSerial()) {
    RCLCPP_ERROR(get_logger(), "串口打开失败，配置阶段失败");
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(
    get_logger(), "CH10X IMU 已配置：port=%s, baud=%d, frame_id=%s, timeout=%.2fs",
    port_.c_str(), baud_rate_, frame_id_.c_str(), timeout_);
  return CallbackReturn::SUCCESS;
}

Ch10xDriver::CallbackReturn
Ch10xDriver::on_activate(const rclcpp_lifecycle::State &)
{
  imu_pub_->on_activate();
  running_.store(true);
  serial_ok_.store(true);
  last_data_time_ = std::chrono::steady_clock::now();

  read_thread_ = std::thread(&Ch10xDriver::readLoop, this);

  timeout_timer_ = create_wall_timer(
    std::chrono::milliseconds(1000),
    std::bind(&Ch10xDriver::checkTimeout, this));

  RCLCPP_INFO(get_logger(), "CH10X IMU 已激活，发布 /imu/data @ 100Hz");
  return CallbackReturn::SUCCESS;
}

Ch10xDriver::CallbackReturn
Ch10xDriver::on_deactivate(const rclcpp_lifecycle::State &)
{
  running_.store(false);
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
  timeout_timer_.reset();
  imu_pub_->on_deactivate();
  RCLCPP_INFO(get_logger(), "CH10X IMU 已停用");
  return CallbackReturn::SUCCESS;
}

Ch10xDriver::CallbackReturn
Ch10xDriver::on_cleanup(const rclcpp_lifecycle::State &)
{
  closeSerial();
  imu_pub_.reset();
  rx_buffer_.clear();
  RCLCPP_INFO(get_logger(), "CH10X IMU 已清理");
  return CallbackReturn::SUCCESS;
}

Ch10xDriver::CallbackReturn
Ch10xDriver::on_shutdown(const rclcpp_lifecycle::State &)
{
  running_.store(false);
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
  closeSerial();
  RCLCPP_INFO(get_logger(), "CH10X IMU 已关闭");
  return CallbackReturn::SUCCESS;
}

bool Ch10xDriver::openSerial()
{
  fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    RCLCPP_ERROR(get_logger(), "无法打开串口 %s: %s", port_.c_str(), strerror(errno));
    return false;
  }

  struct termios tty;
  if (tcgetattr(fd_, &tty) != 0) {
    RCLCPP_ERROR(get_logger(), "tcgetattr 失败: %s", strerror(errno));
    close(fd_);
    fd_ = -1;
    return false;
  }

  speed_t baud = B115200;
  switch (baud_rate_) {
    case 9600: baud = B9600; break;
    case 19200: baud = B19200; break;
    case 38400: baud = B38400; break;
    case 57600: baud = B57600; break;
    case 115200: baud = B115200; break;
    case 230400: baud = B230400; break;
    default: baud = B115200; break;
  }
  cfsetospeed(&tty, baud);
  cfsetispeed(&tty, baud);

  // 8N1，无流控
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;

  // 原始模式
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR);
  tty.c_oflag &= ~OPOST;

  // 读超时：VMIN=1, VTIME=1（0.1s 超时）
  tty.c_cc[VMIN] = 1;
  tty.c_cc[VTIME] = 1;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(get_logger(), "tcsetattr 失败: %s", strerror(errno));
    close(fd_);
    fd_ = -1;
    return false;
  }

  tcflush(fd_, TCIOFLUSH);
  serial_ok_.store(true);
  return true;
}

void Ch10xDriver::closeSerial()
{
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
  serial_ok_.store(false);
}

void Ch10xDriver::readLoop()
{
  constexpr size_t kChunk = 512;
  uint8_t buf[kChunk];

  while (running_.load()) {
    if (!serial_ok_.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    ssize_t n = read(fd_, buf, kChunk);
    if (n > 0) {
      last_data_time_ = std::chrono::steady_clock::now();
      rx_buffer_.insert(rx_buffer_.end(), buf, buf + n);

      ImuData data;
      int ret;
      while ((ret = parseFrame(rx_buffer_, data)) == 1) {
        publishImu(data);
      }

      // 防止异常时缓冲区无限增长
      if (rx_buffer_.size() > 4096) {
        rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.end() - 4096);
      }
    } else if (n == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } else {
      // 读错误：区分非阻塞无数据 与 串口断开
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      } else {
        // 串口断开/设备移除（EIO、ENODEV 等）
        RCLCPP_ERROR(get_logger(), "串口读取错误: %s，尝试重连", strerror(errno));
        serial_ok_.store(false);
        closeSerial();
        while (running_.load() && !serial_ok_.load()) {
          std::this_thread::sleep_for(std::chrono::seconds(1));
          if (!running_.load()) {
            break;
          }
          RCLCPP_INFO(get_logger(), "尝试重新打开串口 %s ...", port_.c_str());
          if (openSerial()) {
            RCLCPP_INFO(get_logger(), "串口重连成功");
          }
        }
      }
    }
  }
}

int Ch10xDriver::parseFrame(std::vector<uint8_t> & buffer, ImuData & data)
{
  // 1. 帧头同步：丢弃帧头之前的字节
  size_t i = 0;
  const size_t n = buffer.size();
  while (i + 1 < n) {
    if (buffer[i] == FRAME_HEAD_1 && buffer[i + 1] == FRAME_HEAD_2) {
      break;
    }
    i++;
  }
  if (i > 0) {
    buffer.erase(buffer.begin(), buffer.begin() + i);
  }
  if (buffer.size() < 2) {
    return 0;  // 数据不足
  }

  // 2. 帧头已在开头，等待完整帧头字段（2帧头+2长度+2CRC）
  if (buffer.size() < FRAME_HEADER_LEN) {
    return 0;  // 数据不足
  }

  // 3. 数据域长度（小端）：buf[2] | buf[3]<<8；上限保护防伪帧
  const uint16_t len = static_cast<uint16_t>(buffer[2] | (buffer[3] << 8));
  if (len < 12 || len > 512) {
    buffer.erase(buffer.begin());  // 长度非法，丢弃 1 字节重新同步
    return -1;
  }
  const size_t total = FRAME_HEADER_LEN + len;
  if (buffer.size() < total) {
    return 0;  // 载荷未收齐
  }

  // 4. CRC16 校验：先算 5A A5 + LEN（buf[0..4)），再算数据域（buf[6..6+LEN)）
  uint16_t crc = 0;
  crc = crc16XmodemUpdate(crc, buffer.data(), 4);
  crc = crc16XmodemUpdate(crc, buffer.data() + FRAME_HEADER_LEN, len);
  const uint16_t expected = static_cast<uint16_t>(buffer[4] | (buffer[5] << 8));
  if (crc != expected) {
    buffer.erase(buffer.begin());  // 校验失败，丢弃 1 字节重新同步
    return -1;
  }

  // 5. 仅处理 0x91 数据包；其它类型整帧跳过
  if (buffer[6] != PACKET_TAG_91 || len != PAYLOAD_LEN_91) {
    buffer.erase(buffer.begin(), buffer.begin() + total);
    return 0;
  }

  // 6. 解析 0x91 数据域（数据域起始 = buffer[6]，全部小端）
  const uint8_t * p = buffer.data() + FRAME_HEADER_LEN;
  float acc[3] = {0.f, 0.f, 0.f};
  float gyr[3] = {0.f, 0.f, 0.f};
  float mag[3] = {0.f, 0.f, 0.f};
  float eul[3] = {0.f, 0.f, 0.f};
  float quat[4] = {1.f, 0.f, 0.f, 0.f};

  // +1  id, +2..3 reserved
  std::memcpy(&data.timestamp_ms, p + 8, sizeof(uint32_t));      // ms
  std::memcpy(acc,  p + 12, 3 * sizeof(float));                  // G
  std::memcpy(gyr,  p + 24, 3 * sizeof(float));                  // °/s
  std::memcpy(mag,  p + 36, 3 * sizeof(float));                  // uT（不发布，仅保留）
  std::memcpy(eul,  p + 48, 3 * sizeof(float));                  // °
  std::memcpy(quat, p + 60, 4 * sizeof(float));                  // w,x,y,z

  // 单位换算到 ROS 约定：角速度 rad/s、加速度 m/s²、欧拉角 rad
  for (int k = 0; k < 3; ++k) {
    data.angular_velocity[k]    = gyr[k] * kDegToRad;
    data.linear_acceleration[k] = acc[k] * kGravity;
    data.euler[k]               = eul[k] * kDegToRad;
  }
  data.quaternion[0] = quat[0];
  data.quaternion[1] = quat[1];
  data.quaternion[2] = quat[2];
  data.quaternion[3] = quat[3];

  buffer.erase(buffer.begin(), buffer.begin() + total);
  return 1;
}

void Ch10xDriver::publishImu(const ImuData & data)
{
  if (!imu_pub_ || !imu_pub_->is_activated()) {
    return;
  }

  auto msg = std::make_unique<sensor_msgs::msg::Imu>();
  msg->header.stamp = this->now();
  msg->header.frame_id = frame_id_;

  // 四元数
  msg->orientation.w = data.quaternion[0];
  msg->orientation.x = data.quaternion[1];
  msg->orientation.y = data.quaternion[2];
  msg->orientation.z = data.quaternion[3];

  // 角速度 (rad/s)
  msg->angular_velocity.x = data.angular_velocity[0];
  msg->angular_velocity.y = data.angular_velocity[1];
  msg->angular_velocity.z = data.angular_velocity[2];

  // 线加速度 (m/s²)
  msg->linear_acceleration.x = data.linear_acceleration[0];
  msg->linear_acceleration.y = data.linear_acceleration[1];
  msg->linear_acceleration.z = data.linear_acceleration[2];

  // 协方差：姿态按文档 3.3.3 静态精度 0.8°(≈0.014rad)、航向 3° 计
  const double ori_var = 0.014 * 0.014;
  msg->orientation_covariance[0] = ori_var;
  msg->orientation_covariance[4] = ori_var;
  msg->orientation_covariance[8] = 0.052 * 0.052;
  msg->angular_velocity_covariance[0] = 0.01;
  msg->angular_velocity_covariance[4] = 0.01;
  msg->angular_velocity_covariance[8] = 0.01;
  msg->linear_acceleration_covariance[0] = 0.1;
  msg->linear_acceleration_covariance[4] = 0.1;
  msg->linear_acceleration_covariance[8] = 0.1;

  imu_pub_->publish(std::move(msg));
}

void Ch10xDriver::checkTimeout()
{
  if (!serial_ok_.load()) {
    return;
  }
  const double elapsed =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - last_data_time_).count();
  if (elapsed > timeout_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "IMU 数据超时：%.2fs 未收到数据（阈值 %.2fs）", elapsed, timeout_);
  }
}

}  // namespace ch10x_driver

