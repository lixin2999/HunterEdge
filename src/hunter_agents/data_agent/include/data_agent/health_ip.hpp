// Copyright 2026 HUNTER Development Team
// health 心跳 `ip` 字段：车端网络地址探测 + 契约校验（HunterCore V1.28.0 起）
//
// 契约（contracts/kafka/schemas/health.schema.json，additionalProperties:false）：
//   顶层可选 `ip`：string，minLength 3 / maxLength 45，IPv4 点分十进制或压缩形式 IPv6。
//   未上报（键缺失）= 平台不回写台账，保留人工录入值；**上报非法值则整条 health 进 DLQ**，
//   该车在线态/状态更新全部断更。故本模块的铁律是：
//     - 只产出**通过契约校验**的地址；
//     - 探测失败/非法固定值 → 返回空串，调用方**省略 ip 键**（绝不写 null / "" / "unknown"）。
//
// 本文件不依赖 rclcpp，可在无 ROS 环境单测；`current()` 约定只在 health 定时器线程调用。
#ifndef DATA_AGENT__HEALTH_IP_HPP_
#define DATA_AGENT__HEALTH_IP_HPP_

#include <chrono>
#include <string>

namespace data_agent
{

class HealthIpReporter
{
public:
  // 模式（与平台侧模拟器 --ip auto|none|<IP> 对齐）：
  //   auto  自动探测默认路由出口地址，失败则省略 ip；
  //   off   永不携带 ip（等同旧版报文，"none" 亦视作 off）；
  //   <IP>  固定值（产线/封闭场地调试），非法值按 off 处理并返回 false。
  enum class Mode
  {
    kAuto,
    kOff,
    kPinned,
  };

  HealthIpReporter() = default;

  // 配置模式；返回 false 表示固定值非法（已自动降级为 off，绝不发脏值）。
  bool configure(const std::string & raw_mode, double ttl_s = 60.0);

  // 解析配置：ROS 参数非空优先，其次环境变量 HEALTH_IP_REPORT，最后 auto。
  static std::string resolveMode(const std::string & param_value, const char * env_name);

  // 当前应上报的地址；空串 = 调用方省略 ip 键。
  std::string current();

  Mode mode() const {return mode_;}
  bool enabled() const {return mode_ != Mode::kOff;}
  const std::string & pinned() const {return pinned_;}

  // 契约校验（IPv4 点分十进制 4 段 0-255 无前导零 / IPv6 无 zone、无 IPv4 内嵌）。
  static bool isValid(const std::string & value);
  // 地址是否可上报：排除 loopback / link-local / fe80::/10 / 未指定地址，
  // 以及 docker0、br-*、veth*、tun*、wg* 等容器/VPN 虚接口（多网卡误报的主因）。
  static bool acceptableAddress(const std::string & ip, const std::string & if_name);
  // 合法 → 规范形式（IPv6 压缩小写）；非法 → 空串。
  static std::string normalize(const std::string & value);

  // 网络事件（link up/down、DHCP 续租、WiFi↔4G 切换）后主动丢缓存，下一拍重采样。
  void invalidate();

private:
  // 默认路由出口地址：UDP connect 到公网字面量地址（不发包、不做 DNS、无需 root）。
  static bool detect(std::string & out);
  // 接口指纹：up 且非 loopback 的地址集合 + 默认路由出口接口；变化即视为"网络事件"。
  static std::string interfaceSnapshot();
  static std::string interfaceOf(const std::string & ip);

  Mode mode_{Mode::kAuto};
  std::string pinned_;
  std::string cached_ip_;
  std::string last_snapshot_;
  bool have_cache_{false};
  std::chrono::steady_clock::time_point cached_at_{};
  double ttl_s_{60.0};
};

}  // namespace data_agent

#endif  // DATA_AGENT__HEALTH_IP_HPP_
