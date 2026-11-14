// Copyright 2026 HUNTER Development Team
// health.ip 契约校验单测（contracts/kafka/schemas/health.schema.json）
//
// 覆盖"错了不报错、只是整条 health 进 DLQ"的三类高危路径：
//   * 合法 IPv4/IPv6（含压缩形式）必须原样/规范化通过；
//   * 契约明确拒绝的写法（端口、CIDR、zone、IPv4-mapped、三段、前导零）必须返回空；
//   * 配置 off/非法固定值时的降级语义：宁可不发 ip，也绝不发脏值。
#include <cstddef>
#include <string>

#include <gtest/gtest.h>

#include "data_agent/health_ip.hpp"

namespace
{

TEST(HealthIpContract, AcceptsDottedIpv4)
{
  EXPECT_EQ(data_agent::HealthIpReporter::normalize("192.168.10.21"), "192.168.10.21");
  EXPECT_EQ(data_agent::HealthIpReporter::normalize("10.0.0.1"), "10.0.0.1");
  // 契约 minLength 3：单个数字不可能是合法地址
  EXPECT_TRUE(data_agent::HealthIpReporter::isValid("1.1.1.1"));
}

TEST(HealthIpContract, RejectsMalformedIpv4)
{
  const char * const bad[] = {
    "192.168.10",              // 三段
    "192.168.10.256",          // 越界
    "192.168.10.21:1883",      // 带端口
    "192.168.10.0/24",         // CIDR
    "010.0.0.1",               // 前导零（schema 宽松但平台 ipaddress 拒绝）
    "192.168..21",             // 空段
    "192.168.10.21.",          // 尾部多余点
    "",                        // 空串
    "unknown",
    "N/A",
  };
  for (const char * value : bad) {
    EXPECT_TRUE(data_agent::HealthIpReporter::normalize(value).empty()) << value;
  }
}

TEST(HealthIpContract, AcceptsCompressedIpv6)
{
  EXPECT_EQ(
    data_agent::HealthIpReporter::normalize("2001:0db8:0000:0000:0000:0000:0000:0001"),
    "2001:db8::1");
  EXPECT_EQ(data_agent::HealthIpReporter::normalize("2001:db8::1"), "2001:db8::1");
  EXPECT_EQ(data_agent::HealthIpReporter::normalize("::1"), "::1");
}

TEST(HealthIpContract, RejectsMalformedIpv6)
{
  const char * const bad[] = {
    "fe80::1%eth0",              // zone id（pattern 不收 %）
    "::ffff:192.168.10.21",      // IPv4-mapped 文本
    "2001:db8:::1",              // 连续三冒号
    "2001:db8::1/64",            // CIDR
    "2001:db8::1:80",            // 合法形式但不带方括号的端口语义由上层拒绝
  };
  // 最后一项本身是合法 IPv6（"2001:db8::1:80"），不能误判；单独断言前四项。
  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_TRUE(data_agent::HealthIpReporter::normalize(bad[i]).empty()) << bad[i];
  }
  EXPECT_FALSE(data_agent::HealthIpReporter::normalize("2001:db8::1:80").empty());
}

TEST(HealthIpContract, ModeOffNeverReports)
{
  data_agent::HealthIpReporter reporter;
  EXPECT_TRUE(reporter.configure("off"));
  EXPECT_FALSE(reporter.enabled());
  EXPECT_EQ(reporter.current(), "");

  EXPECT_TRUE(reporter.configure("none"));   // 平台模拟器 --ip none 的语义
  EXPECT_EQ(reporter.current(), "");
}

TEST(HealthIpContract, PinnedValueIsNormalized)
{
  data_agent::HealthIpReporter reporter;
  EXPECT_TRUE(reporter.configure("192.168.10.21"));
  EXPECT_TRUE(reporter.enabled());
  EXPECT_EQ(reporter.mode(), data_agent::HealthIpReporter::Mode::kPinned);
  EXPECT_EQ(reporter.current(), "192.168.10.21");

  EXPECT_TRUE(reporter.configure("2001:0db8:0000:0000:0000:0000:0000:0001"));
  EXPECT_EQ(reporter.current(), "2001:db8::1");
}

TEST(HealthIpContract, InvalidPinnedValueDegradesToOff)
{
  data_agent::HealthIpReporter reporter;
  // 非法固定值必须降级为"省略 ip 键"，不能"尽力发送"
  EXPECT_FALSE(reporter.configure("10.0.0.3/24"));
  EXPECT_FALSE(reporter.enabled());
  EXPECT_EQ(reporter.current(), "");
}

TEST(HealthIpContract, ResolveModePrecedence)
{
  // 参数 > 环境变量 > auto（环境变量在此用不存在的名字验证 auto 兜底）
  EXPECT_EQ(data_agent::HealthIpReporter::resolveMode("off", "HUNTER_TEST_UNSET_ENV"), "off");
  EXPECT_EQ(data_agent::HealthIpReporter::resolveMode("", "HUNTER_TEST_UNSET_ENV"), "auto");
  EXPECT_EQ(data_agent::HealthIpReporter::resolveMode("  ", "HUNTER_TEST_UNSET_ENV"), "auto");
}

// 多网卡/无路由/仅 IPv6 场景的地址准入规则（探测本身依赖运行时网络状态，
// 这里把「哪些地址/接口可以上报」这部分确定性逻辑钉死）。
TEST(HealthIpContract, RejectsUnusableAddresses)
{
  using data_agent::HealthIpReporter;
  // loopback / link-local / fe80::/10 / 未指定
  EXPECT_FALSE(HealthIpReporter::acceptableAddress("127.0.0.1", "lo"));
  EXPECT_FALSE(HealthIpReporter::acceptableAddress("::1", "lo"));
  EXPECT_FALSE(HealthIpReporter::acceptableAddress("169.254.3.4", "eth0"));
  EXPECT_FALSE(HealthIpReporter::acceptableAddress("fe80::1", "eth0"));
  EXPECT_FALSE(HealthIpReporter::acceptableAddress("0.0.0.0", "eth0"));
  EXPECT_FALSE(HealthIpReporter::acceptableAddress("::", "eth0"));
  EXPECT_FALSE(HealthIpReporter::acceptableAddress("", "eth0"));
}

TEST(HealthIpContract, ExcludesVirtualInterfaces)
{
  using data_agent::HealthIpReporter;
  // 容器网桥 / 虚拟网卡 / VPN 隧道：地址本身合法，仍不得上报
  for (const char * if_name : {"docker0", "br-1a2b3c", "veth9f", "virbr0",
      "tun0", "tap0", "wg0", "zt0", "tailscale0"})
  {
    EXPECT_FALSE(HealthIpReporter::acceptableAddress("10.0.0.2", if_name)) << if_name;
  }
  // 真实出口网卡（4G/以太/无线）放行；接口名未知（空）时按地址前缀规则放行
  EXPECT_TRUE(HealthIpReporter::acceptableAddress("10.0.0.2", "eth0"));
  EXPECT_TRUE(HealthIpReporter::acceptableAddress("192.168.10.21", "wwan0"));
  EXPECT_TRUE(HealthIpReporter::acceptableAddress("192.168.10.21", "wlan0"));
  EXPECT_TRUE(HealthIpReporter::acceptableAddress("2001:db8::1", "wwan0"));
  EXPECT_TRUE(HealthIpReporter::acceptableAddress("192.168.10.21", ""));
}

}  // namespace
