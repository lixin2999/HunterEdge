// Copyright 2026 HUNTER Development Team
// health 心跳 `ip` 字段探测与校验实现（详见 include/data_agent/health_ip.hpp）
#include "data_agent/health_ip.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <vector>

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace data_agent
{

namespace
{

std::string trim(const std::string & raw)
{
  const auto is_space = [](unsigned char c) {return std::isspace(c) != 0;};
  std::size_t begin = 0;
  std::size_t end = raw.size();
  while (begin < end && is_space(static_cast<unsigned char>(raw[begin]))) {
    ++begin;
  }
  while (end > begin && is_space(static_cast<unsigned char>(raw[end - 1]))) {
    --end;
  }
  return raw.substr(begin, end - begin);
}

std::string toLower(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return value;
}

bool startsWith(const std::string & value, const char * prefix)
{
  const std::size_t n = std::strlen(prefix);
  return value.size() >= n && value.compare(0, n, prefix) == 0;
}

// IPv4：4 段十进制、无前导零、每段 0-255。
// 比契约 pattern 更严一点（pattern 允许 "010"），对齐平台侧 ipaddress 严格校验，
// 避免"schema 能过、平台 ipaddress 拒绝"的灰区。
bool isIpv4Literal(const std::string & value)
{
  std::size_t pos = 0;
  int parts = 0;
  while (true) {
    const std::size_t start = pos;
    while (pos < value.size() && value[pos] != '.') {
      if (std::isdigit(static_cast<unsigned char>(value[pos])) == 0) {
        return false;
      }
      ++pos;
    }
    const std::size_t len = pos - start;
    if (len == 0 || len > 3) {
      return false;
    }
    if (len > 1 && value[start] == '0') {
      return false;
    }
    int octet = 0;
    for (std::size_t i = start; i < pos; ++i) {
      octet = octet * 10 + (value[i] - '0');
    }
    if (octet > 255) {
      return false;
    }
    ++parts;
    if (pos >= value.size()) {
      break;
    }
    ++pos;  // 跳过 '.'
    if (pos >= value.size()) {
      return false;  // 尾部多余的点
    }
  }
  return parts == 4;
}

// IPv6：交给 inet_pton 做语法判定，另行排除契约 pattern 不接受的写法：
// zone id（%eth0）、IPv4 内嵌/映射（::ffff:192.168.10.21）一律拒绝。
bool isIpv6Literal(const std::string & value)
{
  if (value.find(':') == std::string::npos) {
    return false;
  }
  if (value.find('.') != std::string::npos || value.find('%') != std::string::npos) {
    return false;
  }
  unsigned char buf[16]{};
  return inet_pton(AF_INET6, value.c_str(), buf) == 1;
}

std::string interfaceNameOfSockaddr(const sockaddr * target)
{
  if (target == nullptr) {
    return "";
  }
  ifaddrs * head = nullptr;
  if (getifaddrs(&head) != 0) {
    return "";
  }
  std::string name;
  const std::size_t len =
    target->sa_family == AF_INET ? sizeof(sockaddr_in) : sizeof(sockaddr_in6);
  for (ifaddrs * it = head; it != nullptr; it = it->ifa_next) {
    if (it->ifa_addr == nullptr || it->ifa_addr->sa_family != target->sa_family) {
      continue;
    }
    if (std::memcmp(it->ifa_addr, target, len) == 0) {
      name = it->ifa_name != nullptr ? it->ifa_name : "";
      break;
    }
  }
  freeifaddrs(head);
  return name;
}

bool isExcludedInterface(const std::string & if_name)
{
  const std::string name = toLower(if_name);
  if (name.empty()) {
    return false;  // 无法判定接口时宁可上报，交由地址前缀过滤兜底
  }
  // 误报来源：容器网桥 / 虚拟网卡 / VPN 隧道（默认路由不会走它们，
  // 但 getifaddrs 兜底与固定值场景仍需显式排除）
  static const char * const kExcluded[] = {
    "lo", "docker", "br-", "veth", "virbr", "tun", "tap", "wg", "zt", "tailscale",
  };
  for (const char * prefix : kExcluded) {
    if (startsWith(name, prefix)) {
      return true;
    }
  }
  return false;
}

// 默认路由指纹（IPv4 /proc/net/route + IPv6 /proc/net/ipv6_route）。
// 「接口地址集合」变化覆盖不到"两条默认路由都在、只是度量切换"的场景，
// 这里补上默认路由的出口接口名：默认路由一变，下一拍（≤1s）即重采样。
std::string defaultRouteSnapshot()
{
  std::string out;
  {
    std::ifstream file("/proc/net/route");
    std::string line;
    std::getline(file, line);   // 表头
    while (std::getline(file, line)) {
      std::istringstream row(line);
      std::string iface;
      std::string dest;
      std::string gateway;
      std::string flags;
      if (!(row >> iface >> dest >> gateway >> flags)) {
        continue;
      }
      if (dest == "00000000") {   // 默认路由（0.0.0.0/0）
        out += "v4:" + iface + ";";
      }
    }
  }
  {
    std::ifstream file("/proc/net/ipv6_route");
    std::string line;
    while (std::getline(file, line)) {
      std::istringstream row(line);
      std::string dest;
      std::string prefix_len;
      std::string src;
      std::string src_prefix_len;
      std::string next_hop;
      std::string metric;
      std::string refcnt;
      std::string use;
      std::string flags;
      std::string iface;
      if (!(row >> dest >> prefix_len >> src >> src_prefix_len >> next_hop >>
        metric >> refcnt >> use >> flags >> iface))
      {
        continue;
      }
      // ::/0：目的地址 32 个 0 + 前缀长度 00
      if (prefix_len == "00" && dest == std::string(32, '0')) {
        out += "v6:" + iface + ";";
      }
    }
  }
  return out;
}

}  // namespace

std::string HealthIpReporter::resolveMode(const std::string & param_value, const char * env_name)
{
  const std::string from_param = trim(param_value);
  if (!from_param.empty()) {
    return from_param;
  }
  const char * env = env_name != nullptr ? std::getenv(env_name) : nullptr;
  if (env != nullptr) {
    const std::string from_env = trim(env);
    if (!from_env.empty()) {
      return from_env;
    }
  }
  return "auto";
}

bool HealthIpReporter::configure(const std::string & raw_mode, double ttl_s)
{
  mode_ = Mode::kAuto;
  pinned_.clear();
  cached_ip_.clear();
  last_snapshot_.clear();
  have_cache_ = false;
  ttl_s_ = ttl_s > 0.0 ? ttl_s : 60.0;

  const std::string raw = trim(raw_mode);
  const std::string lowered = toLower(raw);
  if (lowered.empty() || lowered == "auto" || lowered == "on" || lowered == "true") {
    return true;  // auto（默认）
  }
  if (lowered == "off" || lowered == "none" || lowered == "false" || lowered == "0") {
    mode_ = Mode::kOff;  // 与旧版报文逐字节等价，零风险回滚开关
    return true;
  }
  const std::string normalized = normalize(raw);
  if (normalized.empty()) {
    // 固定值非法：降级为 off（省略 ip 键）并让调用方记 ERROR；
    // 绝不"尽力发送"——一个脏值会毁掉整条 health。
    mode_ = Mode::kOff;
    return false;
  }
  mode_ = Mode::kPinned;
  pinned_ = normalized;
  return true;
}

bool HealthIpReporter::isValid(const std::string & value)
{
  return !normalize(value).empty();
}

std::string HealthIpReporter::normalize(const std::string & value)
{
  const std::string raw = trim(value);
  if (raw.size() < 3 || raw.size() > 45) {   // 契约 minLength 3 / maxLength 45
    return "";
  }
  if (isIpv4Literal(raw)) {
    return raw;
  }
  if (isIpv6Literal(raw)) {
    unsigned char buf[16]{};
    if (inet_pton(AF_INET6, raw.c_str(), buf) != 1) {
      return "";
    }
    char out[INET6_ADDRSTRLEN]{};
    if (inet_ntop(AF_INET6, buf, out, sizeof out) == nullptr) {
      return "";
    }
    return std::string(out);   // 压缩小写形式（契约允许，平台 ipaddress 亦可解析）
  }
  return "";
}

void HealthIpReporter::invalidate()
{
  have_cache_ = false;
}

bool HealthIpReporter::acceptableAddress(const std::string & ip, const std::string & if_name)
{
  if (ip.empty() || ip == "0.0.0.0" || ip == "::") {
    return false;
  }
  if (startsWith(ip, "127.") || ip == "::1") {         // loopback
    return false;
  }
  if (startsWith(ip, "169.254.")) {                    // IPv4 link-local
    return false;
  }
  const std::string lowered = toLower(ip);
  if (startsWith(lowered, "fe8") || startsWith(lowered, "fe9") ||
    startsWith(lowered, "fea") || startsWith(lowered, "feb"))
  {
    return false;                                      // IPv6 fe80::/10
  }
  return !isExcludedInterface(if_name);
}

std::string HealthIpReporter::interfaceOf(const std::string & ip)
{
  in_addr v4{};
  in6_addr v6{};
  if (inet_pton(AF_INET, ip.c_str(), &v4) == 1) {
    sockaddr_in sa{};
    sa.sin_family = AF_INET;
    sa.sin_addr = v4;
    return interfaceNameOfSockaddr(reinterpret_cast<const sockaddr *>(&sa));
  }
  if (inet_pton(AF_INET6, ip.c_str(), &v6) == 1) {
    sockaddr_in6 sa{};
    sa.sin6_family = AF_INET6;
    sa.sin6_addr = v6;
    return interfaceNameOfSockaddr(reinterpret_cast<const sockaddr *>(&sa));
  }
  return "";
}

bool HealthIpReporter::detect(std::string & out)
{
  out.clear();
  struct Probe
  {
    int family;
    const char * host;
  };
  // 优先 IPv4；仅当无 IPv4 时才报 IPv6（一辆车只报一个地址）
  const Probe probes[] = {
    {AF_INET, "8.8.8.8"},
    {AF_INET6, "2606:4700:4700::1111"},
  };

  for (const auto & probe : probes) {
    const int fd = ::socket(probe.family, SOCK_DGRAM, 0);
    if (fd < 0) {
      continue;
    }
    sockaddr_storage target{};
    socklen_t target_len = 0;
    if (probe.family == AF_INET) {
      auto * addr = reinterpret_cast<sockaddr_in *>(&target);
      addr->sin_family = AF_INET;
      addr->sin_port = htons(80);
      if (inet_pton(AF_INET, probe.host, &addr->sin_addr) != 1) {
        ::close(fd);
        continue;
      }
      target_len = sizeof(sockaddr_in);
    } else {
      auto * addr = reinterpret_cast<sockaddr_in6 *>(&target);
      addr->sin6_family = AF_INET6;
      addr->sin6_port = htons(80);
      if (inet_pton(AF_INET6, probe.host, &addr->sin6_addr) != 1) {
        ::close(fd);
        continue;
      }
      target_len = sizeof(sockaddr_in6);
    }
    // UDP connect 不发包、不做 DNS：只让内核做一次路由查找并选定源地址
    if (::connect(fd, reinterpret_cast<sockaddr *>(&target), target_len) != 0) {
      ::close(fd);
      continue;
    }
    sockaddr_storage local{};
    socklen_t local_len = sizeof(local);
    const bool got = ::getsockname(
      fd, reinterpret_cast<sockaddr *>(&local), &local_len) == 0;
    ::close(fd);
    if (!got) {
      continue;
    }

    char buf[INET6_ADDRSTRLEN]{};
    const void * src = nullptr;
    if (local.ss_family == AF_INET) {
      src = &reinterpret_cast<sockaddr_in *>(&local)->sin_addr;
    } else if (local.ss_family == AF_INET6) {
      src = &reinterpret_cast<sockaddr_in6 *>(&local)->sin6_addr;
    } else {
      continue;
    }
    if (inet_ntop(local.ss_family, src, buf, sizeof buf) == nullptr) {
      continue;
    }
    const std::string ip = buf;
    if (!acceptableAddress(ip, interfaceOf(ip))) {
      continue;
    }
    const std::string normalized = normalize(ip);
    if (!normalized.empty()) {
      out = normalized;
      return true;
    }
  }
  return false;
}

std::string HealthIpReporter::interfaceSnapshot()
{
  ifaddrs * head = nullptr;
  if (getifaddrs(&head) != 0) {
    return "";
  }
  std::vector<std::string> items;
  for (ifaddrs * it = head; it != nullptr; it = it->ifa_next) {
    if (it->ifa_addr == nullptr || it->ifa_name == nullptr) {
      continue;
    }
    if ((it->ifa_flags & IFF_UP) == 0 || (it->ifa_flags & IFF_LOOPBACK) != 0) {
      continue;
    }
    if (it->ifa_addr->sa_family != AF_INET && it->ifa_addr->sa_family != AF_INET6) {
      continue;
    }
    const void * src = it->ifa_addr->sa_family == AF_INET ?
      static_cast<const void *>(&reinterpret_cast<sockaddr_in *>(it->ifa_addr)->sin_addr) :
      static_cast<const void *>(&reinterpret_cast<sockaddr_in6 *>(it->ifa_addr)->sin6_addr);
    char buf[INET6_ADDRSTRLEN]{};
    if (inet_ntop(it->ifa_addr->sa_family, src, buf, sizeof buf) == nullptr) {
      continue;
    }
    items.push_back(std::string(it->ifa_name) + "=" + buf);
  }
  freeifaddrs(head);
  std::sort(items.begin(), items.end());
  std::string snapshot;
  for (const auto & item : items) {
    snapshot += item;
    snapshot += ';';
  }
  snapshot += '#';                       // 分隔：下面是默认路由指纹
  snapshot += defaultRouteSnapshot();
  return snapshot;
}

std::string HealthIpReporter::current()
{
  if (mode_ == Mode::kOff) {
    return "";
  }
  if (mode_ == Mode::kPinned) {
    return pinned_;
  }

  const auto now = std::chrono::steady_clock::now();
  // 网络事件触发：接口地址集合变化（link up/down、DHCP 续租、WiFi↔4G 切换）或
  // 默认路由出口变化 → 立刻丢缓存，下一拍（≤1s）即可上报新地址。
  const std::string snapshot = interfaceSnapshot();
  if (snapshot != last_snapshot_) {
    last_snapshot_ = snapshot;
    have_cache_ = false;
  }

  // 失败后 10s 重试（而不是等满 60s），避免"开机时网络未就绪"导致心跳长时间缺 ip
  const double retry_s = cached_ip_.empty() ? std::min(ttl_s_, 10.0) : ttl_s_;
  const double age_s = std::chrono::duration<double>(now - cached_at_).count();
  if (!have_cache_ || age_s >= retry_s) {
    std::string detected;
    const bool ok = detect(detected);
    cached_ip_ = ok ? detected : std::string();
    have_cache_ = true;
    cached_at_ = now;
  }
  return cached_ip_;
}

}  // namespace data_agent
