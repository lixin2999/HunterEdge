// Copyright 2026 HUNTER Development Team
// HunterCore 接入包解析与 librdkafka 配置装配实现（与 hunter_kafka/config.py 同语义）
#include "data_agent/kafka_access.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <vector>

#include <librdkafka/rdkafkacpp.h>

namespace data_agent
{
namespace kafka_access
{

const char * const DEFAULT_PROPERTIES_PATH = "/etc/hunter/kafka/kafka.properties";
const char * const DEFAULT_BUNDLE_DIR = "/etc/hunter/kafka";
const char * const CA_FILE = "ca-cert.pem";
const char * const CLIENT_CERT_FILE = "client-cert.pem";
const char * const CLIENT_KEY_FILE = "client-key.pem";
const char * const CLIENT_P12_FILE = "kafka-client.p12";

namespace
{

std::string trim(const std::string & s)
{
  const auto begin = s.find_first_not_of(" \t\r\n\f\v");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = s.find_last_not_of(" \t\r\n\f\v");
  return s.substr(begin, end - begin + 1);
}

std::string toLower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c)); });
  return s;
}

std::string unescape(const std::string & s)
{
  std::string out;
  out.reserve(s.size());
  for (size_t i = 0; i < s.size(); ++i) {
    if (s[i] == '\\' && i + 1 < s.size()) {
      switch (s[i + 1]) {
        case '=': out += '='; ++i; continue;
        case ':': out += ':'; ++i; continue;
        case 'n': out += '\n'; ++i; continue;
        case 't': out += '\t'; ++i; continue;
        case '\\': out += '\\'; ++i; continue;
        default: break;
      }
    }
    out += s[i];
  }
  return out;
}

// 行尾注释剥离：仅当 # 或 ; 位于串首或其前有空白时才当注释起始，
// 否则会把含 '#' 的 SCRAM 口令截断（Python 侧 _strip_inline_comment 同规则）
std::string stripInlineComment(const std::string & value)
{
  for (size_t i = 0; i < value.size(); ++i) {
    if ((value[i] == '#' || value[i] == ';') &&
      (i == 0 || value[i - 1] == ' ' || value[i - 1] == '\t'))
    {
      return trim(value.substr(0, i));
    }
  }
  return trim(value);
}

// 以奇数个反斜杠结尾 → Properties 续行
bool endsWithContinuation(const std::string & line)
{
  size_t n = 0;
  for (auto it = line.rbegin(); it != line.rend() && *it == '\\'; ++it) {
    ++n;
  }
  return n % 2 == 1;
}

bool isPlaceholderSecret(const std::string & raw)
{
  const std::string v = trim(raw);
  if (v.empty()) {
    return true;
  }
  if (v.front() == '<' && v.back() == '>') {
    return true;
  }
  if (v.size() > 4 && v.front() == '{' && v.back() == '}') {
    return true;
  }
  const std::string lower = toLower(v);
  return lower == "change_me" || lower == "changeme" || lower == "your_password" ||
         lower == "yourpassword" || lower == "password" || lower == "none";
}

// 去掉引号并识别占位口令；非法时返回空串
std::string normalizeSecret(const std::string & raw)
{
  std::string v = trim(raw);
  if (v.size() >= 2 && (v.front() == '"' || v.front() == '\'') && v.back() == v.front()) {
    v = trim(v.substr(1, v.size() - 2));
  }
  return isPlaceholderSecret(v) ? "" : v;
}

std::string envOrEmpty(const char * name)
{
  const char * v = std::getenv(name);
  return v ? std::string(v) : std::string();
}

std::string extractJaasField(const std::string & jaas, const std::string & field)
{
  const auto pos = jaas.find(field);
  if (pos == std::string::npos) {
    return "";
  }
  const auto eq = jaas.find('=', pos + field.size());
  if (eq == std::string::npos) {
    return "";
  }
  const auto quote = jaas.find('"', eq + 1);
  if (quote == std::string::npos) {
    return "";
  }
  const auto close = jaas.find('"', quote + 1);
  if (close == std::string::npos) {
    return "";
  }
  return jaas.substr(quote + 1, close - quote - 1);
}

std::string existingFile(const std::string & dir, const char * name)
{
  const std::filesystem::path p = std::filesystem::path(dir) / name;
  std::error_code ec;
  return std::filesystem::is_regular_file(p, ec) ? p.string() : std::string();
}

}  // namespace

bool loadProperties(const std::string & path, StringMap & out, std::string & err)
{
  std::error_code ec;
  if (!std::filesystem::is_regular_file(path, ec)) {
    err = "未找到 Kafka 接入参数文件：" + path +
          "（请先把 HunterCore 接入包部署到 /etc/hunter/kafka/，见 Deployment_Guide §5.6）";
    return false;
  }
  std::ifstream fin(path);
  if (!fin) {
    err = "Kafka 接入参数文件不可读：" + path +
          "（路径是否传错、运行用户是否在 /etc/hunter/kafka 的属组内；文件应为 0640、目录 0750）";
    return false;
  }

  // 1) 续行合并（jaas 配置跨行）
  std::vector<std::string> logical;
  std::string buf;
  bool inContinuation = false;
  std::string line;
  while (std::getline(fin, line)) {
    const std::string stripped = trim(line);
    if (!inContinuation) {
      if (stripped.empty() || stripped.front() == '#' || stripped.front() == '!') {
        continue;
      }
      buf = line;
      inContinuation = true;
    } else {
      buf += " " + stripped;
    }
    if (endsWithContinuation(buf)) {
      buf.pop_back();
      continue;
    }
    logical.push_back(buf);
    buf.clear();
    inContinuation = false;
  }
  if (inContinuation && !buf.empty()) {
    logical.push_back(buf);
  }

  // 2) 键值拆分（首个未转义的 = 或 : 作分隔符）
  for (const auto & raw : logical) {
    const std::string entry = trim(raw);
    if (entry.empty() || entry.front() == '#' || entry.front() == '!') {
      continue;
    }
    size_t sep = std::string::npos;
    for (size_t i = 0; i < entry.size(); ++i) {
      if (entry[i] == '\\') {
        ++i;
        continue;
      }
      if (entry[i] == '=' || entry[i] == ':') {
        sep = i;
        break;
      }
    }
    if (sep == std::string::npos) {
      continue;  // 无分隔符的行（纯 flag）忽略
    }
    const std::string key = trim(unescape(entry.substr(0, sep)));
    const std::string value = stripInlineComment(unescape(entry.substr(sep + 1)));
    if (!key.empty()) {
      out[key] = value;
    }
  }
  return true;
}

std::string resolveVehicleId(const StringMap & props)
{
  auto it = props.find("sasl.username");
  if (it != props.end() && !it->second.empty()) {
    return it->second;
  }
  const auto jaas = props.find("sasl.jaas.config");
  if (jaas != props.end()) {
    return extractJaasField(jaas->second, "username");
  }
  return "";
}

bool buildConf(
  const StringMap & props,
  const ClientOptions & options,
  StringMap & conf,
  std::string & err)
{
  auto get = [&props](const char * key) -> std::string {
      const auto it = props.find(key);
      return it == props.end() ? std::string() : it->second;
    };

  const std::string bootstrap = get("bootstrap.servers");
  if (bootstrap.empty()) {
    err = "kafka.properties 缺少 bootstrap.servers";
    return false;
  }
  conf["bootstrap.servers"] = bootstrap;

  // ── 安全协议与 SASL(SCRAM) 凭据 ──
  const std::string protocol = toLower(get("security.protocol"));
  if (!protocol.empty()) {
    conf["security.protocol"] = protocol;
  }
  if (!get("sasl.mechanism").empty()) {
    conf["sasl.mechanism"] = get("sasl.mechanism");
  }

  std::string username = normalizeSecret(get("sasl.username"));
  std::string password = normalizeSecret(get("sasl.password"));
  const std::string jaas = get("sasl.jaas.config");
  if (username.empty() || password.empty()) {
    if (username.empty()) {
      username = normalizeSecret(extractJaasField(jaas, "username"));
    }
    if (password.empty()) {
      password = normalizeSecret(extractJaasField(jaas, "password"));
    }
  }
  username = envOrEmpty("HUNTER_KAFKA_SASL_USERNAME").empty() ?
    username : envOrEmpty("HUNTER_KAFKA_SASL_USERNAME");
  password = envOrEmpty("HUNTER_KAFKA_SASL_PASSWORD").empty() ?
    password : envOrEmpty("HUNTER_KAFKA_SASL_PASSWORD");

  if (protocol.rfind("sasl", 0) == 0) {
    if (username.empty() || password.empty()) {
      err = "SASL 凭据缺失：请在 kafka.properties 的 sasl.jaas.config 中把 "
            "password=\"<SCRAM_PASSWORD>\" 替换为运营一次性下发的 SCRAM 口令"
            "（或设置环境变量 HUNTER_KAFKA_SASL_PASSWORD）";
      return false;
    }
    conf["sasl.username"] = username;
    conf["sasl.password"] = password;
  }

  // ── TLS / mTLS ──
  if (protocol.find("ssl") != std::string::npos) {
    const std::string bundle = options.bundle_dir.empty() ? DEFAULT_BUNDLE_DIR : options.bundle_dir;
    // 信任链：显式指定 → bundle 的 ca-cert.pem → 不写（librdkafka 探针系统信任链）
    // JKS truststore 无法被 librdkafka 读取，禁止透传 ssl.truststore.location
    const std::string ca = get("ssl.ca.location").empty() ?
      existingFile(bundle, CA_FILE) : get("ssl.ca.location");
    if (!ca.empty()) {
      conf["ssl.ca.location"] = ca;
    }

    const std::string cert = get("ssl.certificate.location").empty() ?
      existingFile(bundle, CLIENT_CERT_FILE) : get("ssl.certificate.location");
    const std::string key = get("ssl.key.location").empty() ?
      existingFile(bundle, CLIENT_KEY_FILE) : get("ssl.key.location");
    const std::string p12 = get("ssl.keystore.location").empty() ?
      existingFile(bundle, CLIENT_P12_FILE) : get("ssl.keystore.location");

    if (!cert.empty() && !key.empty()) {
      conf["ssl.certificate.location"] = cert;
      conf["ssl.key.location"] = key;
      const std::string keyPwd = normalizeSecret(get("ssl.key.password"));
      if (!keyPwd.empty()) {
        conf["ssl.key.password"] = keyPwd;
      }
    } else if (!p12.empty()) {
      // librdkafka 原生支持 PKCS#12
      conf["ssl.keystore.location"] = p12;
      std::string ksPwd = normalizeSecret(get("ssl.keystore.password"));
      if (ksPwd.empty()) {
        ksPwd = envOrEmpty("HUNTER_KAFKA_P12_PASSWORD");
      }
      if (ksPwd.empty()) {
        err = p12 + " 需要 ssl.keystore.password（接入包 README 中的一次性 PKCS12 口令），"
              "请写入 kafka.properties 或设置环境变量 HUNTER_KAFKA_P12_PASSWORD";
        return false;
      }
      conf["ssl.keystore.password"] = ksPwd;
    } else {
      err = "未找到客户端证书：" + bundle + "/{client-cert.pem,client-key.pem} 或 "
            "kafka-client.p12（SASL_SSL 需 mTLS，缺证书 broker 会在握手期拒绝连接）";
      return false;
    }

    // 主机名校验：Java 的 https ↔ librdkafka 同名枚举（none/https）
    const std::string algo = toLower(trim(get("ssl.endpoint.identification.algorithm")));
    if (!algo.empty()) {
      if (algo == "https" || algo == "dns" || algo == "default") {
        conf["ssl.endpoint.identification.algorithm"] = "https";
      } else if (algo == "none") {
        conf["ssl.endpoint.identification.algorithm"] = "none";
        conf["enable.ssl.certificate.verification"] = "false";
      } else {
        conf["ssl.endpoint.identification.algorithm"] = algo;
      }
    }
  }

  // ── 生产者/消费者基准参数（键名两端一致，白名单透传）──
  const char * const commonKeys[] = {
    "linger.ms", "batch.size", "retries", "compression.type",
    "message.timeout.ms", "request.timeout.ms",
    "reconnect.backoff.ms", "reconnect.backoff.max.ms", "socket.keepalive.enable"};
  for (const char * k : commonKeys) {
    if (!get(k).empty()) {
      conf[k] = get(k);
    }
  }
  if (!options.is_producer) {
    const char * const consumerKeys[] = {
      "enable.auto.commit", "auto.offset.reset", "isolation.level",
      "session.timeout.ms", "max.poll.interval.ms"};
    for (const char * k : consumerKeys) {
      if (!get(k).empty()) {
        conf[k] = get(k);
      }
    }
  }

  const std::string acks = options.acks.empty() ? get("acks") : options.acks;
  if (!acks.empty()) {
    conf["acks"] = acks;
  }
  if (!options.client_id.empty()) {
    conf["client.id"] = options.client_id;
  }
  for (const auto & kv : options.extra) {
    conf[kv.first] = kv.second;      // extra 最后覆盖（group.id / auto.offset.reset 等）
  }
  return true;
}

bool loadClientConf(const ClientOptions & options, StringMap & conf, std::string & err)
{
  const std::string path = options.properties_path.empty() ?
    DEFAULT_PROPERTIES_PATH : options.properties_path;
  StringMap props;
  if (!loadProperties(path, props, err)) {
    return false;
  }
  const std::string bootstrapEnv = envOrEmpty("HUNTER_KAFKA_BOOTSTRAP_SERVERS");
  if (!bootstrapEnv.empty()) {
    props["bootstrap.servers"] = bootstrapEnv;   // 多环境联调可临时覆盖 broker
  }
  ClientOptions effective = options;
  if (effective.bundle_dir.empty()) {
    const std::filesystem::path parent = std::filesystem::path(path).parent_path();
    effective.bundle_dir = parent.empty() ? DEFAULT_BUNDLE_DIR : parent.string();
  }
  return buildConf(props, effective, conf, err);
}

bool applyToGlobalConf(RdKafka::Conf * conf, const StringMap & kvs, std::string & err)
{
  if (!conf) {
    err = "RdKafka::Conf 为空";
    return false;
  }
  for (const auto & kv : kvs) {
    std::string errstr;
    if (conf->set(kv.first, kv.second, errstr) != RdKafka::Conf::CONF_OK) {
      std::ostringstream oss;
      oss << "Kafka 配置项 " << kv.first << " 被 librdkafka 拒绝：" << errstr;
      err = oss.str();
      return false;
    }
  }
  return true;
}

std::string redacted(const StringMap & conf)
{
  // 日志/自检输出用的脱敏副本：口令打星，只保留长度提示
  static const StringMap secrets = {
    {"sasl.password", ""}, {"ssl.keystore.password", ""}, {"ssl.key.password", ""}};
  std::ostringstream oss;
  bool first = true;
  for (const auto & kv : conf) {
    if (!first) {
      oss << ", ";
    }
    first = false;
    if (secrets.count(kv.first)) {
      oss << kv.first << "=***(" << kv.second.size() << ")";
    } else {
      oss << kv.first << "=" << kv.second;
    }
  }
  return oss.str();
}

}  // namespace kafka_access
}  // namespace data_agent
