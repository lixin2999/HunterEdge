// Copyright 2026 HUNTER Development Team
// HunterCore 车端接入包（kafka.properties）解析与 librdkafka 配置装配 —— C++ 侧等价实现
//
// 与 Python 公共包 hunter_kafka/config.py 保持同一套语义（同一份接入包必须被
// C++ 节点与 Python Agent 一致地解析），要点：
//   1. 单一可信源：Broker / SCRAM 凭据 / 证书路径只来自 /etc/hunter/kafka/kafka.properties
//   2. Java 键 → librdkafka 键映射（sasl.jaas.config、ssl.truststore.* 不能透传）
//   3. 行内注释剥离（java.util.Properties 会把 " # 注释" 当值的一部分）
//   4. mTLS：PEM(client-cert/key) 优先，回落 kafka-client.p12，都没有则失败
#ifndef DATA_AGENT__KAFKA_ACCESS_HPP_
#define DATA_AGENT__KAFKA_ACCESS_HPP_

#include <map>
#include <string>

namespace RdKafka
{
class Conf;
}  // namespace RdKafka

namespace data_agent
{
namespace kafka_access
{

// 接入包默认部署位置（Deployment_Guide §5.6）
extern const char * const DEFAULT_PROPERTIES_PATH;
extern const char * const DEFAULT_BUNDLE_DIR;
extern const char * const CA_FILE;
extern const char * const CLIENT_CERT_FILE;
extern const char * const CLIENT_KEY_FILE;
extern const char * const CLIENT_P12_FILE;

using StringMap = std::map<std::string, std::string>;

struct ClientOptions
{
  std::string properties_path;   ///< 空 → DEFAULT_PROPERTIES_PATH
  std::string bundle_dir;        ///< 空 → properties 同目录
  std::string client_id;         ///< client.id（平台侧连接审计定位用）
  bool is_producer = true;
  std::string acks;              ///< 空 → 取 properties（契约要求 event 类 acks=all）
  StringMap extra;               ///< 追加/覆盖任意 librdkafka 键
};

/// 读取 Java Properties 文件（# / ! 注释、\ 续行、\: \= 转义、行内尾注释剥离）。
/// 文件不存在返回 false 并在 err 中给出部署提示。
bool loadProperties(const std::string & path, StringMap & out, std::string & err);

/// 把 Java 风格 properties 装配成 librdkafka 键值对。
/// 缺 bootstrap / SASL 凭据 / 客户端证书时返回 false（fail-fast，不静默降级）。
bool buildConf(
  const StringMap & props,
  const ClientOptions & options,
  StringMap & conf,
  std::string & err);

/// 便捷入口：loadProperties + buildConf。
bool loadClientConf(const ClientOptions & options, StringMap & conf, std::string & err);

/// 逐项写入 RdKafka::Conf；任一键被 librdkafka 拒绝即返回 false 并带上原始 errstr。
bool applyToGlobalConf(
  RdKafka::Conf * conf, const StringMap & kvs, std::string & err);

/// vehicle_id：SASL 用户名（接入包中 username == 证书 CN == vehicle_id）。
std::string resolveVehicleId(const StringMap & props);

/// 日志用脱敏副本（口令打星，只保留长度）。
std::string redacted(const StringMap & conf);

}  // namespace kafka_access
}  // namespace data_agent

#endif  // DATA_AGENT__KAFKA_ACCESS_HPP_
