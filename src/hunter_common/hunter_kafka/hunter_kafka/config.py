#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""kafka.properties 解析与 librdkafka 配置装配（HunterCore 车端接入包）。

设计要点（对应 HunterCore 接入包 README/kafka.properties 契约）：
1. **单一可信源**：Broker 地址、SASL 凭据、证书路径全部来自车端
   ``/etc/hunter/kafka/kafka.properties``（目录 0750 / 文件 0640 / 私钥 0600，
   由运营一次性写入 SCRAM 口令），
   各 Agent 的 YAML 只保留 ``kafka_properties`` 路径与业务参数，
   **严禁把口令写进仓库内的 YAML**（接入包 README「安全注意」）。
2. **Java → librdkafka 映射**：接入包是 Java 生态语法（``sasl.jaas.config``、
   ``ssl.truststore.location``）。车端用的是 librdkafka（confluent-kafka / C++），
   二者键名/能力不完全一致，本模块负责翻译：
     - ``sasl.jaas.config``  → ``sasl.username`` / ``sasl.password``（librdkafka 无 jaas）
     - ``ssl.keystore.location``(.p12) → librdkafka 原生支持 PKCS#12，直接透传
     - ``ssl.truststore.location``(JKS) → 不支持，回落 ``ca-cert.pem`` → ``ssl.ca.location``
     - 其余（``linger.ms``/``batch.size``/``retries``/``compression.type``/``acks``/
       ``enable.auto.commit``/``auto.offset.reset``/``isolation.level``）键名两端一致，透传。
3. **模板行内注释必须剥离**：接入包 ``acks=1   # telemetry 高频；...`` 按
   java.util.Properties 语义会把 ``#`` 之后的内容当作值的一部分，
   librdkafka 会因非法值直接拒绝配置 → 这里做有规则的尾注释剥离
   （仅当 ``#``/``;`` 前有空白才剥离，避免误伤含 ``#`` 的口令）。
4. **mTLS 凭据自动定位**：SASL_SSL 之上平台还要求客户端证书（CN=vehicle_id）。
   优先 bundle 内 PEM（``client-cert.pem`` + ``client-key.pem``），
   缺失时回落 ``kafka-client.p12``；两者都没有则报配置错误（不是静默降级）。
"""
from __future__ import annotations

import os
import re
from typing import Dict, Optional, Tuple

# ─── 车端部署约定（Deployment_Guide §5.6）───
DEFAULT_PROPERTIES_PATH = "/etc/hunter/kafka/kafka.properties"
DEFAULT_BUNDLE_DIR = "/etc/hunter/kafka"

CA_FILE = "ca-cert.pem"
CLIENT_CERT_FILE = "client-cert.pem"
CLIENT_KEY_FILE = "client-key.pem"
CLIENT_P12_FILE = "kafka-client.p12"

# 装配结果中的元信息键（不属于 librdkafka 配置，建客户端前必须用 strip_meta 剔除）
PROPERTIES_PATH_KEY = "__properties_path__"

# 视口：口令占位/未填的判定（运营忘记填 SCRAM 口令时必须是启动即失败，
# 而不是运行期 SASL 认证反复失败——后者在现场极难与网络问题区分）
_INVALID_SECRET = re.compile(r"^\s*(<.*>|\{\{.*\}\}|CHANGE_ME|CHANGE-me|your_?password|)\s*$",
                             re.IGNORECASE)

# librdkafka 完全不认识的 Java 键（必须显式处理，不能透传）
_JAVA_ONLY_KEYS = {
    "sasl.jaas.config",
    "ssl.truststore.location",
    "ssl.truststore.password",
    "ssl.truststore.type",
    "ssl.keystore.type",
    "ssl.key.password.file",
}


class KafkaConfigError(RuntimeError):
    """接入参数缺失/非法（启动即失败，交由 systemd/launch 重试策略处理）。"""


# ---------------------------------------------------------------------------
# properties 读取
# ---------------------------------------------------------------------------
def _strip_inline_comment(value: str) -> str:
    """剥离尾注释：仅当 # 或 ; 前是空白（或位于串首）时才视为注释起始。"""
    out = []
    for i, ch in enumerate(value):
        if ch in "#;" and (i == 0 or value[i - 1] in " \t"):
            break
        out.append(ch)
    return "".join(out).strip()


def load_properties(path: str = DEFAULT_PROPERTIES_PATH) -> Dict[str, str]:
    """读取 Java Properties 格式文件（支持 # / ! 注释、``\\`` 续行、``\\:``/``\\=`` 转义）。

    返回保持原始键名的 dict；文件不存在时抛 KafkaConfigError（附排查提示）。
    """
    if not os.path.isfile(path):
        raise KafkaConfigError(
            f"未找到 Kafka 接入参数文件：{path}"
            "（请先把 HunterCore 接入包部署到 /etc/hunter/kafka/，见 Deployment_Guide §5.6）")

    props: Dict[str, str] = {}
    try:
        with open(path, "r", encoding="utf-8") as fh:
            raw_lines = fh.read().splitlines()
    except OSError as exc:
        raise KafkaConfigError(
            f"读不到 Kafka 接入参数文件：{path}（{exc}）；现为哪个用户可读用 "
            f"`stat -c '%U:%G %a' {path}` 查；该文件需对运行 Agent 的用户开放组读位"
            "（部署脚本按 root:运行组 0640 落盘）") from exc

    # 1) 续行合并：以奇数个反斜杠结尾的行与下一行拼接（jaas 配置常跨行）
    logical: list[str] = []
    buf = ""
    for line in raw_lines:
        stripped = line.strip()
        if not buf:
            if not stripped or stripped[0] in "#!":
                continue
            buf = line
        else:
            buf = buf + " " + stripped
        if _ends_with_continuation(buf):
            buf = buf[:-1]
            continue
        logical.append(buf)
        buf = ""
    if buf:
        logical.append(buf)

    # 2) 键值拆分
    for line in logical:
        stripped = line.strip()
        if not stripped or stripped[0] in "#!":
            continue
        key, sep, value = _split_kv(stripped)
        if not sep:
            # 无分隔符的行（纯 flag）忽略
            continue
        props[key] = _strip_inline_comment(value)
    return props


def _ends_with_continuation(line: str) -> bool:
    n = 0
    for ch in reversed(line):
        if ch == "\\":
            n += 1
        else:
            break
    return n % 2 == 1


def _split_kv(line: str) -> Tuple[str, str, str]:
    """按 Properties 规则拆 key / 分隔符 / value，支持 ``\\:\\`` ``\\=`` 转义。"""
    i = 0
    n = len(line)
    while i < n:
        ch = line[i]
        if ch == "\\":                       # 转义：跳过下一个字符
            i += 2
            continue
        if ch in "=:":
            return (_unescape(line[:i]).strip(), ch, _unescape(line[i + 1:]).strip())
        if ch in " \t":                      # 空白也可能作分隔符（Properties 允许 key value）
            rest_offset = i + len(line[i:].lstrip(" \t"))
            rest = line[rest_offset:rest_offset + 1]
            if rest in ("=", ":"):
                return (_unescape(line[:i]).strip(), rest,
                        _unescape(line[rest_offset + 1:]).strip())
            return _unescape(line[:i]).strip(), "", ""
        i += 1
    return _unescape(line).strip(), "", ""


def _unescape(text: str) -> str:
    return text.replace("\\=", "=").replace("\\:", ":").replace("\\n", "\n").replace("\\t", "\t")


# ---------------------------------------------------------------------------
# 凭据解析
# ---------------------------------------------------------------------------
def parse_jaas_credentials(jaas: str) -> Tuple[Optional[str], Optional[str]]:
    """从 ``sasl.jaas.config`` 提取 username/password（librdkafka 无 jaas 支持）。

    形如：``org.apache.kafka.common.security.scram.ScramLoginModule required
    username="HUNTER-001" password="xxx";``
    """
    user = re.search(r'username\s*=\s*"([^"]*)"', jaas)
    pwd = re.search(r'password\s*=\s*"([^"]*)"', jaas)
    return (user.group(1) if user else None, pwd.group(1) if pwd else None)


def _secret_or_none(value: Optional[str]) -> Optional[str]:
    if value is None:
        return None
    v = value.strip().strip('"').strip("'")
    if not v or _INVALID_SECRET.match(v):
        return None
    return v


# ---------------------------------------------------------------------------
# 配置装配
# ---------------------------------------------------------------------------
def build_client_conf(
    props: Dict[str, str],
    *,
    bundle_dir: str = DEFAULT_BUNDLE_DIR,
    client_id: Optional[str] = None,
    role: str = "producer",
    acks: Optional[str] = None,
    extra: Optional[Dict[str, str]] = None,
) -> Dict[str, str]:
    """把 Java 风格 kafka.properties 装配成 librdkafka 配置 dict。

    Args:
        props: :func:`load_properties` 的结果。
        bundle_dir: 接入包目录（ca/cert/key/p12 所在处）。
        client_id: ``client.id``（日志与平台侧连接审计定位用）。
        role: ``producer`` | ``consumer``（仅用于错误提示）。
        acks: 覆盖 properties 中的 acks。契约要求 telemetry=1、
            event/command/ota_* = all，故必须按 Topic 分别建 producer。
        extra: 追加/覆盖任意 librdkafka 键（group.id 等）。

    Raises:
        KafkaConfigError: 缺 bootstrap/凭据/客户端证书/SCRAM 口令未填。
    """
    if not props.get("bootstrap.servers"):
        raise KafkaConfigError("kafka.properties 缺少 bootstrap.servers")

    conf: Dict[str, str] = {"bootstrap.servers": props["bootstrap.servers"]}

    # ── 安全协议与 SASL（SCRAM）凭据 ──
    for key in ("security.protocol", "sasl.mechanism"):
        if props.get(key):
            conf[key] = props[key].lower() if key == "security.protocol" else props[key]

    username = _secret_or_none(props.get("sasl.username"))
    password = _secret_or_none(props.get("sasl.password"))
    if (not username or not password) and props.get("sasl.jaas.config"):
        j_user, j_pwd = parse_jaas_credentials(props["sasl.jaas.config"])
        username = username or _secret_or_none(j_user)
        password = password or _secret_or_none(j_pwd)

    # 环境变量兜底（临时联调/CI 用；生产仍建议写进接入包 properties）
    username = os.environ.get("HUNTER_KAFKA_SASL_USERNAME") or username
    password = os.environ.get("HUNTER_KAFKA_SASL_PASSWORD") or password

    if conf.get("security.protocol", "").lower().startswith("sasl"):
        if not username or not password:
            raise KafkaConfigError(
                "SASL 凭据缺失：请在 "
                f"{DEFAULT_PROPERTIES_PATH} 的 sasl.jaas.config 中把 "
                "password=\"<SCRAM_PASSWORD>\" 替换为运营一次性下发的 SCRAM 口令"
                "（或设置环境变量 HUNTER_KAFKA_SASL_PASSWORD）")
        conf["sasl.username"] = username
        conf["sasl.password"] = password

    # ── TLS / mTLS ──
    protocol = conf.get("security.protocol", "plaintext")
    if "ssl" in protocol:
        _apply_ssl(conf, props, bundle_dir, role)

    # ── 生产者基准参数（对齐 contracts/kafka/topics.yaml#producer_defaults）──
    for key in ("linger.ms", "batch.size", "retries", "compression.type",
                "message.timeout.ms", "request.timeout.ms",
                "reconnect.backoff.ms", "reconnect.backoff.max.ms",
                "socket.keepalive.enable"):
        if props.get(key):
            conf[key] = props[key]
    if role == "consumer":
        for key in ("enable.auto.commit", "auto.offset.reset", "isolation.level",
                    "session.timeout.ms", "max.poll.interval.ms"):
            if props.get(key):
                conf[key] = props[key]
        # Java 的 true/false 与 librdkafka 一致，无需转换
    if acks:
        conf["acks"] = str(acks)
    elif props.get("acks"):
        conf["acks"] = str(props["acks"])

    if client_id:
        conf["client.id"] = client_id

    if extra:
        conf.update({k: str(v) for k, v in extra.items()})
    return conf


def _apply_ssl(conf: Dict[str, str], props: Dict[str, str],
               bundle_dir: str, role: str) -> None:
    """装配 CA / 客户端证书（mTLS）/ 主机名校验。"""
    # 1) 信任链：properties 显式指定 → bundle 的 ca-cert.pem → 系统 CA（回落并告警）
    ca = props.get("ssl.ca.location") or _existing(bundle_dir, CA_FILE)
    if ca:
        conf["ssl.ca.location"] = ca
    # 没有 ca-cert.pem 时不写 ssl.ca.location：librdkafka 会探针系统信任链。
    # 注意 JKS truststore 无法被 librdkafka 读取，故不能透传 ssl.truststore.location。

    # 2) 客户端证书（平台要求 SASL_SSL + mTLS，CN 必须等于 vehicle_id）
    cert = props.get("ssl.certificate.location") or _existing(bundle_dir, CLIENT_CERT_FILE)
    key = props.get("ssl.key.location") or _existing(bundle_dir, CLIENT_KEY_FILE)
    p12 = props.get("ssl.keystore.location") or _existing(bundle_dir, CLIENT_P12_FILE)

    if cert and key:
        conf["ssl.certificate.location"] = cert
        conf["ssl.key.location"] = key
        p12_pwd = _secret_or_none(props.get("ssl.key.password"))
        if p12_pwd:
            conf["ssl.key.password"] = p12_pwd
    elif p12:
        # librdkafka 原生支持 PKCS#12
        conf["ssl.keystore.location"] = p12
        ks_pwd = _secret_or_none(props.get("ssl.keystore.password")) or \
            os.environ.get("HUNTER_KAFKA_P12_PASSWORD")
        if not ks_pwd:
            raise KafkaConfigError(
                f"{p12} 需要 ssl.keystore.password（接入包 README 中的一次性 PKCS12 口令；"
                "推荐直接写进 kafka.properties 的 ssl.keystore.password= 一行，"
                "或设置环境变量 HUNTER_KAFKA_P12_PASSWORD）")
        conf["ssl.keystore.password"] = ks_pwd
    else:
        raise KafkaConfigError(
            f"未找到客户端证书：{bundle_dir}/{{{CLIENT_CERT_FILE},{CLIENT_KEY_FILE}}} "
            f"或 {CLIENT_P12_FILE}（SASL_SSL 需 mTLS，缺证书 broker 会在握手期拒绝连接）")

    # 3) 主机名校验：Java 的 https ↔ librdkafka 同名枚举（none/https），直接透传
    algorithm = props.get("ssl.endpoint.identification.algorithm")
    if algorithm is not None:
        algorithm = algorithm.strip().lower()
        if algorithm in ("", "https", "dns", "default"):
            conf["ssl.endpoint.identification.algorithm"] = "https"
        elif algorithm == "none":
            conf["ssl.endpoint.identification.algorithm"] = "none"
            conf["enable.ssl.certificate.verification"] = "false"
        else:
            conf["ssl.endpoint.identification.algorithm"] = algorithm


def _existing(directory: str, name: str) -> Optional[str]:
    """从 bundle 目录取文件路径；**存在但读不到时直接报错**，不静默降级。

    静默返回 None 会让 Agent 拿着不完整的 TLS 配置去连 broker，最后报成
    “认证失败”这类假象（实机踩中过：私钥装成 root:组 0600，0600 只授予属主，
    以 agilex 跑的 data_agent 打不开，librdkafka 只给
    `ssl.key.location failed: Permission denied`）。
    """
    path = os.path.join(directory, name)
    if not os.path.isfile(path):
        return None
    if not os.access(path, os.R_OK):
        raise KafkaConfigError(
            f"凭据存在但当前用户读不到：{path}（查：stat -c '%U:%G %a' {path}）。"
            "0600 只授予属主，所以私钥/PKCS#12 的属主必须就是运行 Agent 的用户。修正："
            f"sudo chown <运行用户>:<其主组> {path} && sudo chmod 600 {path}"
            "（或重跑 hunter_core_setup.sh，其步骤②.1 会收敛属主并实测可读性）")
    return path


# ---------------------------------------------------------------------------
# 面向 Agent 的高层入口
# ---------------------------------------------------------------------------
def load_conf(
    *,
    properties_path: Optional[str] = None,
    bundle_dir: Optional[str] = None,
    client_id: Optional[str] = None,
    role: str = "producer",
    acks: Optional[str] = None,
    extra: Optional[Dict[str, str]] = None,
    fallback: Optional[Dict[str, str]] = None,
) -> Dict[str, str]:
    """Agent 统一入口：读 properties 装配配置；properties 不存在时用 ``fallback``。

    ``fallback`` 供尚未部署接入包的开发机/旧环境使用（YAML 内联 SASL 参数），
    生产车端必须走 properties（见 Deployment_Guide §5.6）。
    """
    path = os.environ.get("HUNTER_KAFKA_PROPERTIES") or \
        properties_path or DEFAULT_PROPERTIES_PATH
    if os.path.isfile(path):
        props = load_properties(path)
        # env 可覆盖 broker（多环境联调：HUNTER_KAFKA_BOOTSTRAP_SERVERS）
        if os.environ.get("HUNTER_KAFKA_BOOTSTRAP_SERVERS"):
            props["bootstrap.servers"] = os.environ["HUNTER_KAFKA_BOOTSTRAP_SERVERS"]
        conf = build_client_conf(
            props, bundle_dir=bundle_dir or os.path.dirname(path) or DEFAULT_BUNDLE_DIR,
            client_id=client_id, role=role, acks=acks, extra=extra)
        conf[PROPERTIES_PATH_KEY] = path
        return conf

    if fallback is None:
        raise KafkaConfigError(f"未找到接入参数文件且未提供 fallback：{path}")
    return dict(fallback)


def resolve_vehicle_id(props_or_conf: Dict[str, str], default: str = "") -> str:
    """vehicle_id 优先取 SASL 用户名（接入包里 username == CN == vehicle_id）。"""
    user = props_or_conf.get("sasl.username")
    return user or default


def strip_meta(conf: Dict[str, str]) -> Dict[str, str]:
    """剔除 :data:`PROPERTIES_PATH_KEY` 等元信息键，得到可直接交给 confluent_kafka 的配置。"""
    return {k: v for k, v in conf.items() if not k.startswith("__")}


def redact(conf: Dict[str, str]) -> Dict[str, str]:
    """打印/落日志用的脱敏副本（口令一律打星，只保留长度提示）。"""
    out = {}
    for k, v in conf.items():
        if k in ("sasl.password", "ssl.keystore.password", "ssl.key.password"):
            out[k] = f"***({len(str(v))})"
        else:
            out[k] = v
    return out
