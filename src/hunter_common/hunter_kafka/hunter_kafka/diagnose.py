#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""车端接入自检 CLI：``hunter-kafka-check``（也可 ``python3 -m hunter_kafka.diagnose``）。

按接入包 README 的「联调验证」步骤把**能离线判定的失败原因一次跑完**，
避免现场靠 Agent 日志猜（SASL 认证失败、证书不受信、Topic 不存在、
权限不足、时间漂移导致 TLS 校验失败 这几类错误的表象高度相似）。

检查层次（前一层失败即短路返回，错误信息给出处置动作）：
  1. 接入包文件齐备 + 私钥权限 0600（README「安全注意」硬要求）
  2. kafka.properties 解析（含 SCRAM 口令是否仍为占位符）
  3. 证书有效期与 CN 是否等于 vehicle_id（时间漂移会直接导致 TLS 失败）
  4. Broker TCP 连通（网络/防火墙层）
  5. SASL_SSL + mTLS 握手 + Topic 清单核对（AdminClient）
  6. 端到端投递证实（发一条 telemetry 测试消息，等 on_delivery）

退出码：0=全部通过；10=参数/文件问题；20=TLS/证书问题；30=认证失败；
40=网络不可达；50=Topic/权限问题；60=投递失败。
"""
from __future__ import annotations

import argparse
import datetime as _dt
import os
import socket
import stat
import sys
from typing import Optional

from .client import TOPIC_TYPES
from .config import (CA_FILE, CLIENT_CERT_FILE, CLIENT_KEY_FILE, CLIENT_P12_FILE,
                     DEFAULT_BUNDLE_DIR, DEFAULT_PROPERTIES_PATH, KafkaConfigError,
                     load_conf, load_properties, redact, strip_meta)

EXIT_OK = 0
EXIT_CONFIG = 10
EXIT_TLS = 20
EXIT_AUTH = 30
EXIT_NETWORK = 40
EXIT_TOPIC = 50
EXIT_DELIVERY = 60

_OK = "\033[32m✓\033[0m"
_FAIL = "\033[31m✗\033[0m"
_WARN = "\033[33m!\033[0m"


class Check:
    """简单的检查项累加器（统一输出格式，便于现场截图回传给运营）。"""

    def __init__(self) -> None:
        self.failures: list[tuple[int, str]] = []
        self.warnings: list[str] = []

    def ok(self, name: str, detail: str = "") -> None:
        print(f"{_OK} {name}" + (f"：{detail}" if detail else ""))

    def warn(self, name: str, detail: str) -> None:
        print(f"{_WARN} {name}：{detail}")
        self.warnings.append(name)

    def fail(self, code: int, name: str, detail: str) -> None:
        print(f"{_FAIL} {name}：{detail}")
        self.failures.append((code, name))


def _file_mode_ok(path: str, expect: int = 0o600) -> bool:
    mode = stat.S_IMODE(os.stat(path).st_mode)
    return mode == expect


def check_bundle_dir(chk: Check, bundle_dir: str, properties_path: str,
                     require: bool) -> None:
    """检查接入包文件齐备性与权限（私钥必须 0600）。"""
    missing = [p for p in (properties_path,) if not os.path.isfile(p)]
    if missing:
        if require:
            chk.fail(EXIT_CONFIG, "接入参数文件",
                     f"不存在：{missing[0]}。请先执行 hunter_core_setup.sh 部署接入包"
                     f"（README 步骤 1）")
        else:
            chk.warn("接入参数文件", f"不存在：{missing[0]}（跳过离线检查）")
        return

    chk.ok("接入参数文件", properties_path)

    for name, must_exist in ((CA_FILE, True), (CLIENT_CERT_FILE, True),
                             (CLIENT_KEY_FILE, True), (CLIENT_P12_FILE, False)):
        path = os.path.join(bundle_dir, name)
        if os.path.isfile(path):
            if name.endswith("-key.pem") and not _file_mode_ok(path):
                chk.fail(EXIT_CONFIG, "私钥权限",
                         f"{path} 权限为 {oct(stat.S_IMODE(os.stat(path).st_mode))}，"
                         "必须 0600（README：私钥属敏感凭据）")
            else:
                chk.ok(name, path)
        elif must_exist:
            chk.fail(EXIT_CONFIG, name, f"缺失：{path}")


def check_properties(chk: Check, properties_path: str,
                     bundle_dir: str) -> Optional[dict]:
    """解析参数文件并装配 librdkafka 配置；失败原因直接给出。"""
    try:
        props = load_properties(properties_path)
    except KafkaConfigError as exc:
        chk.fail(EXIT_CONFIG, "kafka.properties 解析", str(exc))
        return None

    required = ("bootstrap.servers", "security.protocol", "sasl.mechanism")
    for key in required:
        if not props.get(key):
            chk.fail(EXIT_CONFIG, "kafka.properties 内容", f"缺少必需键 {key}")
            return None
    chk.ok("kafka.properties 解析",
           f"bootstrap={props['bootstrap.servers']}, "
           f"protocol={props['security.protocol']}, mech={props['sasl.mechanism']}")

    try:
        conf = load_conf(properties_path=properties_path, bundle_dir=bundle_dir,
                         client_id="hunter-kafka-check", role="producer", acks="1")
    except KafkaConfigError as exc:
        chk.fail(EXIT_CONFIG, "SASL/mTLS 配置装配", str(exc))
        return None

    masked = redact(conf)
    secret_keys = ("sasl.password", "ssl.keystore.password", "ssl.key.password")
    for key in secret_keys:
        if key in masked:
            chk.ok(key, masked[key])
    print("   装配后的 librdkafka 配置（脱敏）：")
    for key in sorted(masked):
        if key.startswith("__"):
            continue
        print(f"     {key} = {masked[key]}")
    return conf


def check_certificate(chk: Check, conf: dict, vehicle_id: str) -> None:
    """证书有效期 / CN 与 vehicle_id 一致性 / 系统时间漂移。"""
    cert_path = conf.get("ssl.certificate.location")
    if not cert_path:
        chk.warn("客户端证书", "走 PKCS#12（kafka-client.p12）路径，跳过 PEM 明细检查")
        return
    try:
        from cryptography import x509
        from cryptography.hazmat.primitives import serialization  # noqa: F401
    except ImportError:
        chk.warn("客户端证书", "未安装 cryptography（pip3 install cryptography），跳过 CN/有效期检查")
        return

    try:
        with open(cert_path, "rb") as fh:
            cert = x509.load_pem_x509_certificate(fh.read())
    except Exception as exc:  # noqa: BLE001
        chk.fail(EXIT_TLS, "客户端证书", f"解析失败：{exc}")
        return

    now = _dt.datetime.now(_dt.timezone.utc)
    not_before = getattr(cert, "not_valid_before_utc", None) or \
        cert.not_valid_before.replace(tzinfo=_dt.timezone.utc)
    not_after = getattr(cert, "not_valid_after_utc", None) or \
        cert.not_valid_after.replace(tzinfo=_dt.timezone.utc)
    if now < not_before or now > not_after:
        chk.fail(EXIT_TLS, "证书有效期",
                 f"当前系统时间 {now.isoformat()} 不在 "
                 f"[{not_before.isoformat()}, {not_after.isoformat()}] 内"
                 "——先用 `timedatectl` 校时（TLS 对时钟极敏感）")
        return
    chk.ok("证书有效期", f"{not_before.date()} ~ {not_after.date()}")

    cn = cert.subject.get_attributes_for_oid(x509.NameOID.COMMON_NAME)
    cn_value = cn[0].value if cn else ""
    if cn_value and cn_value != vehicle_id:
        chk.fail(EXIT_TLS, "证书 CN",
                 f"CN={cn_value} 与 vehicle_id={vehicle_id} 不一致"
                 "（平台按 CN 鉴权，不一致会被拒绝或落到别人的 Topic）")
    else:
        chk.ok("证书 CN", cn_value or "(未取到)")


def check_tcp(chk: Check, bootstrap: str, timeout: float) -> None:
    host, _, port = bootstrap.partition(":")
    try:
        with socket.create_connection((host, int(port or 9093)), timeout=timeout):
            chk.ok("Broker TCP 连通", f"{host}:{port or 9093}")
    except OSError as exc:
        chk.fail(EXIT_NETWORK, "Broker TCP 连通",
                 f"{host}:{port or 9093} 不可达：{exc}（检查上联链路/DNS/防火墙）")


def check_broker(chk: Check, conf: dict, vehicle_id: str, timeout: float) -> Optional[set]:
    """SASL_SSL + mTLS 握手 + Topic 清单核对。返回 broker 上的 Topic 集合。"""
    try:
        from confluent_kafka import AdminClient
        from confluent_kafka.errors import KafkaException, SaslAuthenticationException
    except ImportError as exc:
        chk.fail(EXIT_CONFIG, "confluent_kafka", f"未安装：{exc}（pip3 install confluent-kafka）")
        return None

    admin_conf = strip_meta(conf)
    admin_conf["socket.timeout.ms"] = str(int(timeout * 1000))
    admin = AdminClient(admin_conf)
    expected = {f"hunter.{vehicle_id}.{t}" for t in TOPIC_TYPES}
    try:
        names = admin.list_topics(timeout=timeout).topics
    except SaslAuthenticationException as exc:
        chk.fail(EXIT_AUTH, "SASL 认证", f"{exc}（核对 SCRAM 口令/用户名；"
                                         "确认已安装 SCRAM 插件：apt install cyrus-sasl-scram）")
        return None
    except KafkaException as exc:
        code = exc.args[0].code() if exc.args else None
        # 握手期证书不受信 / mTLS 未通过 都落在这里，与认证失败区分开
        chk.fail(EXIT_TLS if code in (13, 29, -151) else EXIT_AUTH,
                 "Broker 握手", f"{exc}（err={code}；若提示 certificate 请确认 "
                 f"{CA_FILE} 与 {CLIENT_CERT_FILE}/{CLIENT_KEY_FILE} 属于同一 vehicle_id）")
        return None

    chk.ok("Broker 握手（SASL_SSL + mTLS）", f"共见 {len(names)} 个 Topic")
    absent = sorted(expected - set(names))
    if absent:
        chk.fail(EXIT_TOPIC, "Topic 清单",
                 f"缺失 {len(absent)}/8：{', '.join(absent)}（联系运营按 vehicle_id 建齐）")
    else:
        chk.ok("Topic 清单", f"8/8 齐备（{vehicle_id}）")
    return expected


def check_delivery(chk: Check, conf: dict, vehicle_id: str, timeout: float) -> None:
    """端到端投递证实：produce 一条测试遥测并等 on_delivery（produce() 成功 ≠ 送达）。"""
    from confluent_kafka import Producer

    topic = f"hunter.{vehicle_id}.telemetry"
    state: dict = {}

    def _dr(err, msg):
        state["err"] = err
        state["ok"] = err is None

    producer = Producer(strip_meta(conf))
    payload = ('{"vehicle_id":"%s","timestamp":%d,"source":"hunter-kafka-check",'
               '"note":"接入自检测试消息"}' % (vehicle_id, int(_dt.datetime.now().timestamp())))
    try:
        producer.produce(topic, value=payload.encode("utf-8"), on_delivery=_dr)
    except Exception as exc:  # noqa: BLE001
        chk.fail(EXIT_DELIVERY, "端到端投递", f"produce 失败：{exc}")
        return
    producer.flush(timeout)
    if state.get("ok"):
        chk.ok("端到端投递", f"{topic} 已收到 broker ack（平台侧应能看到 last_online_time 刷新）")
    elif state.get("err"):
        chk.fail(EXIT_DELIVERY, "端到端投递", f"投递失败：{state['err']}")
    else:
        chk.fail(EXIT_DELIVERY, "端到端投递",
                 f"{timeout:.0f}s 内无投递回调（broker 不可达或分区 leader 异常）")


def main(argv: Optional[list] = None) -> int:
    parser = argparse.ArgumentParser(
        prog="hunter-kafka-check",
        description="HunterCore 车端 Kafka 接入自检（配置/证书/网络/认证/Topic/投递）")
    parser.add_argument("--vehicle-id", default=None,
                        help="车辆 ID（默认取 kafka.properties 的 SASL 用户名，与证书 CN 相同）")
    parser.add_argument("--properties", default=os.environ.get(
        "HUNTER_KAFKA_PROPERTIES", DEFAULT_PROPERTIES_PATH))
    parser.add_argument("--bundle-dir", default=DEFAULT_BUNDLE_DIR)
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--skip-send", action="store_true", help="不做端到端投递测试")
    parser.add_argument("--offline", action="store_true",
                        help="仅做本地配置/证书检查，不连 broker")
    args = parser.parse_args(argv)

    chk = Check()
    print("=" * 72)
    print("HunterCore 车端 Kafka 接入自检")
    print("=" * 72)

    check_bundle_dir(chk, args.bundle_dir, args.properties, require=True)
    if chk.failures:
        return _summary(chk)

    conf = check_properties(chk, args.properties, args.bundle_dir)
    if conf is None:
        return _summary(chk)

    vehicle_id = args.vehicle_id or conf.get("sasl.username")
    if not vehicle_id:
        chk.fail(EXIT_CONFIG, "vehicle_id", "无法从 kafka.properties 推断，请显式传 --vehicle-id")
        return _summary(chk)
    chk.ok("vehicle_id", vehicle_id)

    check_certificate(chk, conf, vehicle_id)
    if chk.failures:
        return _summary(chk)

    if args.offline:
        chk.warn("在线检查", "--offline 指定，已跳过网络/认证/Topic/投递")
        return _summary(chk)

    check_tcp(chk, conf["bootstrap.servers"], args.timeout)
    if chk.failures:
        return _summary(chk)

    check_broker(chk, conf, vehicle_id, args.timeout)
    if chk.failures:
        return _summary(chk)

    if not args.skip_send:
        check_delivery(chk, conf, vehicle_id, args.timeout)
    return _summary(chk)


def _summary(chk: Check) -> int:
    print("-" * 72)
    if chk.failures:
        print(f"结论：{len(chk.failures)} 项失败（警告 {len(chk.warnings)} 项）")
        return chk.failures[0][0]
    print(f"结论：接入链路可用（警告 {len(chk.warnings)} 项）")
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
