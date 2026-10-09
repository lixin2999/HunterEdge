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
  4. SASL 机制插件在位（本地可定，避开“看着像口令错”的 No worthy mechs found）
  5. Broker TCP 连通（网络/防火墙层）
  6. SASL_SSL + mTLS 握手 + Topic 清单核对（AdminClient）
  7. 端到端投递证实（发一条 telemetry 测试消息，等 on_delivery）

退出码：0=全部通过；10=参数/文件问题；20=TLS/证书问题；30=认证失败；
40=网络不可达；50=Topic/权限问题；60=投递失败。
"""
from __future__ import annotations

import argparse
import datetime as _dt
import glob
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


def _ensure_text_output() -> None:
    """给 stdout/stderr 上“编不出就替换”（保留原编码，不强推 UTF-8）。

    输出里带 ✓/✗ 和中文：碰上编不出这些字符的环境（`PYTHONCOERCECLOCALE=0` 后的 C locale、
    Windows GBK 控制台）会直接 UnicodeEncodeError 崩掉，退成“退出码 1”——那不是链路结论，
    现场只能看 Traceback。宁可把符号变成 `?`，也不能让自检整体跑不出结果。
    """
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(errors="replace")
        except (AttributeError, ValueError, OSError):  # 老版本/被重定向的流
            pass


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


def _read_error(path: str) -> str:
    """真打开一次取首字节，能读到就返回空串。

    只看 `st_mode`/`os.access` 会漏掉“属主不是运行用户”这类实际读不到的情况：
    私钥按契约给 0600，而 0600 的**组位与其他位都是 0**——若属主是 root，同组的
    运行用户照样打不开，到 TLS 阶段才报 `ssl.key.location failed: Permission denied`。
    """
    try:
        with open(path, "rb") as fh:
            fh.read(1)
    except OSError as exc:
        return str(exc)
    return ""


def _ownership(path: str) -> str:
    """返回 `owner:group mode`（只看元信息，不读内容），供处置命令直接可粘。"""
    try:
        import grp  # 仅 Linux；Windows 开发机上取不到就降级为 "?"
        import pwd
        st = os.stat(path)
        return "%s:%s %s" % (pwd.getpwuid(st.st_uid).pw_name,
                             grp.getgrgid(st.st_gid).gr_name,
                             oct(stat.S_IMODE(st.st_mode)))
    except Exception:  # noqa: BLE001（缺模块/未知 id 都不能让自检挂）
        return "?"


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
        if not os.path.isfile(path):
            if must_exist:
                chk.fail(EXIT_CONFIG, name, f"缺失：{path}")
            continue
        if name.endswith("-key.pem") and not _file_mode_ok(path):
            chk.fail(EXIT_CONFIG, "私钥权限",
                     f"{path} 权限为 {oct(stat.S_IMODE(os.stat(path).st_mode))}，"
                     "必须 0600（README：私钥属敏感凭据）")
            continue
        why = _read_error(path)
        if why:
            # 不是“权限不够大”，而是“属主不对”：凭据必须能被**运行 Agent 的用户**真的打开
            fix_mode = "0600" if name.endswith("-key.pem") else "0640"
            chk.fail(EXIT_CONFIG, f"{name} 可读性",
                     f"当前用户打不开 {path}：{why}（现为 {_ownership(path)}）。"
                     "0600 只授予属主，**凭据的属主必须就是运行 Agent 的用户**，否则各 Agent "
                     "与自检都会到 TLS 阶段才报 `ssl.key.location failed … Permission denied`。"
                     f"处置：sudo chown <运行用户>:<其主组> {path} && sudo chmod {fix_mode} {path}"
                     "（本版部署脚本已把私钥 chown 到运行用户；旧版装成 root:组 0600 正好踩中）")
            continue
        chk.ok(name, path)


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


def check_sasl_mechanism(chk: Check, conf: dict) -> None:
    """本地判定 SCRAM 机制插件是否在位。

    【包名坑】Ubuntu/Debian **没有** cyrus-sasl-scram 这个包（那是 RHEL/openSUSE 的名字，
    照它装会直接 Unable to locate package）。
    【属主包已现场钉死】Jetson aarch64 jammy 实测：
    `dpkg -S /usr/lib/aarch64-linux-gnu/sasl2/libscram.so`
    → **`libsasl2-modules:arm64`**（不是之前从 Launchpad amd64 清单推的 gssapi-mit）。
    但判据仍一律用**插件文件存不存在**（跨架构/版本都不失真），包名只当处置提示。
    """
    mech = str(conf.get("sasl.mechanism") or "").upper()
    if mech and not mech.startswith("SCRAM"):
        chk.ok("SASL 机制插件", f"{mech}（非 SCRAM，不需 scram 插件）")
        return
    if not mech:
        mech = "SCRAM（properties 未显式指定，按接入包默认）"
    hits: list[str] = []
    for pattern in ("/usr/lib/sasl2/libscram.so", "/usr/lib64/sasl2/libscram.so",
                    "/usr/lib/*/sasl2/libscram.so"):
        hits.extend(glob.glob(pattern))
    if hits:
        chk.ok("SASL 机制插件", f"{mech} → {hits[0]}")
    else:
        chk.fail(EXIT_AUTH, "SASL 机制插件",
                 f"未找到 libscram.so（查过 /usr/lib/sasl2、/usr/lib64/sasl2、/usr/lib/*/sasl2），"
                 f"{mech} 必报 No worthy mechs found；"
                 "aarch64 jammy 实测该文件属主包是 **libsasl2-modules**（不是 gssapi-mit），"
                 "先装它：sudo apt install -y libsasl2-modules；"
                 "无 cyrus-sasl-scram 包（那是 RHEL 系的名字），也别装名为 scram 的包"
                 "（那是概率风险分析工具，与 SASL 无关）；"
                 "已装却无文件时用 dpkg -S /usr/lib/*/sasl2/libscram.so 现查真正属主包，"
                 "仍不存在才需自编 cyrus-sasl2（--enable-scram）；装完重启 Agent 即可（无需重编）")


def check_tcp(chk: Check, bootstrap: str, timeout: float) -> None:
    host, _, port = bootstrap.partition(":")
    try:
        with socket.create_connection((host, int(port or 9093)), timeout=timeout):
            chk.ok("Broker TCP 连通", f"{host}:{port or 9093}")
    except OSError as exc:
        chk.fail(EXIT_NETWORK, "Broker TCP 连通",
                 f"{host}:{port or 9093} 不可达：{exc}（检查上联链路/DNS/防火墙）")


def _err_codes(names: tuple) -> set:
    """按**常量名**取 librdkafka 错误码（数值随版本变，不能硬编码），取不到的跳过。"""
    try:
        from confluent_kafka import KafkaError
    except ImportError:
        return set()
    out = set()
    for name in names:
        val = getattr(KafkaError, name, None)
        try:
            if val is not None:
                out.add(int(val))
        except (TypeError, ValueError):
            pass
    return out


def _classify_broker_error(text: str, code: Optional[int], default: int) -> tuple:
    """把握手/投递期错误归到自检退出码，返回 `(退出码, 标题, 处置提示)`。

    先按文本关键字再按错误码：SASL 失败经常被包成 `_TRANSPORT`，只看码会把“口令错”
    误报成“网络不通”（现场最难排的一类假象）。SCRAM 插件在 Ubuntu 上没有名为
    `cyrus-sasl-scram` 的包（aarch64 jammy 实测属主为 `libsasl2-modules`），
    所以判定只看插件文件存在，属主包用 `dpkg -S` 现查。
    """
    if ("permission denied" in text or "fopen" in text
            or "ssl.key.location" in text or "ssl.certificate.location" in text):
        return (EXIT_TLS, "凭据可读性",
                "进程打不开接入包里的证书/私钥（OpenSSL 直接报 fopen 失败/Permission denied，"
                "表象像 TLS 故障）。0600 只授予**属主**，所以私钥的属主必须就是运行 Agent 的用户。"
                "处置：sudo chown <运行用户>:<其主组> /etc/hunter/kafka/client-key.pem "
                "&& sudo chmod 0600 /etc/hunter/kafka/client-key.pem（本版部署脚本已按此落盘）；"
                "也可直接看自检第 1 层各凭据的可读性结论")
    if ("no worthy mechs" in text or "unsupported sasl mechanism" in text
            or "sasl init" in text):
        return (EXIT_AUTH, "SASL 机制",
                "缺 SCRAM 插件：`ls /usr/lib/*/sasl2/libscram.so` 确认文件在位；"
                "缺则 `sudo apt install -y libsasl2-modules`（aarch64 jammy 实测属主包），"
                "装完重启 Agent 即可（无需重编）")
    if code in _err_codes(("TOPIC_AUTHORIZATION_FAILED", "CLUSTER_AUTHORIZATION_FAILED",
                           "GROUP_AUTHORIZATION_FAILED")) or "authorization" in text \
            or "authorized" in text or "access denied" in text:
        return (EXIT_TOPIC, "Broker 握手",
                "账号无 Describe/List 权限：让平台开通该车账号的 ACL（这不是口令错，改口令无效）")
    if (code in _err_codes(("_AUTHENTICATION", "SASL_AUTHENTICATION_FAILED", "SECURITY_DISABLED"))
            or "authentication" in text or "sasl" in text):
        return (EXIT_AUTH, "SASL 认证",
                "核对 SCRAM 口令与平台侧账号是否已开通；用户名必须等于 vehicle_id 且等于证书 CN")
    if code in _err_codes(("_SSL",)) or "certificate" in text or "ssl" in text:
        return (EXIT_TLS, "Broker 握手",
                f"TLS 校验未过：确认 {CA_FILE} 与 {CLIENT_CERT_FILE}/{CLIENT_KEY_FILE} 属于同一"
                " vehicle_id；先 `timedatectl` 校时（时钟漂会让有效证书直接判为无效）")
    if (code in _err_codes(("_TRANSPORT", "_RESOLVE", "_TIMED_OUT", "_FAIL"))
            or "transport" in text or "resolve" in text or "resolution" in text
            or "timed out" in text or "no broker available" in text):
        return (EXIT_NETWORK, "Broker 握手",
                "握手期连不上：查上联链路/DNS/防火墙，以及 `bootstrap.servers` 是否与 broker 对外地址一致")
    return (default, "Broker 握手", f"未归类错误（err={code}）：按原文排查，必要时带本输出报修")


def check_broker(chk: Check, conf: dict, vehicle_id: str, timeout: float) -> Optional[set]:
    """SASL_SSL + mTLS 握手 + Topic 清单核对。返回 broker 上的 Topic 集合。"""
    try:
        # 【坑（实机误判过）】AdminClient 属于 **confluent_kafka.admin** 子模块，顶层不重导出；
        # 写成 `from confluent_kafka import AdminClient` 会报 `cannot import name 'AdminClient'`，
        # 看上去像“没装 confluent-kafka”其实是导入路径错。异常类则从顶层取（没有
        # `confluent_kafka.errors` 这个模块，也没有 SaslAuthenticationException 这个类）。
        from confluent_kafka import KafkaException
        from confluent_kafka.admin import AdminClient
    except ImportError as exc:
        chk.fail(EXIT_CONFIG, "confluent_kafka 依赖",
                 f"不可用：{exc}（**非接入链路故障**，是车端 Python 包缺失/损坏）；"
                 "查：python3 -c 'import confluent_kafka as c; print(c.__version__, c.__file__)'；"
                 "修：sudo pip3 install -U confluent-kafka 后重跑本自检")
        return None

    admin_conf = strip_meta(conf)
    admin_conf["socket.timeout.ms"] = str(int(timeout * 1000))
    expected = {f"hunter.{vehicle_id}.{t}" for t in TOPIC_TYPES}
    try:
        # AdminClient 的**构造**也会抛 KafkaException(_INVALID_ARG)（如 ssl.key.location 打不开
        # 私钥），必须与 list_topics 走同一套归类；旧写法只包了 list_topics，异常直接穿透成
        # Traceback + 退出码 1（那不是链路结论，现场只能自己读 OpenSSL 报错）。
        admin = AdminClient(admin_conf)
        names = admin.list_topics(timeout=timeout).topics
    except KafkaException as exc:
        err = exc.args[0] if exc.args else None
        code = err.code() if hasattr(err, "code") else None
        text = (err.str() if hasattr(err, "str") else str(err or exc)).lower()
        exit_code, title, hint = _classify_broker_error(text, code, EXIT_AUTH)
        chk.fail(exit_code, title, f"{err if err is not None else exc}：{hint}")
        return None
    except Exception as exc:  # noqa: BLE001（旧版/装坏的 wheel 会抛非 KafkaException）
        chk.fail(EXIT_CONFIG, "Broker 握手",
                 f"意外异常 {type(exc).__name__}: {exc}（若提示没有 AdminClient，说明装的版本异常："
                 "sudo pip3 install -U confluent-kafka）")
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
    from confluent_kafka import Producer  # Producer/Consumer 顶层可得（AdminClient 不在顶层）

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
        err = state["err"]
        code = err.code() if hasattr(err, "code") else None
        text = (err.str() if hasattr(err, "str") else str(err)).lower()
        exit_code, _title, hint = _classify_broker_error(text, code, EXIT_DELIVERY)
        chk.fail(exit_code, "端到端投递", f"{err}：{hint}")
    else:
        chk.fail(EXIT_DELIVERY, "端到端投递",
                 f"{timeout:.0f}s 内无投递回调（broker 不可达或分区 leader 异常；"
                 "Topic 不存在时 broker 可能自动建到未知分区，先查第 6 层 Topic 清单结论）")


def main(argv: Optional[list] = None) -> int:
    parser = argparse.ArgumentParser(
        prog="hunter-kafka-check",
        description="HunterCore 车端 Kafka 接入自检（配置/证书/机制/网络/认证/Topic/投递）")
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
    _ensure_text_output()

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

    check_sasl_mechanism(chk, conf)   # 本地就能定的认证前置条件，先于握手报
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
