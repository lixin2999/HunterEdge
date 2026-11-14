#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""health.ip 自检脚本（HunterCore V1.28.0 契约 · 车端提测前置）。

做三件事，全部离线可跑：

1. 采样一次车端网络地址（默认路由出口地址，UDP connect 不发包、不做 DNS）；
2. 按车端实现语义构造一条 health 报文（`ip` 合法才写键，否则整键省略）；
3. 用 `contracts/kafka/schemas/health.schema.json` 做 draft-07 校验并打印结果。

用法::

    # 单次自检（默认 auto；输出报文 + 校验结论）
    python3 health_ip_selfcheck.py

    # 指定 schema（默认按 HUNTER_CONTRACT_DIR / ../HunterCore / /etc/hunter 顺序探测）
    python3 health_ip_selfcheck.py --schema /path/to/health.schema.json

    # 模拟现场三种取值（对应平台模拟器 --ip auto|none|<IP>）
    python3 health_ip_selfcheck.py --ip none
    python3 health_ip_selfcheck.py --ip 192.168.10.21

    # 全量自检：auto / none / 固定值 / 非法值注入（非法值必须被拦下）
    python3 health_ip_selfcheck.py --selftest

    # 校验抓到的 60s 报文（每行一条 JSON，如 kafka-console-consumer 输出）
    python3 health_ip_selfcheck.py --validate-file captured_health.ndjson

退出码：0 = 全部通过；1 = 校验失败（含"该拦没拦"的非法值分支）。
"""
from __future__ import annotations

import argparse
import ipaddress
import json
import os
import re
import socket
import sys
from typing import Any, Optional

# 与车端 C++ 实现（data_agent/src/health_ip.cpp）保持一致：
# 优先 IPv4（默认路由出口网卡源地址），仅当无 IPv4 才报 IPv6。
PROBES = ((socket.AF_INET, ("8.8.8.8", 80)),
          (socket.AF_INET6, ("2606:4700:4700::1111", 80)))
EXCLUDED_IF_PREFIXES = ("lo", "docker", "br-", "veth", "virbr",
                        "tun", "tap", "wg", "zt", "tailscale")


def _if_name_of(family: int, packed: bytes) -> str:
    """地址 → 接口名（用于排除网桥/VPN；取不到返回空串）。"""
    try:
        import psutil  # 可选依赖；缺失时不做接口名过滤
    except ImportError:
        return ""
    want = ipaddress.ip_address(packed)
    try:
        for name, addrs in psutil.net_if_addrs().items():
            for addr in addrs:
                if str(want) == addr.address.split("%")[0]:
                    return name
    except Exception:  # noqa: BLE001 —— 自检脚本不为接口名探测失败而中断
        return ""
    return ""


def detect_ip(timeout: float = 0.5) -> Optional[str]:
    """默认路由出口地址；失败返回 None（调用方必须**省略** ip 键）。"""
    for family, probe in PROBES:
        sock = socket.socket(family, socket.SOCK_DGRAM)
        try:
            sock.settimeout(timeout)
            sock.connect(probe)                 # UDP connect 不发包
            host, _port = sock.getsockname()[:2]
            addr = ipaddress.ip_address(host)
            if addr.is_loopback or addr.is_link_local or addr.is_unspecified:
                continue
            name = _if_name_of(family, addr.packed)
            if name.lower().startswith(EXCLUDED_IF_PREFIXES):
                continue
            return str(addr)
        except (OSError, ValueError):
            continue
        finally:
            sock.close()
    return None


def normalize_ip(raw: Optional[str]) -> Optional[str]:
    """契约校验（与 C++ 实现同语义）：合法 → 规范形式；非法 → None。

    `ipaddress` 的严格度高于 schema pattern（拒前导零），两者取交集才不会出现
    "schema 能过、平台 ipaddress 拒绝"的灰区；zone id / IPv4-mapped 一并拒绝。
    """
    if raw is None:
        return None
    text = raw.strip()
    if len(text) < 3 or len(text) > 45:         # schema minLength/maxLength
        return None
    if "%" in text:                             # 禁止 zone id
        return None
    if ":" in text and "." in text:             # 禁止 IPv4 内嵌/映射文本（与 C++ 同语义）
        return None
    try:
        addr = ipaddress.ip_address(text)
    except ValueError:
        return None
    if isinstance(addr, ipaddress.IPv4Address):
        return str(addr) if len(text.split(".")) == 4 else None
    if addr.version == 6 and addr.ipv4_mapped is not None:
        return None                             # IPv4-mapped 不在 pattern 内
    return str(addr)                            # 压缩小写形式


def sample_health(vehicle_id: str, ip: Optional[str]) -> dict[str, Any]:
    """按车端实现构造 health 报文：ip 为空则**整个键省略**。"""
    import time
    payload: dict[str, Any] = {
        "vehicle_id": vehicle_id,
        "timestamp": round(time.time(), 3),
        "status": "online_idle",
        "system": {
            "cpu_usage": 45.2,
            "gpu_usage": 62.8,
            "memory_usage_mb": 18500,
            "gpu_temp": 68.5,
            "cpu_temp": 65.2,
            "network_rssi": -65,
            "network_latency_ms": 45,
        },
    }
    if ip:
        payload["ip"] = ip
    return payload


# ---------------------------------------------------------------------------
# schema 校验（jsonschema 优先；缺失时按 schema 内置最小校验，避免现场装不上就跑不了）
# ---------------------------------------------------------------------------
def _load_schema(path: Optional[str]) -> tuple[Optional[dict], str]:
    candidates = []
    if path:
        candidates.append(path)
    env_dir = os.environ.get("HUNTER_CONTRACT_DIR", "")
    if env_dir:
        candidates.append(os.path.join(env_dir, "kafka", "schemas", "health.schema.json"))
    here = os.path.dirname(os.path.abspath(__file__))
    # 开发机常见布局：HunterEdge 与 HunterCore 同级
    candidates.append(os.path.join(
        here, "..", "..", "..", "..", "..", "HunterCore", "contracts", "kafka",
        "schemas", "health.schema.json"))
    candidates.append(os.path.join(
        here, "..", "..", "..", "..", "HunterCore", "contracts", "kafka",
        "schemas", "health.schema.json"))
    candidates.append("/etc/hunter/contracts/kafka/schemas/health.schema.json")
    for candidate in candidates:
        candidate = os.path.abspath(candidate)
        if os.path.isfile(candidate):
            with open(candidate, "r", encoding="utf-8") as fh:
                return json.load(fh), candidate
    return None, ""


def validate_payload(payload: dict, schema: Optional[dict]) -> list[str]:
    """返回错误列表（空 = 通过）。"""
    if schema is not None:
        try:
            import jsonschema
        except ImportError:
            jsonschema = None
        if jsonschema is not None:
            validator = jsonschema.Draft7Validator(schema)
            return [f"{list(e.path) or '<root>'}: {e.message}" for e in validator.iter_errors(payload)]

    # 内置兜底：required / additionalProperties / status 枚举 / ip pattern / system 必填
    if schema is None:
        return ["未找到 health.schema.json，且未安装 jsonschema —— 无法校验"]
    errors: list[str] = []
    props = schema.get("properties", {})
    for key in schema.get("required", []):
        if key not in payload:
            errors.append(f"缺少必填字段 {key}")
    if schema.get("additionalProperties") is False:
        for key in payload:
            if key not in props:
                errors.append(f"不允许的额外字段 {key}")
    ip_pattern = props.get("ip", {}).get("pattern")
    if "ip" in payload:
        value = payload["ip"]
        if not isinstance(value, str):
            errors.append("ip 不是字符串")
        elif ip_pattern and not re.search(ip_pattern, value):
            errors.append(f"ip={value!r} 不匹配 schema pattern")
    status_enum = props.get("status", {}).get("enum")
    if status_enum and payload.get("status") not in status_enum:
        errors.append(f"status={payload.get('status')!r} 不在枚举内")
    system = payload.get("system")
    if not isinstance(system, dict):
        errors.append("system 不是对象")
    else:
        for key in props.get("system", {}).get("required", []):
            if key not in system:
                errors.append(f"system 缺少 {key}")
    return errors


def _report(label: str, payload: dict, schema: Optional[dict], schema_path: str) -> bool:
    errors = validate_payload(payload, schema)
    has_ip = "ip" in payload
    print(f"[{label}] ip={'省略' if not has_ip else payload['ip']!r} "
          f"→ {'PASS' if not errors else 'FAIL'}")
    if errors:
        for err in errors:
            print(f"    ✗ {err}")
    print("    " + json.dumps(payload, ensure_ascii=False))
    if schema_path:
        print(f"    schema={schema_path}")
    return not errors


def main() -> int:
    parser = argparse.ArgumentParser(description="health.ip 契约自检（V1.28.0）")
    parser.add_argument("--schema", default=None,
                        help="health.schema.json 路径（默认 HUNTER_CONTRACT_DIR / HunterCore / /etc/hunter 探测）")
    parser.add_argument("--vehicle-id", default="HUNTER-001", help="台账 vehicle_id（须等于 Kafka key/用户名）")
    parser.add_argument("--ip", default="auto",
                        help="auto | none | 固定地址（对齐平台模拟器 --ip）")
    parser.add_argument("--selftest", action="store_true",
                        help="全量自检：auto / none / 固定值 / 非法值注入")
    parser.add_argument("--validate-file", default=None,
                        help="校验抓包文件（每行一条 health JSON）")
    args = parser.parse_args()

    schema, schema_path = _load_schema(args.schema)
    try:
        import jsonschema  # noqa: F401
        engine = "jsonschema(draft-07)"
    except ImportError:
        engine = "内置兜底校验（未安装 jsonschema，建议 pip3 install jsonschema）"
    print(f"校验引擎：{engine}")
    print(f"schema：{schema_path or '未找到（用 --schema 指定）'}")

    ok = True

    if args.validate_file:
        count = 0
        with open(args.validate_file, "r", encoding="utf-8") as fh:
            for lineno, line in enumerate(fh, 1):
                line = line.strip()
                if not line:
                    continue
                count += 1
                try:
                    payload = json.loads(line)
                except json.JSONDecodeError as exc:
                    print(f"[line {lineno}] JSON 解析失败：{exc}")
                    ok = False
                    continue
                errors = validate_payload(payload, schema)
                if errors:
                    ok = False
                    print(f"[line {lineno}] FAIL")
                    for err in errors:
                        print(f"    ✗ {err}")
                elif "ip" not in payload:
                    print(f"[line {lineno}] PASS（无 ip 键，平台保留人工录入值）")
        print(f"共校验 {count} 条 → {'全部通过' if ok else '存在失败项'}")
        return 0 if ok else 1

    if args.selftest:
        auto_ip = detect_ip()
        print(f"auto 探测结果：{auto_ip or '（无默认路由/探测失败）'}")
        ok &= _report("auto", sample_health(args.vehicle_id, normalize_ip(auto_ip)), schema, schema_path)
        ok &= _report("none", sample_health(args.vehicle_id, None), schema, schema_path)

        pinned = normalize_ip("192.168.10.21")
        ok &= _report("pinned", sample_health(args.vehicle_id, pinned), schema, schema_path)

        # 非法值必须被拦下：normalize_ip 返回 None ⇒ 构造出的报文不含 ip 键
        for bad in ("10.0.0.3/24", "192.168.10.21:1883", "fe80::1%eth0",
                    "::ffff:192.168.10.21", "192.168.10", "", "unknown"):
            if normalize_ip(bad) is not None:
                print(f"[非法值拦截] FAIL：{bad!r} 未被拦下（生产会发脏值 → DLQ）")
                ok = False
            else:
                print(f"[非法值拦截] PASS：{bad!r} → 省略 ip 键")
        # 反向校验：schema 本身必须拒绝"手工塞进去"的脏值（证明校验有效）
        dirty = sample_health(args.vehicle_id, "10.0.0.3/24")
        if validate_payload(dirty, schema):
            print("[schema 反向校验] PASS：脏值被 schema 拒绝")
        else:
            print("[schema 反向校验] FAIL：schema 接受了 10.0.0.3/24，检查 schema 版本")
            ok = False
        print(f"\n自检结论：{'PASS' if ok else 'FAIL'}")
        return 0 if ok else 1

    mode = args.ip.strip()
    lowered = mode.lower()
    if lowered in ("none", "off"):
        ip = None
    elif lowered in ("", "auto"):
        detected = detect_ip()
        ip = normalize_ip(detected)
        if detected and not ip:
            print(f"⚠ 探测到 {detected!r} 但未通过契约校验，已省略 ip 键")
    else:
        ip = normalize_ip(mode)
        if ip is None:
            print(f"✗ 固定值 {mode!r} 非法（拒绝发送脏值）")
            return 1
    ok = _report(f"--ip {args.ip}", sample_health(args.vehicle_id, ip), schema, schema_path)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
