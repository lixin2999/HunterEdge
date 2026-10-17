# -*- coding: utf-8 -*-
"""车端 → 平台文件上传 REST 客户端（平台 V1.21.13 presign 契约）。

通道模型（方案 §1）：
  ① REST 控制面：车端 **mTLS 直连** ``https://<SERVER_IP>:8443``，仅做 presign / complete；
  ② 数据面：文件字节走预签名 URL **裸 PUT 直传 MinIO**（host=MINIO_PUBLIC_ENDPOINT，平台不中转）。

鉴权（方案 §3）：复用 Kafka 接入包的客户端证书（CN=vehicle_id）+ 同一张 CA，
**无 JWT / 无 SCRAM / 无自定义身份头**；服务端强校验 ``body.vehicle_id == 证书 CN``。
凭据（PEM）来自 ``upload.presign.bundle_dir``（与 Kafka 同源，绝不落 YAML）。

错误码 → 类型化异常（方案 §8），供上传引擎按重试矩阵分流：
  1001 TLSAuthFailed（不重试）/ 1002 PermanentDeny（不重试）/ 2001 BadParams（不重试）
  3001 NotFound（重 PUT）/ 5001 ServerBusy（退避重试 complete）/ 6001 IntegrityMismatch（重传）
  429 RateLimited（按 Retry-After 退避，不占 attempt 预算）
"""
from __future__ import annotations

import os
import time
from typing import Any, Dict, Optional

from .logging_util import get_logger

logger = get_logger("presign")

# ── 复用 hunter_kafka 的接入包凭据约定（单一可信源），缺库时内联兜底常量 ──
try:
    from hunter_kafka.config import (CA_FILE, CLIENT_CERT_FILE, CLIENT_KEY_FILE,
                                     CLIENT_P12_FILE, DEFAULT_BUNDLE_DIR, _existing)
    HAS_KAFKA_CFG = True
except ImportError:  # pragma: no cover  开发机未 source hunter_kafka 时的兜底
    HAS_KAFKA_CFG = False
    CA_FILE = "ca-cert.pem"
    CLIENT_CERT_FILE = "client-cert.pem"
    CLIENT_KEY_FILE = "client-key.pem"
    CLIENT_P12_FILE = "kafka-client.p12"
    DEFAULT_BUNDLE_DIR = "/etc/hunter/kafka"

    def _existing(directory: str, name: str) -> Optional[str]:
        path = os.path.join(directory, name)
        if not os.path.isfile(path):
            return None
        if not os.access(path, os.R_OK):
            raise PresignUnavailable(
                f"凭据存在但当前用户读不到：{path}（查：stat -c '%U:%G %a' {path}）")
        return path


# ── 异常族（错误码类型化）────────────────────────────────────────
class PresignError(RuntimeError):
    """presign/complete REST 调用失败基类。"""

    code: Optional[int] = None


class PresignUnavailable(PresignError):
    """客户端不可装配（缺 requests / 缺 mTLS PEM / api_base 未配）——上层按上传不可用处置。"""


class TLSAuthFailed(PresignError):
    """1001 设备证书校验失败（网关重验未过）——不重试，属证书/CA 问题。"""
    code = 1001


class PermanentDeny(PresignError):
    """1002 权限/归属错误（vehicle_id≠CN、bucket↔data_type 不匹配、object_key 越权）——不重试。"""
    code = 1002


class BadParams(PresignError):
    """2001 参数错误（格式/缺失；multipart 未同时带 upload_id+parts）——不重试。"""
    code = 2001


class NotFound(PresignError):
    """3001 对象不存在（PUT 未完成 / URL 过期）——需重新 PUT 该片再 complete。"""
    code = 3001


class ServerBusy(PresignError):
    """5001 服务端暂不可用（MinIO/打标/Kafka 通知失败）——退避重试 complete（幂等安全）。"""
    code = 5001


class IntegrityMismatch(PresignError):
    """6001 完整性校验失败（size/md5/sha256 任一不符）——该归档作废，重打包重算后从头重传。"""
    code = 6001


class RateLimited(PresignError):
    """429 触发限流/带宽——按 Retry-After 退避重试，不占 attempt 预算，并降并发/降速。"""
    code = 429

    def __init__(self, message: str, retry_after: float = 5.0):
        super().__init__(message)
        self.retry_after = max(1.0, retry_after)


#: 响应体 code → 异常类型（429 也可能出现在 HTTP 层，非此表）
_CODE_EXC = {
    1001: TLSAuthFailed, 1002: PermanentDeny, 2001: BadParams,
    3001: NotFound, 5001: ServerBusy, 6001: IntegrityMismatch,
}


def _raise_for_code(code: int, message: str, retry_after: float = 0.0) -> None:
    if code == 429:
        raise RateLimited(message or "限流", retry_after=retry_after or 5.0)
    exc_cls = _CODE_EXC.get(int(code)) if code is not None else None
    if exc_cls is None:
        raise PresignError("平台返回错误 code=%s message=%s" % (code, message))
    raise exc_cls("平台返回错误 code=%s message=%s" % (code, message))


class FileUploadClient:
    """mTLS REST 客户端：presign / complete（控制面）+ 预签名 URL 裸 PUT（数据面）。

    requests.Session 绑定 (client-cert, client-key) 与 ca；控制面请求走 verify=CA，
    数据面 PUT 走 MinIO（其证书可能自签/主机名不符），故 PUT 用 verify=False（方案 §9.2）。
    """

    def __init__(self, cfg):
        try:
            import requests
        except ImportError as exc:  # pragma: no cover
            raise PresignUnavailable("presign 依赖 requests 未安装：%s" % exc) from exc
        self._requests = requests

        ps = cfg.get("upload.presign", {}) or {}
        self.api_base = str(ps.get("api_base", "")).strip().rstrip("/")
        self.bundle_dir = ps.get("bundle_dir", DEFAULT_BUNDLE_DIR)
        self.data_type = ps.get("data_type", "rosbag")
        self.content_type = ps.get("content_type", "application/x-tar")
        self.deadline_s = int(ps.get("deadline_s", 3600))
        self.bucket = cfg.get("upload.bucket", "hunter-rosbag")
        self.vehicle_id = cfg.vehicle_id
        self.presign_path = "/api/v1/data/files/presign"
        self.complete_path = "/api/v1/data/files/complete"

        if not self.api_base:
            raise PresignUnavailable(
                "upload.presign.api_base 未配置（平台设备 REST 入口 https://<SERVER_IP>:8443）")
        if not self.vehicle_id:
            raise PresignUnavailable(
                "vehicle_id 为空：presign body.vehicle_id 必须等于证书 CN（取 kafka.properties SASL 用户名）")

        self.session = self._build_session()

    # ── mTLS 会话装配 ──
    def _build_session(self):
        ca, cert, key = self._locate_certs()
        sess = self._requests.Session()
        sess.cert = (cert, key)
        sess.verify = ca
        return sess

    def _locate_certs(self):
        """从接入包目录定位 PEM 三件套；仅有 p12 时明确报错（requests 不能直读 p12）。

        hunter_kafka 的 ``_existing`` 对“存在但读不到”抛 KafkaConfigError（带 chown/chmod 提示）；
        统一转成 PresignUnavailable 以便上传引擎归位为“上传不可用”（告警而非崩溃）。
        """
        try:
            ca = _existing(self.bundle_dir, CA_FILE)
            cert = _existing(self.bundle_dir, CLIENT_CERT_FILE)
            key = _existing(self.bundle_dir, CLIENT_KEY_FILE)
        except PresignError:
            raise
        except Exception as exc:  # noqa: BLE001  KafkaConfigError / OSError 等
            raise PresignUnavailable("mTLS PEM 定位失败：%s" % exc) from exc
        if ca and cert and key:
            return ca, cert, key
        p12 = os.path.join(self.bundle_dir, CLIENT_P12_FILE)
        if os.path.isfile(p12):
            raise PresignUnavailable(
                "mTLS 需 PEM 证书，接入包缺 {%s,%s,%s} 仅有 %s；"
                "请 openssl pkcs12 导出 PEM 到 %s" % (
                    CA_FILE, CLIENT_CERT_FILE, CLIENT_KEY_FILE, CLIENT_P12_FILE, self.bundle_dir))
        raise PresignUnavailable(
            "未找到 mTLS PEM：期望 %s/{%s,%s,%s}" % (
                self.bundle_dir, CA_FILE, CLIENT_CERT_FILE, CLIENT_KEY_FILE))

    # ── 统一响应解析（{code,message,data,...}）──
    def _unwrap(self, resp) -> Dict[str, Any]:
        retry_after = 0.0
        ra = resp.headers.get("Retry-After")
        if ra:
            try:
                retry_after = float(ra)
            except (TypeError, ValueError):
                retry_after = 0.0
        if resp.status_code == 429:
            _raise_for_code(429, "限流（HTTP 429）", retry_after)
        try:
            payload = resp.json()
        except ValueError:
            if resp.status_code == 401:
                raise TLSAuthFailed("HTTP 401（证书重验失败）")
            if resp.status_code == 403:
                raise PermanentDeny("HTTP 403（越权/归属错误）")
            raise PresignError("非 JSON 响应 HTTP=%d body=%.200s" % (resp.status_code, resp.text))
        code = payload.get("code")
        if code == 0:
            return payload.get("data") or {}
        _raise_for_code(code if code is not None else -1,
                        payload.get("message", ""), retry_after)
        return {}  # 不可达

    def _post(self, path: str, body: Dict[str, Any], timeout: float = 30.0) -> Dict[str, Any]:
        url = self.api_base + path
        resp = self._requests.request(
            "POST", url, json=body, cert=self.session.cert, verify=self.session.verify,
            timeout=timeout)
        return self._unwrap(resp)

    # ── 控制面：presign ──
    def presign(self, file_name: str, file_size: int, part_count: int,
                retention: str) -> Dict[str, Any]:
        body: Dict[str, Any] = {
            "vehicle_id": self.vehicle_id,
            "bucket": self.bucket,
            "data_type": self.data_type,
            "file_name": file_name,
            "file_size": int(file_size),
            "part_count": max(1, int(part_count)),
            "content_type": self.content_type,
            "retention": retention,
        }
        data = self._post(self.presign_path, body)
        logger.info("presign 成功 object_key=%s method=%s bucket=%s",
                    data.get("object_key"), data.get("method"), data.get("bucket"),
                    extra={"component": "presign"})
        return data

    # ── 数据面：预签名 URL 裸 PUT ──
    def put_single(self, upload_url: str, path: str) -> None:
        """整对象单次 PUT（≤ part_size 时使用）；必须带与 presign 一致的 Content-Type，否则 403。"""
        with open(path, "rb") as fh:
            data = fh.read()
        resp = self._requests.put(
            upload_url, data=data, headers={"Content-Type": self.content_type},
            timeout=600, verify=False)
        if resp.status_code >= 300:
            raise PresignError("PUT 直传失败 HTTP=%d body=%.200s" % (resp.status_code, resp.text))

    def put_part(self, upload_url: str, chunk: bytes) -> str:
        """逐片 PUT（multipart）；分片 URL 未签 Content-Type，不带该头。返回去引号 ETag。"""
        resp = self._requests.put(upload_url, data=chunk, timeout=600, verify=False)
        if resp.status_code >= 300:
            raise PresignError("分片 PUT 失败 HTTP=%d body=%.200s" % (resp.status_code, resp.text))
        return (resp.headers.get("ETag", "") or "").strip('"')

    # ── 控制面：complete ──
    def complete(self, object_key: str, size: int, md5: str, sha256: str,
                 retention: str, upload_id: Optional[str] = None,
                 parts: Optional[list] = None) -> Dict[str, Any]:
        body: Dict[str, Any] = {
            "vehicle_id": self.vehicle_id,
            "bucket": self.bucket,
            "object_key": object_key,
            "size_bytes": int(size),
            "md5": md5,
            "sha256": sha256,
            "timestamp": int(time.time()),
            "data_type": self.data_type,
            "retention": retention,
        }
        if upload_id is not None:
            body["upload_id"] = upload_id
            body["parts"] = parts or []
        data = self._post(self.complete_path, body, timeout=60.0)
        if not data.get("verified", False):
            # 服务端未确认一致（理论上应已 6001）：保守按完整性不符处理
            raise IntegrityMismatch("complete 返回 verified!=true")
        logger.info("complete verified object_key=%s notify_topic=%s", object_key,
                    data.get("notify_topic"), extra={"component": "presign"})
        return data
