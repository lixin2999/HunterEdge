# -*- coding: utf-8 -*-
"""上传引擎（设计 §6.7 + 平台 V1.21.13 presign 契约）。

三种通道（``upload.endpoint_type``）：
  - ``presign``（生产默认）：服务端 presign 生成 object_key + 车端 mTLS 走 REST + 裸 PUT
    预签名 URL + 服务端 complete 校验三元组；上传单元 = **一个 bag 的单个 tar 归档**（一轮）；
  - ``s3``：boto3 直连 MinIO，车端自拼 object_key + 逐文件上传（断点续传 part-state）；
  - ``gateway``：边缘网关整对象 HTTP PUT。

presign 模式要点（对照方案）：
  - 打包+流式三元组（md5/sha256/size 对**归档字节**一次算出，口径不同于 manifest 跨文件 sha256）；
  - 续传：part-state 键 ``bag_id``，命中且 ``now-presign_ts<deadline_s`` 且归档在 → 复用 object_key/
    upload_id/已完成 parts；否则重新 presign；verified 才删 state + 删 staging 归档；
  - 重试分流：按 presign 异常类型决定退避/重签/重传/不重试（见 :meth:`UploadEngine._handle_failure`）；
  - retention：``seg.protected==1 → event``（永久保留），否则 ``regular``，presign 与 complete 都带且一致。

boto3/requests 缺失或凭据不可用时抛 :class:`UploadUnavailable`，由 agent 归类为上传失败
（保留现场、不删片段），而非崩溃。presign 走 mTLS 证书，不需 AK/SK。
"""
from __future__ import annotations

import asyncio
import hashlib
import json
import math
import os
import random
import time
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional, Tuple

from .logging_util import get_logger
from .pack import pack_and_hash
from .presign import (FileUploadClient, PresignUnavailable, RateLimited,
                      PermanentDeny, BadParams, TLSAuthFailed, NotFound,
                      ServerBusy, IntegrityMismatch)

logger = get_logger("uploader")


class UploadUnavailable(RuntimeError):
    """上传通道不可用（缺库/缺凭据/网络）——上层按上传失败处置。"""


def sha256_file(path: str, chunk: int = 8 * 1024 * 1024) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for block in iter(lambda: fh.read(chunk), b""):
            h.update(block)
    return h.hexdigest()


class TokenBucket:
    """全局上传限速（字节/秒）。asyncio 单线程内使用，非线程安全。"""

    def __init__(self, rate_bps: float):
        self.rate = max(1.0, rate_bps)
        self.capacity = self.rate          # 1s 突发
        self.tokens = self.capacity
        self.updated = time.monotonic()

    def set_rate(self, rate_bps: float) -> None:
        self.rate = max(1.0, rate_bps)
        self.capacity = self.rate

    async def acquire(self, n: int) -> None:
        while True:
            now = time.monotonic()
            self.tokens = min(self.capacity, self.tokens + (now - self.updated) * self.rate)
            self.updated = now
            if self.tokens >= n:
                self.tokens -= n
                return
            need = (n - self.tokens) / self.rate
            await asyncio.sleep(min(need, 0.5))


class _S3Backend:
    """boto3 S3/MinIO 客户端封装（手动分片以支持断点续传）。"""

    def __init__(self, cfg):
        import boto3
        from botocore.config import Config
        u = cfg.get("upload")
        creds = _load_oss_credentials(cfg)
        endpoint = u["endpoint"]
        verify = u.get("use_tls", True)
        self.s3 = boto3.client(
            "s3", endpoint_url=endpoint or None, region_name=u.get("region", "us-east-1"),
            aws_access_key_id=creds.get("access_key"),
            aws_secret_access_key=creds.get("secret_key"),
            aws_session_token=creds.get("token"),
            use_ssl=bool(endpoint.startswith("https")) if endpoint else verify,
            config=Config(signature_version="s3v4", retries={"max_attempts": 0}))
        self.bucket = u["bucket"]

    def head_size(self, key: str) -> Optional[int]:
        try:
            resp = self.s3.head_object(Bucket=self.bucket, Key=key)
            return int(resp["ContentLength"])
        except Exception:  # noqa: BLE001  不存在/网络都按"未完成"处理
            return None

    def initiate(self, key: str, metadata: Dict[str, str]) -> str:
        resp = self.s3.create_multipart_upload(Bucket=self.bucket, Key=key, Metadata=metadata)
        return resp["UploadId"]

    def upload_part(self, key: str, upload_id: str, part_no: int, data: bytes) -> str:
        resp = self.s3.upload_part(Bucket=self.bucket, Key=key, PartNumber=part_no,
                                   UploadId=upload_id, Body=data)
        return resp["ETag"]

    def complete(self, key: str, upload_id: str, parts: List[Dict[str, Any]]) -> str:
        resp = self.s3.complete_multipart_upload(
            Bucket=self.bucket, Key=key, UploadId=upload_id,
            MultipartUpload={"Parts": [{"PartNumber": p["part_no"], "ETag": p["etag"]}
                                       for p in parts]})
        return resp.get("ETag", "")

    def put_small(self, key: str, data: bytes, metadata: Dict[str, str]) -> str:
        resp = self.s3.put_object(Bucket=self.bucket, Key=key, Body=data, Metadata=metadata)
        return resp.get("ETag", "")


class _GatewayBackend:
    """边缘采集网关（HTTP PUT 代理到对象存储）：整对象 PUT，无客户端分片续传。"""

    def __init__(self, cfg):
        import requests  # noqa: F401  探测依赖
        self._requests = requests
        u = cfg.get("upload")
        self.base = u["endpoint"].rstrip("/")
        creds = _load_oss_credentials(cfg)
        self.token = creds.get("token") or creds.get("secret_key")

    def _headers(self, metadata: Dict[str, str]) -> Dict[str, str]:
        h = {("x-amz-meta-%s" % k if not k.startswith("x-amz-") else k): v
             for k, v in metadata.items()}
        if self.token:
            h["Authorization"] = "Bearer %s" % self.token
        return h

    def head_size(self, key: str) -> Optional[int]:
        resp = self._requests.head("%s/%s" % (self.base, key), headers=self._headers({}),
                                   timeout=30, verify=False)
        if resp.status_code == 200 and "Content-Length" in resp.headers:
            return int(resp.headers["Content-Length"])
        return None

    def put_small(self, key: str, data: bytes, metadata: Dict[str, str]) -> str:
        resp = self._requests.put("%s/%s" % (self.base, key), data=data,
                                  headers=self._headers(metadata), timeout=120, verify=False)
        resp.raise_for_status()
        return resp.headers.get("ETag", "gateway")

    # 网关无 multipart 语义 → 大文件也走整对象 PUT（由网关侧处理）
    initiate = None
    upload_part = None
    complete = None


def _load_oss_credentials(cfg) -> Dict[str, str]:
    """凭据单一可信源：/etc/hunter/oss/credentials(INI/JSON) 或环境变量；YAML 内联兜底。"""
    creds: Dict[str, str] = {}
    path = os.environ.get("HUNTER_OSS_CREDENTIALS", "/etc/hunter/oss/credentials")
    if os.path.isfile(path):
        try:
            with open(path, "r", encoding="utf-8") as fh:
                if path.endswith(".json"):
                    data = json.load(fh)
                else:
                    import configparser
                    cp = configparser.ConfigParser()
                    cp.read_string("[oss]\n" + fh.read())
                    data = dict(cp["oss"])
            creds = {"access_key": data.get("access_key", ""),
                     "secret_key": data.get("secret_key", ""),
                     "token": data.get("session_token", data.get("token", ""))}
        except Exception as exc:  # noqa: BLE001
            logger.warning("OSS 凭据读取失败 %s：%s", path, exc)
    # 环境变量覆盖（STS 轮换）
    creds["access_key"] = os.environ.get("AWS_ACCESS_KEY_ID", creds.get("access_key", ""))
    creds["secret_key"] = os.environ.get("AWS_SECRET_ACCESS_KEY", creds.get("secret_key", ""))
    creds["token"] = os.environ.get("AWS_SESSION_TOKEN", creds.get("token", ""))
    # 开发机兜底：YAML 内联
    u = cfg.get("upload")
    if not creds.get("access_key"):
        creds["access_key"] = u.get("access_key", "")
        creds["secret_key"] = u.get("secret_key", "")
    if not creds.get("access_key"):
        raise UploadUnavailable(
            "对象存储凭据缺失：请部署 /etc/hunter/oss/credentials（STS 优先）或设置 "
            "AWS_* 环境变量（仓库 YAML 不存凭据；生产建议 STS 临时凭证，§10.3）")
    return creds


class UploadJob:
    """单对象上传任务（优先级队列元素）。priority 越小越先（P0 manifest…P4 raw）。

    presign 模式下代表**一个 bag 的 tar 归档**：``local_path`` 为 staging 归档路径，
    ``seg_dir``/``manifest_path`` 为打包源，``object_key`` 待 presign 回填，
    ``md5``/``sha256``/``size`` 为归档三元组，``retention`` 为 ``regular``/``event``。
    """

    __slots__ = ("priority", "seq", "bag_id", "local_path", "object_key",
                 "artifact_class", "sha256", "size", "metadata", "attempt",
                 "retention", "md5", "seg_dir", "manifest_path")

    def __init__(self, priority: int, seq: int, bag_id: str, local_path: str,
                 object_key: str, artifact_class: str, sha256: str, size: int,
                 metadata: Dict[str, str], attempt: int = 0, retention: str = "regular",
                 md5: str = "", seg_dir: str = "", manifest_path: Optional[str] = None):
        self.priority = priority
        self.seq = seq
        self.bag_id = bag_id
        self.local_path = local_path
        self.object_key = object_key
        self.artifact_class = artifact_class
        self.sha256 = sha256
        self.size = size
        self.metadata = metadata
        self.attempt = attempt
        self.retention = retention
        self.md5 = md5
        self.seg_dir = seg_dir
        self.manifest_path = manifest_path

    def __lt__(self, other: "UploadJob") -> bool:
        if self.priority != other.priority:
            return self.priority < other.priority
        return self.seq < other.seq


class UploadEngine:
    def __init__(self, cfg, store, bucket_version: str):
        self.cfg = cfg
        self.store = store
        self.bucket_version = bucket_version
        self.part_state_dir = cfg.part_state_dir
        os.makedirs(self.part_state_dir, exist_ok=True)
        self.endpoint_type = cfg.get("upload.endpoint_type", "s3")
        self.presign_mode = self.endpoint_type == "presign"
        self.staging_dir = cfg.staging_dir
        if self.presign_mode:
            os.makedirs(self.staging_dir, exist_ok=True)
        self.deadline_s = int(cfg.get("upload.presign.deadline_s", 3600))
        self.max_attempts = int(cfg.get("upload.max_attempts", 12))
        self.backoff_base = float(cfg.get("upload.retry_backoff_base_s", 1))
        self.backoff_max = float(cfg.get("upload.retry_backoff_max_s", 600))
        self.part_size = int(cfg.get("upload.part_size_mb", 16)) * 1024 * 1024
        self._queue: "asyncio.PriorityQueue[UploadJob]" = asyncio.PriorityQueue()
        self._seq = 0
        self._backend = None
        self._presign_client: Optional[FileUploadClient] = None
        self._bucket = None
        self._workers: List[asyncio.Task] = []
        self._running = False
        self._bucket_rate = TokenBucket(self._current_rate_bps())
        self._inflight: Dict[str, int] = {}    # bag_id → 未完成对象数
        self._done_cb = None                    # (bag_id, UploadJob) → 全部完成回调
        self._fail_cb = None                    # (bag_id, err) → 失败回调
        self._exec = None                       # 网络 IO 线程池

    # ── 生命周期 ──
    def _current_rate_bps(self) -> float:
        now_min = datetime.now(timezone.utc).hour * 60 + datetime.now(timezone.utc).minute
        return self.cfg.current_rate_limit_mbps(now_min) * 1024 * 1024 / 8.0

    async def start(self, done_cb, fail_cb) -> None:
        self._done_cb = done_cb
        self._fail_cb = fail_cb
        endpoint_type = self.cfg.get("upload.endpoint_type", "s3")
        try:
            if endpoint_type == "presign":
                self._presign_client = FileUploadClient(self.cfg)
            elif endpoint_type == "s3":
                self._backend = _S3Backend(self.cfg)
            else:
                self._backend = _GatewayBackend(self.cfg)
        except ImportError as exc:
            raise UploadUnavailable(
                f"上传依赖缺失（{endpoint_type}）：{exc}；presign/gateway 需 requests，s3 需 boto3")
        except PresignUnavailable as exc:
            raise UploadUnavailable(f"presign 通道不可用：{exc}")
        self._bucket = self.cfg.get("upload.bucket")
        self._running = True
        import concurrent.futures
        self._exec = concurrent.futures.ThreadPoolExecutor(
            max_workers=int(self.cfg.get("upload.concurrency.objects", 3)) + 2,
            thread_name_prefix="oss")
        n = int(self.cfg.get("upload.concurrency.objects", 3))
        self._workers = [asyncio.create_task(self._worker(i), name=f"upload-w{i}")
                         for i in range(n)]
        logger.info("上传引擎就绪 backend=%s bucket=%s workers=%d rate=%.1fMB/s",
                    endpoint_type, self._bucket, n, self._current_rate_bps() / 1e6,
                    extra={"component": "uploader"})

    async def stop(self) -> None:
        self._running = False
        for w in self._workers:
            w.cancel()
        for w in self._workers:
            try:
                await w
            except asyncio.CancelledError:
                pass
        if self._exec:
            self._exec.shutdown(wait=False)

    # ── 任务编排 ──
    def object_key(self, bag_id: str, date: str, rel: str) -> str:
        return "bags/%s/%s/%s/%s/%s" % (self.bucket_version, self.cfg.vehicle_id,
                                        date, bag_id, rel)

    def enqueue_segment(self, seg: Dict[str, Any], manifest_path: Optional[str]) -> int:
        """把一个片段的可上传对象入队。返回入队对象数（用于 done 计数门控）。

        presign 模式：整个片段目录+manifest **打包成单个 tar 归档**，入队**一个** job（一轮），
        打包/三元组延后到 worker 中在线程池执行；返回 1。
        降级模式（s3/gateway）：manifest(P0) + 原始片段内所有文件(P4)，逐文件入队。
        """
        bag_id = seg["bag_id"]
        seg_dir = seg["path"]
        retention = "event" if int(seg.get("protected", 0) or 0) == 1 else "regular"
        if self.presign_mode:
            return self._enqueue_presign_archive(seg, manifest_path, retention)
        date = datetime.utcfromtimestamp(seg.get("start_ns", 0) / 1e9 or time.time()
                                         ).strftime("%Y-%m-%d")
        meta_common = {"bag-id": bag_id, "agent-version": self.cfg.agent_version,
                       "vehicle-id": self.cfg.vehicle_id}
        jobs: List[Tuple[int, str, str, str]] = []   # (priority, local, rel_key, class)
        if manifest_path and os.path.isfile(manifest_path):
            jobs.append((0, manifest_path, "manifest.json", "manifest"))
        for root, _d, files in os.walk(seg_dir):
            for f in sorted(files):
                fp = os.path.join(root, f)
                rel = "raw/" + os.path.relpath(fp, seg_dir).replace(os.sep, "/")
                jobs.append((4, fp, rel, "raw"))
        count = 0
        for prio, local, rel, cls in jobs:
            if not os.path.isfile(local):
                continue
            self._seq += 1
            size = os.path.getsize(local)
            try:
                file_hash = sha256_file(local)
            except OSError as exc:
                logger.warning("跳过不可读文件 %s：%s", local, exc)
                continue
            md = dict(meta_common)
            md["sha256"] = file_hash
            job = UploadJob(prio, self._seq, bag_id, local,
                            self.object_key(bag_id, date, rel), cls, file_hash, size, md,
                            retention=retention)
            self._inflight[bag_id] = self._inflight.get(bag_id, 0) + 1
            self._queue.put_nowait(job)
            count += 1
        logger.info("片段入队 %d 个对象 bag_id=%s", count, bag_id,
                    extra={"bag_id": bag_id, "component": "uploader"})
        return count

    def _enqueue_presign_archive(self, seg: Dict[str, Any], manifest_path: Optional[str],
                                 retention: str) -> int:
        """presign 模式：为单袋归档入队一个 job（打包延后）；返回 1。"""
        bag_id = seg["bag_id"]
        self._seq += 1
        archive_path = os.path.join(self.staging_dir, bag_id + ".tar")
        job = UploadJob(
            int(seg.get("priority", 4) or 4), self._seq, bag_id, archive_path,
            "",  # object_key 待 presign 回填
            "rosbag_archive", "", 0, {},
            retention=retention, seg_dir=seg["path"],
            manifest_path=manifest_path if manifest_path and os.path.isfile(manifest_path) else None)
        self._inflight[bag_id] = self._inflight.get(bag_id, 0) + 1
        self._queue.put_nowait(job)
        logger.info("presign 归档入队 bag_id=%s retention=%s", bag_id, retention,
                    extra={"bag_id": bag_id, "component": "uploader"})
        return 1

    async def _worker(self, idx: int) -> None:
        loop = asyncio.get_running_loop()
        while self._running:
            try:
                job = await asyncio.wait_for(self._queue.get(), timeout=1.0)
            except asyncio.TimeoutError:
                self._maybe_adjust_rate()
                continue
            except asyncio.CancelledError:
                break
            try:
                await self._upload_job(loop, job)
            except Exception as exc:  # noqa: BLE001  单对象失败不影响 worker 存活
                await self._handle_failure(loop, job, exc)
            finally:
                self._queue.task_done()

    def _maybe_adjust_rate(self) -> None:
        self._bucket_rate.set_rate(self._current_rate_bps())   # 跨时间窗动态限速

    async def _upload_job(self, loop, job: UploadJob) -> None:
        if self.presign_mode:
            await self._upload_via_presign(loop, job)
            return
        backend = self._backend
        # 断点：已存在且大小一致 → 视为完成（幂等，object_key 唯一，§8）
        remote = await loop.run_in_executor(self._exec, backend.head_size, job.object_key)
        if remote is not None and remote == job.size:
            self._mark_object_done(job, job.sha256, "resume-head", 0)
            return
        t0 = time.monotonic()
        if job.size <= self.part_size or getattr(backend, "initiate", None) is None:
            etag = await self._put_small(loop, job)
        else:
            etag = await self._put_multipart(loop, job)
        duration_ms = int((time.monotonic() - t0) * 1000)
        self._mark_object_done(job, job.sha256, etag, duration_ms)

    async def _put_small(self, loop, job: UploadJob) -> str:
        with open(job.local_path, "rb") as fh:
            data = fh.read()
        await self._bucket_rate.acquire(len(data))
        return await loop.run_in_executor(
            self._exec, self._backend.put_small, job.object_key, data, job.metadata)

    async def _put_multipart(self, loop, job: UploadJob) -> str:
        state_path = self._part_state_path(job.object_key)
        state = self._load_part_state(state_path)
        upload_id = state.get("upload_id")
        parts = state.get("parts", [])
        done_part_nos = {p["part_no"] for p in parts}
        if not upload_id:
            upload_id = await loop.run_in_executor(
                self._exec, self._backend.initiate, job.object_key, job.metadata)
            state = {"upload_id": upload_id, "parts": parts, "created_at": time.time()}
            self._save_part_state(state_path, state)
        part_no = 0
        with open(job.local_path, "rb") as fh:
            while True:
                chunk = fh.read(self.part_size)
                if not chunk:
                    break
                part_no += 1
                if part_no in done_part_nos:
                    continue
                await self._bucket_rate.acquire(len(chunk))
                etag = await loop.run_in_executor(
                    self._exec, self._backend.upload_part, job.object_key, upload_id,
                    part_no, chunk)
                parts.append({"part_no": part_no, "etag": etag})
                done_part_nos.add(part_no)
                self._save_part_state(state_path, state)
        etag = await loop.run_in_executor(
            self._exec, self._backend.complete, job.object_key, upload_id, parts)
        self._remove_part_state(state_path)
        return etag

    # ── presign 模式：每 bag 一轮 presign → PUT → complete ──
    async def _upload_via_presign(self, loop, job: UploadJob) -> None:
        client = self._presign_client
        archive = job.local_path
        state_path = self._presign_state_path(job.bag_id)
        state = self._load_presign_state(state_path)
        t0 = time.monotonic()

        # 1) 打包 + 三元组：命中续传（state 有三元组且归档在且尺寸一致）→ 复用，不重打包
        reusable = (state.get("size") and state.get("md5") and state.get("sha256")
                    and os.path.isfile(archive)
                    and os.path.getsize(archive) == state.get("size"))
        if reusable:
            job.size, job.md5, job.sha256 = state["size"], state["md5"], state["sha256"]
        else:
            size, md5, sha = await loop.run_in_executor(
                self._exec, pack_and_hash, job.seg_dir, job.manifest_path, archive)
            job.size, job.md5, job.sha256 = size, md5, sha
            state = {}   # 重打包 → 旧 presign/parts 作废

        # 2) presign（命中且未过期 → 复用 object_key/method/upload_url/upload_id/parts）
        presign_data = state.get("presign")
        if not (presign_data and time.time() - state.get("presign_ts", 0) < self.deadline_s):
            part_count = max(1, math.ceil(job.size / self.part_size))
            file_name = "%s.tar" % job.bag_id
            presign_data = await loop.run_in_executor(
                self._exec, client.presign, file_name, job.size, part_count, job.retention)
            state = {"presign": presign_data, "presign_ts": time.time(),
                     "size": job.size, "md5": job.md5, "sha256": job.sha256,
                     "done_parts": []}
            self._save_presign_state(state_path, state)
        job.object_key = presign_data.get("object_key", "") or job.object_key
        method = presign_data.get("method")

        # 3) PUT（单次带声明 Content-Type；分片逐片 PUT 收 ETag）
        if method == "put" or presign_data.get("upload_url"):
            await loop.run_in_executor(
                self._exec, client.put_single, presign_data["upload_url"], archive)
            is_multipart = False
        else:
            is_multipart = True
            part_urls = presign_data.get("parts") or []
            done = {p["part_number"] for p in state.get("done_parts", [])}
            with open(archive, "rb") as fh:
                for meta in part_urls:
                    pno = meta["part_number"]
                    if pno in done:
                        continue
                    fh.seek((pno - 1) * self.part_size)
                    chunk = fh.read(self.part_size)
                    if not chunk:
                        continue
                    await self._bucket_rate.acquire(len(chunk))
                    etag = await loop.run_in_executor(
                        self._exec, client.put_part, meta["upload_url"], chunk)
                    state["done_parts"].append({"part_number": pno, "etag": etag})
                    self._save_presign_state(state_path, state)

        # 4) complete（回带三元组 + retention；multipart 另带 upload_id+parts）
        if is_multipart:
            await loop.run_in_executor(
                self._exec, client.complete, job.object_key, job.size, job.md5, job.sha256,
                job.retention, presign_data.get("upload_id"), state.get("done_parts") or [])
        else:
            await loop.run_in_executor(
                self._exec, client.complete, job.object_key, job.size, job.md5, job.sha256,
                job.retention)

        # verified：删 part-state + 删 staging 归档 → 标记完成
        self._remove_presign_state(state_path)
        self._remove_archive(archive)
        duration_ms = int((time.monotonic() - t0) * 1000)
        self._mark_object_done(job, job.sha256, "verified", duration_ms)

    # ── presign part-state 持久化（键 bag_id）──
    def _presign_state_path(self, bag_id: str) -> str:
        safe = bag_id.replace("/", "_")
        return os.path.join(self.part_state_dir, "presign_%s.json" % safe)

    def _load_presign_state(self, path: str) -> Dict[str, Any]:
        if not os.path.isfile(path):
            return {}
        try:
            with open(path, "r", encoding="utf-8") as fh:
                return json.load(fh)
        except (OSError, ValueError):
            return {}

    def _save_presign_state(self, path: str, state: Dict[str, Any]) -> None:
        try:
            with open(path, "w", encoding="utf-8") as fh:
                json.dump(state, fh)
        except OSError as exc:
            logger.warning("presign part-state 写入失败 %s：%s", path, exc)

    def _remove_presign_state(self, path: str) -> None:
        try:
            if os.path.isfile(path):
                os.remove(path)
        except OSError:
            pass

    def _remove_archive(self, path: str) -> None:
        try:
            if os.path.isfile(path):
                os.remove(path)
        except OSError as exc:
            logger.warning("staging 归档删除失败 %s：%s", path, exc)

    def _mark_object_done(self, job: UploadJob, sha256: str, etag: str,
                          duration_ms: int) -> None:
        logger.info("对象上传完成 key=%s class=%s size=%d", job.object_key,
                    job.artifact_class, job.size,
                    extra={"bag_id": job.bag_id, "component": "uploader"})
        self._inflight[job.bag_id] = max(0, self._inflight.get(job.bag_id, 0) - 1)
        if self._done_cb:
            self._done_cb(job.bag_id, job, etag, duration_ms)
        if self._inflight[job.bag_id] == 0:
            logger.info("片段全部对象上传完成 bag_id=%s", job.bag_id,
                        extra={"bag_id": job.bag_id, "component": "uploader"})

    async def _handle_failure(self, loop, job: UploadJob, exc: Exception) -> None:
        if self.presign_mode:
            await self._handle_presign_failure(job, exc)
            return
        job.attempt += 1
        if job.attempt > self.max_attempts:
            logger.error("对象上传超限 bag_id=%s key=%s：%s", job.bag_id,
                         job.object_key, exc, extra={"bag_id": job.bag_id,
                                                     "component": "uploader"})
            self._inflight[job.bag_id] = max(0, self._inflight.get(job.bag_id, 0) - 1)
            if self._fail_cb:
                self._fail_cb(job.bag_id, exc)
            return
        delay = min(self.backoff_max, self.backoff_base * (2 ** (job.attempt - 1)))
        delay += random.uniform(0, delay * 0.2)          # jitter 防惊群
        logger.warning("上传重试 %d/%d（%.1fs 后）：%s", job.attempt, self.max_attempts,
                       delay, exc, extra={"bag_id": job.bag_id, "component": "uploader"})
        await asyncio.sleep(delay)
        self._inflight[job.bag_id] = self._inflight.get(job.bag_id, 0) + 1  # 仍在途
        self._queue.put_nowait(job)

    async def _handle_presign_failure(self, job: UploadJob, exc: Exception) -> None:
        """presign 重试矩阵（方案 §4 重试分流）。

        - PermanentDeny(1002)/BadParams(2001)/TLSAuthFailed(1001) → 不重试，置 upload_failed（属配置/证书/平台侧）；
        - RateLimited(429) → 按 Retry-After 退避重试，不占 attempt 预算；
        - IntegrityMismatch(6001) → 作废重传：删 state + 删归档 → 重打包重算（attempt+1）；
        - NotFound(3001) → 强制重新 presign（清 presign 保留归档，重传）；
        - ServerBusy(5001) → 保留 state（对象已在），退避重试 complete；
        - 网络/超时（其余 PresignError）→ 指数退避（retry_backoff_*）。
        """
        bag_id = job.bag_id
        state_path = self._presign_state_path(bag_id)
        if isinstance(exc, (PermanentDeny, BadParams, TLSAuthFailed)):
            logger.error("presign 不可重试（配置/证书/平台侧）bag_id=%s：%s", bag_id, exc,
                         extra={"bag_id": bag_id, "component": "uploader"})
            self._inflight[bag_id] = max(0, self._inflight.get(bag_id, 0) - 1)
            if self._fail_cb:
                self._fail_cb(bag_id, exc)
            return
        if isinstance(exc, RateLimited):
            logger.warning("presign 限流 429，%.0fs 后重试（不占 attempt）bag_id=%s",
                           exc.retry_after, bag_id,
                           extra={"bag_id": bag_id, "component": "uploader"})
            await asyncio.sleep(exc.retry_after)
            self._queue.put_nowait(job)      # inflight 不变（仍在途）
            return
        if isinstance(exc, IntegrityMismatch):
            self._remove_presign_state(state_path)      # 作废 → 重打包重算
            self._remove_archive(job.local_path)
            job.object_key = ""
        elif isinstance(exc, NotFound):
            self._remove_presign_state(state_path)      # 强制重新 presign（保留归档）
            job.object_key = ""
        job.attempt += 1
        if job.attempt > self.max_attempts:
            logger.error("presign 上传超限 bag_id=%s key=%s：%s", bag_id, job.object_key, exc,
                         extra={"bag_id": bag_id, "component": "uploader"})
            self._inflight[bag_id] = max(0, self._inflight.get(bag_id, 0) - 1)
            if self._fail_cb:
                self._fail_cb(bag_id, exc)
            return
        delay = min(self.backoff_max, self.backoff_base * (2 ** (job.attempt - 1)))
        delay += random.uniform(0, delay * 0.2)
        logger.warning("presign 重试 %d/%d（%.1fs 后）：%s", job.attempt, self.max_attempts,
                       delay, exc, extra={"bag_id": bag_id, "component": "uploader"})
        await asyncio.sleep(delay)
        self._queue.put_nowait(job)

    # ── part-state 持久化（§6.7 断点续传）──
    def _part_state_path(self, object_key: str) -> str:
        safe = object_key.replace("/", "_")
        return os.path.join(self.part_state_dir, safe + ".json")

    def _load_part_state(self, path: str) -> Dict[str, Any]:
        if not os.path.isfile(path):
            return {}
        try:
            with open(path, "r", encoding="utf-8") as fh:
                st = json.load(fh)
            # upload_id 过期(>7d) → 丢弃重传（§6.7）
            if time.time() - st.get("created_at", 0) > 7 * 86400:
                self._remove_part_state(path)
                return {}
            return st
        except (OSError, ValueError):
            return {}

    def _save_part_state(self, path: str, state: Dict[str, Any]) -> None:
        try:
            with open(path, "w", encoding="utf-8") as fh:
                json.dump(state, fh)
        except OSError as exc:
            logger.warning("part-state 写入失败 %s：%s", path, exc)

    def _remove_part_state(self, path: str) -> None:
        try:
            if os.path.isfile(path):
                os.remove(path)
        except OSError:
            pass
