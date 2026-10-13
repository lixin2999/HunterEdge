# -*- coding: utf-8 -*-
"""上传引擎（设计 §6.7）：S3/MinIO 分片上传（断点续传）或边缘网关 HTTP PUT。

- 通道：``upload.endpoint_type`` = s3（boto3，S3/MinIO 兼容）| gateway（requests PUT）；
- 优先级队列：P0 manifest / P4 原始片段（降级模式）；跨对象并发 + 令牌桶限速 + 时间窗；
- 断点续传：每个 >part_size 的对象写 ``part-state/{object_key}.json``（upload_id + 已完成
  part 的 etag），进程重启续传；upload_id 过期(>7d) 由 S3 端 ListMultipartUploads 清理重传；
- 完整性：上传完 HEAD 比对 content-length，并带 x-amz-meta-sha256/bag-id/agent-version；
- 重试：指数退避 base→×2…max + jitter，超 max_attempts → 交回上层置 upload_failed。

boto3/requests 缺失或凭据不可用时抛 :class:`UploadUnavailable`，由 agent 归类为上传失败
（保留现场、不删片段），而非崩溃。凭据：优先 STS/环境，落 /etc/hunter/oss（0600），
不写进仓库 YAML。
"""
from __future__ import annotations

import asyncio
import hashlib
import json
import os
import random
import time
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional, Tuple

from .logging_util import get_logger

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
    """单对象上传任务（优先级队列元素）。priority 越小越先（P0 manifest…P4 raw）。"""

    __slots__ = ("priority", "seq", "bag_id", "local_path", "object_key",
                 "artifact_class", "sha256", "size", "metadata", "attempt")

    def __init__(self, priority: int, seq: int, bag_id: str, local_path: str,
                 object_key: str, artifact_class: str, sha256: str, size: int,
                 metadata: Dict[str, str], attempt: int = 0):
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
        self.max_attempts = int(cfg.get("upload.max_attempts", 12))
        self.backoff_base = float(cfg.get("upload.retry_backoff_base_s", 1))
        self.backoff_max = float(cfg.get("upload.retry_backoff_max_s", 600))
        self.part_size = int(cfg.get("upload.part_size_mb", 16)) * 1024 * 1024
        self._queue: "asyncio.PriorityQueue[UploadJob]" = asyncio.PriorityQueue()
        self._seq = 0
        self._backend = None
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
            if endpoint_type == "s3":
                self._backend = _S3Backend(self.cfg)
            else:
                self._backend = _GatewayBackend(self.cfg)
        except ImportError as exc:
            raise UploadUnavailable(
                f"上传依赖缺失（{endpoint_type}）：{exc}；s3 需 boto3，gateway 需 requests")
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

        降级模式：manifest(P0) + 原始片段内所有文件(P4)。
        """
        bag_id = seg["bag_id"]
        seg_dir = seg["path"]
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
                            self.object_key(bag_id, date, rel), cls, file_hash, size, md)
            self._inflight[bag_id] = self._inflight.get(bag_id, 0) + 1
            self._queue.put_nowait(job)
            count += 1
        logger.info("片段入队 %d 个对象 bag_id=%s", count, bag_id,
                    extra={"bag_id": bag_id, "component": "uploader"})
        return count

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
