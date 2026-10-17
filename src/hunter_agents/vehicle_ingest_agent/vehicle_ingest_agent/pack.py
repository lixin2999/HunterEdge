# -*- coding: utf-8 -*-
"""片段打包 + 单次流式三元组（方案 V1.21.13 · 变更组 3）。

把封段后的 rosbag2 目录（``metadata.yaml`` + ``*.db3``）连同 ``manifest.json`` 打包成**单个 tar 归档**，
在写归档的同一次遍历中，经 tee 包装把每块字节同时喂给 ``md5`` / ``sha256`` 与 size 计数，
产出 presign 契约要的三元组 ``(size, md5_hex, sha256_hex)``——三者严格对应**归档的真实字节流**，
供 complete 服务端重算比对（方案 §4.1）。

口径说明：manifest 里跨 .db3 的组合 sha256（verifier.compute_sha256）另作它途，
与"整对象字节 sha256"不同；上传三元组独立计算，互不影响。
"""
from __future__ import annotations

import hashlib
import os
import tarfile
from typing import Optional, Tuple


class _HashingWriter:
    """把写入字节同时累积到 md5/sha256 与 size（顺序写，不 seek）。"""

    def __init__(self, fh, md5, sha):
        self._fh = fh
        self._md5 = md5
        self._sha = sha
        self.size = 0

    def writable(self) -> bool:
        return True

    def write(self, data: bytes) -> int:
        self._md5.update(data)
        self._sha.update(data)
        self.size += len(data)
        self._fh.write(data)
        return len(data)

    def tell(self) -> int:
        return self.size

    def seekable(self) -> bool:
        return False

    def flush(self) -> None:
        self._fh.flush()


def pack_and_hash(seg_dir: str, manifest_path: Optional[str],
                  dest_tar: str) -> Tuple[int, str, str]:
    """打包 ``seg_dir``（+ 可选 manifest）到 ``dest_tar``，返回 (size, md5_hex, sha256_hex)。

    对归档字节流式一次算出三元组；调用方（上传引擎）应在 ``run_in_executor`` 中执行，
    避免阻塞事件循环。归档已存在会被覆盖（重打包重算）。
    """
    if not os.path.isdir(seg_dir):
        raise FileNotFoundError(f"打包源目录不存在：{seg_dir}")
    os.makedirs(os.path.dirname(dest_tar), exist_ok=True)
    md5 = hashlib.md5()
    sha = hashlib.sha256()
    with open(dest_tar, "wb") as raw:
        writer = _HashingWriter(raw, md5, sha)
        with tarfile.open(fileobj=writer, mode="w") as tf:
            for root, _dirs, files in os.walk(seg_dir):
                for name in sorted(files):
                    fp = os.path.join(root, name)
                    if not os.path.isfile(fp):
                        continue
                    arcname = os.path.relpath(fp, seg_dir).replace(os.sep, "/")
                    try:
                        tf.add(fp, arcname=arcname, recursive=False)
                    except OSError:
                        continue
            if manifest_path and os.path.isfile(manifest_path):
                tf.add(manifest_path, arcname="manifest.json", recursive=False)
        writer.flush()
    return writer.size, md5.hexdigest(), sha.hexdigest()
