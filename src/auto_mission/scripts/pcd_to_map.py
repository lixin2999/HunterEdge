#!/usr/bin/env python3
# Copyright 2026 HUNTER Development Team
# pcd_to_map — PCD 三维点云自动转换为 Nav2 2D 占据栅格节点
#
# 功能：
#   1. 订阅 /auto_mission/status，检测状态由 MAPPING → 其他（建图结束自动触发）
#   2. 提供 /pcd_to_map/convert 服务，支持手动触发转换
#   3. 读取 FAST-LIO2 输出的 .pcd 文件（ASCII / Binary 格式）
#   4. 按 z_min ~ z_max 范围切片提取障碍物层，投影为 2D 占据栅格
#   5. 输出 .pgm（灰度图）+ .yaml（Nav2 地图元数据）
#   6. 转换完成后自动调用 /map_server/load_map 服务热重载地图
#   7. 发布 /pcd_to_map/status 话题上报状态：IDLE / CONVERTING / DONE / ERROR
#   8. 退出兜底转换（V0.0.88）：建图模式 Ctrl+C 时 FAST-LIO2 才在 main() 中
#      （rclcpp::spin 返回之后）把累积点云写盘（实测 20.7M 点 ≈ 664MB，需数秒
#      至数十秒），而 Ctrl+C 会把 auto_mission/pcd_to_map 与之一并终止——
#      运行期 MAPPING→非MAPPING 跳变永远不会发生，自动转换从不启动，
#      于是 maps/ 只剩 .pcd（.pgm/.yaml 缺失）。为此本节点在自己的退出路径上
#      派生一个**独立会话（setsid）**的后台转换进程：它不受 launch 的
#      SIGINT/SIGTERM/SIGKILL 与终端 Ctrl+C 进程组信号影响，等待 PCD 写完整
#      （依据 header 中 POINTS/WIDTH×HEIGHT 与字段行宽推算的字节数校验）后
#      完成转换，进度写入日志文件。
#   9. 截断容错（V0.0.90）：若写盘方中途被杀（实测 3.8 亿点 ≈ 6.1GB 的地图
#      在 launch 默认 5s 宽限内被 SIGTERM 腰斩），文件大小永远达不到
#      header 推算值——旧逻辑会死等到超时后直接放弃，maps/ 仍只剩 .pcd。
#      现在：① 大小连续多次采样不再变化即视为写盘已终止（截断图）；
#      ② 等待超时但若文件非空，仍按已写入部分尽力转换（解析器本就容忍
#      截断：按可用字节数解析），产物为“部分地图”并在日志/返回值中注明。
#      根因修复见 fast_lio2 pcd_save.save_voxel_size 体素去重与
#      localization.launch.py 退出宽限（同为 V0.0.90）。
#   10. 退出链加固（V0.0.90）：rclpy.spin 改为 0.3s 分时 spin_once + 兜底
#      派生后 os._exit——修复实机“SIGINT 后 15s 不退被 SIGKILL”的信号唤醒
#      挂死（cyclonedds waitset 阻塞 C 层时 Python 信号异常不被执行），
#      确保退出兜底派生机会稳定到达。
#
# 参数：
#   pcd_file        : PCD 文件绝对路径（默认 /home/agilex/HunterEdge/maps/hunter_map.pcd）
#   map_output_dir  : PGM/YAML 输出目录（默认与 pcd_file 同目录）
#   map_name        : 输出文件名前缀（默认 hunter_map）
#   resolution      : 栅格分辨率，m/pixel（默认 0.05 m，与 local_costmap 一致）
#   z_min           : 障碍物 z 轴下限，m（默认 0.1，过滤地面）
#   z_max           : 障碍物 z 轴上限，m（默认 2.0，过滤天花板/树冠）
#   occupied_thresh : 栅格占据概率阈值（默认 0.65）
#   free_thresh     : 栅格空闲概率阈值（默认 0.25）
#   padding_m       : 地图四周填充边距，m（默认 0.5）
#   auto_reload_map : 转换完成后是否自动调用 map_server 重载（默认 true）
#   trigger_on_mapping_end : 检测到建图结束后自动触发转换（默认 true）
#   convert_on_start_if_missing : 启动自愈——YAML 缺失而 PCD 存在时自动补转换（默认 true）
#   final_convert_on_shutdown : 退出兜底转换（建图模式置 true；默认 false）
#   final_wait_timeout        : 退出兜底等待 PCD 写盘完成的最长时间，s（默认 120.0）
#   final_log_file            : 退出兜底转换日志文件（默认 map_output_dir/pcd_to_map_final.log）
#
# 离线 / 后台转换（无需 ROS 环境，与节点模式共用同一套转换管线）：
#   python3 pcd_to_map.py --finalize --pcd-file <pcd> --map-output-dir <dir> \
#       --map-name hunter_map [--resolution 0.05] [--z-min 0.1] [--z-max 2.0] \
#       [--occupied-thresh 0.65] [--free-thresh 0.25] [--padding-m 0.5] \
#       [--wait-timeout 120] [--force]

# 延迟注解求值（PEP 563）：ROS 类型注解（Trigger.Request 等）不在导入期求值，
# 从而可在无 rclpy 的环境导入本模块（--finalize 后台/离线转换依赖此点）
from __future__ import annotations

import argparse
import math
import os
import signal
import struct
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Callable, Optional, Tuple

import numpy as np

# ROS2 依赖仅在节点模式下需要：--finalize 后台进程与离线转换不需要 rclpy
# （可脱离 ROS 环境直接转换 PCD，也便于离线/单元测试调试）
try:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    _ROS_AVAILABLE = True
except ImportError:  # pragma: no cover - 无 ROS 环境（如开发机离线转换）
    rclpy = None
    Node = object
    String = None
    Trigger = None
    _ROS_AVAILABLE = False

    class ExternalShutdownException(Exception):
        """rclpy 不可用时的占位异常（main 中统一按"外部请求关闭"处理）。"""


# ---------------------------------------------------------------------------
# 纯 Python PCD 解析（支持 ASCII / Binary，不依赖 libpcl）
# ---------------------------------------------------------------------------
# PCD 字段类型 → numpy dtype / struct 格式符（用于 binary 快路径与行宽推算）
_NP_TYPE_MAP = {'F': {4: '<f4', 8: '<f8'},
                'I': {1: 'i1', 2: '<i2', 4: '<i4', 8: '<i8'},
                'U': {1: 'u1', 2: '<u2', 4: '<u4', 8: '<u8'}}
_STRUCT_FMT_MAP = {'F': {4: 'f', 8: 'd'},
                   'I': {1: 'b', 2: 'h', 4: 'i', 8: 'q'},
                   'U': {1: 'B', 2: 'H', 4: 'I', 8: 'Q'}}
# header 与数据段的交界标记（含前导换行，故 marker 之前的字节不含 DATA 行本身）
_HEADER_END_MARKER = b'\nDATA '
# header 探测长度：PCD header 仅数百字节，用于只读文件头判断写盘是否完整
_HEADER_PROBE_BYTES = 8192


class PcdParser:
    """最小化 PCD 解析器，只提取 x y z 坐标。"""

    # ------------------------------------------------------------------
    # header 解析（可只传文件前若干 KB）
    # ------------------------------------------------------------------
    @staticmethod
    def read_header(raw: bytes) -> dict:
        """解析 PCD header，返回字段与数据段信息。

        注意：marker 含前导换行，因此 marker 之前的字节不包含 DATA 行本身。
        旧版实现由此永远读不到 DATA 行，data_type 恒为默认 'ascii'，导致
        FAST-LIO2 writeBinary 输出的 binary PCD 被按 ASCII 解析出 0 个点，
        建图结束的自动转换必然失败（maps/ 下只有 .pcd）。

        返回 dict：
            fields/sizes/types/counts : FIELDS / SIZE / TYPE / COUNT
            data_type   : 'ascii' 或 'binary'
            n_points    : POINTS（缺省时取 WIDTH×HEIGHT）
            data_offset : 数据段起始字节偏移
        """
        idx = raw.find(_HEADER_END_MARKER)
        if idx == -1:
            raise ValueError('无效 PCD 文件：找不到 DATA 字段')

        header_str = raw[:idx].decode('ascii', errors='replace')
        data_type = ''
        fields: list = []
        sizes:  list = []
        types:  list = []
        counts: list = []
        n_points = 0
        width = 0
        height = 0

        for line in header_str.splitlines():
            tok = line.strip().split()
            if not tok:
                continue
            key = tok[0].upper()
            if key == 'FIELDS':
                fields = tok[1:]
            elif key == 'SIZE':
                sizes = [int(s) for s in tok[1:]]
            elif key == 'TYPE':
                types = tok[1:]
            elif key == 'COUNT':
                counts = [int(c) for c in tok[1:]]
            elif key == 'WIDTH':
                width = int(tok[1])
            elif key == 'HEIGHT':
                height = int(tok[1])
            elif key == 'POINTS':
                n_points = int(tok[1])
            elif key == 'DATA':
                data_type = tok[1].lower()

        # DATA 行位于 marker 之后，从原始字节补解析（strip 兼容 CRLF）
        if not data_type:
            data_line = raw[idx + 1:idx + 48].split(b'\n')[0].decode(
                'ascii', errors='replace').strip()
            if data_line.startswith('DATA'):
                parts = data_line.split()
                if len(parts) >= 2:
                    data_type = parts[1].lower()
        if data_type not in ('ascii', 'binary'):
            raise ValueError(
                f'不支持的 PCD 数据格式：{data_type or "未知"}（需要 ascii 或 binary）')

        if not counts:
            counts = [1] * len(fields)
        if n_points <= 0:
            n_points = width * height

        return {
            'fields': fields,
            'sizes': sizes,
            'types': types,
            'counts': counts,
            'data_type': data_type,
            'n_points': n_points,
            'data_offset': idx + len(_HEADER_END_MARKER) + len(data_type) + 1,
        }

    # ------------------------------------------------------------------
    # 写盘完整性判定（退出兜底等待 FAST-LIO2 写盘结束时使用）
    # ------------------------------------------------------------------
    @staticmethod
    def row_size(sizes: list, types: list, counts: list) -> Optional[int]:
        """单个点（一行）占用的字节数；含未知 TYPE/SIZE 组合时返回 None。"""
        total = 0
        for t, s, c in zip(types, sizes, counts):
            fmt = _STRUCT_FMT_MAP.get(t, {}).get(s)
            if fmt is None:
                return None
            total += struct.calcsize('<' + fmt * max(int(c), 1))
        return total

    @staticmethod
    def expected_file_size(filepath: str) -> Optional[int]:
        """按 header 推算 PCD 写完整后的文件字节数。

        只读文件头（8KB），不解析点云；ASCII PCD（无固定行宽）或字段类型
        未知时返回 None，调用方退化为"文件大小稳定"判定。
        """
        try:
            with open(filepath, 'rb') as f:
                head = f.read(_HEADER_PROBE_BYTES)
        except OSError:
            return None
        try:
            info = PcdParser.read_header(head)
        except (ValueError, IndexError, UnicodeDecodeError):
            return None
        if info['data_type'] != 'binary' or info['n_points'] <= 0:
            return None
        row = PcdParser.row_size(info['sizes'], info['types'], info['counts'])
        if not row:
            return None
        return info['data_offset'] + info['n_points'] * row

    # ------------------------------------------------------------------
    # 点云加载
    # ------------------------------------------------------------------
    @staticmethod
    def load(filepath: str) -> np.ndarray:
        """返回 shape=(N,3) float32 ndarray，列为 (x, y, z)。"""
        with open(filepath, 'rb') as f:
            raw = f.read()

        info = PcdParser.read_header(raw)
        fields = info['fields']
        counts = info['counts']
        n_points = info['n_points']

        # ---- 找到 x y z 索引 ----
        try:
            xi = fields.index('x')
            yi = fields.index('y')
            zi = fields.index('z')
        except ValueError:
            raise ValueError(f'PCD 缺少 x/y/z 字段，实际字段：{fields}')

        if info['data_type'] == 'ascii':
            return PcdParser._parse_ascii(
                raw[info['data_offset']:], xi, yi, zi, n_points)

        # binary：优先 numpy 向量化解析（FAST-LIO2 writeBinary 输出常达 20M+
        # 点，逐点 struct.unpack 需数十秒且产生 GB 级临时元组），不适用再回退
        pts = PcdParser._parse_binary_numpy(raw, info, xi, yi, zi)
        if pts is not None:
            return pts
        return PcdParser._parse_binary(
            raw[info['data_offset']:], fields, info['sizes'], info['types'],
            counts, xi, yi, zi, n_points)

    @staticmethod
    def _parse_binary_numpy(raw: bytes, info: dict,
                            xi: int, yi: int, zi: int) -> Optional[np.ndarray]:
        """numpy 结构化 dtype 向量化解析 binary PCD；格式不适用时返回 None。"""
        fields, sizes = info['fields'], info['sizes']
        types, counts = info['types'], info['counts']
        n_points = info['n_points']
        if not (len(fields) == len(sizes) == len(types) == len(counts)):
            return None
        if any(int(c) != 1 for c in counts):
            return None            # COUNT>1（多值字段）交由逐点回退实现

        dtype_fields = []
        for name, t, s in zip(fields, types, sizes):
            np_type = _NP_TYPE_MAP.get(t, {}).get(s)
            if np_type is None:
                return None
            dtype_fields.append((name, np_type))

        # 数据段实际可用字节数（FAST-LIO2 退出瞬间可能只写完一部分）
        row = PcdParser.row_size(sizes, types, counts)
        if not row:
            return None
        avail = max(len(raw) - info['data_offset'], 0)
        count = min(int(n_points), avail // row)
        if n_points > 0 and count <= 0:
            return None

        try:
            arr = np.frombuffer(raw, dtype=np.dtype(dtype_fields),
                                count=count, offset=info['data_offset'])
        except (ValueError, TypeError):
            return None
        if arr.size == 0:
            return np.empty((0, 3), dtype=np.float32)
        pts = np.empty((arr.size, 3), dtype=np.float32)
        pts[:, 0] = arr[fields[xi]]
        pts[:, 1] = arr[fields[yi]]
        pts[:, 2] = arr[fields[zi]]
        return pts

    @staticmethod
    def _parse_ascii(data: bytes, xi: int, yi: int, zi: int,
                     n_points: int) -> np.ndarray:
        lines = data.decode('ascii', errors='replace').splitlines()
        pts = []
        for line in lines:
            tok = line.strip().split()
            if len(tok) <= max(xi, yi, zi):
                continue
            try:
                pts.append((float(tok[xi]), float(tok[yi]), float(tok[zi])))
            except ValueError:
                continue
        return np.array(pts, dtype=np.float32) if pts else np.empty((0, 3), dtype=np.float32)

    @staticmethod
    def _parse_binary(data: bytes, fields: list, sizes: list, types: list,
                      counts: list, xi: int, yi: int, zi: int,
                      n_points: int) -> np.ndarray:
        # 构造每个字段的 struct 格式符
        fmt_map = {'F': {4: 'f', 8: 'd'}, 'I': {1: 'b', 2: 'h', 4: 'i', 8: 'q'},
                   'U': {1: 'B', 2: 'H', 4: 'I', 8: 'Q'}}

        field_fmts = []
        for t, s, c in zip(types, sizes, counts):
            fmt_char = fmt_map.get(t, {}).get(s, 'x' * s)
            field_fmts.append(fmt_char * c)

        row_fmt  = '<' + ''.join(field_fmts)
        row_size = struct.calcsize(row_fmt)

        pts = []
        for i in range(n_points):
            offset = i * row_size
            if offset + row_size > len(data):
                break
            row = struct.unpack_from(row_fmt, data, offset)
            # 字段偏移计算：field i 对应 row 中 sum(counts[:i]) 的位置
            offsets = []
            acc = 0
            for c in counts:
                offsets.append(acc)
                acc += c
            pts.append((row[offsets[xi]], row[offsets[yi]], row[offsets[zi]]))

        return np.array(pts, dtype=np.float32) if pts else np.empty((0, 3), dtype=np.float32)


# ---------------------------------------------------------------------------
# 2D 占据栅格生成
# ---------------------------------------------------------------------------
def pcd_to_occupancy_grid(
    points: np.ndarray,
    resolution: float,
    z_min: float,
    z_max: float,
    padding_m: float,
    occupied_thresh: float,
) -> Tuple[np.ndarray, float, float]:
    """将三维点云切片投影为 2D 占据栅格。

    返回：
        grid   : shape=(H,W) uint8，255=空闲，0=占据，205=未知
        origin_x : 栅格原点 x 坐标（map 帧，左下角）
        origin_y : 栅格原点 y 坐标（map 帧，左下角）
    """
    UNKNOWN  = 205
    FREE     = 254
    OCCUPIED = 0

    if points.shape[0] == 0:
        raise ValueError('点云为空，无法生成地图')

    # 按 z 轴范围过滤：只保留地面以上 z_min ~ z_max 的点
    mask = (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
    pts_2d = points[mask, :2]   # 只取 x, y

    if pts_2d.shape[0] == 0:
        raise ValueError(f'z 轴范围 [{z_min}, {z_max}] 内无有效点，请调整 z_min/z_max 参数')

    # 计算边界（含 padding）
    x_min = pts_2d[:, 0].min() - padding_m
    x_max = pts_2d[:, 0].max() + padding_m
    y_min = pts_2d[:, 1].min() - padding_m
    y_max = pts_2d[:, 1].max() + padding_m

    width  = int(math.ceil((x_max - x_min) / resolution))
    height = int(math.ceil((y_max - y_min) / resolution))

    # 初始化为未知
    grid = np.full((height, width), UNKNOWN, dtype=np.uint8)

    # 标记空闲：所有被扫描过的区域先标记为 FREE
    # 简化处理：在障碍物点周围 padding_m 范围内的格子标记为 FREE，其余维持 UNKNOWN
    # 更精确的做法需要 ray casting（超出本实现范围）
    # 这里使用一种实用的近似：将点云投影到栅格，有点的格子为 OCCUPIED，
    # 以 FAST-LIO2 建图覆盖的包络框为 FREE，未覆盖区域为 UNKNOWN。
    # 对室内/结构化场景效果良好。

    # 先将整个点云边界内（去掉 padding）标记为 FREE
    x_inner_min = pts_2d[:, 0].min()
    x_inner_max = pts_2d[:, 0].max()
    y_inner_min = pts_2d[:, 1].min()
    y_inner_max = pts_2d[:, 1].max()

    col_free_start = max(0, int((x_inner_min - x_min) / resolution))
    col_free_end   = min(width,  int(math.ceil((x_inner_max - x_min) / resolution)))
    row_free_start = max(0, int((y_inner_min - y_min) / resolution))
    row_free_end   = min(height, int(math.ceil((y_inner_max - y_min) / resolution)))

    grid[row_free_start:row_free_end, col_free_start:col_free_end] = FREE

    # 将点云投影为 OCCUPIED
    cols = ((pts_2d[:, 0] - x_min) / resolution).astype(np.int32)
    rows = ((pts_2d[:, 1] - y_min) / resolution).astype(np.int32)

    # 边界裁剪
    valid = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
    grid[rows[valid], cols[valid]] = OCCUPIED

    # pgm 约定：行从上到下对应 y 从大到小（北向上），需要翻转
    grid = np.flipud(grid)

    return grid, x_min, y_min


# ---------------------------------------------------------------------------
# 写出 PGM + YAML
# ---------------------------------------------------------------------------
def write_pgm_yaml(
    grid: np.ndarray,
    origin_x: float,
    origin_y: float,
    resolution: float,
    output_dir: str,
    map_name: str,
    occupied_thresh: float,
    free_thresh: float,
) -> Tuple[str, str]:
    """将栅格写出为 .pgm + .yaml，返回 (pgm_path, yaml_path)。

    先写临时文件再 os.replace 原子替换：即使转换进程被中途终止（例如退出
    兜底进程恰好被 SIGKILL），也不会留下"半个地图"（.pgm 完整而 .yaml 缺失
    会让 map_server 加载失败）。
    """
    os.makedirs(output_dir, exist_ok=True)

    pgm_path  = os.path.join(output_dir, f'{map_name}.pgm')
    yaml_path = os.path.join(output_dir, f'{map_name}.yaml')
    pgm_tmp   = f'{pgm_path}.tmp'
    yaml_tmp  = f'{yaml_path}.tmp'

    # ---- 写 PGM（P5 二进制灰度，8位） ----
    height, width = grid.shape
    with open(pgm_tmp, 'wb') as f:
        f.write(f'P5\n{width} {height}\n255\n'.encode('ascii'))
        f.write(grid.tobytes())
        f.flush()
        os.fsync(f.fileno())

    # ---- 写 YAML（Nav2 map_server 格式） ----
    yaml_content = (
        f'image: {os.path.basename(pgm_path)}\n'
        f'resolution: {resolution}\n'
        f'origin: [{origin_x:.6f}, {origin_y:.6f}, 0.0]\n'
        f'negate: 0\n'
        f'occupied_thresh: {occupied_thresh}\n'
        f'free_thresh: {free_thresh}\n'
    )
    with open(yaml_tmp, 'w', encoding='utf-8') as f:
        f.write(yaml_content)
        f.flush()
        os.fsync(f.fileno())

    # 先落 PGM、后落 YAML：YAML 是"转换已完成"的标志（退出兜底与启动自愈
    # 都以 YAML 与 PCD 的 mtime 比较判断是否还需要转换）
    os.replace(pgm_tmp, pgm_path)
    os.replace(yaml_tmp, yaml_path)

    return pgm_path, yaml_path


# ---------------------------------------------------------------------------
# 转换管线（ROS 节点 / --finalize 后台进程 / 离线调用共用）
# ---------------------------------------------------------------------------
def wait_for_complete_pcd(pcd_file: str, wait_timeout: float,
                          log: Callable[[str], None],
                          poll_s: float = 2.0,
                          stable_checks: int = 2) -> bool:
    """等待 FAST-LIO2 把 PCD 写盘完成（大图在退出瞬间需数秒~数十秒）。

    判定依据（按优先级）：
      1) header 推算的完整字节数（POINTS 或 WIDTH×HEIGHT 与字段行宽）已达标
         —— 可精确识别"写完整"；
      2) 无法推算时（ASCII PCD / 未知字段类型）退化为"文件大小连续
         stable_checks 次不变且非空"；
      3) V0.0.90：可推算但达不到推算值时，若大小也连续多轮不变，说明
         写盘方已终止（如 FAST-LIO2 写盘中途被 SIGTERM），按"截断图"
         接受，由调用方按已写入部分尽力转换。

    超时返回 False，由调用方决定如何提示（V0.0.90：文件非空时尽力转换）。
    """
    wait_timeout = max(float(wait_timeout), 0.0)
    started = time.time()
    deadline = started + wait_timeout
    prev_size = -1
    stable = 0
    next_report = started + 10.0

    while True:
        if os.path.isfile(pcd_file):
            size = os.path.getsize(pcd_file)
            expected = PcdParser.expected_file_size(pcd_file)
            if expected is not None and size >= expected:
                log(f'PCD 已写完整：{size / 1024 / 1024:.2f} MB'
                    f'（header 推算 {expected / 1024 / 1024:.2f} MB）')
                return True
            if size > 0:
                if size == prev_size:
                    stable += 1
                    # 截断接受（第 3 条）需要比 ASCII 稳定性判定多一轮观察，
                    # 避免大体积写盘间歇性停顿被误判为已终止
                    need_stable = (stable_checks if expected is None
                                   else max(stable_checks, 3))
                    if stable >= need_stable:
                        if expected is None:
                            log(f'PCD 文件大小稳定：{size / 1024 / 1024:.2f} MB')
                        else:
                            log(f'[警告] PCD 大小连续 {stable} 次采样不变'
                                f'（{size / 1024 / 1024:.2f} MB < 推算 '
                                f'{expected / 1024 / 1024:.2f} MB），写盘方疑已'
                                f'终止，按已写入点尽力转换')
                        return True
                else:
                    stable = 0
                prev_size = size
                if expected is None:
                    # ASCII/未知行宽 PCD 需要 stable_checks 次采样才能确认"写完"，
                    # 为其保证足够的观察窗口（binary 走上面的 header 精确判定，
                    # 无需额外等待）
                    deadline = max(deadline, time.time() + stable_checks * poll_s + 0.5)

        now = time.time()
        if now >= deadline:
            return False
        if now >= next_report:
            if os.path.isfile(pcd_file):
                log(f'等待 PCD 写盘完成：{os.path.getsize(pcd_file) / 1024 / 1024:.2f} MB'
                    f'（已等 {now - started:.0f}s / {wait_timeout:.0f}s）')
            else:
                log(f'等待 PCD 出现：{pcd_file}'
                    f'（已等 {now - started:.0f}s / {wait_timeout:.0f}s）')
            next_report = now + 10.0
        time.sleep(poll_s)


def convert_pcd_to_map(
    pcd_file: str,
    map_output_dir: str,
    map_name: str,
    resolution: float,
    z_min: float,
    z_max: float,
    occupied_thresh: float,
    free_thresh: float,
    padding_m: float,
    log: Callable[[str], None],
    wait_timeout: float = 60.0,
) -> Tuple[bool, str, Optional[str]]:
    """等待 PCD 写完整 → 解析 → 生成 2D 栅格 → 写 .pgm/.yaml。

    wait_timeout > 0 时先等待 PCD 写盘完成（依据 header 推算的完整字节数）；
    wait_timeout <= 0 表示调用方已确认 PCD 写完整，直接进入转换。
    V0.0.90：等待超时但文件非空时不再直接放弃——解析器按可用字节数容忍
    截断，尽力转换已写入部分并在结果中注明“部分地图”。
    返回 (是否成功, 结果说明, yaml_path)；异常统一转成失败说明，不外抛。
    """
    partial = False
    if wait_timeout and wait_timeout > 0:
        if not wait_for_complete_pcd(pcd_file, wait_timeout, log):
            if os.path.isfile(pcd_file) and os.path.getsize(pcd_file) > 0:
                log(f'[警告] 等待 PCD 写盘完成超时（{wait_timeout:.0f}s），'
                    f'按已写入部分尽力转换：'
                    f'{os.path.getsize(pcd_file) / 1024 / 1024:.2f} MB')
                partial = True
            else:
                return (False,
                        f'等待 PCD 文件出现超时（{wait_timeout:.0f}s）：{pcd_file}'
                        f'（FAST-LIO2 未写盘？）',
                        None)

    try:
        points = PcdParser.load(pcd_file)
        log(f'解析完成：{points.shape[0]} 个点，'
            f'z 范围：[{points[:, 2].min():.2f}, {points[:, 2].max():.2f}] m')

        grid, origin_x, origin_y = pcd_to_occupancy_grid(
            points,
            resolution=resolution,
            z_min=z_min,
            z_max=z_max,
            padding_m=padding_m,
            occupied_thresh=occupied_thresh,
        )
        h, w = grid.shape
        occupied_count = int((grid == 0).sum())
        log(f'栅格生成：{w}×{h} px，分辨率 {resolution} m/px，'
            f'占据格 {occupied_count} 个，原点 ({origin_x:.3f}, {origin_y:.3f})')

        pgm_path, yaml_path = write_pgm_yaml(
            grid, origin_x, origin_y,
            resolution=resolution,
            output_dir=map_output_dir,
            map_name=map_name,
            occupied_thresh=occupied_thresh,
            free_thresh=free_thresh,
        )
        msg = (f'转换成功：{points.shape[0]} pts → {w}×{h} px 栅格'
               + ('（PCD 未写完，部分地图）' if partial else '') + '\n'
               f'  PGM : {pgm_path}\n  YAML: {yaml_path}')
        log(msg)
        return True, msg, yaml_path
    except Exception as e:  # noqa: BLE001 - 统一转成失败说明，由调用方上报
        return False, f'转换失败：{e}', None


def make_line_logger() -> Callable[[str], None]:
    """带时间戳的行日志（stdout 由调用方重定向到日志文件或直接显示）。"""
    def log(message: str) -> None:
        stamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        for line in (str(message).splitlines() or ['']):
            print(f'[{stamp}] [finalize] {line}', flush=True)
    return log


# ---------------------------------------------------------------------------
# ROS2 节点主体
# ---------------------------------------------------------------------------
class PcdToMap(Node):

    def __init__(self) -> None:
        super().__init__('pcd_to_map')

        # ---- 参数声明 ----
        self.declare_parameter('pcd_file',        '/home/agilex/HunterEdge/maps/hunter_map.pcd')
        self.declare_parameter('map_output_dir',  '')          # 空 = 与 pcd_file 同目录
        self.declare_parameter('map_name',        'hunter_map')
        self.declare_parameter('resolution',      0.05)
        self.declare_parameter('z_min',           0.1)
        self.declare_parameter('z_max',           2.0)
        self.declare_parameter('occupied_thresh', 0.65)
        self.declare_parameter('free_thresh',     0.25)
        self.declare_parameter('padding_m',       0.5)
        self.declare_parameter('auto_reload_map', True)
        self.declare_parameter('trigger_on_mapping_end', True)
        self.declare_parameter('convert_on_start_if_missing', True)
        # ---- 退出兜底转换（V0.0.88，建图模式启用） ----
        # Ctrl+C 时 auto_mission/pcd_to_map 与 fast_lio2 同时被终止，运行期
        # MAPPING→非MAPPING 跳变不会发生；由本节点在退出路径派生独立会话的
        # 后台转换进程，等 FAST-LIO2 把 PCD 写完后转换（详见文件头说明）。
        self.declare_parameter('final_convert_on_shutdown', False)
        self.declare_parameter('final_wait_timeout',        120.0)
        self.declare_parameter('final_log_file',            '')
        # 运行期自动转换等待 PCD 写盘完成的上限（s）
        self.declare_parameter('convert_wait_timeout',      60.0)

        self._pcd_file        = self.get_parameter('pcd_file').get_parameter_value().string_value
        self._map_output_dir  = self.get_parameter('map_output_dir').get_parameter_value().string_value
        self._map_name        = self.get_parameter('map_name').get_parameter_value().string_value
        self._resolution      = self.get_parameter('resolution').get_parameter_value().double_value
        self._z_min           = self.get_parameter('z_min').get_parameter_value().double_value
        self._z_max           = self.get_parameter('z_max').get_parameter_value().double_value
        self._occupied_thresh = self.get_parameter('occupied_thresh').get_parameter_value().double_value
        self._free_thresh     = self.get_parameter('free_thresh').get_parameter_value().double_value
        self._padding_m       = self.get_parameter('padding_m').get_parameter_value().double_value
        self._auto_reload     = self.get_parameter('auto_reload_map').get_parameter_value().bool_value
        self._trigger_on_end  = self.get_parameter('trigger_on_mapping_end').get_parameter_value().bool_value
        self._convert_on_start = self.get_parameter(
            'convert_on_start_if_missing').get_parameter_value().bool_value
        self._final_on_shutdown = self.get_parameter(
            'final_convert_on_shutdown').get_parameter_value().bool_value
        self._final_wait_timeout = self.get_parameter(
            'final_wait_timeout').get_parameter_value().double_value
        self._final_log_file = os.path.expanduser(self.get_parameter(
            'final_log_file').get_parameter_value().string_value)
        self._convert_wait_timeout = self.get_parameter(
            'convert_wait_timeout').get_parameter_value().double_value

        # launch 命令行传参形如 map_file_path:=~/HunterEdge/... 时 '~' 不会被 shell 展开，
        # 这里统一 expanduser，保证与 FAST-LIO2 落盘路径一致
        self._pcd_file = os.path.expanduser(self._pcd_file)
        self._map_output_dir = os.path.expanduser(self._map_output_dir)

        # 输出目录：为空则与 pcd_file 同目录
        if not self._map_output_dir:
            self._map_output_dir = str(Path(self._pcd_file).parent)

        # 退出兜底转换日志（默认与地图同目录）
        if not self._final_log_file:
            self._final_log_file = os.path.join(self._map_output_dir,
                                                'pcd_to_map_final.log')
        # 退出兜底"锁"：记录已在等待的后台转换进程 PID，避免重复派生
        self._final_lock_file = os.path.join(self._map_output_dir,
                                             '.pcd_to_map_final.lock')

        # ---- 内部状态 ----
        self._prev_mission_status = ''
        self._mapping_seen = False      # 本轮是否进入过 MAPPING（退出兜底判定用）
        self._converting = False        # 防止重入
        self._convert_lock = threading.Lock()
        self._auto_convert_scheduled = False  # 同一轮建图结束只启动一次自动转换
        self._save_cli = None           # /fast_lio2/map_save 客户端（懒创建）
        self._fallback_timer = None     # map_save 无响应时的兜底定时器

        # ---- 发布 ----
        self._status_pub = self.create_publisher(String, '/pcd_to_map/status', 10)
        self._publish_status('IDLE')

        # ---- 订阅 auto_mission/status ----
        self._mission_sub = self.create_subscription(
            String, '/auto_mission/status', self._mission_status_cb, 10)

        # ---- 服务（手动触发） ----
        self._convert_srv = self.create_service(
            Trigger, '/pcd_to_map/convert', self._convert_srv_cb)

        # ---- map_server 重载客户端（延迟创建，nav 模式下才需要） ----
        self._load_map_cli = None
        if self._auto_reload:
            try:
                from nav2_msgs.srv import LoadMap
                self._load_map_cli = self.create_client(LoadMap, '/map_server/load_map')
            except ImportError:
                self.get_logger().warn('nav2_msgs 不可用，auto_reload_map 功能禁用')
                self._auto_reload = False

        # ---- 启动自愈：YAML 缺失而 PCD 存在时自动补转换 ----
        # 场景：建图结束时自动转换因竞态/异常未完成，maps/ 下只有 .pcd。
        # nav 模式启动本节点后自动补齐 .pgm/.yaml；若本次 map_server 已因
        # 缺图启动失败，转换完成后重启 launch 即可（map_server 正常运行时
        # auto_reload_map=true 会直接热重载，无需重启）。
        if self._convert_on_start:
            yaml_path = os.path.join(self._map_output_dir, f'{self._map_name}.yaml')
            if not os.path.isfile(yaml_path) and os.path.isfile(self._pcd_file):
                self.get_logger().warn(
                    '[pcd_to_map] 地图 YAML 缺失而 PCD 存在，2s 后自动补转换\n'
                    f'  YAML: {yaml_path}\n  PCD : {self._pcd_file}')
                threading.Timer(2.0, self._do_convert).start()
            elif not os.path.isfile(yaml_path):
                self.get_logger().info(
                    '[pcd_to_map] 地图 YAML 缺失且 PCD 不存在，跳过启动自愈\n'
                    f'  PCD : {self._pcd_file}')

        self.get_logger().info(
            f'pcd_to_map 节点启动\n'
            f'  PCD 文件   : {self._pcd_file}\n'
            f'  输出目录   : {self._map_output_dir}\n'
            f'  地图名称   : {self._map_name}\n'
            f'  分辨率     : {self._resolution} m/px\n'
            f'  z 切片     : [{self._z_min}, {self._z_max}] m\n'
            f'  自动重载   : {self._auto_reload}\n'
            f'  建图结束触发: {self._trigger_on_end}\n'
            f'  退出兜底转换: {self._final_on_shutdown}'
            f'（等待 PCD 上限 {self._final_wait_timeout:.0f}s，'
            f'日志 {self._final_log_file}）'
        )

    # ------------------------------------------------------------------
    # auto_mission/status 订阅回调
    # ------------------------------------------------------------------
    def _mission_status_cb(self, msg: String) -> None:
        current = msg.data
        if current == 'MAPPING':
            # 记住本轮确实进入过建图状态：退出兜底只在这种情况下派生转换进程
            self._mapping_seen = True
        # 检测 MAPPING → 非 MAPPING 的跳变
        if (self._trigger_on_end and
                self._prev_mission_status == 'MAPPING' and
                current != 'MAPPING'):
            self.get_logger().info(
                f'[pcd_to_map] 检测到建图结束（{self._prev_mission_status} → {current}），'
                f'请求 FAST-LIO2 保存地图后自动转换...'
            )
            with self._convert_lock:
                self._auto_convert_scheduled = False
            self._request_map_save_and_convert()

        self._prev_mission_status = current

    # ------------------------------------------------------------------
    # 建图结束：先请求 /fast_lio2/map_save 把内存地图写入 map_file_path
    # ------------------------------------------------------------------
    def _request_map_save_and_convert(self) -> None:
        """在主 executor 线程中调用（订阅回调），避免在工作线程里做 ROS 服务调用。

        map_save 是 FAST-LIO2 提供的 Trigger 服务，回调内把累积地图写到
        map_file_path（即本节点的 pcd_file），写盘成功后再触发转换。
        若服务长时间无响应（例如 FAST-LIO2 已退出），由兜底定时器回退到
        “直接等待 PCD 文件出现”的旧逻辑。
        """
        try:
            from std_srvs.srv import Trigger
        except ImportError:
            self.get_logger().warn(
                '[pcd_to_map] std_srvs 不可用，跳过 map_save，直接等待 PCD 文件')
            self._schedule_auto_convert(3.0)
            return

        if self._save_cli is None:
            self._save_cli = self.create_client(Trigger, '/fast_lio2/map_save')

        if self._fallback_timer is None:
            # 兜底：15s 内 map_save 未返回则进入文件等待流程（原逻辑最多等 30s）
            self._fallback_timer = self.create_timer(15.0, self._fallback_convert)

        if not self._save_cli.service_is_ready():
            self.get_logger().info(
                '[pcd_to_map] /fast_lio2/map_save 尚未就绪，仍尝试调用（兜底定时器生效中）')

        req = Trigger.Request()
        future = self._save_cli.call_async(req)
        future.add_done_callback(self._on_map_save_done)

    def _on_map_save_done(self, future) -> None:
        if self._fallback_timer is not None:
            self._fallback_timer.cancel()
            self._fallback_timer = None
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info(
                    f'[pcd_to_map] FAST-LIO2 地图保存成功：{resp.message}')
            else:
                self.get_logger().warn(
                    f'[pcd_to_map] FAST-LIO2 地图保存返回失败：{resp.message}')
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'[pcd_to_map] 调用 /fast_lio2/map_save 异常：{e}')
        # 等 1s 让磁盘写入落稳后开始转换
        self._schedule_auto_convert(1.0)

    def _fallback_convert(self) -> None:
        self._fallback_timer = None
        self.get_logger().warn(
            '[pcd_to_map] /fast_lio2/map_save 长时间无响应，'
            '回退为直接等待 PCD 文件出现')
        self._schedule_auto_convert(1.0)

    def _schedule_auto_convert(self, delay: float) -> None:
        with self._convert_lock:
            if self._auto_convert_scheduled:
                self.get_logger().debug('[pcd_to_map] 自动转换已安排，忽略重复触发')
                return
            self._auto_convert_scheduled = True
        threading.Timer(delay, self._do_convert).start()

    # ------------------------------------------------------------------
    # 退出兜底转换：派生独立会话的后台转换进程（V0.0.88）
    # ------------------------------------------------------------------
    def _worker_pid_alive(self) -> Optional[int]:
        """读取锁文件，返回仍在运行的后台转换进程 PID（无则 None）。"""
        try:
            with open(self._final_lock_file, 'r', encoding='utf-8') as f:
                pid = int(f.read().split()[0])
        except (OSError, ValueError, IndexError):
            return None
        if pid <= 0:
            return None
        try:
            os.kill(pid, 0)          # 信号 0：仅探测进程是否存在
        except OSError:
            return None
        return pid

    def _need_final_convert(self) -> Optional[str]:
        """判断是否需要退出兜底转换；返回原因字符串，None 表示无需转换。

        此处不做 YAML/PCD 的 mtime 比较——FAST-LIO2 在退出瞬间才会覆盖写 PCD，
        是否"已是最新"由后台进程在 PCD 写完整之后再判定（见 finalize_convert）。
        """
        if not self._mapping_seen and not os.path.isfile(self._pcd_file):
            return None              # 本轮未进入建图且无历史 PCD，无图可转
        return '本轮建图结束后尚未转换（PCD 在 FAST-LIO2 退出时才写盘）'

    def _spawn_final_worker(self) -> Optional[int]:
        """派生后台转换进程（独立会话），成功返回其 PID。"""
        cmd = [
            sys.executable or 'python3', os.path.abspath(__file__), '--finalize',
            '--pcd-file', self._pcd_file,
            '--map-output-dir', self._map_output_dir,
            '--map-name', self._map_name,
            '--resolution', str(self._resolution),
            '--z-min', str(self._z_min),
            '--z-max', str(self._z_max),
            '--occupied-thresh', str(self._occupied_thresh),
            '--free-thresh', str(self._free_thresh),
            '--padding-m', str(self._padding_m),
            '--wait-timeout', str(self._final_wait_timeout),
            '--lock-file', self._final_lock_file,
        ]
        try:
            log_fp = open(self._final_log_file, 'ab', buffering=0)
        except OSError as e:
            self.get_logger().error(
                f'[pcd_to_map] 无法打开兜底转换日志 {self._final_log_file}：{e}')
            return None
        try:
            # start_new_session=True → setsid：新会话/进程组，不受终端 Ctrl+C
            # 的进程组 SIGINT，也不受 launch 对子进程的 SIGINT/SIGTERM/SIGKILL
            proc = subprocess.Popen(
                cmd, stdout=log_fp, stderr=log_fp, stdin=subprocess.DEVNULL,
                close_fds=True, start_new_session=True)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'[pcd_to_map] 派生后台转换进程失败：{e}')
            return None
        finally:
            log_fp.close()

        try:
            with open(self._final_lock_file, 'w', encoding='utf-8') as f:
                f.write(f'{proc.pid} {int(time.time())}\n')
        except OSError:
            pass
        return proc.pid

    def finalize_on_shutdown(self) -> None:
        """退出兜底：把"等 PCD 写盘 + 转换"交给独立会话的后台进程执行。

        由 main() 在 rclpy 关闭前（finally）调用。本进程随后可能被 launch
        立即终止，但后台进程独立成会话，不受影响。
        """
        if not self._final_on_shutdown:
            return
        reason = self._need_final_convert()
        if reason is None:
            self.get_logger().info(
                '[pcd_to_map] 退出兜底转换：无需转换（本轮未建图且无历史 PCD）')
            return
        pid = self._worker_pid_alive()
        if pid is not None:
            self.get_logger().warn(
                f'[pcd_to_map] 退出兜底转换：已有后台转换进程（PID {pid}）在等待，'
                f'跳过重复派生')
            return
        pid = self._spawn_final_worker()
        if pid is None:
            self.get_logger().error(
                '[pcd_to_map] 退出兜底转换失败：后台进程未能启动；'
                '退出后可手动执行：\n'
                f'  python3 {os.path.abspath(__file__)} --finalize '
                f'--pcd-file {self._pcd_file} '
                f'--map-output-dir {self._map_output_dir} '
                f'--map-name {self._map_name}')
            return
        self.get_logger().info(
            f'[pcd_to_map] 退出兜底转换已启动（{reason}）\n'
            f'  后台进程 PID {pid}（独立会话，不受 launch/Ctrl+C 信号影响）\n'
            f'  待 FAST-LIO2 写完 PCD 后自动生成 .pgm/.yaml\n'
            f'  进度日志：tail -f {self._final_log_file}\n'
            f'  结果查看：ls -lh {self._map_output_dir}')
        # V0.0.90：logger.info 经 DDS 异步转发，本节点随后即 os._exit，该行
        # 可能来不及上屏；同步 print 一份保证现场可在 launch 日志中看到派生证据
        print(f'[pcd_to_map] 退出兜底转换已派生后台进程 PID {pid}'
              f'（进度：tail -f {self._final_log_file}）', flush=True)

    # ------------------------------------------------------------------
    # 手动触发服务回调
    # ------------------------------------------------------------------
    def _convert_srv_cb(self, _req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        self.get_logger().info('[pcd_to_map] 收到手动转换请求')
        ok, msg = self._do_convert()
        resp.success = ok
        resp.message = msg
        return resp

    # ------------------------------------------------------------------
    # 核心转换逻辑（线程安全）
    # ------------------------------------------------------------------
    def _do_convert(self) -> Tuple[bool, str]:
        with self._convert_lock:
            if self._converting:
                msg = '已有转换任务正在进行，请稍后再试'
                self.get_logger().warn(f'[pcd_to_map] {msg}')
                return False, msg
            self._converting = True

        self._publish_status('CONVERTING')
        self.get_logger().info(f'[pcd_to_map] 开始转换：{self._pcd_file}')

        try:
            # 等待 PCD 写盘完成（依据 header 推算的完整字节数）→ 解析 → 2D 栅格
            # → 写 .pgm/.yaml；与退出兜底进程（--finalize）共用同一套转换管线
            ok, result_msg, yaml_path = convert_pcd_to_map(
                pcd_file=self._pcd_file,
                map_output_dir=self._map_output_dir,
                map_name=self._map_name,
                resolution=self._resolution,
                z_min=self._z_min,
                z_max=self._z_max,
                occupied_thresh=self._occupied_thresh,
                free_thresh=self._free_thresh,
                padding_m=self._padding_m,
                log=self.get_logger().info,
                wait_timeout=self._convert_wait_timeout,
            )
            if not ok:
                self.get_logger().error(f'[pcd_to_map] {result_msg}')
                self._publish_status(f'ERROR: {result_msg[:80]}')
                return False, result_msg

            # 通知 map_server 重载（nav 模式启动自愈路径）
            if self._auto_reload and self._load_map_cli is not None and yaml_path:
                self._reload_map(yaml_path)

            self._publish_status('DONE')
            return True, result_msg

        except Exception as e:
            err_msg = f'转换失败：{e}'
            self.get_logger().error(f'[pcd_to_map] {err_msg}')
            self._publish_status(f'ERROR: {str(e)[:80]}')
            return False, err_msg
        finally:
            with self._convert_lock:
                self._converting = False

    # ------------------------------------------------------------------
    # 调用 map_server/load_map 服务重载地图
    # ------------------------------------------------------------------
    def _reload_map(self, yaml_path: str) -> None:
        from nav2_msgs.srv import LoadMap

        if not self._load_map_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                '[pcd_to_map] /map_server/load_map 服务不可用，跳过自动重载\n'
                '  常见原因：本次 bringup 中 map_server 因缺图 configure 失败\n'
                '  （lifecycle_manager 已 Aborting，服务从未创建）。地图文件\n'
                '  此刻已补齐，重启本 launch 即可正常加载，无需手动干预。\n'
                '  若 map_server 处于 active 状态（ros2 lifecycle get '
                '/map_server），可手动重载：\n'
                f'  ros2 service call /map_server/load_map '
                f'nav2_msgs/srv/LoadMap "{{map_url: \'{yaml_path}\'}}"')
            return

        req = LoadMap.Request()
        req.map_url = yaml_path
        future = self._load_map_cli.call_async(req)

        # 等待结果（最多 10s）
        deadline = time.time() + 10.0
        while not future.done() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        if future.done():
            resp = future.result()
            if resp.result == LoadMap.Response.RESULT_SUCCESS:
                self.get_logger().info('[pcd_to_map] map_server 地图重载成功')
            else:
                self.get_logger().warn(
                    f'[pcd_to_map] map_server 重载失败（result={resp.result}），'
                    f'请手动调用 load_map 服务')
        else:
            self.get_logger().warn('[pcd_to_map] map_server 重载请求超时')

    # ------------------------------------------------------------------
    # 发布状态话题
    # ------------------------------------------------------------------
    def _publish_status(self, status: str) -> None:
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)


# ---------------------------------------------------------------------------
# 退出信号处理：首次信号优雅退出（走 finally 内的兜底转换），二次信号立即退出
# ---------------------------------------------------------------------------
class _ShutdownSignals:
    """SIGINT/SIGTERM → 抛 KeyboardInterrupt，统一走正常退出路径。

    只接管第一次信号：用户在兜底/关闭流程中再次 Ctrl+C 时恢复系统默认行为
    立即终止，避免"退不掉"的困惑。
    """

    def __init__(self) -> None:
        self._received = False

    def install(self) -> None:
        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                signal.signal(sig, self._handle)
            except (ValueError, OSError, RuntimeError):  # 非主线程/受限环境
                pass

    def _handle(self, signum, frame) -> None:
        if self._received:
            signal.signal(signum, signal.SIG_DFL)
            os.kill(os.getpid(), signum)
            return
        self._received = True
        raise KeyboardInterrupt


# ---------------------------------------------------------------------------
# --finalize：后台 / 离线转换入口（不初始化 rclpy）
# ---------------------------------------------------------------------------
def build_finalize_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog='pcd_to_map --finalize',
        description='等待 FAST-LIO2 写完 PCD 后转换为 .pgm + .yaml（无需 ROS 环境）')
    parser.add_argument('--finalize', action='store_true',
                        help='后台/离线转换模式标记（由 pcd_to_map 节点在退出时派生）')
    parser.add_argument('--pcd-file', required=True, help='PCD 文件绝对路径')
    parser.add_argument('--map-output-dir', default='',
                        help='PGM/YAML 输出目录（默认与 PCD 同目录）')
    parser.add_argument('--map-name', default='hunter_map',
                        help='输出文件名前缀（默认 hunter_map）')
    parser.add_argument('--resolution', type=float, default=0.05)
    parser.add_argument('--z-min', type=float, default=0.1)
    parser.add_argument('--z-max', type=float, default=2.0)
    parser.add_argument('--occupied-thresh', type=float, default=0.65)
    parser.add_argument('--free-thresh', type=float, default=0.25)
    parser.add_argument('--padding-m', type=float, default=0.5)
    parser.add_argument('--wait-timeout', type=float, default=120.0,
                        help='等待 PCD 写盘完成的最长时间，s（默认 120）')
    parser.add_argument('--lock-file', default='',
                        help='父进程写入 PID 的锁文件（退出时按 PID 清理）')
    parser.add_argument('--force', action='store_true',
                        help='即使 .yaml 比 .pcd 新也强制重新转换')
    return parser


def finalize_convert(argv) -> int:
    """--finalize 入口：等待 PCD 写完整 → 转换 .pgm/.yaml，返回进程退出码。"""
    opts = build_finalize_parser().parse_args(argv)

    pcd_file = os.path.expanduser(opts.pcd_file)
    out_dir = os.path.expanduser(opts.map_output_dir) or str(Path(pcd_file).parent)
    lock_file = (os.path.expanduser(opts.lock_file)
                 or os.path.join(out_dir, '.pcd_to_map_final.lock'))
    log = make_line_logger()

    def _release_lock() -> None:
        """仅当锁文件记录的 PID 是本进程时才删除（避免误删后续会话的锁）。"""
        try:
            with open(lock_file, 'r', encoding='utf-8') as f:
                if int(f.read().split()[0]) != os.getpid():
                    return
            os.remove(lock_file)
        except (OSError, ValueError, IndexError):
            pass

    try:
        os.makedirs(out_dir, exist_ok=True)
        log(f'后台转换进程启动（PID {os.getpid()}，独立会话）')
        log(f'  PCD      : {pcd_file}')
        log(f'  输出目录 : {out_dir}')
        log(f'  等待上限 : {opts.wait_timeout:.0f}s')

        if not wait_for_complete_pcd(pcd_file, opts.wait_timeout, log):
            if not (os.path.isfile(pcd_file) and os.path.getsize(pcd_file) > 0):
                log(f'[错误] 等待 PCD 出现超时（{opts.wait_timeout:.0f}s）——'
                    f'FAST-LIO2 可能未写盘（检查 pcd_save_en 注入与 map_file_path）')
                return 2
            # V0.0.90：写盘超时但文件非空（如大地图写入中被 SIGTERM），
            # 不再放弃，按已写入部分尽力转换
            log(f'[警告] 等待 PCD 写盘完成超时（{opts.wait_timeout:.0f}s），'
                f'尝试按已写入部分尽力转换')

        yaml_path = os.path.join(out_dir, f'{opts.map_name}.yaml')
        if (not opts.force and os.path.isfile(yaml_path)
                and os.path.getmtime(yaml_path) >= os.path.getmtime(pcd_file)):
            log(f'地图已是最新（{yaml_path} 比 PCD 新），跳过转换')
            return 0

        ok, msg, _yaml = convert_pcd_to_map(
            pcd_file=pcd_file,
            map_output_dir=out_dir,
            map_name=opts.map_name,
            resolution=opts.resolution,
            z_min=opts.z_min,
            z_max=opts.z_max,
            occupied_thresh=opts.occupied_thresh,
            free_thresh=opts.free_thresh,
            padding_m=opts.padding_m,
            log=log,
            wait_timeout=0.0,     # 上面已确认 PCD 写完整
        )
        if not ok:
            log(f'[错误] {msg}')
            return 3
        log('后台转换完成，可重启 bring-up 加载新地图')
        return 0
    except Exception as e:  # noqa: BLE001
        log(f'[错误] 转换异常：{e}')
        return 4
    finally:
        _release_lock()


# ---------------------------------------------------------------------------
def main(args=None) -> None:
    argv = list(sys.argv[1:] if args is None else args)
    if '--finalize' in argv:
        # 后台/离线转换模式：不初始化 rclpy（可脱离 ROS 环境运行）
        sys.exit(finalize_convert(argv))

    if not _ROS_AVAILABLE:
        sys.stderr.write(
            'pcd_to_map：未检测到 ROS2 环境（rclpy 导入失败）。\n'
            '  节点模式请先 source install/setup.bash；\n'
            '  离线转换请用：pcd_to_map --finalize --pcd-file <pcd> ...\n')
        sys.exit(1)

    rclpy.init(args=args)
    node = PcdToMap()
    _ShutdownSignals().install()     # SIGINT/SIGTERM 统一走"优雅退出 + 兜底转换"
    try:
        # V0.0.90：rclpy.spin 的单个大 C 阻塞改为 0.3s 分时 spin_once——
        # cyclonedds waitset 阻塞在 C 层时，Python 信号处理器抛出的
        # KeyboardInterrupt 可能长时间不被执行，实机两次复现：SIGINT 后
        # 5s 未退 → SIGTERM → 10s → SIGKILL，退出兜底派生机会命悬一线。
        # 每轮 spin_once 必回 Python 层，信号异常≤ 0.3s 内得到处理。
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.3)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # 退出兜底（V0.0.88）：把"等 PCD 写盘 + 转换"交给独立会话的后台进程
        try:
            node.finalize_on_shutdown()
        except Exception as e:  # noqa: BLE001
            try:
                node.get_logger().error(f'[pcd_to_map] 退出兜底转换异常：{e}')
            except Exception:
                print(f'[pcd_to_map] 退出兜底转换异常：{e}', flush=True)
        # V0.0.90：兜底 worker 已在独立会话，不再执行 destroy_node/rclpy.shutdown
        # （现场同环境下 DDS 上下文清理同样会挂死，坐等 launch 的 SIGKILL 升级），
        # 直接以 0 退出，把退出时间控在亚秒级。
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)


if __name__ == '__main__':
    main()
