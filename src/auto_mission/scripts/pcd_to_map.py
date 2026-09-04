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

import math
import os
import struct
import threading
import time
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


# ---------------------------------------------------------------------------
# 纯 Python PCD 解析（支持 ASCII / Binary，不依赖 libpcl）
# ---------------------------------------------------------------------------
class PcdParser:
    """最小化 PCD 解析器，只提取 x y z 坐标。"""

    @staticmethod
    def load(filepath: str) -> np.ndarray:
        """返回 shape=(N,3) float32 ndarray，列为 (x, y, z)。"""
        with open(filepath, 'rb') as f:
            raw = f.read()

        # ---- 解析 header ----
        header_end_marker = b'\nDATA '
        idx = raw.find(header_end_marker)
        if idx == -1:
            raise ValueError('无效 PCD 文件：找不到 DATA 字段')

        header_str = raw[:idx].decode('ascii', errors='replace')
        data_type  = 'ascii'
        fields: list = []
        sizes:  list = []
        types:  list = []
        counts: list = []
        n_points = 0

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
                n_points = int(tok[1]) * max(1, (int(tok[1]) // max(1, int(tok[1]))))
            elif key == 'POINTS':
                n_points = int(tok[1])
            elif key == 'DATA':
                data_type = tok[1].lower()

        if not counts:
            counts = [1] * len(fields)

        # ---- 找到 x y z 索引 ----
        try:
            xi = fields.index('x')
            yi = fields.index('y')
            zi = fields.index('z')
        except ValueError:
            raise ValueError(f'PCD 缺少 x/y/z 字段，实际字段：{fields}')

        # data 起始偏移 = idx + len('\nDATA ') + len(data_type) + 1(换行)
        data_offset = idx + len(header_end_marker) + len(data_type) + 1

        if data_type == 'ascii':
            return PcdParser._parse_ascii(raw[data_offset:], xi, yi, zi, n_points)
        elif data_type == 'binary':
            return PcdParser._parse_binary(
                raw[data_offset:], fields, sizes, types, counts, xi, yi, zi, n_points)
        else:
            raise ValueError(f'不支持的 PCD 数据格式：{data_type}（需要 ascii 或 binary）')

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
    """将栅格写出为 .pgm + .yaml，返回 (pgm_path, yaml_path)。"""
    os.makedirs(output_dir, exist_ok=True)

    pgm_path  = os.path.join(output_dir, f'{map_name}.pgm')
    yaml_path = os.path.join(output_dir, f'{map_name}.yaml')

    # ---- 写 PGM（P5 二进制灰度，8位） ----
    height, width = grid.shape
    with open(pgm_path, 'wb') as f:
        f.write(f'P5\n{width} {height}\n255\n'.encode('ascii'))
        f.write(grid.tobytes())

    # ---- 写 YAML（Nav2 map_server 格式） ----
    yaml_content = (
        f'image: {os.path.basename(pgm_path)}\n'
        f'resolution: {resolution}\n'
        f'origin: [{origin_x:.6f}, {origin_y:.6f}, 0.0]\n'
        f'negate: 0\n'
        f'occupied_thresh: {occupied_thresh}\n'
        f'free_thresh: {free_thresh}\n'
    )
    with open(yaml_path, 'w', encoding='utf-8') as f:
        f.write(yaml_content)

    return pgm_path, yaml_path


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

        # launch 命令行传参形如 map_file_path:=~/HunterEdge/... 时 '~' 不会被 shell 展开，
        # 这里统一 expanduser，保证与 FAST-LIO2 落盘路径一致
        self._pcd_file = os.path.expanduser(self._pcd_file)
        self._map_output_dir = os.path.expanduser(self._map_output_dir)

        # 输出目录：为空则与 pcd_file 同目录
        if not self._map_output_dir:
            self._map_output_dir = str(Path(self._pcd_file).parent)

        # ---- 内部状态 ----
        self._prev_mission_status = ''
        self._converting = False        # 防止重入
        self._convert_lock = threading.Lock()
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

        self.get_logger().info(
            f'pcd_to_map 节点启动\n'
            f'  PCD 文件   : {self._pcd_file}\n'
            f'  输出目录   : {self._map_output_dir}\n'
            f'  地图名称   : {self._map_name}\n'
            f'  分辨率     : {self._resolution} m/px\n'
            f'  z 切片     : [{self._z_min}, {self._z_max}] m\n'
            f'  自动重载   : {self._auto_reload}\n'
            f'  建图结束触发: {self._trigger_on_end}'
        )

    # ------------------------------------------------------------------
    # auto_mission/status 订阅回调
    # ------------------------------------------------------------------
    def _mission_status_cb(self, msg: String) -> None:
        current = msg.data
        # 检测 MAPPING → 非 MAPPING 的跳变
        if (self._trigger_on_end and
                self._prev_mission_status == 'MAPPING' and
                current != 'MAPPING'):
            self.get_logger().info(
                f'[pcd_to_map] 检测到建图结束（{self._prev_mission_status} → {current}），'
                f'请求 FAST-LIO2 保存地图后自动转换...'
            )
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
            threading.Timer(3.0, self._do_convert).start()
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
        threading.Timer(1.0, self._do_convert).start()

    def _fallback_convert(self) -> None:
        self._fallback_timer = None
        self.get_logger().warn(
            '[pcd_to_map] /fast_lio2/map_save 长时间无响应，'
            '回退为直接等待 PCD 文件出现')
        threading.Timer(1.0, self._do_convert).start()

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
            # 1. 等待 PCD 文件存在
            waited = 0.0
            while not os.path.isfile(self._pcd_file):
                if waited >= 30.0:
                    raise FileNotFoundError(
                        f'等待 PCD 文件超时（30s）：{self._pcd_file}')
                self.get_logger().info(
                    f'[pcd_to_map] 等待 PCD 文件出现：{self._pcd_file} ({waited:.0f}s)')
                time.sleep(2.0)
                waited += 2.0

            # 等待文件大小稳定（确认写入完成）
            prev_size = -1
            stable_count = 0
            while stable_count < 2:
                size = os.path.getsize(self._pcd_file)
                if size == prev_size and size > 0:
                    stable_count += 1
                else:
                    stable_count = 0
                prev_size = size
                time.sleep(1.0)

            self.get_logger().info(
                f'[pcd_to_map] PCD 文件就绪，大小：{prev_size/1024/1024:.2f} MB，开始解析...')

            # 2. 解析 PCD
            points = PcdParser.load(self._pcd_file)
            self.get_logger().info(
                f'[pcd_to_map] 解析完成：{points.shape[0]} 个点，'
                f'z 范围：[{points[:,2].min():.2f}, {points[:,2].max():.2f}] m')

            # 3. 生成 2D 占据栅格
            grid, origin_x, origin_y = pcd_to_occupancy_grid(
                points,
                resolution=self._resolution,
                z_min=self._z_min,
                z_max=self._z_max,
                padding_m=self._padding_m,
                occupied_thresh=self._occupied_thresh,
            )
            h, w = grid.shape
            occupied_count = int((grid == 0).sum())
            self.get_logger().info(
                f'[pcd_to_map] 栅格生成：{w}×{h} px，分辨率 {self._resolution} m/px，'
                f'占据格 {occupied_count} 个，'
                f'原点 ({origin_x:.3f}, {origin_y:.3f})')

            # 4. 写出 PGM + YAML
            pgm_path, yaml_path = write_pgm_yaml(
                grid, origin_x, origin_y,
                resolution=self._resolution,
                output_dir=self._map_output_dir,
                map_name=self._map_name,
                occupied_thresh=self._occupied_thresh,
                free_thresh=self._free_thresh,
            )
            self.get_logger().info(
                f'[pcd_to_map] 地图文件写出：\n  PGM  : {pgm_path}\n  YAML : {yaml_path}')

            # 5. 通知 map_server 重载
            if self._auto_reload and self._load_map_cli is not None:
                self._reload_map(yaml_path)

            result_msg = (
                f'转换成功：{points.shape[0]} pts → {w}×{h} px 栅格\n'
                f'  PGM : {pgm_path}\n'
                f'  YAML: {yaml_path}'
            )
            self._publish_status('DONE')
            self.get_logger().info(f'[pcd_to_map] {result_msg}')
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
                f'  请手动执行：ros2 service call /map_server/load_map '
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
def main(args=None) -> None:
    rclpy.init(args=args)
    node = PcdToMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
