#!/usr/bin/env python3
# Copyright 2026 HUNTER Development Team
# waypoint_recorder — 航点自动采集节点
#
# 功能：
#   订阅 rviz2 "Publish Point" 工具发布的 /clicked_point，
#   自动将点击坐标追加到 autonomous_nav_params.yaml 的 waypoints 列表。
#   支持三个服务：
#     /waypoint_recorder/save  — 立即将当前内存中的航点列表写入 yaml 文件
#     /waypoint_recorder/clear — 清空内存中的航点列表（不自动写文件，需再 save）
#     /waypoint_recorder/list  — 打印当前内存中的航点列表（返回字符串）
#
# 使用流程：
#   1. 启动建图模式，打开 rviz2
#   2. 在 rviz2 工具栏选择 "Publish Point"（快捷键 P）
#   3. 在地图上逐个点击期望的巡检点位
#   4. 每次点击后自动追加到内存列表，节点日志输出坐标确认
#   5. 点击完毕后调用 save 服务写入 yaml（或节点退出时自动 save）
#   6. 切换到导航模式时直接使用已更新的 yaml 文件
#
# 话题：
#   订阅：/clicked_point (geometry_msgs/PointStamped)
#   服务：/waypoint_recorder/save  (std_srvs/Trigger)
#         /waypoint_recorder/clear (std_srvs/Trigger)
#         /waypoint_recorder/list  (std_srvs/Trigger)

import math
import os
import threading

import rclpy
from rclpy.node import Node

import yaml
from geometry_msgs.msg import PointStamped
from std_srvs.srv import Trigger


class WaypointRecorder(Node):
    """订阅 /clicked_point，自动写入 autonomous_nav_params.yaml。"""

    def __init__(self) -> None:
        super().__init__('waypoint_recorder')

        # ---- 参数 ----
        self.declare_parameter(
            'params_file',
            os.path.join(
                os.path.expanduser('~'),
                'HunterEdge', 'install', 'auto_mission',
                'share', 'auto_mission', 'config', 'autonomous_nav_params.yaml',
            ),
        )
        self.declare_parameter('auto_save', True)   # 节点退出时是否自动保存
        self.declare_parameter('yaw_default', 0.0)  # 未提供朝向时的默认 yaw（弧度）

        self._params_file: str = self.get_parameter('params_file').get_parameter_value().string_value
        self._auto_save: bool  = self.get_parameter('auto_save').get_parameter_value().bool_value
        self._yaw_default: float = self.get_parameter('yaw_default').get_parameter_value().double_value

        # ---- 内存航点列表 ----
        self._waypoints: list[str] = []   # 格式："x,y,yaw,label"
        self._lock = threading.Lock()

        # 从文件加载已有航点（保留用户之前保存的内容）
        self._load_existing()

        # ---- 订阅 ----
        self._clicked_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self._clicked_cb,
            10,
        )

        # ---- 服务 ----
        self._save_srv  = self.create_service(Trigger, '/waypoint_recorder/save',  self._save_cb)
        self._clear_srv = self.create_service(Trigger, '/waypoint_recorder/clear', self._clear_cb)
        self._list_srv  = self.create_service(Trigger, '/waypoint_recorder/list',  self._list_cb)

        self.get_logger().info(
            f'waypoint_recorder 启动\n'
            f'  配置文件：{self._params_file}\n'
            f'  已加载航点：{len(self._waypoints)} 个\n'
            f'  用法：在 rviz2 中选择 "Publish Point" 工具，点击地图上的目标位置'
        )

    # ------------------------------------------------------------------
    # 从 yaml 文件加载已有航点
    # ------------------------------------------------------------------
    def _load_existing(self) -> None:
        if not os.path.isfile(self._params_file):
            self.get_logger().warn(
                f'配置文件不存在，将在首次 save 时创建：{self._params_file}')
            return
        try:
            with open(self._params_file, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
            existing = (
                data
                .get('auto_mission_node', {})
                .get('ros__parameters', {})
                .get('waypoints', [])
            )
            with self._lock:
                self._waypoints = list(existing)
            self.get_logger().info(f'从文件加载了 {len(self._waypoints)} 个已有航点')
        except Exception as e:
            self.get_logger().error(f'加载已有航点失败：{e}')

    # ------------------------------------------------------------------
    # /clicked_point 回调
    # ------------------------------------------------------------------
    def _clicked_cb(self, msg: PointStamped) -> None:
        x = msg.point.x
        y = msg.point.y
        yaw = self._yaw_default

        with self._lock:
            idx = len(self._waypoints)
            label = f'wp{idx:02d}'
            entry = f'{x:.4f},{y:.4f},{yaw:.4f},{label}'
            self._waypoints.append(entry)

        self.get_logger().info(
            f'[航点{idx:02d}] 已记录：x={x:.3f}, y={y:.3f}, yaw={yaw:.3f} → {entry}'
        )
        # 每次点击后立即写文件，防止中途断电丢失
        if self._auto_save:
            self._write_yaml()

    # ------------------------------------------------------------------
    # 写入 yaml 文件（线程安全）
    # ------------------------------------------------------------------
    def _write_yaml(self) -> tuple[bool, str]:
        """将当前 _waypoints 列表写入 autonomous_nav_params.yaml。

        只修改 auto_mission_node.ros__parameters.waypoints 字段，
        保留文件中其他所有内容（mapping/navigation 段等）。
        """
        params_dir = os.path.dirname(self._params_file)
        if params_dir and not os.path.isdir(params_dir):
            try:
                os.makedirs(params_dir, exist_ok=True)
            except OSError as e:
                msg = f'无法创建目录 {params_dir}：{e}'
                self.get_logger().error(msg)
                return False, msg

        # 读取现有文件（若存在），只更新 waypoints 字段
        try:
            if os.path.isfile(self._params_file):
                with open(self._params_file, 'r', encoding='utf-8') as f:
                    data = yaml.safe_load(f) or {}
            else:
                data = {}

            # 确保路径存在
            data.setdefault('auto_mission_node', {}) \
                .setdefault('ros__parameters', {})

            with self._lock:
                waypoints_copy = list(self._waypoints)

            data['auto_mission_node']['ros__parameters']['waypoints'] = waypoints_copy

            with open(self._params_file, 'w', encoding='utf-8') as f:
                yaml.dump(data, f, allow_unicode=True, default_flow_style=False, sort_keys=False)

            msg = f'已保存 {len(waypoints_copy)} 个航点到 {self._params_file}'
            self.get_logger().info(msg)
            return True, msg
        except Exception as e:
            msg = f'写入 yaml 失败：{e}'
            self.get_logger().error(msg)
            return False, msg

    # ------------------------------------------------------------------
    # 服务回调
    # ------------------------------------------------------------------
    def _save_cb(self, _req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        ok, msg = self._write_yaml()
        resp.success = ok
        resp.message = msg
        return resp

    def _clear_cb(self, _req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        with self._lock:
            count = len(self._waypoints)
            self._waypoints.clear()
        msg = f'已清空 {count} 个航点（内存），调用 save 服务以同步到文件'
        self.get_logger().warn(msg)
        resp.success = True
        resp.message = msg
        return resp

    def _list_cb(self, _req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        with self._lock:
            lines = [f'  [{i:02d}] {wp}' for i, wp in enumerate(self._waypoints)]
        summary = f'当前航点列表（{len(lines)} 个）：\n' + '\n'.join(lines)
        self.get_logger().info(summary)
        resp.success = True
        resp.message = summary
        return resp

    # ------------------------------------------------------------------
    # 节点退出时自动保存
    # ------------------------------------------------------------------
    def destroy_node(self) -> None:
        if self._auto_save:
            self.get_logger().info('节点退出，自动保存航点...')
            self._write_yaml()
        super().destroy_node()


# ---------------------------------------------------------------------------
def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
