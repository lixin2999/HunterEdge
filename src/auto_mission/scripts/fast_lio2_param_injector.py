#!/usr/bin/env python3
# Copyright 2026 HUNTER Development Team
# fast_lio2_param_injector — FAST-LIO2 pcd_save 参数自动注入节点
#
# 功能：
#   启动后通过 /fast_lio2/set_parameters 服务，向 fast_lio2 节点动态注入
#   pcd_save_en、map_file_path、pcd_save_interval 三个参数。
#
#   建图模式调用时：pcd_save_en=true，map_file_path=<用户指定路径>
#   导航模式调用时：pcd_save_en=false，map_file_path=''（防止意外写盘）
#
#   注入成功后节点自动退出（one-shot 模式），不持续占用资源。
#
# 参数：
#   target_node       : 目标节点名（默认 fast_lio2）
#   pcd_save_en       : 是否开启 PCD 保存（bool，默认 false）
#   map_file_path     : PCD 保存路径（string，默认空）
#   pcd_save_interval : 保存间隔（int，-1=全部帧合并，默认 -1）
#   retry_times       : 服务连接重试次数（默认 10）
#   retry_interval    : 重试间隔秒数（默认 1.0）

import sys
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters


class FastLio2ParamInjector(Node):

    def __init__(self) -> None:
        super().__init__('fast_lio2_param_injector')

        self.declare_parameter('target_node',        'fast_lio2')
        self.declare_parameter('pcd_save_en',        False)
        self.declare_parameter('map_file_path',      '')
        self.declare_parameter('pcd_save_interval',  -1)
        self.declare_parameter('retry_times',        10)
        self.declare_parameter('retry_interval',     1.0)

        self._target_node     = self.get_parameter('target_node').get_parameter_value().string_value
        self._pcd_save_en     = self.get_parameter('pcd_save_en').get_parameter_value().bool_value
        self._map_file_path   = self.get_parameter('map_file_path').get_parameter_value().string_value
        self._interval        = self.get_parameter('pcd_save_interval').get_parameter_value().integer_value
        self._retry_times     = self.get_parameter('retry_times').get_parameter_value().integer_value
        self._retry_interval  = self.get_parameter('retry_interval').get_parameter_value().double_value

        self.get_logger().info(
            f'fast_lio2_param_injector 启动\n'
            f'  目标节点       : /{self._target_node}\n'
            f'  pcd_save_en    : {self._pcd_save_en}\n'
            f'  map_file_path  : {self._map_file_path!r}\n'
            f'  pcd_save_interval: {self._interval}'
        )

        # 使用定时器触发注入（给 fast_lio2 节点足够的启动时间）
        self._attempt = 0
        self._timer = self.create_timer(self._retry_interval, self._try_inject)

    def _try_inject(self) -> None:
        self._attempt += 1
        srv_name = f'/{self._target_node}/set_parameters'

        cli = self.create_client(SetParameters, srv_name)

        if not cli.wait_for_service(timeout_sec=0.5):
            if self._attempt >= self._retry_times:
                self.get_logger().error(
                    f'[注入失败] {srv_name} 服务在 {self._retry_times} 次重试后仍不可用。\n'
                    f'  fast_lio2 节点可能未启动，请检查 localization.launch.py。\n'
                    f'  ⚠ pcd_save_en 未能设置为 {self._pcd_save_en}，'
                    f'需手动在 fast_lio2_params.yaml 中配置。'
                )
                self._timer.cancel()
                # 非致命错误：打印警告后退出，不阻断整个系统
                rclpy.shutdown()
            else:
                self.get_logger().info(
                    f'等待 {srv_name} 服务（第 {self._attempt}/{self._retry_times} 次）...')
            return

        # 构造参数列表
        params = []

        # pcd_save_en（bool）
        p_save = Parameter()
        p_save.name = 'pcd_save_en'
        p_save.value.type = ParameterType.PARAMETER_BOOL
        p_save.value.bool_value = self._pcd_save_en
        params.append(p_save)

        # map_file_path（string）
        if self._map_file_path:
            p_path = Parameter()
            p_path.name = 'map_file_path'
            p_path.value.type = ParameterType.PARAMETER_STRING
            p_path.value.string_value = self._map_file_path
            params.append(p_path)

        # pcd_save_interval（int）—— 仅建图模式写入
        if self._pcd_save_en:
            p_interval = Parameter()
            p_interval.name = 'pcd_save_interval'
            p_interval.value.type = ParameterType.PARAMETER_INTEGER
            p_interval.value.integer_value = self._interval
            params.append(p_interval)

        req = SetParameters.Request()
        req.parameters = params

        future = cli.call_async(req)
        # 等待响应（最多 3s）
        deadline = time.time() + 3.0
        while not future.done() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

        self._timer.cancel()

        if future.done():
            resp = future.result()
            all_ok = all(r.successful for r in resp.results)
            if all_ok:
                self.get_logger().info(
                    f'[注入成功] fast_lio2 参数已设置：\n'
                    f'  pcd_save_en={self._pcd_save_en}，'
                    f'  map_file_path={self._map_file_path!r}，'
                    f'  interval={self._interval}'
                )
            else:
                for i, r in enumerate(resp.results):
                    if not r.successful:
                        self.get_logger().warn(
                            f'[注入部分失败] 参数 {params[i].name}：{r.reason}')
        else:
            self.get_logger().warn(
                f'[注入超时] {srv_name} 调用超时，fast_lio2 pcd_save 参数可能未生效')

        # one-shot：注入完成后关闭节点
        rclpy.shutdown()


# ---------------------------------------------------------------------------
def main(args=None) -> None:
    rclpy.init(args=args)
    node = FastLio2ParamInjector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
