# -*- coding: utf-8 -*-
"""gen_sil_waypoints.py 单测（V0.2.0，纯逻辑，不需要 carla 包与 server）。

覆盖生成器的三条"错了不报错、航点直接出界"的约定：
1. 坐标转换回退式必须与 sil_bridge_agent.codec **逐点一致**——生成器在仿真机上
   独立运行时用的是内置回退，而车端 /Odometry 真值由 codec 翻系，两者漂移一个
   符号整套航点作废；
2. 贪心筛选必须满足两两间距下限（auto_mission 相邻航点 ≥2m 契约的来源）；
3. 渲染的 YAML 必须是 auto_mission 的 ROS 参数契约结构。
"""
from __future__ import annotations

import importlib.util
import math
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from sil_bridge_agent.codec import (                        # noqa: E402
    carla_position_to_ros,
    carla_euler_to_ros,
)

_TOOLS = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      "..", "tools", "gen_sil_waypoints.py")
_spec = importlib.util.spec_from_file_location("gen_sil_waypoints", _TOOLS)
gwp = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(gwp)


class _Loc:
    def __init__(self, x, y):
        self.x, self.y, self.z = float(x), float(y), 0.0


class _Rot:
    def __init__(self, yaw):
        self.yaw, self.pitch, self.roll = float(yaw), 0.0, 0.0


class _TF:
    def __init__(self, x, y, yaw_deg=0.0):
        self.location = _Loc(x, y)
        self.rotation = _Rot(yaw_deg)


class _Map:
    def __init__(self, name, spawns):
        self.name = "/Carla/Maps/%s" % name
        self._spawns = spawns

    def get_spawn_points(self):
        return self._spawns


class _World:
    def __init__(self, cmap):
        self._map = cmap

    def get_map(self):
        return self._map


class _Client:
    """假 CARLA client：load_world 切图、记录调用序列（断言恢复原图）。"""

    def __init__(self, maps, initial):
        self._maps = maps
        self.current = initial
        self.loaded = []

    def get_world(self):
        return _World(self._maps[self.current])

    def load_world(self, name):
        self.loaded.append(name)
        self.current = name


# ---------------------------------------------------------------------------
# 坐标转换回退式 ≡ codec（防双实现漂移）
# ---------------------------------------------------------------------------
def test_xy_transform_matches_codec() -> None:
    for x, y in [(1.0, 2.0), (-3.5, 118.0), (0.0, 0.0), (12.3, -45.6)]:
        assert gwp.carla_to_ros_xy(x, y) == carla_position_to_ros(x, y, 0.0)[:2]


def test_yaw_transform_matches_codec() -> None:
    for yaw_deg in [0.0, 90.0, -45.0, 180.0, 270.5]:
        got = gwp.carla_yaw_deg_to_ros_rad(yaw_deg)
        expect = carla_euler_to_ros(0.0, 0.0, math.radians(yaw_deg))[2]
        assert abs(got - expect) < 1e-12


# ---------------------------------------------------------------------------
# 贪心筛选
# ---------------------------------------------------------------------------
def test_select_respects_count_and_min_gap() -> None:
    spawns = [_TF(x * 10.0, 0.0) for x in range(20)]          # 间距 10m 一排
    chosen = gwp.select_spawn_points(spawns, count=5, min_gap=9.5)
    assert len(chosen) == 5
    pts = [(t.location.x, t.location.y) for t in chosen]
    for i in range(len(pts)):
        for j in range(i + 1, len(pts)):
            assert math.dist(pts[i], pts[j]) >= 9.5


def test_select_skips_close_neighbors() -> None:
    spawns = [_TF(0.0, 0.0), _TF(1.0, 0.0), _TF(2.0, 0.0), _TF(30.0, 0.0)]
    chosen = gwp.select_spawn_points(spawns, count=8, min_gap=5.0)
    assert [t.location.x for t in chosen] == [0.0, 30.0]      # 1/2 被 0 吸收


def test_select_empty_pool() -> None:
    assert gwp.select_spawn_points([], count=4, min_gap=10.0) == []


# ---------------------------------------------------------------------------
# YAML 渲染契约
# ---------------------------------------------------------------------------
def test_render_yaml_structure_and_flip() -> None:
    text = gwp.render_yaml("Town03", [_TF(6.0, -118.0, 90.0)])
    assert "auto_mission_node:" in text and "ros__parameters:" in text
    assert "cruise_max_speed:" in text                        # 巡航参数与模板同源
    assert "waypoints:" in text
    # ROS 侧坐标：y 取反（-118 → +118），yaw 取反（+90° → -1.5708 rad）
    assert '"6.00,118.00,-1.5708,检查点1"' in text


def test_render_yaml_cruise_params_complete() -> None:
    text = gwp.render_yaml("Town03", [_TF(1.0, 2.0)])
    for key, _ in gwp.CRUISE_PARAMS:
        assert "%s:" % key in text
    assert text.count("      - ") == 1


# ---------------------------------------------------------------------------
# generate 全流程（假 client）
# ---------------------------------------------------------------------------
def test_generate_writes_file_and_restores_map() -> None:
    import tempfile
    spawns = [_TF(x * 50.0, 10.0, 0.0) for x in range(6)]
    maps = {"Town01": _Map("Town01", [_TF(0.0, 0.0)]),
            "Town02": _Map("Town02", spawns)}
    client = _Client(maps, initial="Town01")
    with tempfile.TemporaryDirectory() as td:
        n = gwp.generate(client, ["Town02"], count=4, min_gap=20.0, out_dir=td)
        assert n == 1
        path = os.path.join(td, "sil_waypoints_Town02.yaml")
        assert os.path.isfile(path)
        with open(path, encoding="utf-8") as fh:
            body = fh.read()
        assert "地图：Town02" in body and body.count("      - ") == 4
        # 遍历过非当前图后必须恢复运行前地图（不打断仿真机现有会话场景）
        assert client.loaded == ["Town02", "Town01"]


def test_generate_skips_map_with_no_selectable_points() -> None:
    import tempfile
    maps = {"Town04": _Map("Town04", [])}
    client = _Client(maps, initial="Town04")
    with tempfile.TemporaryDirectory() as td:
        n = gwp.generate(client, ["Town04"], count=4, min_gap=20.0, out_dir=td)
        assert n == 0
        assert not os.path.exists(os.path.join(td, "sil_waypoints_Town04.yaml"))
        assert client.loaded == []                            # 本来就是当前图，不切


def main() -> int:
    tests = [(name, fn) for name, fn in sorted(globals().items())
             if name.startswith("test_") and callable(fn)]
    for name, fn in tests:
        fn()
        print("PASS %s" % name)
    print("GEN_WP_OK: %d tests" % len(tests))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
