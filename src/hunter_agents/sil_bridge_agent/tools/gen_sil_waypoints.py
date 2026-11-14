#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gen_sil_waypoints.py — SIL 分地图航点批量生成器（运行在 **CARLA 仿真机**，V0.2.0）

为什么必须有这个脚本
--------------------
CARLA 各 Town 的车道坐标**没有可信的静态表**：不同 CARLA 版本（0.9.12 vs 0.9.13+）
连地图集合与坐标都可能不同，任何硬编码到仓库/文档里的"参考坐标"都会随版本失效。
唯一正确的来源是**实际运行的 CARLA server**：``map.get_spawn_points()`` 返回的
出生点全部位于行车道上（可通行、无碰撞），是天然的候选航点池。

用法（仿真机，需 ``pip install carla==<server版本>``，CARLA server 已在运行）
----------------------------------------------------------------------------
    # 为 server 支持的全部内置地图各生成一个文件
    python gen_sil_waypoints.py --maps all --out-dir ./sil_waypoints.d

    # 只生成指定地图
    python gen_sil_waypoints.py --maps Town03 Town10HD --out-dir ./sil_waypoints.d

产物布局（文件名契约与 sil_bridge ``waypoints_map_dir`` 一致，勿改名）：
    sil_waypoints.d/sil_waypoints_Town01.yaml ... sil_waypoints_Town15.yaml

部署：把整个目录放到 **Jetson** 的 /etc/hunter/sil_waypoints.d/（见部署文档 §9.3）。
之后 sil_bridge 在仿真会话 boot 时按 boot 帧 ``map_id`` 自动把对应文件覆盖到
活动航点文件（默认 /etc/hunter/sil_waypoints.yaml），auto_mission 在
start_mapping_cruise 时热重载——**换地图后需重新发起巡航**才会装载新航点。

坐标系纪律（错一个符号航点就出界）
----------------------------------
输出坐标是 **ROS 侧**（与 sil_bridge 发布的 /Odometry 真值同源），不是 CARLA 原始
坐标：CARLA 左手系 → ROS 右手系做 Y 反射，``x_ros = x``、``y_ros = -y``、
``yaw_ros = -yaw_carla``。规则实现直接复用 ``sil_bridge_agent.codec``（同机装了
HunterEdge 时），未安装时使用内置同式回退——单测保证两者逐点一致。

⚠ 过程会 ``load_world`` 逐图切换 server 地图（读取 spawn points 必须先加载该图），
**不要在正在进行 SIL 会话的 server 上运行**；脚本结束会恢复运行前的地图。
"""
from __future__ import annotations

import argparse
import datetime
import math
import os
import sys
import time
from typing import Any, List, Optional

# ─── 坐标转换：优先复用 codec（唯一事实源），独立运行时用同式回退 ───────────
try:
    from sil_bridge_agent.codec import (  # type: ignore
        carla_position_to_ros, carla_euler_to_ros)

    def carla_to_ros_xy(x: float, y: float) -> tuple:
        rx, ry, _ = carla_position_to_ros(x, y, 0.0)
        return (rx, ry)

    def carla_yaw_deg_to_ros_rad(yaw_deg: float) -> float:
        # codec 的欧拉转换作用于弧度制的 yaw 分量（伪向量规则 → 取反）
        return carla_euler_to_ros(0.0, 0.0, math.radians(yaw_deg))[2]
except ImportError:  # 仿真机上没有 HunterEdge 包：同式回退（勿单独改动）
    def carla_to_ros_xy(x: float, y: float) -> tuple:
        return (float(x), -float(y))

    def carla_yaw_deg_to_ros_rad(yaw_deg: float) -> float:
        return -math.radians(float(yaw_deg))

#: 巡航段参数：与 autonomous_nav_params.yaml / sil_waypoints.example.yaml 同源，
#: 改动必须三处同步（auto_mission 启动巡航时热重载的是本文件产出的整段）。
CRUISE_PARAMS = [
    ("cruise_max_speed", 0.1),      # m/s，SIL 保守速度
    ("cruise_standoff_m", 1.5),
    ("cruise_curb_margin_m", 1.0),
    ("waypoint_xytol", 0.5),
    ("waypoint_ytol", 0.5),
    ("waypoint_yaw_deg_tol", 15.0),
    ("waypoint_timeout_s", 180.0),
    ("cruise_pause_speed_eps", 0.05),
    ("cruise_pause_stall_s", 8.0),
    ("cruise_pause_reset_dist_m", 3.0),
    ("cruise_heartbeat_s", 2.0),
]

#: --maps all 时的候选内置图（实际以 server 的 get_available_maps() 过滤为准）
BUILTIN_MAPS = [
    "Town01", "Town02", "Town03", "Town04", "Town05", "Town06", "Town07",
    "Town10HD", "Town11", "Town12", "Town13", "Town15",
]


def select_spawn_points(spawns: List[Any], count: int, min_gap: float) -> List[Any]:
    """从出生点池贪心筛选：彼此欧氏距离 ≥ min_gap，最多 count 个。

    CARLA 出生点本身就在行车道上且散布全图，贪心间隔筛选即可得到
    "相邻间距 ≥ 2m"（auto_mission 约束）的可达航点，无需路径搜索。
    """
    chosen: List[Any] = []
    for tf in spawns:
        loc = tf.location
        if all(math.hypot(loc.x - c.location.x, loc.y - c.location.y) >= min_gap
               for c in chosen):
            chosen.append(tf)
            if len(chosen) >= count:
                break
    return chosen


def render_yaml(map_name: str, transforms: List[Any]) -> str:
    """按 auto_mission ROS 参数契约渲染一个地图的航点文件。"""
    stamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    lines = [
        "# sil_waypoints_%s.yaml — 由 gen_sil_waypoints.py 自动生成（勿手改坐标）" % map_name,
        "# 生成时间：%s | 地图：%s | 航点数：%d" % (stamp, map_name, len(transforms)),
        "# 坐标系：ROS 侧（CARLA Y 反射：x_ros=x, y_ros=-y, yaw_ros=-yaw_carla），",
        "#         与 sil_bridge 发布的 /Odometry 真值同源，auto_mission 直接消费。",
        "# 装载：sil_bridge boot 按 map_id 覆盖到活动文件 → start_mapping_cruise 热重载。",
        "auto_mission_node:",
        "  ros__parameters:",
    ]
    for key, value in CRUISE_PARAMS:
        lines.append("    %s: %s" % (key, value))
    lines.append("    waypoints:")
    for idx, tf in enumerate(transforms, start=1):
        rx, ry = carla_to_ros_xy(tf.location.x, tf.location.y)
        yaw_rad = carla_yaw_deg_to_ros_rad(tf.rotation.yaw)
        lines.append('      - "%.2f,%.2f,%.4f,检查点%d"' % (rx, ry, yaw_rad, idx))
    return "\n".join(lines) + "\n"


def _load_world(client: Any, name: str, retries: int = 1,
                pause_s: float = 15.0) -> None:
    """带重试的 load_world：超大图（Town12/13/15）或 server 边下载资源边加载时，
    RPC 超时时 server 往往仍在后台加载——等一会先复查当前图，已经切到位就不重发。"""
    for attempt in range(retries + 1):
        try:
            client.load_world(name)
            return
        except RuntimeError as exc:
            if attempt >= retries:
                raise
            print("  [WARN] 加载 %s 超时（%s），等 %.0fs 后复查/重试"
                  "（server 可能仍在后台加载）…" % (name, exc, pause_s))
            time.sleep(pause_s)
            try:  # 超时≠失败：先确认是否其实已经切到位
                if client.get_world().get_map().name.rsplit("/", 1)[-1] == name:
                    print("  [OK] %s 实际已加载完成，继续" % name)
                    return
            except RuntimeError:
                pass  # 还在忙，下一轮重试 load 兜着


def generate(client: Any, maps: List[str], count: int, min_gap: float,
             out_dir: str, load_retries: int = 1,
             load_pause_s: float = 15.0) -> int:
    """逐图 load_world → 筛点 → 写文件；单图失败只跳过不中断整批。

    返回成功生成的文件数。"""
    os.makedirs(out_dir, exist_ok=True)
    original = client.get_world().get_map().name.rsplit("/", 1)[-1]
    written = 0
    failed: List[str] = []
    try:
        for name in maps:
            try:
                loaded = client.get_world().get_map().name.rsplit("/", 1)[-1]
                if loaded != name:
                    print("  切换地图 %s → %s（读取出生点需先加载）" % (loaded, name))
                    _load_world(client, name, retries=load_retries,
                                pause_s=load_pause_s)
                spawns = client.get_world().get_map().get_spawn_points()
                chosen = select_spawn_points(spawns, count, min_gap)
                if not chosen:
                    print("  [WARN] %s 筛不出任何航点（count=%d min-gap=%.1f 过严），跳过"
                          % (name, count, min_gap))
                    continue
                if len(chosen) < min(count, 2):
                    print("  [WARN] %s 仅筛出 %d 个航点（min-gap=%.1fm 过大或出生点稀疏）"
                          % (name, len(chosen), min_gap))
                path = os.path.join(out_dir, "sil_waypoints_%s.yaml" % name)
                with open(path, "w", encoding="utf-8", newline="\n") as fh:
                    fh.write(render_yaml(name, chosen))
                written += 1
                print("  [OK] %-10s %2d 航点 → %s" % (name, len(chosen), path))
            except Exception as exc:  # noqa: BLE001（单图任何失败不得中断整批）
                failed.append(name)
                print("  [FAIL] %s 生成失败，跳过继续下一图：%s" % (name, exc))
        if failed:
            print("[WARN] 失败地图：%s ——可单独重跑：--maps %s"
                  % (", ".join(failed), " ".join(failed)))
    finally:
        try:
            current = client.get_world().get_map().name.rsplit("/", 1)[-1]
        except RuntimeError:
            current = ""    # server 尚在原图后台加载：不催它，交给下面的 load 兜底
        if original and current != original:
            print("  恢复运行前地图：%s" % original)
            try:
                client.load_world(original)
            except Exception as exc:  # noqa: BLE001
                print("  [WARN] 恢复地图失败（不影响已生成文件），请在仿真机手动切回 %s：%s"
                      % (original, exc))
    return written


def _server_version(client: Any) -> str:
    """server 版本字符串（仅展示用，取不到不阻塞生成）。

    CARLA Python API 的 Client 方法名跨版本不稳定：0.9.16 实测为
    `get_server_version()`（无 get_service_version，也无 World.get_client），
    旧版为 `get_client().get_version()`——故按存在性探测。
    """
    for attr in ("get_server_version", "get_service_version"):
        fn = getattr(client, attr, None)
        if fn is not None:
            try:
                return str(fn())
            except Exception:  # noqa: BLE001
                pass
    try:
        return str(client.get_client().get_version())
    except Exception:  # noqa: BLE001
        return "unknown"


def main() -> int:
    parser = argparse.ArgumentParser(
        description="SIL 分地图航点批量生成器（须在有 CARLA server 的仿真机上运行）")
    parser.add_argument("--host", default="127.0.0.1", help="CARLA server 地址")
    parser.add_argument("--port", type=int, default=2000, help="CARLA RPC 端口")
    parser.add_argument("--maps", nargs="+", default=["all"],
                        help="地图名列表，或 all=server 支持的全部内置图（%s）"
                             % "/".join(BUILTIN_MAPS))
    parser.add_argument("--count", type=int, default=8, help="每图航点数上限（默认 8）")
    parser.add_argument("--min-gap", type=float, default=20.0, dest="min_gap",
                        help="航点最小两两间距（米，默认 20；须 ≥ auto_mission 的 2m 下限）")
    parser.add_argument("--out-dir", default="./sil_waypoints.d", dest="out_dir",
                        help="输出目录（部署到 Jetson /etc/hunter/sil_waypoints.d/）")
    parser.add_argument("--timeout", type=float, default=60.0,
                        help="RPC 超时（秒，默认 60；Town12/13/15 等超大图首次加载"
                             "远慢于小图，15s 必超时）")
    args = parser.parse_args()

    if args.min_gap < 2.0:
        parser.error("--min-gap 不得小于 2.0m（auto_mission 相邻航点间距下限）")

    try:
        import carla  # noqa: PLC0415（仅仿真机可导入，报错信息更有用）
    except ImportError:
        print("[ERROR] 本机无 carla 包：pip install carla==<与 server 一致的版本>",
              file=sys.stderr)
        return 2

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(args.timeout)
        client.get_world()          # 连通性验证
    except Exception as exc:  # noqa: BLE001
        print("[ERROR] 连不上 CARLA %s:%s ——%s" % (args.host, args.port, exc),
              file=sys.stderr)
        return 2

    available = [m.rsplit("/", 1)[-1] for m in client.get_available_maps()]
    want = ([m for m in BUILTIN_MAPS if m in available]
            if args.maps == ["all"] else args.maps)
    unknown = [m for m in want if m not in available]
    if unknown:
        print("[WARN] server 不支持的地图，跳过：%s（可用：%s）"
              % (unknown, ", ".join(available)), file=sys.stderr)
        want = [m for m in want if m in available]
    if not want:
        print("[ERROR] 没有可生成的地图", file=sys.stderr)
        return 2

    print("CARLA %s | 目标地图：%s" % (_server_version(client), ", ".join(want)))
    n = generate(client, want, args.count, args.min_gap, args.out_dir)
    print("完成：%d/%d 个文件 → %s" % (n, len(want), args.out_dir))
    print("下一步：整个目录部署到 Jetson /etc/hunter/sil_waypoints.d/（scp -r 即可）")
    return 0 if n else 1


if __name__ == "__main__":
    sys.exit(main())
