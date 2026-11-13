"""SIL Bridge Agent — HunterSim ↔ HunterEdge 真 SIL 桥接（V0.1.38）。

模块划分（便于在无 ROS 环境单测最易出错的纯逻辑）：
* :mod:`sil_bridge_agent.codec`   坐标系转换 + 信封/payload 编解码（无 ROS 依赖）
* :mod:`sil_bridge_agent.session` 会话绑定 + 断流看门狗 + 控制序号（无 ROS 依赖）
* :mod:`sil_bridge_agent.sil_bridge_agent` ROS2 节点 + Kafka 收发线程
"""

__version__ = "1.0.0"
