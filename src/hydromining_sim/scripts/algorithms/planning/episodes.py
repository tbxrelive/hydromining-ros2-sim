"""阶段七回合定义：固定评估区 + 预喷残差工况，供替身环境与真实验收共用。

每个工况是墙面局部坐标 (u, v) 上的喷射点与时长列表；期望达标比例带由
离线真值标定记录，不作为感知输入。
"""
import numpy as np

from .surrogate import spray_target_tcp


def ring_shots(center, radius, count, duration=12.):
    """绕 center 的环形喷射点；count=0 退化为单点。"""
    if not count:
        return [(tuple(center), duration)]
    return [(tuple(np.asarray(center) + radius * np.array(
        [np.cos(2 * np.pi * k / count), np.sin(2 * np.pi * k / count)])), duration)
        for k in range(count)]


def apply_scenario(world, frame, normal_world, shots, standoff, pressure, max_duration=None):
    """按工况在替身世界上顺序喷射；返回每发统计。"""
    results = []
    for uv, duration in shots:
        tcp = spray_target_tcp(frame, np.asarray(uv, float), normal_world, standoff)
        results.append(world.spray_tcp(tcp, pressure, min(duration, max_duration) if max_duration else duration))
    return results
