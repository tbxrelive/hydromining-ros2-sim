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


# 预定义残差工况；fraction 为替身环境下按固定法向实测的 20cm 区域达标
# 面积比例（20 MPa、靶距 0.7 m、单煤壁确定性模型的工程值；随 level-set
# 网格分辨率略有变化，96/64 网格下 tri_r5 约 0.83/0.85，仅作工况标注）。
SCENARIOS = {
    'empty': {'shots': [], 'fraction': 0.000, 'done': False},
    'single': {'shots': [((0., 0.), 12.)], 'fraction': 0.310, 'done': False},
    'pair': {'shots': [((-0.04, 0.), 12.), ((0.04, 0.), 12.)], 'fraction': 0.594,
             'done': False},
    'tri_r5': {'shots': ring_shots((0., 0.), 0.05, 3), 'fraction': 0.832,
               'done': False},
    # 开局即达标的工况：验证策略能及时收尾而不是继续喷。
    'ring_r6': {'shots': ring_shots((0., 0.), 0.06, 6), 'fraction': 1.000,
                'done': True},
    'dense': {'shots': [((0., 0.), 12.)] + ring_shots((0., 0.), 0.06, 6),
              'fraction': 1.000, 'done': True},
}


def apply_scenario(world, frame, normal_world, shots, standoff=0.7):
    """按工况在替身世界上顺序喷射；返回每发统计。"""
    results = []
    for uv, duration in shots:
        tcp = spray_target_tcp(frame, np.asarray(uv, float), normal_world, standoff)
        results.append(world.spray_tcp(tcp, 20., duration))
    return results


def sample_episode(rng, scenarios=SCENARIOS):
    """按均匀分布抽一个工况；返回 (名称, 定义)。"""
    name = rng.choice(list(scenarios))
    return name, scenarios[name]
