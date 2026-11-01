"""场景库：墙面种子划分、初始工况生成、回合种子派生。

设计要点
- **划分不相交**：训练/验证/测试的墙面种子互不重叠，训练集含原有的
  10–12，测试集含原有的 20–22，保证旧结果仍是新库的子集。
- **工况由种子唯一确定**：同一种子在同一进程和新进程里都必须生成逐位相同的
  高度场，因此全部随机量都走 `np.random.default_rng(派生种子)`。
- **回合种子不依赖调用顺序**：用 SHA256 从 (划分, 工况名, 重复序号) 派生。
"""
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path

import numpy as np

# 墙面种子划分；训练含旧 10–12，测试含旧 20–22。
WALL_SEEDS = {'train': tuple(range(10, 20)), 'val': (30, 31, 32),
              'test': tuple(range(20, 26))}
# 旧基线使用的墙面子集，保持 E0.3/E2.1c 的逐回合可比性。
LEGACY_WALL_SEEDS = {'train': (10, 11, 12), 'test': (20, 21, 22)}
# E2.2 非均质强度场种子：此处只预留，不生成。
STRENGTH_SEEDS = tuple(range(100, 140))

# 工况族：A 组是随机初始凹坑，B 组是给阶段 4 的深坑自遮挡。
FAMILIES = ('empty', 'single', 'pair', 'triangle', 'deep_pit')
FAMILY_SPEC = {
    'empty': {'shots': 0},
    'single': {'shots': 1, 'duration': (4., 8.)},
    'pair': {'shots': 2, 'duration': (4., 8.), 'min_separation': .10},
    'triangle': {'shots': 3, 'duration': (4., 8.), 'min_separation': .10},
    # 同点两发 4 s：单发标称孔深约 75 mm，两发约 130–150 mm，
    # 远小于 400 mm 壁厚，保证"深坑但绝不打穿"（8 s×2 在凸起墙面上会打穿）。
    'deep_pit': {'shots': 2, 'duration': (4., 4.), 'same_point': True},
}
MAX_SHOT_RADIUS_M = .18


@dataclass(frozen=True)
class Scenario:
    """一个初始工况：名称、族、喷射序列 ((u, v), 时长)。"""

    name: str
    family: str
    shots: tuple

    def as_dict(self):
        return {'name': self.name, 'family': self.family,
                'shots': [[list(uv), float(d)] for uv, d in self.shots]}


def _digest(*parts):
    """把若干字段散列成 64 位整数种子。"""
    text = '/'.join(str(part) for part in parts)
    return int.from_bytes(hashlib.sha256(text.encode('utf-8')).digest()[:8], 'little')


def episode_seed(split, scenario_name, repeat):
    """回合随机种子：只由划分/工况/重复序号决定，与调用顺序无关。"""
    return _digest('episode', split, scenario_name, repeat)


def wall_seed(split, scenario_name):
    """工况对应的墙面种子（同一划分内按序号轮转）。"""
    seeds = WALL_SEEDS[split]
    return seeds[_scenario_index(scenario_name) % len(seeds)]


def _scenario_index(name):
    """从 `sc_<split>_<序号>` 里取序号。"""
    return int(name.rsplit('_', 1)[-1])


def generate(split, family, index, wall):
    """生成一个工况；同样的 (划分, 族, 序号, 墙面) 必得同一结果。"""
    if split not in WALL_SEEDS:
        raise ValueError(f'unknown split: {split}')
    if family not in FAMILY_SPEC:
        raise ValueError(f'unknown family: {family}')
    spec = FAMILY_SPEC[family]
    name = f'sc_{split}_{index:03d}'
    rng = np.random.default_rng(_digest('scenario', split, family, index, wall))
    count = spec['shots']
    low, high = spec.get('duration', (0., 0.))
    minimum = spec.get('min_separation', 0.)
    if spec.get('same_point'):
        uv = _random_point(rng, np.empty((0, 2)), minimum)
        shots = tuple((uv, float(rng.uniform(low, high))) for _ in range(count))
    else:
        placements = []
        for _ in range(count):
            placements.append(_random_point(rng, np.array(placements).reshape(-1, 2), minimum))
        shots = tuple((np.asarray(uv), float(rng.uniform(low, high)))
                      for uv in placements)
    return Scenario(name, family, shots)


def _random_point(rng, taken, minimum=0.):
    """在任务区圆内均匀取点，并与已有点保持最小间距。"""
    for _ in range(200):
        radius = MAX_SHOT_RADIUS_M*np.sqrt(rng.uniform())
        angle = rng.uniform(0., 2*np.pi)
        point = np.array([radius*np.cos(angle), radius*np.sin(angle)])
        if taken.size == 0 or minimum <= 0 or \
                np.min(np.linalg.norm(taken-point, axis=1)) >= minimum:
            return point
    raise RuntimeError('无法在任务区内找到满足间距的喷射点')


def build_library():
    """按族 × 墙面 × 划分展开成完整场景库。"""
    library = {}
    for split, seeds in WALL_SEEDS.items():
        scenarios = []
        index = 0
        for family in FAMILIES:
            for seed in seeds:
                scenarios.append(generate(split, family, index, seed))
                index += 1
        library[split] = scenarios
    return library


def as_shots(scenario):
    """转成 L0/L1/L2 通用的 shots 格式：((u, v), duration)。"""
    return tuple((tuple(float(v) for v in uv), float(d)) for uv, d in scenario.shots)


def library_dict(library):
    """可写入 JSON 的库清单。"""
    return {split: [scenario.as_dict() for scenario in scenarios]
            for split, scenarios in library.items()}


def save_library(path, library):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(library_dict(library), indent=2, ensure_ascii=False))
    return path


def load_library(path):
    values = json.loads(Path(path).read_text())
    return {split: [Scenario(item['name'], item['family'],
                             tuple((np.asarray(uv, float), float(d)) for uv, d in item['shots']))
                    for item in scenarios]
            for split, scenarios in values.items()}
