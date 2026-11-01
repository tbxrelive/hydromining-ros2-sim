"""墙面种子划分与工况集合：训练/验证/测试必须使用不相交的墙面。

这里是实验侧访问场景库的唯一入口。真正的生成逻辑在 `scenarios.py`，
本模块负责对外暴露划分、种子与回合计数的派生函数。

划分（互不相交）：
    训练 10–19（含旧 10–12）  验证 30–32  测试 20–25（含旧 20–22）

旧基线（E0.3 / E2.1c）仍使用 `LEGACY_WALL_SEEDS`，保证逐回合可比。
"""
from pathlib import Path

from .scenarios import (  # noqa: F401
    FAMILIES, LEGACY_WALL_SEEDS, STRENGTH_SEEDS, WALL_SEEDS, Scenario,
    build_library, episode_seed, generate, load_library, save_library, wall_seed)

# 旧名字保留，便于既有脚本继续工作。
TRAIN_SEEDS = LEGACY_WALL_SEEDS['train']
TEST_SEEDS = LEGACY_WALL_SEEDS['test']
DEFAULT_LIBRARY_PATH = Path(__file__).resolve().parents[2]/'results/thesis/E2.5/library.json'


def seeds(split):
    """该划分的完整墙面种子列表。"""
    if split not in WALL_SEEDS:
        raise ValueError(f'unknown split: {split}')
    return WALL_SEEDS[split]


def legacy_seeds(split):
    """旧基线使用的墙面子集（训练 10–12 / 测试 20–22）。"""
    if split not in LEGACY_WALL_SEEDS:
        raise ValueError(f'no legacy seeds for split: {split}')
    return LEGACY_WALL_SEEDS[split]


def scenarios(split, path=DEFAULT_LIBRARY_PATH):
    """返回该划分的场景列表；库文件不存在时现场生成（结果确定）。"""
    path = Path(path)
    if path.exists():
        return load_library(path)[split]
    return build_library()[split]


def mesh_path(assets, seed):
    return Path(assets)/f'coal_{seed}.obj'


def assert_disjoint():
    """划分之间不得共用墙面种子。"""
    names = list(WALL_SEEDS)
    for i, left in enumerate(names):
        for right in names[i+1:]:
            overlap = set(WALL_SEEDS[left]) & set(WALL_SEEDS[right])
            if overlap:
                raise AssertionError(f'{left} 与 {right} 共用墙面种子: {sorted(overlap)}')
    return True
