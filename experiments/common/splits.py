"""墙面种子划分与工况集合；训练/测试必须使用不相交的墙面。"""
from pathlib import Path

TRAIN_SEEDS = (10, 11, 12)
TEST_SEEDS = (20, 21, 22)


def scenarios(split):
    """返回该划分的工况定义（来自 L0 环境模块，避免重复维护）。"""
    from algorithms.planning.fast_heightfield import TEST_SCENARIOS, TRAIN_SCENARIOS
    if split == 'train':
        return dict(TRAIN_SCENARIOS)
    if split == 'test':
        return dict(TEST_SCENARIOS)
    raise ValueError(f'unknown split: {split}')


def seeds(split):
    return TRAIN_SEEDS if split == 'train' else TEST_SEEDS


def mesh_path(assets, seed):
    return Path(assets)/f'coal_{seed}.obj'
