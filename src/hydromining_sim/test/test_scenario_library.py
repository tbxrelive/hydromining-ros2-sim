"""场景库的复现性与划分约束单测。"""
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(Path(__file__).parents[1]/'scripts'))
sys.path.insert(0, str(ROOT))

from experiments.common import scenarios as scenario_lib  # noqa: E402
from experiments.common import splits  # noqa: E402


def test_splits_are_disjoint():
    """训练/验证/测试不得共用墙面种子。"""
    assert splits.assert_disjoint()
    all_seeds = [s for seeds in scenario_lib.WALL_SEEDS.values() for s in seeds]
    assert len(all_seeds) == len(set(all_seeds))


def test_legacy_seeds_are_subset_of_new_splits():
    """旧基线用的墙面必须仍在新划分里，保证旧结果可比。"""
    assert set(splits.legacy_seeds('train')) <= set(splits.seeds('train'))
    assert set(splits.legacy_seeds('test')) <= set(splits.seeds('test'))


def test_library_is_deterministic():
    """同一进程两次生成必须完全一致。"""
    first = scenario_lib.build_library()
    second = scenario_lib.build_library()
    assert scenario_lib.library_dict(first) == scenario_lib.library_dict(second)
    assert len(first['train']) == 50 and len(first['val']) == 15 and len(first['test']) == 30


def test_episode_seed_is_stable_and_order_independent():
    """回合种子只由三元组决定，与调用顺序无关。"""
    a = scenario_lib.episode_seed('test', 'sc_test_000', 0)
    b = scenario_lib.episode_seed('test', 'sc_test_000', 0)
    c = scenario_lib.episode_seed('test', 'sc_test_000', 1)
    assert a == b and a != c


def test_scenario_shots_are_inside_task_region():
    """所有喷射点都落在作业圆内。"""
    library = scenario_lib.build_library()
    for split, items in library.items():
        for scenario in items:
            for (u, v), duration in scenario_lib.as_shots(scenario):
                assert np.hypot(u, v) <= scenario_lib.MAX_SHOT_RADIUS_M + 1e-9
                assert 0 < duration <= 8.


def test_scenario_round_trip(tmp_path):
    """写盘再读回，喷射序列保持一致。"""
    library = scenario_lib.build_library()
    path = scenario_lib.save_library(tmp_path/'library.json', library)
    reloaded = scenario_lib.load_library(path)
    for split in library:
        for original, restored in zip(library[split], reloaded[split]):
            assert original.name == restored.name
            assert original.family == restored.family
            assert len(original.shots) == len(restored.shots)
            for (uv_a, d_a), (uv_b, d_b) in zip(original.shots, restored.shots):
                assert np.allclose(uv_a, uv_b) and d_a == d_b


def test_generate_rejects_unknown_inputs():
    with pytest.raises(ValueError):
        scenario_lib.generate('nope', 'single', 0, 10)
    with pytest.raises(ValueError):
        scenario_lib.generate('test', 'nope', 0, 10)
