"""干扰等级注册表与受扰比例度量的单测。"""
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(Path(__file__).parents[1]/'scripts'))
sys.path.insert(0, str(ROOT))

from algorithms.jet_effects import cloud_change  # noqa: E402
from experiments.common.interference import (  # noqa: E402
    InterferenceLevel, disturbed_fraction, load_levels, sensor_tolerance)


def test_levels_are_ordered_and_loaded():
    """等级表按 无 < 低 < 中 < 高 排列，能见度递减。"""
    levels = load_levels()
    names = list(levels)
    assert names == ['none', 'low', 'medium', 'high']
    visibility = [levels[name].visibility_m for name in names]
    assert np.isinf(visibility[0])
    assert all(a > b for a, b in zip(visibility[1:-1], visibility[2:]))
    assert [levels[name].label for name in names] == ['无', '低', '中', '高']


def test_sensor_tolerance_respects_floor_and_grows_with_range():
    """近距取固定下限，远距随 σ=1%Z² 增长。"""
    near = sensor_tolerance(np.array([.3]), 'depth')[0]
    far = sensor_tolerance(np.array([3.]), 'depth')[0]
    assert near == pytest.approx(.01)
    assert far > near
    # LiDAR 的 3σ 恒为 9 cm，永远高于 2 cm 下限，因此下限在 LiDAR 上不起作用。
    assert sensor_tolerance(np.array([.1]), 'lidar')[0] == pytest.approx(.09)
    assert sensor_tolerance(np.array([.1]), 'lidar', floor=.2)[0] == pytest.approx(.2)


def test_disturbed_fraction_matches_cloud_change_for_constant_tolerance():
    """逐射线容差在常数情形下必须与 Gazebo 的 cloud_change 同口径。"""
    rng = np.random.default_rng(0)
    reference = rng.normal(0., 1., (200, 3))
    observed = reference + rng.normal(0., .004, reference.shape)
    observed[::7] = np.nan
    ours = disturbed_fraction(reference, observed, .01)
    theirs = cloud_change(reference, observed, np.ones(200, bool), tolerance=.01)
    for key in ('target_returns', 'visible_fraction', 'disturbed_fraction', 'finite_fraction'):
        assert ours[key] == pytest.approx(theirs[key])


def test_clean_condition_is_not_disturbed():
    """洁净条件下受扰比例应保持在小量级。

    3σ 包络作用在三维误差范数上时，卡方(3 自由度)的超出概率约 3%，
    远小于固定 1 cm 容差在远距离下的 80%——这正是逐射线容差要解决的问题。
    """
    rng = np.random.default_rng(1)
    reference = np.zeros((2000, 3))
    range_value = 1.
    sigma = .01*range_value**2
    observed = reference + rng.normal(0., sigma, reference.shape)
    result = disturbed_fraction(reference, observed,
                                sensor_tolerance(np.full(2000, range_value), 'depth'))
    assert result['disturbed_fraction'] < .05
    # 对照：固定 1 cm 容差在同一噪声下受扰比例明显更高
    fixed = disturbed_fraction(reference, observed, .01)
    assert fixed['disturbed_fraction'] > result['disturbed_fraction']*1.5


def test_level_validation():
    with pytest.raises(ValueError):
        InterferenceLevel('x', -1.)
    with pytest.raises(ValueError):
        InterferenceLevel('x', 10., noise_scale=-1.)
