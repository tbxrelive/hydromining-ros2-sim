"""快照退化的距离语义、随机流、无效回波与保真回归。"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from algorithms.perception.environment import degrade_snapshot, label_counts, sensor_rng
from algorithms.perception.sensor_model import DROPPED, FALSE_RETURN, INVALID, VALID


def degraded(depth, lidar, visibility=np.inf, noise_scale=0.0, seed=1):
    return degrade_snapshot(
        depth, np.eye(3), lidar, visibility, noise_scale,
        sensor_rng(seed, 'snapshot', 'depth'), sensor_rng(seed, 'snapshot', 'lidar'))


def test_clean_identity_and_input_not_mutated():
    depth = np.array([[1.0, 2.0], [np.inf, 0.0]])
    lidar = np.array([[3., 4., 0.], [np.nan, 1., 2.], [0., 0., 0.]])
    d0, l0 = depth.copy(), lidar.copy()
    result = degraded(depth, lidar)
    np.testing.assert_allclose(result['depth'][0], depth[0])
    np.testing.assert_allclose(result['lidar_local'][0], lidar[0])
    assert np.isnan(result['depth'][1]).all()
    assert np.isnan(result['lidar_local'][1:]).all()
    np.testing.assert_array_equal(result['depth_labels'], [[VALID, VALID], [INVALID, INVALID]])
    np.testing.assert_array_equal(depth, d0)
    np.testing.assert_array_equal(lidar, l0)


def test_axial_sigma_converted_to_slant_and_back():
    # 每次返回相同的标准正态样本，便于解析检验边缘像素的轴向 sigma。
    class UnitNormal:
        def normal(self, mean, sigma, size):
            return np.ones(size)

        def random(self, shape):
            return np.zeros(shape)

    result = degrade_snapshot(
        np.full((1, 2), 2.0), np.eye(3), np.array([[3., 4., 0.]]),
        np.inf, 1.0, UnitNormal(), UnitNormal())
    np.testing.assert_allclose(result['depth'], [[2.04, 2.04]])
    np.testing.assert_allclose(result['lidar_local'], [[3.018, 4.024, 0.]])


def test_slant_not_axial_distance_controls_extinction():
    depth = np.ones((1, 40000))
    k = np.array([[20000., 0, 0], [0, 1, 0], [0, 0, 1]])
    result = degrade_snapshot(depth, k, np.empty((0, 3)), 3.0, 0.0,
                              sensor_rng(7, 'a', 'depth'), sensor_rng(7, 'a', 'lidar'))
    labels = result['depth_labels'][0]
    assert np.mean(labels[30000:] == DROPPED) > np.mean(labels[:10000] == DROPPED)


def test_seed_repeatable_independent_of_processing_order_and_sensor():
    a = sensor_rng(12, 'one', 'depth').normal(size=30)
    sensor_rng(12, 'two', 'depth').normal(size=30)
    np.testing.assert_array_equal(a, sensor_rng(12, 'one', 'depth').normal(size=30))
    assert not np.array_equal(a, sensor_rng(12, 'one', 'lidar').normal(size=30))
    assert not np.array_equal(a, sensor_rng(13, 'one', 'depth').normal(size=30))


def test_false_returns_remain_on_original_rays():
    lidar = np.tile([3., 4., 0.], (20000, 1)) / 5
    result = degraded(np.ones((1, 1)), lidar, visibility=3.0)
    labels, points = result['lidar_labels'], result['lidar_local']
    finite = np.isfinite(points).all(axis=1)
    np.testing.assert_allclose(points[finite, 1] / points[finite, 0], 4 / 3)
    assert (labels == FALSE_RETURN).any()
    assert np.all(np.linalg.norm(points[labels == FALSE_RETURN], axis=1) < 1)
    assert np.isnan(points[labels == DROPPED]).all()
    assert sum(label_counts(labels).values()) == len(lidar)


@pytest.mark.parametrize('seed', [-1, 0.5, True])
def test_invalid_seed_rejected(seed):
    with pytest.raises(ValueError):
        sensor_rng(seed, 'a', 'depth')


@pytest.mark.parametrize('lidar', [np.ones(3), np.ones((2, 2)), np.ones((1, 1, 3))])
def test_invalid_cloud_shape_rejected(lidar):
    with pytest.raises(ValueError):
        degraded(np.ones((1, 1)), lidar)
