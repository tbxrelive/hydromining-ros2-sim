"""时间戳、静止约束和不吞掉真实退缩的过滤回归。"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from algorithms.perception.alignment import aligned_sensor, stamp_ns, validate_sequence
from algorithms.perception.interference_filter import (
    FOREGROUND, INVALID, KEEP, LOW_SUPPORT, TEMPORAL_OUTLIER,
    foreground_mask, temporal_filter)


def snapshot(sec=1):
    return {
        'depth': np.array([[2., np.inf]]), 'K': np.eye(3),
        'lidar_local': np.array([[0., 3., 4.], [np.nan, 0., 0.]]),
        'tf_depth': np.array([1., 0., 0., 0., 0., 0., 1.]),
        'tf_lidar': np.array([0., 0., 0., 0., 0., 0., 1.]),
        'depth_frame': 'camera_link', 'lidar_frame': 'lidar_link',
        'depth_stamp': np.array([sec, 0]), 'lidar_stamp': np.array([sec, 100000000]),
    }


def test_alignment_preserves_invalid_points_and_sensor_axis():
    world, directions, sigma = aligned_sensor(snapshot(), 'depth')
    np.testing.assert_allclose(world[0, 0], [3, 0, 0])
    np.testing.assert_allclose(directions[0, 0], [1, 0, 0])
    assert sigma[0, 0] == pytest.approx(0.04)
    assert np.isnan(world[0, 1]).all()
    assert np.isnan(sigma[0, 1])
    world, directions, sigma = aligned_sensor(snapshot(), 'lidar')
    np.testing.assert_allclose(directions[0], [0, 0.6, 0.8])
    assert np.isnan(world[1]).all()


def test_sequence_uses_each_timestamp_and_rejects_duplicate_or_moving_frames():
    a, b = snapshot(1), snapshot(2)
    assert validate_sequence([a, b])['frames'] == 2
    with pytest.raises(ValueError, match='duplicate'):
        validate_sequence([a, a])
    b['tf_depth'][0] += 0.01
    with pytest.raises(ValueError, match='stationary'):
        validate_sequence([a, b])
    b = snapshot(2)
    b['depth_frame'] = 'camera_optical_frame'
    with pytest.raises(ValueError, match='frame'):
        validate_sequence([b])
    b = snapshot(2)
    b['lidar_stamp'] = np.array([4, 0])
    with pytest.raises(ValueError, match='skew'):
        validate_sequence([b])


@pytest.mark.parametrize('stamp', [[1.0, 0.0], [-1, 0], [0, 1000000000], [1]])
def test_invalid_stamp_rejected(stamp):
    with pytest.raises(ValueError):
        stamp_ns(stamp)


def test_nan_temporal_filter_keeps_missing_missing_and_removes_transient_particle():
    values = np.array([[1., np.nan, 1.], [1., np.nan, 0.4], [1., np.nan, 1.]])
    filtered, labels = temporal_filter(values, 0.001)
    assert np.isnan(filtered[:, 1]).all()
    assert (labels[:, 1] == INVALID).all()
    assert labels[1, 2] == TEMPORAL_OUTLIER
    assert np.isnan(filtered[1, 2])
    assert (labels[:, 0] == KEEP).all()
    _, labels = temporal_filter([[1.], [np.nan], [np.nan]], 0.001)
    assert labels[0, 0] == LOW_SUPPORT


def test_geometric_filter_does_not_reject_real_retreat_or_uniform_damage():
    heights = np.array([-0.06, -0.02, 0., 0.1, np.nan])
    mask = foreground_mask(heights, np.zeros(5), 0.001)
    np.testing.assert_array_equal(mask, [False, False, False, True, False])
    assert FOREGROUND != KEEP
