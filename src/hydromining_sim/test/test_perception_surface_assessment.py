"""融合、误差下限、三分类、均匀退缩与区域外变化解析回归。"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from algorithms.perception.assessment import assess, circle_fractions
from algorithms.perception.change_detection import (
    compare_surfaces, external_change_alarm, volume_summary)
from algorithms.perception.config import PerceptionConfig
from algorithms.perception.height_map import HeightMap
from algorithms.perception.pipeline import PerceptionSession
from algorithms.perception.wall_frame import WallFrame


def test_plane_fusion_normals_and_systematic_floor():
    frame = WallFrame.from_normal([0, 0, 0], [0, 0, 1])
    surface = HeightMap(frame, 0.2, 0.02)
    u, v = np.meshgrid(np.linspace(-0.095, 0.095, 40), np.linspace(-0.095, 0.095, 40))
    points = np.c_[u.ravel(), v.ravel(), (0.1 * u + 0.2 * v).ravel()]
    direction = np.tile([0, 0, -1.], (len(points), 1))
    for _ in range(5):
        surface.integrate(points, direction, np.full(len(points), 0.005))
    state = surface.state()
    assert (state['coverage'] == 2).all()
    expected = np.array([-0.1, -0.2, 1.])
    expected /= np.linalg.norm(expected)
    normals = state['normal'][2:-2, 2:-2]
    np.testing.assert_allclose(normals, np.broadcast_to(expected, normals.shape), atol=1e-6)
    assert np.nanmin(state['sigma']) >= 0.0015
    local = frame.to_local(points)
    local[:, 0] += 10
    surface.integrate(frame.to_world(local), direction, np.full(len(points), 0.005))
    assert np.all(surface.count_ == 5), 'out-of-bounds points must not accumulate at map edges'


def test_sparse_cells_and_multimodal_surface_are_not_reliable():
    frame = WallFrame.from_normal([0, 0, 0], [0, 0, 1])
    surface = HeightMap(frame, 0.2, 0.02)
    points = np.r_[np.tile([[0.001, 0.001, 0]], (8, 1)),
                   np.tile([[0.001, 0.001, 0.1]], (8, 1))]
    for _ in range(3):
        surface.integrate(points, np.tile([0, 0, 1.], (16, 1)), np.full(16, 0.001))
    state = surface.state()
    assert not (state['coverage'] == 2).any()
    assert state['ambiguous'].any()


def test_cross_view_disagreement_cannot_be_averaged_into_false_confidence():
    frame = WallFrame.from_normal([0, 0, 0], [0, 0, 1])
    surface = HeightMap(frame, 0.2, 0.02)
    points = np.tile([[0.001, 0.001, 0.]], (10, 1))
    direction = np.tile([0., 0, 1.], (10, 1))
    for height in (0., 0.08, 0., 0.08):
        points[:, 2] = height
        surface.integrate(points, direction, np.full(10, 0.001))
    state = surface.state()
    observed = state['count'] > 0
    assert (state['sigma'][observed] > 0.03).all()
    assert not (state['coverage'] == 2).any()


def change(delta, sigma=0.001, valid=True):
    values = np.asarray(delta, float)
    return {'delta': values, 'sigma': np.full_like(values, sigma),
            'valid': np.full(values.shape, valid)}


def test_three_way_bounds_and_unknown_cells():
    areas = np.ones(100)
    assert assess(change(np.full(100, 0.03)), areas)['status'] == 'achieved'
    assert assess(change(np.zeros(100)), areas)['status'] == 'not_achieved'
    assert assess(change(np.full(100, 0.02)), areas)['status'] == 'insufficient_evidence'
    unknown = change(np.full(100, np.nan), valid=False)
    report = assess(unknown, areas)
    assert report['fraction_lower'] == 0 and report['fraction_upper'] == 1
    assert report['status'] == 'insufficient_evidence'
    unknown['valid'][:11] = True
    unknown['delta'][:11] = 0
    assert assess(unknown, areas)['status'] == 'not_achieved'


def test_circle_area_quadrature():
    centers = WallFrame.from_normal([0, 0, 0], [0, 0, 1]).grid(0.3, 0.01)
    fraction = circle_fractions(centers, 0.01, 0.1)
    assert np.sum(fraction) * 0.01 ** 2 == pytest.approx(np.pi * 0.1 ** 2, rel=0.001)


def test_volume_uses_observed_area_and_does_not_average_common_error_away():
    delta = change(np.full((10, 10), 0.03), sigma=0.002)
    area = np.full((10, 10), 0.0001)
    report = volume_summary(delta, area)
    assert report['removed_volume_m3'] == pytest.approx(0.0003)
    assert report['volume_sigma_m3'] == pytest.approx(0.002 * area.sum())
    delta['valid'][:5] = False
    assert volume_summary(delta, area)['removed_volume_m3'] == pytest.approx(0.00015)


def test_external_change_does_not_enter_region_assessment():
    delta = change(np.zeros((10, 10)))
    delta['delta'][0] = 0.05
    region = np.ones((10, 10))
    region[0] = 0
    assert external_change_alarm(delta, region, 0.0001)['triggered']
    assert assess(delta, region)['status'] == 'not_achieved'


def test_uniform_retreat_is_not_absorbed_as_registration():
    before = {'height': np.zeros((5, 5)), 'sigma': np.full((5, 5), 0.001),
              'coverage': np.full((5, 5), 2)}
    after = {**before, 'height': np.full((5, 5), -0.03)}
    delta = compare_surfaces(before, after, 0.0001)
    np.testing.assert_allclose(delta['delta'], 0.03)


def plane_snapshots(retreat=0., missing=False):
    snapshots = []
    for sec in (1, 2, 3):
        snapshots.append({
            'depth': np.full((41, 41), np.nan if missing else 1. + retreat),
            'K': np.array([[100., 0, 20.], [0, 100., 20.], [0, 0, 1.]]),
            'lidar_local': np.full((1, 3), np.nan),
            'tf_depth': np.array([0., 0, 0, 0, 0, 0, 1.]),
            'tf_lidar': np.array([0., 0, 0, 0, 0, 0, 1.]),
            'depth_frame': 'camera_link', 'lidar_frame': 'lidar_link',
            'depth_stamp': np.array([sec, 0]), 'lidar_stamp': np.array([sec, 0]),
        })
    return snapshots


def test_pipeline_end_to_end_plane_and_cancel():
    config = PerceptionConfig(extent=0.4, resolution=0.02, noise_scale=0.)
    session = PerceptionSession([1., 0, 0], config)
    with pytest.raises(ValueError, match='baseline'):
        session.evaluate([plane_snapshots()])
    session.set_baseline([plane_snapshots()])
    result = session.evaluate([plane_snapshots(0.03)])
    assert result['report']['assessment']['status'] == 'achieved'
    assert session.evaluate([plane_snapshots()])['report']['assessment']['status'] == 'not_achieved'
    unknown = session.evaluate([plane_snapshots(missing=True)])
    assert unknown['report']['assessment']['status'] == 'insufficient_evidence'
    with pytest.raises(InterruptedError):
        session.evaluate([plane_snapshots()], cancel=lambda: True)


@pytest.mark.parametrize('values', [
    {'noise_scale': -1}, {'resolution': 0.0001}, {'maximum_views': 100},
    {'systematic_sigma': 0}, {'required_fraction': 1.1}, {'typo': 1}])
def test_config_rejects_invalid_or_unbounded_parameters(values):
    with pytest.raises(ValueError):
        PerceptionConfig.from_dict(values)


def test_observe_accumulates_views_and_evaluate_replaces_epoch():
    config = PerceptionConfig(extent=0.4, resolution=0.02, noise_scale=0.)
    session = PerceptionSession([1., 0, 0], config)
    with pytest.raises(ValueError, match='baseline'):
        session.observe([plane_snapshots()])
    session.set_baseline([plane_snapshots()])
    first = session.evaluate([plane_snapshots(0.03)])
    merged = session.observe([plane_snapshots(0.03)])
    observed = first['current']['count'] > 0
    assert (merged['current']['count'][observed] > first['current']['count'][observed]).all()
    assert (merged['current']['sigma'][observed] < first['current']['sigma'][observed]).all()
    assert merged['report']['assessment']['status'] == 'achieved'
    replaced = session.evaluate([plane_snapshots(0.03)])
    np.testing.assert_array_equal(replaced['current']['count'], first['current']['count'])
    for _ in range(config.maximum_views - 1):
        session.observe([plane_snapshots(0.03)])
    with pytest.raises(ValueError, match='budget'):
        session.observe([plane_snapshots(0.03)])
