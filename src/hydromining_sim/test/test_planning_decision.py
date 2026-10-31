"""规则决策的补看去重、轮次预算与置信带目标回归。"""
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from algorithms.perception.config import PerceptionConfig
from algorithms.perception.wall_frame import WallFrame
from algorithms.planning.decision import RulePolicy


@pytest.fixture
def inputs():
    config = PerceptionConfig()
    frame = WallFrame.from_normal([0, 0, 0], [0, 0, 1])
    centers = frame.grid(config.extent, config.resolution)
    shape = centers.shape[:2]
    return {
        'state': {'coverage': np.ones(shape), 'sigma': np.full(shape, 0.01)},
        'assessment': {'status': 'insufficient_evidence', 'coverage': 0.7,
                       'fraction_lower': 0.5},
        'change': {'sigma': np.full(shape, 0.003), 'delta': np.zeros(shape),
                   'valid': np.ones(shape, bool)},
        'surface': SimpleNamespace(centers_=centers),
        'frame': frame,
        'view_poses': {f'view_{i}': [0, 0, 0.7 + i * 0.01] for i in range(7)},
        'visibility': {f'view_{i}': np.ones(shape, bool) for i in range(7)},
        'region_mask': np.linalg.norm(centers[..., :2], axis=-1) < 0.1,
    }


def test_observations_are_unique_and_bounded_per_round(inputs):
    policy = RulePolicy(PerceptionConfig())
    actions = [policy.decide(**inputs) for _ in range(6)]
    assert [action['type'] for action in actions] == ['observe'] * 5 + ['spray']
    assert len({action['view'] for action in actions[:5]}) == 5
    assert not policy.observed_views_
    assert policy.decide(**inputs)['type'] == 'observe'
    assert all(mask.all() for mask in inputs['visibility'].values())


def test_reset_episode_clears_truncated_round(inputs):
    policy = RulePolicy(PerceptionConfig())
    first = policy.decide(**inputs)['view']
    policy.reset_episode()
    assert policy.decide(**inputs)['view'] == first


def test_achieved_finishes_without_spraying(inputs):
    policy = RulePolicy(PerceptionConfig())
    policy.decide(**inputs)
    inputs['assessment'].update(status='achieved', fraction_lower=0.95)
    assert policy.decide(**inputs)['type'] == 'finish'
    assert not policy.observed_views_


def test_spray_target_includes_confidence_margin(inputs):
    config = PerceptionConfig()
    policy = RulePolicy(config)
    inputs['change']['delta'][:] = config.required_depth
    uv, duration = policy.spray_target(
        inputs['change'], inputs['region_mask'], inputs['surface'])
    assert uv is not None
    assert duration >= 2
    expected = config.confidence_k * 0.003
    np.testing.assert_allclose(
        policy.deficit_map(inputs['change'], inputs['region_mask'])[
            inputs['region_mask']], expected)


def test_no_deficit_or_observe_budget_finishes_honestly(inputs):
    policy = RulePolicy(PerceptionConfig(), observe_budget=0)
    inputs['change']['delta'][:] = 0.1
    action = policy.decide(**inputs)
    assert action['type'] == 'finish'
    assert '预算已用尽' in action['rationale']
    assert inputs['assessment']['status'] == 'insufficient_evidence'
