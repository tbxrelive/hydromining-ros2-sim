"""Regression for fast geometry and observation semantics."""
import sys
from pathlib import Path

import numpy as np
import pytest
from gymnasium.utils.env_checker import check_env

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from algorithms.perception.kinematics import look_at
from algorithms.perception.wall_frame import WallFrame
from algorithms.planning.fast_heightfield import FastHeightfieldEnv
from algorithms.planning.observations import depth_progress


def environment(pressure=30., noise_scale=0.):
    frame = WallFrame.from_normal([0, 0, 0], [-1, 0, 0])
    pose = look_at(np.array([-.8, 0., 0.]), np.zeros(3))
    intrinsics = np.array([[171.7, 0, 160], [0, 171.7, 120], [0, 0, 1]])
    return FastHeightfieldEnv(np.zeros((144, 144)), frame, [pose], intrinsics,
                              (240, 320), pressure=pressure, standoff=.5, noise_scale=noise_scale,
                              scenario_names=['empty'])


def test_pressure_and_repeat_sprays_change_actual_geometry():
    low, high = environment(20.), environment(30.)
    action = np.array([0., 0., 0.], dtype=np.float32)
    _, _, _, _, a = low.step(action)
    _, _, _, _, b = high.step(action)
    assert b['spray']['volume_removed_m3'] > a['spray']['volume_removed_m3'] > 0
    old_height = high.heights_.copy()
    high.step(action)
    assert np.all(high.heights_ <= old_height)
    high.reset(seed=2, options={'scenario': 'empty'})
    np.testing.assert_array_equal(high.heights_, high.initial_)


def test_observations_are_reproducible_and_keep_unknown_cells():
    env = environment(noise_scale=1.)
    a, _ = env.reset(seed=42)
    b, _ = env.reset(seed=42)
    np.testing.assert_array_equal(a, b)
    # Turning the sole camera away must leave all task cells unknown.
    env.camera_poses_[0][:3, :3] = env.camera_poses_[0][:3, :3] @ np.diag([-1., -1., 1.])
    obs, _ = env.reset(seed=42)
    assert env.last_assessment_['coverage'] == 0
    assert not env.last_change_['valid'].any()
    assert np.isfinite(obs).all()


def test_continuous_conservative_progress_and_gym_contract():
    env = environment()
    check_env(env, skip_render_check=True)
    weights = np.ones((1, 2))
    change = {'valid': np.array([[True, False]]), 'delta': np.array([[.03, .1]]),
              'sigma': np.array([[.002, .002]])}
    assert depth_progress(change, weights) == pytest.approx((.03-1.645*.002)/.05/2)


def test_spray_action_stays_in_the_circular_task_region():
    env = environment()
    _, _, _, _, info = env.step(np.ones(3, dtype=np.float32))
    assert np.hypot(info['spray']['u'], info['spray']['v']) == pytest.approx(.2)


def test_repeated_sprays_cannot_remove_more_than_wall_thickness():
    env = environment()
    for _ in range(20):
        before = env.heights_.copy()
        _, reward, terminated, _, info = env.step(np.array([0., 0., 1.]))
        if terminated:
            break
    assert info['verdict'] == 'through_cut' and reward < 0
    np.testing.assert_array_equal(env.heights_, before)
    assert np.max(env.initial_-env.heights_) < .4
