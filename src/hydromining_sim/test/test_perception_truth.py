"""固定初始法向的解析几何与真值积分回归。"""
import sys
from pathlib import Path

import numpy as np
import pytest
import trimesh
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from algorithms.perception.wall_frame import WallFrame
from algorithms.perception.metrics import mesh_heights, truth_change


def test_frame_roundtrip_and_right_handedness():
    frame = WallFrame.from_normal([1, 2, 3], [-1, 0, -0.3])
    points = np.random.default_rng(4).normal(size=(20, 3))
    np.testing.assert_allclose(frame.to_world(frame.to_local(points)), points, atol=1e-14)
    assert np.linalg.det(frame.axes) == pytest.approx(1)
    # 冻结数组会拒绝 SciPy memoryview；适配层必须显式传副本（阶段六验收曾因此失败）。
    np.testing.assert_allclose(Rotation.from_matrix(frame.axes.copy()).as_matrix(), frame.axes)
    with pytest.raises(ValueError):
        frame.axes[0, 0] = 2


@pytest.mark.parametrize('normal', [[0, 0, 0], [np.nan, 0, 1], [1, 2]])
def test_invalid_normal_rejected(normal):
    with pytest.raises(ValueError):
        WallFrame.from_normal([0, 0, 0], normal)


def test_grid_centers_and_bound():
    frame = WallFrame.from_normal([0, 0, 0], [0, 0, 1])
    grid = frame.grid(0.2, 0.01)
    assert grid.shape == (20, 20, 3)
    assert grid[0, 0, 0] == pytest.approx(-0.095)
    assert grid[-1, -1, 1] == pytest.approx(0.095)
    with pytest.raises(ValueError):
        frame.grid(0.2, 0.003)
    with pytest.raises(ValueError):
        frame.grid(0.2, 0.00001)


def test_ray_cast_retreat_and_projected_volume(tmp_path):
    frame = WallFrame.from_normal([0, 0, 0], [0, 0, 1])
    before = trimesh.creation.box(extents=[1, 1, 0.2])
    after = before.copy()
    after.apply_translation([0, 0, -0.025])
    before.export(tmp_path / 'before.obj')
    after.export(tmp_path / 'after.obj')
    heights = [mesh_heights(tmp_path / f'{name}.obj', [0] * 6, frame, 0.24, 0.004)
               for name in ('before', 'after')]
    np.testing.assert_allclose(heights[0] - heights[1], 0.025, atol=1e-7)
    report = truth_change(*heights, frame, 0.24, 0.004, 0.1, 0.02, 0.9)
    assert report['status'] == 'achieved'
    assert report['removed_volume_m3'] == pytest.approx(
        report['sampled_area_m2'] * 0.025, abs=1e-8)
    same = truth_change(heights[0], heights[0], frame, 0.24, 0.004, 0.1, 0.02, 0.9)
    assert same['status'] == 'not_achieved'
    heights[1][30, 30] = np.nan
    with pytest.raises(ValueError, match='cover'):
        truth_change(*heights, frame, 0.24, 0.004, 0.1, 0.02, 0.9)
    missing = truth_change(*heights, frame, 0.24, 0.004, 0.1, 0.02, 0.9, allow_missing=True)
    assert missing['status'] == 'unknown'
    assert missing['fraction_ge_threshold'] is None
    assert missing['removed_volume_m3'] is None
    assert 0 < missing['valid_fraction'] < 1


def test_through_cut_with_missing_truth_returns_unknown(monkeypatch):
    """A failed candidate and missing pre-shot truth must end evaluation honestly."""
    from types import SimpleNamespace
    from algorithms.planning import rl_env
    frame = WallFrame.from_normal([0, 0, 0], [0, 0, 1])
    before = np.zeros((10, 10))
    after = before.copy()
    after[5, 5] = np.nan
    env = rl_env.HydrominingSurrogateEnv.__new__(rl_env.HydrominingSurrogateEnv)
    env.task_ = {'diameter': .08}
    env.frame_, env.normal_, env.standoff_ = frame, np.array([0, 0, 1]), .5
    env.pristine_, env.truth_extent_, env.truth_resolution_ = before, .1, .01
    env.perception_ = SimpleNamespace(diameter=.08, required_depth=.05, required_fraction=.9)
    env.last_evaluated_ = {'report': {'assessment': {'status': 'not_achieved', 'fraction_lower': 0.}}}
    env._decode_spray = lambda action: (0., 0., 2.)
    env._state_tensor = lambda evaluated: np.zeros(3)
    env.steps_used_, env.cost_spray_per_s_, env.pressure_, env.terminal_reward_ = 0, .3, 30., 10.
    def through_cut(*args):
        raise ValueError('Through-cut detected: depth metric requires a remaining surface')
    env.world_ = SimpleNamespace(spray_tcp=through_cut, truth_heights=lambda *args: after)
    monkeypatch.setattr(rl_env, 'spray_target_tcp', lambda *args: np.eye(4))
    _, _, terminated, truncated, info = env._step_spray(np.zeros(3))
    assert terminated and not truncated
    assert info['verdict'] == 'through_cut' and info['truth'] == 'unknown'
    assert info['truth_fraction'] is None and info['removed_volume_m3'] is None
    assert 0 < info['truth_valid_fraction'] < 1
