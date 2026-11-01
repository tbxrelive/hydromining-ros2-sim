"""冲蚀核参数注入、孔形度量与统计判据的单测。

E2.0 的核心要求是"可注入且默认值逐位不变"，因此这里同时锁定
旧公式的数值与参数化的默认行为。
"""
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(Path(__file__).parents[1]/'scripts'))
sys.path.insert(0, str(ROOT))

from algorithms.erosion_model import DefaultErosionKernel, ErosionKernel  # noqa: E402
from experiments.common.crater import flat_plate, measure, through_check  # noqa: E402
from experiments.common.regression import compare  # noqa: E402
from experiments.common.stats import adjacent_separation, bootstrap_ci, kendall_tau  # noqa: E402


# 标称工况 30 MPa、0.5 m 处的中心冲蚀速度：历次改核都锚定在这个值上，
# 保证任务压力下的全部下游结果可比。
NOMINAL_RC = 0.013945840774443334


def test_default_kernel_follows_calibrated_formula():
    """默认参数按文献标定：阈值压力 + 线性压力 + 0.22x 半宽 + 剖面指数 2。"""
    kernel = DefaultErosionKernel()
    assert kernel.threshold() == 5.
    assert kernel.R_c(5., .5) == 0.            # 阈值处不破煤
    assert kernel.R_c(3., .5) == 0.            # 阈值下不破煤
    for pressure in (10., 20., 30., 50.):
        for distance in (.1, .5, 1.0):
            expected = kernel.k*(pressure - kernel.pressure_threshold)/(distance**2.0)
            assert kernel.R_c(pressure, distance) == expected
            # b 与压力无关（Zhang & Zou）：b = b0 + 0.22·s
            assert kernel.b(pressure, distance) == .001 + .22*distance
    assert kernel.R_c(30., .5) == pytest.approx(NOMINAL_RC, rel=1e-12)


def test_default_radius_clamps_near_zero_distance():
    """靶距下限仍是 0.01 m，避免除零。"""
    kernel = DefaultErosionKernel()
    assert kernel.R_c(30., 0.) == kernel.R_c(30., .01)


def test_radial_kernel_default_matches_literature_profile():
    """径向侵蚀核为 (1-ξ^1.5)^6：速度剖面 (1-η^1.5)^2 的三次方（式(8)：∝u³）。"""
    xi = np.linspace(0., 1.2, 50)
    expected = np.where(xi < 1, (1 - xi**(3/2))**6, 0)
    assert np.array_equal(DefaultErosionKernel().kernel_F(xi), expected)
    assert DefaultErosionKernel().kernel_F(np.array([1.0]))[0] == 0.


def test_kernel_parameters_are_injectable():
    """参数注入生效，且只改动对应项。"""
    base = DefaultErosionKernel()
    faster = DefaultErosionKernel(k=2*base.k)
    assert faster.R_c(30., .5) == pytest.approx(2*base.R_c(30., .5))
    assert faster.b(30., .5) == base.b(30., .5)
    wider = DefaultErosionKernel(b_distance=.3)
    assert wider.b(30., .5) == pytest.approx(.001 + .3*.5)
    assert np.array_equal(DefaultErosionKernel(radial_decay=8).kernel_F(np.array([.5])),
                          np.array([(1 - .5**1.5)**8]))


def test_strength_interface_is_neutral_by_default():
    """强度指数为 0 时，任何强度都不改变结果（E2.2 之前必须中性）。"""
    kernel = DefaultErosionKernel()
    base = kernel.R_c(30., .5)
    for strength in (5., 30., 120.):
        assert kernel.R_c(30., .5, strength) == base
        assert kernel.strength_factor(strength) == 1.0
    # 强度通过缩放阈值压力进入：煤越硬、阈值越高、R_c 越小。
    # 注意强度因子只在显式传入强度时生效；核内 coal_strength_mpa=0 表示未知，取中性。
    hardened = DefaultErosionKernel(strength_exponent=1.)
    assert hardened.threshold(60.) == pytest.approx(10.)
    assert hardened.R_c(30., .5, 60.) < hardened.R_c(30., .5, 30.)
    assert hardened.R_c(30., .5) == hardened.R_c(30., .5, 30.)  # 未指定时用参考强度


def test_unknown_kernel_parameter_is_rejected():
    with pytest.raises(ValueError):
        DefaultErosionKernel(gamma=1.)


def test_custom_erosion_kernel_still_supported():
    """旧的 ErosionKernel(函数, 函数) 构造方式必须继续可用。"""
    kernel = ErosionKernel(lambda p, s: .015, lambda p, s: .18)
    assert kernel.R_c(20., .4) == .015 and kernel.b(20., .4) == .18
    assert kernel.kernel_F(np.array([.5]))[0] == pytest.approx((1 - .5**1.5)**6)


def test_crater_measure_on_synthetic_hole():
    """直径为 6 cm、深 2 cm 的圆坑，度量的深度/孔径/体积应可信。"""
    spacing, extent = .005, .72
    heights, grid, axis = flat_plate(extent, spacing)
    before = heights.copy()
    radial = np.hypot(grid[..., 0], grid[..., 1])
    after = heights.copy()
    # 高度场 z 指向观察者，冲蚀使高度下降，故退缩量 = before - after = +0.02。
    after[radial <= .03] = -.02
    result = measure(before, after, spacing, center=(0., 0.))
    assert result['depth_max_m'] == pytest.approx(.02)
    assert result['depth_center_m'] == pytest.approx(.02)
    assert result['diameter_half_m'] == pytest.approx(.06, abs=.01)
    assert result['volume_m3'] == pytest.approx(np.pi*.03**2*.02, rel=.05)
    assert result['symmetry'] == pytest.approx(1., abs=.1)
    assert not through_check(before, after, .4)


def test_adjacent_separation_requires_all_four_criteria():
    """可分判据要求间距、显著性、实际差异与顺序同时成立。"""
    rng = np.random.default_rng(0)
    near = rng.normal(.10, .001, 30)
    apart = rng.normal(.20, .001, 30)
    far = adjacent_separation(near, apart)
    assert far['separable'] and far['order_ok'] and far['practical_ok']
    # 统计显著但差异只有 0.5 个百分点，不应判为可分。
    tight = adjacent_separation(rng.normal(.10, .0001, 30), rng.normal(.105, .0001, 30))
    assert not tight['separable'] and not tight['practical_ok']
    # 顺序反了也不可分。
    assert not adjacent_separation(apart, near)['separable']


def test_bootstrap_ci_and_kendall():
    ci = bootstrap_ci([1., 2., 3., 4., 5.])
    assert ci[0] < 3. < ci[1]
    assert bootstrap_ci([1.]) is None
    perfect = kendall_tau([1., 2., 3., 4.], [10., 20., 30., 40.])
    assert perfect['tau'] == pytest.approx(1.)


def test_regression_compare_detects_field_changes(tmp_path):
    base = tmp_path/'a.jsonl'
    other = tmp_path/'b.jsonl'
    base.write_text('{"steps": 3, "wall_time_s": 1.0}\n')
    other.write_text('{"steps": 3, "wall_time_s": 9.0}\n')
    assert compare(base, other)[0] == []
    other.write_text('{"steps": 4, "wall_time_s": 9.0}\n')
    differences, count = compare(base, other)
    assert count == 1 and differences[0][1] == 'steps'
