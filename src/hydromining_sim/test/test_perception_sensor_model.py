"""传感器与环境退化模型：数量级、单调性、可复现与边界输入。"""
import sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from algorithms.perception.sensor_model import (
    depth_sigma, lidar_sigma, extinction, transmittance, degrade_ranges,
    ENV_PRESETS, VALID, DROPPED, FALSE_RETURN, INVALID)


def test_depth_sigma_matches_datasheet_two_percent_at_two_metres():
    assert depth_sigma(2.0) == pytest.approx(0.02 * 2.0)
    assert depth_sigma(1.0) < depth_sigma(2.0) < depth_sigma(3.0)
    assert depth_sigma(1.0, 0.5) == pytest.approx(0.5 * depth_sigma(1.0))


def test_lidar_sigma_constant():
    assert np.all(lidar_sigma(np.array([1., 5.])) == 0.03)


def test_clean_environment_is_lossless_apart_from_noise():
    rng = np.random.default_rng(0)
    r = np.full(2000, 1.0)
    out, lab = degrade_ranges(r, 0.0, np.inf, rng)
    assert np.array_equal(out, r) and np.all(lab == VALID)


def test_transmittance_and_dropout_monotone_in_visibility():
    assert transmittance(1.0, 8.0) > transmittance(1.0, 3.0) > transmittance(1.0, 1.5)
    drops = []
    for v in (8.0, 3.0, 1.5):
        _, lab = degrade_ranges(np.full(20000, 1.0), 0.0, v, np.random.default_rng(1))
        drops.append(np.mean(lab == DROPPED))
    assert drops[0] < drops[1] < drops[2]
    assert drops[2] == pytest.approx(1 - transmittance(1.0, 1.5), abs=0.02)


def test_false_returns_lie_in_front_of_target():
    r = np.full(20000, 1.0)
    out, lab = degrade_ranges(r, 0.0, 1.5, np.random.default_rng(2))
    false = lab == FALSE_RETURN
    assert false.any() and np.all(out[false] < 1.0) and np.all(out[false] >= 0.1)


def test_noise_grows_with_dust_and_is_capped():
    s = []
    for v in (np.inf, 3.0):
        out, lab = degrade_ranges(np.full(20000, 0.6), 0.01, v, np.random.default_rng(3))
        s.append(np.std(out[lab == VALID]))
    assert s[1] > s[0]


def test_reproducible_and_nan_preserved():
    r = np.array([1.0, np.nan, 2.0] * 100)
    a = degrade_ranges(r, 0.01, 3.0, np.random.default_rng(7))
    b = degrade_ranges(r, 0.01, 3.0, np.random.default_rng(7))
    assert np.array_equal(a[0], b[0], equal_nan=True) and np.array_equal(a[1], b[1])
    assert np.all(np.isnan(a[0][1::3])) and np.all(a[1][1::3] == INVALID)


def test_invalid_visibility_rejected():
    for bad in (0.0, -1.0, np.nan):
        with pytest.raises(ValueError):
            extinction(bad)
    assert set(ENV_PRESETS) == {'clean', 'dim_dust_light', 'dim_dust_heavy', 'dim_dust_extreme'}


@pytest.mark.parametrize('bad', [-1.0, np.nan, np.inf])
def test_invalid_noise_scale_rejected(bad):
    for model in (depth_sigma, lidar_sigma):
        with pytest.raises(ValueError):
            model(1.0, bad)


@pytest.mark.parametrize('sigma', [-0.1, np.nan, np.inf])
def test_invalid_sigma_rejected(sigma):
    with pytest.raises(ValueError):
        degrade_ranges([1.0], sigma, np.inf, np.random.default_rng(0))


def test_nonphysical_returns_are_not_treated_as_valid_measurements():
    out, labels = degrade_ranges([0.0, -1.0, np.inf, np.nan, 1.0], 0.0,
                                 3.0, np.random.default_rng(5))
    assert np.isnan(out[:4]).all()
    assert (labels[:4] == INVALID).all()


def test_list_input_and_zero_dimensional_input():
    out, labels = degrade_ranges([1.0, 2.0], 0.0, 3.0, np.random.default_rng(0))
    assert out.shape == labels.shape == (2,)
    out, labels = degrade_ranges(1.0, 0.0, np.inf, np.random.default_rng(0))
    assert out == 1.0 and labels == VALID
