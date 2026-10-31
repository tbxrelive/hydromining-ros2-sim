"""Regression for the high-fidelity evaluation that stopped on missing truth."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'test_scripts'))
from phase7_evaluate_large import evaluate


def test_through_cut_missing_truth_is_not_reported_as_zero_volume():
    class Environment:
        pressure_ = 30.
        steps_used_ = 1
        sprays_used_ = 0
        total_spray_duration_ = 0.

        def reset(self, **kwargs):
            return np.zeros(3), {'initially_achieved': False}

        def step(self, action):
            return np.zeros(3), -5., True, False, {
                'verdict': 'through_cut', 'truth': 'unknown', 'truth_fraction': None,
                'removed_volume_m3': None, 'truth_valid_fraction': .99}

    result = evaluate(lambda _: Environment(), lambda *_: np.zeros(3), 1, 10000, 'L2')
    assert result['episodes'][0]['truth'] == 'unknown'
    assert result['episodes'][0]['truth_valid_fraction'] == .99
    assert result['summary']['all']['truth_unknown'] == 1
    assert result['summary']['all']['mean_removed_volume_m3'] is None
    assert result['summary']['all']['removed_volume_known_episodes'] == 0
