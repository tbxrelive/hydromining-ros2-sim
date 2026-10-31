#!/usr/bin/env python3
"""Measure L0 and fine L1 on the same held-out wall and action, without training."""
import sys
import argparse
import json
from pathlib import Path
import time

import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from experiments.common.large_env import make_fast_env, make_reference_env


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--assets', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    action = np.array([0., 0., -1.], dtype=np.float32)
    fast = make_fast_env(args.assets, 10000, 'test')
    reference = make_reference_env(args.assets, 0, 'L1', 10000)
    reference.max_steps_ = 1
    try:
        fast.reset(seed=10000, options={'scenario': 'test_empty', 'mesh_index': 0})
        start = time.monotonic()
        fast.step(action)
        fast_s = time.monotonic()-start
        start = time.monotonic()
        reference.reset(seed=10000, options={'scenario': 'test_empty'})
        reset_s = time.monotonic()-start
        start = time.monotonic()
        reference.step(action)
        reference_s = time.monotonic()-start
        report = {'action': [0., 0., 2.], 'fast_step_s': fast_s, 'reference_step_s': reference_s,
                  'reference_reset_s': reset_s, 'speed_ratio': reference_s/fast_s,
                  'reference_profile': reference.profile_, 'training_performed': False,
                  'scope': 'one matched 2-second action; excludes learner updates'}
        Path(args.output).write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2))
    finally:
        fast.close()
        reference.close()


if __name__ == '__main__':
    main()
