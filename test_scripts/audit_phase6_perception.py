#!/usr/bin/env python3
"""独立比较感知状态图与真实网格；真值不传入感知管线。"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

WORKSPACE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORKSPACE / 'src/hydromining_sim/scripts'))
from algorithms.perception.assessment import circle_fractions
from algorithms.perception.metrics import mesh_heights, truth_change
from algorithms.perception.wall_frame import WallFrame


def evaluate_states(dataset, results, output, extent=0.5, resolution=0.01):
    """有效区域几何误差与分类安全指标分开统计，覆盖不足不冒充几何正确。"""
    rows = json.loads((results / 'assessment.json').read_text())
    pose = [1.5, 0., 1.5, 0., -1.9198621771937625, 0.]
    report = {'scope': 'fixed simulator meshes only', 'calibrated': False, 'scenarios': {}}
    for name, row in rows.items():
        with np.load(results / f'{name}_state.npz') as state:
            frame = WallFrame(state['origin'], state['axes'])
            before = mesh_heights(dataset / 'truth/baseline.obj', pose, frame, extent, resolution)
            after, normal = mesh_heights(
                dataset / f'truth/{name}.obj', pose, frame, extent, resolution, return_normals=True)
            fraction = circle_fractions(frame.grid(extent, resolution), resolution, 0.1)
            target = fraction > 0
            valid = target & state['change_valid'] & np.isfinite(before) & np.isfinite(after)
            height_error = state['current_height'][valid] - after[valid]
            retreat_error = state['change_delta'][valid] - (before - after)[valid]
            normal_valid = valid & np.isfinite(state['current_normal']).all(axis=-1)
            dot = np.sum(state['current_normal'][normal_valid] * normal[normal_valid], axis=-1)
            angles = np.degrees(np.arccos(np.clip(dot, -1, 1)))
            reference = truth_change(before, after, frame, extent, resolution, 0.1, 0.02, 0.9)
            area = fraction * resolution ** 2
            truth_volume = float(np.sum(np.maximum((before - after)[valid], 0) * area[valid]))
            estimated_volume = row['volume']['removed_volume_m3']
            volume_error = abs(estimated_volume - truth_volume)
            rmse = lambda values: float(np.sqrt(np.mean(values ** 2))) if len(values) else None
            report['scenarios'][name] = {
                'truth': reference, 'predicted': row['assessment']['status'],
                'false_pass': (reference['status'] == 'not_achieved' and
                               row['assessment']['status'] == 'achieved'),
                'coverage': row['assessment']['coverage'],
                'evaluated_cells': int(valid.sum()),
                'height_rmse_m': rmse(height_error),
                'retreat_rmse_m': rmse(retreat_error),
                'normal_median_error_deg': float(np.median(angles)) if len(angles) else None,
                'height_95pct_interval_coverage': float(np.mean(
                    np.abs(height_error) <= 1.96 * state['current_sigma'][valid])) if valid.any() else None,
                'effective_truth_volume_m3': truth_volume,
                'volume_absolute_error_m3': volume_error,
                'volume_relative_error': volume_error / truth_volume if truth_volume > 1e-8 else None,
            }
    report['false_pass_count'] = sum(row['false_pass'] for row in report['scenarios'].values())
    report['passed_safety_for_tested_cases'] = report['false_pass_count'] == 0
    output.write_text(json.dumps(report, indent=2, allow_nan=False))
    print(json.dumps(report, indent=2))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, required=True, help='原始数据集，含 truth')
    parser.add_argument('--results', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output already exists')
    report = evaluate_states(args.dataset, args.results, args.output)
    return 0 if report['passed_safety_for_tested_cases'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
