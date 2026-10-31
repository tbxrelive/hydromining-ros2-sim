#!/usr/bin/env python3
"""6.0 固定初始局部法向下的场景真值；基线坐标系仅由观测拟合。"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import yaml

WORKSPACE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORKSPACE / 'src/hydromining_sim/scripts'))
from algorithms.perception.frames import backproject_depth, fit_reference_plane, transform_points
from algorithms.perception.metrics import mesh_heights, truth_change
from algorithms.perception.wall_frame import WallFrame


def reference_frame(paths, center, radius):
    """从喷前局部点拟合朝观测者法向，不读取网格。"""
    selected, viewpoints = [], []
    for path in paths:
        with np.load(path) as data:
            world = transform_points(backproject_depth(data['depth'], data['K']), data['tf_depth'])
            points = world.reshape(-1, 3)
            keep = np.isfinite(points).all(axis=1) & (np.linalg.norm(points - center, axis=1) < radius)
            selected.append(points[keep])
            viewpoints.append(data['tf_depth'][:3])
    if not selected:
        raise ValueError('baseline observations are required')
    _, normal, residual = fit_reference_plane(
        np.concatenate(selected), center, np.mean(viewpoints, axis=0))
    return WallFrame.from_normal(center, normal), residual


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output already exists')
    config = yaml.safe_load((WORKSPACE / 'src/hydromining_robot/config/viewpoints.yaml').read_text())
    center = np.array(config['region_center_world'])
    frame, residual = reference_frame(sorted((args.dataset / 'baseline').glob('*.npz')), center, 0.15)
    # 此位姿只用于真值评估，不能输入感知重建。
    pose = [1.5, 0., 1.5, 0., -1.9198621771937625, 0.]
    report = {'normal_world': frame.axes[:, 2].tolist(), 'origin_world': frame.origin.tolist(),
              'axes_world': frame.axes.tolist(), 'baseline_plane_residual_m': residual,
              'calibrated': False, 'scope': 'fixed simulator meshes, not physical coal truth',
              'scenarios': {}}
    for resolution in (0.004, 0.002, 0.001):
        before = mesh_heights(args.dataset / 'truth/baseline.obj', pose, frame, 0.24, resolution)
        for name in json.loads((args.dataset / 'truth/truth.json').read_text()):
            after = mesh_heights(args.dataset / f'truth/{name}.obj', pose, frame, 0.24, resolution)
            row = truth_change(before, after, frame, 0.24, resolution, 0.1, 0.02, 0.9)
            report['scenarios'].setdefault(name, []).append(row)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False))
    print(json.dumps(report, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
