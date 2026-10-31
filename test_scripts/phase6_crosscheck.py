#!/usr/bin/env python3
"""辅助 TSDF 与官方 py4dgeo M3C2-EP 交叉验证，仅使用真实观测。"""
import argparse
import json
import multiprocessing.shared_memory
import sys
from pathlib import Path

import numpy as np
import open3d as o3d
import py4dgeo

from phase6_evaluate import load_groups

WORKSPACE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORKSPACE / 'src/hydromining_sim/scripts'))
from algorithms.perception.alignment import aligned_sensor
from algorithms.perception.tsdf_mesh import TSDFReconstructor
from algorithms.perception.wall_frame import WallFrame


def epoch_from_groups(groups, frame, extent=0.3):
    """保留扫描来源；sigma 是工程交叉验证值，不冒充 RGB-D 校准模型。"""
    clouds, scan_ids, scan_info = [], [], []
    for snapshots in groups:
        snapshot = snapshots[-1]
        world, _, _ = aligned_sensor(snapshot, 'depth')
        points = world.reshape(-1, 3)
        local = frame.to_local(points)
        valid = np.isfinite(local).all(axis=1)
        valid &= (np.max(np.abs(local[:, :2]), axis=1) < extent / 2)
        valid &= np.abs(local[:, 2]) < 0.2
        points = points[valid][::2]
        scan_info.append({'origin': snapshot['tf_depth'][:3].tolist(),
                          'sigma_range': 0.01, 'sigma_scan': 0.0005, 'sigma_yaw': 0.0005})
        clouds.append(points)
        scan_ids.append(np.full(len(points), len(scan_info), dtype=np.int32))
    epoch = py4dgeo.Epoch(np.ascontiguousarray(np.concatenate(clouds)), scanpos_info=scan_info)
    epoch.scanpos_id = np.concatenate(scan_ids)
    return epoch


def crosscheck(dataset, state_path, scenario, output):
    """原始点云 M3C2-EP 与高度图变化比较，TSDF 只负责独立几何重建。"""
    if output.exists():
        raise ValueError('crosscheck output already exists')
    output.mkdir(parents=True)
    with np.load(state_path) as state:
        frame = WallFrame(state['origin'], state['axes'])
        baseline_height = state['baseline_height']
        change = state['change_delta']
        valid = state['change_valid']
    baseline_groups = load_groups(dataset / 'baseline')
    current_groups = load_groups(dataset / scenario)
    core_grid = frame.grid(0.5, 0.01)
    mask = valid & (np.linalg.norm(core_grid[..., :2], axis=-1) < 0.1)
    core_grid[..., 2] = baseline_height
    indices = np.argwhere(mask)[::5]
    if len(indices) < 16:
        raise ValueError('insufficient reliable corepoints')
    corepoints = frame.to_world(core_grid)[indices[:, 0], indices[:, 1]]
    epochs = (epoch_from_groups(baseline_groups, frame), epoch_from_groups(current_groups, frame))
    covariance = np.zeros((12, 12))
    covariance[9:, 9:] = np.eye(3) * 0.001 ** 2
    algorithm = py4dgeo.M3C2EP(
        epochs=epochs, corepoints=np.ascontiguousarray(corepoints),
        corepoint_normals=np.ascontiguousarray(frame.axes[:, 2][None, :]),
        cyl_radius=0.01, max_distance=0.2, Cxx=covariance,
        perform_trans=False)
    distances, uncertainties, _ = algorithm.run()
    expected = change[indices[:, 0], indices[:, 1]]
    comparable = np.isfinite(distances) & np.isfinite(expected)
    np.savez_compressed(output / 'm3c2ep.npz', corepoints=corepoints,
                        retreat=distances, uncertainty=uncertainties, grid_retreat=expected)
    tsdf = TSDFReconstructor()
    for snapshots in current_groups:
        tsdf.integrate(snapshots[-1], frame)
    mesh = tsdf.mesh()
    if not len(mesh.vertices):
        raise ValueError('TSDF produced an empty mesh')
    o3d.io.write_triangle_mesh(str(output / 'tsdf_mesh.ply'), mesh)
    report = {
        'py4dgeo_version': py4dgeo.__version__, 'corepoints': len(corepoints),
        'comparable_corepoints': int(comparable.sum()),
        'm3c2ep_grid_rmse_m': float(np.sqrt(np.mean(
            (distances[comparable] - expected[comparable]) ** 2))) if comparable.any() else None,
        'tsdf_vertices': len(mesh.vertices), 'tsdf_triangles': len(mesh.triangles),
        'calibrated': False,
        'scope': 'clean original point-cloud crosscheck; EP scanning sigma is an engineering assumption',
    }
    (output / 'summary.json').write_text(json.dumps(report, indent=2, allow_nan=False))
    print(json.dumps(report, indent=2))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--state', type=Path, required=True)
    parser.add_argument('--scenario', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    crosscheck(args.dataset, args.state, args.scenario, args.output)


if __name__ == '__main__':
    main()
