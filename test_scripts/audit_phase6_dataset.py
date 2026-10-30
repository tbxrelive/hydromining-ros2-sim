#!/usr/bin/env python3
"""独立核对录制快照的坐标与真实网格，不将真值输入感知算法。"""
import argparse
import json
import sys
from pathlib import Path
import numpy as np
import open3d as o3d
import trimesh
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/hydromining_sim/scripts'))
from algorithms.perception.frames import backproject_depth, transform_points


def make_scene(path):
    """从归档网格新建查询场景，独立于喷射求解器。"""
    mesh = trimesh.load(path, process=False)
    scene = o3d.t.geometry.RaycastingScene(nthreads=1)
    scene.add_triangles(
        o3d.core.Tensor(np.ascontiguousarray(mesh.vertices, np.float32)),
        o3d.core.Tensor(np.ascontiguousarray(mesh.faces, np.uint32)))
    return scene


def residuals(scene, world, mesh_rotation, mesh_translation, center):
    """只评估作业区域周边实际观测点，不用真值反向补齐缺失点。"""
    points = world.reshape(-1, 3)
    valid = np.isfinite(points).all(axis=1)
    region = valid & (np.linalg.norm(points - center, axis=1) < 0.30)
    selected = (points[region] - mesh_translation) @ mesh_rotation
    if not len(selected):
        return {'points': 0, 'p95_m': None, 'rmse_m': None}
    distance = scene.compute_distance(
        o3d.core.Tensor(selected.astype(np.float32)), nthreads=1).numpy()
    return {'points': len(selected), 'p95_m': float(np.percentile(distance, 95)),
            'rmse_m': float(np.sqrt(np.mean(distance ** 2)))}


def audit(root, output):
    """核查四种场景、两种传感器以及快照完整性，输出机器可读报告。"""
    truth = json.loads((root / 'truth/truth.json').read_text())
    center = np.array([1.517, 0.011, 0.694])
    rotation = Rotation.from_euler('xyz', [0, -1.9198621771937625, 0]).as_matrix()
    translation = np.array([1.5, 0, 1.5])
    rows = []
    for scenario in ['baseline', *truth]:
        scene = make_scene(root / 'truth' / f'{scenario}.obj')
        paths = sorted((root / scenario).glob('*.npz'))
        if not paths:
            raise ValueError(f'Missing observations for {scenario}')
        for path in paths:
            with np.load(path) as data:
                depth = data['depth']
                points = backproject_depth(depth, data['K'])
                camera = transform_points(points, data['tf_depth'])
                lidar = transform_points(data['lidar_local'], data['tf_lidar'])
                camera_stats = residuals(scene, camera, rotation, translation, center)
                lidar_stats = residuals(scene, lidar, rotation, translation, center)
                rows.append({'snapshot': str(path.relative_to(root)), 'camera': camera_stats,
                             'lidar': lidar_stats,
                             'depth_stamp': data['depth_stamp'].tolist(),
                             'lidar_stamp': data['lidar_stamp'].tolist()})
    # 这里验证的是洁净、无噪声数据，不是最终感知指标。
    good = all(row['camera']['points'] >= 20 and row['camera']['p95_m'] <= 0.003 and
               row['lidar']['points'] >= 10 and row['lidar']['p95_m'] <= 0.01 for row in rows)
    report = {'passed': good, 'snapshot_count': len(rows), 'rows': rows,
              'scope': 'clean snapshots only; temporal sequences and native interference not covered',
              'max_camera_p95_m': max(row['camera']['p95_m'] or 0 for row in rows),
              'max_lidar_p95_m': max(row['lidar']['p95_m'] or 0 for row in rows)}
    output.write_text(json.dumps(report, indent=2))
    print(json.dumps({k: v for k, v in report.items() if k != 'rows'}, indent=2))
    return good


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    return 0 if audit(args.dataset, args.output) else 1


if __name__ == '__main__':
    sys.exit(main())
