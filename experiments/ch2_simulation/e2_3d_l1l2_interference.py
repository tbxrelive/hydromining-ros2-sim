"""E2.3d 干扰等级接入 L1/L2：在真实替身渲染路径上验证四档可分。

不跑完整回合，而是直接调用 `SurrogateWorld.render_depth / render_lidar`
（L1/L2 感知链使用的同一函数），对每个等级统计受扰比例 D，
确认接入后等级仍单调可分，且 `visibility=inf` 与改动前逐位一致。

命令：
    python experiments/ch2_simulation/e2_3d_l1l2_interference.py
"""
import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT/'src/hydromining_sim/scripts'))

from algorithms.perception.frames import backproject_depth  # noqa: E402
from algorithms.perception.kinematics import look_at  # noqa: E402
from algorithms.perception.wall_frame import WallFrame  # noqa: E402
from algorithms.perception.environment import sensor_rng  # noqa: E402
from algorithms.planning.surrogate import SurrogateWorld  # noqa: E402
from experiments.common.interference import disturbed_fraction, load_levels, sensor_tolerance  # noqa: E402
from experiments.common.task import load_task  # noqa: E402

IMAGE_SHAPE = (240, 320)
FOCAL = 320/(2*np.tan(.75))
K = np.array([[FOCAL, 0., 160.], [0., FOCAL, 120.], [0., 0., 1.]])
WALL = ROOT/'src/hydromining_sim/models/coal_wall/meshes/coal_wall.obj'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default=str(ROOT/'results/thesis/E2.3d'))
    parser.add_argument('--repeats', type=int, default=5)
    parser.add_argument('--seed', type=int, default=20261101)
    args = parser.parse_args()

    task = load_task()
    levels = load_levels()
    frame = WallFrame.from_normal(task.region_center, task.wall_normal)
    world = SurrogateWorld(str(WALL), (96, 96, 64), .2,
                           mesh_pose=tuple(task.source_coal_pose[:3]) + tuple(task.source_coal_pose[3:]))
    started = time.monotonic()
    rows = []
    for view_name, view in task.viewpoints.items():
        matrix = look_at(np.array(view['camera_position_world']), frame.origin)
        # render_* 需要 [x,y,z,qx,qy,qz,qw]；look_at 给的是 4×4 齐次矩阵。
        pose = np.r_[matrix[:3, 3], Rotation.from_matrix(matrix[:3, :3]).as_quat()]
        clean_depth = world.render_depth(pose, K, IMAGE_SHAPE, rng=None)
        clean_points = backproject_depth(clean_depth, K).reshape(-1, 3)
        clean_lidar = world.render_lidar(pose, rng=None)
        for level_name, level in levels.items():
            for gear in (1.0, 2.0):
                for repeat in range(args.repeats):
                    rng = sensor_rng(args.seed+repeat, f'{view_name}_{level_name}_{gear}', 'depth')
                    depth = world.render_depth(pose, K, IMAGE_SHAPE, rng=rng,
                                               noise_scale=level.noise_scale*gear,
                                               visibility=level.visibility_m)
                    lidar = world.render_lidar(pose, rng=rng,
                                               noise_scale=level.noise_scale*gear,
                                               visibility=level.visibility_m)
                    observed = backproject_depth(depth, K).reshape(-1, 3)
                    depth_cloud = disturbed_fraction(
                        clean_points, observed,
                        sensor_tolerance(np.linalg.norm(clean_points, axis=1), 'depth'))
                    lidar_cloud = disturbed_fraction(
                        clean_lidar, lidar,
                        sensor_tolerance(np.linalg.norm(clean_lidar, axis=1), 'lidar'))
                    rows.append({'view': view_name, 'level': level_name, 'noise_gear': gear,
                                 'repeat': repeat,
                                 'disturbed_depth': depth_cloud['disturbed_fraction'],
                                 'disturbed_lidar': lidar_cloud['disturbed_fraction'],
                                 'target_returns': depth_cloud['target_returns']})

    names = list(levels)
    report = {'levels': {k: v.as_dict() for k, v in levels.items()},
              'repeats': args.repeats, 'views': list(task.viewpoints),
              'wall_time_s': time.monotonic()-started, 'summary': {}}
    for gear in (1.0, 2.0):
        for name in names:
            values = [r['disturbed_depth'] for r in rows
                      if r['level'] == name and r['noise_gear'] == gear]
            report['summary'][f'{name}@{gear:g}x'] = {
                'depth_mean': float(np.mean(values)), 'depth_std': float(np.std(values, ddof=1)),
                'lidar_mean': float(np.mean([r['disturbed_lidar'] for r in rows
                                             if r['level'] == name and r['noise_gear'] == gear])),
                'n': len(values)}
    # 单调性：同一噪声档下，四档均值必须严格递增
    report['monotone'] = {}
    for gear in (1.0, 2.0):
        means = [report['summary'][f'{name}@{gear:g}x']['depth_mean'] for name in names]
        report['monotone'][f'{gear:g}x'] = bool(all(a < b for a, b in zip(means[:-1], means[1:])))

    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output/'l1l2_levels.json').write_text(json.dumps(report, indent=2, ensure_ascii=False))
    (output/'rows.jsonl').write_text('\n'.join(json.dumps(r) for r in rows) + '\n')
    print(f'{"等级@噪声":>14} {"深度D":>9} {"LiDAR D":>9}')
    for key, block in report['summary'].items():
        print(f'{key:>14} {block["depth_mean"]*100:8.2f}% {block["lidar_mean"]*100:8.2f}%')
    print('单调:', report['monotone'])
    print(f'结果写入 {output}')


if __name__ == '__main__':
    main()
