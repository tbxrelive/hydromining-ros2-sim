"""E2.3b 干扰等级定义与重复统计：验证四档在统计上可分。

判据（写死在 `stats.adjacent_separation` 里，四条同时成立）：
  1. |Δ均值| > 2 × 合并标准差
  2. Mann–Whitney p < 0.01
  3. |Δ均值| ≥ 3 个百分点（防"统计显著但差异空洞"）
  4. 顺序正确（无 < 低 < 中 < 高）

统计单位：每个 (噪声档, 墙面) 组合内，把视点包络上的全部重复合并成一个样本，
再逐对相邻等级比较。另报"每个视点格子都单调"的比例。

命令：
    python experiments/ch2_simulation/e2_3b_interference_levels.py
"""
import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT/'src/hydromining_sim/scripts'))

from experiments.common.interference import (  # noqa: E402
    disturbed_fraction, load_levels, sensor_tolerance)
from algorithms.perception.environment import degrade_snapshot, sensor_rng  # noqa: E402
from algorithms.perception.frames import backproject_depth  # noqa: E402
from experiments.common.stats import adjacent_separation, summarize  # noqa: E402

IMAGE_SHAPE = (240, 320)
FOCAL = 320/(2*np.tan(.75))
K = np.array([[FOCAL, 0., 160.], [0., FOCAL, 120.], [0., 0., 1.]])
DISTANCES = (.5, .8, 1.2, 1.6, 2.0)
ANGLES = (0., 20., 40., 60.)
WALL_ROUGHNESS = {'smooth': 0., 'rough_5mm': .005, 'rough_15mm': .015}


def plane_geometry(distance, angle_deg, roughness=0., seed=0):
    """倾斜粗糙平面：在理想平面上叠加低频起伏，模拟不同墙面。"""
    rows, cols = np.indices(IMAGE_SHAPE)
    a = (cols - K[0, 2])/K[0, 0]
    b = (rows - K[1, 2])/K[1, 1]
    theta = np.deg2rad(angle_deg)
    denominator = np.cos(theta) - a*np.sin(theta)
    with np.errstate(divide='ignore', invalid='ignore'):
        depth = distance*np.cos(theta)/denominator
    if roughness:
        rng = np.random.default_rng(seed)
        phase = rng.uniform(0., 2*np.pi, 4)
        ripple = (np.sin(2*np.pi*a/0.4+phase[0])*np.sin(2*np.pi*b/0.4+phase[1])
                  + .5*np.sin(2*np.pi*a/0.13+phase[2])*np.sin(2*np.pi*b/0.17+phase[3]))
        depth = depth + roughness*ripple/1.5
    depth = np.where((denominator > .2) & (depth > 0), depth, np.nan)
    direction = np.stack((np.ones_like(a), -a, -b), axis=-1)
    points = (direction*depth[..., None]).reshape(-1, 3)
    lidar = (direction[::8, ::8]*depth[::8, ::8, None]).reshape(-1, 3)
    return depth, points, lidar


def trial(depth, clean_points, lidar, visibility, noise_scale, seed, snapshot):
    """一次独立重复：返回深度相机/LiDAR 受扰比例与有效覆盖率。"""
    degraded = degrade_snapshot(depth, K, lidar, visibility, noise_scale,
                               sensor_rng(seed, snapshot, 'depth'),
                               sensor_rng(seed, snapshot, 'lidar'))
    observed = backproject_depth(degraded['depth'], K).reshape(-1, 3)
    depth_cloud = disturbed_fraction(clean_points, observed,
                                     sensor_tolerance(np.linalg.norm(clean_points, axis=1), 'depth'))
    lidar_cloud = disturbed_fraction(lidar, degraded['lidar_local'],
                                     sensor_tolerance(np.linalg.norm(lidar, axis=1), 'lidar'))
    target = np.isfinite(clean_points).all(axis=1)
    usable = target & np.isfinite(observed).all(axis=1)
    return {'disturbed_depth': depth_cloud['disturbed_fraction'],
            'disturbed_lidar': lidar_cloud['disturbed_fraction'],
            'finite_fraction': float(usable.sum()/max(target.sum(), 1))}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default=str(ROOT/'results/thesis/E2.3b'))
    parser.add_argument('--repeats', type=int, default=30)
    parser.add_argument('--seed', type=int, default=20261101)
    parser.add_argument('--levels', default=str(ROOT/'configs/interference_levels.yaml'))
    args = parser.parse_args()

    levels = load_levels(args.levels)
    gears = (1.0, 2.0)
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    rows = []
    index = 0
    for wall_index, (wall_name, roughness) in enumerate(WALL_ROUGHNESS.items()):
        # 墙面起伏用固定种子，不能用 hash()（Python 的字符串哈希每次进程都不同）。
        geometries = {(d, a): plane_geometry(d, a, roughness, seed=1000+wall_index)
                      for d in DISTANCES for a in ANGLES}
        for level_name, level in levels.items():
            for gear in gears:
                for (distance, angle), (depth, clean_points, lidar) in geometries.items():
                    for repeat in range(args.repeats):
                        result = trial(depth, clean_points, lidar, level.visibility_m,
                                       level.noise_scale*gear, args.seed+repeat,
                                       f'{wall_name}_{level_name}_{gear}_{distance}_{angle}_{repeat}')
                        rows.append({'level': level_name, 'wall': wall_name,
                                     'noise_gear': gear, 'distance_m': distance,
                                     'angle_deg': angle, 'repeat': repeat, **result})
                index += 1

    names = list(levels)
    pairs = list(zip(names[:-1], names[1:]))
    separability = []
    for gear in gears:
        for wall_name in WALL_ROUGHNESS:
            for lower, upper in pairs:
                low_values = [r['disturbed_depth'] for r in rows
                              if r['level'] == lower and r['noise_gear'] == gear
                              and r['wall'] == wall_name]
                high_values = [r['disturbed_depth'] for r in rows
                               if r['level'] == upper and r['noise_gear'] == gear
                               and r['wall'] == wall_name]
                result = adjacent_separation(low_values, high_values)
                separability.append({'noise_gear': gear, 'wall': wall_name,
                                     'pair': f'{lower}->{upper}', **result})

    # 每个视点格子都单调的比例
    monotone_total = monotone_ok = 0
    for gear in gears:
        for wall_name in WALL_ROUGHNESS:
            for distance in DISTANCES:
                for angle in ANGLES:
                    means = [np.mean([r['disturbed_depth'] for r in rows
                                      if r['level'] == name and r['noise_gear'] == gear
                                      and r['wall'] == wall_name
                                      and r['distance_m'] == distance
                                      and r['angle_deg'] == angle])
                             for name in names]
                    monotone_total += 1
                    monotone_ok += int(all(a < b for a, b in zip(means[:-1], means[1:])))

    summary = {
        'levels': {name: level.as_dict() for name, level in levels.items()},
        'noise_gears': list(gears), 'repeats': args.repeats, 'seed': args.seed,
        'walls': WALL_ROUGHNESS, 'distances_m': list(DISTANCES), 'angles_deg': list(ANGLES),
        'separability': separability,
        'all_separable': bool(all(item['separable'] for item in separability)),
        'monotone_cell_fraction': float(monotone_ok/max(monotone_total, 1)),
        'monotone_cells': [monotone_ok, monotone_total],
        'level_summary': {},
        'wall_time_s': time.monotonic()-started,
    }
    for gear in gears:
        for name in names:
            values = [r['disturbed_depth'] for r in rows
                      if r['level'] == name and r['noise_gear'] == gear]
            block = summarize([{'v': v} for v in values], ('v',))
            summary['level_summary'][f'{name}@{gear:g}x'] = {
                'mean': block['v_mean'], 'std': block['v_std'], 'n': block['v_n']}

    (output/'levels.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False))
    (output/'rows.jsonl').write_text('\n'.join(json.dumps(r, ensure_ascii=False) for r in rows) + '\n')

    print(f'{"等级@噪声":>14} {"受扰比例均值":>14} {"标准差":>10}')
    for key, block in summary['level_summary'].items():
        print(f'{key:>14} {block["mean"]*100:13.2f}% {block["std"]*100:9.2f}%')
    print('\n相邻可分：', '全部通过' if summary['all_separable'] else '存在不通过')
    for item in separability:
        if not item['separable']:
            print(f'  不通过 {item["pair"]} gear={item["noise_gear"]} wall={item["wall"]} '
                  f'gap={item["gap_in_sd"]:.2f}σ diff={item["diff"]*100:.1f}pp p={item["p_value"]:.2g}')
    print(f'视点格子单调比例: {summary["monotone_cell_fraction"]*100:.1f}% '
          f'({monotone_ok}/{monotone_total})')
    print(f'结果写入 {output}')


if __name__ == '__main__':
    main()
