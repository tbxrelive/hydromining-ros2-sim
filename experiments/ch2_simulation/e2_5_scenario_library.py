"""E2.5 场景库与种子划分：生成、复现性校验、60° 倾角核对、全库规则基线。

完成标准（计划第七节阶段 2）：
1. 场景库与真值可复现：同一种子在同一进程与新进程里生成逐位一致；
2. 划分不相交；
3. 难度覆盖：初始已达标比例 ≤ 30%；
4. 全库跑一遍规则基线，作为论文 B 的规则参照与 E5.pre 的基准。

命令：
    python experiments/ch2_simulation/e2_5_scenario_library.py
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT/'src/hydromining_sim/scripts'))

from experiments.common import scenarios as scenario_lib  # noqa: E402
from experiments.common import splits  # noqa: E402
from experiments.common.crater import flat_plate  # noqa: E402
from experiments.common.large_env import prepare_meshes  # noqa: E402

OUTPUT = ROOT/'results/thesis/E2.5'
ASSETS = OUTPUT/'assets'


def library_hash(library):
    """库清单的 sha256，用于跨进程比对。"""
    text = json.dumps(scenario_lib.library_dict(library), sort_keys=True, ensure_ascii=False)
    return hashlib.sha256(text.encode('utf-8')).hexdigest()


def heightfield_hash(scenario, kernel, plate, pressure, standoff):
    """在平整靶上施加该工况，返回高度场的 sha256（float64 字节）。"""
    from algorithms import heightfield_erosion
    extent, spacing = plate
    heights, grid, _ = flat_plate(extent, spacing)
    for (u, v), duration in scenario_lib.as_shots(scenario):
        heightfield_erosion.spray_heightfield(heights, grid, u, v, duration,
                                              pressure, standoff, kernel, spacing, extent)
    return hashlib.sha256(np.ascontiguousarray(heights, dtype=np.float64).tobytes()).hexdigest()


def scenario_depth_check(assets, library, task):
    """核对每个工况的预喷深度，避免"初始工况自己把墙打穿"。"""
    from algorithms import heightfield_erosion
    from algorithms.perception.metrics import mesh_heights
    from algorithms.perception.wall_frame import WallFrame
    frame = WallFrame.from_normal(task.region_center, task.wall_normal)
    kernel = task.erosion_kernel()
    spacing, extent = task.geometry_spacing_m, task.patch_extent_m
    heights_cache = {}
    records = []
    for split, scenarios in library.items():
        seeds = scenario_lib.WALL_SEEDS[split]
        for index, scenario in enumerate(scenarios):
            seed = seeds[index % len(seeds)]
            if seed not in heights_cache:
                heights_cache[seed] = mesh_heights(
                    str(Path(assets)/f'coal_{seed}.obj'), task.source_coal_pose, frame,
                    extent, spacing)
            initial = heights_cache[seed].copy()
            heights = initial.copy()
            # 预喷只用到 u,v 坐标，与格点网格一致
            size = initial.shape[0]
            axis = (np.arange(size) + .5)*spacing - extent/2
            u_grid, v_grid = np.meshgrid(axis, axis)
            grid = np.stack((u_grid, v_grid, np.zeros_like(u_grid)), axis=-1)
            for (u, v), duration in scenario_lib.as_shots(scenario):
                if not duration or duration <= 0:
                    continue
                heightfield_erosion.spray_heightfield(heights, grid, u, v, min(duration, 8.),
                                                      task.pressure_mpa, task.standoff_m,
                                                      kernel, spacing, extent)
            depth = float(np.max(initial - heights))
            records.append({'split': split, 'scenario': scenario.name,
                            'family': scenario.family, 'wall_seed': seed,
                            'initial_erosion_depth_m': depth,
                            'margin_to_wall_m': float(task.wall_thickness_m - depth)})
    limit = .5*task.wall_thickness_m
    return {'records': records,
            'max_depth_m': float(max(r['initial_erosion_depth_m'] for r in records)),
            'limit_m': float(limit),
            'all_below_half_thickness': bool(
                all(r['initial_erosion_depth_m'] <= limit for r in records))}


def subprocess_library_hash():
    """在新进程里生成一次库，返回哈希；用来排除进程内缓存的影响。"""
    code = ('import sys, json; sys.path.insert(0, %r); '
            'from experiments.common import scenarios; '
            'from experiments.ch2_simulation.e2_5_scenario_library import library_hash; '
            'print(library_hash(scenarios.build_library()))' % str(ROOT))
    result = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True,
                            check=True, cwd=ROOT)
    return result.stdout.strip()


def check_wall_tilt(assets, seed, task):
    """核对墙面法向与任务配置是否一致，并给出相对重力的倾角。"""
    import trimesh
    from scipy.spatial.transform import Rotation
    mesh = trimesh.load(Path(assets)/f'coal_{seed}.obj', force='mesh', process=False)
    rotation = Rotation.from_euler('xyz', task.source_coal_pose[3:]).as_matrix()
    world = np.asarray(mesh.vertices) @ rotation.T + np.asarray(task.source_coal_pose[:3])
    local = (world - np.asarray(task.region_center)) @ np.asarray(task.wall_normal)
    # 煤壁是一块板：投影到法向上会落在两个面。取朝向机械臂的那个面（投影最大）做拟合。
    near = world[local >= local.max() - .03]
    if len(near) < 10:
        return {'wall_seed': seed, 'points': int(len(near)), 'error': 'insufficient points'}
    centred = near - near.mean(axis=0)
    _, _, vh = np.linalg.svd(centred, full_matrices=False)
    fitted = vh[2]
    if np.dot(fitted, np.asarray(task.wall_normal)) < 0:
        fitted = -fitted
    angle_to_config = float(np.degrees(np.arccos(
        np.clip(np.dot(fitted, np.asarray(task.wall_normal)), -1., 1.))))
    inclination = float(np.degrees(np.arccos(
        np.clip(abs(fitted[2]), 0., 1.))))  # 法向与竖直方向的夹角
    return {'wall_seed': seed, 'points': int(len(near)),
            'angle_to_configured_normal_deg': angle_to_config,
            'fitted_plane_inclination_from_vertical_deg': inclination,
            'configured_wall_inclination_deg': float(task.wall_inclination_deg)}


def run_rule(assets, split, library, task, seed=10000, limit=0):
    """在给定划分上用规则策略跑一遍。"""
    from experiments.common.large_env import make_fast_env
    wall_seeds = scenario_lib.WALL_SEEDS[split]
    table = {scenario.name: {'shots': scenario_lib.as_shots(scenario)}
             for scenario in library[split]}
    env = make_fast_env(assets, seed, split, wall_seeds=wall_seeds, scenarios=table)
    records = []
    try:
        for index, scenario in enumerate(library[split][:limit or None]):
            mesh_index = index % len(wall_seeds)
            obs, info = env.reset(seed=seed+index,
                                  options={'scenario': scenario.name, 'mesh_index': mesh_index})
            if env.initially_achieved_:
                records.append({'split': split, 'scenario': scenario.name,
                                'family': scenario.family, 'initially_achieved': True,
                                'success': True, 'verdict': 'initially_achieved',
                                'sprays': 0, 'steps': 0,
                                'duration_s': 0., 'removed_volume_m3': 0.,
                                'truth_fraction': info['truth_fraction']})
                continue
            while True:
                _, _, terminated, truncated, info = env.step(env.rule_action())
                if terminated or truncated:
                    break
            retreat = env._map_heights(env.initial_-env.heights_)
            records.append({'split': split, 'scenario': scenario.name,
                            'family': scenario.family, 'initially_achieved': False,
                            'success': bool(info['verdict'] == 'achieved'),
                            'verdict': info['verdict'],
                            'sprays': int(env.sprays_used_), 'steps': int(env.steps_used_),
                            'duration_s': float(env.total_spray_duration_),
                            'removed_volume_m3': float(np.sum(np.maximum(retreat, 0)*env.area_)),
                            'truth_fraction': float(info['truth_fraction'])})
    finally:
        env.close()
    return records


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default=str(OUTPUT))
    parser.add_argument('--limit', type=int, default=0, help='每个划分只跑前 N 个（冒烟用）')
    parser.add_argument('--skip-baseline', action='store_true')
    args = parser.parse_args()

    output = Path(args.output)
    assets = output/'assets'
    output.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    from experiments.common.task import load_task
    task = load_task()
    kernel = task.erosion_kernel()
    plate = (task.patch_extent_m, task.geometry_spacing_m)

    # 1. 生成并保存场景库
    library = scenario_lib.build_library()
    scenario_lib.save_library(output/'library.json', library)
    counts = {split: len(items) for split, items in library.items()}

    # 2. 跨进程逐位一致
    first_hash = library_hash(library)
    second_hash = library_hash(scenario_lib.build_library())
    other_process_hash = subprocess_library_hash()

    # 3. 高度场逐位一致（抽样，覆盖每个族）
    height_hashes = {}
    sample = [item for item in library['test'] if item.family in
              ('single', 'pair', 'deep_pit')][:3]
    for scenario in sample:
        height_hashes[scenario.name] = heightfield_hash(scenario, kernel, plate,
                                                       task.pressure_mpa, task.standoff_m)

    # 4. 划分不相交
    splits.assert_disjoint()

    # 5. 墙面倾角核对
    prepare_meshes(assets, seeds=scenario_lib.WALL_SEEDS['test'])
    tilts = [check_wall_tilt(assets, seed, task) for seed in scenario_lib.WALL_SEEDS['test'][:2]]

    # 5b. 预喷深度核对：初始工况本身不得接近或打穿壁厚
    prepare_meshes(assets, seeds=[s for seeds in scenario_lib.WALL_SEEDS.values() for s in seeds])
    depth_check = scenario_depth_check(assets, library, task)

    # 6. 全库规则基线（墙面已在上一步补齐）
    records = []
    if not args.skip_baseline:
        for split in library:
            records.extend(run_rule(assets, split, library, task, limit=args.limit))
    (output/'episodes.jsonl').write_text(
        '\n'.join(json.dumps(r, ensure_ascii=False) for r in records) + '\n')

    by_split = {}
    for split in library:
        subset = [r for r in records if r['split'] == split]
        if not subset:
            continue
        by_split[split] = {
            'episodes': len(subset),
            'success_rate': float(np.mean([r['success'] for r in subset])),
            'through_cut_rate': float(np.mean([r['verdict'] == 'through_cut' for r in subset])),
            'initially_achieved_fraction': float(np.mean([r['initially_achieved'] for r in subset])),
            'mean_sprays': float(np.mean([r['sprays'] for r in subset])),
            'mean_duration_s': float(np.mean([r['duration_s'] for r in subset])),
            'mean_removed_volume_m3': float(np.mean([r['removed_volume_m3'] for r in subset])),
        }
    all_initial = float(np.mean([r['initially_achieved'] for r in records])) if records else None

    report = {
        'library_hash': first_hash,
        'reproducible': bool(first_hash == second_hash == other_process_hash),
        'hash_same_process': bool(first_hash == second_hash),
        'hash_other_process': bool(first_hash == other_process_hash),
        'counts': counts,
        'wall_seeds': {k: list(v) for k, v in scenario_lib.WALL_SEEDS.items()},
        'strength_seeds_reserved': list(scenario_lib.STRENGTH_SEEDS),
        'heightfield_hashes': height_hashes,
        'wall_tilt': tilts,
        'scenario_depth_check': {
            'max_depth_m': depth_check['max_depth_m'],
            'limit_m': depth_check['limit_m'],
            'all_below_half_thickness': depth_check['all_below_half_thickness'],
            'worst': sorted(depth_check['records'],
                            key=lambda r: -r['initial_erosion_depth_m'])[:5]},
        'rule_baseline': by_split,
        'initially_achieved_fraction_all': all_initial,
        'initially_achieved_within_limit': bool(all_initial is not None and all_initial <= .30),
        'wall_time_s': time.monotonic()-started,
    }
    (output/'summary.json').write_text(json.dumps(report, indent=2, ensure_ascii=False))
    def relative(path):
        """尽量写成相对仓库根的路径，仓库外则用绝对路径。"""
        try:
            return str(Path(path).relative_to(ROOT))
        except ValueError:
            return str(path)

    (output/'manifest.json').write_text(json.dumps({
        'experiment': 'E2.5', 'library': relative(output/'library.json'),
        'assets': relative(assets), 'seed': 10000,
        'kernel': kernel.params()}, indent=2, ensure_ascii=False))

    print(json.dumps({k: v for k, v in report.items()
                      if k not in ('heightfield_hashes', 'wall_tilt')},
                     indent=2, ensure_ascii=False)[:2200])
    print('墙面倾角核对：')
    for item in tilts:
        print(f"  墙 {item['wall_seed']}: 与配置法向夹角 "
              f"{item['angle_to_configured_normal_deg']:.2f}°，拟合倾角 "
              f"{item['fitted_plane_inclination_from_vertical_deg']:.2f}° "
              f"（配置 {item['configured_wall_inclination_deg']:.1f}°）")


if __name__ == '__main__':
    main()
