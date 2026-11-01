"""E2.1a 破煤响应扫描：压力、靶距、时长 → 孔深、孔径、体积。

目的：证明仿真的破煤响应与水射流破煤文献**趋势一致**，并给每个核参数
找到出处。当前文献依据（已核实的摘要结论）：

- Zhang et al. 2019（Applied Sciences 9(23):5250，DOI 10.3390/app9235250）：
  割缝深度随射流压力**线性**增加；随割缝时间增加，但增速逐渐减小并趋于稳定。
- Zhang & Zou 2018（Results in Physics，DOI 10.1016/j.rinp.2018.11.020）：
  射流冲击功率与冲蚀体积呈线性关系；给出射流压力沿程衰减规律。
- Crow 1973（DOI 10.1016/0148-9062(73)90006-5）：全文未获取，仅作方法学引用。

L0 在平板上做全因子扫描（便宜）；Level Set 在同样的平板上做子集交叉验证。
命令：
    python experiments/ch2_simulation/e2_1a_erosion_scan.py \
        --config configs/experiments/E2.1a.yaml --output results/thesis/E2.1a/l0
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import time

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT/'src/hydromining_sim/scripts'))

from experiments.common.crater import flat_plate, measure, through_check  # noqa: E402
from experiments.common.records import RunWriter  # noqa: E402
from experiments.common.task import load_task  # noqa: E402

METRICS = ('depth_center_m', 'depth_max_m', 'diameter_half_m', 'diameter_abs_m',
           'volume_m3', 'symmetry')


def build_kernel(config, override=None):
    """按配置构造冲蚀核；override（JSON 字符串）优先，其次 config 的 kernel 段，
    最后用任务配置里的参数。三轮核的历史对照靠 override 复现。"""
    from algorithms.erosion_model import DefaultErosionKernel
    if override:
        params = json.loads(override)
        return DefaultErosionKernel(**params), 'cli'
    params = config.get('kernel')
    if params is None:
        return load_task().erosion_kernel(), 'task'
    return DefaultErosionKernel(**params), 'config'


def l0_condition(kernel, plate, pressure, standoff, duration, shots=1, center=(0., 0.)):
    """在平整高度场上喷射若干次，返回孔形度量。"""
    from algorithms import heightfield_erosion
    extent, spacing = plate['extent_m'], plate['spacing_m']
    heights, grid, _ = flat_plate(extent, spacing)
    before = heights.copy()
    removed = 0.
    for _ in range(shots):
        stats = heightfield_erosion.spray_heightfield(
            heights, grid, center[0], center[1], duration, pressure, standoff,
            kernel, spacing, extent)
        removed += stats.get('volume_removed_m3', 0.)
    result = measure(before, heights, spacing, center=center)
    result['through'] = through_check(before, heights, plate['thickness_m'])
    result['solver_volume_m3'] = float(removed)
    return result


def levelset_condition(kernel, config, pressure, standoff, duration, center=(0., 0.)):
    """在同一平板上用 Level Set 求解，再用射线取高度场做同口径度量。"""
    from algorithms.local_spray_simulation import surface_mesh, surface_rays
    from algorithms.erosion_model import SprayAction
    from algorithms.spray_simulation import SpraySimulation
    import open3d as o3d

    level = config['levelset']
    spacing = config['plate']['spacing_m']
    span, thickness = level['span_m'], level['thickness_m']
    size = int(round(span/spacing))
    axis = (np.arange(size) + .5)*spacing - span/2
    plate_heights = np.zeros((size, size))
    # 边界两格射线可能擦过侧壁打空，取内缩窗口做度量，避免 NaN 污染统计量。
    ray_axis = axis[2:-2]
    with tempfile.TemporaryDirectory() as directory:
        mesh_path = Path(directory)/'plate.obj'
        surface_mesh(axis, axis, plate_heights, thickness).export(mesh_path)
        sim = SpraySimulation(str(mesh_path), tuple(level['resolution']),
                              wall_budget_s=level['wall_budget_s'], reinitialize_interval_s=.5,
                              kernel=kernel)
        before = surface_rays(sim.model.surface_scene(
            sim.initial_phi, sim.grid.origin, sim.grid.spacing), ray_axis, ray_axis).copy()
        if np.isfinite(before).mean() < .99:
            raise ValueError(f'喷前射线命中率过低: {np.isfinite(before).mean():.3f}')
        # 平板喷前高度按定义为 0；少数未命中处用 0 补齐。
        before = np.where(np.isfinite(before), before, 0.)
        jet = SprayAction([center[0], center[1], standoff], [0., 0., -1.], pressure, duration)
        phi, stats = sim.simulate(jet)
        sim.commit(phi)
        after = surface_rays(sim.model.surface_scene(
            phi, sim.grid.origin, sim.grid.spacing), ray_axis, ray_axis)
    # 射线未命中处记为 NaN，度量前用喷前高度补齐，避免把"没打到"当成"打穿了"。
    after = np.where(np.isfinite(after), after, before)
    result = measure(before, after, spacing, center=center)
    result['through'] = through_check(before, after, thickness)
    result['solver_volume_m3'] = stats['volume_removed_m3']
    result['steps'] = stats['steps']
    result['elapsed_s'] = stats['elapsed_s']
    return result


def fit_power(x, y, mask=None):
    """log-log 拟合 y = a·x^n，返回指数 n、系数 a 与 R²。"""
    x, y = np.asarray(x, float), np.asarray(y, float)
    ok = np.isfinite(x) & np.isfinite(y) & (x > 0) & (y > 0)
    if mask is not None:
        ok &= np.asarray(mask, bool)
    if ok.sum() < 2:
        return None
    exponent, intercept = np.polyfit(np.log(x[ok]), np.log(y[ok]), 1)
    predicted = exponent*np.log(x[ok]) + intercept
    residual = np.sum((np.log(y[ok]) - predicted)**2)
    total = np.sum((np.log(y[ok]) - np.log(y[ok]).mean())**2)
    return {'exponent': float(exponent), 'coefficient': float(np.exp(intercept)),
            'r2': float(1 - residual/total) if total > 0 else 1., 'n': int(ok.sum())}


def fit_saturation(durations, depths):
    """逐段增长率：用来判断"增速递减、趋于稳定"。"""
    ratios = []
    for i in range(1, len(durations)):
        span = durations[i] - durations[i-1]
        if span <= 0 or not np.isfinite(depths[i-1]):
            continue
        ratios.append(float((depths[i] - depths[i-1])/span))
    return ratios


def summarize_fits(rows, config):
    """在 nominal 切片上分别拟合三个因素的响应曲线。

    多次喷射（shots>1）的记录只用于看饱和，不进入响应曲线拟合。
    """
    nominal = config['nominal']
    rows = [r for r in rows if r.get('shots', 1) == 1]
    # 打穿壁厚的条件不能进响应曲线：孔深已经超过壁厚，几何不再有效。
    rows = [r for r in rows if not r.get('through')]
    # 累加实验的 shots=1 记录与全因子记录重复，按 (P, s, t) 去重后只用第一条。
    seen, unique = set(), []
    for row in rows:
        key = (row['pressure_mpa'], row['standoff_m'], row['duration_s'])
        if key not in seen:
            seen.add(key)
            unique.append(row)
    rows = unique
    def slice_of(key):
        if key == 'pressure_mpa':
            return [r for r in rows if r['standoff_m'] == nominal['standoff_m']
                    and r['duration_s'] == nominal['duration_s']]
        if key == 'standoff_m':
            return [r for r in rows if r['pressure_mpa'] == nominal['pressure_mpa']
                    and r['duration_s'] == nominal['duration_s']]
        return [r for r in rows if r['pressure_mpa'] == nominal['pressure_mpa']
                and r['standoff_m'] == nominal['standoff_m']]
    pressure_rows = sorted(slice_of('pressure_mpa'), key=lambda r: r['pressure_mpa'])
    standoff_rows = sorted(slice_of('standoff_m'), key=lambda r: r['standoff_m'])
    duration_rows = sorted(slice_of('duration_s'), key=lambda r: r['duration_s'])
    return {
        'depth_vs_pressure': fit_power([r['pressure_mpa'] for r in pressure_rows],
                                       [r['depth_center_m'] for r in pressure_rows]),
        'volume_vs_pressure': fit_power([r['pressure_mpa'] for r in pressure_rows],
                                        [r['volume_m3'] for r in pressure_rows]),
        'depth_vs_standoff': fit_power([r['standoff_m'] for r in standoff_rows],
                                       [r['depth_center_m'] for r in standoff_rows]),
        'diameter_vs_pressure': fit_power([r['pressure_mpa'] for r in pressure_rows],
                                          [r['diameter_half_m'] for r in pressure_rows]),
        'depth_vs_duration': fit_power([r['duration_s'] for r in duration_rows],
                                       [r['depth_center_m'] for r in duration_rows]),
        'duration_slope_m_per_s': fit_saturation([r['duration_s'] for r in duration_rows],
                                                 [r['depth_center_m'] for r in duration_rows]),
        'slices': {'pressure': [r['pressure_mpa'] for r in pressure_rows],
                   'standoff': [r['standoff_m'] for r in standoff_rows],
                   'duration': [r['duration_s'] for r in duration_rows],
                   'depth_at_pressure': [r['depth_center_m'] for r in pressure_rows],
                   'depth_at_standoff': [r['depth_center_m'] for r in standoff_rows],
                   'depth_at_duration': [r['depth_center_m'] for r in duration_rows],
                   'volume_at_pressure': [r['volume_m3'] for r in pressure_rows]},
    }


def plot(fits, output):
    """画三张响应曲线：压力、靶距、时长。"""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    figures = Path(output)/'figures'
    figures.mkdir(parents=True, exist_ok=True)
    slices = fits['slices']
    panels = (('pressure', '射流压力 [MPa]', '孔深 [mm]', 'pressure', 1000.),
              ('standoff', '靶距 [m]', '孔深 [mm]', 'standoff', 1000.),
              ('duration', '喷射时长 [s]', '孔深 [mm]', 'duration', 1000.))
    for key, xlabel, ylabel, slice_key, scale in panels:
        fig, axis = plt.subplots(figsize=(4.2, 3.2))
        axis.plot(slices[slice_key], np.asarray(slices[f'depth_at_{key}'])*scale, 'o-')
        axis.set_xlabel(xlabel)
        axis.set_ylabel(ylabel)
        axis.grid(alpha=.3)
        fig.tight_layout()
        fig.savefig(figures/f'{key}.png', dpi=150)
        plt.close(fig)


def compare_runs(l0_path, levelset_path, output, task_standoff=0.5):
    """比较 L0 与 Level Set 在相同条件下的孔深与体积。

    分全体与"任务工况"（靶距 ≥ 任务标称值）两个口径报告，因为 L0 是 2.5D
    高度场，深孔（孔深/孔径大）会高估，任务工况是浅孔因而一致。
    """
    l0_rows = load_rows(l0_path)
    levelset_rows = load_rows(levelset_path)
    index = {(r['pressure_mpa'], r['standoff_m'], r['duration_s']): r
             for r in l0_rows if r.get('shots', 1) == 1}
    records = []
    for row in levelset_rows:
        key = (row['pressure_mpa'], row['standoff_m'], row['duration_s'])
        if key not in index:
            continue
        reference = index[key]
        depth_error = row['depth_center_m'] - reference['depth_center_m']
        volume_error = (row['volume_m3'] - reference['volume_m3'])/reference['volume_m3'] \
            if reference['volume_m3'] else float('nan')
        records.append({'pressure_mpa': key[0], 'standoff_m': key[1], 'duration_s': key[2],
                        'l0_depth_mm': reference['depth_center_m']*1000,
                        'levelset_depth_mm': row['depth_center_m']*1000,
                        'depth_error_mm': depth_error*1000,
                        'l0_volume_cm3': reference['volume_m3']*1e6,
                        'levelset_volume_cm3': row['volume_m3']*1e6,
                        'volume_relative_error': volume_error})

    def aggregate(rows):
        if not rows:
            return None
        depths = np.abs([r['depth_error_mm'] for r in rows])
        volumes = np.abs([r['volume_relative_error'] for r in rows])
        return {'n': len(rows),
                'depth_error_mean_mm': float(depths.mean()),
                'depth_error_max_mm': float(depths.max()),
                'volume_error_mean_pct': float(volumes.mean()*100),
                'volume_error_max_pct': float(volumes.max()*100),
                'depth_pass': bool(depths.mean() <= 5.),
                'volume_pass': bool(volumes.mean() <= .15)}

    report = {'all': aggregate(records),
              'task_regime': aggregate([r for r in records
                                        if r['standoff_m'] >= task_standoff]),
              'thresholds': {'depth_mean_mm': 5., 'volume_mean_relative': .15},
              'records': records}
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    (output/'levelset_compare.json').write_text(
        json.dumps(report, indent=2, ensure_ascii=False))
    for name in ('all', 'task_regime'):
        block = report[name]
        if block:
            print(f'{name:12s} n={block["n"]:2d}  孔深误差 均值 {block["depth_error_mean_mm"]:.1f} mm '
                  f'最大 {block["depth_error_max_mm"]:.1f} mm  |  体积误差 均值 '
                  f'{block["volume_error_mean_pct"]:.1f}% 最大 {block["volume_error_max_pct"]:.1f}%')
    return report


def load_rows(path):
    """读取 episodes.jsonl。"""
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default=str(ROOT/'configs/experiments/E2.1a.yaml'))
    parser.add_argument('--output', default=str(ROOT/'results/thesis/E2.1a/l0'))
    parser.add_argument('--engine', choices=('l0', 'levelset'), default='l0')
    parser.add_argument('--limit', type=int, default=0, help='只跑前 N 个条件（冒烟用）')
    parser.add_argument('--compare-l0', default='', help='只做比对：给定 L0 运行目录')
    parser.add_argument('--kernel-json', default='', help='覆盖冲蚀核参数（JSON），用于历史核对照')
    args = parser.parse_args()

    if args.compare_l0:
        compare_runs(Path(args.compare_l0)/'episodes.jsonl',
                     Path(args.output)/'episodes.jsonl', args.output)
        return

    config = yaml.safe_load(Path(args.config).read_text())
    kernel, kernel_source = build_kernel(config, args.kernel_json)
    output = Path(args.output)
    started = time.monotonic()

    if args.engine == 'l0':
        conditions = [(p, s, t, 1) for p in config['pressure_mpa']
                      for s in config['standoff_m'] for t in config['duration_s']]
        accumulate = config['accumulate']
        conditions += [(accumulate['pressure_mpa'], accumulate['standoff_m'],
                        accumulate['duration_s'], n) for n in range(1, accumulate['shots']+1)]
        run = lambda c: l0_condition(kernel, config['plate'], c[0], c[1], c[2], c[3])
    else:
        level = config['levelset']
        conditions = [(p, s, t, 1) for p in level['pressure_mpa']
                      for s in level['standoff_m'] for t in level['duration_s']]
        run = lambda c: levelset_condition(kernel, config, c[0], c[1], c[2])
    if args.limit:
        conditions = conditions[:args.limit]

    rows = []
    with RunWriter(output, f'E2.1a_{args.engine}',
                   {'engine': args.engine, 'kernel': kernel.params(),
                    'kernel_source': kernel_source, 'config': args.config,
                    'conditions': len(conditions)}) as writer:
        for index, (pressure, standoff, duration, shots) in enumerate(conditions, 1):
            result = run((pressure, standoff, duration, shots))
            row = {'pressure_mpa': pressure, 'standoff_m': standoff,
                   'duration_s': duration, 'shots': shots,
                   'total_duration_s': duration*shots, **result}
            writer.write_episode(row)
            rows.append(row)
            print(f'[{index}/{len(conditions)}] P={pressure} s={standoff} t={duration} '
                  f'x{shots}: 深 {result["depth_center_m"]*1000:.1f} mm  '
                  f'体积 {result["volume_m3"]*1e6:.0f} cm3', flush=True)

        extra = {'wall_time_s': time.monotonic()-started}
        if args.engine == 'l0' and not args.limit:
            fits = summarize_fits(rows, config)
            plot(fits, output)
            extra['fits'] = fits
        summary = writer.finish(METRICS, extra)
    print(json.dumps({k: v for k, v in summary.items() if not k.startswith('slices')},
                     indent=2, ensure_ascii=False)[:1500])


if __name__ == '__main__':
    main()
