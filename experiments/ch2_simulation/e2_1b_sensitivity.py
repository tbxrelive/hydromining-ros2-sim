"""E2.1b 参数敏感性分析：核参数与任务量的敏感度排序、结论稳定性。

两层：
- 层 1（便宜）：平整靶单次喷射，Sobol 一阶/总效应指数，输出孔深、孔径、体积。
- 层 2（较贵）：任务级规则基线，Morris 基本效应，输出成功率、喷射次数、
  总时长、总去除体积。

两档扰动幅度（±20%、±50%），比较排序是否稳定（Kendall τ ≥ 0.7）。

**层 2 必须连压力与靶距一起扰动**：第一版只扰动核参数，而任务压力恒为 30 MPa，
此时 R_c = k·P^α/s^β 里 k 与 α 只有乘积可辨识，二者名次必然互换（τ 0.47–0.60）。
把任务量纳入扰动集后才能分开；同时按文献把恒为 0 的 b_pressure 从参数集里去掉。

命令：
    python experiments/ch2_simulation/e2_1b_sensitivity.py --layer 1
    python experiments/ch2_simulation/e2_1b_sensitivity.py --layer 2
"""
import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT/'src/hydromining_sim/scripts'))

from experiments.common.crater import flat_plate, measure  # noqa: E402
from experiments.common.records import RunWriter  # noqa: E402
from experiments.common.stats import bootstrap_ci, kendall_tau  # noqa: E402
from experiments.common.task import load_task  # noqa: E402

# 层 1 敏感的参数：核参数 + 任务量（压力、靶距）
LAYER1_PARAMS = ('k', 'alpha', 'beta', 'pressure_threshold', 'b_base', 'b_distance',
                 'pressure_mpa', 'standoff_m')
# 层 2 必须包含任务量：R_c = k·(P−P_th)^α/s^β 在 P 固定时 k 与 α 数学上不可分
# （只有乘积可辨识），这是第一版 Morris 排序 τ 偏低的根因。把压力、靶距、
# 时长一并扰动后，k 与 α 才能靠各自的特征方向分开。
LAYER2_PARAMS = ('k', 'alpha', 'beta', 'pressure_threshold', 'b_base', 'b_distance',
                 'pressure_mpa', 'standoff_m')
OUTPUTS1 = ('depth_center_m', 'diameter_half_m', 'volume_m3')
OUTPUTS2 = ('success', 'sprays', 'duration_s', 'removed_volume_m3')


def ranges_for(base, scale):
    """把基准值按相对幅度展开成 [lo, hi] 区间。"""
    ranges = {}
    for name in LAYER1_PARAMS:
        value = base[name]
        low, high = value*(1-scale), value*(1+scale)
        # 物理下限：这些量必须为正
        ranges[name] = (max(low, 1e-9), high)
    return ranges


def task_ranges(scale):
    """任务量的扰动区间（压力与靶距围绕标称值）。"""
    task = load_task()
    return {'pressure_mpa': (task.pressure_mpa*(1-scale), task.pressure_mpa*(1+scale)),
            'standoff_m': (task.standoff_m*(1-scale), task.standoff_m*(1+scale))}


def base_values():
    """核参数的标定值 + 任务标称量。"""
    task = load_task()
    values = {name: value for name, value in task.erosion_kernel().params().items()}
    values['pressure_mpa'] = task.pressure_mpa
    values['standoff_m'] = task.standoff_m
    return values


def kernel_from(row, names):
    """从归一化取样行还原核参数（只取 names 里的键）。"""
    from algorithms.erosion_model import DefaultErosionKernel
    return DefaultErosionKernel(**{name: row[name] for name in names
                                   if name in DefaultErosionKernel.DEFAULTS})


def run_single(kernel, pressure, standoff, duration, plate):
    """平板单次喷射，返回三个响应。"""
    from algorithms import heightfield_erosion
    extent, spacing = plate['extent_m'], plate['spacing_m']
    heights, grid, _ = flat_plate(extent, spacing)
    before = heights.copy()
    heightfield_erosion.spray_heightfield(heights, grid, 0., 0., duration, pressure,
                                          standoff, kernel, spacing, extent)
    result = measure(before, heights, spacing, center=(0., 0.))
    return [result[name] for name in OUTPUTS1]


def run_task(kernel, episodes, cache, pressure=None, standoff=None):
    """任务级规则基线：返回成功率、喷射次数、总时长、总去除体积。

    压力/靶距省略时用任务配置值；层 2 的 Morris 需要扰动它们以分开 k 与 α。
    """
    from experiments.common.large_env import make_fast_env
    env = make_fast_env(cache, 10000, 'test', kernel=kernel,
                        pressure_override=pressure, standoff_override=standoff)
    successes, sprays, durations, volumes = 0, 0, 0., 0.
    try:
        for index in range(episodes):
            scenario = env.scenario_names_[index % len(env.scenario_names_)]
            env.reset(seed=10000+index, options={'scenario': scenario, 'mesh_index': index % 3})
            while True:
                _, _, terminated, truncated, info = env.step(env.rule_action())
                if terminated or truncated:
                    break
            successes += int(info['verdict'] == 'achieved')
            sprays += env.sprays_used_
            durations += env.total_spray_duration_
            volumes += float(np.sum(np.maximum(env._map_heights(env.initial_-env.heights_), 0)*env.area_))
    finally:
        env.close()
    return [successes/episodes, sprays/episodes, durations/episodes, volumes/episodes]


def sobol_indices_for(evaluate, names, lo, hi, outputs, samples=128, seed=0):
    """Saltelli/Jansen 估计：一次取样同时给出全部输出的一阶与总效应指数。"""
    from scipy.stats import qmc
    dimension = len(names)
    rng = np.random.default_rng(seed)
    qrng = qmc.Sobol(d=2*dimension, scramble=True, seed=rng)
    block = qrng.random(samples)
    matrix_a, matrix_b = block[:, :dimension], block[:, dimension:]
    ab = [np.concatenate([matrix_a[:, :i], matrix_b[:, i:i+1], matrix_a[:, i+1:]], axis=1)
          for i in range(dimension)]
    design = np.vstack([matrix_a, matrix_b, *ab])

    def to_physical(row):
        return {name: float(np.exp(np.log(lo[name]) + value*(np.log(hi[name])-np.log(lo[name]))))
                for name, value in zip(names, row)}

    started = time.monotonic()
    results = np.array([evaluate(to_physical(row)) for row in design], float)
    elapsed = time.monotonic()-started
    total_samples = len(design)

    def indices_for(column):
        y = results[:, column]
        fa, fb = y[:samples], y[samples:2*samples]
        variance = float(np.var(np.r_[fa, fb]))
        first, total = {}, {}
        for i, name in enumerate(names):
            fab = y[2*samples+i*samples: 2*samples+(i+1)*samples]
            first[name] = float(np.mean(fb*(fab-fa))/variance) if variance > 0 else 0.
            total[name] = float(.5*np.mean((fa-fab)**2)/variance) if variance > 0 else 0.
        return first, total

    def bootstrap(column):
        """对 N 个样本重采样，给出指数的 95% 置信区间。"""
        y = results[:, column]
        fa, fb = y[:samples], y[samples:2*samples]
        fab_all = [y[2*samples+i*samples: 2*samples+(i+1)*samples] for i in range(dimension)]
        picker = np.random.default_rng(seed+1)
        first_runs, total_runs = [], []
        for _ in range(300):
            pick = picker.integers(0, samples, samples)
            a, b = fa[pick], fb[pick]
            variance = float(np.var(np.r_[a, b]))
            if variance <= 0:
                continue
            first_runs.append([float(np.mean(b*(fab_all[i][pick]-a))/variance)
                               for i in range(dimension)])
            total_runs.append([float(.5*np.mean((a-fab_all[i][pick])**2)/variance)
                               for i in range(dimension)])
        low = np.percentile(first_runs, 2.5, axis=0) if first_runs else np.zeros(dimension)
        high = np.percentile(first_runs, 97.5, axis=0) if first_runs else np.zeros(dimension)
        low_t = np.percentile(total_runs, 2.5, axis=0) if total_runs else np.zeros(dimension)
        high_t = np.percentile(total_runs, 97.5, axis=0) if total_runs else np.zeros(dimension)
        return {name: [float(low[i]), float(high[i])] for i, name in enumerate(names)}, \
               {name: [float(low_t[i]), float(high_t[i])] for i, name in enumerate(names)}

    report = {'samples': samples, 'evaluations': total_samples,
              'wall_time_s': elapsed, 'outputs': {}}
    for column, output in enumerate(outputs):
        first, total = indices_for(column)
        ci_first, ci_total = bootstrap(column)
        report['outputs'][output] = {'first_order': first, 'total': total,
                                     'first_order_ci': ci_first, 'total_ci': ci_total}
    return report


def morris_for(evaluate, names, lo, hi, outputs, trajectories=6, levels=4, seed=0):
    """Morris 基本效应筛选：返回 μ*、μ、σ 与排序。

    每条轨迹对每个参数各做一次单参数扰动，因此必须逐条记录"扰动的是哪个参数"，
    不能用下标取模推断（轨迹内参数顺序是随机的）。
    """
    rng = np.random.default_rng(seed)
    dimension = len(names)
    delta = levels/(2*(levels-1))

    def to_physical(row):
        return {name: float(np.exp(np.log(lo[name]) + value*(np.log(hi[name])-np.log(lo[name]))))
                for name, value in zip(names, row)}

    records = []  # (参数下标, 该步的基本效应向量)
    for _ in range(trajectories):
        point = rng.uniform(0., 1.-delta, dimension)
        order = rng.permutation(dimension)
        base_y = np.asarray(evaluate(to_physical(point)), float)
        for index in order:
            stepped = point.copy()
            stepped[index] += delta
            new_y = np.asarray(evaluate(to_physical(stepped)), float)
            records.append((int(index), (new_y-base_y)/delta))
            point, base_y = stepped, new_y

    report = {'trajectories': trajectories, 'levels': levels,
              'evaluations': len(records)+trajectories, 'outputs': {}}
    for column, output in enumerate(outputs):
        stats = {}
        for index, name in enumerate(names):
            values = np.array([effect[column] for step, effect in records if step == index])
            stats[name] = {'mu_star': float(np.mean(np.abs(values))),
                           'mu': float(np.mean(values)),
                           'sigma': float(np.std(values, ddof=1)) if len(values) > 1 else 0.,
                           'n': int(len(values))}
        report['outputs'][output] = {
            'stats': stats,
            'ranking': sorted(names, key=lambda n: stats[n]['mu_star'], reverse=True)}
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--layer', choices=('1', '2', 'both'), default='both')
    parser.add_argument('--output', default=str(ROOT/'results/thesis/E2.1b'))
    parser.add_argument('--episodes', type=int, default=3, help='层 2 每个参数点的回合数')
    parser.add_argument('--samples', type=int, default=128, help='层 1 Sobol 取样数')
    parser.add_argument('--trajectories', type=int, default=6, help='层 2 Morris 轨迹数')
    args = parser.parse_args()

    output = Path(args.output)
    config = yaml.safe_load((ROOT/'configs/experiments/E2.1a.yaml').read_text())
    plate = config['plate']
    duration = config['nominal']['duration_s']
    base = base_values()
    report = {'base': base, 'layers': {}}

    for scale, tag in ((.2, 'pct20'), (.5, 'pct50')):
        ranges = ranges_for(base, scale)
        ranges.update(task_ranges(scale))
        if args.layer in ('1', 'both'):
            def evaluate1(row):
                kernel = kernel_from(row, LAYER1_PARAMS)
                return run_single(kernel, row['pressure_mpa'], row['standoff_m'], duration, plate)
            low = {name: ranges[name][0] for name in LAYER1_PARAMS}
            high = {name: ranges[name][1] for name in LAYER1_PARAMS}
            result = sobol_indices_for(evaluate1, LAYER1_PARAMS, low, high, OUTPUTS1,
                                       samples=args.samples, seed=0)
            report['layers'].setdefault('layer1', {})[tag] = result
            print(f'[层1 {tag}] 完成，{result["evaluations"]} 次单次喷射，'
                  f'{result["wall_time_s"]:.1f} s', flush=True)
        if args.layer in ('2', 'both'):
            cache = output/'assets'
            cache.mkdir(parents=True, exist_ok=True)
            from experiments.common.large_env import prepare_meshes
            prepare_meshes(cache)
            def evaluate2(row):
                return run_task(kernel_from(row, LAYER2_PARAMS), args.episodes, cache,
                                pressure=row['pressure_mpa'], standoff=row['standoff_m'])
            low = {name: ranges[name][0] for name in LAYER2_PARAMS}
            high = {name: ranges[name][1] for name in LAYER2_PARAMS}
            result = morris_for(evaluate2, LAYER2_PARAMS, low, high, OUTPUTS2,
                                trajectories=args.trajectories, seed=0)
            report['layers'].setdefault('layer2', {})[tag] = result
            print(f'[层2 {tag}] 完成', flush=True)

    # 排序稳定性：两种幅度下的排序 Kendall τ
    for layer, key in (('layer1', 'total'), ('layer2', 'mu_star')):
        if layer not in report['layers']:
            continue
        a, b = report['layers'][layer].get('pct20'), report['layers'][layer].get('pct50')
        if not a or not b:
            continue
        stability = {}
        for output_name in a['outputs']:
            if layer == 'layer1':
                va = [a['outputs'][output_name]['total'][n] for n in LAYER1_PARAMS]
                vb = [b['outputs'][output_name]['total'][n] for n in LAYER1_PARAMS]
            else:
                va = [a['outputs'][output_name]['stats'][n]['mu_star'] for n in LAYER2_PARAMS]
                vb = [b['outputs'][output_name]['stats'][n]['mu_star'] for n in LAYER2_PARAMS]
            stability[output_name] = kendall_tau(va, vb)
        report['layers'][layer]['ranking_stability'] = stability

    output.mkdir(parents=True, exist_ok=True)
    (output/'sensitivity.json').write_text(json.dumps(report, indent=2, ensure_ascii=False))
    print(f'结果写入 {output/"sensitivity.json"}')


if __name__ == '__main__':
    main()
