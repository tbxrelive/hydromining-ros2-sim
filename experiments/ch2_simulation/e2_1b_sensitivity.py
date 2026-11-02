"""E2.1b 参数敏感性分析：核参数与任务量的敏感度排序、结论稳定性。

两层：
- 层 1（便宜）：平整靶单次喷射，Sobol 一阶/总效应指数，输出孔深、孔径、体积。
- 层 2（较贵）：任务级规则基线，Morris 基本效应，输出真值达标率、喷射次数、
  总时长、耗水。

两档扰动幅度（±20%、±50%），比较排序是否稳定（Kendall τ ≥ 0.7，带自助法 CI）。

**层 2 必须连压力与靶距一起扰动**：第一版只扰动核参数，而任务压力恒为 30 MPa，
此时 R_c = k·P^α/s^β 里 k 与 α 只有乘积可辨识，二者名次必然互换（τ 0.47–0.60）。
把任务量纳入扰动集后才能分开；同时按文献把恒为 0 的 b_pressure 从参数集里去掉。

**输出集为什么在 v1c 里换过**：成功率是 4 回合的量化量（只取 5 个值）且规则近饱和，
去除体积由"达标即停"的任务定义决定、几乎恒定——两者都测不出参数影响。
判据输出改为连续的 `truth_fraction`（达标率）、`sprays`、`duration_s`、`water_m3`；
成功率仍计算并报告，但不参与排序判定。

命令：
    python experiments/ch2_simulation/e2_1b_sensitivity.py --layer 1
    python experiments/ch2_simulation/e2_1b_sensitivity.py --layer 2
"""
import argparse
import json
import multiprocessing as mp
from pathlib import Path
import sys
import time

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT/'src/hydromining_sim/scripts'))

from experiments.common.crater import flat_plate, measure  # noqa: E402
from experiments.common.stats import (  # noqa: E402
    bootstrap_kendall, kendall_tau, top_k_agreement)
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
# 判据输出：都是连续量
OUTPUTS2 = ('truth_fraction', 'sprays', 'duration_s', 'water_m3')
# 只报告、不参与判定的输出
REPORT_ONLY2 = ('success', 'removed_volume_m3')
ALL_OUTPUTS2 = OUTPUTS2 + REPORT_ONLY2
TAU_THRESHOLD = .7


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


def run_single(plate, kernel, pressure, standoff, duration):
    """平板单次喷射，返回三个响应。

    `plate` 是预构造的 (heights, grid, spacing, extent)，避免每次重算网格。
    """
    from algorithms import heightfield_erosion
    base, grid, spacing, extent = plate
    heights = base.copy()
    heightfield_erosion.spray_heightfield(heights, grid, 0., 0., duration, pressure,
                                          standoff, kernel, spacing, extent)
    result = measure(base, heights, spacing, center=(0., 0.))
    return [result[name] for name in OUTPUTS1]


def run_task(kernel, episodes, cache, pressure=None, standoff=None):
    """任务级规则基线，返回全部输出（判据输出 + 只报告输出）。"""
    from experiments.common.large_env import make_fast_env
    env = make_fast_env(cache, 10000, 'test', kernel=kernel,
                        pressure_override=pressure, standoff_override=standoff)
    successes, sprays, durations, volumes, fractions, water = 0, 0, 0., 0., 0., 0.
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
            water += env.total_water_m3_
            fractions += float(info['truth_fraction'])
            volumes += float(np.sum(np.maximum(env._map_heights(env.initial_-env.heights_), 0)*env.area_))
    finally:
        env.close()
    values = {'success': successes/episodes, 'sprays': sprays/episodes,
              'duration_s': durations/episodes, 'water_m3': water/episodes,
              'truth_fraction': fractions/episodes, 'removed_volume_m3': volumes/episodes}
    return [values[name] for name in ALL_OUTPUTS2]


def _task_job(job):
    """多进程工作单元（必须是模块级函数才能被 pickle）。"""
    physical, episodes, cache = job
    kernel = kernel_from(physical, LAYER2_PARAMS)
    return run_task(kernel, episodes, cache,
                    pressure=physical['pressure_mpa'], standoff=physical['standoff_m'])


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
    # 原始响应数组留给主流程做 τ 的自助法；写盘前会删掉。
    report['_raw_results'] = results
    return report


def sobol_total_vector(raw, names, samples, column, pick=None):
    """总效应向量；`pick` 给了就按该下标重采样 Saltelli 的三个块。

    供 Kendall τ 的自助法使用：对同一批基样本重采样，两档扰动共用同一个 `pick`，
    这样 CI 反映的是"估计误差"，不是两档之间的系统差异。
    """
    y = raw[:, column]
    index = np.arange(samples) if pick is None else np.asarray(pick)
    fa, fb = y[:samples][index], y[samples:2*samples][index]
    variance = float(np.var(np.r_[fa, fb]))
    if variance <= 0:
        return np.zeros(len(names))
    total = np.empty(len(names))
    for i in range(len(names)):
        fab = y[2*samples+i*samples: 2*samples+(i+1)*samples][index]
        total[i] = .5*float(np.mean((fa-fab)**2))/variance
    return total


def to_physical(row, names, lo, hi):
    """归一化设计点 -> 物理参数（对数均匀，保证都是正数）。"""
    return {name: float(np.exp(np.log(lo[name]) + value*(np.log(hi[name])-np.log(lo[name]))))
            for name, value in zip(names, row)}


def morris_for(evaluate_many, names, lo, hi, outputs, trajectories=30, levels=4, seed=0):
    """Morris 基本效应筛选：返回 μ*、μ、σ 与排序。

    每条轨迹对每个参数各做一次单参数扰动，必须逐条记录"扰动的是哪个参数"，
    不能用下标取模推断（轨迹内参数顺序是随机的）。

    `evaluate_many(list[dict]) -> ndarray`：一次吃一批物理参数点，便于并行。
    """
    rng = np.random.default_rng(seed)
    dimension = len(names)
    delta = levels/(2*(levels-1))
    plans, orders = [], []
    for _ in range(trajectories):
        point = rng.uniform(0., 1.-delta, dimension)
        order = rng.permutation(dimension)
        chain = [point.copy()]
        for index in order:
            point[index] += delta
            chain.append(point.copy())
        plans.append(chain)
        orders.append(order)

    flat = [to_physical(point, names, lo, hi) for chain in plans for point in chain]
    responses = np.asarray(evaluate_many(flat), float)
    if responses.shape[0] != len(flat):
        raise ValueError('evaluate_many 返回的行数与输入不一致')

    effects = np.empty((trajectories, dimension, responses.shape[1]))
    cursor = 0
    for t, chain in enumerate(plans):
        block = responses[cursor:cursor+len(chain)]
        cursor += len(chain)
        for step, parameter in enumerate(orders[t]):
            effects[t, parameter] = (block[step+1]-block[step])/delta

    report = {'trajectories': trajectories, 'levels': levels,
              'evaluations': len(flat), 'outputs': {}}
    for column, output in enumerate(outputs):
        stats = {}
        for index, name in enumerate(names):
            values = effects[:, index, column]
            stats[name] = {'mu_star': float(np.mean(np.abs(values))),
                           'mu': float(np.mean(values)),
                           'sigma': float(np.std(values, ddof=1)) if len(values) > 1 else 0.,
                           'n': int(len(values))}
        report['outputs'][output] = {
            'stats': stats,
            'ranking': sorted(names, key=lambda n: stats[n]['mu_star'], reverse=True)}
    report['_effects'] = effects
    return report


def morris_mu_star_vector(effects, names, column, pick=None):
    """μ* 向量；`pick` 给了就按轨迹重采样。供 τ 的自助法使用。"""
    selected = effects if pick is None else effects[np.asarray(pick)]
    return np.mean(np.abs(selected[:, :, column]), axis=0)


def make_batch_evaluator(episodes, cache, processes):
    """返回 evaluate_many(points)；processes>1 时用多进程并行。

    每个进程自建环境，但每个评估点的随机种子固定，所以并行与串行结果逐位相同。
    """
    cache = str(cache)

    def evaluate_many(points):
        jobs = [(point, episodes, cache) for point in points]
        if processes <= 1:
            return np.asarray([_task_job(job) for job in jobs], float)
        with mp.get_context('fork').Pool(processes) as pool:
            return np.asarray(pool.map(_task_job, jobs, chunksize=1), float)
    return evaluate_many


def stability_block(layer, run_a, run_b, names, outputs, criterion_outputs, vector_pair, units):
    """算两档扰动的 τ、τ 的自助法 CI，以及前 4 名集合的补充口径。"""
    result = {}
    for column, output in enumerate(outputs):
        pair = vector_pair(column)
        entry = {
            'kendall': bootstrap_kendall(pair, units, resamples=300),
            'top_k': top_k_agreement(pair(None)[0], pair(None)[1], names, k=4),
            'in_criterion': output in criterion_outputs,
        }
        tau = entry['kendall']['tau'] if entry['kendall'] else None
        entry['meets_threshold'] = (tau is not None and tau >= TAU_THRESHOLD)
        result[output] = entry
    judged = [(name, block['meets_threshold']) for name, block in result.items()
              if block['in_criterion']]
    return {'per_output': result,
            'criterion_outputs': list(criterion_outputs),
            'passed_all': bool(judged) and all(flag for _, flag in judged),
            'passed_count': sum(flag for _, flag in judged),
            'judged_count': len(judged)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--layer', choices=('1', '2', 'both'), default='both')
    parser.add_argument('--output', default=str(ROOT/'results/thesis/E2.1b/final'))
    parser.add_argument('--episodes', type=int, default=12, help='层 2 每个参数点的回合数')
    parser.add_argument('--samples', type=int, default=1024, help='层 1 Sobol 取样数')
    parser.add_argument('--trajectories', type=int, default=30, help='层 2 Morris 轨迹数')
    parser.add_argument('--processes', type=int, default=min(8, mp.cpu_count()),
                        help='层 2 并行进程数；1 表示串行')
    args = parser.parse_args()

    output = Path(args.output)
    config = yaml.safe_load((ROOT/'configs/experiments/E2.1a.yaml').read_text())
    plate = config['plate']
    duration = config['nominal']['duration_s']
    base = base_values()
    report = {'base': base, 'settings': {'episodes': args.episodes, 'samples': args.samples,
                                         'trajectories': args.trajectories,
                                         'processes': args.processes,
                                         'tau_threshold': TAU_THRESHOLD},
              'layers': {}}

    # 层 1 的平板靶预先构造，避免每次重算网格
    base_heights, grid, _ = flat_plate(plate['extent_m'], plate['spacing_m'])
    plate_bundle = (base_heights, grid, plate['spacing_m'], plate['extent_m'])

    for scale, tag in ((.2, 'pct20'), (.5, 'pct50')):
        ranges = ranges_for(base, scale)
        ranges.update(task_ranges(scale))
        low = {name: ranges[name][0] for name in LAYER1_PARAMS}
        high = {name: ranges[name][1] for name in LAYER1_PARAMS}
        if args.layer in ('1', 'both'):
            def evaluate1(row):
                kernel = kernel_from(row, LAYER1_PARAMS)
                return run_single(plate_bundle, kernel, row['pressure_mpa'],
                                  row['standoff_m'], duration)
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
            evaluate_many = make_batch_evaluator(args.episodes, cache, args.processes)
            started = time.monotonic()
            result = morris_for(evaluate_many, LAYER2_PARAMS, low, high, ALL_OUTPUTS2,
                                trajectories=args.trajectories, seed=0)
            result['wall_time_s'] = time.monotonic()-started
            report['layers'].setdefault('layer2', {})[tag] = result
            print(f'[层2 {tag}] 完成，{result["evaluations"]} 次任务级评估，'
                  f'{result["wall_time_s"]:.1f} s', flush=True)

    # 排序稳定性：完整 τ（主判据）+ 前 4 名集合（补充口径）
    if 'layer1' in report['layers']:
        a, b = report['layers']['layer1'].get('pct20'), report['layers']['layer1'].get('pct50')
        if a and b:
            samples = a['samples']

            def sobol_pair(column, ra=a['_raw_results'], rb=b['_raw_results']):
                return lambda pick: (sobol_total_vector(ra, LAYER1_PARAMS, samples, column, pick),
                                     sobol_total_vector(rb, LAYER1_PARAMS, samples, column, pick))
            report['layers']['layer1']['stability'] = stability_block(
                'layer1', a, b, LAYER1_PARAMS, OUTPUTS1, OUTPUTS1, sobol_pair, samples)
    if 'layer2' in report['layers']:
        a, b = report['layers']['layer2'].get('pct20'), report['layers']['layer2'].get('pct50')
        if a and b:
            trajectories = a['trajectories']

            def morris_pair(column, ea=a['_effects'], eb=b['_effects']):
                return lambda pick: (morris_mu_star_vector(ea, LAYER2_PARAMS, column, pick),
                                     morris_mu_star_vector(eb, LAYER2_PARAMS, column, pick))
            report['layers']['layer2']['stability'] = stability_block(
                'layer2', a, b, LAYER2_PARAMS, ALL_OUTPUTS2, OUTPUTS2, morris_pair, trajectories)

    passed = [layer for layer in report['layers']
              if report['layers'][layer].get('stability', {}).get('passed_all')]
    report['passed_all_layers'] = len(passed) == len(report['layers']) and bool(report['layers'])

    # 原始数组只用于自助法，不写盘
    for layer in report['layers'].values():
        for tag in ('pct20', 'pct50'):
            if tag in layer:
                layer[tag].pop('_raw_results', None)
                layer[tag].pop('_effects', None)

    output.mkdir(parents=True, exist_ok=True)
    (output/'sensitivity.json').write_text(json.dumps(report, indent=2, ensure_ascii=False))
    for layer_name, layer in report['layers'].items():
        block = layer.get('stability')
        if block:
            print(f'{layer_name}: 通过 {block["passed_count"]}/{block["judged_count"]} 项')
            for name, entry in block['per_output'].items():
                tau = entry['kendall']['tau'] if entry['kendall'] else None
                ci = (f"[{entry['kendall']['ci_low']:.2f},{entry['kendall']['ci_high']:.2f}]"
                      if entry['kendall'] and entry['kendall']['ci_low'] is not None else '')
                shown = 'None' if tau is None else f'{tau:.3f}'
                mark = '判据' if entry['in_criterion'] else '报告'
                print(f'  {name:20s} τ={shown:>6} {ci:>14} '
                      f'前4重合 {entry["top_k"]["shared_count"]}/4  ({mark})')
    print(f'结果写入 {output/"sensitivity.json"}')


if __name__ == '__main__':
    main()
