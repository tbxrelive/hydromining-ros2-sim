"""统计口径：样本均值/标准差、置信区间、相邻等级可分性与配对检验。"""
import numpy as np
from scipy.stats import kendalltau, mannwhitneyu, wilcoxon


def summarize(records, metrics):
    """逐指标给出 mean/std/n；缺失或非有限值不计入。"""
    summary = {'episodes': len(records)}
    for name in metrics:
        values = [float(r[name]) for r in records
                  if r.get(name) is not None and np.isfinite(r[name])]
        summary[name+'_mean'] = float(np.mean(values)) if values else None
        summary[name+'_std'] = float(np.std(values, ddof=1)) if len(values) > 1 else 0.
        summary[name+'_n'] = len(values)
    return summary


def bootstrap_ci(values, statistic=np.mean, resamples=2000, confidence=0.95, seed=0):
    """自助法置信区间；样本不足 2 个时返回 None。

    返回 (下界, 上界)；统计量默认取均值。
    """
    data = np.asarray([float(v) for v in values if np.isfinite(v)], float)
    if len(data) < 2:
        return None
    rng = np.random.default_rng(seed)
    samples = data[rng.integers(0, len(data), size=(resamples, len(data)))]
    estimates = np.sort(np.asarray(statistic(samples, axis=1), float))
    tail = (1-confidence)/2
    return float(np.quantile(estimates, tail)), float(np.quantile(estimates, 1-tail))


def adjacent_separation(a, b, *, min_gap=0.03, min_sd=2.0, alpha=0.01):
    """判断两组样本是否"可分"（E2.3b 的等级判据）。

    四条判据同时成立才算可分：
      1. 合并标准差口径：|mean_a - mean_b| > min_sd * s_p，s_p = sqrt((s_a^2+s_b^2)/2)
      2. Mann–Whitney 检验 p < alpha
      3. 实际差异不小于 min_gap（默认 3 个百分点，防"统计显著但差异空洞"）
      4. 均值顺序与传入顺序一致（便于断言单调性）

    Returns:
        字典：两组均值/标准差、合并标准差、差值、以标准差计的间隔、
        检验 p 值、以及四项判据的布尔结果。
    """
    x = np.asarray([v for v in a if np.isfinite(v)], float)
    y = np.asarray([v for v in b if np.isfinite(v)], float)
    if len(x) < 2 or len(y) < 2:
        return {'separable': False, 'reason': 'samples too few',
                'n_a': int(len(x)), 'n_b': int(len(y))}
    mean_x, mean_y = float(x.mean()), float(y.mean())
    std_x = float(x.std(ddof=1)) if len(x) > 1 else 0.
    std_y = float(y.std(ddof=1)) if len(y) > 1 else 0.
    pooled = float(np.sqrt((std_x**2 + std_y**2)/2))
    diff = mean_y - mean_x
    p_value = float(mannwhitneyu(x, y, alternative='two-sided').pvalue)
    gap_ok = abs(diff) >= min_sd*pooled
    test_ok = p_value < alpha
    practical_ok = abs(diff) >= min_gap
    order_ok = diff > 0
    return {'n_a': int(len(x)), 'n_b': int(len(y)),
            'mean_a': mean_x, 'mean_b': mean_y,
            'std_a': std_x, 'std_b': std_y, 'pooled_std': pooled,
            'diff': diff, 'gap_in_sd': float(abs(diff)/pooled) if pooled > 0 else float('inf'),
            'p_value': p_value,
            'gap_ok': bool(gap_ok), 'test_ok': bool(test_ok),
            'practical_ok': bool(practical_ok), 'order_ok': bool(order_ok),
            'separable': bool(gap_ok and test_ok and practical_ok and order_ok)}


def kendall_tau(a, b):
    """Kendall τ 秩相关，用于判断两组参数下的敏感度排序是否一致。

    任一组为常数（例如输出饱和、全部 μ* 为 0）时排序无意义，返回 None，
    避免把 nan 写进 summary。
    """
    x, y = np.asarray(a, float), np.asarray(b, float)
    if x.shape != y.shape or len(x) < 3:
        return None
    if np.allclose(x, x[0]) or np.allclose(y, y[0]):
        return None
    result = kendalltau(x, y)
    if not np.isfinite(result.statistic):
        return None
    return {'tau': float(result.statistic), 'p_value': float(result.pvalue), 'n': int(len(x))}


def top_k_agreement(values_a, values_b, names, k=4):
    """两档扰动下"前 k 名集合"是否一致（E2.1b 的补充口径）。

    只作说明用，不参与通过/不通过的判定：尾部参数的 μ* 常常只有主控参数的
    千分之一，名次纯噪声，逐位比较完整排序会被这些噪声拖低。

    Returns:
        字典：两档各自的前 k 名、重合个数、是否完全一致、重合率。
    """
    a = dict(zip(names, np.asarray(values_a, float)))
    b = dict(zip(names, np.asarray(values_b, float)))
    if len(a) != len(b) or len(a) < k:
        return None
    top_a = set(sorted(a, key=lambda n: -a[n])[:k])
    top_b = set(sorted(b, key=lambda n: -b[n])[:k])
    shared = top_a & top_b
    return {'k': int(k), 'top_a': sorted(top_a), 'top_b': sorted(top_b),
            'shared': sorted(shared), 'shared_count': len(shared),
            'fraction': len(shared)/k, 'identical': len(shared) == k}


def bootstrap_kendall(index_pair, n, resamples=300, confidence=0.95, seed=0):
    """Kendall τ 的自助法置信区间。

    Args:
        index_pair: 可调用对象，`index_pair(pick)` 返回 (向量 a, 向量 b)。
            `pick` 是长度为 n 的重采样下标数组；传 None 表示用全样本（点估计）。
        n: 设计点个数（Sobol 的基样本数或 Morris 的轨迹数）。
        resamples: 重采样次数。

    Returns:
        字典：点估计 τ 与 95% 置信区间；点估计为 None 时返回 None。
    """
    if not callable(index_pair) or n < 3:
        return None
    point = kendall_tau(*index_pair(None))
    if point is None:
        return None
    rng = np.random.default_rng(seed)
    draws = []
    for _ in range(resamples):
        pick = rng.integers(0, n, n)
        result = kendall_tau(*index_pair(pick))
        if result is not None:
            draws.append(result['tau'])
    if len(draws) < 10:
        return {'tau': point['tau'], 'ci_low': None, 'ci_high': None,
                'resamples': len(draws)}
    tail = (1-confidence)/2
    return {'tau': point['tau'],
            'ci_low': float(np.quantile(draws, tail)),
            'ci_high': float(np.quantile(draws, 1-tail)),
            'resamples': len(draws)}


def paired_wilcoxon(baseline, proposed):
    """配对样本 Wilcoxon 符号秩检验；样本不足时返回 None。"""
    a, b = np.asarray(baseline, float), np.asarray(proposed, float)
    if a.shape != b.shape or len(a) < 6 or np.allclose(a, b):
        return None
    result = wilcoxon(a, b)
    return {'statistic': float(result.statistic), 'p_value': float(result.pvalue),
            'n': int(len(a))}
