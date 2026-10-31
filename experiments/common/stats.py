"""统计口径：样本均值/标准差与配对检验；summary 可离线复算。"""
import numpy as np
from scipy.stats import wilcoxon


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


def paired_wilcoxon(baseline, proposed):
    """配对样本 Wilcoxon 符号秩检验；样本不足时返回 None。"""
    a, b = np.asarray(baseline, float), np.asarray(proposed, float)
    if a.shape != b.shape or len(a) < 6 or np.allclose(a, b):
        return None
    result = wilcoxon(a, b)
    return {'statistic': float(result.statistic), 'p_value': float(result.pvalue),
            'n': int(len(a))}
