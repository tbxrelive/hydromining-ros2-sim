"""基于面积区间界的三分类；未知或置信区间跨阈值不能按已达标处理。"""
import numpy as np


def circle_fractions(centers, resolution, radius, subsamples=12):
    """对子格积分圆形任务区，避免边界格整格纳入带来的面积跳变。"""
    if radius <= 0 or resolution <= 0 or not 1 <= subsamples <= 64:
        raise ValueError('invalid task region quadrature')
    uv = np.asarray(centers, float)[..., :2]
    count = np.zeros(uv.shape[:-1])
    offsets = ((np.arange(subsamples) + 0.5) / subsamples - 0.5) * resolution
    for u in offsets:
        for v in offsets:
            count += np.sum((uv + [u, v]) ** 2, axis=-1) <= radius ** 2
    return count / subsamples ** 2


def assess(change, area_weights, required_depth=0.02, required_fraction=0.9,
           confidence_k=1.645):
    """用保守/乐观面积比例给出 achieved/not_achieved/insufficient_evidence。"""
    if (not np.isfinite([required_depth, required_fraction, confidence_k]).all() or
            required_depth <= 0 or not 0 < required_fraction <= 1 or confidence_k <= 0):
        raise ValueError('invalid assessment requirement')
    weights = np.asarray(area_weights, float)
    if weights.shape != change['delta'].shape or not np.isfinite(weights).all() or np.any(weights < 0):
        raise ValueError('invalid area weights')
    total = float(weights.sum())
    if total <= 0:
        raise ValueError('task region area is empty')
    known = (change['valid'] & np.isfinite(change['delta']) &
             np.isfinite(change['sigma']) & (change['sigma'] >= 0))
    low = known & (change['delta'] - confidence_k * change['sigma'] >= required_depth)
    high = (~known) | (change['delta'] + confidence_k * change['sigma'] >= required_depth)
    lower = float(weights[low].sum() / total)
    upper = float(weights[high].sum() / total)
    if lower >= required_fraction:
        status = 'achieved'
    elif upper < required_fraction:
        status = 'not_achieved'
    else:
        status = 'insufficient_evidence'
    uncertain = (weights > 0) & (~known | (high & ~low))
    return {
        'status': status, 'fraction_lower': lower, 'fraction_upper': upper,
        'coverage': float(weights[known].sum() / total), 'task_area_m2': total,
        'required_depth_m': required_depth, 'required_fraction': required_fraction,
        'confidence_k': confidence_k, 'missing_evidence_cells': np.argwhere(uncertain).tolist(),
        'uncertainty_calibrated': False,
    }
