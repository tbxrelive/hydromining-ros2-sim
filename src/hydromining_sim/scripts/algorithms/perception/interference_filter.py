"""静止视点多帧鲁棒过滤；不使用干扰标签或网格真值。"""
import warnings

import numpy as np

KEEP, INVALID, TEMPORAL_OUTLIER, FOREGROUND, LOW_SUPPORT = range(5)


def temporal_filter(ranges, sigma, minimum_support=2, k=3.5, noise_floor=0.003):
    """逐射线中位数/MAD 剔除瞬态异常；缺失数据不能被时序插值补成观测。"""
    values = np.asarray(ranges, float)
    sigmas = np.broadcast_to(np.asarray(sigma, float), values.shape)
    if values.ndim < 2 or not 1 <= minimum_support <= values.shape[0]:
        raise ValueError('invalid temporal dimensions or support')
    finite_sigmas = sigmas[np.isfinite(values) & (values > 0)]
    if (not np.isfinite(k) or not np.isfinite(noise_floor) or
            k <= 0 or noise_floor <= 0 or not np.isfinite(finite_sigmas).all() or
            np.any(finite_sigmas < 0)):
        raise ValueError('invalid temporal filter uncertainty parameters')
    valid = np.isfinite(values) & (values > 0)
    clean = np.where(valid, values, np.nan)
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', category=RuntimeWarning)
        median = np.nanmedian(clean, axis=0)
        mad = 1.4826 * np.nanmedian(np.abs(clean - median), axis=0)
    tolerance = k * np.maximum(mad[None, ...], np.maximum(sigmas, noise_floor))
    outlier = valid & (np.abs(values - median) > tolerance)
    kept = valid & ~outlier
    support = kept.sum(axis=0)
    labels = np.full(values.shape, KEEP, dtype=np.int8)
    labels[~valid] = INVALID
    labels[outlier] = TEMPORAL_OUTLIER
    insufficient = kept & (support[None, ...] < minimum_support)
    labels[insufficient] = LOW_SUPPORT
    accepted = kept & ~insufficient
    return np.where(accepted, values, np.nan), labels


def foreground_mask(local_height, reference_height, sigma, minimum_gap=0.015, k=3.5):
    """只排除位于喷前可靠表面前方的回波；真实向内退缩绝不按此规则剔除。"""
    local_height = np.asarray(local_height, float)
    reference_height = np.asarray(reference_height, float)
    sigma = np.asarray(sigma, float)
    return (np.isfinite(local_height) & np.isfinite(reference_height) &
            (local_height - reference_height > np.maximum(minimum_gap, k * sigma)))
