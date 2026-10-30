"""传感器误差与煤矿环境退化模型（软件层，未经实测标定）。

规格来源（2026-10-27 核对公开规格书）：
- 深度相机取 Intel RealSense D435 级：Depth Quality 规范给出 RMS 误差 <= 2% Z
  （2 m 内、80% ROI），时间噪声 <= 1% Z。立体深度误差随 Z^2 增长，故取
  sigma = K_DEPTH * z^2，K_DEPTH=0.01 1/m 在 2 m 处恰为 2% Z，属规格上限。
- LiDAR 取 Velodyne VLP-16 级：Range Accuracy 典型值 +/-3 cm。规格没有给出
  这一范围对应的概率分布；此处假设 0.03 m 为 1 sigma，仅为压力测试参数。
环境退化（粉尘/水雾）使用 Koschmieder 关系 beta = 3.912 / V，往返透过率
T = exp(-2 beta r)。丢点概率、噪声放大、近距散射假点的形式均为工程假设，
BACKSCATTER_GAIN 无实测依据，只用于压力测试，不得当作已标定参数。
"""
import numpy as np

K_DEPTH = 0.01
SIGMA_LIDAR = 0.03
BACKSCATTER_GAIN = 0.3
NOISE_AMPLIFY_CAP = 10.0
VALID, DROPPED, FALSE_RETURN, INVALID = 0, 1, 2, 3

ENV_PRESETS = {'clean': np.inf, 'dim_dust_light': 8.0,
               'dim_dust_heavy': 3.0, 'dim_dust_extreme': 1.5}


def depth_sigma(z, scale=1.0):
    """深度相机 1 sigma（m），随距离平方增长。"""
    _validate_scale(scale)
    if np.any(~np.isfinite(z)) or np.any(np.asarray(z) < 0):
        raise ValueError('depth must be finite and nonnegative')
    return scale * K_DEPTH * np.asarray(z, dtype=float) ** 2


def lidar_sigma(r, scale=1.0):
    """LiDAR 1 sigma（m），取规格典型精度作常数。"""
    _validate_scale(scale)
    if np.any(~np.isfinite(r)) or np.any(np.asarray(r) < 0):
        raise ValueError('range must be finite and nonnegative')
    return scale * SIGMA_LIDAR * np.ones_like(np.asarray(r, dtype=float))


def _validate_scale(scale):
    """拒绝会导致负方差或非有限噪声的参数。"""
    if not np.isscalar(scale) or not np.isfinite(scale) or scale < 0:
        raise ValueError('noise scale must be finite and nonnegative')


def extinction(visibility):
    """消光系数 beta（1/m）；能见度无穷大时为 0。"""
    if not visibility > 0:
        raise ValueError('visibility must be positive')
    return 0.0 if np.isinf(visibility) else 3.912 / visibility


def transmittance(r, visibility):
    """往返透过率。"""
    if np.any(~np.isfinite(r)) or np.any(np.asarray(r) < 0):
        raise ValueError('range must be finite and nonnegative')
    return np.exp(-2.0 * extinction(visibility) * np.asarray(r, dtype=float))


def degrade_ranges(ranges, sigma, visibility, rng, r_min=0.1):
    """对逐射线距离施加环境退化。

    Args:
        ranges: 干净距离（m），NaN 表示本来无回波。
        sigma: 与 ranges 同形的 1 sigma（m）。
        visibility: 能见度（m），np.inf 表示洁净。
        rng: numpy Generator，保证可复现。
    Returns:
        (退化后距离, 标签)；原本无效的回波标记为 INVALID，不计作环境丢点。
    """
    r = np.asarray(ranges, dtype=float).copy()
    sig = np.broadcast_to(np.asarray(sigma, dtype=float), r.shape)
    if not np.isfinite(r_min) or r_min <= 0:
        raise ValueError('r_min must be finite and positive')
    if np.any(~np.isfinite(sig)) or np.any(sig < 0):
        raise ValueError('sigma must be finite and nonnegative')
    labels = np.full(r.shape, VALID, dtype=np.int8)
    beta = extinction(visibility)
    has = np.isfinite(r) & (r >= r_min)
    original = np.where(has, r, 0.0)
    labels[~has] = INVALID
    r[~has] = np.nan
    t = np.where(has, np.exp(-2.0 * beta * np.where(has, r, 0.0)), 1.0)
    amplify = np.minimum(1.0 / np.sqrt(np.maximum(t, 1e-12)), NOISE_AMPLIFY_CAP)
    r[has] += rng.normal(0.0, 1.0, has.sum()) * (sig * amplify)[has]
    drop = has & (rng.random(r.shape) > t)
    r[drop] = np.nan
    labels[drop] = DROPPED
    if beta > 0:
        p_false = BACKSCATTER_GAIN * (1.0 - np.exp(-beta * original))
        false = has & ~drop & (rng.random(r.shape) < p_false)
        if false.any():
            upper = original[false]
            u = rng.random(false.sum())
            span = 1.0 - np.exp(-beta * np.maximum(upper - r_min, 0.0))
            r[false] = r_min - np.log(1.0 - u * span) / beta
            labels[false] = FALSE_RETURN
    below_minimum = has & np.isfinite(r) & (r < r_min)
    r[below_minimum] = np.nan
    labels[below_minimum] = DROPPED
    return r, labels
