"""干扰等级注册表与受扰比例度量。

两层干扰必须共用同一套口径：
- 软件层（`sensor_model` / `environment`）：能见度 V 驱动的丢点、假回波、
  噪声放大；
- Gazebo 粒子层：`jet_effects.cloud_change` 统计的受扰比例。

**容差为什么必须逐射线**：深度相机的噪声模型是 σ = 1%·Z²，在 1 m 外
σ 已超过 1 cm，若用固定 1 cm 容差，"洁净"条件下也会被判成大面积受扰，
测到的是传感器本身而不是水雾。因此容差取 `max(下限, k·σ)`：洁净时
D≈0，只有超出传感器噪声包络的偏差才算受扰。Gazebo 的传感器噪声远小于
1 cm，两种口径在该前提下退化为同一个数（见 `test_interference_levels.py`）。
"""
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from algorithms.perception import sensor_model

# 受扰判据的默认参数：3σ 覆盖约 99.7% 的传感器噪声，下限与 Gazebo 口径一致。
DEFAULT_SIGMA_MULTIPLE = 3.0
DEPTH_TOLERANCE_FLOOR_M = .01
LIDAR_TOLERANCE_FLOOR_M = .02

DEFAULT_LEVELS_PATH = Path(__file__).resolve().parents[2]/'configs/interference_levels.yaml'


@dataclass(frozen=True)
class InterferenceLevel:
    """一个干扰等级：能见度 + 传感器噪声倍数。"""

    name: str
    visibility_m: float
    noise_scale: float = 1.0

    def __post_init__(self):
        if not self.visibility_m > 0:
            raise ValueError('visibility must be positive')
        if not np.isfinite(self.noise_scale) or self.noise_scale < 0:
            raise ValueError('noise_scale must be finite and nonnegative')

    @property
    def label(self):
        """中文等级名，用于图表与文档。"""
        return {'none': '无', 'low': '低', 'medium': '中', 'high': '高'}.get(self.name, self.name)

    def as_dict(self):
        return {'name': self.name, 'visibility_m': self.visibility_m,
                'noise_scale': self.noise_scale}


def load_levels(path=DEFAULT_LEVELS_PATH):
    """从 yaml 读取等级表，返回按声明顺序排列的字典。"""
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f'干扰等级配置不存在: {path}')
    values = yaml.safe_load(path.read_text())['levels']
    levels = {}
    for name, spec in values.items():
        levels[name] = InterferenceLevel(name, float(spec['visibility_m']),
                                         float(spec.get('noise_scale', 1.0)))
    return levels


def sensor_tolerance(distance, sensor, sigma_multiple=DEFAULT_SIGMA_MULTIPLE,
                     floor=None):
    """逐射线受扰容差 [m]：传感器噪声包络与固定下限取大者。"""
    distance = np.asarray(distance, dtype=float)
    if sensor == 'depth':
        sigma = sensor_model.depth_sigma(np.where(np.isfinite(distance), distance, 0.))
        floor = DEPTH_TOLERANCE_FLOOR_M if floor is None else floor
    elif sensor == 'lidar':
        sigma = sensor_model.lidar_sigma(np.where(np.isfinite(distance), distance, 0.))
        floor = LIDAR_TOLERANCE_FLOOR_M if floor is None else floor
    else:
        raise ValueError('sensor must be depth or lidar')
    return np.maximum(sigma_multiple*sigma, floor)


def disturbed_fraction(reference, observed, tolerance, mask=None):
    """逐射线容差版的受扰比例；`jet_effects.cloud_change` 的推广。

    Args:
        reference: 洁净点云 (N, 3)
        observed: 观测点云 (N, 3)，丢点处为 NaN
        tolerance: 标量或 (N,) 容差 [m]
        mask: 参与统计的射线；省略时要求参考点为有限值
    Returns:
        与 `cloud_change` 同名字段：目标回波数、可见比例、受扰比例、有限比例
    """
    a, b = np.asarray(reference, float), np.asarray(observed, float)
    if a.shape != b.shape or a.ndim != 2 or a.shape[1] != 3:
        raise ValueError('reference/observed 必须是同形状的 (N,3) 点云')
    base = np.isfinite(a).all(axis=1)
    mask = base if mask is None else (np.asarray(mask, bool) & base)
    if mask.sum() < 10:
        raise ValueError('洁净目标回波不足 10 个')
    error = np.linalg.norm(b[mask]-a[mask], axis=1)
    limit = np.asarray(tolerance, float)
    if limit.ndim:
        limit = limit[mask]
    visible = np.isfinite(error) & (error <= limit)
    return {'target_returns': int(mask.sum()), 'visible_fraction': float(visible.mean()),
            'disturbed_fraction': float((~visible).mean()),
            'finite_fraction': float(np.isfinite(error).mean())}
