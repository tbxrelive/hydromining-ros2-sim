"""感知配置与资源边界；阈值为待验收工程值，不宣称实测标定。"""
from dataclasses import dataclass, fields

import numpy as np


@dataclass(frozen=True)
class PerceptionConfig:
    """算法参数集中定义，ROS 适配层与离线实验共用同一份配置。"""

    extent: float = 0.5
    resolution: float = 0.01
    diameter: float = 0.2
    required_depth: float = 0.02
    required_fraction: float = 0.9
    confidence_k: float = 1.645
    reference_padding: float = 0.05
    noise_scale: float = 1.0
    systematic_sigma: float = 0.0015
    registration_sigma: float = 0.001
    minimum_observations: int = 2
    maximum_sigma: float = 0.01
    temporal_support: int = 2
    temporal_k: float = 3.5
    temporal_noise_floor: float = 0.003
    foreground_gap: float = 0.015
    maximum_skew_s: float = 0.6
    stationary_translation: float = 0.002
    stationary_rotation: float = 0.01
    coal_density: float = 1400.
    external_retreat: float = 0.01
    maximum_frames_per_view: int = 30
    maximum_views: int = 8

    def __post_init__(self):
        values = [getattr(self, field.name) for field in fields(self)]
        if not np.isfinite(values).all():
            raise ValueError('perception parameters must be finite')
        nonnegative = ('noise_scale', 'registration_sigma')
        for field in fields(self):
            value = getattr(self, field.name)
            if value < 0 or (value == 0 and field.name not in nonnegative):
                raise ValueError(f'{field.name} must be positive')
        if (self.diameter > self.extent or self.required_fraction > 1 or
                not 0.002 <= self.resolution <= 0.02 or
                self.extent / self.resolution > 150 or
                not np.isclose(round(self.extent / self.resolution) * self.resolution, self.extent)):
            raise ValueError('invalid region or bounded map dimensions')
        for name in ('minimum_observations', 'temporal_support',
                     'maximum_frames_per_view', 'maximum_views'):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int):
                raise ValueError(f'{name} must be an integer')
        if self.maximum_frames_per_view > 30 or self.maximum_views > 8:
            raise ValueError('observation storage exceeds resource budget')

    @classmethod
    def from_dict(cls, values):
        """拒绝拼错字段，避免配置悄悄退回默认值。"""
        allowed = {field.name for field in fields(cls)}
        if not isinstance(values, dict) or set(values) - allowed:
            raise ValueError('unknown perception configuration field')
        return cls(**values)
