"""任务参数的唯一来源：从任务 yaml 读取，禁止在算法代码里写死压力/靶距/作业区。

同一份 TaskSpec 供 L0/L1/L2 替身环境、规则决策与 ROS 适配层使用，
保证训练、评估和真实闭环看到的是同一组任务参数。
"""
from dataclasses import dataclass, fields
from pathlib import Path

import numpy as np
import yaml


@dataclass(frozen=True)
class TaskSpec:
    """水力破煤局部作业任务；字段与 `phase7_large_task.yaml` 一一对应。"""

    region_center: tuple
    wall_normal: tuple
    wall_inclination_deg: float
    wall_thickness_m: float
    source_coal_pose: tuple
    diameter: float
    required_depth: float
    required_fraction: float
    pressure_mpa: float
    standoff_m: float
    geometry_spacing_m: float
    patch_extent_m: float
    max_steps: int
    viewpoints: dict

    def __post_init__(self):
        finite = [*self.region_center, *self.wall_normal, *self.source_coal_pose,
                  self.wall_inclination_deg, self.wall_thickness_m, self.diameter,
                  self.required_depth, self.required_fraction, self.pressure_mpa,
                  self.standoff_m, self.geometry_spacing_m, self.patch_extent_m]
        if not np.isfinite(finite).all():
            raise ValueError('task parameters must be finite')
        for name, size in (('region_center', 3), ('wall_normal', 3), ('source_coal_pose', 6)):
            if len(getattr(self, name)) != size:
                raise ValueError(f'{name} must have {size} components')
        if np.linalg.norm(self.wall_normal) == 0:
            raise ValueError('wall_normal must be non-zero')
        if min(self.diameter, self.required_depth, self.pressure_mpa,
               self.standoff_m, self.geometry_spacing_m, self.patch_extent_m) <= 0:
            raise ValueError('task dimensions and pressure must be positive')
        if not 0 < self.required_fraction <= 1:
            raise ValueError('required_fraction must be in (0,1]')
        if self.diameter > self.patch_extent_m:
            raise ValueError('task region must fit inside the simulation patch')
        if isinstance(self.max_steps, bool) or not isinstance(self.max_steps, int) or self.max_steps <= 0:
            raise ValueError('max_steps must be a positive integer')
        if not isinstance(self.viewpoints, dict) or not self.viewpoints:
            raise ValueError('viewpoints must be a non-empty mapping')

    @property
    def action_radius(self):
        """喷射点相对区域中心的最大偏移（m）。"""
        return self.diameter/2

    @property
    def duration_range(self):
        """单次喷射时长范围（s）。"""
        return (2., 8.)

    def perception_config(self):
        """由任务生成感知配置；extent/resolution 等地图参数沿用 PerceptionConfig。"""
        from .perception.config import PerceptionConfig
        return PerceptionConfig(diameter=self.diameter, required_depth=self.required_depth,
                                required_fraction=self.required_fraction)

    def as_dict(self):
        """回写为 yaml 同构字典，供仍按字典传参的旧接口使用。"""
        return {'region_center_world': list(self.region_center),
                'wall_normal_world': list(self.wall_normal),
                'wall_inclination_deg': self.wall_inclination_deg,
                'wall_thickness_m': self.wall_thickness_m,
                'source_coal_pose': list(self.source_coal_pose),
                'diameter': self.diameter, 'required_depth': self.required_depth,
                'required_fraction': self.required_fraction,
                'pressure_mpa': self.pressure_mpa, 'standoff_m': self.standoff_m,
                'geometry_spacing_m': self.geometry_spacing_m,
                'patch_extent_m': self.patch_extent_m, 'max_steps': self.max_steps,
                'viewpoints': self.viewpoints}

    @classmethod
    def from_dict(cls, values):
        """拒绝拼错字段，避免配置悄悄退回默认值。"""
        mapping = {'region_center': 'region_center_world', 'wall_normal': 'wall_normal_world'}
        allowed = {mapping.get(field.name, field.name) for field in fields(cls)}
        if not isinstance(values, dict) or set(values) - allowed:
            raise ValueError('unknown task configuration field')
        kwargs = {field.name: values[mapping.get(field.name, field.name)]
                  for field in fields(cls)}
        for name in ('region_center', 'wall_normal', 'source_coal_pose'):
            kwargs[name] = tuple(float(v) for v in kwargs[name])
        return cls(**kwargs)

    @classmethod
    def from_yaml(cls, path):
        return cls.from_dict(yaml.safe_load(Path(path).read_text()))
