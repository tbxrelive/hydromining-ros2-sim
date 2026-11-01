#!/usr/bin/env python3
"""
冲蚀模型 - Phase 4.1.2

实现用户论文的冲蚀响应公式：
  ⑥ 径向距离 r(x,t)
  ⑦ 有效作用掩码 χ(x,t)
  ⑨ 冲蚀速度 v_e(x,t)

遵循AGENTS.md第2️⃣：纯算法层，无ROS依赖
"""

import numpy as np
from typing import Callable, Tuple, Optional
from dataclasses import dataclass


@dataclass
class SprayAction:
    """喷射动作参数"""
    position: np.ndarray    # 喷嘴位置 (x, y, z) [m]
    direction: np.ndarray   # 射流方向（单位向量）
    pressure: float         # 压力 [MPa]
    duration: float         # 作用时间 [s]
    g: int = 1              # 喷射状态 0=关闭, 1=开启

    def __post_init__(self):
        """Reject invalid inputs before touching the coal state."""
        self.position = np.asarray(self.position, dtype=float)
        self.direction = np.asarray(self.direction, dtype=float)
        if self.position.shape != (3,) or self.direction.shape != (3,):
            raise ValueError('position and direction must be 3-vectors')
        if not np.all(np.isfinite(np.r_[self.position, self.direction, self.pressure, self.duration])):
            raise ValueError('Spray inputs must be finite')
        if abs(np.linalg.norm(self.direction)-1) > 1e-5:
            raise ValueError('direction must be a unit vector')
        if not 0 <= self.pressure <= 60 or not 0 < self.duration <= 60 or self.g not in (0, 1):
            raise ValueError('Require 0<=pressure<=60 MPa, 0<duration<=60 s, g in {0,1}')


class ErosionKernel:
    """
    冲蚀核函数及参数

    实现公式⑧和公式⑨中的参数函数。径向核的指数做成实例参数，供 E2.1a
    标定扫描使用；构造默认值与此前硬编码的 (3/2, 6) 完全相同。
    """

    def __init__(
        self,
        R_c_func: Callable[..., float],
        b_func: Callable[[float, float], float],
        radial_power=1.5,
        radial_decay=6,
    ):
        """
        初始化冲蚀核

        Args:
            R_c_func: Callable(pressure, distance) -> float
                      中心冲蚀速度 [m/s]
            b_func: Callable(pressure, distance) -> float
                    有效冲蚀半径 [m]
            radial_power: 径向核 ξ 的幂次（默认 3/2）
            radial_decay: 径向核外侧衰减幂次（默认 6）
        """
        self.R_c = R_c_func
        self.b = b_func
        # 不做 float 转换：默认值保持 int 6，避免与旧实现的整数幂快速路径产生末位差异。
        self.radial_power = radial_power
        self.radial_decay = radial_decay

    def kernel_F(self, xi: np.ndarray) -> np.ndarray:
        """
        冲蚀径向核函数

        公式⑧: F(ξ) = (1-ξ^(3/2))^6, for 0 ≤ ξ < 1
                      = 0,              for ξ ≥ 1

        Args:
            xi: 归一化径向距离 r/b (形状任意)

        Returns:
            F(xi) ∈ [0, 1]
        """
        return np.where(xi < 1, (1 - xi**self.radial_power)**self.radial_decay, 0)


class DefaultErosionKernel(ErosionKernel):
    """默认冲蚀核：参数可注入，默认值按 E2.1a 文献标定（全文证据）。

    公式与文献依据见 docs/文献/E2.1_标定依据.md。要点：
    - 阈值压力 P_th：h = k₁(P − Pc)（Zhang 2019 式(8)；Crow 模型与
      Zhang & Zou 式(18) 同样要求阈值）；低于阈值不破煤；
    - 压力指数 α = 1（Zhang 2019 拟合值 1.0005）；
    - 靶距指数 β = 2（充分发展射流：v∝1/s，动压∝1/s²）；
    - 射流半宽 b = b0 + 0.22·s（Zhang & Zou：半径按 0.22x 增长，与压力无关）；
    - 径向剖面 (1−ξ^1.5)^6：Zhang & Zou 式(3) 给出的是**速度**剖面 (1−η^1.5)^2，
      而式(8) 的冲蚀速率 ∝ ux³，故**侵蚀**剖面是速度剖面的三次方，即 (1−η^1.5)^6。
      式(9) 的显式结果正是 6 次方，这一点写反会把射流的影响范围拓得太宽。

    煤强度接口：强度通过**缩放阈值压力**进入（强度越高、阈值越高），
    `strength_exponent=0` 时恒为中性，公式不变。
    """

    # 全部可调参数的默认值；键名同时是 yaml `kernel:` 段允许的字段。
    DEFAULTS = {
        'k': 1.3945840774443334e-4,   # R_c 经验系数；锚定“标称单次喷射（30MPa,0.5m,4s）恰好 50 mm”
        'alpha': 1.0,                 # R_c 的压力指数（文献拟合 1.0005）
        'beta': 2.0,                  # R_c 的靶距指数（充分发展射流：v∝1/s，动压∝1/s²）
        'pressure_threshold': 5.0,    # 阈值破煤压力 [MPa]；工程取值（见标定依据 4.2）
        'min_distance': 0.01,         # 靶距下限，避免除零 [m]
        'b_base': 0.001,              # 喷嘴半宽 [m]（2 mm 喷嘴）
        'b_pressure': 0.0,            # 文献：半宽与压力无关；参数保留供敏感性检验
        'b_distance': 0.22,           # 射流半宽增长系数（Zhang & Zou：b = 0.22x）
        'radial_power': 1.5,          # 径向核 ξ 幂次（文献一致）
        'radial_decay': 6,            # 侵蚀剖面指数 = 速度剖面 (1−η^1.5)^2 的三次方（式(8)(9)）
        'reference_strength_mpa': 30.,  # 强度参考值，E2.2 使用
        'strength_exponent': 0.,      # 强度指数，0 表示不启用强度（当前状态）
        'coal_strength_mpa': 0.,      # 煤体单轴抗压强度，0 表示未知/未启用
    }

    def __init__(self, **params):
        """按关键字覆盖默认参数；未知参数名直接报错，避免拼写错误被忽略。"""
        unknown = set(params) - set(self.DEFAULTS)
        if unknown:
            raise ValueError(f'未知的冲蚀核参数: {sorted(unknown)}')
        for name, value in params.items():
            if not np.isfinite(value):
                raise ValueError(f'冲蚀核参数必须有限: {name}={value}')
        values = {**self.DEFAULTS, **params}
        self.k = values['k']
        self.alpha = values['alpha']
        self.beta = values['beta']
        self.pressure_threshold = values['pressure_threshold']
        self.min_distance = values['min_distance']
        self.b_base = values['b_base']
        self.b_pressure = values['b_pressure']
        self.b_distance = values['b_distance']
        self.reference_strength_mpa = values['reference_strength_mpa']
        self.strength_exponent = values['strength_exponent']
        self.coal_strength_mpa = values['coal_strength_mpa']
        super().__init__(R_c_func=self.R_c, b_func=self.b,
                         radial_power=values['radial_power'],
                         radial_decay=values['radial_decay'])

    def params(self):
        """返回全部参数，供 manifest 记录与敏感性分析枚举。"""
        return {name: getattr(self, name) for name in self.DEFAULTS
                if name not in ('radial_power', 'radial_decay')} | {
            'radial_power': self.radial_power, 'radial_decay': self.radial_decay}

    def strength_factor(self, coal_strength_mpa=None):
        """强度对阈值压力的缩放因子；强度指数为 0 或强度未给出时恒等于 1.0。"""
        strength = self.coal_strength_mpa if coal_strength_mpa is None else coal_strength_mpa
        if self.strength_exponent == 0 or strength is None or strength <= 0:
            return 1.0
        return (strength/self.reference_strength_mpa)**self.strength_exponent

    def threshold(self, coal_strength_mpa=None):
        """有效阈值压力 [MPa]；煤体越硬阈值越高。"""
        return self.pressure_threshold*self.strength_factor(coal_strength_mpa)

    def R_c(self, pressure: float, distance: float, coal_strength_mpa=None) -> float:
        """
        中心冲蚀速度 [m/s]

        R_c = k·(P − P_th)^α / s^β；P ≤ P_th 时为 0（不破煤）。

        Args:
            pressure: 压力 [MPa]
            distance: 靶距 [m]
            coal_strength_mpa: 可选，该点的煤体强度；省略时用核内设定值

        Returns:
            中心冲蚀速度 [m/s]
        """
        if distance < self.min_distance:
            distance = self.min_distance  # 避免除以零
        effective = pressure - self.threshold(coal_strength_mpa)
        if effective <= 0:
            return 0.0
        return self.k*(effective**self.alpha)/(distance**self.beta)

    def b(self, pressure: float, distance: float) -> float:
        """
        有效冲蚀半径 [m]

        b = b_base + b_pressure·P + b_distance·s；
        文献形（Zhang & Zou）：b = b0 + 0.22·s，与压力无关（b_pressure=0）。

        Args:
            pressure: 压力 [MPa]
            distance: 靶距 [m]

        Returns:
            有效冲蚀半径 [m]
        """
        return self.b_base + self.b_pressure*pressure + self.b_distance*distance


class ErosionModel:
    """Surface response (9), recomputed on the current zero set each step.

    Closest-point extension carries the visible surface speed into a 3h narrow
    band. BVH first-hit tests include self occlusion and optional external mesh
    occluders in the same coordinate frame. No angular attenuation is invented.
    """

    def __init__(self, erosion_kernel):
        self.kernel = erosion_kernel
        self.last_hit_distance = None

    @staticmethod
    def surface_scene(phi, origin, spacing):
        """Build the current triangle surface, with outward-facing normals."""
        import open3d as o3d
        from .mesh_converter import MeshConverter
        vertices, faces = MeshConverter.voxel_to_mesh(phi, origin, spacing)
        mesh = o3d.t.geometry.TriangleMesh(
            o3d.core.Tensor(vertices.astype(np.float32)),
            o3d.core.Tensor(faces.astype(np.uint32)))
        scene = o3d.t.geometry.RaycastingScene(nthreads=1)
        scene.add_triangles(mesh)
        return scene

    @staticmethod
    def compute_radial_distance(points, ray_origin, ray_direction):
        """Equation (6), requiring a unit direction."""
        v = np.asarray(points) - ray_origin
        return np.linalg.norm(v - np.sum(v*ray_direction, axis=-1)[..., None]*ray_direction, axis=-1)

    @classmethod
    def find_ray_mesh_intersection(cls, ray_origin, ray_direction, phi, origin, spacing):
        """Current first hit from the positive ray; None means no erosion."""
        import open3d as o3d
        scene = cls.surface_scene(phi, origin, spacing)
        ray = np.r_[ray_origin, ray_direction].astype(np.float32)[None, :]
        hit = float(scene.cast_rays(o3d.core.Tensor(ray), nthreads=1)['t_hit'].numpy()[0])
        return hit if np.isfinite(hit) and hit > 1e-6 else None

    def compute_erosion_velocity(self, points, spray_action, phi, origin, spacing,
                                 normals=None, occluder_meshes=()):
        """Evaluate (7)--(9); geometry is current, duration is handled by integration."""
        import open3d as o3d
        action = spray_action
        velocity = np.zeros_like(phi, dtype=np.float32)
        self.last_hit_distance = None
        if action.g == 0 or action.pressure == 0:
            return velocity
        scene = self.surface_scene(phi, origin, spacing)
        ray = np.r_[action.position, action.direction].astype(np.float32)[None, :]
        hit = float(scene.cast_rays(o3d.core.Tensor(ray), nthreads=1)['t_hit'].numpy()[0])
        if not np.isfinite(hit) or hit <= 1e-6:
            return velocity
        self.last_hit_distance = hit
        rc = float(self.kernel.R_c(action.pressure, hit))
        radius = float(self.kernel.b(action.pressure, hit))
        if not np.isfinite(rc+radius) or rc < 0 or radius <= 0:
            raise ValueError('R_c must be nonnegative and b positive, both finite')
        radial = self.compute_radial_distance(points, action.position, action.direction)
        mask = (np.abs(phi) <= 3*spacing) & (radial < radius+3*spacing)
        selected = points[mask].astype(np.float32)
        if not len(selected):
            return velocity
        closest = scene.compute_closest_points(o3d.core.Tensor(selected), nthreads=1)
        surface = closest['points'].numpy()
        outward = closest['primitive_normals'].numpy()
        delta = surface-action.position
        distances = np.linalg.norm(delta, axis=1)
        directions = delta/np.maximum(distances[:, None], 1e-12)
        for vertices, faces in occluder_meshes:
            scene.add_triangles(o3d.t.geometry.TriangleMesh(
                o3d.core.Tensor(np.asarray(vertices, dtype=np.float32)),
                o3d.core.Tensor(np.asarray(faces, dtype=np.uint32))))
        rays = np.c_[np.broadcast_to(action.position, surface.shape), directions].astype(np.float32)
        first = scene.cast_rays(o3d.core.Tensor(rays), nthreads=1)['t_hit'].numpy()
        exposed = (np.abs(first-distances) < 0.05*spacing) & (np.sum(outward*directions, axis=1) < -1e-4)
        forward = delta@action.direction > 0
        r = self.compute_radial_distance(surface, action.position, action.direction)
        velocity[mask] = exposed*forward*rc*self.kernel.kernel_F(r/radius)
        return velocity
