"""高度场蚀除：L0 快速环境使用的径向核积分。

与 Level Set 求解器共用同一个 `ErosionKernel`，这里只负责在 2.5D 高度场上
做时间积分、暴露判定与射线遮挡。数值过程与重构前的 FastHeightfieldEnv._spray
逐位一致；改动需与 L0 规则基线逐回合比对。
"""
import numpy as np
from scipy.ndimage import map_coordinates

STEP_S = .2
EXPOSURE_FRACTIONS = (.5, .75, .875, .9375, .96875, .984375)
EXPOSURE_TOLERANCE = .00025
MIN_STANDOFF = .01


def spray_heightfield(heights, surface_grid, u, v, duration, pressure, standoff,
                      kernel, spacing, extent):
    """在高度场 `heights` 上原地积分一次喷射，返回体量与最大退缩。

    heights: 2D 高度场（本地坐标，z 指向观察者）
    surface_grid: (…,3) 本地格点坐标，[...,0]=u，[...,1]=v
    u, v: 喷射点（本地坐标，米）；duration: 秒；pressure: MPa；standoff: 米
    """
    before = heights.copy()
    radial = np.hypot(surface_grid[..., 0]-u, surface_grid[..., 1]-v)
    target = np.array([[(v+extent/2)/spacing-.5],
                       [(u+extent/2)/spacing-.5]])
    elapsed = 0.
    while elapsed < duration-1e-9:
        step = min(STEP_S, duration-elapsed)
        height = float(map_coordinates(heights, target, order=1, mode='nearest')[0])
        distance = max(MIN_STANDOFF, standoff-height)
        radius = kernel.b(pressure, distance)
        xi = radial / radius
        inside = xi < 1.
        response = kernel.kernel_F(xi[inside])
        # 深坑会遮住自己的表面：同时检查正对面与首次命中两个条件。
        grad_v, grad_u = np.gradient(heights, spacing)
        du = surface_grid[..., 0][inside]-u
        dv = surface_grid[..., 1][inside]-v
        local_heights = heights[inside]
        gu, gv = grad_u[inside], grad_v[inside]
        exposed = (-gu*du-gv*dv+local_heights-standoff) < 0.
        for fraction in EXPOSURE_FRACTIONS:
            x = u + fraction*du
            y = v + fraction*dv
            coords = np.stack(((y+extent/2)/spacing-.5,
                               (x+extent/2)/spacing-.5))
            foreground = map_coordinates(heights, coords, order=1,
                                         mode='constant', cval=-np.inf)
            ray_height = standoff + fraction*(local_heights-standoff)
            exposed &= ray_height >= foreground-EXPOSURE_TOLERANCE
        # 斜面上的法向速度对应更大的高度变化（高度场 Level Set 方程）。
        slope_factor = np.sqrt(1. + gu**2 + gv**2)
        heights[inside] -= kernel.R_c(pressure, distance)*response*exposed*slope_factor*step
        elapsed += step
    retreat = before-heights
    return {'volume_removed_m3': float(retreat.sum()*spacing**2),
            'max_depth_m': float(retreat.max())}
