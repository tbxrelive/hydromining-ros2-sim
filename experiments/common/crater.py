"""孔形度量与平板靶：E2.1 破煤响应扫描的基本工具。

高度场约定与 `heightfield_erosion.spray_heightfield` 一致：`heights` 是二维
高度场（本地坐标，z 指向观察者），格心坐标按
`(index + 0.5) * spacing - extent / 2` 排布（与 `WallFrame.grid` 相同）。

`measure` 只依赖 NumPy，因此 L0 高度场与 Level Set 射线实测的高度场
（`local_spray_simulation.surface_rays`）可以直接用同一套口径比较。
"""
import numpy as np


def flat_plate(extent, spacing):
    """生成平整煤壁靶：返回 (heights, grid, axis)。

    heights: (n, n) 全零高度场
    grid:    (n, n, 3) 本地格心坐标，[..., 0]=u（随列变化），[..., 1]=v（随行变化）
    axis:    (n,) 一维坐标轴，用于构造 Level Set 平板网格
    """
    size = int(round(extent/spacing))
    if size < 2 or not np.isclose(size*spacing, extent):
        raise ValueError('extent 必须是 spacing 的整数倍')
    axis = (np.arange(size) + .5)*spacing - extent/2
    u, v = np.meshgrid(axis, axis)
    return np.zeros((size, size)), np.stack((u, v, np.zeros_like(u)), axis=-1), axis


def _index_of(coord, spacing, extent, size):
    """把本地坐标换算成最近的格心下标（与 heightfield_erosion 的换算一致）。"""
    return int(np.clip(round((coord + extent/2)/spacing - .5), 0, size - 1))


def _spans(region, spacing):
    """区域在 u、v 两个方向的跨度（m）；空区域返回 (0, 0)。"""
    rows = np.flatnonzero(np.any(region, axis=1))
    cols = np.flatnonzero(np.any(region, axis=0))
    if not len(rows) or not len(cols):
        return 0., 0.
    span_v = (rows[-1] - rows[0] + 1)*spacing
    span_u = (cols[-1] - cols[0] + 1)*spacing
    return float(span_u), float(span_v)


def _circle_diameter(area):
    """等效圆直径：面积相同圆的直径。"""
    return float(2*np.sqrt(max(area, 0.)/np.pi))


def measure(before, after, spacing, *, center=None, fraction=0.5, absolute=0.01):
    """度量一次（或多次）喷射形成的孔。

    Args:
        before, after: 喷射前/后的高度场，形状相同
        spacing: 格距 [m]
        center: 喷射点 (u, v)；省略时以最大孔深处为中心
        fraction: 孔径判据之一——深度达到最大孔深该比例的区域
        absolute: 孔径判据之二——深度达到该绝对值 [m] 的区域

    Returns:
        字典：最大孔深、中心孔深、两种孔径、去除体积、孔形对称性、中心偏移
    """
    before, after = np.asarray(before, float), np.asarray(after, float)
    if before.shape != after.shape or before.ndim != 2:
        raise ValueError('before/after 必须是同形状的二维高度场')
    retreat = before - after
    size = retreat.shape[0]
    extent = size*spacing
    depth_max = float(retreat.max())
    row, col = np.unravel_index(int(np.argmax(retreat)), retreat.shape)
    if center is not None:
        # 行下标对应 v，列下标对应 u（见 flat_plate 的坐标约定）。
        row = _index_of(center[1], spacing, extent, size)
        col = _index_of(center[0], spacing, extent, size)
    depth_center = float(retreat[row, col])
    cell_area = spacing**2
    relative = retreat >= fraction*depth_max if depth_max > 0 else np.zeros(retreat.shape, bool)
    absolute_region = retreat >= absolute
    span_u, span_v = _spans(absolute_region, spacing)
    u_at = (col + .5)*spacing - extent/2
    v_at = (row + .5)*spacing - extent/2
    return {
        'depth_max_m': depth_max,
        'depth_center_m': depth_center,
        'diameter_half_m': _circle_diameter(relative.sum()*cell_area),
        'diameter_abs_m': _circle_diameter(absolute_region.sum()*cell_area),
        'volume_m3': float(retreat.sum()*cell_area),
        'area_abs_m2': float(absolute_region.sum()*cell_area),
        'symmetry': float(span_u/span_v) if span_v > 0 else 1.,
        'center_offset_m': (float(np.hypot(u_at - center[0], v_at - center[1]))
                            if center is not None else 0.),
    }


def through_check(before, after, thickness, tolerance=1e-9):
    """检查是否被打穿（任一格退缩量达到壁厚）。"""
    retreat = np.asarray(before, float) - np.asarray(after, float)
    return bool(np.any(retreat >= thickness - tolerance))
