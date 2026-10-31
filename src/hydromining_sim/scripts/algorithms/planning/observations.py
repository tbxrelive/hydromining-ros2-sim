"""Versioned policy observations shared by training and real ROS deployment."""
import numpy as np
from scipy.ndimage import zoom


# 默认值属于旧 20 cm 任务；论文实验必须由 TaskSpec 显式传参，1.3 删除默认值。
def encode_observation(state, change, assessment, centers, required_depth, *,
                       confidence_k=1.645, steps=0, max_steps=12, grid=25,
                       diameter, pressure, standoff):
    """Encode measured state only; no mesh, truth labels or hidden simulator state.

    v3 编码：任务区域网格层（深度/不确定度/覆盖/显著/缺口/区域）+ 6 个标量。
    """
    delta, sigma = np.asarray(change['delta']), np.asarray(change['sigma'])
    valid = np.isfinite(delta) & np.isfinite(sigma) & (state['coverage'] > 0)
    depth = np.where(valid, np.clip(delta/required_depth, -2, 8), 0.)
    uncertainty = np.where(valid, np.clip(sigma/required_depth, 0, 8), 0.)
    deficit = np.where(valid, np.clip(1 + confidence_k*sigma/required_depth - delta/required_depth, 0, 8), 0.)
    region = np.linalg.norm(np.asarray(centers)[..., :2], axis=-1) <= (diameter / 2)
    layers = np.stack([depth, uncertainty, state['coverage'].astype(float)/2.,
                       change['significant'].astype(float), deficit, region.astype(float)], axis=-1)
    if layers.shape[:2] != (grid, grid):
        layers = zoom(layers, (grid/layers.shape[0], grid/layers.shape[1], 1), order=1)
    scalars = [assessment['fraction_lower'], assessment['fraction_upper'],
               assessment['coverage'], steps/max_steps, pressure / 60., standoff / 2.]
    return np.r_[layers.reshape(-1), scalars].astype(np.float32)


def potential_reward(previous, current, gamma=.99, terminal=False):
    """Potential shaping uses zero potential at true terminal states, not timeouts."""
    return 8 * ((0. if terminal else gamma*current) - previous)


def depth_progress(change, area_weights, required_depth=.05, confidence_k=1.645):
    """Conservative continuous completion; unknown cells contribute zero."""
    valid = np.asarray(change['valid'], bool)
    delta = np.asarray(change['delta'])
    sigma = np.asarray(change['sigma'])
    progress = np.where(valid, np.clip((delta-confidence_k*sigma)/required_depth, 0., 1.), 0.)
    weights = np.asarray(area_weights)
    return float(np.sum(progress * weights) / np.sum(weights))
