"""Versioned policy observations shared by training and real ROS deployment."""
import numpy as np
from scipy.ndimage import zoom


def encode_observation(state, change, assessment, centers, required_depth=.02,
                       confidence_k=1.645, steps=0, max_steps=12, grid=25, version='v2',
                       diameter=.2, pressure=20., standoff=.7):
    """Encode measured state only; no mesh, truth labels or hidden simulator state."""
    if version not in ('v1', 'v2', 'v3'):
        raise ValueError('unsupported observation version')
    delta, sigma = np.asarray(change['delta']), np.asarray(change['sigma'])
    if version == 'v1':
        layers = [np.nan_to_num(delta, nan=0., posinf=0., neginf=0.), np.nan_to_num(sigma, nan=1., posinf=1., neginf=1.),
                  state['coverage'].astype(float)/2., change['significant'].astype(float)]
    else:
        valid = np.isfinite(delta) & np.isfinite(sigma) & (state['coverage'] > 0)
        depth = np.where(valid, np.clip(delta/required_depth, -2, 8), 0.)
        uncertainty = np.where(valid, np.clip(sigma/required_depth, 0, 8), 0.)
        deficit = np.where(valid, np.clip(1 + confidence_k*sigma/required_depth - delta/required_depth, 0, 8), 0.)
        region = np.linalg.norm(np.asarray(centers)[..., :2], axis=-1) <= (diameter / 2 if version == 'v3' else .1)
        layers = [depth, uncertainty, state['coverage'].astype(float)/2.,
                  change['significant'].astype(float), deficit, region.astype(float)]
    layers = np.stack(layers, axis=-1)
    center = layers.shape[0]//2
    half = min(grid//2, center)
    patch = layers if version == 'v3' else layers[center-half:center+half, center-half:center+half]
    if patch.shape[:2] != (grid, grid):
        patch = zoom(patch, (grid/patch.shape[0], grid/patch.shape[1], 1), order=1)
    scalars = [assessment['fraction_lower'], assessment['fraction_upper'],
               assessment['coverage'], steps/max_steps]
    if version == 'v3':
        scalars.extend([pressure / 60., standoff / 2.])
    return np.r_[patch.reshape(-1), scalars].astype(np.float32)


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
