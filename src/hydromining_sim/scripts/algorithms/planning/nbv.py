"""阶段七主动观测：信息增益下一最佳视点（NBV）。

经典 utility-NBV 思路，适配 2.5D 高度图：
- 可见性先验 = 各候选视点单独建图后 count>0 的经验掩码（含入射角约束）；
- 增益 = 在可见且当前不可靠的单元上，按传感器模型的精度叠加预测 σ 改善；
- 返回完整评分表用于可解释决策。
"""
import numpy as np

from ..perception.sensor_model import depth_sigma


def uncertain_mask(state, region_mask=None):
    """coverage<2 的单元为待确认；可选限制在任务区域内。"""
    mask = np.asarray(state['coverage']) < 2
    if region_mask is not None:
        mask &= np.asarray(region_mask, bool)
    return mask


def visibility_masks(session, view_groups):
    """各候选视点单独建图，count>0 的单元构成其可见掩码。

    view_groups: {name: snapshots列表(每视点一组)}；不改动 session 自身
    状态——建图在独立会话上进行。
    """
    from ..perception.pipeline import PerceptionSession
    masks = {}
    probe = PerceptionSession(session.center_, session.config_)
    probe.frame_ = session.frame_
    for name, snapshots in view_groups.items():
        surface, _ = probe.build_map([snapshots], session.frame_, None)
        masks[name] = surface.count_ > 0
    return masks


def expected_sigma(centers_world, view_position, normal_axis, noise_scale=1.0):
    """视点对各单元的期望深度 σ：按距离与入射角的传感器模型。"""
    delta = np.asarray(centers_world, float) - np.asarray(view_position, float)
    ranges = np.linalg.norm(delta, axis=-1)
    with np.errstate(divide='ignore', invalid='ignore'):
        directions = delta / ranges[..., None]
    incidence = np.abs(directions @ np.asarray(normal_axis, float))
    return depth_sigma(np.where(np.isfinite(ranges), ranges, 0.), noise_scale) * incidence


def score_views(state, frame, view_poses, visibility, config, region_mask=None):
    """按精度叠加模型预测每视点对不确定单元的 σ 改善量。

    view_poses: {name: (x,y,z)}；visibility: {name: bool掩码}。
    gain = Σ max(σ_current - σ_combined, 0)，只在可见且不可靠的单元上累计。
    """
    uncertain = uncertain_mask(state, region_mask)
    current_sigma = np.where(np.isfinite(state['sigma']), state['sigma'], 2 * config.maximum_sigma)
    centers_world = frame.to_world(frame.grid(config.extent, config.resolution))
    scores = {}
    for name, position in view_poses.items():
        mask = np.asarray(visibility.get(name, np.zeros_like(uncertain)), bool)
        cells = mask & uncertain
        sigma_v = expected_sigma(centers_world, position, frame.axes[:, 2],
                                 config.noise_scale)
        # Repeated views cannot average away the systematic uncertainty floor.
        independent_variance = np.maximum(current_sigma**2-config.systematic_sigma**2, 1e-12)
        combined = np.sqrt(1. / (1. / independent_variance + 1. / np.maximum(sigma_v, 1e-9)**2)
                           + config.systematic_sigma**2)
        improvement = np.where(cells & np.isfinite(combined),
                               np.maximum(current_sigma - combined, 0.), 0.)
        newly_reliable = int((cells & (combined <= config.maximum_sigma) &
                              (current_sigma > config.maximum_sigma)).sum())
        scores[name] = {'gain': float(improvement.sum()), 'newly_reliable': newly_reliable,
                        'cells': int(cells.sum())}
    return scores


def choose_view(state, frame, view_poses, visibility, config, region_mask=None):
    """返回 (最佳视点名, 评分表)；无增益时返回 (None, 评分表)。"""
    scores = score_views(state, frame, view_poses, visibility, config, region_mask)
    if not scores:
        return None, scores
    best = max(scores, key=lambda n: scores[n]['gain'])
    return (best if scores[best]['gain'] > 0 else None), scores


def observed_visibility(state, frame, view_specs, config):
    """Predict dual-sensor FOV/occlusion from measured heights only, never truth meshes.

    Missing heights are not filled: their visibility remains unknown (False).
    March rays through the observed height field; known foreground cells occlude.
    """
    from scipy.ndimage import map_coordinates
    heights = np.asarray(state['height'], float)
    local = frame.grid(config.extent, config.resolution)
    local[..., 2] = heights
    world = frame.to_world(local)
    finite = np.isfinite(heights)
    result = {}
    for name, spec in view_specs.items():
        visible = np.zeros_like(finite)
        for sensor in ('depth', 'lidar'):
            pose = np.asarray(spec[f'tf_{sensor}'], float)
            origin = pose[:3, 3]
            sensor_points = (world - origin) @ pose[:3, :3]
            distance = np.linalg.norm(sensor_points, axis=-1)
            if sensor == 'depth':
                K, (height, width) = spec['K'], spec['image_shape']
                with np.errstate(divide='ignore', invalid='ignore'):
                    col = K[0, 2] - K[0, 0] * sensor_points[..., 1] / sensor_points[..., 0]
                    row = K[1, 2] - K[1, 1] * sensor_points[..., 2] / sensor_points[..., 0]
                mask = (sensor_points[..., 0] > .1) & (col >= 0) & (col < width) & (row >= 0) & (row < height)
            else:
                elevation = np.arctan2(sensor_points[..., 2], np.linalg.norm(sensor_points[..., :2], axis=-1))
                mask = np.abs(elevation) <= np.deg2rad(15)
            mask &= finite & (distance > .1) & (distance < 10.)
            origin_local = frame.to_local(origin)
            # Height is positive towards the observer. A ray below a measured
            # surface has entered coal, so points behind that surface are hidden.
            blocked = np.zeros_like(finite)
            for fraction in np.linspace(.05, .98, 64):
                ray = origin_local + fraction * (local - origin_local)
                indices = [(ray[..., i] - local[0, 0, i]) / config.resolution for i in (1, 0)]
                sampled = map_coordinates(heights, indices, order=0, mode='constant', cval=np.nan)
                blocked |= np.isfinite(sampled) & (ray[..., 2] < sampled - .5 * config.resolution)
            visible |= mask & ~blocked
        result[name] = visible
    return result
