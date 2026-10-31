"""离线与 ROS 共用的感知会话，观测输入不包含真值或评估标签。"""
import numpy as np

from .alignment import aligned_sensor, validate_sequence
from .assessment import assess, circle_fractions
from .change_detection import compare_surfaces, external_change_alarm, volume_summary
from .config import PerceptionConfig
from .frames import fit_reference_plane
from .height_map import HeightMap
from .interference_filter import foreground_mask, temporal_filter
from .wall_frame import WallFrame


class PerceptionSession:
    """保存固定初始参考系和基线图；每次喷后评估构建独立新图，绝不混合旧表面。"""

    def __init__(self, center, config=None):
        self.config_ = config or PerceptionConfig()
        self.center_ = np.asarray(center, float)
        if self.center_.shape != (3,) or not np.isfinite(self.center_).all():
            raise ValueError('task center must contain three finite coordinates')
        self.frame_ = self.baseline_ = self.current_groups_ = None

    def validate_groups(self, groups):
        """对每个静止视点独立校验时间戳/姿态，并限制缓存规模。"""
        cfg = self.config_
        if not 1 <= len(groups) <= cfg.maximum_views:
            raise ValueError('viewpoint count exceeds budget')
        for snapshots in groups:
            if not 1 <= len(snapshots) <= cfg.maximum_frames_per_view:
                raise ValueError('frame count exceeds budget')
            validate_sequence(snapshots, cfg.maximum_skew_s, cfg.stationary_translation,
                              cfg.stationary_rotation)

    def set_baseline(self, groups, progress=None, cancel=None):
        """只由喷前局部观测拟合参考法向，成功构建后一次性提交基线。"""
        self.validate_groups(groups)
        points, viewpoints = [], []
        radius = self.config_.diameter / 2 + self.config_.reference_padding
        for snapshots in groups:
            for snapshot in snapshots:
                world, _, _ = aligned_sensor(snapshot, 'depth', self.config_.noise_scale)
                world = world.reshape(-1, 3)
                valid = np.isfinite(world).all(axis=1)
                valid &= np.linalg.norm(world - self.center_, axis=1) <= radius
                points.append(world[valid])
                viewpoints.append(snapshot['tf_depth'][:3])
        _, normal, _ = fit_reference_plane(
            np.concatenate(points), self.center_, np.mean(viewpoints, axis=0))
        frame = WallFrame.from_normal(self.center_, normal)
        baseline, filtering = self.build_map(groups, frame, None, progress, cancel)
        self.frame_, self.baseline_ = frame, baseline
        self.current_groups_ = None
        return {'frame': frame, 'map': baseline, 'filtering': filtering}

    def build_map(self, groups, frame, reference=None, progress=None, cancel=None):
        """顺序处理有界视点；在单视点内先过滤，再按独立帧融合。"""
        cfg = self.config_
        surface = HeightMap(frame, cfg.extent, cfg.resolution, cfg.systematic_sigma)
        filtering = {'depth': {'input': 0, 'retained': 0, 'foreground': 0},
                     'lidar': {'input': 0, 'retained': 0, 'foreground': 0}}
        for view_index, snapshots in enumerate(groups):
            for sensor in ('depth', 'lidar'):
                aligned = [aligned_sensor(s, sensor, cfg.noise_scale) for s in snapshots]
                ranges = []
                for snapshot, (world, _, _) in zip(snapshots, aligned):
                    ranges.append(np.linalg.norm(world - snapshot[f'tf_{sensor}'][:3], axis=-1))
                sigmas = np.stack([a[2] for a in aligned])
                filtered, _ = temporal_filter(
                    np.stack(ranges), sigmas, min(cfg.temporal_support, len(snapshots)),
                    cfg.temporal_k, cfg.temporal_noise_floor)
                for index, (world, directions, sigma) in enumerate(aligned):
                    if cancel is not None and cancel():
                        raise InterruptedError('perception cancelled')
                    keep = np.isfinite(filtered[index])
                    stats = filtering[sensor]
                    stats['input'] += int(np.isfinite(ranges[index]).sum())
                    if reference is not None:
                        local = frame.to_local(world)
                        old_height = reference.reference_at(world)
                        projected_sigma = sigma * np.abs(directions @ frame.axes[:, 2])
                        foreground = foreground_mask(
                            local[..., 2], old_height,
                            np.sqrt(projected_sigma ** 2 + cfg.systematic_sigma ** 2),
                            cfg.foreground_gap, cfg.temporal_k)
                        stats['foreground'] += int((keep & foreground).sum())
                        keep &= ~foreground
                    stats['retained'] += int(keep.sum())
                    surface.integrate(world[keep], directions[keep], sigma[keep])
            if progress is not None:
                progress(view_index + 1, len(groups))
        return surface, filtering

    def evaluate(self, groups, progress=None, cancel=None):
        """替换当前轮次观测并评估；喷射后的第一轮观测必须走这里。"""
        if self.baseline_ is None:
            raise ValueError('a completed baseline is required before assessment')
        self.validate_groups(groups)
        self.current_groups_ = [list(snapshots) for snapshots in groups]
        return self._assess(self.current_groups_, progress, cancel)

    def observe(self, groups, progress=None, cancel=None):
        """把新观测并入当前轮次（补看）；尚无轮次时等价于 evaluate。"""
        if self.baseline_ is None:
            raise ValueError('a completed baseline is required before assessment')
        self.validate_groups(groups)
        if self.current_groups_ is None:
            return self.evaluate(groups, progress, cancel)
        combined = self.current_groups_ + [list(snapshots) for snapshots in groups]
        if len(combined) > self.config_.maximum_views:
            raise ValueError('accumulated viewpoints exceed budget')
        self.current_groups_ = combined
        return self._assess(self.current_groups_, progress, cancel)

    def _assess(self, groups, progress, cancel):
        """对给定观测组集合建图、对比基线并输出三分类；不读取真值。"""
        cfg = self.config_
        surface, filtering = self.build_map(groups, self.frame_, self.baseline_, progress, cancel)
        before = self.baseline_.state(cfg.minimum_observations, cfg.maximum_sigma)
        after = surface.state(cfg.minimum_observations, cfg.maximum_sigma)
        change = compare_surfaces(before, after, cfg.resolution ** 2,
                                  cfg.registration_sigma, cfg.confidence_k)
        fractions = circle_fractions(surface.centers_, cfg.resolution, cfg.diameter / 2)
        area = fractions * cfg.resolution ** 2
        assessment = assess(change, area, cfg.required_depth, cfg.required_fraction, cfg.confidence_k)
        report = {
            'assessment': assessment,
            'volume': volume_summary(change, area, cfg.coal_density),
            'external_alarm': external_change_alarm(
                change, fractions, cfg.resolution ** 2, cfg.external_retreat, cfg.confidence_k),
            'filtering': filtering, 'center_world': self.center_.tolist(),
            'normal_world': self.frame_.axes[:, 2].tolist(),
            'calibrated': False,
        }
        return {'report': report, 'baseline': before, 'current': after,
                'change': change, 'frame': self.frame_, 'surface': surface}
