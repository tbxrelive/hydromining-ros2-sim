"""阶段七 RL 环境（gymnasium）：自适应喷射动作 (u, v, duration)。

动作空间是三维连续量，判决达标即自动收尾（与规则层同语义）；观测补看由
决策层负责。观测编码只保留 v3（任务区域网格层 + 压力/靶距标量），奖励只
保留势函数形式。两级保真度：L1 用真值高度+视点可见性+噪声直接建图（快，
批量初训）；L2 走完整感知管线（合成观测 -> PerceptionSession）。
真值仅用于奖励与回合标注，不进观测。
"""
import time

import numpy as np

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError:  # 允许未装 gym 的环境导入本模块的非 Env 部分
    gym = None
    spaces = None

from .observations import depth_progress, encode_observation, potential_reward
from .episodes import apply_scenario
from .surrogate import pose_to_matrix, spray_target_tcp
from ..perception import visibility
from ..perception.assessment import assess, circle_fractions
from ..perception.change_detection import (compare_surfaces,
                                           external_change_alarm, volume_summary)
from ..perception.config import PerceptionConfig
from ..perception.height_map import HeightMap
from ..perception.metrics import truth_change
from ..perception.pipeline import PerceptionSession
from ..perception.sensor_model import depth_sigma, lidar_sigma
from ..perception.wall_frame import WallFrame

OBSERVE_SET = ['front_0p8', 'left_0p8', 'right_0p8']
DURATION_RANGE = (2., 8.)
# L1 每视点融入帧数：与真实管线 frames_per_view=3 的计数语义一致，
# 否则单视点覆盖单元永远达不到 minimum_observations 而无法变可靠。
L1_FRAMES_PER_VIEW = 3


class HydrominingSurrogateEnv(gym.Env if gym else object):
    """固定评估区的续作任务；状态为区域内图层张量+标量。"""

    metadata = {'render_modes': []}

    def __init__(self, world, view_specs, task_config, fidelity='L2', noise_scale=0.,
                 max_steps=12, grid=25, cost_observe=2., cost_spray_per_s=0.3,
                 terminal_reward=10., external_penalty=5., seed=None,
                 cache_dir=None, scenario_names=None, scenarios=None):
        if gym is None:
            raise ImportError('gymnasium is required')
        assert fidelity in ('L1', 'L2')
        if not isinstance(task_config, dict) or 'pressure_mpa' not in task_config:
            raise ValueError('task_config is required')
        if scenarios is None:
            raise ValueError('scenarios are required')
        self.scenarios_ = scenarios
        self.scenario_names_ = list(self.scenarios_) if scenario_names is None else list(scenario_names)
        if not self.scenario_names_ or any(n not in self.scenarios_ for n in self.scenario_names_):
            raise ValueError('invalid training scenarios')
        self.pressure_ = task_config['pressure_mpa']
        self.standoff_ = task_config['standoff_m']
        self.duration_range_ = DURATION_RANGE
        self.action_radius_ = task_config['diameter']/2
        self.truth_extent_, self.truth_resolution_ = (.5, .005)
        self.cache_dir_ = cache_dir
        self.profile_ = {'reset_s': 0., 'spray_s': 0., 'assessment_s': 0.}
        self.world_, self.specs_, self.cfg_ = world, view_specs, task_config
        self.fidelity_, self.noise_scale_ = fidelity, noise_scale
        self.max_steps_, self.grid_ = max_steps, grid
        self.cost_observe_, self.cost_spray_per_s_ = cost_observe, cost_spray_per_s
        self.terminal_reward_, self.external_penalty_ = terminal_reward, external_penalty
        self.rng_ = np.random.default_rng(seed)
        # 观测：grid x grid x 6 图层 + 6 标量（含压力/靶距）。
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(grid * grid * 6 + 6,), dtype=np.float32)
        # 纯连续子问题：只学自适应喷射参数 (u, v, duration)；
        # 模式/视点选择由决策层负责（裸 DDPG 对 argmax 编码的混合动作
        # 拿不到模式梯度，会塌缩——已实测）。
        self.action_space = spaces.Box(low=-1., high=1., shape=(3,), dtype=np.float32)
        self.center_ = np.array(task_config['region_center_world'])
        self.normal_ = np.array(task_config['wall_normal_world'])
        self.perception_ = PerceptionConfig(
            diameter=task_config['diameter'], required_depth=task_config['required_depth'],
            required_fraction=task_config['required_fraction'])
        # L1 工况缓存：确定性蚀除 + 固定参考系 -> 预喷结果可整局复用，
        # 省去每回合重复求解（reset 从 ~10-30s 降到 <1s）。
        self._episode_cache = {}
        self._reset_state()

    def _reset_state(self):
        self.session_ = None
        self.last_fraction_lower_ = 0.
        self.last_coverage_ = 0.
        self.steps_used_ = self.sprays_used_ = 0
        self.total_spray_duration_ = 0.
        self.last_info_ = {}
        self.pristine_ = None
        self.l1_views_ = []
        self.l1_baseline_state_ = None
        self.mask_cache_ = {}
        self._sensor_mask_cache_ = {}

    def _observe_snapshots(self, names, frames=3, stamp=0):
        return [[self.world_.snapshot(self.specs_[n]['tf_depth'],
                                      self.specs_[n]['tf_lidar'], self.specs_[n]['K'],
                                      self.specs_[n]['image_shape'], stamp + 10 * i + f,
                                      self.rng_, noise_scale=self.noise_scale_)
                 for f in range(frames)] for i, n in enumerate(names)]

    def _state_tensor(self, evaluated):
        """评估结果 -> 固定尺寸张量；未观测单元填哨兵值。"""
        surface = evaluated['surface']
        state = surface.state(self.perception_.minimum_observations,
                              self.perception_.maximum_sigma)
        change = evaluated['change']
        return encode_observation(state, change, evaluated['report']['assessment'],
                                  surface.centers_, self.perception_.required_depth,
                                  confidence_k=self.perception_.confidence_k,
                                  steps=self.steps_used_, max_steps=self.max_steps_,
                                  grid=self.grid_, diameter=self.perception_.diameter,
                                  pressure=self.pressure_, standoff=self.standoff_)

    def _evaluate(self, names):
        return self.session_.evaluate(self._observe_snapshots(names, stamp=100))

    # ---------- L1 快速通道：真值表面 + 视点可见性 + σ 模型 -> 同构评估 ----------
    # 与 L2 的边界一致：真值只经可见性掩码和注入噪声进入地图，σ 仍由传感器
    # 模型按距离给出；判决仍走 compare_surfaces/assess 同一套三分类。

    def _l1_surface(self):
        """当前世界的任务区表面点（local + world）；z=NaN 表示无表面。"""
        cfg = self.perception_
        local = self.frame_.grid(cfg.extent, cfg.resolution)
        local[..., 2] = self.world_.truth_heights(self.frame_, cfg.extent,
                                                  cfg.resolution)
        return local, self.frame_.to_world(local)

    def _l1_visibility(self, view, world):
        """该视点任一传感器可观测的单元掩码（融合语义：相机 ∪ LiDAR）。"""
        key = (self.world_.version_, view)
        if key not in self.mask_cache_:
            mask = np.zeros(world.shape[:2], bool)
            for sensor in ('depth', 'lidar'):
                mask |= self._l1_sensor_mask(view, world, sensor)
            self.mask_cache_[key] = mask
        return self.mask_cache_[key]

    def _l1_sensor_mask(self, view, world, sensor):
        """单传感器对网格表面点的可见掩码（视场 + 遮挡，逐点射线求交）。"""
        key = (self.world_.version_, view, sensor)
        if key in self._sensor_mask_cache_:
            return self._sensor_mask_cache_[key]
        spec = self.specs_[view]
        pose = pose_to_matrix(spec[f'tf_{sensor}'])
        origin = pose[:3, 3]
        flat = world.reshape(-1, 3)
        local = visibility.sensor_points(flat, pose)
        distance = np.linalg.norm(local, axis=1)
        if sensor == 'depth':
            fov = visibility.camera_inside(local, spec['K'], spec['image_shape'])
        else:
            fov = visibility.lidar_inside(local)
        with np.errstate(divide='ignore', invalid='ignore'):
            directions = (flat-origin) / np.maximum(distance[:, None], 1e-9)
        hits = self.world_.cast(np.broadcast_to(origin, flat.shape), directions)
        tolerance = max(3 * self.perception_.resolution, 0.02)
        mask = (fov & np.isfinite(hits) &
                (np.abs(hits - distance) <= tolerance))
        mask = mask.reshape(world.shape[:2])
        self._sensor_mask_cache_[key] = mask
        return mask

    def _l1_build_map(self, views, surface=None, masks=None):
        """把各视点覆盖的表面点带噪融入一张 HeightMap。"""
        cfg = self.perception_
        surface_map = HeightMap(self.frame_, cfg.extent, cfg.resolution,
                                cfg.systematic_sigma)
        if surface is None:
            surface = self._l1_surface()
        local, world = surface
        for view in views:
            for sensor in ('depth', 'lidar'):
                mask = (masks[view][sensor] if masks is not None
                        else self._l1_sensor_mask(view, world, sensor))
                mask = mask & np.isfinite(local[..., 2])
                if not mask.any():
                    continue
                origin = pose_to_matrix(
                    self.specs_[view][f'tf_{sensor}'])[:3, 3]
                delta = world[mask] - origin
                ranges = np.linalg.norm(delta, axis=1)
                directions = delta / ranges[:, None]
                sigma_fn = depth_sigma if sensor == 'depth' else lidar_sigma
                sigma = sigma_fn(ranges, cfg.noise_scale)
                for _ in range(L1_FRAMES_PER_VIEW):
                    points = world[mask].copy()
                    if self.noise_scale_ > 0:
                        noise = self.rng_.normal(
                            0., sigma_fn(ranges, self.noise_scale_))
                        points = points + directions * noise[:, None]
                    surface_map.integrate(points, directions, sigma)
        return surface_map

    def _l1_assess(self):
        """L1 评估：与 PerceptionSession._assess 同构的返回。"""
        cfg = self.perception_
        surface = self._l1_build_map(self.l1_views_)
        after = surface.state(cfg.minimum_observations, cfg.maximum_sigma)
        change = compare_surfaces(self.l1_baseline_state_, after,
                                  cfg.resolution ** 2, cfg.registration_sigma,
                                  cfg.confidence_k)
        fractions = circle_fractions(surface.centers_, cfg.resolution,
                                     cfg.diameter / 2)
        area = fractions * cfg.resolution ** 2
        assessment = assess(change, area, cfg.required_depth,
                            cfg.required_fraction, cfg.confidence_k)
        report = {
            'assessment': assessment,
            'volume': volume_summary(change, area, cfg.coal_density),
            'external_alarm': external_change_alarm(
                change, fractions, cfg.resolution ** 2, cfg.external_retreat,
                cfg.confidence_k),
            'filtering': {'mode': 'L1'},
            'center_world': self.center_.tolist(),
            'normal_world': self.frame_.axes[:, 2].tolist(),
            'calibrated': False,
        }
        return {'report': report, 'baseline': self.l1_baseline_state_,
                'current': after, 'change': change, 'frame': self.frame_,
                'surface': surface}

    def _do_observe(self, view, stamp):
        """observe 动作：L2 走会话补看，L1 把视点并入当前轮次重建。"""
        if self.fidelity_ == 'L2':
            return self.session_.observe(self._observe_snapshots([view], stamp=stamp))
        if view not in self.l1_views_:
            if len(self.l1_views_) >= self.perception_.maximum_views:
                raise ValueError('accumulated viewpoints exceed budget')
            self.l1_views_.append(view)
        return self._l1_assess()

    def _do_evaluate(self, stamp):
        """喷后新一轮评估：L2 全管线，L1 重置轮次视点集后重建。"""
        if self.fidelity_ == 'L2':
            return self._evaluate(OBSERVE_SET)
        self.l1_views_ = list(OBSERVE_SET)
        return self._l1_assess()

    def reset(self, seed=None, options=None):
        started = time.monotonic()
        super().reset(seed=seed)
        if seed is not None:
            self.rng_ = np.random.default_rng(seed)
        self._reset_state()
        name = (options or {}).get('scenario') or self.rng_.choice(self.scenario_names_)
        scenario = self.scenarios_[name]
        # 真值基线必须在工况预喷之前取，否则已达标工况的收尾判决会被
        # 误记为 not_achieved（预喷量被排除在终局对比之外）。
        if self.fidelity_ == 'L2':
            self.world_.reset()
            self.session_ = PerceptionSession(self.center_, self.perception_)
            self.session_.set_baseline(self._observe_snapshots(OBSERVE_SET, stamp=0))
            self.frame_ = self.session_.frame_
            self.pristine_ = self.world_.truth_heights(self.frame_, self.truth_extent_, self.truth_resolution_)
            apply_scenario(self.world_, self.frame_, -self.normal_, scenario['shots'],
                           self.standoff_, self.pressure_, self.duration_range_[1])
        else:
            self.session_ = None
            self.frame_ = WallFrame.from_normal(self.center_, self.normal_)
            self.l1_views_ = list(OBSERVE_SET)
            entry = self._episode_cache.get(name)
            if entry is None and self.cache_dir_:
                from .scenario_cache import load_entry
                entry = load_entry(self, name)
                if entry is not None:
                    self._episode_cache[name] = entry
            if entry is None:
                self.world_.reset()
                surface = self._l1_surface()
                # 预喷前几何的传感器掩码：基线建图与缓存复用都必须用它，
                # 缓存命中时世界已是预喷后状态，不可对当前场景重算。
                masks = {v: {s: self._l1_sensor_mask(v, surface[1], s)
                             for s in ('depth', 'lidar')}
                         for v in OBSERVE_SET}
                baseline = self._l1_build_map(self.l1_views_, surface=surface, masks=masks)
                self.pristine_ = self.world_.truth_heights(self.frame_, self.truth_extent_, self.truth_resolution_)
                apply_scenario(self.world_, self.frame_, -self.normal_, scenario['shots'],
                               self.standoff_, self.pressure_, self.duration_range_[1])
                self._episode_cache[name] = {
                    'phi': self.world_.snapshot_phi(), 'surface': surface,
                    'masks': masks, 'pristine': self.pristine_}
                if self.cache_dir_:
                    from .scenario_cache import save_entry
                    save_entry(self, name, self._episode_cache[name])
            else:
                self.world_.restore(entry['phi'])
                self.pristine_ = entry['pristine']
                baseline = self._l1_build_map(self.l1_views_, surface=entry['surface'],
                                              masks=entry['masks'])
            self.l1_baseline_state_ = baseline.state(
                self.perception_.minimum_observations,
                self.perception_.maximum_sigma)
        self.scenario_name_ = name
        evaluated = self._do_evaluate(stamp=100)
        a = evaluated['report']['assessment']
        self.last_fraction_lower_, self.last_coverage_ = a['fraction_lower'], a['coverage']
        self.last_evaluated_ = evaluated
        weights = circle_fractions(evaluated['surface'].centers_, self.perception_.resolution,
                                   self.perception_.diameter/2)
        self.last_progress_ = depth_progress(evaluated['change'], weights,
                                             self.perception_.required_depth,
                                             self.perception_.confidence_k)
        self.profile_['reset_s'] += time.monotonic() - started
        return self._state_tensor(evaluated), {'scenario': name,
            'initially_achieved': a['status'] == 'achieved'}

    def _decode_spray(self, action):
        """动作向量 -> (u, v, duration)。"""
        ua, va, da = np.asarray(action, float)[:3]
        u = float(np.clip(ua, -1, 1) * self.action_radius_)
        v = float(np.clip(va, -1, 1) * self.action_radius_)
        radius = np.hypot(u, v)
        if radius > self.action_radius_:
            u, v = np.array([u, v])*self.action_radius_/radius
        duration = float(np.clip(da, -1, 1) * .5 + .5)
        duration = duration * (self.duration_range_[1] - self.duration_range_[0]) + self.duration_range_[0]
        return u, v, duration

    def _finish(self, a, reward, truncated=False, allow_missing_truth=False):
        """终局核对与终局奖励；返回 (obs, reward, terminated, truncated, info)。"""
        frame = self.frame_
        truth = truth_change(self.pristine_,
                             self.world_.truth_heights(frame, self.truth_extent_, self.truth_resolution_),
                             frame, self.truth_extent_, self.truth_resolution_, self.perception_.diameter/2,
                             self.perception_.required_depth, self.perception_.required_fraction,
                             allow_missing=allow_missing_truth)
        # 作业语义：判决达标才拿终局奖励；虚报达标重罚。
        success = truth['status'] == 'achieved' and a['status'] == 'achieved'
        if success:
            reward += self.terminal_reward_
        elif a['status'] == 'achieved':
            reward -= 5.
        info = {'mode': -1, 'verdict': a['status'], 'truth': truth['status'],
                'truth_fraction': truth['fraction_ge_threshold'], 'removed_volume_m3': truth['removed_volume_m3']}
        if truth['status'] == 'unknown':
            info['truth_valid_fraction'] = truth['valid_fraction']
        return (self._state_tensor(self.last_evaluated_), float(reward),
                not truncated, truncated, info)

    def step(self, action):
        """spray 模式一步：喷射 -> 重评估；判决达标即自动收尾（同规则层）。"""
        a_prev = self.last_evaluated_['report']['assessment']
        if a_prev['status'] == 'achieved':
            return self._finish(a_prev, 0.)
        u, v, duration = self._decode_spray(action)
        tcp = spray_target_tcp(self.frame_, np.array([u, v]), -self.normal_, self.standoff_)
        started = time.monotonic()
        try:
            stats = self.world_.spray_tcp(tcp, self.pressure_, duration)
        except ValueError as error:
            if 'Through-cut detected' not in str(error):
                raise
            self.steps_used_ += 1
            obs, reward, terminated, truncated, info = self._finish(
                a_prev, -5.-self.cost_spray_per_s_*duration, allow_missing_truth=True)
            info['verdict'] = 'through_cut'
            return obs, reward, terminated, truncated, info
        self.profile_['spray_s'] += time.monotonic() - started
        self.sprays_used_ += 1
        self.total_spray_duration_ += duration
        started = time.monotonic()
        evaluated = self._do_evaluate(stamp=100 + self.steps_used_)
        self.profile_['assessment_s'] += time.monotonic() - started
        a = evaluated['report']['assessment']
        reward = -self.cost_spray_per_s_ * duration
        weights = circle_fractions(evaluated['surface'].centers_, self.perception_.resolution,
                                   self.perception_.diameter/2)
        progress = depth_progress(evaluated['change'], weights, self.perception_.required_depth,
                                  self.perception_.confidence_k)
        reward += potential_reward(self.last_progress_, progress, terminal=a['status'] == 'achieved')
        self.last_progress_ = progress
        self.last_fraction_lower_, self.last_coverage_ = a['fraction_lower'], a['coverage']
        self.last_evaluated_ = evaluated
        self.steps_used_ += 1
        self.last_info_ = {'mode': 1, 'spray': {
            'u': u, 'v': v, 'duration': duration,
            'volume': stats['volume_removed_m3']}}
        if a['status'] == 'achieved' or self.steps_used_ >= self.max_steps_:
            return self._finish(a, reward,
                                truncated=self.steps_used_ >= self.max_steps_ and a['status'] != 'achieved')
        return self._state_tensor(evaluated), float(reward), False, False, self.last_info_
