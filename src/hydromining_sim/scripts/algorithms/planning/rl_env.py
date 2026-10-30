"""阶段七 RL 环境（gymnasium）：与规则决策层同语义的 观测/喷射/收尾 动作。

两级保真度：L1 用真值高度+视点可见性+噪声直接建图（快，批量初训）；
L2 走完整感知管线（合成观测 -> PerceptionSession，微调/筛选）。
动作模式：
- 'full'：连续向量 [observe,spray,finish] argmax + 7 视点 argmax +
  (u,v,duration)——混合离散-连续编码，裸 DDPG 实测模式塌缩（critic
  学的是原始向量而非解码语义，Q 曲面平台化），仅供后续 P-DQN/混合策略；
- 'spray'：三维 (u,v,duration)，判决达标自动收尾——DDPG v1 交付口径，
  对应"自适应喷射"子问题，模式/视点选择由决策层负责。
真值仅用于奖励与回合标注，不进观测。
"""
import numpy as np

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError:  # 允许未装 gym 的环境导入本模块的非 Env 部分
    gym = None
    spaces = None

from .episodes import SCENARIOS, apply_scenario
from .surrogate import pose_to_matrix, spray_target_tcp
from ..perception.assessment import assess, circle_fractions
from ..perception.change_detection import (compare_surfaces,
                                           external_change_alarm, volume_summary)
from ..perception.height_map import HeightMap
from ..perception.pipeline import PerceptionSession
from ..perception.sensor_model import depth_sigma, lidar_sigma
from ..perception.wall_frame import WallFrame
from ..perception.metrics import truth_change

VIEW_NAMES = ['front_0p6', 'front_0p8', 'front_1p0', 'left_0p8', 'right_0p8',
              'up_0p8', 'down_0p8']
OBSERVE_SET = ['front_0p8', 'left_0p8', 'right_0p8']

# 动作向量布局：3 模式 + 7 视点 + (u,v,duration)
ACTION_MODES, N_VIEWS = 3, len(VIEW_NAMES)
ACTION_DIM = ACTION_MODES + N_VIEWS + 3
U_RANGE, V_RANGE, DURATION_RANGE = (-0.1, 0.1), (-0.1, 0.1), (2., 12.)
# L1 每视点融入帧数：与真实管线 frames_per_view=3 的计数语义一致，
# 否则单视点覆盖单元永远达不到 minimum_observations 而无法变可靠。
L1_FRAMES_PER_VIEW = 3


class HydrominingSurrogateEnv(gym.Env if gym else object):
    """固定评估区的续作任务；状态为区域内图层张量+标量。"""

    metadata = {'render_modes': []}

    def __init__(self, world, view_specs, cfg, fidelity='L2', noise_scale=0.,
                 max_steps=12, grid=25, cost_observe=2., cost_spray_per_s=0.3,
                 terminal_reward=10., external_penalty=5., seed=None,
                 action_mode='full'):
        if gym is None:
            raise ImportError('gymnasium is required')
        assert fidelity in ('L1', 'L2')
        assert action_mode in ('full', 'spray')
        self.action_mode_ = action_mode
        self.world_, self.specs_, self.cfg_ = world, view_specs, cfg
        self.fidelity_, self.noise_scale_ = fidelity, noise_scale
        self.max_steps_, self.grid_ = max_steps, grid
        self.cost_observe_, self.cost_spray_per_s_ = cost_observe, cost_spray_per_s
        self.terminal_reward_, self.external_penalty_ = terminal_reward, external_penalty
        self.rng_ = np.random.default_rng(seed)
        # 观测：grid x grid x 4 图层(delta,sigma,coverage,significant) + 4 标量
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(grid * grid * 4 + 4,), dtype=np.float32)
        if action_mode == 'spray':
            # 纯连续子问题：DDPG 只学自适应喷射参数 (u, v, duration)；
            # 模式/视点选择由决策层负责（裸 DDPG 对 argmax 编码的混合
            # 动作拿不到模式梯度，会塌缩——已实测）。
            self.action_space = spaces.Box(low=-1., high=1., shape=(3,),
                                           dtype=np.float32)
        else:
            self.action_space = spaces.Box(low=-1., high=1.,
                                           shape=(ACTION_DIM,), dtype=np.float32)
        self.center_ = np.array(cfg['region_center_world'])
        self.normal_ = np.array(cfg['wall_normal_world'])
        self.perception_ = __import__(
            'algorithms.perception.config', fromlist=['PerceptionConfig']).PerceptionConfig()
        # L1 工况缓存：确定性蚀除 + 固定参考系 -> 预喷结果可整局复用，
        # 省去每回合重复求解（reset 从 ~10-30s 降到 <1s）。
        self._episode_cache = {}
        self._reset_state()

    def _reset_state(self):
        self.session_ = None
        self.last_fraction_lower_ = 0.
        self.last_coverage_ = 0.
        self.steps_used_ = self.sprays_used_ = 0
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
        layers = np.stack([
            np.nan_to_num(change['delta'], nan=0.),
            np.nan_to_num(change['sigma'], nan=1.),
            state['coverage'].astype(float) / 2.,
            change['significant'].astype(float)], axis=-1)
        # 中心裁剪任务区附近 grid x grid（分辨率自适应）
        size = layers.shape[0]
        center = size // 2
        half = min(self.grid_ // 2, center)
        patch = layers[center - half:center + half, center - half:center + half]
        from scipy.ndimage import zoom
        if patch.shape[:2] != (self.grid_, self.grid_):
            factors = (self.grid_ / patch.shape[0], self.grid_ / patch.shape[1], 1)
            patch = zoom(patch, factors, order=1)
        a = evaluated['report']['assessment']
        scalars = np.array([a['fraction_lower'], a['fraction_upper'], a['coverage'],
                            self.steps_used_ / self.max_steps_], dtype=np.float32)
        return np.concatenate([patch.reshape(-1).astype(np.float32), scalars])

    def _evaluate(self, names):
        evaluated = self.session_.evaluate(self._observe_snapshots(names, stamp=100))
        return evaluated

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
        """该视点任一传感器可观测的单元掩码（融合语义：相机 ∪ LiDAR）。

        按世界版本缓存：喷射改变几何后自动失效，观测动作不重复求交。
        """
        key = (self.world_.version_, view)
        if key not in self.mask_cache_:
            mask = np.zeros(world.shape[:2], bool)
            for sensor in ('depth', 'lidar'):
                mask |= self._l1_sensor_mask(view, world, sensor)
            self.mask_cache_[key] = mask
        return self.mask_cache_[key]

    def _l1_sensor_mask(self, view, world, sensor):
        """单传感器对网格表面点的可见掩码（视场 + 遮挡）。"""
        key = (self.world_.version_, view, sensor)
        if key in self._sensor_mask_cache_:
            return self._sensor_mask_cache_[key]
        spec = self.specs_[view]
        pose = pose_to_matrix(spec[f'tf_{sensor}'])
        origin = pose[:3, 3]
        flat = world.reshape(-1, 3)
        local = (flat - origin) @ pose[:3, :3]
        distance = np.linalg.norm(local, axis=1)
        if sensor == 'depth':
            K = spec['K']
            height, width = spec['image_shape']
            with np.errstate(divide='ignore', invalid='ignore'):
                col = K[0, 2] - K[0, 0] * local[:, 1] / local[:, 0]
                row = K[1, 2] - K[1, 1] * local[:, 2] / local[:, 0]
            fov = ((local[:, 0] > 0) & (col >= 0) & (col < width) &
                   (row >= 0) & (row < height))
        else:
            # VLP-16 类 360° 方位、±15° 俯仰带（与 render_lidar 同约定）。
            elevation = np.arctan2(local[:, 2],
                                   np.linalg.norm(local[:, :2], axis=1))
            fov = np.abs(elevation) <= np.radians(15)
        with np.errstate(divide='ignore', invalid='ignore'):
            directions = local / np.maximum(distance[:, None], 1e-9)
        hits = self.world_.cast(np.broadcast_to(origin, flat.shape), directions)
        tolerance = max(3 * self.perception_.resolution, 0.02)
        mask = (fov & np.isfinite(hits) &
                (np.abs(hits - distance) <= tolerance))
        mask = mask.reshape(world.shape[:2])
        self._sensor_mask_cache_[key] = mask
        return mask

    def _l1_build_map(self, views, surface=None, masks=None):
        """把各视点覆盖的表面点带噪融入一张 HeightMap。

        每视点每格一个观测：count = 覆盖视点数，精度自然叠加；σ 按
        depth_sigma(距离, cfg.noise_scale) 上报，注入噪声用 env 的
        noise_scale（0 即确定性几何，与 L2 的干净渲染一致）。
        surface/masks 可显式传入（基线用预喷前的缓存几何）。
        """
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
            return self.session_.observe(self._observe_snapshots([view],
                                                               stamp=stamp))
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
        super().reset(seed=seed)
        self._reset_state()
        name = (options or {}).get('scenario') or self.rng_.choice(list(SCENARIOS))
        scenario = SCENARIOS[name]
        # 真值基线必须在工况预喷之前取，否则已达标工况的收尾判决会被
        # 误记为 not_achieved（预喷量被排除在终局对比之外）。
        if self.fidelity_ == 'L2':
            self.world_.reset()
            self.session_ = PerceptionSession(self.center_, self.perception_)
            self.session_.set_baseline(self._observe_snapshots(OBSERVE_SET,
                                                             stamp=0))
            self.frame_ = self.session_.frame_
            self.pristine_ = self.world_.truth_heights(self.frame_, .24, .002)
            apply_scenario(self.world_, self.frame_, -self.normal_,
                           scenario['shots'])
        else:
            self.session_ = None
            self.frame_ = WallFrame.from_normal(self.center_, self.normal_)
            self.l1_views_ = list(OBSERVE_SET)
            entry = self._episode_cache.get(name)
            if entry is None:
                self.world_.reset()
                surface = self._l1_surface()
                # 预喷前几何的传感器掩码：基线建图与缓存复用都必须用它，
                # 缓存命中时世界已是预喷后状态，不可对当前场景重算。
                masks = {v: {s: self._l1_sensor_mask(v, surface[1], s)
                             for s in ('depth', 'lidar')}
                         for v in OBSERVE_SET}
                baseline = self._l1_build_map(self.l1_views_, surface=surface,
                                              masks=masks)
                self.pristine_ = self.world_.truth_heights(self.frame_, .24,
                                                           .002)
                apply_scenario(self.world_, self.frame_, -self.normal_,
                               scenario['shots'])
                self._episode_cache[name] = {
                    'phi': self.world_.snapshot_phi(), 'surface': surface,
                    'masks': masks, 'pristine': self.pristine_}
            else:
                self.world_.restore(entry['phi'])
                self.pristine_ = entry['pristine']
                baseline = self._l1_build_map(self.l1_views_,
                                              surface=entry['surface'],
                                              masks=entry['masks'])
            self.l1_baseline_state_ = baseline.state(
                self.perception_.minimum_observations,
                self.perception_.maximum_sigma)
        self.scenario_name_ = name
        evaluated = self._do_evaluate(stamp=100)
        a = evaluated['report']['assessment']
        self.last_fraction_lower_, self.last_coverage_ = a['fraction_lower'], a['coverage']
        self.last_evaluated_ = evaluated
        return self._state_tensor(evaluated), {'scenario': name}

    def _decode_spray(self, action):
        """动作向量 -> (u, v, duration)，full/spray 两模式共用。"""
        if self.action_mode_ == 'spray':
            ua, va, da = np.asarray(action, float)[:3]
        else:
            ua, va, da = np.asarray(action, float)[
                ACTION_MODES + N_VIEWS:ACTION_MODES + N_VIEWS + 3]
        u = float(np.clip(ua, -1, 1) * U_RANGE[1])
        v = float(np.clip(va, -1, 1) * V_RANGE[1])
        duration = float(np.clip(da, -1, 1) * .5 + .5)
        duration = duration * (DURATION_RANGE[1] - DURATION_RANGE[0]) + DURATION_RANGE[0]
        return u, v, duration

    def _finish(self, a, reward, truncated=False):
        """终局核对与终局奖励；返回 (obs, reward, terminated, truncated, info)。"""
        frame = self.frame_
        truth = truth_change(self.pristine_,
                             self.world_.truth_heights(frame, .24, .002),
                             frame, .24, .002, .1, .02, .9)
        # 作业语义：判决+真值双达标才拿正奖励；虚报达标最重罚；其余
        # 按置信下界缺口比例罚——尽力推进仍优于直接放弃。
        success = truth['status'] == 'achieved' and a['status'] == 'achieved'
        if success:
            reward += self.terminal_reward_
        elif a['status'] == 'achieved':
            reward -= self.terminal_reward_
        else:
            reward -= self.terminal_reward_ * (1. - a['fraction_lower'])
        info = {'mode': -1, 'verdict': a['status'], 'truth': truth['status']}
        return (self._state_tensor(self.last_evaluated_), float(reward),
                not truncated, truncated, info)

    def _step_spray(self, action):
        """spray 模式一步：喷射 -> 重评估；判决达标即自动收尾（同规则层）。"""
        a_prev = self.last_evaluated_['report']['assessment']
        if a_prev['status'] == 'achieved':
            return self._finish(a_prev, 0.)
        u, v, duration = self._decode_spray(action)
        tcp = spray_target_tcp(self.frame_, np.array([u, v]), -self.normal_, 0.7)
        stats = self.world_.spray_tcp(tcp, 20., duration)
        self.sprays_used_ += 1
        evaluated = self._do_evaluate(stamp=100 + self.steps_used_)
        a = evaluated['report']['assessment']
        reward = -self.cost_spray_per_s_ * duration
        reward += (a['fraction_lower'] - self.last_fraction_lower_) * 8.
        reward += (a['coverage'] - self.last_coverage_) * 2.
        self.last_fraction_lower_, self.last_coverage_ = \
            a['fraction_lower'], a['coverage']
        self.last_evaluated_ = evaluated
        self.steps_used_ += 1
        self.last_info_ = {'mode': 1, 'spray': {
            'u': u, 'v': v, 'duration': duration,
            'volume': stats['volume_removed_m3']}}
        if a['status'] == 'achieved' or self.steps_used_ >= self.max_steps_:
            return self._finish(a, reward,
                                truncated=self.steps_used_ >= self.max_steps_)
        obs = self._state_tensor(evaluated)
        return obs, float(reward), False, False, self.last_info_

    def step(self, action):
        action = np.asarray(action, float)
        if self.action_mode_ == 'spray':
            return self._step_spray(action)
        mode = int(np.argmax(action[:ACTION_MODES]))
        reward, terminated, truncated, info = 0., False, False, {'mode': mode}
        frame = self.frame_
        if mode == 0:  # observe
            view = VIEW_NAMES[int(np.argmax(action[ACTION_MODES:ACTION_MODES + N_VIEWS]))]
            evaluated = self._do_observe(view, stamp=200 + self.steps_used_)
            reward -= self.cost_observe_
            info['view'] = view
        elif mode == 1:  # spray
            u, v, duration = self._decode_spray(action)
            tcp = spray_target_tcp(frame, np.array([u, v]), -self.normal_, 0.7)
            stats = self.world_.spray_tcp(tcp, 20., duration)
            reward -= self.cost_spray_per_s_ * duration
            self.sprays_used_ += 1
            info['spray'] = {'u': float(u), 'v': float(v), 'duration': duration,
                             'volume': stats['volume_removed_m3']}
            evaluated = self._do_evaluate(stamp=100 + self.steps_used_)
        else:  # finish
            evaluated = self.last_evaluated_
            terminated = True
        a = evaluated['report']['assessment']
        reward += (a['fraction_lower'] - self.last_fraction_lower_) * 8.
        reward += (a['coverage'] - self.last_coverage_) * 2.
        self.last_fraction_lower_, self.last_coverage_ = a['fraction_lower'], a['coverage']
        self.last_evaluated_ = evaluated
        self.steps_used_ += 1
        if self.steps_used_ >= self.max_steps_:
            truncated = True
        if terminated or truncated:
            obs, reward, ter, tru, finfo = self._finish(a, reward, truncated)
            finfo.update(info)
            return obs, reward, ter, tru, finfo
        return self._state_tensor(evaluated), float(reward), terminated, truncated, info
