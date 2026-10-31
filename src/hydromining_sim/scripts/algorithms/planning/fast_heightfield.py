"""Fast normal-incidence task environment for training and fidelity comparisons.

The height field approximates the level-set solver, not measured coal behavior.
Policy observations are synthesized measurements; true heights stay inside the
simulator and are used only to evolve geometry and independently score results.
"""
import gymnasium as gym
from gymnasium import spaces
import numpy as np
from scipy.ndimage import map_coordinates

from .. import heightfield_erosion
from ..erosion_model import DefaultErosionKernel
from ..jet_effects import jet_hydraulics
from ..perception.assessment import assess, circle_fractions
from ..perception.config import PerceptionConfig
from ..perception import visibility
from ..perception.wall_frame import WallFrame
from .decision import RulePolicy
from .observations import depth_progress, encode_observation, potential_reward


# Independent initial spray layouts for training and test; wall meshes are split too.
TRAIN_SCENARIOS = {
    'empty': {'shots': []},
    'center': {'shots': [((0., 0.), 8.)]},
    'horizontal_pair': {'shots': [((-.07, 0.), 8.), ((.07, 0.), 8.)]},
    'triangle': {'shots': [((-.07, -.04), 6.), ((.07, -.04), 6.), ((0., .07), 6.)]},
}
TEST_SCENARIOS = {
    'test_empty': {'shots': []},
    'test_offset': {'shots': [((.04, -.05), 7.)]},
    'test_vertical_pair': {'shots': [((0., -.08), 6.), ((0., .08), 6.)]},
    'test_triangle': {'shots': [((-.08, .04), 7.), ((.08, .04), 7.), ((0., -.08), 7.)]},
}


class FastHeightfieldEnv(gym.Env):
    """Gym environment for the 40 cm local task, with measured-state inputs."""

    metadata = {'render_modes': []}

    def __init__(self, initial_heights, frame, camera_poses, intrinsics, image_shape, *,
                 pressure, standoff, noise_scale=0., seed=0, max_steps=128,
                 scenario_names=None, scenarios, patch_extent=.72, diameter=.4,
                 required_depth=.05, required_fraction=.9, wall_thickness=.4):
        # pressure/standoff 必填，由 TaskSpec 提供；其余默认值属于任务形状，
        # 调用方应显式传入（1.3 统一为必填）。
        self.frame_ = frame
        heights = np.asarray(initial_heights, dtype=np.float64)
        self.initial_surfaces_ = heights[None, ...] if heights.ndim == 2 else heights
        self.initial_ = self.initial_surfaces_[0].copy()
        self.extent_, self.spacing_ = patch_extent, .005
        self.map_extent_ = .5
        self.map_spacing_, self.diameter_, self.required_depth_ = .01, diameter, required_depth
        self.required_fraction_ = required_fraction
        self.wall_thickness_ = wall_thickness
        self.pressure_, self.standoff_, self.noise_scale_ = pressure, standoff, noise_scale
        self.camera_poses_, self.K_, self.image_shape_ = camera_poses, intrinsics, image_shape
        self.max_steps_ = max_steps
        self.rng_ = np.random.default_rng(seed)
        self.rule_ = RulePolicy(PerceptionConfig(
            diameter=diameter, required_depth=required_depth,
            required_fraction=required_fraction), max_duration=8.)
        self.scenarios_ = scenarios
        self.scenario_names_ = list(self.scenarios_) if scenario_names is None else list(scenario_names)
        self.surface_grid_ = frame.grid(self.extent_, self.spacing_)
        self.map_grid_ = frame.grid(self.map_extent_, self.map_spacing_)
        self.area_ = circle_fractions(self.map_grid_, self.map_spacing_, self.diameter_/2) * self.map_spacing_**2
        self.kernel_ = DefaultErosionKernel()
        self.action_space = spaces.Box(-1., 1., shape=(3,), dtype=np.float32)
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(3756,), dtype=np.float32)
        self.reset(seed=seed)

    def _map_heights(self, heights):
        """Average the four 5 mm surface cells in each 1 cm observation cell."""
        start = round((self.extent_-self.map_extent_)/(2*self.spacing_))
        patch = heights[start:start+100, start:start+100]
        return patch.reshape(50, 2, 50, 2).mean(axis=(1, 3))

    def _visibility(self, heights):
        """Project the surface into the cameras and reject height-field occlusion."""
        grid = self.map_grid_.copy()
        grid[..., 2] = self._map_heights(heights)
        world = self.frame_.to_world(grid)
        visible = np.zeros(grid.shape[:2], bool)
        for pose in self.camera_poses_:
            inside = visibility.camera_inside(
                visibility.sensor_points(world, pose), self.K_, self.image_shape_)
            blocked = visibility.march_blocked(
                heights, self.frame_.to_local(visibility.pose_matrix(pose)[:3, 3]), grid,
                (.8, .9, .95, .98), self.spacing_, self.extent_, order=1, tolerance=.002)
            visible |= inside & ~blocked
        return visible

    def _spray(self, u, v, duration):
        """Integrate the shared radial kernel on the local surface."""
        return heightfield_erosion.spray_heightfield(
            self.heights_, self.surface_grid_, u, v, duration, self.pressure_,
            self.standoff_, self.kernel_, self.spacing_, self.extent_)

    def _observe(self):
        """Generate noisy depth measurements, preserving unknown cells."""
        visible = self.baseline_visible_ & self._visibility(self.heights_)
        if self.noise_scale_:
            visible &= self.rng_.random(visible.shape) >= min(.2, .02*self.noise_scale_)
        sigma = np.full(visible.shape, np.sqrt(2*.0015**2+.001**2))
        delta = self.baseline_measurement_ - self._map_heights(self.heights_)
        if self.noise_scale_:
            delta += self.rng_.normal(0., sigma*self.noise_scale_, delta.shape)
        change = {'delta': np.where(visible, delta, np.nan),
                  'sigma': np.where(visible, sigma, np.nan), 'valid': visible,
                  'significant': visible & (delta-1.645*sigma > 0)}
        state = {'coverage': np.where(visible, 2, 0).astype(np.int8)}
        assessment = assess(change, self.area_, self.required_depth_, self.required_fraction_, 1.645)
        self.last_change_, self.last_assessment_ = change, assessment
        self.last_progress_ = depth_progress(change, self.area_, self.required_depth_)
        return encode_observation(state, change, assessment, self.map_grid_, self.required_depth_,
                                  steps=self.steps_used_, max_steps=self.max_steps_,
                                  diameter=self.diameter_, pressure=self.pressure_, standoff=self.standoff_)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        if seed is not None:
            self.rng_ = np.random.default_rng(seed)
        mesh_index = (options or {}).get('mesh_index')
        if mesh_index is None:
            mesh_index = int(self.rng_.integers(len(self.initial_surfaces_)))
        self.initial_ = self.initial_surfaces_[mesh_index].copy()
        self.heights_ = self.initial_.copy()
        self.baseline_visible_ = self._visibility(self.heights_)
        self.baseline_measurement_ = self._map_heights(self.initial_)
        if self.noise_scale_:
            self.baseline_measurement_ = self.baseline_measurement_ + self.rng_.normal(
                0., .0015*self.noise_scale_, self.baseline_measurement_.shape)
        scenario = (options or {}).get('scenario')
        if scenario is None:
            scenario = self.rng_.choice(self.scenario_names_)
        for (u, v), duration in self.scenarios_[scenario]['shots']:
            self._spray(u, v, min(duration, 8.))
        self.steps_used_ = self.sprays_used_ = 0
        self.total_spray_duration_ = self.total_water_m3_ = self.total_energy_j_ = 0.
        observation = self._observe()
        self.last_observation_ = observation
        self.initially_achieved_ = self.last_assessment_['status'] == 'achieved'
        return observation, {'scenario': scenario, 'mesh_index': mesh_index,
                             'initially_achieved': self.initially_achieved_}

    def rule_action(self):
        """Existing maximum conservative deficit rule, expressed in normalized actions.

        The deficit formula is shared with RulePolicy.deficit_map; selection keeps
        the original argmax semantics (fall back to the first cell when no cell
        has a valid deficit) so the L0 baseline stays bit-identical.
        """
        deficit = self.rule_.deficit_map(self.last_change_, self.area_ > 0)
        index = np.nanargmax(deficit) if np.isfinite(deficit).any() else 0
        row, col = np.unravel_index(index, deficit.shape)
        u, v = self.map_grid_[row, col, :2]
        duration = np.clip(6*max(float(deficit[row, col]), 0.)/self.required_depth_, 2., 8.)
        return np.array([u/(self.diameter_/2), v/(self.diameter_/2), (duration-5.)/3.], dtype=np.float32)

    def step(self, action):
        if self.steps_used_ == 0 and self.initially_achieved_:
            truth_delta = self._map_heights(self.initial_-self.heights_)
            truth_fraction = float(self.area_[truth_delta >= self.required_depth_].sum()/self.area_.sum())
            return self.last_observation_, 0., True, False, {
                'verdict': 'achieved', 'truth': 'achieved' if truth_fraction >= self.required_fraction_ else 'not_achieved',
                'truth_fraction': truth_fraction, 'initially_achieved': True}
        u, v = np.clip(action[:2], -1., 1.)*(self.diameter_/2)
        radius = np.hypot(u, v)
        if radius > self.diameter_/2:
            u, v = np.array([u, v])*(self.diameter_/2)/radius
        duration = float(5. + 3.*np.clip(action[2], -1., 1.))
        previous = self.last_progress_
        before = self.heights_.copy()
        stats = self._spray(u, v, duration)
        self.steps_used_ += 1
        if np.any(self.initial_-self.heights_ >= self.wall_thickness_):
            self.heights_ = before
            truth_delta = self._map_heights(self.initial_-self.heights_)
            truth_fraction = float(self.area_[truth_delta >= self.required_depth_].sum()/self.area_.sum())
            return self.last_observation_, float(-5.-.3*duration), True, False, {
                'verdict': 'through_cut', 'truth': 'achieved' if truth_fraction >= self.required_fraction_ else 'not_achieved',
                'truth_fraction': truth_fraction}
        self.sprays_used_ += 1
        self.total_spray_duration_ += duration
        hydraulics = jet_hydraulics(self.pressure_)
        self.total_water_m3_ += hydraulics['flow_m3_s']*duration
        self.total_energy_j_ += hydraulics['hydraulic_power_w']*duration
        observation = self._observe()
        self.last_observation_ = observation
        achieved = self.last_assessment_['status'] == 'achieved'
        truncated = self.steps_used_ >= self.max_steps_ and not achieved
        reward = -.3*duration + potential_reward(previous, self.last_progress_, terminal=achieved)
        truth_delta = self._map_heights(self.initial_-self.heights_)
        truth_fraction = float(self.area_[truth_delta >= self.required_depth_].sum()/self.area_.sum())
        truth_achieved = truth_fraction >= self.required_fraction_
        if achieved:
            reward += 10. if truth_achieved else -5.
        info = {'verdict': self.last_assessment_['status'],
                'truth': 'achieved' if truth_achieved else 'not_achieved',
                'truth_fraction': truth_fraction, 'spray': {'u': float(u), 'v': float(v),
                'duration': duration, **stats}, 'water_m3': self.total_water_m3_,
                'energy_j': self.total_energy_j_}
        return observation, float(reward), achieved, truncated, info
