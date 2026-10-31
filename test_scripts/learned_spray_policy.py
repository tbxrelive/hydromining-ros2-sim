"""Real execution adapter: NBV/mode selection stays rule based, DDPG chooses spray."""
import json
from pathlib import Path
import numpy as np
from stable_baselines3 import DDPG, SAC, TD3
from algorithms.planning.ddpg_optimized import DemonstrationDDPG
from algorithms.planning.observations import encode_observation


class LearnedSprayPolicy:
    """Version-checked actor inference on measured state; no truth inputs."""

    def __init__(self, checkpoint, rule, config):
        path = Path(checkpoint)
        self.metadata_ = json.loads((path/'metadata.json').read_text())
        task = self.metadata_.get('task_parameters')
        if task:
            actual = (config.diameter, config.required_depth, config.required_fraction,
                      rule.pressure_, rule.standoff_)
            expected = tuple(task[k] for k in
                ('diameter', 'required_depth', 'required_fraction', 'pressure_mpa', 'standoff_m'))
            if not np.allclose(actual, expected):
                raise ValueError('Model task parameters do not match the active simulation')
        elif self.metadata_['observation_version'] == 'v3':
            raise ValueError('v3 model is missing its task parameters')
        elif self.metadata_['observation_version'] in ('v1', 'v2') and (
                not np.isclose(config.diameter, .2) or not np.isclose(config.required_depth, .02)):
            raise ValueError('Legacy model requires the original 20 cm / 2 cm task')
        self.max_steps_ = task['max_steps'] if task else 12
        self.algorithm_ = self.metadata_.get('algorithm')
        if self.algorithm_:
            cls = {'ddpg': DDPG, 'ddpg_demo': DemonstrationDDPG, 'td3': TD3, 'sac': SAC}[self.algorithm_]
        else:
            cls = DDPG if self.metadata_['variant'] == 'baseline' else DemonstrationDDPG
        kwargs = {'replay_buffer_kwargs': {'demo_path': None}} if cls is DemonstrationDDPG else {}
        self.model_ = cls.load(str(path/'checkpoint'), device='cpu', **kwargs)
        self.rule_, self.config_ = rule, config
        self.standoff_, self.pressure_ = rule.standoff_, rule.pressure_
        self.steps_ = 0

    def reset_episode(self):
        self.steps_ = 0
        self.rule_.reset_episode()

    def decide(self, state, assessment, change, surface, frame, view_poses, visibility, region_mask):
        action = self.rule_.decide(state, assessment, change, surface, frame, view_poses, visibility, region_mask)
        if action['type'] != 'spray':
            return action
        version = self.metadata_['observation_version']
        obs = encode_observation(state, change, assessment, surface.centers_,
                                 self.config_.required_depth, self.config_.confidence_k,
                                 self.steps_, self.max_steps_, version=version,
                                 diameter=self.config_.diameter, pressure=self.pressure_, standoff=self.standoff_)
        if obs.shape != self.model_.observation_space.shape:
            raise ValueError('Model observation schema mismatch')
        vector, _ = self.model_.predict(obs, deterministic=True)
        if vector.shape != (3,) or not np.isfinite(vector).all():
            raise ValueError('Invalid learned action')
        self.steps_ += 1
        uv = np.clip(vector[:2], -1, 1)*(self.config_.diameter/2 if version == 'v3' else .1)
        radius = np.linalg.norm(uv)
        if version == 'v3' and radius > self.config_.diameter/2:
            uv *= (self.config_.diameter/2)/radius
        duration = (float(5+3*np.clip(vector[2],-1,1)) if version == 'v3'
                    else min(float((np.clip(vector[2],-1,1)+1)*5+2),8.))
        name = self.algorithm_ or 'DDPG ' + self.metadata_['variant']
        action.update(uv=uv,duration=duration,rationale=f'{name}: target {uv.tolist()}, {duration:.2f}s')
        return action
