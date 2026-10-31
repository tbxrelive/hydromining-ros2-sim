"""训练检查点回调：保存学习器/回放/RNG，中断后可从回合边界恢复。

从 phase7_train_optimized.py 迁出，供大任务训练脚本共用。
"""
import json
import os
from pathlib import Path
import time

import numpy as np
import torch
from stable_baselines3.common.callbacks import BaseCallback


class CheckpointCallback(BaseCallback):
    """Persist learner/replay/RNG; interrupted rollouts restart at episode boundaries."""

    def __init__(self, output, config, initial_noise=.2, final_noise=.05):
        super().__init__()
        self.output, self.config = Path(output), config
        self.initial_noise, self.final_noise = initial_noise, final_noise
        self.last_saved = 0
        self.last_logged = 0
        self.started = time.monotonic()
        self.prior_wall_s = config.get('prior_wall_s', 0.)

    def save(self):
        self.output.mkdir(parents=True, exist_ok=True)
        snapshots = self.output/'snapshots'; snapshots.mkdir(exist_ok=True)
        generation = f'step_{self.model.num_timesteps}_{time.time_ns()}'
        temporary = snapshots/('.'+generation); temporary.mkdir()
        self.model.save(str(temporary/'checkpoint'))
        self.model.save_replay_buffer(str(temporary/'replay.pkl'))
        torch.save({'numpy':np.random.get_state(),'torch':torch.get_rng_state(),
                    'cuda':torch.cuda.get_rng_state_all() if torch.cuda.is_available() else None}, temporary/'rng.pt')
        metadata = {**self.config,'num_timesteps':self.model.num_timesteps,
                    'wall_s':self.prior_wall_s+time.monotonic()-self.started,
                    'resume_semantics':'learner/replay/RNG restored; environments restart episodes'}
        (temporary/'metadata.json').write_text(json.dumps(metadata,indent=2))
        destination = snapshots/generation
        os.replace(temporary,destination)
        pending = self.output/'.latest_pending'
        pending.unlink(missing_ok=True)
        pending.symlink_to(Path('snapshots')/generation)
        os.replace(pending,self.output/'latest')
        for name in ('checkpoint.zip','replay.pkl','rng.pt','metadata.json'):
            alias = self.output/name
            if not alias.is_symlink():
                alias.unlink(missing_ok=True)
                alias.symlink_to(Path('latest')/name)
        # Retain two complete generations instead of duplicating GB replay arrays.
        import shutil
        generations = sorted((p for p in snapshots.glob('step_*') if p.is_dir()), key=lambda p:p.stat().st_mtime)
        retained = set(generations[-2:])
        for step in self.config.get('retain_steps', []):
            milestone = [p for p in generations if p.name.startswith(f'step_{step}_')]
            if milestone:
                retained.add(milestone[-1])
        for old in generations:
            if old not in retained:
                shutil.rmtree(old)

    def _on_step(self):
        progress = min(self.model.num_timesteps/max(self.config['timesteps']*.7,1),1.)
        sigma = self.initial_noise + progress*(self.final_noise-self.initial_noise)
        noise = self.model.action_noise
        for obj in getattr(noise,'noises',[noise]):
            if obj is not None and hasattr(obj,'_sigma'):
                obj._sigma[:] = sigma
        if self.model.num_timesteps-self.last_logged >= 100 or self.config['smoke']:
            record = {'timesteps':self.model.num_timesteps,'updates':self.model._n_updates,
                      'wall_s':self.prior_wall_s+time.monotonic()-self.started}
            for key,value in self.model.logger.name_to_value.items():
                if key.startswith('train/') and np.isscalar(value):
                    if not np.isfinite(value):
                        raise FloatingPointError(f'Nonfinite learner metric {key}; resume from last valid checkpoint')
                    record[key] = float(value)
            with (self.output/'training_metrics.jsonl').open('a') as stream:
                stream.write(json.dumps(record)+'\n')
            self.last_logged = self.model.num_timesteps
        if self.model.num_timesteps-self.last_saved >= 1000:
            self.save(); self.last_saved = self.model.num_timesteps
        if self.config['smoke'] and (self.model.num_timesteps >= min(self.config['timesteps'],100) or time.monotonic()-self.started > 110):
            return False
        return True
