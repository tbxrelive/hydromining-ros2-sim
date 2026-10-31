"""SB3 2.4 DDPG extensions: demonstration replay, separate rates and Q-filter BC.

Algorithm idea: Nair et al., arXiv:1709.10089. This module adapts the idea to
SB3/PyTorch, not the older TensorFlow/HER implementation. No HER relabeling.
"""
import numpy as np
import torch as th
from stable_baselines3 import DDPG
from stable_baselines3.common.buffers import ReplayBuffer
from stable_baselines3.common.type_aliases import ReplayBufferSamples


class DemonstrationReplayBuffer(ReplayBuffer):
    """Mix training-only demonstrations without overwriting online transitions."""

    def __init__(self, *args, demo_path=None, demo_fraction=.25, **kwargs):
        super().__init__(*args, **kwargs)
        if not 0 <= demo_fraction < 1:
            raise ValueError('demo_fraction must be in [0,1)')
        self.demo_fraction = demo_fraction
        self.demonstrations = None
        if demo_path:
            with np.load(demo_path, allow_pickle=False) as data:
                self.demonstrations = {key: data[key].copy() for key in
                                       ('observations', 'next_observations', 'actions', 'rewards', 'dones')}
            d = self.demonstrations
            if (not len(d['actions']) or d['observations'].shape[1:] != self.obs_shape or
                    d['actions'].shape[1] != self.action_dim):
                raise ValueError('demonstration dimensions do not match policy')
            if any(len(a) != len(d['actions']) for a in d.values()):
                raise ValueError('inconsistent demonstration lengths')
            if d['next_observations'].shape != d['observations'].shape:
                raise ValueError('next observation dimensions mismatch')
            if not np.isin(d['dones'], [0., 1.]).all():
                raise ValueError('demonstration terminal flags must be binary')
            if any(not np.isfinite(a).all() for a in d.values()):
                raise ValueError('nonfinite demonstration data')
            if (np.abs(d['actions']) > 1).any():
                raise ValueError('demonstration actions must be normalized')

    def demo_sample(self, count, env=None):
        data = self.demonstrations
        indices = np.random.randint(len(data['actions']), size=count)
        return ReplayBufferSamples(
            self.to_torch(self._normalize_obs(data['observations'][indices], env)),
            self.to_torch(data['actions'][indices]),
            self.to_torch(self._normalize_obs(data['next_observations'][indices], env)),
            self.to_torch(data['dones'][indices].reshape(-1, 1)),
            self.to_torch(self._normalize_reward(data['rewards'][indices].reshape(-1, 1), env)))

    def sample(self, batch_size, env=None):
        count = int(batch_size * self.demo_fraction) if self.demonstrations is not None else 0
        online = super().sample(batch_size-count, env)
        if not count:
            return online
        demo = self.demo_sample(count, env)
        return ReplayBufferSamples(*(th.cat((a, b), dim=0) for a, b in zip(online, demo)))


class DemonstrationDDPG(DDPG):
    """Retain single-critic DDPG, optionally adding Q-filtered demonstration loss."""

    def __init__(self, *args, actor_lr=1e-4, critic_lr=1e-3, bc_weight=.1,
                 q_filter=True, **kwargs):
        self.actor_lr, self.critic_lr = actor_lr, critic_lr
        self.bc_weight, self.q_filter = bc_weight, q_filter
        super().__init__(*args, **kwargs)

    def _update_learning_rate(self, optimizers):
        for optimizer in optimizers if isinstance(optimizers, list) else [optimizers]:
            rate = self.actor_lr if optimizer is self.actor.optimizer else self.critic_lr
            for group in optimizer.param_groups:
                group['lr'] = rate

    def pretrain_actor(self, updates=1000, batch_size=256):
        """User-run behavior cloning warm start; synchronize the actor target."""
        buffer = self.replay_buffer
        if not isinstance(buffer, DemonstrationReplayBuffer) or buffer.demonstrations is None:
            raise ValueError('pretraining requires demonstration replay')
        self.policy.set_training_mode(True)
        self._update_learning_rate([self.actor.optimizer])
        losses = []
        for _ in range(updates):
            batch = buffer.demo_sample(batch_size)
            loss = th.mean((self.actor(batch.observations)-batch.actions)**2)
            self.actor.optimizer.zero_grad()
            loss.backward()
            self.actor.optimizer.step()
            losses.append(float(loss.detach()))
        self.actor_target.load_state_dict(self.actor.state_dict())
        return losses

    def train(self, gradient_steps, batch_size=100):
        super().train(gradient_steps, batch_size)
        buffer = self.replay_buffer
        if self.bc_weight <= 0 or not isinstance(buffer, DemonstrationReplayBuffer) or buffer.demonstrations is None:
            return
        for _ in range(gradient_steps):
            batch = buffer.demo_sample(batch_size)
            action = self.actor(batch.observations)
            with th.no_grad():
                mask = (self.critic.q1_forward(batch.observations, batch.actions) >
                        self.critic.q1_forward(batch.observations, action)) if self.q_filter else th.ones((batch_size,1), device=self.device, dtype=th.bool)
            per_sample = ((action-batch.actions)**2).mean(dim=1, keepdim=True)
            loss = self.bc_weight*(per_sample*mask).sum()/mask.sum().clamp_min(1)
            self.actor.optimizer.zero_grad()
            loss.backward()
            self.actor.optimizer.step()
            self.logger.record('train/bc_loss', float(loss.detach()))
            self.logger.record('train/q_filter_fraction', float(mask.float().mean()))
