#!/usr/bin/env python3
"""User-run SB3 learners for the local task; smoke runs are strictly bounded."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import time

import numpy as np
import torch
import yaml
from stable_baselines3 import DDPG, SAC, TD3
from stable_baselines3.common.noise import NormalActionNoise
from stable_baselines3.common.vec_env import SubprocVecEnv

from phase7_large_env import TASK, fidelity_signature, make_fast_env
from phase7_train_optimized import CheckpointCallback
from algorithms.planning.ddpg_optimized import DemonstrationDDPG, DemonstrationReplayBuffer

ALGORITHMS = {'ddpg': DDPG, 'ddpg_demo': DemonstrationDDPG, 'td3': TD3, 'sac': SAC}


def collect_demonstrations(path, assets, episodes, seed, max_steps=None):
    """Rule demonstrations use only the separate training wall/scenario split."""
    env = make_fast_env(assets, seed, 'train')
    arrays = {k: [] for k in ('observations', 'next_observations', 'actions', 'rewards', 'dones')}
    try:
        for episode in range(episodes):
            obs, _ = env.reset(seed=seed+episode)
            while True:
                action = env.rule_action()
                new, reward, terminated, truncated, _ = env.step(action)
                for key, value in zip(arrays, (obs, new, action, reward, float(terminated))):
                    arrays[key].append(value)
                obs = new
                if terminated or truncated or (max_steps and len(arrays['actions']) >= max_steps):
                    break
            if max_steps and len(arrays['actions']) >= max_steps:
                break
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(path, **{k: np.asarray(v, dtype=np.float32) for k, v in arrays.items()})
        path.with_suffix('.json').write_text(json.dumps({'split': 'train', 'seed': seed,
            'episodes_requested': episodes, 'samples': len(arrays['actions']),
            'observation_version': 'v3'}, indent=2))
    finally:
        env.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--algorithm', choices=ALGORITHMS, default='ddpg_demo')
    parser.add_argument('--assets', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--fidelity-report', default='results/phase7_large_task/fidelity_release_v2/fidelity.json')
    parser.add_argument('--timesteps', type=int, default=50000)
    parser.add_argument('--num-envs', type=int, default=2)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--device', default='cuda:0')
    parser.add_argument('--demo-path')
    parser.add_argument('--collect-demo', action='store_true')
    parser.add_argument('--demo-episodes', type=int, default=24)
    parser.add_argument('--demo-max-steps', type=int)
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--smoke', action='store_true')
    parser.add_argument('--bc-updates', type=int, default=1000)
    parser.add_argument('--noise-scale', type=float, default=0.)
    parser.add_argument('--schedule-steps', type=int, default=50000)
    args = parser.parse_args()
    if args.collect_demo:
        collect_demonstrations(args.output, args.assets, args.demo_episodes, args.seed, args.demo_max_steps)
        return
    if args.smoke:
        if args.timesteps > 100 or args.bc_updates > 100:
            parser.error('smoke requires timesteps and BC updates <=100')
        signal.signal(signal.SIGALRM, lambda *_: (_ for _ in ()).throw(TimeoutError('120s smoke limit')))
        signal.alarm(120)
    else:
        report = json.loads(Path(args.fidelity_report).read_text())
        if (not report['passed'] or report.get('scope') != 'full case set' or
                report.get('signature') != fidelity_signature()):
            raise RuntimeError('L0 fidelity gate has not passed; run the full fidelity benchmark first')
    if args.algorithm == 'ddpg_demo' and not args.demo_path:
        parser.error('ddpg_demo requires training demonstrations')
    torch.set_num_threads(1)
    if args.device.startswith('cuda'):
        torch.cuda.set_per_process_memory_fraction(.6, torch.device(args.device))
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    config = vars(args).copy()
    config.update(task='large', fidelity='L0', observation_version='v3',
                  physical_gpu_mask=os.environ.get('CUDA_VISIBLE_DEVICES', 'unset'),
                  target_timesteps=args.timesteps, timesteps=args.timesteps if args.smoke else args.schedule_steps,
                  retain_steps=[15000])
    task = yaml.safe_load(TASK.read_text())
    config['task_parameters'] = {key: task[key] for key in
        ('diameter', 'required_depth', 'required_fraction', 'pressure_mpa', 'standoff_m', 'max_steps')}
    sources = sorted(Path('src/hydromining_sim/scripts/algorithms').rglob('*.py')) + [TASK]
    config['source_sha256'] = hashlib.sha256(b''.join(p.read_bytes() for p in sources)).hexdigest()
    if args.demo_path:
        config['demo_sha256'] = hashlib.sha256(Path(args.demo_path).read_bytes()).hexdigest()
        if json.loads(Path(args.demo_path).with_suffix('.json').read_text())['split'] != 'train':
            raise ValueError('demonstrations must use the training split')
    env = None
    try:
        factories = [lambda i=i: make_fast_env(args.assets, args.seed+i, 'train', args.noise_scale)
                     for i in range(args.num_envs)]
        env = SubprocVecEnv(factories, start_method='spawn') if args.num_envs > 1 else factories[0]()
        cls = ALGORITHMS[args.algorithm]
        if args.resume:
            old = json.loads((output/'metadata.json').read_text())
            for key in ('algorithm', 'observation_version', 'num_envs', 'seed', 'source_sha256', 'smoke', 'demo_sha256'):
                if old.get(key) != config.get(key):
                    raise ValueError(f'resume {key} mismatch')
            config['prior_wall_s'] = old['wall_s']
            model = cls.load(str(output/'checkpoint'), env=env, device=args.device)
            model.load_replay_buffer(str(output/'replay.pkl'))
            rng = torch.load(output/'rng.pt', weights_only=False)
            np.random.set_state(rng['numpy'])
            torch.set_rng_state(rng['torch'])
            if rng['cuda'] is not None and torch.cuda.is_available():
                torch.cuda.set_rng_state_all(rng['cuda'])
        else:
            kwargs = dict(seed=args.seed, device=args.device, buffer_size=128 if args.smoke else 50000,
                          learning_starts=2 if args.smoke else 500, batch_size=16 if args.smoke else 256,
                          tau=.005, gamma=.99, train_freq=(1, 'step'), gradient_steps=-1,
                          policy_kwargs={'net_arch': [256, 256]}, verbose=1)
            if args.algorithm in ('ddpg', 'ddpg_demo', 'td3'):
                kwargs['action_noise'] = NormalActionNoise(np.zeros(3), .2*np.ones(3))
            if args.algorithm == 'ddpg_demo':
                kwargs.update(actor_lr=1e-4, critic_lr=1e-3, bc_weight=.1, q_filter=True,
                              replay_buffer_class=DemonstrationReplayBuffer,
                              replay_buffer_kwargs={'demo_path': args.demo_path, 'demo_fraction': .25})
            model = cls('MlpPolicy', env, **kwargs)
            if args.algorithm == 'ddpg_demo':
                model.pretrain_actor(args.bc_updates, 16 if args.smoke else 256)
        callback = CheckpointCallback(output, config)
        remaining = max(args.timesteps-model.num_timesteps, 0)
        if remaining:
            model.learn(remaining, callback=callback, reset_num_timesteps=not args.resume)
        callback.model = model
        callback.save()
        (output/'result.json').write_text(json.dumps({'timesteps': model.num_timesteps,
            'updates': model._n_updates, 'algorithm': args.algorithm, 'smoke': args.smoke,
            'performance_evaluated': False}, indent=2))
    finally:
        if env is not None:
            env.close()
        if args.smoke:
            signal.alarm(0)


if __name__ == '__main__':
    main()
