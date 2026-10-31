#!/usr/bin/env python3
"""阶段七 DDPG 训练：stable-baselines3 开源实现，两级替身环境。

L1 初训可选 --l2 切换到完整感知管线微调。评估时与规则基线对照。
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).parents[1] / 'src/hydromining_sim/scripts'))
sys.path.insert(0, str(Path(__file__).parents[1] / 'test_scripts'))

from stable_baselines3 import DDPG
from stable_baselines3.common.noise import NormalActionNoise
from stable_baselines3.common.vec_env import SubprocVecEnv

from phase7_surrogate_loop import load_specs
from algorithms.planning.surrogate import SurrogateWorld
from algorithms.planning.rl_env import HydrominingSurrogateEnv, VIEW_NAMES

MESH = ('install/hydromining_sim/share/hydromining_sim/models/coal_wall/meshes/'
        'coal_wall.obj')
# 离线实测：12s 喷射求解 4.1s -> 1.3s，弹坑 8.5cm -> 6.6cm（仍 >> 2cm 达标线）。
FIDELITY_RESOLUTION = {'L1': (64, 64, 40), 'L2': (96, 96, 64)}


def make_env(cfg, specs, fidelity, noise_scale, seed, action_mode='spray'):
    world = SurrogateWorld(MESH, resolution=FIDELITY_RESOLUTION[fidelity])
    return HydrominingSurrogateEnv(world, specs, cfg, fidelity=fidelity,
                                   noise_scale=noise_scale, seed=seed,
                                   action_mode=action_mode)


def _env_factory(fidelity, noise_scale, seed, action_mode):
    """SubprocVecEnv 在子进程内重新构建环境与配置（对象须可 pickle 或惰性构造）。"""
    def _init():
        import yaml
        cfg = yaml.safe_load(open(
            'install/hydromining_robot/share/hydromining_robot/config/'
            'viewpoints.yaml'))
        specs = load_specs(VIEW_NAMES)
        return make_env(cfg, specs, fidelity, noise_scale, seed, action_mode)
    return _init


def evaluate_policy(model, env, episodes=10):
    """确定性评估：统计完成率、误达、平均步数。"""
    stats = {'achieved': 0, 'not_achieved': 0, 'insufficient': 0,
             'false_achieved': 0, 'steps': [], 'verdicts': []}
    for _ in range(episodes):
        obs, _ = env.reset()
        done = False
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, _, terminated, truncated, info = env.step(action)
            done = terminated or truncated
        verdict = info.get('verdict')
        truth = info.get('truth')
        stats['steps'].append(env.steps_used_)
        stats['verdicts'].append(verdict)
        if verdict == 'achieved':
            stats['achieved'] += 1
            if truth == 'not_achieved':
                stats['false_achieved'] += 1
        elif verdict == 'not_achieved':
            stats['not_achieved'] += 1
        else:
            stats['insufficient'] += 1
    stats['mean_steps'] = float(np.mean(stats['steps']))
    return stats


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--timesteps', type=int, default=20000)
    parser.add_argument('--fidelity', default='L1', choices=['L1', 'L2'])
    parser.add_argument('--action-mode', default='spray',
                        choices=['full', 'spray'])
    parser.add_argument('--num-envs', type=int, default=4)
    parser.add_argument('--noise-scale', type=float, default=0.)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--output', required=True)
    parser.add_argument('--eval-episodes', type=int, default=10)
    args = parser.parse_args()
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    cfg = yaml.safe_load(open('install/hydromining_robot/share/hydromining_robot/'
                              'config/viewpoints.yaml'))
    specs = load_specs(VIEW_NAMES)
    if args.num_envs > 1:
        train_env = SubprocVecEnv([
            _env_factory(args.fidelity, args.noise_scale, args.seed + i,
                         args.action_mode)
            for i in range(args.num_envs)])
    else:
        train_env = make_env(cfg, specs, args.fidelity, args.noise_scale,
                             args.seed, args.action_mode)
    # DDPG 连续动作；用 SB3 默认 MlpPolicy + 正态探索噪声
    noise = NormalActionNoise(mean=np.zeros(train_env.action_space.shape),
                              sigma=0.2 * np.ones(train_env.action_space.shape))
    model = DDPG('MlpPolicy', train_env, action_noise=noise, verbose=1,
                 seed=args.seed, buffer_size=50000, learning_starts=500,
                 batch_size=128, tau=0.005, gamma=0.95,
                 train_freq=(1, 'step'), policy_kwargs={'net_arch': [256, 256]})
    start = time.time()
    model.learn(total_timesteps=args.timesteps)
    train_s = time.time() - start
    model.save(str(output / 'ddpg_policy'))
    train_env.close()
    env = make_env(cfg, specs, args.fidelity, args.noise_scale, args.seed + 1000,
                   args.action_mode)
    stats = evaluate_policy(model, env, args.eval_episodes)
    stats.update({'timesteps': args.timesteps, 'train_s': train_s,
                  'fidelity': args.fidelity, 'noise_scale': args.noise_scale,
                  'num_envs': args.num_envs, 'action_mode': args.action_mode,
                  'model': str(output / 'ddpg_policy.zip')})
    (output / 'train_report.json').write_text(json.dumps(stats, indent=2))
    print(json.dumps(stats, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
