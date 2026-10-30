#!/usr/bin/env python3
"""阶段七策略对照评估：random / rule / DDPG 在替身环境上跑同一组工况。

DDPG 模型由 phase7_ddpg_train.py 产出（stable-baselines3）。rule 通过
动作向量适配层接入（与 RL 同一动作语义）；visibility 用均匀掩码，与
真实闭环一致。输出按策略和工况分列的判决分布与误达统计。
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).parents[1] / 'src/hydromining_sim/scripts'))
sys.path.insert(0, str(Path(__file__).parents[1] / 'test_scripts'))

from stable_baselines3 import DDPG

from phase7_surrogate_loop import load_specs
from algorithms.perception.config import PerceptionConfig
from algorithms.planning.decision import RulePolicy
from algorithms.planning.episodes import SCENARIOS
from algorithms.planning.rl_env import (ACTION_MODES, N_VIEWS, U_RANGE,
                                      DURATION_RANGE, HydrominingSurrogateEnv,
                                      VIEW_NAMES)
from algorithms.planning.surrogate import SurrogateWorld

MESH = ('install/hydromining_sim/share/hydromining_sim/models/coal_wall/meshes/'
        'coal_wall.obj')
FIDELITY_RESOLUTION = {'L1': (64, 64, 40), 'L2': (96, 96, 64)}


def make_env(fidelity, noise_scale, seed, action_mode='spray'):
    world = SurrogateWorld(MESH, resolution=FIDELITY_RESOLUTION[fidelity])
    cfg = yaml.safe_load(open(
        'install/hydromining_robot/share/hydromining_robot/config/'
        'viewpoints.yaml'))
    return HydrominingSurrogateEnv(world, load_specs(VIEW_NAMES), cfg,
                                   fidelity=fidelity, noise_scale=noise_scale,
                                   seed=seed, action_mode=action_mode)


def spray_vector(uv, duration):
    """(u,v,duration) -> spray 模式三维动作向量。"""
    scaled = 2 * (duration - DURATION_RANGE[0]) / \
        (DURATION_RANGE[1] - DURATION_RANGE[0]) - 1
    return np.array([np.clip(uv[0] / U_RANGE[1], -1, 1),
                     np.clip(uv[1] / U_RANGE[1], -1, 1),
                     np.clip(scaled, -1, 1)])


def action_vector(mode, view=None, uv=None, duration=None):
    """把语义动作编码成环境的连续动作向量（argmax 各段取所选）。"""
    a = -np.ones(ACTION_MODES + N_VIEWS + 3)
    a[mode] = 1.
    if view is not None:
        a[ACTION_MODES + VIEW_NAMES.index(view)] = 1.
    if uv is not None:
        a[ACTION_MODES + N_VIEWS] = np.clip(uv[0] / U_RANGE[1], -1, 1)
        a[ACTION_MODES + N_VIEWS + 1] = np.clip(uv[1] / U_RANGE[1], -1, 1)
    if duration is not None:
        scaled = 2 * (duration - DURATION_RANGE[0]) / \
            (DURATION_RANGE[1] - DURATION_RANGE[0]) - 1
        a[ACTION_MODES + N_VIEWS + 2] = np.clip(scaled, -1, 1)
    return a


class RuleAdapter:
    """RulePolicy -> 环境动作向量；可见性掩码取环境真实 FOV/遮挡结果。

    不能用均匀掩码：预测增益若落在该视点实际看不见的单元上，将永远
    无法兑现，规则层会陷入无限补看（实测 12 步 0 喷射全截断）。
    """

    def __init__(self, env):
        self.env_, self.policy_ = env, RulePolicy(env.perception_)
        self.view_poses_ = {n: np.asarray(s['tf_depth'][:3])
                            for n, s in env.specs_.items()}

    def act(self, obs):
        env, cfg = self.env_, self.env_.perception_
        if env.steps_used_ == 0:
            self.policy_.reset_episode()
        evaluated = env.last_evaluated_
        _, world = env._l1_surface()
        vis = {n: env._l1_visibility(n, world) for n in self.view_poses_}
        region = np.linalg.norm(env.frame_.grid(cfg.extent, cfg.resolution)[..., :2],
                                axis=-1) <= cfg.diameter / 2
        action = self.policy_.decide(
            evaluated['current'], evaluated['report']['assessment'],
            evaluated['change'], evaluated['surface'], env.frame_,
            self.view_poses_, vis, region)
        if env.action_mode_ == 'spray':
            # spray 模式无 observe/finish：finish 由环境达标自动收尾；
            # observe 决策退化为按缺口图补喷（基线近似，评估口径已注明）。
            if action['type'] == 'spray':
                return spray_vector(action['uv'], action['duration'])
            uv, duration = self.policy_.spray_target(
                evaluated['change'], region, evaluated['surface'])
            if uv is None:
                uv, duration = np.array([0., 0.]), 2.
            return spray_vector(uv, duration)
        if action['type'] == 'observe':
            return action_vector(0, view=action['view'])
        if action['type'] == 'spray':
            return action_vector(1, uv=action['uv'], duration=action['duration'])
        return action_vector(2)


def run_policy(env, act, episodes, scenarios):
    """每个工况轮换跑 episodes 次；返回判决/步数/误达统计。"""
    stats = {'verdicts': {}, 'steps': [], 'sprays': [], 'false_achieved': 0,
             'per_scenario': {}}
    order = (scenarios * (episodes // len(scenarios) + 1))[:episodes]
    for name in order:
        obs, _ = env.reset(options={'scenario': name})
        done, info = False, {}
        while not done:
            obs, _, terminated, truncated, info = env.step(act(obs))
            done = terminated or truncated
        verdict = info.get('verdict', 'truncated')
        truth = info.get('truth')
        stats['verdicts'][verdict] = stats['verdicts'].get(verdict, 0) + 1
        stats['steps'].append(env.steps_used_)
        stats['sprays'].append(env.sprays_used_)
        per = stats['per_scenario'].setdefault(name, {'verdicts': {}})
        per['verdicts'][verdict] = per['verdicts'].get(verdict, 0) + 1
        if verdict == 'achieved' and truth != 'achieved':
            stats['false_achieved'] += 1
    stats['mean_steps'] = float(np.mean(stats['steps']))
    stats['mean_sprays'] = float(np.mean(stats['sprays']))
    return stats


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', default=None,
                        help='SB3 DDPG zip；不给则跳过 learned 策略')
    parser.add_argument('--fidelity', default='L1', choices=['L1', 'L2'])
    parser.add_argument('--action-mode', default='spray',
                        choices=['full', 'spray'])
    parser.add_argument('--episodes', type=int, default=12)
    parser.add_argument('--noise-scale', type=float, default=0.)
    parser.add_argument('--seed', type=int, default=1000)
    parser.add_argument('--output', default=None)
    args = parser.parse_args()
    env = make_env(args.fidelity, args.noise_scale, args.seed, args.action_mode)
    scenarios = list(SCENARIOS)
    results = {}
    results['random'] = run_policy(env, lambda o: env.action_space.sample(),
                                   args.episodes, scenarios)
    env.reset()
    results['rule'] = run_policy(env, RuleAdapter(env).act,
                                 args.episodes, scenarios)
    if args.model:
        model = DDPG.load(args.model)
        results['ddpg'] = run_policy(
            env, lambda o: model.predict(o, deterministic=True)[0],
            args.episodes, scenarios)
    print(json.dumps(results, indent=2))
    if args.output:
        out = Path(args.output)
        out.mkdir(parents=True, exist_ok=True)
        (out / 'evaluation.json').write_text(json.dumps(results, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
