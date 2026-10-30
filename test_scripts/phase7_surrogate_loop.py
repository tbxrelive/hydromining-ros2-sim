#!/usr/bin/env python3
"""阶段七替身环境闭环：规则决策层在离线世界上跑完整回合。

回合语义：基线建在任务开始时的表面（原始煤壁）-> 脚本预喷模拟此前会话
已完成的部分工作量 -> 决策循环继续完成剩余任务。观测 = 射线合成快照 ->
PerceptionSession；喷射 = TCP 位姿 -> 蚀除模型；真值只用于回合标注与核对。
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).parents[1] / 'src/hydromining_sim/scripts'))
from algorithms.perception.config import PerceptionConfig
from algorithms.perception.pipeline import PerceptionSession
from algorithms.perception.metrics import truth_change
from algorithms.perception.wall_frame import WallFrame
from algorithms.planning.surrogate import SurrogateWorld, spray_target_tcp
from algorithms.planning.episodes import SCENARIOS, apply_scenario
from algorithms.planning.nbv import visibility_masks
from algorithms.planning.decision import RulePolicy

MESH = ('install/hydromining_sim/share/hydromining_sim/models/coal_wall/meshes/'
        'coal_wall.obj')
DATASET = 'results/phase6/6.0/clean_dataset/dataset/baseline'
VIEW_NAMES = ['front_0p6', 'front_0p8', 'front_1p0', 'left_0p8', 'right_0p8',
              'up_0p8', 'down_0p8']
OBSERVE_SET = ['front_0p8', 'left_0p8', 'right_0p8']


def load_specs(names):
    specs = {}
    for name in names:
        data = np.load(Path(DATASET) / f'baseline_{name}.npz')
        specs[name] = dict(K=data['K'], image_shape=data['depth'].shape,
                           tf_depth=data['tf_depth'], tf_lidar=data['tf_lidar'],
                           joints=data['joints'])
    return specs


def observe(world, specs, names, rng, stamp_base, frames=3, noise_scale=0.):
    """按给定视点生成一组合成快照组（PerceptionSession 的 groups 格式）。"""
    return [[world.snapshot(specs[name]['tf_depth'], specs[name]['tf_lidar'],
                            specs[name]['K'], specs[name]['image_shape'],
                            stamp_base + 10 * index + frame, rng,
                            noise_scale=noise_scale)
             for frame in range(frames)] for index, name in enumerate(names)]


def run_episode(world, specs, name, scenario, cfg, rng, policy, noise_scale=0.,
                max_steps=12):
    """单回合：预喷残差 -> 决策循环；返回逐步日志与真值核对。"""
    center, normal = np.array(cfg['region_center_world']), np.array(cfg['wall_normal_world'])
    policy.reset_episode()
    world.reset()
    session = PerceptionSession(center)
    session.set_baseline(observe(world, specs, OBSERVE_SET, rng, 0,
                                 noise_scale=noise_scale))
    frame = session.frame_
    pristine = world.truth_heights(frame, .24, .002)
    apply_scenario(world, frame, -normal, scenario['shots'])
    latest = session.evaluate(observe(world, specs, OBSERVE_SET, rng, 100,
                                      noise_scale=noise_scale))
    view_poses = {n: s['tf_depth'][:3] for n, s in specs.items()}
    region = np.linalg.norm(frame.grid(session.config_.extent,
                                       session.config_.resolution)[..., :2],
                            axis=-1) <= session.config_.diameter / 2
    vis, stamp_base, log = None, 500, []
    for step in range(max_steps):
        state = latest['surface'].state(session.config_.minimum_observations,
                                        session.config_.maximum_sigma)
        assessment = latest['report']['assessment']
        if vis is None:
            vgroups = {n: observe(world, specs, [n], rng, stamp_base + 10 * i,
                                  noise_scale=noise_scale)[0]
                       for i, n in enumerate(VIEW_NAMES)}
            stamp_base += 200
            vis = visibility_masks(session, vgroups)
        action = policy.decide(state, assessment, latest['change'],
                               latest['surface'], frame, view_poses, vis, region)
        entry = {'step': step, 'type': action['type'],
                 'rationale': action['rationale'],
                 'assessment': {key: assessment[key] for key in
                                ('status', 'coverage', 'fraction_lower',
                                 'fraction_upper')},
                 'scores': action.get('scores', {})}
        log.append(entry)
        print(f"  step {step}: {action['type']} - {action['rationale']}", flush=True)
        if action['type'] == 'finish':
            break
        if action['type'] == 'observe':
            latest = session.observe(observe(world, specs, [action['view']], rng,
                                             stamp_base, noise_scale=noise_scale))
        else:
            tcp = spray_target_tcp(frame, action['uv'], -normal, policy.standoff_)
            stats = world.spray_tcp(tcp, policy.pressure_, action['duration'])
            entry['spray_stats'] = {k: stats[k] for k in
                                    ('volume_removed_m3', 'max_depth_m')}
            latest = session.evaluate(observe(world, specs, OBSERVE_SET, rng,
                                              stamp_base + 50,
                                              noise_scale=noise_scale))
        stamp_base += 100
    final = truth_change(pristine, world.truth_heights(frame, .24, .002), frame,
                         .24, .002, .1, .02, .9)
    return {'scenario': name, 'steps': len(log), 'log': log, 'truth': final,
            'initial_fraction': scenario['fraction'],
            'verdict': log[-1]['assessment']['status'] if log else None}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenarios', nargs='*', default=list(SCENARIOS))
    parser.add_argument('--noise-scale', type=float, default=0.)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--output', default=None)
    args = parser.parse_args()
    cfg = yaml.safe_load(open('install/hydromining_robot/share/hydromining_robot/'
                              'config/viewpoints.yaml'))
    specs = load_specs(VIEW_NAMES)
    world = SurrogateWorld(MESH)
    policy = RulePolicy(PerceptionConfig())
    rng = np.random.default_rng(args.seed)
    results = {}
    for name in args.scenarios:
        print(f'===== 工况 {name} =====', flush=True)
        start = time.time()
        results[name] = run_episode(world, specs, name, SCENARIOS[name], cfg, rng,
                                    policy, noise_scale=args.noise_scale)
        results[name]['wall_time_s'] = time.time() - start
        print(f"-> 判决 {results[name]['verdict']}，真值达标比例 "
              f"{results[name]['truth']['fraction_ge_threshold']:.3f}，"
              f"用时 {results[name]['wall_time_s']:.0f}s\n", flush=True)
    if args.output:
        out = Path(args.output)
        out.mkdir(parents=True, exist_ok=True)
        (out / 'episodes.json').write_text(json.dumps(results, indent=2, default=float))
    return 0


if __name__ == '__main__':
    sys.exit(main())
