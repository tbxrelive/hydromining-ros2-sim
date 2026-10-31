#!/usr/bin/env python3
"""Evaluate learned, rule and random policies on held-out walls and layouts."""
import sys
import argparse
import json
from pathlib import Path
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from experiments.common.large_env import make_fast_env, make_reference_env
from phase7_train_large import ALGORITHMS
from algorithms.planning.fast_heightfield import TEST_SCENARIOS
from algorithms.planning.ddpg_optimized import DemonstrationDDPG
from algorithms.planning.decision import RulePolicy
from algorithms.jet_effects import jet_hydraulics


def evaluate(get_env, act, episodes, seed, fidelity):
    scenarios = list(TEST_SCENARIOS)
    records = []
    for episode in range(episodes):
        # All policies and training seeds see exactly the same held-out cases.
        name = scenarios[episode % len(scenarios)]
        mesh_index = (episode // len(scenarios)) % 3
        env = get_env(mesh_index)
        options = {'scenario': name, **({'mesh_index': mesh_index} if fidelity == 'L0' else {})}
        started = time.monotonic()
        obs, reset_info = env.reset(seed=seed+episode, options=options)
        reset_s = time.monotonic()-started
        while True:
            obs, _, terminated, truncated, info = env.step(act(env, obs))
            if terminated or truncated:
                break
        if fidelity == 'L0':
            retreat = env._map_heights(env.initial_-env.heights_)
            volume = float(np.sum(np.maximum(retreat, 0)*env.area_))
        else:
            volume = info['removed_volume_m3']
        hydraulics = jet_hydraulics(env.pressure_)
        records.append({'scenario': name, 'mesh_index': mesh_index,
            'initially_achieved': reset_info['initially_achieved'], 'verdict': info['verdict'],
            'truth': info['truth'], 'truth_fraction': info['truth_fraction'],
            'false_achieved': info['verdict'] == 'achieved' and info['truth'] == 'not_achieved',
            'truth_valid_fraction': info.get('truth_valid_fraction', 1.0),
            'steps': env.steps_used_, 'sprays': env.sprays_used_, 'spray_duration_s': env.total_spray_duration_,
            'water_m3': hydraulics['flow_m3_s']*env.total_spray_duration_,
            'energy_j': hydraulics['hydraulic_power_w']*env.total_spray_duration_,
            'removed_volume_m3': volume, 'wall_time_s': time.monotonic()-started,
            'reset_wall_s': reset_s,
            'truncated': bool(truncated)})
    summary = {}
    for label, selected in [('all', records), ('needs_work', [r for r in records if not r['initially_achieved']]),
                            ('initially_achieved', [r for r in records if r['initially_achieved']])]:
        summary[label] = {'episodes': len(selected),
            'success_rate': (sum(r['verdict'] == 'achieved' and r['truth'] == 'achieved' for r in selected)/len(selected)
                             if selected else None),
            'false_achieved': sum(r['false_achieved'] for r in selected),
            'truth_unknown': sum(r['truth'] == 'unknown' for r in selected),
            'removed_volume_known_episodes': sum(r['removed_volume_m3'] is not None for r in selected),
            'mean_sprays': float(np.mean([r['sprays'] for r in selected])) if selected else None,
            'mean_steps': float(np.mean([r['steps'] for r in selected])) if selected else None,
            'mean_spray_duration_s': float(np.mean([r['spray_duration_s'] for r in selected])) if selected else None,
            'mean_removed_volume_m3': (float(np.mean([r['removed_volume_m3'] for r in selected
                                                      if r['removed_volume_m3'] is not None]))
                                       if any(r['removed_volume_m3'] is not None for r in selected) else None),
            'mean_water_m3': float(np.mean([r['water_m3'] for r in selected])) if selected else None,
            'mean_energy_j': float(np.mean([r['energy_j'] for r in selected])) if selected else None,
            'mean_wall_time_s': float(np.mean([r['wall_time_s'] for r in selected])) if selected else None}
    return {'summary': summary, 'episodes': records}


def rule_action(env, _):
    if hasattr(env, 'rule_action'):
        return env.rule_action()
    evaluated = env.last_evaluated_
    surface, change = evaluated['surface'], evaluated['change']
    region = np.linalg.norm(surface.centers_[..., :2], axis=-1) <= env.perception_.diameter/2
    rule = RulePolicy(env.perception_, max_duration=8., pressure=env.pressure_, standoff=env.standoff_)
    uv, duration = rule.spray_target(change, region, surface)
    if uv is None:
        uv, duration = np.array([0., 0.]), 2.
    return np.array([uv[0]/env.action_radius_, uv[1]/env.action_radius_, (duration-5.)/3.], dtype=np.float32)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--checkpoint', required=True)
    parser.add_argument('--assets', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--episodes', type=int, default=12)
    parser.add_argument('--seed', type=int, default=10000)
    parser.add_argument('--noise-scale', type=float, default=0.)
    parser.add_argument('--fidelity', choices=('L0', 'L1', 'L2'), default='L0')
    parser.add_argument('--max-steps', type=int, default=None,
                        help='Optional shorter evaluation budget, recorded in the output')
    parser.add_argument('--include-reference', action='store_true')
    args = parser.parse_args()
    checkpoint = Path(args.checkpoint)
    metadata = json.loads((checkpoint/'metadata.json').read_text())
    cls = ALGORITHMS[metadata['algorithm']]
    kwargs = {'replay_buffer_kwargs': {'demo_path': None}} if cls is DemonstrationDDPG else {}
    model = cls.load(str(checkpoint/'checkpoint'), device='cpu', **kwargs)
    envs = {}
    def get_env(mesh_index):
        key = 0 if args.fidelity == 'L0' else mesh_index
        if key not in envs:
            envs[key] = (make_fast_env(args.assets, args.seed, 'test', args.noise_scale)
                         if args.fidelity == 'L0' else
                         make_reference_env(args.assets, mesh_index, args.fidelity, args.seed, args.noise_scale))
            if args.max_steps is not None:
                envs[key].max_steps_ = args.max_steps
        return envs[key]
    try:
        results = {'fidelity': args.fidelity, 'test_seed': args.seed, 'noise_scale': args.noise_scale,
                   'max_steps_override': args.max_steps,
                   'algorithm': metadata['algorithm'], 'training_seed': metadata['seed'],
                   'trained_steps': metadata['num_timesteps'], 'training_wall_s': metadata['wall_s']}
        if args.include_reference:
            rng = np.random.default_rng(args.seed)
            results['random'] = evaluate(get_env, lambda _, obs: rng.uniform(-1., 1., 3), args.episodes, args.seed, args.fidelity)
            results['rule'] = evaluate(get_env, rule_action, args.episodes, args.seed, args.fidelity)
        results[metadata['algorithm']] = evaluate(get_env, lambda _, obs: model.predict(obs, deterministic=True)[0],
                                                   args.episodes, args.seed, args.fidelity)
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(results, indent=2))
        print(json.dumps({k: v['summary'] for k, v in results.items() if isinstance(v, dict)}, indent=2))
    finally:
        for env in envs.values():
            env.close()


if __name__ == '__main__':
    main()
