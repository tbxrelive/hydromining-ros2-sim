#!/usr/bin/env python3
"""Compare L0 with the fine Level Set solver; no policy training is performed."""
import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/hydromining_sim/scripts'))
from algorithms.erosion_model import SprayAction
from algorithms.local_spray_simulation import LocalSpraySimulation, surface_rays
from algorithms.perception.kinematics import look_at
from algorithms.planning.fast_heightfield import FastHeightfieldEnv
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from experiments.common.large_env import fidelity_signature


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--long-only', action='store_true', help='diagnose the 8-second case')
    args = parser.parse_args()
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    task = yaml.safe_load(Path('src/hydromining_robot/config/phase7_large_task.yaml').read_text())
    started = time.monotonic()
    sim = LocalSpraySimulation('src/hydromining_sim/models/coal_wall/meshes/coal_wall.obj',
        task['source_coal_pose'], task['region_center_world'], task['wall_normal_world'], output,
        wall_budget_s=180., spacing=task['geometry_spacing_m'], patch_extent=task['patch_extent_m'],
        wall_thickness=task['wall_thickness_m'])
    setup_s = time.monotonic()-started
    coordinates = np.arange(-.2475, .25, .005)
    full_coordinates = (np.arange(round(task['patch_extent_m']/.005))+.5)*.005-task['patch_extent_m']/2
    scene = sim.model.surface_scene(sim.initial_phi, sim.grid.origin, sim.grid.spacing)
    initial = surface_rays(scene, coordinates, coordinates)
    initial_full = surface_rays(scene, full_coordinates, full_coordinates)
    frame = sim.frame_
    cameras = [look_at(np.asarray(task['viewpoints'][name]['camera_position_world']), frame.origin)
               for name in ('front_0p8', 'left_0p8', 'right_0p8')]
    K = np.array([[171.7, 0, 160], [0, 171.7, 120], [0, 0, 1]])
    fast = FastHeightfieldEnv(initial_full, frame, cameras, K, (240, 320),
                              pressure=task['pressure_mpa'], standoff=task['standoff_m'],
                              scenario_names=['empty'], scenarios={'empty': {'shots': []}},
                              patch_extent=task['patch_extent_m'],
                              diameter=task['diameter'], required_depth=task['required_depth'],
                              required_fraction=task['required_fraction'], wall_thickness=task['wall_thickness_m'])
    inside = np.linalg.norm(frame.grid(.5, .005)[..., :2], axis=-1) <= task['diameter']/2
    runs = []
    cases = [(20., [(0., 0., 2.)]), (30., [(0., 0., 2.)]),
             (40., [(0., 0., 2.)]), (30., [(0., 0., 8.)]),
             (30., [(.18, 0., 2.)]),
             (30., [(0., 0., 2.), (.06, -.03, 2.)])]
    for pressure, actions in ([cases[3]] if args.long_only else cases):
        sim.reset()
        fast.heights_ = initial_full.copy()
        fast.pressure_ = pressure
        fast_s = reference_s = 0.
        for u, v, duration in actions:
            started = time.monotonic()
            fast._spray(u, v, duration)
            fast_s += time.monotonic()-started
            started = time.monotonic()
            phi, _ = sim.simulate(SprayAction(np.array([u, v, task['standoff_m']]),
                                               np.array([0., 0., -1.]), pressure, duration))
            sim.commit(phi)
            reference_s += time.monotonic()-started
        reference = surface_rays(sim.model.surface_scene(sim.grid.phi, sim.grid.origin, sim.grid.spacing),
                                 coordinates, coordinates)
        offset = round((task['patch_extent_m']-.5)/(.005*2))
        fast_delta = (initial_full-fast.heights_)[offset:offset+100, offset:offset+100]
        ref_delta = initial-reference
        valid = inside & np.isfinite(ref_delta) & np.isfinite(fast_delta)
        ref_volume = float(np.maximum(ref_delta[valid], 0).sum()*.005**2)
        fast_volume = float(np.maximum(fast_delta[valid], 0).sum()*.005**2)
        row = {'pressure_mpa': pressure, 'actions': actions, 'valid_fraction': float(valid.sum()/inside.sum()),
               'mean_absolute_depth_error_m': float(np.mean(np.abs(ref_delta[valid]-fast_delta[valid]))),
               'reference_volume_m3': ref_volume, 'fast_volume_m3': fast_volume,
               'relative_volume_error': abs(ref_volume-fast_volume)/ref_volume,
               'fast_s': fast_s, 'reference_s': reference_s}
        runs.append(row)
        print(json.dumps(row), flush=True)
    passed = all(r['valid_fraction'] == 1. and r['mean_absolute_depth_error_m'] <= .005 and
                 r['relative_volume_error'] <= .15 for r in runs)
    report = {'passed': passed, 'scope': '8-second diagnostic only' if args.long_only else 'full case set',
              'signature': fidelity_signature(),
              'setup_s': setup_s, 'grid_spacing_m': sim.grid.spacing,
              'grid_shape': sim.grid.resolution, 'training_performed': False, 'runs': runs}
    (output/'fidelity.json').write_text(json.dumps(report, indent=2))
    print('fidelity passed:', passed, flush=True)


if __name__ == '__main__':
    main()
