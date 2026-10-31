#!/usr/bin/env python3
"""Bounded offline regression for the existing Phase 4 integration entry point.

Deterministic local erosion only: seed is a compatibility result label, not a
claim of a calibrated random material model. Full ROS/Gazebo acceptance lives in
 test_phase4_acceptance.py.
"""
import os
for name in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[name] = '1'
import argparse
import json
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/hydromining_sim/scripts'))
from algorithms.spray_simulation import SpraySimulation
from algorithms.erosion_model import SprayAction
from algorithms.mesh_converter import MeshConverter


class PhaseIntegrationTest:
    """Reuse initial BVH/grid; all integration paths use physical spacing and duration."""

    def __init__(self, mesh_path, output_dir='/tmp/phase4_tests'):
        self.mesh_path = mesh_path
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True,exist_ok=True)
        self.simulation = None

    def test_full_pipeline(self, resolution=(80,80,56), seed=42, pressure=10.):
        if self.simulation is None or self.simulation.grid.resolution != tuple(resolution):
            self.simulation = SpraySimulation(self.mesh_path,resolution)
        self.simulation.reset()
        action = SprayAction(np.array([0.,0.,1.]),np.array([0.,0.,-1.]),pressure,12.)
        phi, stats = self.simulation.simulate(action)
        v,f = self.simulation.mesh(phi)
        path = self.output_dir/f'coal_wall_after_spray_seed{seed}.obj'
        assert MeshConverter.save_mesh(v,f,str(path))
        assert stats['elapsed_s'] == action.duration or abs(stats['elapsed_s']-action.duration)<1e-9
        assert stats['volume_removed_m3'] > 0
        assert stats['max_depth_m'] > 0
        stats.update({'output_mesh_path':str(path),'resolution':resolution,
                      'mesh_vertices':len(v),'mesh_faces':len(f)})
        return stats

    def test_reproducibility(self, seed=42, n_runs=2):
        s=self.simulation
        if s is None:
            self.test_full_pipeline()
            s=self.simulation
        a=SprayAction(np.array([0.,0.,1.]),np.array([0.,0.,-1.]),10.,12.)
        first=None
        for _ in range(n_runs):
            s.reset();phi,_=s.simulate(a)
            if first is None:
                first=phi.copy()
            else:
                assert np.array_equal(first,phi)
        return True

    def test_parameter_sensitivity(self):
        s=self.simulation
        assert s is not None
        result=[]
        for pressure in [5.,10.,15.,20.]:
            s.reset()
            a=SprayAction(np.array([0.,0.,1.]),np.array([0.,0.,-1.]),pressure,12.)
            _,stats=s.simulate(a)
            result.append({'P':pressure,'volume_removed':stats['volume_removed_m3']})
        volumes=[r['volume_removed'] for r in result]
        assert all(a<=b for a,b in zip(volumes,volumes[1:])),result
        assert volumes[-1]>volumes[0],result
        return result


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--quick',action='store_true')
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[1]
    tester=PhaseIntegrationTest(str(root/'src/hydromining_sim/models/coal_wall/meshes/coal_wall.obj'))
    result=tester.test_full_pipeline()
    if not args.quick:
        result['reproducible']=tester.test_reproducibility()
        result['pressure_scan']=tester.test_parameter_sensitivity()
    (tester.output_dir/'integration_test_result.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))
    return 0


if __name__=='__main__':
    sys.exit(main())
