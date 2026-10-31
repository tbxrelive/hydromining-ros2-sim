#!/usr/bin/env python3
"""阶段 6.0：求解分辨率敏感性（离线，无 Gazebo）。单进程测一个分辨率，输出 JSON。"""
import argparse, json, resource, sys, time
from pathlib import Path
import numpy as np
import open3d as o3d
ws = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ws / 'src/hydromining_sim/scripts'))
from algorithms.spray_simulation import SpraySimulation
from algorithms.erosion_model import SprayAction
from algorithms.voxel_grid import VoxelGrid

MESH = ws / 'src/hydromining_sim/models/coal_wall/meshes/coal_wall.obj'
POS = [-0.4610369606053094, -1.4578341658855187e-10, 1.0723548361037436]
DIR = [-0.34202018572081666, 5.264312857161668e-10, -0.9396926053553356]


def retreat_map(sim, before, after, half=0.15, n=151):
    """沿射流方向逐射线的首次命中距离差（喷后-喷前），正为退缩。"""
    def scene(phi):
        v, f = sim.mesh(phi)
        s = o3d.t.geometry.RaycastingScene()
        s.add_triangles(o3d.core.Tensor(np.ascontiguousarray(v, np.float32)), o3d.core.Tensor(np.ascontiguousarray(f, np.uint32)))
        return s
    p, u = np.array(POS), np.array(DIR)
    e1 = np.cross(u, [0, 1, 0]); e1 /= np.linalg.norm(e1); e2 = np.cross(u, e1)
    g = np.linspace(-half, half, n); X, Y = np.meshgrid(g, g)
    o = p + X.reshape(-1, 1) * e1 + Y.reshape(-1, 1) * e2
    rays = o3d.core.Tensor(np.c_[o, np.tile(u, (len(o), 1))].astype(np.float32))
    ta = scene(before).cast_rays(rays)['t_hit'].numpy()
    tb = scene(after).cast_rays(rays)['t_hit'].numpy()
    return tb - ta, np.hypot(X, Y).ravel()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--res', type=int, nargs=3, required=True)
    ap.add_argument('--dt', type=float, default=0.2)
    ap.add_argument('--duration', type=float, default=12.0)
    a = ap.parse_args()
    if np.prod(a.res, dtype=np.int64) > VoxelGrid.MAX_VOXELS:
        ap.error('resolution exceeds the existing 1,000,000-sample protection')
    t0 = time.monotonic()
    sim = SpraySimulation(str(MESH), tuple(a.res), a.dt)
    t_init = time.monotonic() - t0
    before = sim.grid.phi.copy()
    t1 = time.monotonic()
    try:
        phi, stats = sim.simulate(SprayAction(POS, DIR, 20.0, a.duration, 1))
    except Exception as e:
        print(json.dumps({'res': a.res, 'error': str(e), 'init_s': t_init, 'sim_s': time.monotonic() - t1,
                          'peak_rss_mib': resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024}))
        return
    t_sim = time.monotonic() - t1
    dd, R = retreat_map(sim, before, phi)
    out = {'res': a.res, 'spacing_m': stats['spacing_m'], 'init_s': t_init, 'sim_s': t_sim, 'steps': stats['steps'],
           'peak_rss_mib': resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024,
           'volume_cm3': stats['volume_removed_m3'] * 1e6, 'max_depth_cm': float(np.nanmax(dd)) * 100,
           'frac_ge_2cm': {f'D{int(2 * r * 100)}cm': float(np.mean(dd[R <= r] >= 0.02)) for r in (0.05, 0.07, 0.10)},
           'radial_mean_cm': [float(np.mean(dd[(R >= r) & (R < r + .01)]) * 100) for r in np.arange(0, .12, .01)]}
    print(json.dumps(out))


if __name__ == '__main__':
    main()
