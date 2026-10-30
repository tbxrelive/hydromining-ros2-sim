#!/usr/bin/env python3
"""阶段 6.0：离线设计喷射工况，沿射流轴初步筛选，不替代固定法向的最终判决。"""
import json, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
import measure_resolution as M
from algorithms.spray_simulation import SpraySimulation
from algorithms.erosion_model import SprayAction
p, u = np.array(M.POS), np.array(M.DIR)
e1 = np.cross(u, [0, 1, 0]); e1 /= np.linalg.norm(e1); e2 = np.cross(u, e1)


def ring(n, r):
    return [(0.0, 0.0)] + [(r * np.cos(2 * np.pi * k / n), r * np.sin(2 * np.pi * k / n)) for k in range(n)] if n else [(0.0, 0.0)]


PLANS = {'single': [(0, 0)], 'tri_r5': [(5e-2 * np.cos(2*np.pi*k/3), 5e-2 * np.sin(2*np.pi*k/3)) for k in range(3)],
         'hex_r6': ring(6, 0.06), 'hex_r7': ring(6, 0.07), 'hex2_r5_r9': ring(6, 0.05) + [(0.09*np.cos(2*np.pi*k/8), 0.09*np.sin(2*np.pi*k/8)) for k in range(8)]}
res = {}
for name in (sys.argv[1:] or PLANS):
    sim = SpraySimulation(str(M.MESH), (96, 96, 64), 0.2)
    before = sim.grid.phi.copy()
    for a, b in PLANS[name]:
        phi, _ = sim.simulate(SprayAction(p + a * e1 + b * e2, u, 20.0, 12.0, 1))
        sim.commit(phi)
    dd, R = M.retreat_map(sim, before, sim.grid.phi)
    m = R <= 0.10
    res[name] = {'shots': len(PLANS[name]), 'frac_ge_2cm_D20': float(np.mean(dd[m] >= 0.02)),
                 'frac_ge_1cm_D20': float(np.mean(dd[m] >= 0.01)), 'min_retreat_cm_D20': float(np.nanmin(dd[m]) * 100),
                 'max_cm': float(np.nanmax(dd) * 100)}
    print(name, json.dumps(res[name]), flush=True)
