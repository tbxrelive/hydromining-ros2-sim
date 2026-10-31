#!/usr/bin/env python3
"""Positive-speed retreat, equation (12), with a physical Godunov upwind norm."""
import numpy as np


class LevelSetEvolver:
    """Advance phi_t = v |grad(phi)| for nonnegative extended surface speed."""

    def __init__(self, dt=0.1):
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError('dt must be positive and finite')
        self.dt = float(dt)

    @staticmethod
    def compute_gradient(phi, spacing=1.0):
        """Physical centred gradient for normals, not the time update."""
        return np.stack(np.gradient(phi, spacing, edge_order=1), axis=-1)

    @staticmethod
    def compute_gradient_norm(phi, spacing=1.0):
        """Physical gradient magnitude."""
        return np.linalg.norm(LevelSetEvolver.compute_gradient(phi, spacing), axis=-1)

    @staticmethod
    def upwind_norm(phi, spacing):
        """Godunov selection for Hamiltonian H=-v|grad phi|, v>=0."""
        squared = np.zeros_like(phi)
        for axis in range(3):
            pad = [(0, 0)]*3
            pad[axis] = (1, 1)
            p = np.pad(phi, pad, mode='edge')
            middle = [slice(None)]*3
            before = middle.copy(); after = middle.copy()
            middle[axis] = slice(1, -1)
            before[axis] = slice(0, -2); after[axis] = slice(2, None)
            dm = (p[tuple(middle)]-p[tuple(before)])/spacing
            dp = (p[tuple(after)]-p[tuple(middle)])/spacing
            squared += np.maximum(np.minimum(dm, 0)**2, np.maximum(dp, 0)**2)
        return np.sqrt(squared)

    def check_cfl_stability(self, v_e_max, spacing):
        """Conservative 3D CFL bound for the explicit update."""
        return v_e_max <= 0 or self.dt*v_e_max/spacing <= 0.3

    def evolve_step(self, phi, v_e, spacing=1.0, dt=None):
        """One monotone step; caller may shorten the final step to end exactly."""
        step = self.dt if dt is None else float(dt)
        if step <= 0 or spacing <= 0 or not np.all(np.isfinite(v_e)) or np.any(v_e < 0):
            raise ValueError('Invalid dt, spacing or erosion velocity')
        if step*float(np.max(v_e))/spacing > 0.3+1e-10:
            raise ValueError('CFL bound exceeded')
        return (phi+step*v_e*self.upwind_norm(phi, spacing)).astype(np.float32)
