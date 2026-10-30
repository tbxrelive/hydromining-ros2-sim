#!/usr/bin/env python3
"""Transactional state shared by the ROS adapter and offline regression tests."""
import numpy as np
import math
import time
from .voxel_grid import VoxelGrid
from .erosion_model import ErosionModel, DefaultErosionKernel
from .level_set_evolver import LevelSetEvolver
from .geometry_utils import GeometryUtils
from .mesh_converter import MeshConverter


class SpraySimulation:
    """Keep initial/current coal separately; failed requests never commit state."""

    def __init__(self, mesh_path, resolution=(96, 96, 64), dt=0.2, kernel=None,
                 wall_budget_s=30.):
        self.grid = VoxelGrid.from_mesh_simple(mesh_path, resolution)
        self.initial_phi = self.grid.phi.copy()
        self.model = ErosionModel(kernel or DefaultErosionKernel())
        self.evolver = LevelSetEvolver(dt)
        if not np.isfinite(wall_budget_s) or wall_budget_s <= 0:
            raise ValueError('wall_budget_s must be finite and positive')
        self.wall_budget_ = float(wall_budget_s)
        self.shots = 0

    def reset(self):
        """Restore the exact initial numerical state."""
        self.grid.phi = self.initial_phi.copy()
        self.shots = 0

    def simulate(self, action, cancelled=None):
        """Return a candidate field and diagnostics; caller commits after mesh update."""
        g = self.grid
        before = g.phi
        phi = before.copy()
        points = g.points
        elapsed = 0.0
        max_speed = 0.0
        steps = 0
        radius = 0.0
        started = time.monotonic()
        if math.ceil(action.duration/self.evolver.dt) > 600:
            raise ValueError('Request exceeds 600 integration steps; increase dt or shorten duration')
        while elapsed < action.duration-1e-10:
            if cancelled is not None and cancelled():
                raise RuntimeError('Spray cancelled; candidate field discarded')
            if time.monotonic()-started > self.wall_budget_:
                raise ValueError(f'Computation exceeded the {self.wall_budget_:.0f}-second '
                                 'wall-time budget; state unchanged')
            speed = self.model.compute_erosion_velocity(points, action, phi, g.origin, g.spacing)
            vmax = float(speed.max())
            if self.model.last_hit_distance is not None:
                radius = max(radius, self.model.kernel.b(action.pressure, self.model.last_hit_distance))
            max_speed = max(max_speed, vmax)
            step = min(self.evolver.dt, action.duration-elapsed,
                       0.3*g.spacing/max(vmax, 1e-12))
            phi = self.evolver.evolve_step(phi, speed, g.spacing, dt=step)
            elapsed += step
            steps += 1
            if steps > 600:
                raise ValueError('Step budget exceeded; lower pressure or duration')
        depth = GeometryUtils.surface_depth(before, phi, g.origin, g.spacing, action, radius) if radius else 0.0
        cumulative_depth = GeometryUtils.surface_depth(self.initial_phi, phi, g.origin, g.spacing, action, radius) if radius else 0.0
        stats = {'volume_removed_m3': GeometryUtils.compute_volume_removed(before, phi, g.spacing),
                 'cumulative_volume_m3': GeometryUtils.compute_volume_removed(self.initial_phi, phi, g.spacing),
                 'max_depth_m': depth, 'cumulative_depth_m': cumulative_depth,
                 'elapsed_s': elapsed, 'steps': steps, 'max_speed_m_s': max_speed,
                 'spacing_m': g.spacing, 'shot': self.shots+1,
                 'calibrated': False}
        return phi, stats

    def mesh(self, phi=None):
        """Extract exactly the surface used by the numerical solver."""
        g = self.grid
        return MeshConverter.voxel_to_mesh(g.phi if phi is None else phi, g.origin, g.spacing)

    def commit(self, phi):
        """Commit after the caller has successfully updated the simulator."""
        self.grid.phi = phi.copy()
        self.shots += 1
