#!/usr/bin/env python3
"""Bounded-memory, triangle-based signed distance grid (negative in coal)."""
import numpy as np
import math
import trimesh
from scipy.ndimage import map_coordinates


class VoxelGrid:
    """Uniform grid with explicit origin and physical spacing in metres."""

    MAX_VOXELS = 1_000_000

    def __init__(self, nx, ny, nz, origin, spacing, max_voxels=MAX_VOXELS):
        self.resolution = tuple(int(v) for v in (nx, ny, nz))
        if min(self.resolution) < 2 or math.prod(self.resolution) > max_voxels:
            raise ValueError(f'Grid must contain 2..{max_voxels:,} samples')
        if not np.isfinite(spacing) or spacing <= 0:
            raise ValueError('spacing must be positive and finite')
        self.nx, self.ny, self.nz = self.resolution
        self.origin = np.asarray(origin, dtype=np.float64)
        self.spacing = float(spacing)
        self.phi = np.ones(self.resolution, dtype=np.float32)

    @classmethod
    def from_mesh_simple(cls, mesh_path, resolution=(96, 96, 64), chunk_size=8192,
                         max_voxels=MAX_VOXELS):
        """Preserve the legacy entry point; use BVH triangle queries, never NxM cdist.

        Seam welding is limited to 1e-7 m; open meshes are rejected. Padding keeps
        the zero set away from the boundary. Open3D and numerical workers use one
        thread. Samples are grid nodes, not inconsistently offset voxel centres.
        """
        import open3d as o3d
        mesh = trimesh.load(str(mesh_path), force='mesh', process=True)
        # OBJ seams may duplicate positions with different shading normals/UVs.
        # Geometry welding must ignore those attributes before watertight checks.
        mesh.merge_vertices(digits_vertex=7, merge_norm=True, merge_tex=True)
        mesh.remove_unreferenced_vertices()
        if not mesh.is_watertight:
            raise ValueError('Coal mesh must be watertight after seam welding')
        resolution = np.asarray(resolution, dtype=int)
        if resolution.shape != (3,) or np.any(resolution < 8):
            raise ValueError('resolution must have three dimensions >= 8')
        h = float(np.max(mesh.extents / (resolution - 6)))
        grid = cls(*resolution, mesh.bounds[0] - 2.37*h, h, max_voxels=max_voxels)
        points = grid.points.reshape(-1, 3)
        legacy = o3d.geometry.TriangleMesh(
            o3d.utility.Vector3dVector(mesh.vertices),
            o3d.utility.Vector3iVector(mesh.faces.astype(np.int32)))
        scene = o3d.t.geometry.RaycastingScene(nthreads=1)
        scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(legacy))
        values = grid.phi.reshape(-1)
        chunk_size = min(max(int(chunk_size), 1), 32768)
        for start in range(0, len(points), chunk_size):
            query = o3d.core.Tensor(points[start:start+chunk_size].astype(np.float32))
            values[start:start+chunk_size] = scene.compute_signed_distance(
                query, nthreads=1, nsamples=3).numpy()
        grid.source_mesh = mesh
        return grid

    @property
    def points(self):
        """Grid-node coordinates in the mesh coordinate frame."""
        return np.moveaxis(np.indices(self.resolution), 0, -1)*self.spacing + self.origin

    def sample(self, points, phi=None):
        """Trilinear interpolation; samples outside the grid are exterior."""
        p = np.asarray(points)
        coords = ((p.reshape(-1, 3) - self.origin)/self.spacing).T
        values = map_coordinates(self.phi if phi is None else phi, coords,
                                 order=1, mode='constant', cval=1e3)
        return values.reshape(p.shape[:-1])

    def info(self):
        """Geometry and storage diagnostics."""
        interior = int(np.count_nonzero(self.phi <= 0))
        return {'resolution': self.resolution, 'total_voxels': int(self.phi.size),
                'interior_voxels': interior, 'exterior_voxels': int(self.phi.size)-interior,
                'interior_volume_m3': interior*self.spacing**3,
                'spacing_m': self.spacing, 'bounds_min': self.origin,
                'bounds_max': self.origin+(np.array(self.resolution)-1)*self.spacing,
                'phi_min': float(self.phi.min()), 'phi_max': float(self.phi.max()),
                'phi_mean': float(self.phi.mean()), 'phi_std': float(self.phi.std())}
