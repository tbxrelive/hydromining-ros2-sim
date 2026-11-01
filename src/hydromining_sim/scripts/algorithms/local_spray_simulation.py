"""Fine local Level Set erosion embedded in the original 3 m wall surface."""
from pathlib import Path

import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation
import trimesh

from .spray_simulation import SpraySimulation
from .perception.wall_frame import WallFrame


def surface_mesh(u, v, heights, thickness, back_heights=None):
    """Close a rectangular height surface with a matching back and side walls."""
    U, V = np.meshgrid(u, v)
    front = np.stack((U, V, heights), axis=-1).reshape(-1, 3)
    back = front.copy()
    back[:, 2] = (heights-thickness if back_heights is None else back_heights).ravel()
    rows, cols = heights.shape
    index = np.arange(rows*cols).reshape(rows, cols)
    a, b, c, d = index[:-1, :-1].ravel(), index[:-1, 1:].ravel(), index[1:, :-1].ravel(), index[1:, 1:].ravel()
    top = np.vstack((np.stack((a, b, c), axis=1), np.stack((b, d, c), axis=1)))
    bottom = top[:, ::-1] + len(front)
    edge = np.r_[index[0, :], index[1:, -1], index[-1, -2::-1], index[-2:0:-1, 0]]
    following = np.roll(edge, -1)
    sides = np.vstack((np.stack((edge+len(front), following+len(front), edge), axis=1),
                       np.stack((following+len(front), following, edge), axis=1)))
    return trimesh.Trimesh(vertices=np.vstack((front, back)),
                           faces=np.vstack((top, bottom, sides)), process=False)


def surface_rays(scene, u, v, ray_start=.6):
    """Read front heights by rays in the wall frame; no data reaches a policy."""
    U, V = np.meshgrid(u, v)
    rays = np.zeros((*U.shape, 6), dtype=np.float32)
    rays[..., 0], rays[..., 1], rays[..., 2], rays[..., 5] = U, V, ray_start, -1.
    distance = scene.cast_rays(o3d.core.Tensor(rays), nthreads=1)['t_hit'].numpy()
    return np.where(np.isfinite(distance), ray_start-distance, np.nan)


class LocalSpraySimulation(SpraySimulation):
    """Evolve a fine local volume and retain the entire surrounding wall mesh."""

    def __init__(self, source_mesh, source_pose, center, normal, output_dir, *,
                 dt=.2, wall_budget_s=120., spacing=.005, patch_extent, wall_thickness,
                 kernel=None):
        self.frame_ = WallFrame.from_normal(center, normal)
        self.wall_thickness_ = wall_thickness
        mesh = trimesh.load(source_mesh, force='mesh', process=True)
        rotation = Rotation.from_euler('xyz', source_pose[3:]).as_matrix()
        world = mesh.vertices @ rotation.T + source_pose[:3]
        local = self.frame_.to_local(world)
        scene = o3d.t.geometry.RaycastingScene(nthreads=1)
        scene.add_triangles(o3d.t.geometry.TriangleMesh(
            o3d.core.Tensor(np.asarray(local, np.float32)),
            o3d.core.Tensor(np.asarray(mesh.faces, np.uint32))))
        bounds = np.array([local.min(axis=0), local.max(axis=0)])
        self.patch_ = np.round(np.linspace(-patch_extent/2, patch_extent/2,
                                          round(patch_extent/spacing)+1), 9)
        # Preserve the full source wall while adding 5 mm vertices in the task patch.
        coarse_u = np.linspace(bounds[0, 0]+1e-5, bounds[1, 0]-1e-5, 101)
        coarse_v = np.linspace(bounds[0, 1]+1e-5, bounds[1, 1]-1e-5, 101)
        coarse_u = coarse_u[(coarse_u < self.patch_[0]) | (coarse_u > self.patch_[-1])]
        coarse_v = coarse_v[(coarse_v < self.patch_[0]) | (coarse_v > self.patch_[-1])]
        self.u_ = np.unique(np.round(np.r_[coarse_u, self.patch_], 9))
        self.v_ = np.unique(np.round(np.r_[coarse_v, self.patch_], 9))
        self.front_ = surface_rays(scene, self.u_, self.v_)
        patch_front = surface_rays(scene, self.patch_, self.patch_)
        if not np.isfinite(self.front_).all() or not np.isfinite(patch_front).all():
            raise ValueError('The source wall must cover the local simulation patch')
        # The fine patch must retain the source wall's full 40 cm thickness.
        patch_mesh = surface_mesh(self.patch_, self.patch_, patch_front, self.wall_thickness_)
        path = Path(output_dir) / 'local_initial.obj'
        path.parent.mkdir(parents=True, exist_ok=True)
        patch_mesh.export(path)
        extents = patch_mesh.extents
        resolution = tuple((np.ceil(extents/spacing).astype(int)+6).tolist())
        super().__init__(str(path), resolution, dt, wall_budget_s=wall_budget_s,
                         max_voxels=3_000_000, reinitialize_interval_s=.5, kernel=kernel)
        self.patch_u_indices_ = np.searchsorted(self.u_, self.patch_)
        self.patch_v_indices_ = np.searchsorted(self.v_, self.patch_)
        self.initial_patch_ = surface_rays(self.model.surface_scene(
            self.initial_phi, self.grid.origin, self.grid.spacing), self.patch_, self.patch_)

    def mesh(self, phi=None):
        """Project local surface retreat onto the full closed visual/collision mesh."""
        phi = self.grid.phi if phi is None else phi
        if phi is self.initial_phi or np.array_equal(phi, self.initial_phi):
            front = self.front_
        else:
            current = surface_rays(self.model.surface_scene(phi, self.grid.origin, self.grid.spacing),
                                   self.patch_, self.patch_)
            delta = current-self.initial_patch_
            # Fade numerical surface drift over the outer 3 cm, away from the task region.
            distance = self.patch_[-1]-np.abs(self.patch_)
            weight = np.clip(distance/.03, 0., 1.)
            weight = weight*weight*(3.-2.*weight)
            delta *= np.minimum(weight[:, None], weight[None, :])
            # Ray misses on the fixed boundary produce NaN even after multiplication by zero.
            delta[[0, -1], :] = 0.
            delta[:, [0, -1]] = 0.
            front = self.front_.copy()
            front[np.ix_(self.patch_v_indices_, self.patch_u_indices_)] += delta
        mesh = surface_mesh(self.u_, self.v_, front, self.wall_thickness_, self.front_-self.wall_thickness_)
        return np.asarray(mesh.vertices), np.asarray(mesh.faces)
