"""辅助 TSDF optical 外参、几何位置与资源边界验证。"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from algorithms.perception.tsdf_mesh import TSDFReconstructor
from algorithms.perception.wall_frame import WallFrame


def test_gazebo_x_forward_depth_integrates_wall_at_correct_world_location():
    frame = WallFrame.from_normal([1., 0, 0], [-1., 0, 0])
    snapshot = {
        'depth': np.ones((60, 80), np.float32),
        'K': np.array([[100., 0, 39.5], [0, 100., 29.5], [0, 0, 1.]]),
        'tf_depth': np.array([0., 0, 0, 0, 0, 0, 1.]),
    }
    tsdf = TSDFReconstructor(voxel_size=0.01, maximum_blocks=512, maximum_frames=2)
    tsdf.integrate(snapshot, frame, extent=0.4)
    tsdf.integrate(snapshot, frame, extent=0.4)
    mesh = tsdf.mesh()
    assert len(mesh.vertices) > 20 and len(mesh.triangles) > 20
    vertices = np.asarray(mesh.vertices)
    assert np.median(np.abs(vertices[:, 0] - 1)) < 0.005
    assert np.ptp(vertices[:, 1]) > 0.2 and np.ptp(vertices[:, 2]) > 0.2
    with pytest.raises(ValueError, match='frame budget'):
        tsdf.integrate(snapshot, frame)


def test_tsdf_limits_are_not_silently_expanded():
    with pytest.raises(ValueError):
        TSDFReconstructor(voxel_size=0.0001)
    frame = WallFrame.from_normal([1., 0, 0], [-1., 0, 0])
    tsdf = TSDFReconstructor(voxel_size=0.005, maximum_blocks=1)
    snapshot = {'depth': np.ones((60, 80)), 'K': np.eye(3),
                'tf_depth': np.array([0., 0, 0, 0, 0, 0, 1.])}
    with pytest.raises(MemoryError, match='block budget'):
        tsdf.integrate(snapshot, frame, extent=20)
