"""坐标与距离语义的解析回归，防止 frame 标签与数值坐标混淆。"""
import sys
from pathlib import Path
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from algorithms.perception.frames import (
    apply_range_measurements, backproject_depth, fit_reference_plane, transform_points)


def test_backprojection_conventions_and_invalid_pixels():
    depth = np.array([[2.0, 2.0, np.nan], [2.0, 0.0, -1.0]])
    k = np.array([[2.0, 0, 0], [0, 2.0, 0], [0, 0, 1.0]])
    gazebo = backproject_depth(depth, k)
    optical = backproject_depth(depth, k, 'optical')
    np.testing.assert_allclose(gazebo[0, 1], [2, -1, 0])
    np.testing.assert_allclose(optical[0, 1], [1, 0, 2])
    np.testing.assert_allclose(gazebo[1, 0], [2, 0, -1])
    assert np.isnan(gazebo[0, 2]).all()
    assert np.isnan(gazebo[1, 1:]).all()


def test_transform_preserves_shape_and_invalid_points():
    points = np.array([[[1.0, 0, 0], [np.nan, 0, 0]]])
    pose = [2, 3, 4, *Rotation.from_euler('z', np.pi / 2).as_quat()]
    world = transform_points(points, pose)
    assert world.shape == points.shape
    np.testing.assert_allclose(world[0, 0], [2, 4, 4], atol=1e-12)
    assert np.isnan(world[0, 1]).all()


def test_depth_is_axial_not_ray_distance():
    points = backproject_depth(np.ones((1, 2)), np.eye(3))
    assert points[0, 1, 0] == 1.0
    assert np.linalg.norm(points[0, 1]) == pytest.approx(np.sqrt(2))


def test_replace_range_along_ray():
    points = np.array([[3, 4, 0], [0, 0, 0], [0, 0, 2]])
    output = apply_range_measurements(points, [10, 1, np.nan])
    np.testing.assert_allclose(output[0], [6, 8, 0])
    assert np.isnan(output[1:]).all()


@pytest.mark.parametrize('k', [np.zeros((3, 3)), np.eye(2), np.full((3, 3), np.nan)])
def test_invalid_intrinsics_rejected(k):
    with pytest.raises(ValueError):
        backproject_depth(np.ones((2, 2)), k)


def test_invalid_pose_and_convention_rejected():
    with pytest.raises(ValueError):
        transform_points([[1, 2, 3]], [0] * 7)
    with pytest.raises(ValueError):
        backproject_depth(np.ones((2, 2)), np.eye(3), 'unknown')


def test_reference_plane_is_fixed_from_observations_and_oriented_to_sensor():
    x, y = np.meshgrid(np.linspace(-1, 1, 20), np.linspace(-1, 1, 20))
    points = np.c_[x.ravel(), y.ravel(), np.full(x.size, 2.0)]
    points = np.vstack([points, [[0, 0, 4], [0.1, 0.1, 4]]])
    origin, normal, _ = fit_reference_plane(points, [0, 0, 1], [0, 0, 0])
    np.testing.assert_allclose(origin, [0, 0, 2], atol=1e-10)
    np.testing.assert_allclose(normal, [0, 0, -1], atol=1e-10)


def test_degenerate_reference_plane_rejected():
    with pytest.raises(ValueError):
        fit_reference_plane(np.zeros((30, 3)), [0, 0, 0], [1, 0, 0])
