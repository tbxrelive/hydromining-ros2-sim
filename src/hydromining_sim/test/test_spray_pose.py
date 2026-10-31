"""喷射位姿生成的几何解析回归，不需要启动 ROS 或 Gazebo。"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from algorithms.planning.spray_pose import aim_rotation, solve_spray_pose, tcp_target_world
from algorithms.perception.kinematics import Chain

URDF = """<robot name="test">
  <joint name="slide" type="revolute">
    <parent link="base"/><child link="arm"/><axis xyz="0 0 1"/>
    <limit lower="-2.9" upper="2.9"/>
  </joint>
  <joint name="tip" type="fixed">
    <parent link="arm"/><child link="tcp"/><origin xyz="1 0 0"/>
  </joint>
</robot>"""


def test_aim_rotation_z_axis_and_handedness():
    direction = np.array([0.5, 0.4, -0.6])
    rotation = aim_rotation(direction)
    np.testing.assert_allclose(rotation[:, 2], direction / np.linalg.norm(direction), atol=1e-12)
    assert np.linalg.det(rotation) == pytest.approx(1)
    np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-12)


def test_aim_rotation_rejects_degenerate_and_parallel_up():
    with pytest.raises(ValueError):
        aim_rotation([0, 0, 0])
    with pytest.raises(ValueError):
        aim_rotation([0, 0, np.nan])
    rotation = aim_rotation([0, 0, 1])
    np.testing.assert_allclose(rotation[:, 2], [0, 0, 1], atol=1e-12)
    assert np.linalg.det(rotation) == pytest.approx(1)


def test_tcp_target_places_nozzle_at_standoff():
    target = [1.5, 0, 1.5]
    direction = np.array([0.94, 0, 0.34])
    direction /= np.linalg.norm(direction)
    pose = tcp_target_world(target, direction, 0.6)
    nozzle = pose[:3, 3] - 0.05 * pose[:3, :3] @ np.array([0, 0, 1])
    np.testing.assert_allclose(nozzle, np.asarray(target) - 0.6 * direction, atol=1e-12)
    np.testing.assert_allclose(pose[:3, :3] @ np.array([0, 0, 1]), direction, atol=1e-12)


def test_tcp_target_rejects_invalid_inputs():
    with pytest.raises(ValueError):
        tcp_target_world([0, 0], [0, 0, 1], 0.5)
    with pytest.raises(ValueError):
        tcp_target_world([0, 0, 0], [0, 0, 1], 0.04)  # standoff 小于喷嘴偏移
    with pytest.raises(ValueError):
        tcp_target_world([0, 0, 0], [0, 0, 1], np.nan)


def test_solve_spray_pose_reports_reachability_and_world_target():
    chain = Chain(URDF, 'base', 'tcp')
    T_world_base = np.eye(4)
    # tip 只能沿 +X 半径 1 处、z 轴随关节旋转，选 world +X 的脱靶方向无解。
    solved = solve_spray_pose(chain, T_world_base, [1, 0, 0], [0, 0, 1], 0.5, [0.0])
    desired = solved['tcp_world']
    np.testing.assert_allclose(desired[:3, :3], aim_rotation([0, 0, 1]), atol=1e-12)
    # 单关节无法同时满足位置与姿态时，误差如实上报、可达性为假或真都由误差决定。
    assert solved['reachable'] == (solved['position_error_m'] <= 0.005 and
                                 solved['rotation_error_rad'] <= 0.02)
    np.testing.assert_allclose(solved['joints'], np.clip(solved['joints'], -2.85, 2.85))
