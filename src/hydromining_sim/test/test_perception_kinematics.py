"""用解析运动链验证离线 FK/IK，不需要启动 ROS 或 Gazebo。"""
import sys
from pathlib import Path
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from algorithms.perception.kinematics import Chain, look_at

URDF = """<robot name="test">
  <joint name="angle" type="revolute">
    <parent link="base"/><child link="arm"/><axis xyz="0 0 1"/>
    <limit lower="-2.9" upper="2.9"/>
  </joint>
  <joint name="tip" type="fixed">
    <parent link="arm"/><child link="tip"/><origin xyz="1 0 0"/>
  </joint>
  <ros2_control name="ignored" type="system"><joint name="angle"/></ros2_control>
</robot>"""


def test_fixed_offset_rotates_with_revolute_joint_and_control_tags_are_ignored():
    chain = Chain(URDF, 'base', 'tip')
    assert chain.names_ == ['angle']
    np.testing.assert_allclose(chain.fk([np.pi / 2])[:3, 3], [0, 1, 0], atol=1e-12)


def test_ik_reproduces_reachable_pose():
    chain = Chain(URDF, 'base', 'tip')
    target = chain.fk([0.8])
    q, position_error, orientation_error = chain.ik(target, [0.0])
    assert position_error < 1e-6 and orientation_error < 1e-6
    assert q[0] == pytest.approx(0.8, abs=1e-6)


def test_look_at_gazebo_axis_and_handedness():
    pose = look_at([0, 0, 1], [1, 0, 1])
    np.testing.assert_allclose(pose[:3, :3], np.eye(3))
    assert np.linalg.det(pose[:3, :3]) == pytest.approx(1)


@pytest.mark.parametrize('q', [[], [0, 0], [np.nan]])
def test_joint_vector_validation(q):
    with pytest.raises(ValueError):
        Chain(URDF, 'base', 'tip').fk(q)


def test_unknown_or_unsupported_chain_rejected():
    with pytest.raises(ValueError):
        Chain(URDF, 'missing', 'tip')
    with pytest.raises(ValueError):
        Chain(URDF.replace('type="revolute"', 'type="prismatic"'), 'base', 'tip')
