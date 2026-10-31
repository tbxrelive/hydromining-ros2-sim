"""真实 rclpy 生命周期和输入验证；运动/传感器端到端另行验收。"""
import sys
from pathlib import Path

import pytest
import rclpy
from rclpy.action import GoalResponse
from rclpy.parameter import Parameter

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/hydromining_robot/scripts'))
from wall_perception_node import WallPerceptionNode
from hydromining_robot.action import AssessRegion
from hydromining_robot.msg import ObservationView


@pytest.fixture
def node(tmp_path):
    rclpy.init()
    instance = WallPerceptionNode()
    instance.set_parameters([Parameter('output_directory', value=str(tmp_path / 'reports'))])
    yield instance
    instance.release_resources()
    instance.destroy_node()
    rclpy.shutdown()


def goal():
    request = AssessRegion.Goal()
    request.center.x, request.center.y, request.center.z = 1.517, 0.011, 0.694
    request.diameter, request.required_depth = 0.2, 0.02
    request.required_fraction, request.confidence_k = 0.9, 1.645
    request.frames_per_view = 3
    request.new_baseline = True
    request.viewpoints = [ObservationView(name='test', joints=[0.] * 7)]
    return request


def test_external_lifecycle_configure_activate_deactivate_cleanup(node):
    assert node._state_machine.current_state[1] == 'unconfigured'
    assert node.accept_goal(goal()) == GoalResponse.REJECT
    node.trigger_configure()
    assert node._state_machine.current_state[1] == 'inactive'
    assert not node.active_
    assert node.accept_goal(goal()) == GoalResponse.REJECT
    node.trigger_activate()
    assert node._state_machine.current_state[1] == 'active'
    node.trigger_deactivate()
    assert node._state_machine.current_state[1] == 'inactive'
    node.trigger_cleanup()
    assert node._state_machine.current_state[1] == 'unconfigured'
    assert node.observation_ is None and node.server_ is None and node.session_ is None


def test_single_goal_reservation_and_invalid_goals(node):
    node.trigger_configure()
    node.trigger_activate()
    request = goal()
    request.new_baseline = False
    assert node.accept_goal(request) == GoalResponse.REJECT
    request.new_baseline = True
    request.frames_per_view = 31
    assert node.accept_goal(request) == GoalResponse.REJECT
    request.frames_per_view = 3
    request.required_fraction = 1.1
    assert node.accept_goal(request) == GoalResponse.REJECT
    request.required_fraction = 0.9
    assert node.accept_goal(request) == GoalResponse.ACCEPT
    assert node.accept_goal(request) == GoalResponse.REJECT
    assert not node.set_parameters([Parameter('maximum_sigma', value=0.008)])[0].successful
    node.busy_ = False
    node.trigger_deactivate()


def test_parameters_are_bounded_and_geometry_is_not_mutated_while_configured(node):
    assert not node.set_parameters([Parameter('noise_scale', value=-1.)])[0].successful
    assert not node.set_parameters([Parameter('sensor_timeout_s', value=0.)])[0].successful
    node.trigger_configure()
    assert not node.set_parameters([Parameter('resolution', value=0.002)])[0].successful
    assert node.set_parameters([Parameter('maximum_sigma', value=0.008)])[0].successful
    node.trigger_cleanup()
