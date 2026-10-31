#!/usr/bin/env python3
"""阶段六真实无 GUI 验收：Gazebo、机械臂、喷射、感知 Action 与独立真值核对。

只管理自己启动的进程组；同一个验收脚本复用阶段四的内存/清理保护。
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import threading
from pathlib import Path

import numpy as np
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from lifecycle_msgs.srv import GetState
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters

from hydromining_robot.action import AssessRegion
from hydromining_robot.msg import CoalWallState, ObservationView
from hydromining_robot.srv import SprayAction
from test_phase4_acceptance import AcceptanceTest, cleanup_acceptance

sys.path.insert(0, get_package_share_directory('hydromining_sim'))
from algorithms.perception.metrics import mesh_heights, truth_change
from algorithms.perception.wall_frame import WallFrame

POS = np.array([-0.4610369606053094, -1.4578341658855187e-10, 1.0723548361037436])
DIR = np.array([-0.34202018572081666, 5.264312857161668e-10, -0.9396926053553356])
E1 = np.cross(DIR, [0, 1, 0]); E1 /= np.linalg.norm(E1); E2 = np.cross(DIR, E1)
RING = lambda n, r: [(r * np.cos(2 * np.pi * k / n), r * np.sin(2 * np.pi * k / n)) for k in range(n)]
SHOTS = {'none': [], 'single': [(0., 0.)], 'tri_r5': RING(3, .05), 'hex_r6': [(0., 0.)] + RING(6, .06)}
MESH_POSE = [1.5, 0., 1.5, 0., -1.9198621771937625, 0.]


class Phase6Acceptance(AcceptanceTest):
    """发出真实 Action，并只用归档网格独立核对 Action 结果。"""

    def __init__(self, output, process, views, center, frames_per_view=3):
        super().__init__(output, process)
        self.views_, self.center_, self.frames_per_view_ = views, center, frames_per_view
        self.perception_state_ = self.create_client(GetState, '/wall_perception_node/get_state')
        self.perception_params_ = self.create_client(SetParameters, '/wall_perception_node/set_parameters')
        self.assess_ = ActionClient(self, AssessRegion, '/coal_wall/assess_region')
        self.map_messages_ = []
        self.create_subscription(CoalWallState, '/coal_wall/state', self.map_messages_.append, 10)
        self.feedback_ = []
        self.baseline_mesh_ = None

    def active(self, client):
        """要求外部 launch 已完成 configure/activate，而不是仅发现服务。"""
        self.wait(client.service_is_ready, 90, 'lifecycle service missing')
        state = self.future(client.call_async(GetState.Request()), 10)
        return state.current_state.id == 3

    def reset(self):
        reply = self.future(self.reset_.call_async(Trigger.Request()), 60)
        assert reply.success, reply.message
        return Path(reply.message)

    def spray(self, scenario):
        mesh = None
        for a, b in SHOTS[scenario]:
            p = POS + a * E1 + b * E2
            request = SprayAction.Request(use_tcp=False, pressure=20., duration=12., enabled=True)
            request.position_x, request.position_y, request.position_z = p.tolist()
            request.direction_x, request.direction_y, request.direction_z = DIR.tolist()
            reply = self.future(self.spray_.call_async(request), 180)
            assert reply.success, reply.message
            mesh = Path(reply.output_mesh_path)
        return mesh

    def goal(self, new_baseline, view_names=None, frames=3):
        goal = AssessRegion.Goal()
        goal.center.x, goal.center.y, goal.center.z = self.center_
        goal.diameter, goal.required_depth = .2, .02
        goal.required_fraction, goal.confidence_k = .9, 1.645
        goal.frames_per_view, goal.new_baseline = frames or self.frames_per_view_, new_baseline
        goal.viewpoints = [ObservationView(name=name, joints=self.views_[name]['joints'])
                           for name in (view_names or list(self.views_))]
        return goal

    def send(self, goal):
        self.wait(self.assess_.server_is_ready, 30, 'assessment action missing')
        return self.future(self.assess_.send_goal_async(goal, feedback_callback=self.feedback_.append), 20)

    def assess(self, goal, timeout=420):
        handle = self.send(goal)
        assert handle.accepted
        result = self.future(handle.get_result_async(), timeout)
        assert result.status == GoalStatus.STATUS_SUCCEEDED, (result.status, result.result.message)
        return result.result

    def truth_summary(self, result, mesh):
        """网格只在验收脚本中使用；与感知 Action 输入隔离。"""
        with np.load(result.state_path) as state:
            frame = WallFrame(state['origin'], state['axes'])
        before = mesh_heights(self.baseline_mesh_, MESH_POSE, frame, .24, .002)
        after = mesh_heights(mesh, MESH_POSE, frame, .24, .002)
        return truth_change(before, after, frame, .24, .002, .1, .02, .9)

    def run(self):
        self.wait(lambda: self.active(self.state_) and self.active(self.perception_state_), 120,
                  'lifecycle nodes did not become active')
        if self.noise_mode_:
            self.wait(self.perception_params_.service_is_ready, 30, 'perception set_parameters missing')
            reply = self.future(self.perception_params_.call_async(SetParameters.Request(
                parameters=[Parameter('maximum_sigma', value=.03).to_parameter_msg()])), 20)
            assert reply.results[0].successful, reply.results[0].reason
        self.wait(self.motion_.server_is_ready, 90, 'trajectory action missing')
        self.baseline_mesh_ = self.reset()
        invalid = self.goal(True)
        invalid.frames_per_view = 31
        rejected = self.send(invalid)
        assert not rejected.accepted
        before = self.assess(self.goal(True))
        assert before.assessment == 'baseline_ready' and Path(before.report_path).is_file()
        unchanged = self.assess(self.goal(False))
        # 噪声模式下可靠性门限更紧；未达标区域宁可证据不足，也不得误达。
        assert unchanged.assessment in ('not_achieved', 'insufficient_evidence'), unchanged
        busy_goal = self.send(self.goal(False))
        assert busy_goal.accepted
        second = self.send(self.goal(False))
        assert not second.accepted, 'concurrent assessment must be rejected'
        busy_goal.cancel_goal_async()
        cancelled = self.future(busy_goal.get_result_async(), 120)
        assert cancelled.status == GoalStatus.STATUS_CANCELED, cancelled.status
        report = {'unchanged': json.loads(Path(unchanged.report_path).read_text())['assessment']}
        for scenario in ('single', 'tri_r5', 'hex_r6'):
            mesh = self.spray(scenario)
            result = self.assess(self.goal(False))
            truth = self.truth_summary(result, mesh)
            report[scenario] = {'predicted': result.assessment, 'truth': truth, 'coverage': result.coverage,
                                'fraction_lower': result.fraction_lower, 'fraction_upper': result.fraction_upper,
                                'volume_m3': result.removed_volume_m3, 'external_alarm': result.external_alarm,
                                'report_path': result.report_path}
            assert not (truth['status'] == 'not_achieved' and result.assessment == 'achieved'), report[scenario]
            self.reset()
        # 噪声模式下六角多喷仍可证据不足；只有单/三点绝不能误达、七点绝不能误未达。
        assert report['single']['predicted'] in ('not_achieved', 'insufficient_evidence')
        assert report['tri_r5']['predicted'] in ('not_achieved', 'insufficient_evidence')
        assert report['hex_r6']['predicted'] in ('achieved', 'insufficient_evidence')
        assert self.map_messages_ and self.map_messages_[-1].assessment == report['hex_r6']['predicted']
        assert self.feedback_
        assert self.max_joint_drift_ < .002 or not hasattr(self, 'fixed_joints_')
        return {'passed': True, 'scenarios': report, 'map_messages': len(self.map_messages_),
                'feedback_messages': len(self.feedback_), 'calibrated': False,
                'aggregate_peak_rss_mib': self.peak_rss_ / 1024 ** 2, 'GUI_verified': False}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--depth-noise-stddev', type=float, default=0.)
    parser.add_argument('--lidar-noise-stddev', type=float, default=0.)
    parser.add_argument('--noise-mode', action='store_true',
                        help='放宽可靠观测门限并增加每视点帧数，验证噪声下不失效且不误达')
    args = parser.parse_args()
    output = Path(args.output).resolve()
    if output.exists() and any(output.iterdir()):
        parser.error('output directory must be new or empty')
    output.mkdir(parents=True, exist_ok=True)
    os.environ.update({'ROS_LOG_DIR': str(output / 'ros_logs'), 'ROS_DOMAIN_ID': '49',
                       'GZ_PARTITION': 'hydromining_phase6_acceptance_' + str(os.getpid())})
    robot = Path(get_package_share_directory('hydromining_robot'))
    config = yaml.safe_load((robot / 'config/viewpoints.yaml').read_text())
    views = {name: config['viewpoints'][name] for name in ('front_0p8', 'left_0p8', 'right_0p8')}
    command = ['ros2', 'launch', 'hydromining_robot', 'phase6_simulation.launch.py', 'gui:=false',
               f'depth_noise_stddev:={args.depth_noise_stddev}', f'lidar_noise_stddev:={args.lidar_noise_stddev}',
               f'spray_output_dir:={output / "meshes"}', f'perception_output_dir:={output / "perception"}']
    __import__("runtime_resources").reserve_desktop_cpu(); os.nice(10)
    log = (output / 'simulation.log').open('w')
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    interrupted = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: interrupted.set()); signal.signal(signal.SIGTERM, lambda *_: interrupted.set())
    node = Phase6Acceptance(output, process, views, config['region_center_world'],
                            frames_per_view=6 if args.noise_mode else 3)
    node.noise_mode_ = args.noise_mode
    node.interrupted_ = interrupted
    code, report = 1, None
    try:
        report = node.run(); code = 0
    except (KeyboardInterrupt, ExternalShutdownException):
        code = 130
    except Exception as error:
        import traceback; traceback.print_exc()
        (output / 'failure.json').write_text(json.dumps({'error': str(error), 'peak_rss_mib': node.peak_rss_ / 1024 ** 2}))
    finally:
        cleanup = cleanup_acceptance(node, process)
        log.close()
        if report is not None:
            report.update(cleanup)
            report['passed'] = code == 0 and not cleanup['owned_processes_remaining'] and not cleanup['cleanup_errors']
            (output / 'summary.json').write_text(json.dumps(report, indent=2, allow_nan=False))
            code = 0 if report['passed'] else 1
    print(json.dumps(report, indent=2) if report else 'FAILED')
    return code


if __name__ == '__main__':
    sys.exit(main())
