#!/usr/bin/env python3
"""阶段七真实 Gazebo 闭环：规则决策层驱动 观测/喷射/收尾 Action 与 IK 喷射。

回合语义与替身一致：reset -> 基线（原始面）-> 预喷残差 -> 决策循环。
决策只读 AssessRegion 结果与 state.npz 图层；真值网格仅用于离线核对。
"""
import argparse
import json
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from lifecycle_msgs.srv import GetState
from rclpy.action import ActionClient
from rclpy.signals import SignalHandlerOptions
from scipy.spatial.transform import Rotation
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from hydromining_robot.action import AssessRegion
from hydromining_robot.msg import ObservationView
from hydromining_robot.srv import SprayAction
from test_phase4_acceptance import AcceptanceTest, cleanup_acceptance

sys.path.insert(0, get_package_share_directory('hydromining_sim'))
from algorithms.perception.config import PerceptionConfig
from algorithms.perception.kinematics import Chain
from algorithms.perception.metrics import mesh_heights, truth_change
from algorithms.perception.wall_frame import WallFrame
from algorithms.planning.decision import RulePolicy
from algorithms.planning.episodes import SCENARIOS
from algorithms.planning.spray_pose import solve_spray_pose

MESH_POSE = [1.5, 0., 1.5, 0., -1.9198621771937625, 0.]
OBSERVE_SET = ['front_0p8', 'left_0p8', 'right_0p8']


class SurfaceView:
    """让决策层像访问 HeightMap 一样访问 state.npz 的中心网格。"""

    def __init__(self, frame, extent, resolution):
        self.centers_ = frame.grid(extent, resolution)


class Phase7Loop(AcceptanceTest):
    """复用进程/内存看护；叠加 IK 移动、TCP 喷射与评估 Action。"""

    def __init__(self, output, process, views, chain, pcfg, center):
        super().__init__(output, process)
        self.views_, self.chain_, self.config_, self.center_ = views, chain, pcfg, center
        self.assess_ = ActionClient(self, AssessRegion, '/coal_wall/assess_region')

    def lifecycle_active(self):
        self.wait(self.state_.service_is_ready, 90, 'lifecycle service missing')
        state = self.future(self.state_.call_async(GetState.Request()), 10)
        return state.current_state.id == 3

    def perception_active(self):
        client = self.create_client(GetState, '/wall_perception_node/get_state')
        self.wait(client.service_is_ready, 90, 'perception lifecycle service missing')
        state = self.future(client.call_async(GetState.Request()), 10)
        return state.current_state.id == 3

    def tf_matrix(self, source='panda_link0'):
        tf = self.tf_.lookup_transform('world', source, rclpy.time.Time())
        t, q = tf.transform.translation, tf.transform.rotation
        T = np.eye(4)
        T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        T[:3, 3] = [t.x, t.y, t.z]
        return T

    def move(self, joints, seconds=6):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['panda_joint' + str(i) for i in range(1, 8)]
        goal.trajectory.points = [JointTrajectoryPoint(
            positions=list(joints), time_from_start=Duration(sec=seconds))]
        handle = self.future(self.motion_.send_goal_async(goal))
        assert handle.accepted
        result = self.future(handle.get_result_async(), 90)
        assert result.result.error_code == 0, result.result.error_string
        self.wait(lambda: self.joints_ is not None and
                np.max(np.abs(self.joints_ - joints)) < .02, 15,
                'Arm did not reach pose')

    def reset(self):
        reply = self.future(self.reset_.call_async(Trigger.Request()), 60)
        assert reply.success, reply.message
        return Path(reply.message)

    def goal(self, names, new_baseline=False, accumulate=False):
        goal = AssessRegion.Goal()
        goal.center.x, goal.center.y, goal.center.z = self.center_
        cfg = self.config_
        goal.diameter, goal.required_depth = cfg.diameter, cfg.required_depth
        goal.required_fraction, goal.confidence_k = cfg.required_fraction, cfg.confidence_k
        goal.frames_per_view, goal.new_baseline = 3, new_baseline
        goal.accumulate = accumulate
        goal.viewpoints = [ObservationView(name=n, joints=self.views_[n]['joints'])
                           for n in names]
        return goal

    def assess(self, names, new_baseline=False, accumulate=False, timeout=420):
        self.wait(self.assess_.server_is_ready, 30, 'assessment action missing')
        handle = self.future(self.assess_.send_goal_async(self.goal(names, new_baseline, accumulate)), 20)
        assert handle.accepted, 'assessment goal rejected'
        result = self.future(handle.get_result_async(), timeout)
        assert result.status == GoalStatus.STATUS_SUCCEEDED, result.result.message
        return result.result

    def spray_tcp(self, duration=12., pressure=20.):
        request = SprayAction.Request(use_tcp=True, pressure=pressure,
                                      duration=duration, enabled=True)
        reply = self.future(self.spray_.call_async(request), 300)
        assert reply.success, reply.message
        return reply

    def load_eval(self, result):
        """state.npz -> 决策层需要的 state/change/surface/frame。"""
        with np.load(result.state_path) as data:
            frame = WallFrame(data['origin'], data['axes'])
            state = {key[len('current_'):]: data[key] for key in data.files
                     if key.startswith('current_')}
            change = {key[len('change_'):]: data[key] for key in data.files
                      if key.startswith('change_')}
        return (state, change, SurfaceView(frame, self.config_.extent,
                                         self.config_.resolution), frame)


def run_episode(node, views, scenario, cfg, policy, vis, view_poses,
                spray_chain, T_world_base, max_steps=12):
    center, normal = np.array(node.center_), np.array(cfg['wall_normal_world'])
    policy.reset_episode()
    baseline_mesh = node.reset()
    before = node.assess(OBSERVE_SET, new_baseline=True)
    assert before.assessment == 'baseline_ready', before.assessment
    # 基线 Goal 不落盘 state.npz；参考法向在 report.json 的 normal_world。
    baseline_report = json.loads(Path(before.report_path).read_text())
    frame = WallFrame.from_normal(center, baseline_report['normal_world'])
    # 预喷残差：IK 位姿 + TCP 喷射，与替身/真实一致的喷射路径
    jet_dir = -normal
    # 已蚀除面上喷射求解随暴露面积变慢；真实机负载下 30s 求解预算内，
    # 预喷时长封顶 8 s（阶段7系统约束，已记录）。
    for uv, duration in scenario['shots']:
        solved = solve_spray_pose(spray_chain, T_world_base,
                                frame.to_world(np.array([uv[0], uv[1], 0.])),
                                jet_dir, 0.7, views['front_0p8']['joints'])
        assert solved['reachable'], f'spray pose unreachable for residual {uv}'
        node.move(solved['joints'])
        node.spray_tcp(duration=min(duration, 8.))
    latest = node.assess(OBSERVE_SET)
    region = np.linalg.norm(frame.grid(node.config_.extent,
                                       node.config_.resolution)[..., :2],
                            axis=-1) <= node.config_.diameter / 2
    log = []
    for step in range(max_steps):
        state, change, surface, _ = node.load_eval(latest)
        if vis is None:
            # 无先验掩码时给均匀掩码：评分仅靠期望 σ（距离/入射角）区分。
            vis = {name: np.ones(state['coverage'].shape, bool) for name in view_poses}
        assessment = {'status': latest.assessment, 'coverage': latest.coverage,
                      'fraction_lower': latest.fraction_lower,
                      'fraction_upper': latest.fraction_upper}
        action = policy.decide(state, assessment, change, surface, frame,
                               view_poses, vis, region)
        entry = {'step': step, 'type': action['type'], 'rationale': action['rationale'],
                 'assessment': assessment}
        log.append(entry)
        print(f"  step {step}: {action['type']} - {action['rationale']}", flush=True)
        if action['type'] == 'finish':
            break
        if action['type'] == 'observe':
            latest = node.assess([action['view']], accumulate=True)
        else:
            solved = solve_spray_pose(spray_chain, T_world_base,
                                    frame.to_world(np.r_[action['uv'], 0.]),
                                    jet_dir, policy.standoff_,
                                    views['front_0p8']['joints'])
            assert solved['reachable'], 'spray pose unreachable'
            node.move(solved['joints'])
            reply = node.spray_tcp(duration=action['duration'])
            entry['spray_stats'] = {'volume': reply.volume_removed_m3,
                                    'max_depth': reply.max_depth_m}
            latest = node.assess(OBSERVE_SET)  # 喷射后新一轮观测
    return {'steps': len(log), 'log': log, 'verdict': log[-1]['assessment']['status'],
            'last_report': latest.report_path}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', default='tri_r5', choices=list(SCENARIOS))
    parser.add_argument('--output', required=True)
    parser.add_argument('--max-steps', type=int, default=12)
    args = parser.parse_args()
    output = Path(args.output).resolve()
    if output.exists() and any(output.iterdir()):
        parser.error('output directory must be new or empty')
    output.mkdir(parents=True, exist_ok=True)
    os.environ.update({'ROS_LOG_DIR': str(output / 'ros_logs'), 'ROS_DOMAIN_ID': '55',
                       'GZ_PARTITION': 'hydromining_phase7_' + str(os.getpid())})
    robot = Path(get_package_share_directory('hydromining_robot'))
    sim = Path(get_package_share_directory('hydromining_sim'))
    cfg = yaml.safe_load((robot / 'config/viewpoints.yaml').read_text())
    pcfg = PerceptionConfig()
    views = cfg['viewpoints']
    chain = Chain(subprocess.run(['xacro', str(robot / 'urdf/panda_with_water_gun.urdf.xacro'),
                                  'name:=panda', 'arm_id:=panda'],
                                 capture_output=True, text=True, check=True).stdout,
                  'panda_link0', 'tcp')
    command = ['ros2', 'launch', 'hydromining_robot', 'phase6_simulation.launch.py',
               'gui:=false', 'enable_effects:=false',
               'spray_config:=' + str(robot / 'config/spray_phase7.yaml'),
               f'spray_output_dir:={output / "meshes"}',
               f'perception_output_dir:={output / "perception"}']
    log = (output / 'simulation.log').open('w')
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                               start_new_session=True)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    # views 必须给全集：决策层的补看视点不限于 OBSERVE_SET 三个。
    node = Phase7Loop(output, process, views, chain, pcfg,
                      cfg['region_center_world'])
    code, report = 1, None
    try:
        node.wait(lambda: node.lifecycle_active() and node.perception_active(), 180,
                  'lifecycle nodes not active')
        node.wait(node.motion_.server_is_ready, 90, 'trajectory action missing')
        T_world_base = node.tf_matrix()
        # 可见性先验：离线替身为真实系统预计算（几何相同）；本次先给均匀掩码占位。
        view_poses = {n: np.asarray(v['camera_position_world']) for n, v in views.items()
                      if v.get('reachable')}
        vis = None
        scenario = SCENARIOS[args.scenario]
        policy = RulePolicy(pcfg, max_duration=8.)
        report = run_episode(node, views, scenario, cfg, policy, vis, view_poses,
                             chain, T_world_base, args.max_steps)
        code = 0
    except Exception:
        import traceback
        traceback.print_exc()
        (output / 'failure.json').write_text(json.dumps({'error': traceback.format_exc(limit=4)}))
    finally:
        cleanup = cleanup_acceptance(node, process)
        log.close()
        if report is not None:
            report.update(cleanup)
            (output / 'summary.json').write_text(json.dumps(report, indent=2, allow_nan=False))
    print(json.dumps({k: v for k, v in (report or {}).items() if k != 'log'}, indent=2))
    return code


if __name__ == '__main__':
    sys.exit(main())
