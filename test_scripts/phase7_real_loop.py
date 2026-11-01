#!/usr/bin/env python3
"""阶段七真实 Gazebo 闭环：规则决策层驱动 观测/喷射/收尾 Action 与 IK 喷射。

回合语义与替身一致：reset -> 基线（原始面）-> 预喷残差 -> 决策循环。
决策只读 AssessRegion 结果与 state.npz 图层；真值网格仅用于离线核对。
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time
from dataclasses import replace
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
from hydromining_robot.msg import ObservationView, SprayStatus
from hydromining_robot.srv import SprayAction
from test_phase4_acceptance import AcceptanceTest, cleanup_acceptance

sys.path.insert(0, get_package_share_directory('hydromining_sim'))
from algorithms.perception.config import PerceptionConfig
from algorithms.perception.kinematics import Chain
from algorithms.perception.metrics import mesh_heights, truth_change
from algorithms.perception.wall_frame import WallFrame
from algorithms.planning.decision import RulePolicy
from algorithms.planning.nbv import observed_visibility
from algorithms.planning.episodes import ring_shots
from algorithms.planning.fast_heightfield import TEST_SCENARIOS, TRAIN_SCENARIOS

# 旧 20 cm 工况保留给 --task legacy 的批量回归；新实验一律用 test_*/train 工况。
LEGACY_SCENARIOS = {
    'empty': {'shots': []},
    'single': {'shots': [((0., 0.), 12.)]},
    'pair': {'shots': [((-.04, 0.), 12.), ((.04, 0.), 12.)]},
    'tri_r5': {'shots': ring_shots((0., 0.), 0.05, 3)},
    'ring_r6': {'shots': ring_shots((0., 0.), 0.06, 6)},
    'dense': {'shots': [((0., 0.), 12.)] + ring_shots((0., 0.), 0.06, 6)},
}
SCENARIOS = {**LEGACY_SCENARIOS, **TRAIN_SCENARIOS, **TEST_SCENARIOS}
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
        self.gui_ = False
        self.gui_capture_errors_ = []
        self.gui_frames_ = []
        self.create_subscription(SprayStatus, '/spray/status', self.on_spray_status, __import__('rclpy.qos', fromlist=['QoSProfile']).QoSProfile(depth=10))

    def on_spray_status(self, message):
        """Capture an actual water jet frame once per session while SPRAYING."""
        if self.gui_ and message.state == 'SPRAYING' and not getattr(self, 'capturing_', False):
            self.capturing_ = True
            def capture():
                try:
                    from gui_view import capture_gui
                    self.gui_frames_.append(capture_gui(self.output_, self.center_, 'spraying'))
                except Exception as error:
                    self.gui_capture_errors_.append(str(error))
            self.capture_thread_ = threading.Thread(target=capture)
            self.capture_thread_.start()

    def lifecycle_active(self):
        self.wait(self.state_.service_is_ready, 90, 'lifecycle service missing')
        # Fine local geometry is initialized inside the lifecycle transition.
        state = self.future(self.state_.call_async(GetState.Request()), 120)
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

    def spray_tcp(self, duration=12., pressure=None):
        request = SprayAction.Request(use_tcp=True, pressure=self.task_pressure_ if pressure is None else pressure,
                                      duration=duration, enabled=True)
        reply = self.future(self.spray_.call_async(request), 300)
        assert reply.success, reply.message
        stats = json.loads(reply.statistics_json)
        previous_depth = getattr(self, 'last_depth_mm_', 0.)
        self.last_depth_mm_ = stats['max_depth_m']*1000
        if self.gui_:
            status = self.output_/'gui_status.txt'
            status.write_text(status.read_text().replace(
                f'Last max retreat: {previous_depth:.1f} mm',
                f'Last max retreat: {self.last_depth_mm_:.1f} mm'))
        print(f"    喷射结果: 压力 {stats['pressure_mpa']:.1f} MPa, "
              f"出口速度 {stats['outlet_velocity_m_s']:.1f} m/s, "
              f"流量 {stats['flow_m3_s']*60000:.1f} L/min, "
              f"本次最大退缩 {stats['max_depth_m']*1000:.1f} mm, "
              f"蚀除计算 {stats['solve_wall_s']:.1f} s", flush=True)
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
    episode_started = time.monotonic()
    center, normal = np.array(node.center_), np.array(cfg['wall_normal_world'])
    print(f"任务: 直径 {node.config_.diameter:.2f} m, "
          f"目标退缩 {node.config_.required_depth*100:.1f} cm, "
          f"达标面积 {node.config_.required_fraction:.0%}, 压力 {policy.pressure_:.1f} MPa", flush=True)
    policy.reset_episode()
    baseline_mesh = node.reset()
    current_mesh = baseline_mesh
    before = node.assess(OBSERVE_SET, new_baseline=True)
    assert before.assessment == 'baseline_ready', before.assessment
    # 基线 Goal 不落盘 state.npz；参考法向在 report.json 的 normal_world。
    baseline_report = json.loads(Path(before.report_path).read_text())
    frame = WallFrame.from_normal(center, baseline_report['normal_world'])
    # 预喷残差：IK 位姿 + TCP 喷射，与替身/真实一致的喷射路径
    jet_dir = -normal
    # 已蚀除面上喷射求解随暴露面积变慢；真实机负载下 30s 求解预算内，
    # 预喷时长封顶 8 s（阶段7系统约束，已记录）。
    for index, (uv, duration) in enumerate(scenario['shots'], 1):
        print(f'  预喷 {index}/{len(scenario["shots"])}: 正在移动并计算喷射', flush=True)
        solved = solve_spray_pose(spray_chain, T_world_base,
                                frame.to_world(np.array([uv[0], uv[1], 0.])),
                                jet_dir, policy.standoff_, views['front_0p8']['joints'])
        assert solved['reachable'], f'spray pose unreachable for residual {uv}'
        node.move(solved['joints'])
        current_mesh = node.spray_tcp(duration=min(duration, 8.)).output_mesh_path
    latest = node.assess(OBSERVE_SET)
    region = np.linalg.norm(frame.grid(node.config_.extent,
                                       node.config_.resolution)[..., :2],
                            axis=-1) <= node.config_.diameter / 2
    log = []
    for step in range(max_steps):
        state, change, surface, _ = node.load_eval(latest)
        # Recompute after every assessment, including each spray; no truth input.
        vis = observed_visibility(state, frame, node.view_specs_, node.config_)
        assessment = {'status': latest.assessment, 'coverage': latest.coverage,
                      'fraction_lower': latest.fraction_lower,
                      'fraction_upper': latest.fraction_upper}
        action = policy.decide(state, assessment, change, surface, frame,
                               view_poses, vis, region)
        entry = {'step': step, 'type': action['type'], 'rationale': action['rationale'],
                 'assessment': assessment, 'nbv_scores': action.get('scores', {})}
        log.append(entry)
        print(f"  step {step}: {action['type']} [{assessment['status']}] "
              f"coverage={assessment['coverage']:.3f}, "
              f"F=[{assessment['fraction_lower']:.3f}, {assessment['fraction_upper']:.3f}] "
              f"- {action['rationale']}", flush=True)
        if node.gui_:
            (node.output_/'gui_status.txt').write_text(
                f"Wall: {node.wall_angle_} | Zone: {node.config_.diameter*100:.0f} cm\n"
                f"Goal: {node.config_.required_fraction*100:.0f}% area / {node.config_.required_depth*1000:.0f} mm\n"
                f"Pressure: {node.task_pressure_:.1f} MPa\n"
                f"Flow: {node.flow_l_min_:.1f} L/min\n"
                f"Last max retreat: {getattr(node, 'last_depth_mm_', 0.):.1f} mm\n"
                f"Completion: {assessment['fraction_lower']*100:.1f}–{assessment['fraction_upper']*100:.1f}%\n"
                f"Decision: {action['type']} ({assessment['status']})")
        if action['type'] == 'finish':
            break
        if action['type'] == 'observe':
            started = time.monotonic()
            sigma_before = float(np.nanmedian(state['sigma']))
            latest = node.assess([action['view']], accumulate=True)
            observed_state, _, _, _ = node.load_eval(latest)
            entry['observation_effect'] = {
                'view': action['view'], 'elapsed_s': time.monotonic() - started,
                'predicted_gain': action['scores'][action['view']]['gain'],
                'coverage_before': assessment['coverage'], 'coverage_after': latest.coverage,
                'fraction_lower_before': assessment['fraction_lower'],
                'fraction_lower_after': latest.fraction_lower,
                'sigma_median_before': sigma_before,
                'sigma_median_after': float(np.nanmedian(observed_state['sigma']))}
        else:
            started = time.monotonic()
            solved = solve_spray_pose(spray_chain, T_world_base,
                                    frame.to_world(np.r_[action['uv'], 0.]),
                                    jet_dir, policy.standoff_,
                                    views['front_0p8']['joints'])
            assert solved['reachable'], 'spray pose unreachable'
            node.move(solved['joints'])
            entry['motion_wall_s'] = time.monotonic()-started
            started = time.monotonic()
            reply = node.spray_tcp(duration=action['duration'])
            entry['spray_service_wall_s'] = time.monotonic()-started
            current_mesh = reply.output_mesh_path
            entry['spray_stats'] = {'volume': reply.volume_removed_m3,
                                    'max_depth': reply.max_depth_m}
            started = time.monotonic()
            latest = node.assess(OBSERVE_SET)  # 喷射后新一轮观测
            entry['perception_wall_s'] = time.monotonic()-started
            if node.gui_:
                status = node.output_/'gui_status.txt'
                status.write_text(status.read_text().replace(
                    f"Completion: {assessment['fraction_lower']*100:.1f}–{assessment['fraction_upper']*100:.1f}%",
                    f'Completion: {latest.fraction_lower*100:.1f}–{latest.fraction_upper*100:.1f}%'))
            print(f"    本轮耗时: 运动 {entry['motion_wall_s']:.1f} s, "
                  f"喷射服务 {entry['spray_service_wall_s']:.1f} s, "
                  f"喷后感知 {entry['perception_wall_s']:.1f} s", flush=True)
    # Independent final audit only; truth is never an input to action selection.
    extent, resolution = (.5, .005) if cfg.get('wall_inclination_deg') else (.24, .002)
    before_h = mesh_heights(str(baseline_mesh), node.mesh_pose_, frame, extent, resolution)
    after_h = mesh_heights(str(current_mesh), node.mesh_pose_, frame, extent, resolution)
    truth = truth_change(before_h, after_h, frame, extent, resolution,
                         node.config_.diameter/2, node.config_.required_depth,
                         node.config_.required_fraction)
    verdict = latest.assessment
    assert not (verdict == 'achieved' and truth['status'] != 'achieved'), 'false achieved'
    elapsed = time.monotonic()-episode_started
    estimate = None
    if cfg.get('wall_inclination_deg') and verdict == 'achieved' and truth['status'] == 'achieved':
        estimate = {'target_volume_m3': .45,
                    'by_measured_volume_s': .45*elapsed/truth['removed_volume_m3'],
                    'by_completed_area_s': 9.*elapsed/truth['sampled_area_m2'],
                    'scope': 'linear extrapolation from this local run; excludes repositioning and changing coal properties'}
        print(f"整墙估算（不含基座移动）: 体积速率法 {estimate['by_measured_volume_s']/3600:.1f} h, "
              f"作业面积法 {estimate['by_completed_area_s']/3600:.1f} h", flush=True)
    return {'steps': len(log), 'log': log, 'verdict': verdict, 'truth':truth,
            'scenario':scenario, 'last_report': latest.report_path,
            'episode_wall_s': elapsed, 'whole_wall_estimate': estimate,
            'aggregate_peak_rss_mib':node.peak_rss_/1024**2}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', default='tri_r5', choices=list(SCENARIOS))
    parser.add_argument('--output', required=True)
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--keep-open', action='store_true')
    parser.add_argument('--model', default=None, help='Versioned DDPG checkpoint directory')
    parser.add_argument('--max-steps', type=int, default=24)
    parser.add_argument('--task', choices=('legacy', 'large'), default='legacy')
    parser.add_argument('--low-light', action='store_true')
    args = parser.parse_args()
    __import__('runtime_resources').reserve_desktop_cpu()
    output = Path(args.output).resolve()
    if output.exists() and any(output.iterdir()):
        parser.error('output directory must be new or empty')
    output.mkdir(parents=True, exist_ok=True)
    os.environ.update({'ROS_LOG_DIR': str(output / 'ros_logs'), 'ROS_DOMAIN_ID': '55',
                       'GZ_PARTITION': 'hydromining_phase7_' + str(os.getpid())})
    robot = Path(get_package_share_directory('hydromining_robot'))
    sim = Path(get_package_share_directory('hydromining_sim'))
    if args.gui:
        os.environ['HYDROMINING_GUI_STATUS'] = str(output/'gui_status.txt')
        os.environ['GZ_GUI_PLUGIN_PATH'] = str(sim.parent.parent/'lib') + os.pathsep + os.environ.get('GZ_GUI_PLUGIN_PATH', '')
        (output/'gui_status.txt').write_text('Initializing task...')
    task_path = robot / 'config/phase7_large_task.yaml'
    cfg = yaml.safe_load((task_path if args.task == 'large' else robot / 'config/viewpoints.yaml').read_text())
    pcfg = PerceptionConfig()
    if args.task == 'large':
        pcfg = replace(pcfg, diameter=cfg['diameter'], required_depth=cfg['required_depth'],
                       required_fraction=cfg['required_fraction'])
        if args.max_steps == 24:
            args.max_steps = cfg['max_steps']
    views = cfg['viewpoints']
    chain = Chain(subprocess.run(['xacro', str(robot / 'urdf/panda_with_water_gun.urdf.xacro'),
                                  'name:=panda', 'arm_id:=panda'],
                                 capture_output=True, text=True, check=True).stdout,
                  'panda_link0', 'tcp')
    command = ['ros2', 'launch', 'hydromining_robot', 'phase6_simulation.launch.py',
               'gui:=' + str(args.gui).lower(), 'enable_effects:=' + str(args.gui).lower(),
               'spray_config:=' + str(robot / 'config/spray_phase7.yaml'),
               f'spray_output_dir:={output / "meshes"}',
               f'perception_output_dir:={output / "perception"}']
    if args.task == 'large' or args.low_light or args.gui:
        if args.task == 'large':
            spray_config = yaml.safe_load((robot/'config/spray_phase7.yaml').read_text())
            parameters = spray_config['spray_controller_node']['ros__parameters']
            parameters.update(local_task_config=str(task_path), erosion_tcp_offset_m=.05, solve_budget_s=180.)
            spray_path = output/'spray_config.yaml'
            spray_path.write_text(yaml.safe_dump(spray_config, sort_keys=False))
            command = [x if not x.startswith('spray_config:=') else f'spray_config:={spray_path}' for x in command]
        world_path = output/'world.sdf'
        from gui_view import write_scene_world
        write_scene_world(sim/'worlds/coal_wall_acceptance.sdf', world_path,
                          cfg['source_coal_pose'] if args.task == 'large' else None, args.low_light, args.gui)
        command.append(f'world:={world_path}')
    log = (output / 'simulation.log').open('w')
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                               start_new_session=True)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    # views 必须给全集：决策层的补看视点不限于 OBSERVE_SET 三个。
    node = Phase7Loop(output, process, views, chain, pcfg,
                      cfg['region_center_world'])
    node.task_pressure_ = cfg.get('pressure_mpa', 20.)
    node.wall_angle_ = f"{cfg['wall_inclination_deg']:.0f} deg" if 'wall_inclination_deg' in cfg else 'legacy'
    from algorithms.jet_effects import jet_hydraulics
    node.flow_l_min_ = jet_hydraulics(node.task_pressure_)['flow_m3_s']*60000
    if args.task == 'large':
        initial_frame = WallFrame.from_normal(cfg['region_center_world'], cfg['wall_normal_world'])
        node.mesh_pose_ = [*initial_frame.origin, *Rotation.from_matrix(initial_frame.axes.copy()).as_euler('xyz')]
    else:
        node.mesh_pose_ = MESH_POSE
    node.gui_ = args.gui
    code, report = 1, None
    try:
        node.wait(lambda: node.lifecycle_active() and node.perception_active(), 180,
                  'lifecycle nodes not active')
        node.wait(node.motion_.server_is_ready, 90, 'trajectory action missing')
        if args.gui:
            from gui_view import capture_gui
            capture_gui(output, cfg['region_center_world'], 'before')
        T_world_base = node.tf_matrix()
        # 可见性先验：离线替身为真实系统预计算（几何相同）；本次先给均匀掩码占位。
        urdf = subprocess.run(['xacro', str(robot / 'urdf/panda_with_water_gun.urdf.xacro')], capture_output=True, text=True, check=True).stdout
        camera_chain = Chain(urdf, 'panda_link0', 'camera_link')
        lidar_chain = Chain(urdf, 'panda_link0', 'lidar_link')
        node.view_specs_ = {}
        for name, view in views.items():
            if view.get('reachable'):
                node.view_specs_[name] = {
                    'tf_depth': T_world_base @ camera_chain.fk(view['joints']),
                    'tf_lidar': T_world_base @ lidar_chain.fk([]),
                    'K': np.array([[320/(2*np.tan(.75)), 0, 160], [0, 320/(2*np.tan(.75)), 120], [0, 0, 1]]),
                    'image_shape': (240, 320)}
        view_poses = {n: np.asarray(v['camera_position_world']) for n, v in views.items()
                      if v.get('reachable')}
        vis = None
        scenario = SCENARIOS[args.scenario]
        policy = RulePolicy(pcfg, max_duration=8., pressure=node.task_pressure_,
                            observe_gain_min=float('inf') if args.task == 'large' else .05,
                            standoff=cfg.get('standoff_m', .7))
        if args.model:
            from learned_spray_policy import LearnedSprayPolicy
            policy = LearnedSprayPolicy(args.model, policy, pcfg)
        report = run_episode(node, views, scenario, cfg, policy, vis, view_poses,
                             chain, T_world_base, args.max_steps)
        if args.gui:
            from gui_view import capture_gui
            report['gui'] = capture_gui(output, center=cfg['region_center_world'])
            if hasattr(node, 'capture_thread_'):
                node.capture_thread_.join(timeout=10)
            report['gui_spraying_frames'] = node.gui_frames_
            report['gui_capture_errors'] = node.gui_capture_errors_
            assert node.gui_frames_ and not node.gui_capture_errors_, 'missing GUI spray evidence'
        report['passed'] = report['verdict'] == 'achieved' and report['truth']['status'] == 'achieved'
        code = 0 if report['passed'] else 1
        (output/'checks.json').write_text(json.dumps(report,indent=2,allow_nan=False))
        if args.keep_open:
            print('GUI inspection ready; Ctrl+C closes this session.', flush=True)
            while process.poll() is None and rclpy.ok():
                node.monitor()
                rclpy.spin_once(node, timeout_sec=.2)
    except KeyboardInterrupt:
        code = 0 if report is not None else 130
    except Exception:
        import traceback
        traceback.print_exc()
        (output / 'failure.json').write_text(json.dumps({'error': traceback.format_exc(limit=4)}))
    finally:
        # A second Ctrl+C must not interrupt the staged shutdown and strand the
        # GUI / simulation children. The process is about to exit after cleanup.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        print('Stopping this simulation and its GUI; please wait for cleanup...', flush=True)
        cleanup = cleanup_acceptance(node, process)
        log.close()
        if report is not None:
            report.update(cleanup)
            (output / 'summary.json').write_text(json.dumps(report, indent=2, allow_nan=False))
    print(json.dumps({k: v for k, v in (report or {}).items() if k != 'log'}, indent=2))
    return code


if __name__ == '__main__':
    sys.exit(main())
