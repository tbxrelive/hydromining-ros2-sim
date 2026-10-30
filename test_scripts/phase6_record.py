#!/usr/bin/env python3
"""阶段 6.0 数据集录制：多视点观测 + 真值网格 + 喷射记录（无 GUI，默认无噪声无干扰）。

默认录原始干净数据，环境退化离线按种子叠加；原生噪声选项用于独立验证传感器配置。
机械臂只负责移动到视点，喷射用网格坐标 use_tcp=false 的多点方案（见 design_shot_plans.py）。
"""
import argparse, hashlib, json, os, shutil, signal, subprocess, sys, threading
from pathlib import Path
import numpy as np
import psutil
import yaml
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
from scipy.spatial.transform import Rotation
from std_srvs.srv import Trigger
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from ament_index_python.packages import get_package_share_directory
from test_phase4_acceptance import AcceptanceTest, cleanup_acceptance
from hydromining_robot.srv import SprayAction
sys.path.insert(0, get_package_share_directory('hydromining_sim'))
from algorithms.perception.kinematics import Chain

POS = np.array([-0.4610369606053094, -1.4578341658855187e-10, 1.0723548361037436])
DIR = np.array([-0.34202018572081666, 5.264312857161668e-10, -0.9396926053553356])
E1 = np.cross(DIR, [0, 1, 0]); E1 /= np.linalg.norm(E1); E2 = np.cross(DIR, E1)
ring = lambda n, r: [(r * np.cos(2 * np.pi * k / n), r * np.sin(2 * np.pi * k / n)) for k in range(n)]
SCENARIOS = {'none': [], 'single': [(0.0, 0.0)], 'tri_r5': ring(3, 0.05), 'hex_r6': [(0.0, 0.0)] + ring(6, 0.06)}


class Recorder(AcceptanceTest):
    """复用阶段四验收的有界等待、资源监控与 TF 时间戳对齐方式。"""

    def __init__(self, output, process, viewpoints, chain, frames_per_view=5):
        super().__init__(output, process)
        self.viewpoints_, self.chain_ = viewpoints, chain
        self.frames_per_view_ = frames_per_view
        self.depth_ = self.info_ = self.lidar_ = None
        self.depth_seq_ = self.lidar_seq_ = 0
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, '/hydromining/camera/depth/image_rect_raw', self.on_depth, qos)
        self.create_subscription(CameraInfo, '/hydromining/camera/color/camera_info', self.on_info, qos)
        self.create_subscription(PointCloud2, '/hydromining/lidar/points', self.on_lidar, qos)

    def on_depth(self, msg):
        self.depth_, self.depth_seq_ = msg, self.depth_seq_ + 1

    def on_info(self, msg):
        self.info_ = msg

    def on_lidar(self, msg):
        self.lidar_, self.lidar_seq_ = msg, self.lidar_seq_ + 1

    def tf_at(self, frame, msg):
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        self.wait(lambda: self.tf_.can_transform('world', frame, stamp), 8, 'TF did not reach sensor stamp: ' + frame)
        t = self.tf_.lookup_transform('world', frame, stamp).transform
        return [t.translation.x, t.translation.y, t.translation.z, t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]

    def move(self, joints, seconds=5):
        if hasattr(self, 'fixed_joints_'):
            del self.fixed_joints_
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['panda_joint' + str(i) for i in range(1, 8)]
        goal.trajectory.points = [JointTrajectoryPoint(positions=list(joints), time_from_start=Duration(sec=seconds))]
        handle = self.future(self.motion_.send_goal_async(goal))
        assert handle.accepted
        result = self.future(handle.get_result_async(), 90)
        assert result.result.error_code == 0, result.result.error_string
        self.wait(lambda: self.joints_ is not None and np.max(np.abs(self.joints_ - joints)) < .02, 10, 'Arm not at viewpoint')
        start = self.get_clock().now().nanoseconds
        self.wait(lambda: (self.get_clock().now().nanoseconds - start) * 1e-9 > 1.5, 20, 'Settle')
        self.fixed_joints_ = self.joints_.copy()
        self.max_joint_drift_ = 0.0

    def snapshot(self, path, name):
        """三帧新鲜数据后保存深度图、LiDAR 点、内参及按各自时间戳查询的 TF。"""
        d0, l0 = self.depth_seq_, self.lidar_seq_
        self.wait(lambda: self.depth_seq_ >= d0 + 3 and self.lidar_seq_ >= l0 + 2 and self.info_ is not None, 60, 'Fresh frames')
        d, li, info = self.depth_, self.lidar_, self.info_
        depth = np.ndarray((d.height, d.width), '>f4' if d.is_bigendian else '<f4', buffer=d.data, strides=(d.step, 4)).astype(np.float32)
        cloud = read_points_numpy(li, field_names=['x', 'y', 'z'], skip_nans=False).reshape(-1, 3).astype(np.float32)
        joints = self.joints_.copy()
        np.savez_compressed(path / f'{name}.npz', depth=depth, lidar_local=cloud, K=np.array(info.k).reshape(3, 3),
                            depth_frame=d.header.frame_id, lidar_frame=li.header.frame_id,
                            depth_stamp=[d.header.stamp.sec, d.header.stamp.nanosec],
                            lidar_stamp=[li.header.stamp.sec, li.header.stamp.nanosec],
                            tf_depth=self.tf_at(d.header.frame_id, d), tf_lidar=self.tf_at(li.header.frame_id, li),
                            joints=joints, lidar_shape=[li.height, li.width])
        return joints

    def capture_all(self, path, tag, check):
        path.mkdir(parents=True, exist_ok=True)
        for name, vp in self.viewpoints_.items():
            self.move(vp['joints'])
            for index in range(self.frames_per_view_):
                label = f'{tag}_{name}_f{index:03d}'
                q = self.snapshot(path, label)
            T = self.chain_.fk(q)
            with np.load(path / f'{label}.npz') as data:
                tf = data['tf_depth']
            dp = float(np.linalg.norm(T[:3, 3] - tf[:3]))
            dr = float(np.linalg.norm(Rotation.from_matrix(T[:3, :3] @ Rotation.from_quat(tf[3:]).as_matrix().T).as_rotvec()))
            check.append({'view': name, 'tag': tag, 'fk_vs_tf_position_m': dp, 'fk_vs_tf_rotation_rad': dr})
            assert dp < 2e-3 and dr < 0.01, check[-1]
            self.get_logger().info(f'{tag}/{name}: FK-TF position {dp * 1000:.3f} mm, rotation {dr:.5f} rad')

    def run(self, only=None):
        self.ready()
        out = self.output_ / 'dataset'
        check, truth = [], {}
        reset = self.future(self.reset_.call_async(Trigger.Request()), 40)
        assert reset.success, reset.message
        base_mesh = Path(reset.message)
        (out / 'truth').mkdir(parents=True, exist_ok=True)
        shutil.copy(base_mesh, out / 'truth' / 'baseline.obj')
        self.capture_all(out / 'baseline', 'baseline', check)
        for scenario in (only or SCENARIOS):
            reset = self.future(self.reset_.call_async(Trigger.Request()), 40)
            assert reset.success, reset.message
            shots, mesh = [], Path(reset.message)
            for a, b in SCENARIOS[scenario]:
                p = POS + a * E1 + b * E2
                req = SprayAction.Request(use_tcp=False, pressure=20., duration=12., enabled=True)
                req.position_x, req.position_y, req.position_z = p.tolist()
                req.direction_x, req.direction_y, req.direction_z = DIR.tolist()
                r = self.future(self.spray_.call_async(req), 120)
                assert r.success, r.message
                mesh = Path(r.output_mesh_path)
                shots.append(json.loads(r.statistics_json))
            shutil.copy(mesh, out / 'truth' / f'{scenario}.obj')
            truth[scenario] = {'shots': shots, 'mesh': f'truth/{scenario}.obj'}
            self.capture_all(out / scenario, scenario, check)
        (out / 'truth' / 'truth.json').write_text(json.dumps(truth, indent=2))
        (out / 'fk_tf_check.json').write_text(json.dumps(check, indent=2))
        return {'passed': True, 'scenarios': list(only or SCENARIOS), 'viewpoints': list(self.viewpoints_),
                'frames_per_view': self.frames_per_view_,
                'max_fk_tf_position_mm': max(c['fk_vs_tf_position_m'] for c in check) * 1e3,
                'max_fk_tf_rotation_rad': max(c['fk_vs_tf_rotation_rad'] for c in check),
                'aggregate_peak_rss_mib': self.peak_rss_ / 1024 ** 2, 'noise': 'off', 'effects': 'off', 'calibrated': False}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--output', default='/tmp/hydromining_phase6_dataset')
    ap.add_argument('--viewpoints', nargs='*', help='默认全部可达视点')
    ap.add_argument('--scenarios', nargs='*', choices=list(SCENARIOS))
    ap.add_argument('--frames-per-view', type=int, default=5)
    ap.add_argument('--depth-noise-stddev', type=float, default=0.0)
    ap.add_argument('--lidar-noise-stddev', type=float, default=0.0)
    args = ap.parse_args()
    if not 1 <= args.frames_per_view <= 30:
        ap.error('--frames-per-view must be in [1,30]')
    for value in (args.depth_noise_stddev, args.lidar_noise_stddev):
        if not np.isfinite(value) or not 0 <= value <= 0.1:
            ap.error('native noise standard deviation must be finite and in [0,0.1] m')
    output = Path(args.output).resolve()
    if output.exists() and any(output.iterdir()):
        ap.error('output directory must be new or empty; existing evidence will not be overwritten')
    output.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_LOG_DIR'] = str(output / 'ros_logs'); os.environ['ROS_DOMAIN_ID'] = '48'
    os.environ['GZ_PARTITION'] = 'hydromining_phase6_' + str(os.getpid())
    robot = Path(get_package_share_directory('hydromining_robot')); sim = Path(get_package_share_directory('hydromining_sim'))
    cfg = yaml.safe_load((robot / 'config/viewpoints.yaml').read_text())
    views = {k: v for k, v in cfg['viewpoints'].items() if v['reachable'] and (not args.viewpoints or k in args.viewpoints)}
    if not views or (args.viewpoints and set(args.viewpoints) - set(views)):
        ap.error('requested viewpoints must be nonempty and marked reachable in viewpoints.yaml')
    workspace = Path(__file__).resolve().parents[1]
    source_paths = [Path(__file__), workspace / 'scripts/run_phase6_record.sh',
                    robot / 'config/viewpoints.yaml', robot / 'config/spray_acceptance.yaml']
    provenance = {'schema_version': 1, 'frames_per_view': args.frames_per_view,
                  'source_hashes': {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                                    for path in source_paths},
                  'viewpoints': views,
                  'raw_noise_stddev_m': {'depth': args.depth_noise_stddev, 'lidar': args.lidar_noise_stddev},
                  'native_jet_effects': 'off',
                  'scope': 'sensor snapshots, not rosbag and not native particle observations',
                  'collision_free_paths_verified': False}
    (output / 'provenance.json').write_text(json.dumps(provenance, indent=2))
    chain = Chain(subprocess.run(['xacro', str(robot / 'urdf/panda_with_water_gun.urdf.xacro'), 'name:=panda', 'arm_id:=panda'],
                                 capture_output=True, text=True, check=True).stdout, 'panda_link0', 'camera_link')
    command = ['ros2', 'launch', 'hydromining_robot', 'robot_simulation.launch.py', 'gui:=false',
               'world:=' + str(sim / 'worlds/coal_wall_acceptance.sdf'), 'start_spray:=true',
               'spray_config:=' + str(robot / 'config/spray_acceptance.yaml'), 'spray_output_dir:=' + str(output / 'meshes'),
               'camera_rate:=5', 'camera_width:=320', 'camera_height:=240', 'lidar_rate:=2',
               f'depth_noise_stddev:={args.depth_noise_stddev}',
               f'lidar_noise_stddev:={args.lidar_noise_stddev}']
    os.sched_setaffinity(0, sorted(os.sched_getaffinity(0))[:2]); os.nice(10)
    process = subprocess.Popen(command, stdout=(output / 'simulation.log').open('w'), stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    interrupted = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: interrupted.set()); signal.signal(signal.SIGTERM, lambda *_: interrupted.set())
    node = Recorder(output, process, views, chain, args.frames_per_view); node.interrupted_ = interrupted
    code, report = 1, None
    try:
        report = node.run(args.scenarios); code = 0
        report['noise'] = provenance['raw_noise_stddev_m']
    except (KeyboardInterrupt, ExternalShutdownException):
        code = 130
    except Exception as error:
        import traceback; traceback.print_exc()
        (output / 'failure.json').write_text(json.dumps({'error': str(error), 'peak_rss_mib': node.peak_rss_ / 1024 ** 2}))
    finally:
        cleanup = cleanup_acceptance(node, process)
        if report is not None:
            report.update(cleanup); report['passed'] = code == 0 and not cleanup['owned_processes_remaining'] and not cleanup['cleanup_errors']
            (output / 'summary.json').write_text(json.dumps(report, indent=2)); code = 0 if report['passed'] else 1
    print(json.dumps(report) if report else 'FAILED')
    return code


if __name__ == '__main__':
    sys.exit(main())
