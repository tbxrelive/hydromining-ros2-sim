#!/usr/bin/env python3
"""阶段七 7.0 回归：喷射位姿生成 + use_tcp 命中/脱靶行为。

命中工况：IK 解出喷嘴指向评估区中心的喷射位姿，移动到后按 TCP 喷射，
验证 has_hit、蚀除量与命中位置；脱靶工况：射线离线预检确认不命中后再喷，
验证零蚀除、零最大深度。网格真值只在本验收脚本中使用，不进入感知链路。
"""
import argparse
import json
import os
import subprocess
import sys
import threading
from pathlib import Path

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.signals import SignalHandlerOptions
from scipy.spatial.transform import Rotation
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.qos import QoSProfile, ReliabilityPolicy
from lifecycle_msgs.srv import GetState
from std_srvs.srv import Trigger

from hydromining_robot.msg import SprayStatus
from hydromining_robot.srv import SprayAction
from test_phase4_acceptance import AcceptanceTest, cleanup_acceptance

sys.path.insert(0, get_package_share_directory('hydromining_sim'))
from algorithms.perception.kinematics import Chain
from algorithms.perception.metrics import mesh_heights
from algorithms.perception.wall_frame import WallFrame
from algorithms.planning.spray_pose import solve_spray_pose


class SprayTcpTest(AcceptanceTest):
    """复用阶段四的进程/内存看护，叠加喷射位姿与 TCP 喷射验证。"""

    def __init__(self, output, process):
        super().__init__(output, process)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.spray_status_ = []
        self.create_subscription(SprayStatus, '/spray/status', self.spray_status_.append, qos)

    def move(self, joints, seconds=6):
        """与 phase6_record 相同的轨迹下发与到位确认。"""
        if hasattr(self, 'fixed_joints_'):
            del self.fixed_joints_
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['panda_joint' + str(i) for i in range(1, 8)]
        goal.trajectory.points = [JointTrajectoryPoint(positions=list(joints), time_from_start=Duration(sec=seconds))]
        handle = self.future(self.motion_.send_goal_async(goal))
        assert handle.accepted
        result = self.future(handle.get_result_async(), 90)
        assert result.result.error_code == 0, result.result.error_string
        self.wait(lambda: self.joints_ is not None and np.max(np.abs(self.joints_ - joints)) < .02, 15,
                  'Arm did not reach the spray pose')

    def tf_matrix(self, target='world', source='panda_link0'):
        tf = self.tf_.lookup_transform(target, source, rclpy.time.Time())
        t, q = tf.transform.translation, tf.transform.rotation
        T = np.eye(4)
        T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        T[:3, 3] = [t.x, t.y, t.z]
        return T

    def spray_tcp(self):
        """use_tcp=True：服务按当前 TCP 位姿计算喷射几何，位置/方向字段忽略。"""
        request = SprayAction.Request(use_tcp=True, pressure=20., duration=12., enabled=True)
        return self.future(self.spray_.call_async(request), 240)

    def reset(self):
        reply = self.future(self.reset_.call_async(Trigger.Request()), 60)
        assert reply.success, reply.message
        return Path(reply.message)

    def latest_status(self):
        assert self.spray_status_, 'no /spray/status messages'
        return self.spray_status_[-1]

    def lifecycle_active(self):
        """真实 active 状态判定：服务存在且当前生命周期 id=3。"""
        self.wait(self.state_.service_is_ready, 90, 'lifecycle service missing')
        state = self.future(self.state_.call_async(GetState.Request()), 10)
        return state.current_state.id == 3

    def run(self, chain, views, center, normal):
        self.wait(self.lifecycle_active, 180, 'spray lifecycle not active')
        self.wait(self.motion_.server_is_ready, 90, 'trajectory action missing')
        T_world_base = self.tf_matrix()
        seed = views['front_0p8']['joints']
        report = {}

        # ---- 命中工况：喷嘴沿 -normal 指向评估区中心 ----
        into_wall = -np.asarray(normal, float)
        hit = None
        for standoff in (0.7, 0.6, 0.5, 0.45, 0.4):
            candidate = solve_spray_pose(chain, T_world_base, center, into_wall, standoff, seed)
            if candidate['reachable']:
                hit = candidate
                break
        assert hit is not None, 'no reachable spray pose at any standoff'
        self.move(hit['joints'])
        baseline = self.reset()
        before_status = len(self.spray_status_)
        reply = self.spray_tcp()
        assert reply.success, reply.message
        assert reply.volume_removed_m3 > 1e-5, f'expected erosion, got {reply.volume_removed_m3}'
        # geometry 在服务内部更新前会短暂沿用上一发状态，应看本次喷射结束后的最后一条。
        fresh = self.spray_status_[before_status:]
        assert fresh and fresh[-1].shot_id > 0 and fresh[-1].has_hit, \
            'spray status did not report a hit'
        status = self.latest_status()
        frame = WallFrame.from_normal(center, normal)
        mesh_pose = [1.5, 0., 1.5, 0., -1.9198621771937625, 0.]
        before = mesh_heights(baseline, mesh_pose, frame, .5, .004)
        after = mesh_heights(Path(reply.output_mesh_path), mesh_pose, frame, .5, .004)
        retreat = np.clip(before - after, 0, None)
        significant = retreat > 0.005
        assert significant.any() and retreat.max() > 0.01
        u, v = np.meshgrid(np.arange(retreat.shape[0]), np.arange(retreat.shape[1]), indexing='ij')
        centroid_uv = np.array([u[significant], v[significant]]) @ retreat[significant]
        centroid_uv /= retreat[significant].sum()
        # 高度图索引以 frame 中心即评估区中心为原点。
        centroid_offset = np.linalg.norm((centroid_uv - (np.array(retreat.shape) - 1) / 2) * .004)
        report['hit'] = {'standoff': hit['tcp_world'][:3, 3].tolist(),
                         'position_error_m': hit['position_error_m'],
                         'volume_removed_m3': reply.volume_removed_m3,
                         'has_hit': bool(status.has_hit),
                         'max_retreat_m': float(retreat.max()),
                         'erosion_centroid_offset_m': float(centroid_offset)}
        assert centroid_offset < 0.2, f'erosion centroid {centroid_offset:.3f} m off target'

        # ---- 脱靶工况：TCP 保持在可达空间，射流朝天/朝地等离线预检确认脱靶 ----
        self.reset()
        miss = None
        miss_candidates = [([0.459, -0.615, -0.405], [-0.14, -0.33, -0.93]),
                           ([0.31, -0.97, 0.005], [-0.37, -0.88, -0.3]),
                           ([0.55, -1.02, 0.005], [0., -0.96, -0.3]),
                           ([0.6, 0., 1.15], [0., 0., 1.])]
        for aim, direction in miss_candidates:
            direction = np.asarray(direction, float) / np.linalg.norm(direction)
            candidate = solve_spray_pose(chain, T_world_base, aim, direction, 0.7, seed)
            if not candidate['reachable']:
                continue
            tcp = candidate['tcp_world']
            jet_z = tcp[:3, 2]
            hit_distance = self.ray_check_(baseline, mesh_pose, tcp[:3, 3] - 0.05 * jet_z, jet_z)
            if hit_distance is None:
                miss = candidate
                break
        assert miss is not None, 'no reachable off-wall spray pose found'
        self.move(miss['joints'])
        before_status = len(self.spray_status_)
        reply = self.spray_tcp()
        assert reply.success, reply.message
        assert reply.volume_removed_m3 == 0, f'miss must not erode, got {reply.volume_removed_m3}'
        assert reply.max_depth_m == 0
        fresh = self.spray_status_[before_status:]
        assert fresh and fresh[-1].shot_id > 0 and not fresh[-1].has_hit, \
            'off-wall spray reported a hit'
        report['miss'] = {'volume_removed_m3': reply.volume_removed_m3,
                          'has_hit': False, 'max_depth_m': reply.max_depth_m}
        report['passed'] = True
        report['aggregate_peak_rss_mib'] = self.peak_rss_ / 1024 ** 2
        return report

    def ray_check_(self, mesh_path, mesh_pose, source, direction):
        """与 hit_geometry 相同的射线求交，只读真值网格做离线命中预检。"""
        import open3d as o3d
        import trimesh
        mesh = trimesh.load(mesh_path, process=False, force='mesh')
        vertices = (np.asarray(mesh.vertices, dtype=np.float32) @ Rotation.from_euler(
            'xyz', mesh_pose[3:]).as_matrix().T + np.asarray(mesh_pose[:3], np.float32)
            ).astype(np.float32)
        scene = o3d.t.geometry.RaycastingScene(nthreads=1)
        scene.add_triangles(o3d.t.geometry.TriangleMesh(
            o3d.core.Tensor(vertices), o3d.core.Tensor(np.asarray(mesh.faces, np.uint32))))
        ray = np.r_[np.asarray(source, np.float32), np.asarray(direction, np.float32)][None, :]
        t = float(scene.cast_rays(o3d.core.Tensor(ray))['t_hit'].numpy()[0])
        return t if np.isfinite(t) and t > 1e-6 else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    output = Path(args.output).resolve()
    if output.exists() and any(output.iterdir()):
        parser.error('output directory must be new or empty')
    output.mkdir(parents=True, exist_ok=True)
    os.environ.update({'ROS_LOG_DIR': str(output / 'ros_logs'), 'ROS_DOMAIN_ID': '54',
                       'GZ_PARTITION': 'hydromining_phase7_tcp_' + str(os.getpid())})
    robot = Path(get_package_share_directory('hydromining_robot'))
    sim = Path(get_package_share_directory('hydromining_sim'))
    config = yaml.safe_load((robot / 'config/viewpoints.yaml').read_text())
    views = {name: config['viewpoints'][name] for name in ('front_0p8', 'left_0p8', 'right_0p8')}
    chain = Chain(subprocess.run(['xacro', str(robot / 'urdf/panda_with_water_gun.urdf.xacro'),
                                  'name:=panda', 'arm_id:=panda'],
                                 capture_output=True, text=True, check=True).stdout,
                'panda_link0', 'tcp')
    command = ['ros2', 'launch', 'hydromining_robot', 'robot_simulation.launch.py', 'gui:=false',
               'world:=' + str(sim / 'worlds/coal_wall_acceptance.sdf'), 'start_spray:=true',
               'enable_effects:=true',
               'spray_config:=' + str(robot / 'config/spray_acceptance.yaml'),
               'spray_output_dir:=' + str(output / 'meshes'),
               'camera_rate:=5', 'camera_width:=320', 'camera_height:=240', 'lidar_rate:=2']
    os.sched_setaffinity(0, sorted(os.sched_getaffinity(0))[:2]); os.nice(10)
    log = (output / 'simulation.log').open('w')
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    interrupted = threading.Event()
    node = SprayTcpTest(output, process)
    node.interrupted_ = interrupted
    code, report = 1, None
    try:
        report = node.run(chain, views, config['region_center_world'], config['wall_normal_world'])
        code = 0
    except Exception:
        import traceback
        traceback.print_exc()
        (output / 'failure.json').write_text(json.dumps({'error': traceback.format_exc(limit=3)}))
    finally:
        cleanup = cleanup_acceptance(node, process)
        log.close()
        if report is not None:
            report.update(cleanup)
            report['passed'] = code == 0 and not cleanup['owned_processes_remaining']
            (output / 'summary.json').write_text(json.dumps(report, indent=2, allow_nan=False))
            code = 0 if report['passed'] else 1
    print(json.dumps(report, indent=2) if report else 'FAILED')
    return code


if __name__ == '__main__':
    sys.exit(main())
