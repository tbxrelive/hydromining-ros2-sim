#!/usr/bin/env python3
"""Six-step acceptance against real headless Gazebo RGB-D data.

Owns only its launched process group; it never kills other user simulations.
Limits affinity to two CPU cores and monitors aggregate RSS/available RAM.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import threading
import numpy as np
import psutil
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2, JointState, Image
from sensor_msgs_py.point_cloud2 import read_points_numpy
from std_srvs.srv import Trigger
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from tf2_ros import Buffer, TransformListener
from lifecycle_msgs.srv import GetState
from hydromining_robot.srv import SprayAction
from ament_index_python.packages import get_package_share_directory


class AcceptanceTest(Node):
    """Bounded waits, fresh frames and sensor-based comparison with a fixed arm."""

    TARGET = [-0.008505014392977845, -1.4199413710436999, 0.010434980743538633,
              -2.8105112161245147, 0.038694409085547034, 2.961313735583135, 0.785]

    def __init__(self, output, process):
        super().__init__('phase4_acceptance', parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.output_ = output
        (self.output_/'resources.jsonl').unlink(missing_ok=True)
        self.interrupted_ = threading.Event()
        self.process_ = process
        self.peak_rss_ = 0
        self.owned_processes_ = {}
        self.cloud_ = None
        self.image_ = None
        self.cloud_seq_ = 0
        self.joints_ = None
        self.joint_hold_samples_ = []
        self.max_joint_drift_ = 0.0
        self.tf_ = Buffer()
        self.listener_ = TransformListener(self.tf_, self)
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        state_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PointCloud2,'/hydromining/camera/depth/points',self.on_cloud,sensor_qos)
        self.create_subscription(Image,'/hydromining/camera/color/image_raw',self.on_image,sensor_qos)
        self.create_subscription(JointState,'/joint_states',self.on_joints,state_qos)
        self.spray_ = self.create_client(SprayAction,'/spray_action')
        self.reset_ = self.create_client(Trigger,'/reset_coal_wall')
        self.state_ = self.create_client(GetState,'/spray_controller_node/get_state')
        self.motion_ = ActionClient(self,FollowJointTrajectory,'/panda_arm_controller/follow_joint_trajectory')

    def on_cloud(self, msg):
        self.cloud_ = msg
        self.cloud_seq_ += 1

    def on_image(self, msg):
        self.image_ = msg

    def on_joints(self, msg):
        mapping = dict(zip(msg.name,msg.position))
        if all('panda_joint'+str(i) in mapping for i in range(1,8)):
            self.joints_ = np.array([mapping['panda_joint'+str(i)] for i in range(1,8)])
            if hasattr(self,'fixed_joints_'):
                drift = float(np.max(np.abs(self.joints_-self.fixed_joints_)))
                self.max_joint_drift_ = max(self.max_joint_drift_,drift)
                stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                self.joint_hold_samples_.append([stamp,*self.joints_])

    def monitor(self):
        """Abort before test workers exhaust desktop memory."""
        if self.process_.poll() is not None:
            raise RuntimeError(f'Simulation exited ({self.process_.returncode}); see simulation.log')
        if self.max_joint_drift_ >= .002:
            raise AssertionError('Arm moved during the fixed-arm interval')
        # /clock can wake spin_once at 1 kHz. Scanning the whole process tree on
        # each message consumed a core and starved the bounded erosion solver.
        now = time.monotonic()
        if now - getattr(self, 'last_monitor_', 0.) < .5:
            return
        self.last_monitor_ = now
        root = psutil.Process(self.process_.pid)
        rss = 0
        processes = []
        children = root.children(recursive=True)
        self.owned_processes_.update({p.pid:p for p in children})
        for p in [root]+children+[psutil.Process()]:
            try:
                used = p.memory_info().rss
                rss += used
                processes.append({'pid':p.pid, 'name':p.name(), 'rss_mib':used/1024**2})
            except psutil.NoSuchProcess:
                pass
        self.peak_rss_ = max(self.peak_rss_,rss)
        available = psutil.virtual_memory().available
        snapshot = {'wall_time':time.time(), 'rss_mib':rss/1024**2,
                    'available_mib':available/1024**2,
                    'state':getattr(getattr(self, 'status_', None), 'state', 'UNKNOWN'),
                    'processes':sorted(processes,key=lambda p:p['rss_mib'],reverse=True)}
        self.resource_snapshot_ = snapshot
        if now-getattr(self, 'last_resource_log_', 0.) >= 2. or rss > 3*1024**3 or available < 1024**3:
            with (self.output_/'resources.jsonl').open('a') as stream:
                stream.write(json.dumps(snapshot)+'\n')
            self.last_resource_log_ = now
        if rss > 3*1024**3 or available < 1024**3:
            raise RuntimeError(f'Memory guard: aggregate RSS {rss/1024**2:.1f} MiB '
                               f'(limit 3072); available RAM {available/1024**2:.1f} MiB '
                               '(minimum 1024); see resources.jsonl')
        if self.max_joint_drift_ >= .002:
            raise AssertionError('Arm moved during the fixed-arm interval')

    def wait(self, predicate, timeout=60, label='condition'):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            if self.interrupted_.is_set():
                raise KeyboardInterrupt
            self.monitor()
            if predicate():
                return
            rclpy.spin_once(self,timeout_sec=.05)
        raise TimeoutError(label)

    def future(self, future, timeout=60):
        self.wait(future.done,timeout,'ROS future timed out')
        return future.result()

    def ready(self):
        """Require actual active lifecycle state, not just an advertised service."""
        self.wait(self.state_.service_is_ready,90,'Lifecycle service missing')
        deadline = time.monotonic()+90
        while time.monotonic() < deadline:
            # Services exist before heavy initial SDF/mesh construction completes.
            # First offscreen renderer startup can also stall physics activation.
            state = self.future(self.state_.call_async(GetState.Request()),30)
            if state.current_state.id == 3:
                break
            self.wait(lambda: self.cloud_ is not None,15,'No RGB-D frames')
        else:
            raise RuntimeError('Spray node did not activate')
        self.wait(self.motion_.server_is_ready,60,'Trajectory action missing')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['panda_joint'+str(i) for i in range(1,8)]
        point = JointTrajectoryPoint(positions=self.TARGET,time_from_start=Duration(sec=5))
        goal.trajectory.points = [point]
        handle = self.future(self.motion_.send_goal_async(goal))
        assert handle.accepted, 'Trajectory rejected'
        result = self.future(handle.get_result_async(),90)
        assert result.result.error_code == 0, result.result.error_string
        self.capture('settling')
        self.wait(lambda:self.joints_ is not None,10,'No joint states')
        self.fixed_joints_ = self.joints_.copy()
        assert np.max(np.abs(self.fixed_joints_-self.TARGET)) < .02
        self.get_logger().info('Arm positioned; now holding the same commanded joint state')

    def capture(self, name):
        """Wait three new rendered frames, transform at sensor timestamp, save PCD+NPZ."""
        seq = self.cloud_seq_
        self.wait(lambda:self.cloud_seq_ >= seq+3,30,'Fresh RGB-D frame timeout')
        msg = self.cloud_
        local = read_points_numpy(msg,field_names=['x','y','z'],skip_nans=False).reshape(-1,3)
        invalid = ~np.isfinite(local).all(axis=1)
        local[invalid] = 0
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        # Point clouds and /tf arrive independently. Wait for this exact stamp;
        # never relabel a moving sensor frame with a latest-time transform.
        try:
            self.wait(lambda:self.tf_.can_transform('world',msg.header.frame_id,stamp),
                      5,'TF did not reach the captured sensor timestamp')
        except TimeoutError as error:
            detail = {'cloud_stamp_ns':stamp.nanoseconds,'cloud_frame':msg.header.frame_id,
                      'ros_clock_ns':self.get_clock().now().nanoseconds,
                      'frames':self.tf_.all_frames_as_yaml(),
                      'reason':str(self.tf_.can_transform('world',msg.header.frame_id,stamp,
                                                        return_debug_tuple=True))}
            (self.output_/'tf_failure.json').write_text(json.dumps(detail,indent=2))
            raise error
        tf = self.tf_.lookup_transform('world',msg.header.frame_id,stamp)
        t,q = tf.transform.translation,tf.transform.rotation
        points = local@Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix().T+[t.x,t.y,t.z]
        points[invalid] = np.nan
        if name == 'settling':
            return points
        # These PNGs are actual offscreen Gazebo sensor renders, not synthetic plots.
        from PIL import Image as PngImage
        self.wait(lambda:self.image_ is not None and
                  rclpy.time.Time.from_msg(self.image_.header.stamp).nanoseconds >= stamp.nanoseconds,
                  10,'Fresh RGB image timeout')
        image = self.image_
        if image.encoding not in ('rgb8','bgr8'):
            raise ValueError('Unsupported image encoding: '+image.encoding)
        pixels = np.ndarray((image.height,image.width,3),dtype=np.uint8,
                            buffer=image.data,strides=(image.step,3,1)).copy()
        if image.encoding == 'bgr8':
            pixels = pixels[:,:,::-1]
        PngImage.fromarray(pixels).save(self.output_/(name+'_rgb.png'))
        np.savez_compressed(self.output_/(name+'.npz'),world=points,
                            local=local,tf_translation=[t.x,t.y,t.z],tf_quaternion=[q.x,q.y,q.z,q.w],
                            stamp=[msg.header.stamp.sec,msg.header.stamp.nanosec],
                            source_frame=msg.header.frame_id,frame='world',width=msg.width,height=msg.height)
        finite = points[np.isfinite(points).all(axis=1)]
        header = ('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                  f'WIDTH {len(finite)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(finite)}\nDATA ascii\n')
        with (self.output_/(name+'.pcd')).open('w') as f:
            f.write(header)
            np.savetxt(f,finite,fmt='%.7f')
        self.get_logger().info(f'{name}: {len(finite)} finite measured points (world frame)')
        return points

    def spray(self, replay=None):
        req = SprayAction.Request(use_tcp=replay is None,pressure=20.,duration=12.,enabled=True)
        if replay is not None:
            req.position_x,req.position_y,req.position_z = replay['position_mesh']
            req.direction_x,req.direction_y,req.direction_z = replay['direction_mesh']
        result = self.future(self.spray_.call_async(req),90)
        assert result.success,result.message
        assert result.volume_removed_m3 > 0
        assert result.max_depth_m > 0
        assert self.max_joint_drift_ < .002,'Arm moved during spray'
        self.get_logger().info(
            f'喷射结束：本次深度 {result.max_depth_m*100:.2f} cm，'
            f'本次去除体积 {result.volume_removed_m3*1e6:.1f} cm³；'
            f'累计深度 {result.cumulative_depth_m*100:.2f} cm，'
            f'累计体积 {result.cumulative_volume_m3*1e6:.1f} cm³')
        return result,json.loads(result.statistics_json)

    @staticmethod
    def compare_clouds(before,after,position,direction):
        """Same-pixel changes along jet axis; isolate the footprint and a reference annulus."""
        good = np.isfinite(before).all(1)&np.isfinite(after).all(1)
        delta = before-position
        radial = np.linalg.norm(delta-np.outer(delta@direction,direction),axis=1)
        # Coal is in front of the actual nozzle; exclude near gun and far background.
        distance = delta@direction
        region = good&(radial<.16)&(distance>.3)&(distance<1.5)
        reference = good&(radial>.23)&(radial<.35)&(distance>.3)&(distance<1.5)
        change = (after-before)@direction
        if np.count_nonzero(region) < 20:
            raise AssertionError('Insufficient measured points on the target footprint')
        return {'roi_points':int(region.sum()),'changed_points':int(np.count_nonzero(change[region]>.005)),
                'p95_retreat_m':float(np.percentile(change[region],95)),
                'reference_p95_abs_m':float(np.percentile(np.abs(change[reference]),95)) if reference.any() else None}

    def run(self):
        self.ready()
        before = self.capture('01_before')
        first,a = self.spray()
        after = self.capture('02_after_first')
        tf = self.tf_.lookup_transform('world','tcp',rclpy.time.Time())
        t,q = tf.transform.translation,tf.transform.rotation
        position = np.array([t.x,t.y,t.z])
        direction = Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()[:,2]
        cloud1 = self.compare_clouds(before,after,position,direction)
        assert cloud1['changed_points'] >= 20 and cloud1['p95_retreat_m'] > .01,cloud1
        assert cloud1['reference_p95_abs_m'] is not None and cloud1['reference_p95_abs_m'] < .005,cloud1
        second,b = self.spray()
        after2 = self.capture('03_after_second')
        cloud2 = self.compare_clouds(before,after2,position,direction)
        assert second.cumulative_volume_m3 > first.cumulative_volume_m3
        assert second.cumulative_depth_m > first.cumulative_depth_m
        assert cloud2['p95_retreat_m'] > cloud1['p95_retreat_m'],(cloud1,cloud2)
        reset = self.future(self.reset_.call_async(Trigger.Request()),30)
        assert reset.success,reset.message
        restored = self.capture('04_reset')
        reset_delta = np.linalg.norm(restored-before,axis=1)
        reset_p95 = float(np.nanpercentile(reset_delta,95))
        assert reset_p95 < .005,reset_p95
        replay,c = self.spray(replay=a)
        replay_cloud = self.capture('05_replay')
        mesh_equal = hashlib.sha256(Path(first.output_mesh_path).read_bytes()).hexdigest() == hashlib.sha256(Path(replay.output_mesh_path).read_bytes()).hexdigest()
        assert mesh_equal,'Identical recorded action did not regenerate identical mesh'
        assert replay.volume_removed_m3 == first.volume_removed_m3
        error = float(np.nanpercentile(np.linalg.norm(replay_cloud-after,axis=1),95))
        assert error < .005,error
        # Bad inputs must be rejected and leave the process responsive.
        bad = SprayAction.Request(pressure=-1.,duration=1.,direction_z=-1.,enabled=True)
        rejected = self.future(self.spray_.call_async(bad),10)
        assert not rejected.success
        assert self.joint_hold_samples_ and self.max_joint_drift_ < .002
        np.savez_compressed(self.output_/'joint_hold.npz',
                            samples=np.array(self.joint_hold_samples_),
                            reference=self.fixed_joints_,limit_rad=.002)
        return {'passed':True,'calibrated':False,'first':a,'second':b,'replay':c,
                'sensor_first':cloud1,'sensor_cumulative':cloud2,'reset_cloud_p95_m':reset_p95,
                'replay_cloud_p95_m':error,'replay_mesh_identical':mesh_equal,
                'joint_drift_rad':self.max_joint_drift_,
                'joint_hold_sample_count':len(self.joint_hold_samples_),
                'aggregate_peak_rss_mib':self.peak_rss_/1024**2,'invalid_input_rejected':True}


def live_processes(processes):
    """Filter exit races and reused PIDs using psutil process identities."""
    live=[]
    for process in processes:
        try:
            if process.is_running() and process.status()!=psutil.STATUS_ZOMBIE:
                live.append(process)
        except psutil.NoSuchProcess:
            pass
    return live


def cleanup_acceptance(node,process):
    """Always attempt owned process cleanup, even after ROS teardown errors."""
    errors=[]
    owned=dict(node.owned_processes_)
    try:
        owned.update({p.pid:p for p in psutil.Process(process.pid).children(recursive=True)})
    except psutil.NoSuchProcess:
        pass
    # Stop the command producer while its bridge/context are still alive. A
    # simultaneous launch-wide SIGINT can race a final ROS->Gazebo publication
    # against parameter_bridge teardown (native std::system_error/abort).
    producers=[]
    for child in owned.values():
        try:
            if any(Path(arg).name=='spray_controller_node.py' for arg in child.cmdline()):
                producers.append(child)
                child.send_signal(signal.SIGINT)
        except psutil.NoSuchProcess:
            pass
    if producers:
        _,pending=psutil.wait_procs(producers,timeout=5)
        if pending:
            errors.append('spray producer did not stop before bridge teardown')
    context=node.context
    try:
        node.destroy_node()
    except Exception as error:
        errors.append('destroy_node: '+str(error))
    try:
        rclpy.try_shutdown(context=context)
    except Exception as error:
        errors.append('try_shutdown: '+str(error))
    try:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill();process.wait(timeout=5)
    except Exception as error:
        errors.append('launch stop: '+str(error))
    live=live_processes(owned.values())
    for child in live:
        try:
            child.terminate()
        except psutil.NoSuchProcess:
            pass
        except psutil.AccessDenied as error:
            errors.append('child terminate: '+str(error))
    _,live=psutil.wait_procs(live,timeout=5)
    for child in live:
        try:
            child.kill()
        except psutil.NoSuchProcess:
            pass
        except psutil.AccessDenied as error:
            errors.append('child kill: '+str(error))
    psutil.wait_procs(live,timeout=5)
    return {'owned_processes_remaining':[p.pid for p in live_processes(owned.values())],
            'cleanup_errors':errors}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--gui',action='store_true',help='Explicit user visual acceptance only')
    parser.add_argument('--output',default='/tmp/hydromining_acceptance')
    parser.add_argument('--keep-open',action='store_true',help='Keep GUI or headless simulation open after checks')
    args = parser.parse_args()
    output = Path(args.output).resolve();output.mkdir(parents=True,exist_ok=True)
    for stale in ('summary.json', 'failure.json'):
        (output/stale).unlink(missing_ok=True)
    os.environ['ROS_LOG_DIR'] = str(output/'ros_logs')
    # Isolate our ROS graph and Gazebo transport from any user simulation.
    os.environ['ROS_DOMAIN_ID'] = '46'
    os.environ['GZ_PARTITION'] = 'hydromining_acceptance_'+str(os.getpid())
    robot = Path(get_package_share_directory('hydromining_robot'))
    sim = Path(get_package_share_directory('hydromining_sim'))
    command = ['ros2','launch','hydromining_robot','robot_simulation.launch.py',
               'gui:='+str(args.gui).lower(),'world:='+str(sim/'worlds/coal_wall_acceptance.sdf'),
               'start_spray:=true','spray_config:='+str(robot/'config/spray_acceptance.yaml'),
               'spray_output_dir:='+str(output/'meshes'),
               'camera_rate:=5','camera_width:=320','camera_height:=240','lidar_rate:=2']
    cpus = sorted(os.sched_getaffinity(0))[:2]
    os.sched_setaffinity(0,cpus)
    os.nice(10)
    log = (output/'simulation.log').open('w')
    process = subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    interrupted=threading.Event()
    signal.signal(signal.SIGINT,lambda signum,frame:interrupted.set())
    signal.signal(signal.SIGTERM,lambda signum,frame:interrupted.set())
    node = AcceptanceTest(output,process)
    node.interrupted_=interrupted
    code = 1
    report = None
    try:
        report = node.run()
        report.update({'mode':'GUI' if args.gui else 'headless','cpu_affinity':cpus})
        code = 0
        if args.keep_open:
            label = 'GUI' if args.gui else 'Headless simulation'
            print(label+' remains open for inspection; Ctrl+C closes this test session.',flush=True)
            while process.poll() is None and rclpy.ok() and not interrupted.is_set():
                # Retain identities even if launch exits before final cleanup.
                try:
                    node.owned_processes_.update({p.pid:p for p in psutil.Process(process.pid).children(recursive=True)})
                except psutil.NoSuchProcess:
                    pass
                rclpy.spin_once(node,timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C after --keep-open is a normal user exit, not a failed run.
        code = 0 if report is not None else 130
    except Exception as e:
        (output/'failure.json').write_text(json.dumps({'error':str(e),'peak_rss_mib':node.peak_rss_/1024**2},indent=2))
        import traceback
        traceback.print_exc()
    finally:
        # A terminal SIGINT may already have shut down the ROS context. Avoid a
        # second interrupt aborting cleanup; process termination is independent
        # of node/context teardown succeeding.
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        cleanup = cleanup_acceptance(node,process)
        if report is not None:
            report.update(cleanup)
            report['passed'] = report['passed'] and not cleanup['owned_processes_remaining'] and not cleanup['cleanup_errors']
            (output/'summary.json').write_text(json.dumps(report,indent=2))
            print(json.dumps(report,indent=2),flush=True)
        if cleanup['owned_processes_remaining'] or cleanup['cleanup_errors']:
            (output/'cleanup_failure.json').write_text(json.dumps(cleanup,indent=2))
            code = 1
        log.close()
    return code


if __name__ == '__main__':
    sys.exit(main())
