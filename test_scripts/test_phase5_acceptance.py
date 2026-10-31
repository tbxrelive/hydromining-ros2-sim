#!/usr/bin/env python3
"""Real offscreen particle/sensor acceptance. Never opens a GUI without --gui."""
import argparse
from collections import deque
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
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs_py.point_cloud2 import read_points_numpy
from scipy.spatial.transform import Rotation
from std_msgs.msg import String
from std_srvs.srv import Trigger
from hydromining_robot.msg import SprayStatus
from hydromining_robot.srv import SprayAction
from ament_index_python.packages import get_package_share_directory
from test_phase4_acceptance import AcceptanceTest, cleanup_acceptance
sys.path.insert(0, get_package_share_directory('hydromining_sim'))
from algorithms.jet_effects import cloud_change


def xyz(msg):
    """Retain invalid returns and organized pixel correspondence."""
    return read_points_numpy(msg, field_names=['x','y','z'], skip_nans=False).reshape(-1,3).copy()


def pixels(msg):
    """Honor row stride instead of assuming tightly packed images."""
    return np.ndarray((msg.height,msg.width,3), np.uint8, buffer=msg.data,
                      strides=(msg.step,3,1)).copy()


class Phase5Acceptance(AcceptanceTest):
    """Bounded native particle measurements, cancellation, reset and replay."""

    def __init__(self, output, process):
        super().__init__(output, process)
        self.status_ = None
        self.status_log_ = []
        self.diag_ = None
        self.lidar_ = self.depth_ = None
        self.overview_ = None
        self.record_ = False
        self.samples_ = {k: deque(maxlen=12) for k in ('rgb','cloud','lidar','depth')}
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        state = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(SprayStatus, '/spray/status', self.on_status, state)
        self.create_subscription(String, '/hydromining/jet/diagnostics', self.on_diag, state)
        self.create_subscription(PointCloud2, '/hydromining/lidar/points', self.on_lidar, qos)
        self.create_subscription(Image, '/hydromining/camera/depth/image_rect_raw', self.on_depth, qos)
        self.create_subscription(Image, '/hydromining/jet/overview', self.on_overview, qos)
        self.parameters_ = self.create_client(SetParameters, '/spray_controller_node/set_parameters')

    def on_status(self, msg):
        self.status_ = msg
        if len(self.status_log_) < 18000:
            self.status_log_.append({'shot':msg.shot_id,'state':msg.state,'elapsed':msg.elapsed,
                                     'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                                     'strength':msg.effect_strength,'has_hit':msg.has_hit})

    def on_diag(self, msg):
        self.diag_ = list(map(float,msg.data.split()))

    def on_cloud(self, msg):
        super().on_cloud(msg)
        if self.record_:
            self.samples_['cloud'].append(xyz(msg))

    def on_image(self, msg):
        super().on_image(msg)
        if self.record_:
            self.samples_['rgb'].append(pixels(msg))

    def on_lidar(self, msg):
        self.lidar_ = msg
        if self.record_:
            self.samples_['lidar'].append(xyz(msg))

    def on_depth(self, msg):
        self.depth_ = msg
        if self.record_:
            dtype = np.dtype('>f4' if msg.is_bigendian else '<f4')
            self.samples_['depth'].append(np.ndarray((msg.height,msg.width),dtype,
                                         buffer=msg.data,strides=(msg.step,4)).copy())

    def on_overview(self, msg):
        self.overview_ = msg

    def configure(self, **values):
        """All settings changes are checked by the actual parameter service."""
        parameters = [Parameter(k,value=v).to_parameter_msg() for k,v in values.items()]
        response = self.future(self.parameters_.call_async(SetParameters.Request(parameters=parameters)),10)
        assert all(r.successful for r in response.results),response

    def collect(self, seconds):
        for frames in self.samples_.values():
            frames.clear()
        self.record_ = True
        start = self.get_clock().now().nanoseconds
        self.wait(lambda:(self.get_clock().now().nanoseconds-start)*1e-9 >= seconds and
                  all(self.samples_.values()),
                  max(30,seconds*5),'Sensor collection did not advance')
        self.record_ = False
        assert all(self.samples_.values()), {k:len(v) for k,v in self.samples_.items()}
        return {k:np.stack(v) for k,v in self.samples_.items()}

    def paired_shot(self, name, preset='medium', duration=12., replay=None):
        """Compare a fixed current wall before/during particles, never across erosion."""
        self.configure(effect_preset=preset)
        clean = self.collect(2.6)
        previous = self.status_.shot_id
        request = SprayAction.Request(use_tcp=replay is None, pressure=20.,
                                      duration=duration, enabled=True)
        if replay is not None:
            request.position_x,request.position_y,request.position_z = replay['position_mesh']
            request.direction_x,request.direction_y,request.direction_z = replay['direction_mesh']
        future = self.spray_.call_async(request)
        self.wait(lambda:future.done() or (self.status_ is not None and self.status_.shot_id>previous and
                  self.status_.state=='SPRAYING'),90,'Shot never entered SPRAYING')
        assert not future.done(), future.result().message
        self.wait(lambda:self.diag_ is not None and (self.diag_[1]>0 or preset=='off'),10,
                  'Plugin did not start')
        busy = self.future(self.spray_.call_async(SprayAction.Request(use_tcp=True,pressure=20.,duration=1.,enabled=True)),5)
        assert not busy.success and 'Busy' in busy.message
        buildup = self.get_clock().now().nanoseconds
        self.wait(lambda:(self.get_clock().now().nanoseconds-buildup)*1e-9>=1.,15,'Mist buildup')
        noisy = self.collect(min(4.,duration-2))
        if self.overview_ is not None:
            from PIL import Image as Png
            Png.fromarray(pixels(self.overview_)).save(self.output_/(name+'_overview.png'))
        diag = self.diag_.copy()
        result = self.future(future,90)
        assert result.success,result.message
        self.wait(lambda:self.diag_ is not None and self.diag_[1]==0 and self.diag_[2]==0,15,
                  'Particles did not drain')
        recovered = self.collect(1.5)
        # Base sensor ray directions are fixed. Use only target returns in clean clouds.
        base = clean['cloud'][-1]
        mask = np.isfinite(base).all(1) & (base[:,0]>.3) & (base[:,0]<1.8)
        metrics = [cloud_change(base, frame, mask) for frame in noisy['cloud']]
        lidar_base = clean['lidar'][-1]
        lidar_mask = np.isfinite(lidar_base).all(1) & (np.linalg.norm(lidar_base,axis=1)<4.)
        lm = [cloud_change(lidar_base,frame,lidar_mask,.02) for frame in noisy['lidar']]
        rgb_difference = float(np.mean(np.abs(noisy['rgb'].astype(float)-clean['rgb'][-1].astype(float))))
        depth_base=clean['depth'][-1]
        valid=np.isfinite(depth_base)&(depth_base>.3)&(depth_base<1.8)
        errors=np.abs(noisy['depth'][:,valid]-depth_base[valid])
        depth_disturbed=float(np.mean(~np.isfinite(errors)|(errors>.01)))
        recovered_base = recovered['cloud'][-1]
        recovered_mask = np.isfinite(recovered_base).all(1) & (recovered_base[:,0]>.3) & (recovered_base[:,0]<1.8)
        recovery = float(np.mean([cloud_change(recovered_base, frame, recovered_mask)['disturbed_fraction']
                                  for frame in recovered['cloud']]))
        assert recovery < .001, ('Cloud did not stabilize after particles drained', recovery)
        np.savez_compressed(self.output_/(name+'_clean.npz'), **clean)
        np.savez_compressed(self.output_/(name+'_during.npz'), **noisy)
        np.savez_compressed(self.output_/(name+'_recovered.npz'), **recovered)
        from PIL import Image as Png
        Png.fromarray(noisy['rgb'][-1]).save(self.output_/(name+'_during.png'))
        report={'preset':preset,'rgb_mean_abs_change':rgb_difference,
                'depth_disturbed_fraction':depth_disturbed,
                'cloud_visible_fraction':float(np.mean([m['visible_fraction'] for m in metrics])),
                'cloud_disturbed_fraction':float(np.mean([m['disturbed_fraction'] for m in metrics])),
                'lidar_disturbed_fraction':float(np.mean([m['disturbed_fraction'] for m in lm])),
                'recovered_temporal_disturbed_fraction':recovery,
                'plugin_during':diag,'statistics':json.loads(result.statistics_json)}
        self.get_logger().info(name+': '+json.dumps(report))
        return result,report

    def run(self):
        self.ready()
        self.wait(lambda:self.diag_ is not None and self.lidar_ is not None and self.depth_ is not None,30,
                  'Effects bridge or sensor missing')
        invalid = [Parameter('effect_reference_pressure_mpa', value=0.).to_parameter_msg(),
                   Parameter('jet_mist_rate_scale', value=2.).to_parameter_msg(),
                   Parameter('jet_splash_spread', value=.8).to_parameter_msg()]
        rejection = self.future(self.parameters_.call_async(SetParameters.Request(parameters=invalid)),10)
        assert all(not result.successful for result in rejection.results),rejection
        before=self.capture('01_before')
        first,medium=self.paired_shot('medium')
        after=self.capture('02_after_first')
        tf = self.tf_.lookup_transform('world','tcp',rclpy.time.Time())
        t,q = tf.transform.translation,tf.transform.rotation
        position = np.array([t.x,t.y,t.z])
        direction = Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()[:,2]
        cloud1 = self.compare_clouds(before,after,position,direction)
        assert cloud1['changed_points'] >= 20 and cloud1['p95_retreat_m'] > .01,cloud1
        assert cloud1['reference_p95_abs_m'] is not None and cloud1['reference_p95_abs_m'] < .005,cloud1
        assert medium['rgb_mean_abs_change']>.01,medium
        assert medium['cloud_disturbed_fraction']>.001 and medium['cloud_visible_fraction']>=.3,medium
        assert medium['depth_disturbed_fraction']>.001,medium
        assert medium['lidar_disturbed_fraction']>0,medium
        assert first.volume_removed_m3>0 and first.max_depth_m>0
        second,second_report=self.paired_shot('second')
        after2 = self.capture('03_after_second')
        cloud2 = self.compare_clouds(before,after2,position,direction)
        assert cloud2['p95_retreat_m'] > cloud1['p95_retreat_m'],(cloud1,cloud2)
        assert second.cumulative_volume_m3>first.cumulative_volume_m3
        reset=self.future(self.reset_.call_async(Trigger.Request()),40)
        assert reset.success,reset.message
        restored=self.capture('04_reset')
        reset_error = float(np.nanpercentile(np.linalg.norm(restored-before,axis=1),95))
        assert reset_error < .005
        replay,replay_report=self.paired_shot('replay',replay=json.loads(first.statistics_json))
        self.capture('05_replay')
        assert Path(first.output_mesh_path).read_bytes()==Path(replay.output_mesh_path).read_bytes()
        scans={}
        for preset in ('off','low','high'):
            assert self.future(self.reset_.call_async(Trigger.Request()),40).success
            _,scans[preset]=self.paired_shot(preset,preset=preset,duration=8.)
        assert scans['off']['cloud_disturbed_fraction']<.001,scans['off']
        assert scans['off']['rgb_mean_abs_change']<.01,scans['off']
        assert scans['high']['cloud_disturbed_fraction']>scans['low']['cloud_disturbed_fraction'],scans
        # Verify configurable rates/ranges and pressure mapping through native
        # emitter diagnostics, not only the ROS parameter response.
        assert self.future(self.reset_.call_async(Trigger.Request()),40).success
        self.configure(effect_preset='medium', effect_reference_pressure_mpa=40.,
                       jet_mist_rate_scale=.5, jet_splash_rate_scale=0., jet_dust_rate_scale=0.,
                       jet_mist_spread=.15, jet_splash_spread=.12, jet_dust_spread=.18)
        _, configured = self.paired_shot('configured', duration=4.)
        assert configured['plugin_during'][3]==2,configured
        length = configured['plugin_during'][5]
        speed = configured['statistics']['outlet_velocity_m_s']
        # At 20 MPa with a 40 MPa reference, strength=0.5; mist scale=0.5.
        expected_rate = .5*max(80., 30.*speed/length) + .5*110*.5
        assert np.isclose(configured['plugin_during'][4], expected_rate, rtol=1e-5),configured
        self.configure(effect_reference_pressure_mpa=20., jet_mist_rate_scale=1.,
                       jet_splash_rate_scale=1., jet_dust_rate_scale=1.,
                       jet_mist_spread=.24, jet_splash_spread=.10, jet_dust_spread=.20)
        # Cancel a timed shot by reset; service remains responsive and no candidate commits.
        self.configure(effect_preset='medium')
        prior=self.status_.shot_id
        pending=self.spray_.call_async(SprayAction.Request(use_tcp=True,pressure=20.,duration=12.,enabled=True))
        self.wait(lambda:self.status_.shot_id>prior and self.status_.state=='SPRAYING',90,'Cancel shot missing')
        # Pause must freeze the requested exposure, not consume wall-clock time.
        control=['gz','service','-s','/world/hydromining_world/control',
                 '--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean',
                 '--timeout','2000','--req']
        paused=subprocess.run(control+['pause: true'],capture_output=True,text=True,timeout=5)
        assert paused.returncode==0 and 'true' in paused.stdout,paused
        drain=time.monotonic()
        self.wait(lambda:time.monotonic()-drain>.2,5,'Clock queue drain')
        frozen=self.get_clock().now().nanoseconds
        wall=time.monotonic()
        self.wait(lambda:time.monotonic()-wall>.6,5,'Pause check')
        assert abs(self.get_clock().now().nanoseconds-frozen)<5_000_000
        assert not pending.done()
        resumed=subprocess.run(control+['pause: false'],capture_output=True,text=True,timeout=5)
        assert resumed.returncode==0 and 'true' in resumed.stdout,resumed
        assert self.future(self.reset_.call_async(Trigger.Request()),40).success
        assert not self.future(pending,10).success
        self.wait(lambda:self.diag_[1]==0 and self.diag_[2]==0 and self.diag_[3]==0,10,'Reset retained particles')
        # A no-hit action must create only the free stream, never impact clouds.
        shot=json.loads(first.statistics_json)
        request=SprayAction.Request(pressure=20.,duration=4.,enabled=True)
        request.position_x,request.position_y,request.position_z=shot['position_mesh']
        request.direction_x,request.direction_y,request.direction_z=-np.array(shot['direction_mesh'])
        prior=self.status_.shot_id
        pending=self.spray_.call_async(request)
        self.wait(lambda:self.status_.shot_id>prior and self.status_.state=='SPRAYING',40,'No-hit shot missing')
        self.wait(lambda:self.diag_[1]>0,10,'No-hit stream missing')
        assert not self.status_.has_hit and self.diag_[3]==1,self.diag_
        nohit=self.future(pending,40)
        assert nohit.success and nohit.volume_removed_m3==0,nohit.message
        phases={s['state'] for s in self.status_log_}
        assert {'PREPARING','SPRAYING','UPDATING','SETTLING','IDLE','RESETTING'}<=phases,phases
        (self.output_/'status.json').write_text(json.dumps(self.status_log_))
        np.savez_compressed(self.output_/'joint_hold.npz',samples=np.array(self.joint_hold_samples_))
        return {'passed':True,'medium':medium,'second':second_report,'replay':replay_report,
                'preset_scan':scans,'reset_cancels_shot':True,'no_hit_has_no_impact':True,
                'configured_effects':configured,'invalid_effect_settings_rejected':True,
                'paused_clock_freezes_exposure':True,
                'replay_mesh_identical':True,'joint_drift_rad':self.max_joint_drift_,
                'cloud_after_first':cloud1,'cloud_after_second':cloud2,'reset_p95_m':reset_error,
                'aggregate_peak_rss_mib':self.peak_rss_/1024**2,'calibrated':False}


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--gui',action='store_true')
    parser.add_argument('--keep-open',action='store_true')
    parser.add_argument('--overview',action=argparse.BooleanOptionalAction,default=None,
                        help='Extra screenshot camera (default: headless only; GUI already supplies a view)')
    parser.add_argument('--output',default='/tmp/hydromining_phase5_acceptance')
    args=parser.parse_args()
    output=Path(args.output).resolve();output.mkdir(parents=True,exist_ok=True)
    for stale in ('summary.json','failure.json','checks.json','resources.jsonl'):
        (output/stale).unlink(missing_ok=True)
    os.environ['ROS_LOG_DIR']=str(output/'ros_logs')
    os.environ['ROS_DOMAIN_ID']='47'
    os.environ['GZ_PARTITION']='hydromining_phase5_'+str(os.getpid())
    robot=Path(get_package_share_directory('hydromining_robot'))
    sim=Path(get_package_share_directory('hydromining_sim'))
    overview = not args.gui if args.overview is None else args.overview
    world = sim/'worlds'/('coal_wall_phase5.sdf' if overview else 'coal_wall_acceptance.sdf')
    command=['ros2','launch','hydromining_robot','robot_simulation.launch.py',
             'gui:='+str(args.gui).lower(),'world:='+str(world),
             'start_spray:=true','enable_effects:=true',
             'spray_config:='+str(robot/'config/spray_acceptance.yaml'),
             'spray_output_dir:='+str(output/'meshes'),
             'camera_rate:=5','camera_width:=320','camera_height:=240','lidar_rate:=2']
    __import__("runtime_resources").reserve_desktop_cpu();os.nice(10)
    log=(output/'simulation.log').open('w')
    process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    interrupted=threading.Event()
    signal.signal(signal.SIGINT,lambda signum,frame:interrupted.set())
    signal.signal(signal.SIGTERM,lambda signum,frame:interrupted.set())
    node=Phase5Acceptance(output,process)
    node.interrupted_=interrupted
    report=None;code=1;checks_complete=False
    try:
        report=node.run();code=0
        if args.keep_open:
            # Boundary tests finish on a reset wall with a no-hit stream. Leave
            # a real, reproducible crater for final inspection instead.
            _, report['final_view'] = node.paired_shot('final_view', replay=report['medium']['statistics'])
            node.capture('06_final_view')
        report.update({'GUI_started':args.gui, 'overview_camera':overview,
                       'world':world.name, 'resource_limits_mib':dict(zip(('rss','available_minimum'), [v/1024**2 for v in __import__('runtime_resources').memory_limits()])),
                       'aggregate_peak_rss_mib':node.peak_rss_/1024**2})
        (output/'checks.json').write_text(json.dumps(report,indent=2))
        checks_complete=True
        print('All acceptance checks passed; results: '+str(output/'checks.json'),flush=True)
        if args.keep_open:
            print('Acceptance finished; Ctrl+C closes the owned session.',flush=True)
            while process.poll() is None and rclpy.ok() and not interrupted.is_set():
                node.monitor()
                rclpy.spin_once(node,timeout_sec=.1)
    except (KeyboardInterrupt,ExternalShutdownException):
        code=0 if checks_complete else 130
    except Exception as error:
        import traceback
        traceback.print_exc()
        (output/'failure.json').write_text(json.dumps({'error':str(error),
            'peak_rss_mib':node.peak_rss_/1024**2,
            'resources':getattr(node,'resource_snapshot_',None)},indent=2))
        code=1
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        cleanup=cleanup_acceptance(node,process)
        if report is not None:
            report.update(cleanup)
            report['aggregate_peak_rss_mib']=node.peak_rss_/1024**2
            report['passed']=code==0 and not cleanup['owned_processes_remaining'] and not cleanup['cleanup_errors']
            (output/'summary.json').write_text(json.dumps(report,indent=2))
            print(json.dumps(report,indent=2),flush=True)
        if cleanup['owned_processes_remaining'] or cleanup['cleanup_errors']:
            code=1
        log.close()
    return code


if __name__=='__main__':
    sys.exit(main())
