#!/usr/bin/env python3
"""Lifecycle ROS adapter for the existing local erosion model, Jazzy/Harmonic."""
import json
import sys
import signal
import threading
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from hydromining_robot.msg import SprayStatus
from rcl_interfaces.msg import SetParametersResult
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from hydromining_robot.srv import SprayAction
from gazebo_coal_wall import GazeboCoalWall

# Installed resource location, independent of current directory or symlink layout.
sys.path.insert(0, get_package_share_directory('hydromining_sim'))
from algorithms.spray_simulation import SpraySimulation
from algorithms.erosion_model import SprayAction as AlgorithmAction
from algorithms.mesh_converter import MeshConverter
from algorithms.jet_effects import effect_strength, hit_geometry, command_packet


class SprayControllerNode(LifecycleNode):
    """Serialize reset/spray transactions; TF is serviced by a separate executor thread."""

    EFFECT_SETTINGS = ('jet_mist_rate_scale', 'jet_splash_rate_scale', 'jet_dust_rate_scale',
                       'jet_mist_spread', 'jet_splash_spread', 'jet_dust_spread')

    def __init__(self):
        super().__init__('spray_controller_node')
        mesh = str(Path(get_package_share_directory('hydromining_sim'))/'models/coal_wall/meshes/coal_wall.obj')
        for name, value in [('coal_wall_mesh_path', mesh), ('output_mesh_dir', '/tmp/hydromining_meshes'),
                            ('voxel_resolution', [96, 96, 64]), ('dt', 0.2),
                            ('update_gazebo', True), ('world_name', 'hydromining_world'),
                            ('coal_pose', [2.5, 0.0, 1.5, 0.0, 0.0, 0.0]), ('tcp_frame', 'tcp'),
                            ('enable_effects', False), ('effect_preset', 'medium'),
                            ('solve_budget_s', 30.),
                            ('jet_core', True), ('jet_mist', True), ('jet_splash', True),
                            ('jet_dust', True), ('jet_radius', 0.008)]:
            self.declare_parameter(name, value)
        for name, value in zip(self.EFFECT_SETTINGS, (1., 1., 1., .24, .10, .20)):
            self.declare_parameter(name, value)
        self.declare_parameter('effect_reference_pressure_mpa', 20.)
        self.simulation_ = None
        self.active_ = False
        self.sequence_ = 0
        self.group_ = ReentrantCallbackGroup()
        self.operation_lock_ = threading.Lock()
        self.effect_lock_ = threading.RLock()
        self.cancel_ = threading.Event()
        self.phase_ = 'IDLE'
        self.shot_id_ = 0
        self.command_id_ = 0
        self.geometry_ = {'source': [0., 0., 0.], 'direction': [0., 0., 1.],
                          'hit': [0., 0., 2.], 'normal': [0., 0., -1.], 'has_hit': False}
        self.strength_ = 0.0
        self.eroding_ = False
        self.pressure_ = self.duration_ = self.started_ = 0.0
        self.detail_ = ''
        self.diagnostics_ = None
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.status_pub_ = self.create_publisher(SprayStatus, '/spray/status', qos)
        self.command_pub_ = self.create_publisher(String, '/hydromining/jet/command', qos)
        self.create_subscription(String, '/hydromining/jet/diagnostics', self.on_diagnostics, qos,
                                 callback_group=self.group_)
        self.create_timer(0.05, self.publish_effects, callback_group=self.group_)
        self.context.on_shutdown(self.cancel_.set)
        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)
        self.spray_service_ = self.create_service(SprayAction, '/spray_action', self.spray_callback,
                                                  callback_group=self.group_)
        self.reset_service_ = self.create_service(Trigger, '/reset_coal_wall', self.reset_callback,
                                                  callback_group=self.group_)
        self.add_on_set_parameters_callback(self.validate_parameters)
        self.get_logger().info('Spray lifecycle node initialized; parameters are uncalibrated')

    def validate_parameters(self, parameters):
        """Permit bounded dt updates; reject state-changing geometry updates when configured."""
        for p in parameters:
            if p.name == 'dt' and (not isinstance(p.value, (float, int)) or not np.isfinite(p.value) or not 0.01 <= p.value <= 0.5):
                return SetParametersResult(successful=False, reason='dt must be in [0.01, 0.5] seconds')
            if p.name == 'effect_preset' and p.value not in ('off', 'low', 'medium', 'high'):
                return SetParametersResult(successful=False, reason='Unknown effects preset')
            if p.name == 'jet_radius' and (not np.isfinite(p.value) or not .001 <= p.value <= .03):
                return SetParametersResult(successful=False, reason='jet_radius must be in [.001,.03] m')
            if p.name in self.EFFECT_SETTINGS:
                lower, upper = (0., 1.) if p.name.endswith('_scale') else (.02, .5)
                if not np.isfinite(p.value) or not lower <= p.value <= upper:
                    return SetParametersResult(successful=False, reason=f'{p.name} must be in [{lower},{upper}]')
            if p.name == 'effect_reference_pressure_mpa' and (not np.isfinite(p.value) or not 1 <= p.value <= 60):
                return SetParametersResult(successful=False, reason='effect_reference_pressure_mpa must be in [1,60]')
            mutable = ('dt', 'effect_preset', 'jet_core', 'jet_mist', 'jet_splash', 'jet_dust', 'jet_radius',
                       'effect_reference_pressure_mpa', *self.EFFECT_SETTINGS)
            if self.operation_lock_.locked() and p.name in mutable:
                return SetParametersResult(successful=False, reason='Cannot change shot parameters while busy')
            if self.simulation_ is not None and p.name not in (*mutable, 'use_sim_time'):
                return SetParametersResult(successful=False, reason='Geometry/configuration requires a node restart')
        return SetParametersResult(successful=True)

    def on_configure(self, state):
        """Load the coal once, bound storage, and prepare the world transformation."""
        try:
            validation = self.validate_parameters([self.get_parameter(name) for name in
                ('dt', 'effect_preset', 'jet_radius', 'effect_reference_pressure_mpa', *self.EFFECT_SETTINGS)])
            if not validation.successful:
                raise ValueError(validation.reason)
            self.output_dir_ = Path(self.get_parameter('output_mesh_dir').value)
            self.output_dir_.mkdir(parents=True, exist_ok=True)
            budget = self.get_parameter('solve_budget_s').value
            if not np.isfinite(budget) or budget <= 0:
                raise ValueError('solve_budget_s must be finite and positive')
            self.simulation_ = SpraySimulation(self.get_parameter('coal_wall_mesh_path').value,
                tuple(self.get_parameter('voxel_resolution').value), self.get_parameter('dt').value,
                wall_budget_s=budget)
            pose = self.get_parameter('coal_pose').value
            self.rotation_ = Rotation.from_euler('xyz', pose[3:]).as_matrix()
            self.translation_ = np.array(pose[:3])
            self.gazebo_ = GazeboCoalWall(self.get_parameter('world_name').value, pose,
                scene_probe=Path(get_package_prefix('hydromining_robot'))/'lib/hydromining_robot/coal_wall_scene',
                warning=self.get_logger().warning) if self.get_parameter('update_gazebo').value else None
            self.get_logger().info(f'Configured grid: {self.simulation_.grid.resolution}, h={self.simulation_.grid.spacing:.4f} m')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Configure failed: {e}')
            return TransitionCallbackReturn.FAILURE

    def export_and_update(self, phi):
        """Use immutable filenames; geometry and numerical state must agree."""
        self.sequence_ += 1
        path = self.output_dir_/f'coal_wall_{self.sequence_:04d}.obj'
        if not MeshConverter.save_mesh(*self.simulation_.mesh(phi), str(path)):
            raise RuntimeError('Cannot save coal mesh')
        if self.gazebo_ is not None:
            self.gazebo_.replace(path)
        return str(path)

    def on_activate(self, state):
        """Display the initial numerical surface before any sensor baseline capture."""
        try:
            if self.operation_lock_.locked():
                raise RuntimeError('Previous operation is still stopping')
            self.export_and_update(self.simulation_.grid.phi)
            self.cancel_.clear()
            self.active_ = True
            self.set_phase('IDLE')
            self.get_logger().info('Active: /spray_action and /reset_coal_wall ready')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Activate failed: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_deactivate(self, state):
        """Reject work while inactive without changing the retained coal."""
        self.cancel_.set()
        self.set_phase('RESETTING')
        self.active_ = False
        return TransitionCallbackReturn.SUCCESS

    def on_diagnostics(self, msg):
        """Receive actual server heartbeat; missing effects must fail visibly."""
        try:
            values = [float(x) for x in msg.data.split()]
            if len(values) == 7 and np.isfinite(values).all():
                self.diagnostics_ = values
        except ValueError:
            pass

    def set_phase(self, phase, detail=''):
        """Publish state transitions immediately and heartbeat on a separate callback."""
        with self.effect_lock_:
            self.phase_, self.detail_ = phase, detail
        self.get_logger().info('Spray state: ' + phase)
        self.publish_effects()

    def publish_effects(self):
        """ROS clock owns timing; native Gazebo particles own sensor scattering."""
        if not self.active_ or not rclpy.ok(context=self.context):
            return
        with self.effect_lock_:
            status = SprayStatus()
            status.header.stamp = self.get_clock().now().to_msg()
            status.header.frame_id = 'world'
            status.shot_id, status.state = self.shot_id_, self.phase_
            status.pressure, status.duration = self.pressure_, self.duration_
            status.elapsed = max(0., self.get_clock().now().nanoseconds * 1e-9 - self.started_)
            status.effect_strength = self.strength_
            status.has_hit = self.geometry_['has_hit']
            status.message = self.detail_
            for target, key in ((status.nozzle_position, 'source'), (status.direction, 'direction'),
                                (status.hit_position, 'hit'), (status.hit_normal, 'normal')):
                target.x, target.y, target.z = map(float, self.geometry_[key])
            self.status_pub_.publish(status)
            if self.get_parameter('enable_effects').value:
                mode = 1 if self.phase_ == 'SPRAYING' and not self.cancel_.is_set() else (
                    2 if self.phase_ in ('UPDATING', 'SETTLING') else 0)
                flags = [self.get_parameter('jet_' + name).value for name in ('core','mist','splash','dust')]
                packet = command_packet(self.command_id_, mode, self.geometry_, self.strength_,
                                        self.eroding_, flags, self.get_parameter('jet_radius').value,
                                        [self.get_parameter(name).value for name in self.EFFECT_SETTINGS])
                self.command_pub_.publish(String(data=packet))

    def timed_wait(self, seconds):
        """Wait in simulation time, remain cancellable during a paused simulation."""
        start = self.get_clock().now().nanoseconds * 1e-9
        while self.get_clock().now().nanoseconds * 1e-9 - start < seconds:
            if self.cancel_.wait(.02) or not rclpy.ok(context=self.context):
                raise RuntimeError('Spray cancelled; candidate field discarded')

    def reset_callback(self, request, response):
        """Cancel in-flight work, clear effects immediately, then restore the coal."""
        if not self.active_:
            response.message = 'Node is not active'
            return response
        self.cancel_.set()
        self.set_phase('RESETTING')
        with self.operation_lock_:
            try:
                self.command_id_ += 1
                self.set_phase('RESETTING')
                path = self.export_and_update(self.simulation_.initial_phi)
                self.simulation_.reset()
                response.success, response.message = True, path
                self.set_phase('IDLE')
            except Exception as error:
                response.success, response.message = False, str(error)
                self.set_phase('ERROR', str(error))
            finally:
                self.cancel_.clear()
        return response

    def spray_callback(self, request, response):
        """Validate, compute, update Gazebo and only then commit the remaining coal."""
        if not self.operation_lock_.acquire(blocking=False):
            response.message = 'Busy: an operation is already in progress'
            return response
        try:
            if not self.active_:
                raise ValueError('Node is not active')
            if self.cancel_.is_set():
                raise ValueError('Reset or deactivation is in progress')
            if self.gazebo_ is not None and not self.gazebo_.synchronized_:
                raise RuntimeError('Coal geometry is unconfirmed; reset the coal wall before spraying')
            if request.use_tcp:
                tf = self.tf_buffer_.lookup_transform('world', self.get_parameter('tcp_frame').value,
                                                      rclpy.time.Time())
                t, q = tf.transform.translation, tf.transform.rotation
                position = self.rotation_.T@(np.array([t.x, t.y, t.z])-self.translation_)
                direction = self.rotation_.T@Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()[:, 2]
            else:
                position = [request.position_x, request.position_y, request.position_z]
                direction = [request.direction_x, request.direction_y, request.direction_z]
            action = AlgorithmAction(position, direction, request.pressure, request.duration, int(request.enabled))
            self.simulation_.evolver.dt = self.get_parameter('dt').value
            self.shot_id_ += 1
            self.command_id_ += 1
            self.pressure_, self.duration_ = float(action.pressure), float(action.duration)
            self.strength_ = effect_strength(action.pressure, self.get_parameter('effect_preset').value,
                                            self.get_parameter('effect_reference_pressure_mpa').value)
            self.started_ = self.get_clock().now().nanoseconds * 1e-9
            effects = self.get_parameter('enable_effects').value
            self.set_phase('PREPARING')
            if effects:
                if self.diagnostics_ is None:
                    raise RuntimeError('WaterJetEffects plugin/bridge heartbeat missing')
                self.geometry_ = hit_geometry(self.simulation_, action, self.rotation_, self.translation_)
            phi, stats = self.simulation_.simulate(action, self.cancel_.is_set)
            self.eroding_ = stats['volume_removed_m3'] > 0
            if self.cancel_.is_set():
                raise RuntimeError('Spray cancelled before commit')
            if effects and action.g and action.pressure > 0:
                self.started_ = self.get_clock().now().nanoseconds * 1e-9
                self.set_phase('SPRAYING')
                self.timed_wait(action.duration)
                self.set_phase('UPDATING')
            if self.cancel_.is_set():
                raise RuntimeError('Spray cancelled before commit')
            path = self.export_and_update(phi)
            self.simulation_.commit(phi)
            if effects and action.g and action.pressure > 0:
                self.set_phase('SETTLING')
                self.timed_wait(2.7)
            self.set_phase('IDLE')
            stats.update({'position_mesh': action.position.tolist(), 'direction_mesh': action.direction.tolist()})
            (self.output_dir_/f'shot_{self.simulation_.shots:03d}.json').write_text(json.dumps(stats, indent=2))
            response.success = True
            response.message = 'Local erosion complete; kernel parameters are NOT calibrated'
            for key in ('volume_removed_m3', 'cumulative_volume_m3', 'max_depth_m', 'cumulative_depth_m'):
                setattr(response, key, stats[key])
            response.output_mesh_path = path
            response.statistics_json = json.dumps(stats)
            self.get_logger().info(f'Shot {self.simulation_.shots}: V={stats["volume_removed_m3"]:.7f} m3, depth={stats["max_depth_m"]:.4f} m')
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'Spray rejected: {e}')
            if self.phase_ != 'RESETTING':
                self.set_phase('ERROR', str(e))
        finally:
            self.operation_lock_.release()
        return response


def main(args=None):
    # Keep the context alive until in-flight service/timer callbacks have exited.
    # Default ROS signal shutdown can otherwise invalidate their native handles.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = SprayControllerNode()
    stop = threading.Event()

    def request_stop(signum, frame):
        node.active_ = False
        node.cancel_.set()
        stop.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        node.trigger_configure()
        if stop.is_set():
            return
        if node.simulation_ is None:
            raise RuntimeError('Configuration failed')
        node.trigger_activate()
        if not node.active_:
            raise RuntimeError('Activation failed')
        while rclpy.ok(context=node.context) and not stop.is_set():
            executor.spin_once(timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.cancel_.set()
        # This installed Jazzy executor has no public wait_for_threads argument.
        # Join its ThreadPoolExecutor before destroying node/native ROS handles;
        # ThreadPoolExecutor.shutdown is idempotent on newer executors too.
        pool = getattr(executor, '_executor', None)
        if pool is not None:
            pool.shutdown(wait=True)
        executor.shutdown(timeout_sec=10)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
