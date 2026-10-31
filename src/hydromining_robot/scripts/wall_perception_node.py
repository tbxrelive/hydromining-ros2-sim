#!/usr/bin/env python3
"""阶段六标准生命周期适配器：有反馈、可取消的煤壁区域评估。"""
from dataclasses import asdict, replace
import json
from pathlib import Path
import sys
import threading
import uuid

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation

from hydromining_robot.action import AssessRegion
from hydromining_robot.msg import CoalWallState
from perception_observation import ObservationAdapter

sys.path.insert(0, get_package_share_directory('hydromining_sim'))
from algorithms.perception.config import PerceptionConfig
from algorithms.perception.pipeline import PerceptionSession


class WallPerceptionNode(LifecycleNode):
    """传感器适配与算法分离；仅外部生命周期事件可激活节点。"""

    def __init__(self):
        super().__init__('wall_perception_node')
        self.lock_ = threading.RLock()
        self.active_, self.busy_, self.stopping_ = False, False, threading.Event()
        self.observation_ = self.server_ = self.state_pub_ = self.session_ = None
        self.group_ = ReentrantCallbackGroup()
        for name, value in asdict(PerceptionConfig()).items():
            self.declare_parameter(name, value)
        self.declare_parameter('output_directory', '/tmp/hydromining_perception')
        self.timing_names_ = {
            'move_duration_s': 5., 'motion_timeout_s': 90., 'sensor_timeout_s': 30.,
            'settle_s': 1.5, 'status_timeout_s': 2.,
        }
        for name, value in self.timing_names_.items():
            self.declare_parameter(name, value)
        self.add_on_set_parameters_callback(self.validate_parameters)
        self.context.on_shutdown(self.stopping_.set)
        self.get_logger().info('Wall perception initialized, unconfigured; uncertainty is uncalibrated')

    def configuration(self, overrides=None):
        """从统一算法参数生成配置，运行中不静默修改已有基线。"""
        values = {key: self.get_parameter(key).value for key in asdict(PerceptionConfig())}
        values.update(overrides or {})
        return PerceptionConfig.from_dict(values)

    def validate_parameters(self, parameters):
        """支持未配置时改全部参数；空闲时只允许运行判决/可靠性阈值更新。"""
        mutable = {'maximum_sigma', 'required_depth', 'required_fraction', 'confidence_k'}
        with self.lock_:
            try:
                values = {}
                for parameter in parameters:
                    if parameter.name == 'use_sim_time':
                        continue
                    if self.busy_ or (self.observation_ is not None and parameter.name not in mutable):
                        raise ValueError('parameter change requires idle/unconfigured node')
                    if parameter.name in self.timing_names_:
                        if not np.isfinite(parameter.value) or parameter.value <= 0:
                            raise ValueError('timing parameters must be finite and positive')
                    elif parameter.name in asdict(PerceptionConfig()):
                        values[parameter.name] = parameter.value
                    elif parameter.name != 'output_directory':
                        raise ValueError('unknown perception parameter')
                self.configuration(values)
            except (ValueError, TypeError) as error:
                return SetParametersResult(successful=False, reason=str(error))
        return SetParametersResult(successful=True)

    def on_configure(self, state):
        """配置资源但不接收作业；失败时释放已创建实体。"""
        try:
            self.configuration()
            timing = {name: self.get_parameter(name).value for name in self.timing_names_}
            if not all(np.isfinite(value) and value > 0 for value in timing.values()):
                raise ValueError('invalid observation timing')
            self.output_ = Path(self.get_parameter('output_directory').value).resolve()
            self.output_.mkdir(parents=True, exist_ok=True)
            self.observation_ = ObservationAdapter(self, self.group_, timing)
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.state_pub_ = self.create_lifecycle_publisher(CoalWallState, '/coal_wall/state', qos)
            self.server_ = ActionServer(
                self, AssessRegion, '/coal_wall/assess_region', self.execute,
                callback_group=self.group_, goal_callback=self.accept_goal,
                cancel_callback=lambda _: CancelResponse.ACCEPT)
            self.stopping_.clear()
            self.get_logger().info('Configured perception resources; awaiting external activation')
            return TransitionCallbackReturn.SUCCESS
        except Exception as error:
            self.get_logger().error(f'Configure failed: {error}')
            self.release_resources()
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state):
        """启用生命周期发布者；不在节点内自触发生命周期转换。"""
        result = super().on_activate(state)
        with self.lock_:
            self.active_ = result == TransitionCallbackReturn.SUCCESS
        self.get_logger().info('Perception activated')
        return result

    def on_deactivate(self, state):
        """忙碌时拒绝停用，客户端应先取消当前任务。"""
        with self.lock_:
            if self.busy_:
                self.get_logger().warning('Cancel the active assessment before deactivation')
                return TransitionCallbackReturn.FAILURE
            self.active_ = False
        return super().on_deactivate(state)

    def release_resources(self):
        """只释放当前节点拥有的资源。"""
        if self.server_ is not None:
            self.server_.destroy()
            self.server_ = None
        if self.observation_ is not None:
            self.observation_.destroy()
            self.observation_ = None
        if self.state_pub_ is not None:
            self.destroy_lifecycle_publisher(self.state_pub_)
            self.state_pub_ = None
        self.session_ = None

    def on_cleanup(self, state):
        """标准 cleanup 后清除基线，下次 configure 必须重新建立。"""
        with self.lock_:
            if self.busy_:
                return TransitionCallbackReturn.FAILURE
            self.active_ = False
            self.release_resources()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        """请求终止任务，主执行器退出前由任务处理取消。"""
        self.stopping_.set()
        with self.lock_:
            self.active_ = False
        return TransitionCallbackReturn.SUCCESS

    def goal_configuration(self, request):
        """Goal 明确给出任务标准，禁止评估时偷换原始任务区域。"""
        config = replace(self.configuration(), diameter=request.diameter,
                         required_depth=request.required_depth,
                         required_fraction=request.required_fraction,
                         confidence_k=request.confidence_k)
        center = np.array([request.center.x, request.center.y, request.center.z])
        if not np.isfinite(center).all():
            raise ValueError('invalid task center')
        if not 1 <= len(request.viewpoints) <= config.maximum_views:
            raise ValueError('invalid viewpoint count')
        if not 1 <= request.frames_per_view <= config.maximum_frames_per_view:
            raise ValueError('invalid frames per viewpoint')
        for view in request.viewpoints:
            if not np.isfinite(view.joints).all():
                raise ValueError('nonfinite viewpoint joints')
        if not request.new_baseline:
            if self.session_ is None:
                raise ValueError('baseline is required')
            if (not np.allclose(center, self.session_.center_, atol=1e-9) or
                    config.diameter != self.session_.config_.diameter):
                raise ValueError('task region differs from fixed baseline')
        if request.accumulate and request.new_baseline:
            raise ValueError('accumulate is meaningless for a new baseline')
        return config, center

    def accept_goal(self, request):
        """原子预留任务，未激活、已有作业或非法输入一律拒绝。"""
        with self.lock_:
            if not self.active_ or self.busy_ or self.stopping_.is_set():
                return GoalResponse.REJECT
            try:
                self.goal_configuration(request)
            except (ValueError, TypeError) as error:
                self.get_logger().warning(f'Assessment rejected: {error}')
                return GoalResponse.REJECT
            self.busy_ = True
        return GoalResponse.ACCEPT

    def publish_map(self, result):
        """发布具名状态层，未知保持 NaN；不把网格真值混入消息。"""
        surface, state, change, report = (
            result['surface'], result['current'], result['change'], result['report'])
        message = CoalWallState()
        message.header.frame_id = 'world'
        message.header.stamp = self.get_clock().now().to_msg()
        origin, axes = result['frame'].origin, result['frame'].axes
        message.wall_frame.position.x, message.wall_frame.position.y, message.wall_frame.position.z = origin.tolist()
        # WallFrame 有意把坐标系数组设为只读；SciPy memoryview 需要可写缓冲。
        q = Rotation.from_matrix(axes.copy()).as_quat().tolist()
        (message.wall_frame.orientation.x, message.wall_frame.orientation.y,
         message.wall_frame.orientation.z, message.wall_frame.orientation.w) = q
        message.resolution = surface.resolution_
        message.height, message.width = surface.shape_
        layers = {
            'height': state['height'], 'sigma': state['sigma'], 'count': state['count'],
            'coverage': state['coverage'], 'normal_x': state['normal'][..., 0],
            'normal_y': state['normal'][..., 1], 'normal_z': state['normal'][..., 2],
            'delta': change['delta'], 'delta_sigma': change['sigma'],
            'significant': change['significant'],
        }
        message.layer_names = list(layers)
        message.data = np.concatenate([v.ravel() for v in layers.values()]).astype(np.float32).tolist()
        assessment = report['assessment']
        message.assessment = assessment['status']
        message.fraction_lower, message.fraction_upper = assessment['fraction_lower'], assessment['fraction_upper']
        message.coverage = assessment['coverage']
        message.uncertainty_calibrated = False
        self.state_pub_.publish(message)

    def execute(self, handle):
        """观测、计算、持久化和结果反馈；失败或取消不提交新基线。"""
        result = AssessRegion.Result(uncertainty_calibrated=False)
        cancelled = lambda: handle.is_cancel_requested or self.stopping_.is_set()
        try:
            with self.lock_:
                config, center = self.goal_configuration(handle.request)
                session = self.session_
            def feedback(stage, completed, total):
                handle.publish_feedback(AssessRegion.Feedback(
                    stage=stage, completed_views=completed, total_views=total))
            groups = self.observation_.collect(
                handle.request.viewpoints, handle.request.frames_per_view, cancelled,
                lambda done, total: feedback('observing', done, total))
            directory = self.output_ / uuid.uuid4().hex
            directory.mkdir()
            for index, snapshots in enumerate(groups):
                for frame_index, snapshot in enumerate(snapshots):
                    np.savez_compressed(directory / f'view{index}_f{frame_index:03d}.npz', **snapshot)
            if handle.request.new_baseline:
                candidate = PerceptionSession(center, config)
                candidate.set_baseline(groups, lambda d, t: feedback('baseline', d, t), cancelled)
                if cancelled():
                    raise InterruptedError('baseline cancelled before commit')
                with self.lock_:
                    self.session_ = candidate
                result.assessment = 'baseline_ready'
                report = {'status': 'baseline_ready', 'normal_world': candidate.frame_.axes[:, 2].tolist(),
                          'config': asdict(config), 'calibrated': False}
            else:
                session.config_ = config
                if handle.request.accumulate:
                    evaluated = session.observe(groups, lambda d, t: feedback('fusion', d, t),
                                                cancelled)
                else:
                    evaluated = session.evaluate(groups, lambda d, t: feedback('fusion', d, t),
                                                 cancelled)
                report = evaluated['report']
                if cancelled():
                    raise InterruptedError('assessment cancelled before commit')
                self.publish_map(evaluated)
                assessment, volume, alarm = report['assessment'], report['volume'], report['external_alarm']
                result.assessment = assessment['status']
                result.fraction_lower, result.fraction_upper = assessment['fraction_lower'], assessment['fraction_upper']
                result.coverage = assessment['coverage']
                result.removed_volume_m3, result.volume_sigma_m3 = volume['removed_volume_m3'], volume['volume_sigma_m3']
                result.removed_mass_kg = volume['removed_mass_kg']
                result.external_alarm, result.external_change_area_m2 = alarm['triggered'], alarm['area_m2']
                state_path = directory / 'state.npz'
                np.savez_compressed(state_path, **{
                    f'{side}_{key}': value for side in ('baseline', 'current', 'change')
                    for key, value in evaluated[side].items()},
                    origin=session.frame_.origin, axes=session.frame_.axes)
                result.state_path = str(state_path)
            report_path = directory / 'report.json'
            report_path.write_text(json.dumps(report, indent=2, allow_nan=False))
            result.report_path, result.success, result.message = str(report_path), True, 'completed'
            handle.succeed()
            self.get_logger().info(f'Assessment completed: {result.assessment}')
        except InterruptedError as error:
            result.message, result.assessment = str(error), 'cancelled'
            if handle.is_cancel_requested:
                handle.canceled()
            else:
                handle.abort()
        except Exception as error:
            result.message, result.assessment = str(error), 'failed'
            self.get_logger().error(f'Assessment failed: {error}')
            handle.abort()
        finally:
            with self.lock_:
                self.busy_ = False
        return result


def main(args=None):
    """多线程执行传感器、TF、控制器和任务；不自动 configure/activate。"""
    rclpy.init(args=args)
    node = WallPerceptionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.stopping_.set()
    finally:
        node.stopping_.set()
        executor.shutdown(timeout_sec=5.)
        node.release_resources()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
