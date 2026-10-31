"""ROS 观测适配：运动、传感器缓存、准确时间戳 TF 与有界等待。"""
import threading
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

from hydromining_robot.msg import SprayStatus


class ObservationAdapter:
    """只存最新消息，按任务要求采集；运动和模型更新期间的帧不得进入评估。"""

    def __init__(self, node, group, timing):
        self.node_, self.timing_ = node, timing
        self.condition_ = threading.Condition()
        self.messages_, self.sequence_ = {}, {'depth': 0, 'lidar': 0}
        self.subscriptions_ = []
        self.motion_handle_ = None
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        state_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        topics = (
            (Image, '/hydromining/camera/depth/image_rect_raw', 'depth', sensor_qos),
            (CameraInfo, '/hydromining/camera/color/camera_info', 'info', sensor_qos),
            (PointCloud2, '/hydromining/lidar/points', 'lidar', sensor_qos),
            (JointState, '/joint_states', 'joints', state_qos),
            (SprayStatus, '/spray/status', 'spray', state_qos),
        )
        for message_type, topic, key, qos in topics:
            self.subscriptions_.append(node.create_subscription(
                message_type, topic, lambda message, name=key: self.receive(name, message),
                qos, callback_group=group))
        self.tf_ = Buffer()
        self.listener_ = TransformListener(self.tf_, node)
        self.motion_ = ActionClient(
            node, FollowJointTrajectory, '/panda_arm_controller/follow_joint_trajectory',
            callback_group=group)

    def receive(self, name, message):
        """缓存有界最新数据并唤醒等待任务。"""
        with self.condition_:
            self.messages_[name] = message
            if name in self.sequence_:
                self.sequence_[name] += 1
            self.condition_.notify_all()

    def wait(self, predicate, timeout, cancelled, description):
        """用墙钟保证暂停仿真时也能取消或超时，不在执行线程 spin 节点。"""
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if cancelled():
                self.cancel_motion()
                raise InterruptedError('observation cancelled')
            if predicate():
                return
            with self.condition_:
                self.condition_.wait(timeout=0.05)
        self.cancel_motion()
        raise TimeoutError(description)

    def result(self, future, timeout, cancelled, description):
        """等待由 MultiThreadedExecutor 其他线程处理的 ROS future。"""
        self.wait(future.done, timeout, cancelled, description)
        return future.result()

    def current_joints(self):
        """按名字读取关节，禁止依赖 JointState 下标顺序。"""
        with self.condition_:
            message = self.messages_.get('joints')
        if message is None:
            return None
        values = dict(zip(message.name, message.position))
        names = ['panda_joint' + str(i) for i in range(1, 8)]
        return np.array([values[name] for name in names]) if all(n in values for n in names) else None

    def idle_token(self):
        """IDLE 只在阶段五模型事务确认完成后允许；喷射期间不得评估形变。"""
        with self.condition_:
            status = self.messages_.get('spray')
        if status is None or status.state != 'IDLE':
            return None
        age = (self.node_.get_clock().now().nanoseconds -
               status.header.stamp.sec * 1_000_000_000 - status.header.stamp.nanosec) / 1e9
        if not -0.05 <= age <= self.timing_['status_timeout_s']:
            return None
        return status.shot_id

    def cancel_motion(self):
        """仅取消本适配器提交的运动，不影响其他客户端。"""
        handle = self.motion_handle_
        if handle is not None and handle.accepted:
            handle.cancel_goal_async()
        self.motion_handle_ = None

    def move(self, joints, cancelled):
        """执行预定义关节位置并验证到位、静止；本接口不声称实机碰撞安全。"""
        joints = np.asarray(joints, float)
        self.wait(self.motion_.server_is_ready, self.timing_['motion_timeout_s'],
                  cancelled, 'trajectory action unavailable')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['panda_joint' + str(i) for i in range(1, 8)]
        seconds = self.timing_['move_duration_s']
        goal.trajectory.points = [JointTrajectoryPoint(
            positions=joints.tolist(),
            time_from_start=Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9)))]
        self.motion_handle_ = self.result(
            self.motion_.send_goal_async(goal), self.timing_['motion_timeout_s'],
            cancelled, 'trajectory goal response timeout')
        if not self.motion_handle_.accepted:
            raise RuntimeError('trajectory goal rejected')
        result = self.result(
            self.motion_handle_.get_result_async(), self.timing_['motion_timeout_s'],
            cancelled, 'trajectory execution timeout')
        self.motion_handle_ = None
        if result.result.error_code != 0:
            raise RuntimeError(result.result.error_string)
        self.wait(lambda: self.current_joints() is not None and
                  np.max(np.abs(self.current_joints() - joints)) < 0.02,
                  self.timing_['sensor_timeout_s'], cancelled, 'arm failed to reach viewpoint')
        start = self.node_.get_clock().now().nanoseconds
        self.wait(lambda: (self.node_.get_clock().now().nanoseconds - start) / 1e9 >=
                  self.timing_['settle_s'], self.timing_['sensor_timeout_s'],
                  cancelled, 'simulation stopped during settling')

    def pose_at(self, message, cancelled):
        """只用测量时间戳查 TF，不回退到最新变换。"""
        stamp = rclpy.time.Time.from_msg(message.header.stamp)
        frame = message.header.frame_id
        self.wait(lambda: self.tf_.can_transform('world', frame, stamp),
                  self.timing_['sensor_timeout_s'], cancelled, 'TF missing at sensor timestamp')
        transform = self.tf_.lookup_transform('world', frame, stamp).transform
        t, q = transform.translation, transform.rotation
        return np.array([t.x, t.y, t.z, q.x, q.y, q.z, q.w])

    def snapshot(self, token, cancelled):
        """分别等待新深度与 LiDAR，保留各自时间戳、TF 和内参。"""
        with self.condition_:
            before = self.sequence_.copy()
        self.wait(lambda: self.sequence_['depth'] > before['depth'] and
                  self.sequence_['lidar'] > before['lidar'] and 'info' in self.messages_,
                  self.timing_['sensor_timeout_s'], cancelled, 'fresh sensor frames timeout')
        if self.idle_token() != token:
            raise RuntimeError('wall state changed during assessment; acquire a new observation')
        with self.condition_:
            depth, lidar, info = (self.messages_[key] for key in ('depth', 'lidar', 'info'))
        if depth.encoding != '32FC1' or depth.width * depth.height > 320 * 240:
            raise ValueError('depth encoding or resolution exceeds perception budget')
        if info.width != depth.width or info.height != depth.height:
            raise ValueError('camera intrinsics do not match depth dimensions')
        dtype = '>f4' if depth.is_bigendian else '<f4'
        image = np.ndarray((depth.height, depth.width), dtype=dtype,
                           buffer=depth.data, strides=(depth.step, 4)).copy()
        cloud = read_points_numpy(
            lidar, field_names=['x', 'y', 'z'], skip_nans=False).reshape(-1, 3).copy()
        if len(cloud) > 16 * 1024:
            raise ValueError('lidar point count exceeds perception budget')
        return {
            'depth': image, 'lidar_local': cloud, 'K': np.array(info.k).reshape(3, 3),
            'depth_stamp': np.array([depth.header.stamp.sec, depth.header.stamp.nanosec]),
            'lidar_stamp': np.array([lidar.header.stamp.sec, lidar.header.stamp.nanosec]),
            'depth_frame': depth.header.frame_id, 'lidar_frame': lidar.header.frame_id,
            'tf_depth': self.pose_at(depth, cancelled), 'tf_lidar': self.pose_at(lidar, cancelled),
            'joints': self.current_joints(),
        }

    def collect(self, viewpoints, frames_per_view, cancelled, progress):
        """一次任务固定煤壁状态；完成所有视点前发生喷射则拒绝混合表面。"""
        self.wait(lambda: self.idle_token() is not None, self.timing_['sensor_timeout_s'],
                  cancelled, 'confirmed idle wall state unavailable')
        token = self.idle_token()
        groups = []
        for index, view in enumerate(viewpoints):
            self.move(view.joints, cancelled)
            snapshots = [self.snapshot(token, cancelled) for _ in range(frames_per_view)]
            groups.append(snapshots)
            progress(index + 1, len(viewpoints))
        if self.idle_token() != token:
            raise RuntimeError('wall changed before observation completed')
        return groups

    def destroy(self):
        """释放 configure 阶段创建的所有实体。"""
        self.cancel_motion()
        self.motion_.destroy()
        self.listener_.unregister()
        for subscription in self.subscriptions_:
            self.node_.destroy_subscription(subscription)
        self.subscriptions_.clear()
