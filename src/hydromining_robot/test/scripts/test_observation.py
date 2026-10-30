#!/usr/bin/env python3
"""
多视角观测测试脚本
执行序列：A → B → A，每个位姿采集点云
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_srvs.srv import Trigger
import time


class ObservationTest(Node):
    """多视角观测测试节点"""

    # 预定义观测位姿（关节角度，单位：弧度）
    VIEWPOINT_A = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
    VIEWPOINT_B = [0.524, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

    def __init__(self):
        super().__init__('observation_test')

        # Action 客户端：控制机械臂
        self.trajectory_client_ = ActionClient(
            self,
            FollowJointTrajectory,
            '/panda_arm_controller/follow_joint_trajectory'
        )

        # Service 客户端：触发点云采集
        self.capture_client_ = self.create_client(
            Trigger,
            '/capture_pointcloud'
        )

        self.get_logger().info('等待 Action Server...')
        self.trajectory_client_.wait_for_server()

        self.get_logger().info('等待点云采集服务...')
        self.capture_client_.wait_for_service()

        self.get_logger().info('✓ 所有服务已就绪')

    def move_to_position(self, joint_positions, duration=5.0):
        """移动到指定关节位置"""
        goal_msg = FollowJointTrajectory.Goal()

        # 关节名称
        goal_msg.trajectory.joint_names = [
            'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
            'panda_joint5', 'panda_joint6', 'panda_joint7'
        ]

        # 目标点
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.velocities = [0.0] * 7
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1.0) * 1e9))
        goal_msg.trajectory.points.append(point)

        self.get_logger().info(f'发送目标位置: {joint_positions[:3]}...')

        # 等待发布器连接
        time.sleep(0.5)

        # 发送并等待完成
        future = self.trajectory_client_.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('目标被拒绝')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        self.get_logger().info('✓ 运动完成')
        return True

    def capture_pointcloud(self, label):
        """触发点云采集"""
        self.get_logger().info(f'正在采集点云: {label}')

        request = Trigger.Request()
        future = self.capture_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response.success:
            self.get_logger().info(f'✓ {response.message}')
        else:
            self.get_logger().error(f'✗ {response.message}')

        return response.success

    def run_observation_sequence(self):
        """执行完整观测序列"""
        self.get_logger().info('=== 开始多视角观测测试 ===')

        # 步骤 1：移动到观测位姿 A
        self.get_logger().info('\n[1/5] 移动到观测位姿 A（正面）')
        if not self.move_to_position(self.VIEWPOINT_A, duration=5.0):
            return False

        time.sleep(1.0)

        # 步骤 2：采集点云 A
        self.get_logger().info('\n[2/5] 采集点云 A')
        if not self.capture_pointcloud('viewpoint_A'):
            return False

        # 步骤 3：移动到观测位姿 B
        self.get_logger().info('\n[3/5] 移动到观测位姿 B（侧面 +30°）')
        if not self.move_to_position(self.VIEWPOINT_B, duration=5.0):
            return False

        time.sleep(1.0)

        # 步骤 4：采集点云 B
        self.get_logger().info('\n[4/5] 采集点云 B')
        if not self.capture_pointcloud('viewpoint_B'):
            return False

        # 步骤 5：返回 A，采集基准观测
        self.get_logger().info('\n[5/5] 返回观测位姿 A（基准观测）')
        if not self.move_to_position(self.VIEWPOINT_A, duration=5.0):
            return False

        time.sleep(1.0)

        self.get_logger().info('\n[5/5] 采集基准观测点云')
        if not self.capture_pointcloud('viewpoint_A_baseline'):
            return False

        self.get_logger().info('\n=== ✓ 观测序列完成 ===')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = ObservationTest()

    try:
        success = node.run_observation_sequence()
        if success:
            node.get_logger().info('\n✓✓✓ 测试成功！')
        else:
            node.get_logger().error('\n✗✗✗ 测试失败')
    except KeyboardInterrupt:
        node.get_logger().info('用户中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
