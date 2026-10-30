#!/usr/bin/env python3
"""
简单的机器人运动测试脚本
测试 Panda 机器人能否响应关节命令（使用话题接口）
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time


class PandaMotionTest(Node):
    def __init__(self):
        super().__init__('panda_motion_test')

        # 创建轨迹发布器（话题接口）
        # 关键：使用 BEST_EFFORT QoS 匹配控制器配置
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            '/panda_arm_controller/joint_trajectory',
            qos
        )

        self.get_logger().info('✓ 轨迹发布器已创建')

    def send_trajectory(self, positions_list, duration_list):
        """发送轨迹序列

        Args:
            positions_list: 关节位置列表，每个元素是 7 个关节的目标位置
            duration_list: 每个位置的时间（秒）
        """
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            'panda_joint1', 'panda_joint2', 'panda_joint3',
            'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'
        ]

        # 创建轨迹点
        time_acc = 0.0
        for positions, duration in zip(positions_list, duration_list):
            point = JointTrajectoryPoint()
            point.positions = positions
            # 关键：必须提供 velocities，否则控制器会拒绝执行
            point.velocities = [0.0] * 7
            time_acc += duration
            # 修复：正确转换浮点数时间
            point.time_from_start = Duration(sec=int(time_acc), nanosec=int((time_acc % 1.0) * 1e9))
            trajectory.points.append(point)

        self.get_logger().info(f'发送轨迹 ({len(trajectory.points)} 个点)...')

        # 关键：等待发布器连接到订阅者
        time.sleep(0.5)

        self.trajectory_publisher.publish(trajectory)
        self.get_logger().info('✓ 轨迹已发送')

    def test_sequence(self):
        """测试一系列运动"""
        # 位置 1：初始姿态
        pos1 = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

        # 位置 2：向前伸展
        pos2 = [0.0, -0.3, 0.0, -2.0, 0.0, 1.8, 0.785]

        # 位置 3：向上
        pos3 = [0.0, -1.2, 0.0, -1.8, 0.0, 1.0, 0.0]

        # 位置 4：向左摆动
        pos4 = [-0.8, -0.5, 0.2, -2.0, 0.3, 1.5, 0.5]

        # 位置 5：返回初始
        pos5 = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

        self.get_logger().info('=' * 50)
        self.get_logger().info('开始运动测试序列...')
        self.get_logger().info('=' * 50)

        # 发送完整轨迹
        positions = [pos1, pos2, pos3, pos4, pos5]
        durations = [3.0, 4.0, 4.0, 4.0, 3.0]  # 秒

        self.send_trajectory(positions, durations)

        total_time = sum(durations)
        self.get_logger().info(f'等待运动完成 ({total_time} 秒)...')
        time.sleep(total_time + 1.0)

        self.get_logger().info('=' * 50)
        self.get_logger().info('✓ 测试完成！')
        self.get_logger().info('=' * 50)


def main(args=None):
    rclpy.init(args=args)

    node = PandaMotionTest()

    # 等待 ROS 2 系统初始化
    time.sleep(1.0)

    try:
        node.test_sequence()
    except KeyboardInterrupt:
        node.get_logger().info('测试被中断')
    except Exception as e:
        node.get_logger().error(f'错误: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
