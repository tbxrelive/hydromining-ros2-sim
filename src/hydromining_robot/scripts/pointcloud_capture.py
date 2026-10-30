#!/usr/bin/env python3
"""
点云采集节点
从激光雷达采集点云，转换到世界坐标系并保存
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
import tf2_sensor_msgs
import os
import numpy as np


class PointCloudCapture(Node):
    """
    点云采集节点

    Subscriptions:
        /hydromining/lidar/points: 激光雷达点云

    Services:
        /capture_pointcloud: 触发采集并保存点云

    Parameters:
        output_dir: 点云保存目录
        sensor_frame: 传感器坐标系
        world_frame: 世界坐标系
    """

    def __init__(self):
        super().__init__('pointcloud_capture')

        # 声明参数
        self.declare_parameter('output_dir', os.path.expanduser('~/pointcloud_data'))
        self.declare_parameter('sensor_frame', 'lidar_link')
        self.declare_parameter('world_frame', 'world')

        # QoS 配置（传感器话题使用 BEST_EFFORT）
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        # 订阅点云
        self.subscription_ = self.create_subscription(
            PointCloud2,
            '/hydromining/lidar/points',
            self.pointcloud_callback,
            qos_profile
        )

        # TF 监听器
        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)

        # 服务：触发采集
        self.capture_service_ = self.create_service(
            Trigger,
            '/capture_pointcloud',
            self.capture_callback
        )

        # 最新点云缓存
        self.latest_pointcloud_ = None

        self.get_logger().info('点云采集节点已启动')

    def pointcloud_callback(self, msg: PointCloud2):
        """缓存最新点云"""
        self.latest_pointcloud_ = msg

    def capture_callback(self, request, response):
        """服务回调：保存当前点云"""
        if self.latest_pointcloud_ is None:
            response.success = False
            response.message = "无可用点云数据"
            self.get_logger().warn(response.message)
            return response

        try:
            # 获取参数
            output_dir = self.get_parameter('output_dir').value

            # 保存为 PCD 文件（暂时直接保存，不做 TF 变换）
            # TODO: 未来可以添加 TF 变换到 world 坐标系
            os.makedirs(output_dir, exist_ok=True)
            timestamp = self.get_clock().now().nanoseconds
            filename = f"{output_dir}/pointcloud_{timestamp}.pcd"

            self.save_as_pcd(self.latest_pointcloud_, filename)

            response.success = True
            response.message = f"点云已保存: {filename}"
            self.get_logger().info(response.message)

        except Exception as e:
            response.success = False
            response.message = f"采集失败: {str(e)}"
            self.get_logger().error(response.message)

        return response

    def save_as_pcd(self, cloud_msg, filename):
        """保存点云为 PCD 格式（不依赖 Open3D）"""
        import struct

        # 解析点云数据
        points = []
        point_step = cloud_msg.point_step
        row_step = cloud_msg.row_step
        data = cloud_msg.data

        # 查找 x, y, z 字段的偏移量
        x_offset = None
        y_offset = None
        z_offset = None

        for field in cloud_msg.fields:
            if field.name == 'x':
                x_offset = field.offset
            elif field.name == 'y':
                y_offset = field.offset
            elif field.name == 'z':
                z_offset = field.offset

        if x_offset is None or y_offset is None or z_offset is None:
            raise ValueError(f"点云缺少 x/y/z 字段，实际字段: {[f.name for f in cloud_msg.fields]}")

        # 提取点云坐标
        for i in range(0, len(data), point_step):
            # 从二进制数据中解析 float32
            x = struct.unpack_from('f', data, i + x_offset)[0]
            y = struct.unpack_from('f', data, i + y_offset)[0]
            z = struct.unpack_from('f', data, i + z_offset)[0]

            # 跳过 NaN 和 Inf
            if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
                points.append([x, y, z])

        if len(points) == 0:
            raise ValueError("点云为空（所有点都是 NaN）")

        points_array = np.array(points, dtype=np.float32)

        # 写入 PCD 文件（ASCII 格式）
        with open(filename, 'w') as f:
            f.write("# .PCD v0.7 - Point Cloud Data file format\n")
            f.write("VERSION 0.7\n")
            f.write("FIELDS x y z\n")
            f.write("SIZE 4 4 4\n")
            f.write("TYPE F F F\n")
            f.write("COUNT 1 1 1\n")
            f.write(f"WIDTH {len(points_array)}\n")
            f.write("HEIGHT 1\n")
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write(f"POINTS {len(points_array)}\n")
            f.write("DATA ascii\n")

            for point in points_array:
                f.write(f"{point[0]} {point[1]} {point[2]}\n")

        self.get_logger().info(f"保存了 {len(points_array)} 个点到 {filename}")


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudCapture()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
