#!/usr/bin/env python3
"""
ROS 2 生命周期节点模板
遵循鱼香ROS代码规范，支持参数配置和状态管理
"""

import rclpy
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class TemplateNode(Node):
    """
    模板节点：描述节点功能
    
    Subscriptions:
        /input_topic (std_msgs/String): 输入描述
    
    Publishers:
        /output_topic (std_msgs/String): 输出描述
    
    Parameters:
        param_name (type): 参数描述
    """
    
    def __init__(self, node_name: str):
        super().__init__(node_name)
        
        # 声明参数（带默认值和描述）
        self.declare_parameter('param_name', 'default_value')
        
        # 成员变量（以 _ 结尾）
        self.param_value_ = None
        self.publisher_ = None
        self.subscription_ = None
        
        self.get_logger().info(f'{node_name} 节点已初始化')
    
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """配置状态：初始化发布者、订阅者、参数"""
        self.get_logger().info('正在配置节点...')
        
        # 读取参数
        self.param_value_ = self.get_parameter('param_name').value
        
        # 配置 QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # 创建发布者
        self.publisher_ = self.create_lifecycle_publisher(
            msg_type=String,
            topic='/output_topic',
            qos_profile=qos_profile
        )
        
        # 创建订阅者
        self.subscription_ = self.create_subscription(
            msg_type=String,
            topic='/input_topic',
            callback=self.topic_callback,
            qos_profile=qos_profile
        )
        
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """激活状态：开始发布数据"""
        self.get_logger().info('节点已激活')
        return super().on_activate(state)
    
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """停用状态：停止发布"""
        self.get_logger().info('节点已停用')
        return super().on_deactivate(state)
    
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """清理状态：销毁资源"""
        self.get_logger().info('正在清理节点...')
        self.destroy_subscription(self.subscription_)
        self.destroy_publisher(self.publisher_)
        return TransitionCallbackReturn.SUCCESS
    
    def topic_callback(self, msg):
        """话题回调函数"""
        self.get_logger().info(f'收到消息: {msg.data}')
        # 处理逻辑...


def main(args=None):
    rclpy.init(args=args)
    node = TemplateNode('template_node')
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
