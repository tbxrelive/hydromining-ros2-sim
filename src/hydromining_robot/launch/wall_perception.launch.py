"""阶段六感知节点；生命周期由 launch 事件外部管理。"""
import os
import sys

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def configure_launch(context):
    """读取统一算法配置，将无效配置暴露为启动错误。"""
    path = LaunchConfiguration('config').perform(context)
    with open(path, encoding='utf-8') as stream:
        parameters = yaml.safe_load(stream)
    parameters.update({
        'use_sim_time': LaunchConfiguration('use_sim_time').perform(context) == 'true',
        'output_directory': LaunchConfiguration('output_directory').perform(context),
    })
    node = LifecycleNode(
        package='hydromining_robot', executable='wall_perception_node.py',
        name='wall_perception_node', namespace='', output='screen',
        prefix=[os.environ.get('HYDROMINING_PYTHON', sys.executable)],
        parameters=[parameters])
    configure = RegisterEventHandler(OnProcessStart(
        target_action=node, on_start=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=Transition.TRANSITION_CONFIGURE))]))
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=node, start_state='configuring', goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=Transition.TRANSITION_ACTIVATE))]))
    return [configure, activate, node]


def generate_launch_description():
    """可与现有仿真组合，不隐式启动第二个 Gazebo 实例。"""
    robot = get_package_share_directory('hydromining_robot')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=os.path.join(robot, 'config', 'perception.yaml')),
        DeclareLaunchArgument('output_directory', default_value='/tmp/hydromining_perception'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=configure_launch),
    ])
