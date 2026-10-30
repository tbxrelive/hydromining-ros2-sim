#!/usr/bin/env python3
"""
ROS 2 Launch 文件模板
遵循鱼香ROS代码规范，支持参数传递和条件启动
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """生成 Launch 描述"""
    
    # 声明 Launch 参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='是否使用仿真时间'
    )
    
    enable_feature_arg = DeclareLaunchArgument(
        'enable_feature',
        default_value='true',
        description='是否启用某功能'
    )
    
    # 获取参数值
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_feature = LaunchConfiguration('enable_feature')
    
    # 查找配置文件路径
    config_file = PathJoinSubstitution([
        FindPackageShare('package_name'),
        'config',
        'params.yaml'
    ])
    
    # 启动节点
    main_node = Node(
        package='package_name',
        executable='node_executable',
        name='node_name',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/old_topic', '/new_topic'),
        ]
    )
    
    # 条件启动节点
    optional_node = Node(
        package='package_name',
        executable='optional_executable',
        name='optional_node',
        output='screen',
        condition=IfCondition(enable_feature)
    )
    
    # 包含其他 Launch 文件
    include_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('other_package'),
            'launch',
            'other_launch.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        enable_feature_arg,
        main_node,
        optional_node,
        include_launch,
    ])
