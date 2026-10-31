#!/usr/bin/env python3
import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 包路径
    pkg_hydromining_robot = get_package_share_directory('hydromining_robot')
    pkg_hydromining_sim = get_package_share_directory('hydromining_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # 设置 Gazebo 资源路径（煤壁模型）
    models_path = os.path.join(pkg_hydromining_sim, 'models')
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += os.pathsep + models_path
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = models_path

    plugin_path = os.path.join(get_package_share_directory('hydromining_sim'), '..', '..', 'lib')
    os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH'] = plugin_path + os.pathsep + os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')

    # 参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')

    # 文件路径
    urdf_file = os.path.join(pkg_hydromining_robot, 'urdf', 'panda_with_water_gun.urdf.xacro')
    default_world_file = os.path.join(pkg_hydromining_sim, 'worlds', 'coal_wall_v02.sdf')
    world_file = LaunchConfiguration('world')
    rviz_config = os.path.join(pkg_hydromining_robot, 'config', 'robot_view.rviz')

    # 处理 xacro 生成 URDF
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        urdf_file,
        ' ',
        'name:=panda',
        ' ',
        'arm_id:=panda',
        ' camera_rate:=', LaunchConfiguration('camera_rate'),
        ' camera_width:=', LaunchConfiguration('camera_width'),
        ' camera_height:=', LaunchConfiguration('camera_height'),
        ' lidar_rate:=', LaunchConfiguration('lidar_rate'),
        ' depth_noise_stddev:=', LaunchConfiguration('depth_noise_stddev'),
        ' lidar_noise_stddev:=', LaunchConfiguration('lidar_noise_stddev')
    ])

    robot_description = {'robot_description': robot_description_content}

    # 声明参数
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Gazebo GUI if true'
    )

    # 启动 Gazebo
    # 根据 gui 参数选择 Gazebo 运行模式
    # gui=true: -r (运行并显示GUI)
    # gui=false: -r -s (运行但无GUI，仅服务器模式)
    from launch.substitutions import PythonExpression

    gz_args_base = ['-r ', world_file]
    gz_args_with_gui = PythonExpression([
        "'-r '", " + '", world_file, "' if '", gui,
        "' == 'true' else '-r -s --headless-rendering ' + '", world_file, "'"
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': gz_args_with_gui,
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # 生成机器人到 Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'panda_water_gun',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0'
        ],
        output='screen'
    )

    # 加载控制器：Joint State Broadcaster
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager',
                   '--switch-timeout', '30', '--service-call-timeout', '40'],
        output='screen'
    )

    # 加载控制器：Panda Arm Controller
    load_panda_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['panda_arm_controller', '--controller-manager', '/controller_manager',
                   '--switch-timeout', '30', '--service-call-timeout', '40'],
        output='screen'
    )

    # 在机器人生成后启动控制器
    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[load_joint_state_broadcaster],
        )
    )

    delay_panda_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_panda_arm_controller],
        )
    )

    # ROS-Gazebo 桥接（相机和激光雷达）
    # 注意：rgbd_camera 在 Gazebo 中实际发布的话题是:
    #   - /hydromining/camera/image (RGB)
    #   - /hydromining/camera/depth_image (Depth)
    #   - /hydromining/camera/camera_info
    #   - /hydromining/camera/points (PointCloud)
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/hydromining/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/hydromining/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/hydromining/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/hydromining/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/hydromining/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        remappings=[
            ('/hydromining/camera/image', '/hydromining/camera/color/image_raw'),
            ('/hydromining/camera/depth_image', '/hydromining/camera/depth/image_rect_raw'),
            ('/hydromining/camera/camera_info', '/hydromining/camera/color/camera_info'),
            ('/hydromining/camera/points', '/hydromining/camera/depth/points'),
        ],
        output='screen'
    )

    # RViz2（可选）
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(
            LaunchConfiguration('start_rviz', default='false')
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world_file),
        DeclareLaunchArgument('camera_rate', default_value='30'),
        DeclareLaunchArgument('camera_width', default_value='640'),
        DeclareLaunchArgument('camera_height', default_value='480'),
        DeclareLaunchArgument('lidar_rate', default_value='10'),
        DeclareLaunchArgument('depth_noise_stddev', default_value='0'),
        DeclareLaunchArgument('lidar_noise_stddev', default_value='0'),
        DeclareLaunchArgument('start_spray', default_value='false'),
        DeclareLaunchArgument('enable_effects', default_value='false'),
        DeclareLaunchArgument('effect_preset', default_value='medium'),
        DeclareLaunchArgument('effects_config', default_value=os.path.join(pkg_hydromining_robot, 'config', 'jet_effects.yaml')),
        DeclareLaunchArgument('spray_config', default_value=os.path.join(pkg_hydromining_robot, 'config', 'spray.yaml')),
        DeclareLaunchArgument('spray_output_dir', default_value='/tmp/hydromining_meshes'),
        declare_use_sim_time,
        declare_gui,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        delay_joint_state_broadcaster,
        delay_panda_arm_controller,
        ros_gz_bridge,
        TimerAction(period=8.0, actions=[Node(package='hydromining_robot', executable='spray_controller_node.py',
             prefix=[os.environ.get('HYDROMINING_PYTHON', sys.executable)],
             parameters=[LaunchConfiguration('spray_config'), LaunchConfiguration('effects_config'), {'use_sim_time': use_sim_time,
                         'output_mesh_dir': LaunchConfiguration('spray_output_dir'),
                         'enable_effects': LaunchConfiguration('enable_effects'),
                         'effect_preset': LaunchConfiguration('effect_preset')}],
             condition=IfCondition(LaunchConfiguration('start_spray')), output='screen')]),
        Node(package='hydromining_robot', executable='jet_bridge', name='jet_effects_bridge',
             condition=IfCondition(LaunchConfiguration('enable_effects')), output='screen'),
        # rviz2,  # 取消注释以启动 RViz
    ])
