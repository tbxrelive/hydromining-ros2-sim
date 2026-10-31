"""阶段六整链路启动；保持阶段四五默认启动行为不变。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """组合既有仿真和外部生命周期管理的感知节点，GUI 仅显式开启。"""
    robot = get_package_share_directory('hydromining_robot')
    sim = get_package_share_directory('hydromining_sim')
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(robot, 'launch', 'robot_simulation.launch.py')),
        launch_arguments={
            'gui': LaunchConfiguration('gui'), 'world': LaunchConfiguration('world'),
            'start_spray': 'true', 'enable_effects': LaunchConfiguration('enable_effects'),
            'spray_config': LaunchConfiguration('spray_config'),
            'spray_output_dir': LaunchConfiguration('spray_output_dir'),
            'camera_rate': '5', 'camera_width': '320', 'camera_height': '240',
            'lidar_rate': '2',
            'depth_noise_stddev': LaunchConfiguration('depth_noise_stddev'),
            'lidar_noise_stddev': LaunchConfiguration('lidar_noise_stddev'),
        }.items())
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(robot, 'launch', 'wall_perception.launch.py')),
        launch_arguments={'output_directory': LaunchConfiguration('perception_output_dir'),
                          'use_sim_time': 'true'}.items())
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('world', default_value=os.path.join(sim, 'worlds', 'coal_wall_acceptance.sdf')),
        DeclareLaunchArgument('enable_effects', default_value='false'),
        DeclareLaunchArgument('spray_config',
                              default_value=os.path.join(robot, 'config', 'spray_acceptance.yaml')),
        DeclareLaunchArgument('depth_noise_stddev', default_value='0'),
        DeclareLaunchArgument('lidar_noise_stddev', default_value='0'),
        DeclareLaunchArgument('spray_output_dir', default_value='/tmp/hydromining_phase6/meshes'),
        DeclareLaunchArgument('perception_output_dir', default_value='/tmp/hydromining_phase6/perception'),
        simulation, perception,
    ])
