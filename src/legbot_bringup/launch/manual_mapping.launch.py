"""Manual simulated mapping; no PCT, SCAN, matcher or automatic motion."""
import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package = share('legbot_bringup')
    def include(name, arguments):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(package, 'launch', name)), launch_arguments=arguments.items())
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('gravity_alignment', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('output_root', default_value=os.path.join(
            os.environ.get('LEGBOT_DATA_DIR', os.path.expanduser('~/legbot_data')), 'lio_maps')),
        include('simulation.launch.py', {'gui': LaunchConfiguration('gui'),
            'navigation_source': 'fastlio', 'publish_ground_truth': 'false'}),
        include('fastlio.launch.py', {'use_sim_time': 'true', 'wait_for_start': 'true',
            'gravity_alignment': LaunchConfiguration('gravity_alignment'),
            'config': os.path.join(package, 'config', 'fastlio_sim.yaml')}),
        Node(package='legbot_bringup', executable='mapping_pcd_recorder', output='screen',
             parameters=[{'use_sim_time': True, 'output_root': LaunchConfiguration('output_root')}]),
        Node(package='legbot_bringup', executable='mapping_map_viewer', output='screen',
             parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('rviz'))),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(package, 'rviz', 'manual_mapping.rviz')],
             parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
