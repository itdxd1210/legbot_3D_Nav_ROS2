"""View a recorded mapping session without simulation or localization."""
import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_directory', description='Directory containing manifest.json and PCD chunks'),
        DeclareLaunchArgument('voxel_size', default_value='0.10'),
        DeclareLaunchArgument('max_points', default_value='500000'),
        Node(package='legbot_bringup', executable='mapping_map_viewer', output='screen', parameters=[{
            'map_directory': LaunchConfiguration('map_directory'),
            'voxel_size': ParameterValue(LaunchConfiguration('voxel_size'), value_type=float),
            'max_points': ParameterValue(LaunchConfiguration('max_points'), value_type=int),
            'use_sim_time': False}]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', os.path.join(
            share('legbot_bringup'), 'rviz', 'manual_mapping.rviz')], parameters=[{'use_sim_time': False}]),
    ])
