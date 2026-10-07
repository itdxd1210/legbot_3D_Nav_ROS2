"""Autonomous initial search, standing only; truth is an independent observer."""
import json
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    package = Path(share('legbot_bringup'))
    config_path = Path(LaunchConfiguration('map_config').perform(context)).resolve()
    config = json.loads(config_path.read_text())
    assets = config_path.parent/'initial_localization/occupancy.yaml'
    alignment = config_path.parent/'alignment.json'
    if not assets.is_file() or not alignment.is_file():
        raise ValueError('Prepare initial-localization assets first; alignment is required only for truth validation')
    spawn = config['spawn']
    if len(spawn) != 3:
        raise ValueError('spawn must contain XYZ')
    def include(name, arguments):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(package/'launch'/name)),
                                        launch_arguments=arguments.items())
    # Spawn/world alignment are NEVER parameters of the search node.
    sequencer = Node(package='legbot_bringup', executable='go2_demo_sequencer', output='screen',
        parameters=[{'use_sim_time': True, 'gate_fastlio_start': True}],
        remappings=[('/scan/planning/bspline', '/initial_localization/unused_bspline'),
                    ('/scan/plan_ready', '/initial_localization/unused_plan_ready'),
                    ('/scan/mission_complete', '/initial_localization/unused_complete')])
    return [
        include('simulation.launch.py', {'gui': LaunchConfiguration('gui'),
            'navigation_source': 'fastlio', 'publish_ground_truth': 'true',
            'x': str(spawn[0]), 'y': str(spawn[1]), 'z': str(spawn[2]),
            'yaw': str(config.get('spawn_yaw', 0.)), 'lidar_pitch': str(.7853981633974483)}),
        include('fastlio.launch.py', {'use_sim_time': 'true', 'wait_for_start': 'true',
            'gravity_alignment': 'true', 'config': str(package/'config/fastlio_sim.yaml')}),
        TimerAction(period=8., actions=[sequencer]),
        Node(package='legbot_bringup', executable='initial_localization_probe', output='screen',
             parameters=[{'use_sim_time': True, 'occupancy_yaml': str(assets)}]),
        Node(package='legbot_bringup', executable='initial_localization_truth', output='screen',
             parameters=[{'use_sim_time': True, 'alignment_file': str(alignment)}]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', str(package/'rviz/initial_localization.rviz')],
             parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    data = os.environ.get('LEGBOT_DATA_DIR', os.path.expanduser('~/legbot_data'))
    return LaunchDescription([
        DeclareLaunchArgument('map_config', default_value=os.path.join(data, 'full_tilt45_20260928/navigation.json')),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=setup),
    ])
