"""Map matching alongside FAST-LIO, without taking over the navigation TF."""

import os

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    map_path = os.path.join(share('pct_planner'), 'src', 'pcd', 'building2_9.pcd')
    params = os.path.join(share('lidar_localization_ros2'), 'param', 'localization.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    node = LifecycleNode(
        package='lidar_localization_ros2',
        executable='lidar_localization_node',
        name='lidar_localization',
        namespace='',
        output='screen',
        parameters=[params, {
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'map_path': LaunchConfiguration('map_path'),
            'use_pcd_map': True,
            'set_initial_pose': False,
            'global_frame_id': LaunchConfiguration('global_frame_id'),
            'odom_frame_id': 'odom',
            'base_frame_id': 'base',
            # Retain the internal map/odom anchor for odometry prediction;
            # publish_tf independently gates external TF broadcasts.
            'enable_map_odom_tf': True,
            'publish_tf': ParameterValue(LaunchConfiguration('publish_tf'), value_type=bool),
            'use_odom': True,
            'use_odom_tf_prediction': True,
            'scan_max_range': ParameterValue(LaunchConfiguration('scan_max_range'), value_type=float),
            'enable_registration_localizability_diagnostics': ParameterValue(LaunchConfiguration('match_diagnostics'), value_type=bool),
            'enable_directional_constraint_shadow': ParameterValue(
                LaunchConfiguration('direction_shadow'), value_type=bool),
            'directional_constraint_min_eigen_ratio': ParameterValue(
                LaunchConfiguration('direction_min_ratio'), value_type=float),
            'ndt_resolution': 0.5,
            'enable_odom_tf_prediction_correction_guard': True,
            'odom_tf_prediction_correction_guard_translation_m': 0.5,
            'enable_odom_recovery_review': ParameterValue(
                LaunchConfiguration('recovery_review'), value_type=bool),
            'enable_odom_candidate_review': ParameterValue(
                LaunchConfiguration('candidate_review'), value_type=bool),
            # Experimental innovation bound, not an absolute body-pitch limit.
            'odom_tf_prediction_correction_guard_rotation_deg': ParameterValue(
                LaunchConfiguration('rotation_guard_deg'), value_type=float),
            'enable_rejected_seed_update': False,
            'use_imu_preintegration': False,
            'use_continuous_time_deskew': False,
        }],
        remappings=[
            ('/cloud', LaunchConfiguration('cloud_topic')),
            ('/odom', LaunchConfiguration('odom_topic')),
        ],
    )
    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(node),
        transition_id=Transition.TRANSITION_CONFIGURE))
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=node,
        start_state='configuring',
        goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=Transition.TRANSITION_ACTIVATE))]))
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map_path', default_value=map_path),
        DeclareLaunchArgument('scan_max_range', default_value='8.0'),
        DeclareLaunchArgument('rotation_guard_deg', default_value='5.0'),
        DeclareLaunchArgument('recovery_review', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('candidate_review', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('direction_shadow', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('direction_min_ratio', default_value='0.05'),
        DeclareLaunchArgument('match_diagnostics', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('publish_tf', default_value='false', choices=['true', 'false']),
        # This frame has the PCT map's coordinates but a distinct TF name. The
        # existing mission still owns odom -> building_pct during its run.
        DeclareLaunchArgument('global_frame_id', default_value='localization_probe'),
        DeclareLaunchArgument('cloud_topic', default_value='/livox/points_raw'),
        DeclareLaunchArgument('odom_topic', default_value='/fast_lio/odometry_base'),
        activate,
        node,
        configure,
    ])
