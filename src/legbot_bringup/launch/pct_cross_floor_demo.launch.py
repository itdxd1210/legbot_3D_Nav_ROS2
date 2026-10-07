"""One-command Gazebo + PCT + SCAN cross-floor mission."""

import os
import json
import math

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def include(package, launch_file, arguments=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share(package), 'launch', launch_file)),
        launch_arguments=(arguments or {}).items(),
        condition=condition)


def setup(context):
    navigation_source = LaunchConfiguration('navigation_source').perform(context)
    use_ground_truth = navigation_source == 'ground_truth'
    correction = LaunchConfiguration('localization_correction').perform(context) == 'true'
    initial_search = LaunchConfiguration('initial_localization').perform(context) == 'true'
    planning_frame = 'nav_map' if correction else 'odom'
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')
    use_sim_time = LaunchConfiguration('use_sim_time')
    diagnose_fastlio = (
        not use_ground_truth and
        LaunchConfiguration('diagnose_fastlio').perform(context) == 'true')
    align_with_ground_truth = (
        not use_ground_truth and
        LaunchConfiguration('align_with_ground_truth').perform(context) == 'true')
    localization_map_path = LaunchConfiguration('localization_map_path').perform(context)
    if localization_map_path:
        localization_map_path = os.path.abspath(os.path.expanduser(localization_map_path))
        if not os.path.isfile(localization_map_path):
            raise ValueError('Localization PCD does not exist: ' + localization_map_path)

    map_config = LaunchConfiguration('map_config').perform(context)
    custom_map = None
    start_mode = 'fixed'
    if map_config:
        with open(os.path.expanduser(map_config), encoding='utf-8') as handle:
            custom_map = json.load(handle)
        start_mode = custom_map.get('start_mode', 'fixed')
        if start_mode not in ('fixed', 'current'):
            raise ValueError('start_mode must be fixed or current')
        if start_mode == 'current' and not initial_search:
            raise ValueError('start_mode=current requires initial_localization:=true')
        if use_ground_truth or not (align_with_ground_truth or initial_search):
            raise ValueError('Custom-map experiment requires fastlio and align_with_ground_truth:=true')
        required = [('world_from_pct', 7), ('goal', 3), ('spawn', 3)]
        if start_mode == 'fixed':
            required.append(('start', 3))
        for key, size in required:
            values = custom_map[key]
            if len(values) != size or not all(math.isfinite(float(v)) for v in values):
                raise ValueError('Invalid custom-map ' + key)
        if abs(sum(float(v)**2 for v in custom_map['world_from_pct'][3:]) - 1.0) > 1e-3:
            raise ValueError('Custom-map quaternion must be normalized')
        spawn_x, spawn_y, spawn_z = [TextSubstitution(text=str(v)) for v in custom_map['spawn']]
        if LaunchConfiguration('localization_probe').perform(context) == 'true':
            for filename in (['alignment.json'] if localization_map_path else ['map_odom.pcd', 'alignment.json']):
                if not os.path.isfile(os.path.join(os.path.dirname(os.path.abspath(map_config)), filename)):
                    raise ValueError('Self-map comparison requires ' + filename + ' next to map_config')

    if initial_search:
        if use_ground_truth or not custom_map or align_with_ground_truth:
            raise ValueError('initial_localization requires a self-map, fastlio and align_with_ground_truth:=false')
        native_pct = custom_map.get('map_from_pct')
        if (not isinstance(native_pct, list) or len(native_pct) != 7
                or not all(math.isfinite(float(v)) for v in native_pct)
                or abs(sum(float(v)**2 for v in native_pct[3:])-1.) > 1e-3):
            raise ValueError('initial_localization requires map_from_pct calibration in map_config')
        assets = os.path.join(os.path.dirname(os.path.abspath(map_config)), 'initial_localization', 'occupancy.yaml')
        if not os.path.isfile(assets):
            raise ValueError('Initial-localization search assets missing: '+assets)

    localization_probe = (
        not use_ground_truth and
        (correction or LaunchConfiguration('localization_probe').perform(context) == 'true'))
    if correction and (use_ground_truth or not custom_map):
        raise ValueError('localization_correction requires fastlio and a self-map map_config')
    if correction:
        for filename in (['alignment.json'] if localization_map_path else ['map_odom.pcd', 'alignment.json']):
            if not os.path.isfile(os.path.join(os.path.dirname(os.path.abspath(map_config)), filename)):
                raise ValueError('Correction requires ' + filename)
    if use_ground_truth:
        odom_topic = '/Odometry_gazebo'
        cloud_topic = '/livox/points_world'
        sensor_pose_topic = '/go2/lidar_pose'
        localization_label = 'Gazebo ground truth'
        rviz_config = os.path.join(
            share('legbot_bringup'), 'rviz', 'scan_stairs_demo.rviz')
    else:
        odom_topic = '/fast_lio/odometry_base'
        cloud_topic = '/fast_lio/cloud_registered'
        sensor_pose_topic = '/fast_lio/odometry_lidar'
        localization_label = 'FAST-LIO'
        rviz_config = os.path.join(
            share('legbot_bringup'), 'rviz', 'scan_fastlio_flat.rviz')

    if correction:
        odom_topic = '/navigation/odometry_base'
        cloud_topic = '/navigation/cloud_registered'
        sensor_pose_topic = '/navigation/odometry_lidar'
        localization_label = 'FAST-LIO with map correction'

    actions = [
        include('legbot_bringup', 'simulation.launch.py', {
            'gui': gui,
            'x': spawn_x,
            'y': spawn_y,
            'z': spawn_z,
            'yaw': spawn_yaw,
            'lidar_pitch': LaunchConfiguration('lidar_pitch'),
            'navigation_source': navigation_source,
            # In the explicit 6D-alignment experiment, Gazebo supplies only
            # the initial map pose; SCAN and control still use FAST-LIO odom.
            'publish_ground_truth': 'true' if (
                use_ground_truth or diagnose_fastlio or align_with_ground_truth) else 'false',
            'diagnose_fastlio': 'true' if diagnose_fastlio else 'false',
        }),
    ]
    if not use_ground_truth:
        actions.append(include('legbot_bringup', 'fastlio.launch.py', {
            'use_sim_time': use_sim_time,
            'config': os.path.join(
                share('legbot_bringup'), 'config', 'fastlio_sim.yaml'),
            'gravity_alignment': 'true' if custom_map else 'false',
            'wait_for_start': 'true',
            'start_topic': '/fast_lio/start',
        }))

    if initial_search:
        actions.extend([
            Node(package='legbot_bringup', executable='initial_localization_probe', output='screen',
                 parameters=[{'use_sim_time': True, 'occupancy_yaml': assets}]),
            Node(package='legbot_bringup', executable='initial_localization_bridge', output='screen',
                 parameters=[{'use_sim_time': True, 'seed_map_matching': localization_probe}]),
        ])

    if localization_probe:
        probe_args = {'use_sim_time': use_sim_time, 'publish_tf': 'false',
                      'scan_max_range': LaunchConfiguration('localization_scan_max_range'),
                      'rotation_guard_deg': LaunchConfiguration('localization_rotation_guard_deg'),
                      'recovery_review': LaunchConfiguration('localization_recovery_review'),
                      'candidate_review': LaunchConfiguration('localization_candidate_review'),
                      'direction_shadow': LaunchConfiguration('localization_direction_shadow'),
                      'direction_min_ratio': LaunchConfiguration('localization_direction_min_ratio'),
                      'match_diagnostics': LaunchConfiguration('localization_diagnostics')}
        if custom_map:
            map_dir = os.path.dirname(os.path.abspath(map_config))
            # An optional crop must retain the original map coordinates;
            # route configuration and alignment.json still belong to map_dir.
            probe_args['map_path'] = localization_map_path or os.path.join(map_dir, 'map_odom.pcd')
            actions.append(Node(
                package='legbot_bringup', executable='localization_comparison',
                output='screen', parameters=[{'use_sim_time': True,
                    'map_config': os.path.abspath(map_config),
                    'alignment_frame': planning_frame,
                    'record_details': LaunchConfiguration('localization_diagnostics').perform(context) == 'true',
                    'seed_initial_pose': not initial_search,
                    'alignment_file': os.path.join(map_dir, 'alignment.json')}]))
            if correction:
                actions.append(Node(
                    package='legbot_bringup', executable='localization_navigation_bridge',
                    output='screen', parameters=[{'use_sim_time': True,
                        'map_config': os.path.abspath(map_config),
                        **({'map_from_pct': native_pct} if initial_search else {}),
                        'alignment_file': os.path.join(map_dir, 'alignment.json')}]))
        if localization_map_path:
            probe_args['map_path'] = localization_map_path
        actions.append(include('legbot_bringup', 'map_localization.launch.py', {
            **probe_args,
        }))

    # The normal six-terminal workflow starts this coordinator only after all
    # ros2_control spawners finish. Preserve that ordering in the all-in-one
    # launch so its first mode pulse is never lost.
    actions.extend([
        TimerAction(period=8.0, actions=[include(
            'legbot_bringup', 'go2_demo_control.launch.py', {
                'use_sim_time': use_sim_time,
                'odom_topic': odom_topic,
                'cloud_topic': cloud_topic,
                'localization_label': localization_label,
                'gate_fastlio_start': 'true' if not use_ground_truth else 'false',
                'fastlio_start_topic': '/fast_lio/start',
            })]),
        include('legbot_bringup', 'pct_plan.launch.py', {
            'use_sim_time': use_sim_time,
            'tomogram': custom_map['tomogram'] if custom_map else 'building2_9',
            'start': ' '.join(map(str, custom_map['start'])) if custom_map and start_mode == 'fixed' else '',
            'start_mode': start_mode,
            'start_pose_topic': '/initial_localization/odometry_base' if start_mode == 'current' else '',
            'start_pose_frame': 'initial_map',
            'map_from_pct': ' '.join(map(str, native_pct)) if start_mode == 'current' else '',
            'goal': ' '.join(map(str, custom_map['goal'])) if custom_map else '',
            'path_topic': '/pct_path_raw',
            'frame_id': 'building_pct',
            'display_frame': planning_frame,
        }),
        include('legbot_bringup', 'scan.launch.py', {
            'use_sim_time': use_sim_time,
            'navi_mode': '3',
            'align_reference_height_to_odom': 'false' if custom_map else 'true',
            'global_path_topic': '/pct_path',
            # pct_plan also declares a generic frame_id argument.  Pass this
            # explicitly so the include cannot leak building_pct into SCAN's
            # grid-map and marker headers.
            'frame_id': planning_frame,
            'manual_goal_use_message_z': 'true',
            'finish_dist': '0.50',
            'odom_topic': odom_topic,
            'cloud_topic': cloud_topic,
            'sensor_pose_topic': sensor_pose_topic,
        }),
        Node(
            package='legbot_bringup',
            executable='pct_path_mission',
            name='pct_path_mission',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'input_topic': '/pct_path_raw',
                # PCT emits terrain + 0.50 m. This explicit base reference
                # replaces alignment to a potentially distant first point.
                'path_height_offset': (float(LaunchConfiguration('custom_map_base_height').perform(context)) - 0.50) if custom_map else 0.0,
                'output_topic': '/pct_path',
                'goal_topic': '/scan/goal',
                'odom_topic': odom_topic,
                'map_pose_topic': '/initial_localization/odometry_base' if initial_search else ('/Odometry_gazebo' if align_with_ground_truth else ''),
                'ready_topic': '/go2/demo_ready',
                'frame_id': planning_frame,
                'pct_frame_id': 'building_pct',
                # Ground-truth odom is the Gazebo world frame, so its map TF
                # is published immediately below.  FAST-LIO still needs the
                # mission bridge's start-pose calibration.
                'publish_map_tf': not use_ground_truth,
                'world_from_pct': native_pct if initial_search else (custom_map['world_from_pct'] if custom_map else [13.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]),
                'building_x': 13.0,
                'building_y': 0.0,
                'building_z': 0.0,
                'building_yaw': 0.0,
                'spawn_x': ParameterValue(spawn_x, value_type=float),
                'spawn_y': ParameterValue(spawn_y, value_type=float),
                'spawn_z': ParameterValue(spawn_z, value_type=float),
                'spawn_yaw': ParameterValue(spawn_yaw, value_type=float),
            }],
        ),
        *([Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='building_pct_ground_truth_tf',
            output='screen',
            arguments=[
                '--x', '13.0', '--y', '0.0', '--z', '0.0',
                '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
                '--frame-id', 'odom', '--child-frame-id', 'building_pct',
            ],
        )] if use_ground_truth else []),
        include(
            'legbot_bringup', 'scan_rviz.launch.py',
            {'use_sim_time': use_sim_time, 'config': rviz_config, 'fixed_frame': planning_frame},
            condition=IfCondition(rviz)),
    ])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('lidar_pitch', default_value='0.0',
                              description='LiDAR and internal IMU mount pitch in radians; positive tilts forward beam down'),
        DeclareLaunchArgument('custom_map_base_height', default_value='0.50',
                              description='Self-map base reference above terrain, metres; PCT output uses 0.50'),
        DeclareLaunchArgument('map_config', default_value='',
                              description='Optional self-map navigation JSON; empty preserves Building defaults'),
        DeclareLaunchArgument('initial_localization', default_value='false', choices=['true', 'false'],
                              description='Scan-only ground-floor initial pose; rejected matches block mission startup'),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument(
            'navigation_source', default_value='ground_truth',
            choices=['ground_truth', 'fastlio'],
            description=(
                'Ground truth is the reliable default for the long simulated cross-floor run; '
                'fastlio remains available for localization integration tests.')),
        DeclareLaunchArgument(
            'diagnose_fastlio', default_value='true', choices=['true', 'false'],
            description=(
                'When navigation_source:=fastlio, publish Gazebo truth for '
                'fastlio_truth_monitor. It is also an initial map-alignment '
                'reference only if align_with_ground_truth:=true.')),
        DeclareLaunchArgument(
            'align_with_ground_truth', default_value='false',
            choices=['true', 'false'],
            description=(
                'Simulation-only 6D initial map/odom alignment experiment. '
                'When true with FAST-LIO, pct_path_mission samples Gazebo world '
                'pose once; normal navigation still uses FAST-LIO.')),
        DeclareLaunchArgument('spawn_x', default_value='-27.0'),
        DeclareLaunchArgument(
            'localization_probe', default_value='false', choices=['true', 'false'],
            description=(
                'Run LiDAR-to-PCD map matching in a separate TF frame for '
                'evaluation only; it does not control SCAN or overwrite the '
                'existing PCT alignment.')),
        DeclareLaunchArgument('spawn_y', default_value='6.0'),
        DeclareLaunchArgument('localization_diagnostics', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('localization_recovery_review', default_value='true', choices=['true', 'false'],
                              description='Review large translation corrections across three consistent scans'),
        DeclareLaunchArgument('localization_candidate_review', default_value='true', choices=['true', 'false'],
                             description='Audit consecutive ordinary map-matching candidates before committing map/odom'),
        DeclareLaunchArgument('localization_direction_shadow', default_value='false', choices=['true', 'false'],
                              description='Record directional translation previews only; navigation remains unchanged'),
        DeclareLaunchArgument('localization_direction_min_ratio', default_value='0.05',
                              description='Unvalidated translation eigenvalue ratio for shadow analysis only'),
        DeclareLaunchArgument('localization_rotation_guard_deg', default_value='5.0',
                              description='Experimental full rotation innovation limit, not absolute body pitch'),
        DeclareLaunchArgument('localization_correction', default_value='false', choices=['true', 'false'],
                              description='Experimental corrected navigation frame; raw LIO stays unchanged'),
        DeclareLaunchArgument('localization_map_path', default_value='',
                              description='Optional localization-only PCD in the original map coordinates; PCT route and alignment remain unchanged'),
        DeclareLaunchArgument('localization_scan_max_range', default_value='8.0',
                              description='Maximum scan range in metres for shadow map matching only'),
        DeclareLaunchArgument('spawn_z', default_value='0.50'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        OpaqueFunction(function=setup),
    ])
