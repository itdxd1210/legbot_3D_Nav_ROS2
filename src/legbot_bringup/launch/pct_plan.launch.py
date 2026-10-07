from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('start', default_value=''),
        DeclareLaunchArgument('start_mode', default_value='fixed'),
        DeclareLaunchArgument('start_pose_topic', default_value=''),
        DeclareLaunchArgument('start_pose_frame', default_value='initial_map'),
        DeclareLaunchArgument('map_from_pct', default_value=''),
        DeclareLaunchArgument('goal', default_value=''),
        DeclareLaunchArgument('tomogram', default_value='building2_9'),
        DeclareLaunchArgument('path_topic', default_value='/pct_path_raw'),
        DeclareLaunchArgument('frame_id', default_value='building_pct'),
        DeclareLaunchArgument('display_frame', default_value=LaunchConfiguration('frame_id')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=setup),
    ])


def setup(context):
    extra = []
    mode = LaunchConfiguration('start_mode').perform(context)
    if mode not in ('fixed', 'current'):
        raise ValueError('start_mode must be fixed or current')
    extra += ['--start-mode', mode]
    if mode == 'current':
        topic = LaunchConfiguration('start_pose_topic').perform(context)
        transform = LaunchConfiguration('map_from_pct').perform(context).split()
        if not topic or len(transform) != 7:
            raise ValueError('Current PCT start requires pose topic and map_from_pct')
        extra += ['--start-pose-topic', topic, '--start-pose-frame',
                  LaunchConfiguration('start_pose_frame').perform(context),
                  '--map-from-pct'] + [str(float(v)) for v in transform]
    for key in ('start', 'goal'):
        if key == 'start' and mode == 'current':
            continue
        value = LaunchConfiguration(key).perform(context).split()
        if value:
            if len(value) != 3:
                raise ValueError(key + ' must contain three coordinates')
            extra += ['--' + key] + [str(float(v)) for v in value]
    return [Node(package='pct_planner', executable='pct_plan', output='screen',
             parameters=[{'use_sim_time':ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)}],
             arguments=['--scene','Go2',
                        '--tomogram',LaunchConfiguration('tomogram'),
                        '--path-topic',LaunchConfiguration('path_topic'),
                        '--frame-id',LaunchConfiguration('frame_id'),
                        '--display-frame',LaunchConfiguration('display_frame')] + extra)]
