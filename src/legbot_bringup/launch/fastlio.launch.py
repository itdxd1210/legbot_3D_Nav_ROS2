import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def setup(context):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    config = LaunchConfiguration('config').perform(context)
    sensor_frame = LaunchConfiguration('sensor_frame').perform(context)
    return [
        Node(package='fast_lio', executable='fastlio_mapping',
             parameters=[config, {
                 'use_sim_time': use_sim_time,
                 'common.wait_for_start': ParameterValue(
                     LaunchConfiguration('wait_for_start'), value_type=bool),
                 'common.start_topic': ParameterValue(
                     LaunchConfiguration('start_topic'), value_type=str),
             }],
             remappings=[('/Odometry','/fast_lio/odometry_sensor'),
                         ('/cloud_registered','/fast_lio/cloud_registered'),
                         ('/cloud_registered_body','/fast_lio/cloud_registered_body'),
                         ('/Laser_map','/fast_lio/map'),
                         ('/path','/fast_lio/path')]),
        Node(package='legbot_bringup', executable='fastlio_odom_adapter',
             parameters=[{'use_sim_time': use_sim_time, 'sensor_frame': sensor_frame,
                          'lidar_frame': LaunchConfiguration('lidar_frame').perform(context),
                          'base_frame': 'base'}],
             remappings=[('input','/fast_lio/odometry_sensor'),
                         ('output','/fast_lio/odometry_base'),
                         ('lidar_output','/fast_lio/odometry_lidar')]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('config', default_value=os.path.join(share('legbot_bringup'),'config/fastlio_real.yaml')),
        DeclareLaunchArgument('sensor_frame', default_value='livox_imu_link'),
        DeclareLaunchArgument('lidar_frame', default_value='livox_link'),
        DeclareLaunchArgument('wait_for_start', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('start_topic', default_value='/fast_lio/start'),
        OpaqueFunction(function=setup),
    ])
