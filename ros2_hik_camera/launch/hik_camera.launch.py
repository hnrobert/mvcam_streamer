import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('hik_camera'), 'config', 'camera_params.yaml')

    camera_info_url = 'package://hik_camera/config/camera_info.yaml'

    return LaunchDescription([
        DeclareLaunchArgument(name='params_file',
                              default_value=params_file),
        DeclareLaunchArgument(name='camera_info_url',
                              default_value=camera_info_url),
        DeclareLaunchArgument(name='use_sensor_data_qos',
                              default_value='false'),
        DeclareLaunchArgument(name='publish_compressed',
                              default_value='true'),
        DeclareLaunchArgument(name='jpeg_quality',
                              default_value='85'),
        DeclareLaunchArgument(name='start_stream_server',
                              default_value='true'),
        DeclareLaunchArgument(name='stream_bind_address',
                              default_value='127.0.0.1'),
        DeclareLaunchArgument(name='stream_port',
                              default_value='8080'),
        DeclareLaunchArgument(name='stream_path',
                              default_value='/stream.mjpg'),

        Node(
            package='hik_camera',
            executable='hik_camera_node',
            output='both',
            emulate_tty=True,
            parameters=[LaunchConfiguration('params_file'), {
                'camera_info_url': LaunchConfiguration('camera_info_url'),
                'use_sensor_data_qos': LaunchConfiguration('use_sensor_data_qos'),
                'publish_compressed': LaunchConfiguration('publish_compressed'),
                'jpeg_quality': LaunchConfiguration('jpeg_quality'),
            }],
            arguments=['--ros-args', '--log-level', 'info'],
        ),
        Node(
            package='hik_camera',
            executable='mjpeg_stream_server',
            output='both',
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration('start_stream_server')),
            parameters=[{
                'bind_address': LaunchConfiguration('stream_bind_address'),
                'port': LaunchConfiguration('stream_port'),
                'stream_path': LaunchConfiguration('stream_path'),
                'input_topic': '/image_raw/compressed',
            }],
            arguments=['--ros-args', '--log-level', 'info'],
        ),
    ])
