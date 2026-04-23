import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('mvcam_streamer'), 'config', 'camera_params.yaml')

    camera_info_url = 'package://mvcam_streamer/config/camera_info.yaml'

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
        DeclareLaunchArgument(name='start_webrtc',
                              default_value='true'),
        DeclareLaunchArgument(name='webrtc_bind_address',
                              default_value='0.0.0.0'),
        DeclareLaunchArgument(name='webrtc_port',
                              default_value='8554'),
        DeclareLaunchArgument(name='webrtc_base_path',
                              default_value='/webrtc'),

        Node(
            package='mvcam_streamer',
            executable='mvcam_streamer_node',
            name='hik_camera',
            output='both',
            emulate_tty=True,
            parameters=[{
                'camera_info_url': LaunchConfiguration('camera_info_url'),
                'use_sensor_data_qos': LaunchConfiguration('use_sensor_data_qos'),
                'publish_compressed': LaunchConfiguration('publish_compressed'),
                'jpeg_quality': LaunchConfiguration('jpeg_quality'),
            }, LaunchConfiguration('params_file')],
            arguments=['--ros-args', '--log-level', 'info'],
        ),
        Node(
            package='mvcam_streamer',
            executable='webrtc_stream_server',
            name='webrtc_stream_server',
            output='both',
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration('start_webrtc')),
            parameters=[{
                'bind_address': LaunchConfiguration('webrtc_bind_address'),
                'port': LaunchConfiguration('webrtc_port'),
                'base_path': LaunchConfiguration('webrtc_base_path'),
                'input_topic': '/image_raw/compressed',
            }, LaunchConfiguration('params_file')],
            arguments=['--ros-args', '--log-level', 'info'],
        ),
    ])
