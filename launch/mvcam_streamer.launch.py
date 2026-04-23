import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
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
        DeclareLaunchArgument(name='start_stream_server',
                              default_value='true'),
        DeclareLaunchArgument(name='stream_bind_address',
                              default_value='127.0.0.1'),
        DeclareLaunchArgument(name='stream_port',
                              default_value='8554'),
        DeclareLaunchArgument(name='stream_path',
                              default_value='/stream'),
        DeclareLaunchArgument(name='start_web_preview',
                              default_value='true'),
        DeclareLaunchArgument(name='web_bind_address',
                              default_value='0.0.0.0'),
        DeclareLaunchArgument(name='web_port',
                              default_value='8554'),
        DeclareLaunchArgument(name='web_stream_path',
                              default_value='/stream.mjpg'),
        DeclareLaunchArgument(name='start_preview_window',
                              default_value='false'),
        DeclareLaunchArgument(name='preview_url',
                              default_value='udp://127.0.0.1:8554?overrun_nonfatal=1&fifo_size=50000000'),

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
            executable='h264_rtsp_stream_server',
            name='h264_rtsp_stream_server',
            output='both',
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration('start_stream_server')),
            parameters=[{
                'bind_address': LaunchConfiguration('stream_bind_address'),
                'port': LaunchConfiguration('stream_port'),
                'stream_path': LaunchConfiguration('stream_path'),
                'input_topic': '/image_raw/compressed',
            }, LaunchConfiguration('params_file')],
            arguments=['--ros-args', '--log-level', 'info'],
        ),
        Node(
            package='mvcam_streamer',
            executable='mjpeg_stream_server',
            name='mjpeg_stream_server',
            output='both',
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration('start_web_preview')),
            parameters=[{
                'bind_address': LaunchConfiguration('web_bind_address'),
                'port': LaunchConfiguration('web_port'),
                'stream_path': LaunchConfiguration('web_stream_path'),
                'input_topic': '/image_raw/compressed',
            }, LaunchConfiguration('params_file')],
            arguments=['--ros-args', '--log-level', 'info'],
        ),
        ExecuteProcess(
            condition=IfCondition(LaunchConfiguration('start_preview_window')),
            cmd=[
                'ffplay',
                '-fflags', 'nobuffer',
                '-flags', 'low_delay',
                '-framedrop',
                '-window_title', 'Mvcam Stream Preview',
                LaunchConfiguration('preview_url'),
            ],
            output='screen',
        ),
    ])
