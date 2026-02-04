#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'device',
            default_value='cpu',
            description='Compute device: cpu, cuda, or cuda:0'
        ),
        DeclareLaunchArgument(
            'model',
            default_value='yolo11n.pt',
            description='Model file name'
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/ascamera/camera_publisher/rgb0/image',
            description='Image topic to subscribe to'
        ),
        DeclareLaunchArgument(
            'confidence',
            default_value='0.35',
            description='Minimum confidence threshold'
        ),
        DeclareLaunchArgument(
            'show_result',
            default_value='false',
            description='Display detection results in window'
        ),
        DeclareLaunchArgument(
            'pub_result_img',
            default_value='false',
            description='Publish annotated result images'
        ),

        Node(
            package='yolov11_ros2',
            executable='yolov11_detect',
            name='yolov11_ros2',
            output='screen',
            parameters=[{
                'device': LaunchConfiguration('device'),
                'model': LaunchConfiguration('model'),
                'image_topic': LaunchConfiguration('image_topic'),
                'confidence': LaunchConfiguration('confidence'),
                'show_result': LaunchConfiguration('show_result'),
                'pub_result_img': LaunchConfiguration('pub_result_img'),
            }]
        ),
    ])
