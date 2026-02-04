#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Get package paths
    app_package_path = get_package_share_directory('app')
    yolov11_package_path = get_package_share_directory('yolov11_ros2')

    # Config file paths
    waypoints_file = os.path.join(app_package_path, 'config', 'waypoints.yaml')
    inventory_config = os.path.join(app_package_path, 'config', 'inventory_config.yaml')

    # Inventory FSM Node
    inventory_fsm_node = Node(
        package='app',
        executable='inventory_fsm_node',
        name='inventory_fsm',
        output='screen',
        parameters=[{
            'backend_url': 'http://localhost:8000',
            'backend_timeout': 10.0,
        }]
    )

    # Waypoint Navigator Node
    waypoint_navigator_node = Node(
        package='app',
        executable='waypoint_navigator_node',
        name='waypoint_navigator',
        output='screen',
        parameters=[{
            'speed_linear': 0.2,
            'speed_angular': 0.5,
            'goal_tolerance': 0.15,
            'obstacle_distance': 0.3,
            'waypoints_file': waypoints_file,
        }]
    )

    # QR Scanner Node
    qr_scanner_node = Node(
        package='app',
        executable='qr_scanner_node',
        name='qr_scanner',
        output='screen',
        parameters=[{
            'timeout': 10.0,
            'camera_topic': '/ascamera/camera_publisher/rgb0/image',
        }]
    )

    # YOLO Detector Wrapper Node
    yolo_detector_node = Node(
        package='app',
        executable='yolo_detector_node',
        name='yolo_detector',
        output='screen',
        parameters=[{
            'detection_duration': 3.0,
            'min_confidence': 0.35,
        }]
    )

    # Include YOLOv11 detection node
    yolov11_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(yolov11_package_path, 'launch', 'yolov11_ros2.launch.py')
        ),
        launch_arguments={
            'device': 'cpu',
            'confidence': '0.35',
            'show_result': 'false',
            'pub_result_img': 'false',
        }.items()
    )

    return LaunchDescription([
        yolov11_launch,
        inventory_fsm_node,
        waypoint_navigator_node,
        qr_scanner_node,
        yolo_detector_node,
    ])
