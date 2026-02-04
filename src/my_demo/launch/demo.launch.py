#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    publisher_node = Node(
        package='my_demo',
        executable='publisher_node',
        name='simple_publisher',
        output='screen',
    )

    subscriber_node = Node(
        package='my_demo',
        executable='subscriber_node',
        name='simple_subscriber',
        output='screen',
    )

    return LaunchDescription([
        publisher_node,
        subscriber_node,
    ])
