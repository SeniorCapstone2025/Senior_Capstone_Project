#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')

        self.publisher = self.create_publisher(String, 'demo_topic', 10)

        self.timer = self.create_timer(1.0, self.timer_callback)

        self.counter = 0

        self.get_logger().info('Simple Publisher Node started!')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello from MentorPi! Count: {self.counter}'

        self.publisher.publish(msg)

        self.get_logger().info(f'Publishing: "{msg.data}"')

        self.counter += 1


def main(args=None):
    rclpy.init(args=args)

    node = SimplePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
