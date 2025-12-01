import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys


class MockBackendNode(Node):
    def __init__(self):
        super().__init__('mock_backend_node')

        # Publisher to send commands to ControllerNode
        self.backend_cmd_pub = self.create_publisher(String, '/backend_commands', 10)

        # Subscriber to monitor what ControllerNode publishes to /commands
        self.cmd_monitor_sub = self.create_subscription(
            String,
            '/commands',
            self.commands_monitor_callback,
            10
        )

        self.get_logger().info('Mock Backend Node started')
        self.get_logger().info('Monitoring /commands topic...')
        self.get_logger().info('Use: ros2 run robot_controller mock_backend <command>')

    def commands_monitor_callback(self, msg):
        """Monitor and echo messages published to /commands by ControllerNode."""
        self.get_logger().info(f'[/commands] Received: {msg.data}')

    def send_command(self, command: str):
        """Send a command to ControllerNode via /backend_commands."""
        msg = String()
        msg.data = command
        self.backend_cmd_pub.publish(msg)
        self.get_logger().info(f'[/backend_commands] Sent: {command}')


def main(args=None):
    print("Starting Mock Backend Node...")
    rclpy.init(args=args)

    node = MockBackendNode()

    # Check if a command was provided via CLI
    if len(sys.argv) > 1:
        command = sys.argv[1]
        node.get_logger().info(f'Sending command from CLI: {command}')
        # Give publishers time to connect
        import time
        time.sleep(0.5)
        node.send_command(command)
        node.get_logger().info('Command sent. Now monitoring /commands topic...')
    else:
        node.get_logger().info('No command provided. Monitoring /commands topic only.')
        node.get_logger().info('Available commands: start_scan_row, stop_scan')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Mock Backend Node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()