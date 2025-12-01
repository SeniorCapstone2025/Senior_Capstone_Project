import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json


class ControllerNode(Node):
    def __init__(self, locations: list = None):
        super().__init__('controller_node')
        
        self.cmd_pub = self.create_publisher(String, '/commands', 10)
        
        self.fsm_sub = self.create_subscription(String, '/fsm_status', self.fsm_status_cb, 10)
        self.backend_cmds_sub = self.create_subscription(String, '/backend_commands', self.scan_command_callback, 10)
        
        self.locations = locations if locations is not None else []
        self.current_idx = 0
        self.waiting_for_result = False
        
        
    def scan_command_callback(self, msg):
        """Receives a high-level command from backend."""
        try:
            data = msg.data

            if data.startswith("start_scan_row"):
                self.locations = ["A1", "A2", "A3"]  # placeholder for real list
                self.current_idx = 0
                self.get_logger().info("Mission started: scanning row")
                self.run_next_item()
        except Exception as e:
            self.get_logger().error(f"Error in scan_command_callback: {e}")
            
    def run_next_item(self):
        """Triggers FSM for next item."""
        try:
            if self.current_idx >= len(self.locations):
                self.get_logger().info("ScanRow complete.")
                return

            target = self.locations[self.current_idx]

            cmd = String()
            cmd.data = json.dumps({"action": "scan_item", "position": target})

            self.cmd_pub.publish(cmd)
            self.get_logger().info(f"Sent scan command for {target}")

            self.waiting_for_result = True
        except (json.JSONDecodeError, TypeError) as e:
            self.get_logger().error(f"JSON encoding error in run_next_item: {e}")
            self.waiting_for_result = False
        except Exception as e:
            self.get_logger().error(f"Error in run_next_item: {e}")
            self.waiting_for_result = False

    def fsm_status_cb(self, msg):
        """Receives result from FSM Node."""
        try:
            if not self.waiting_for_result:
                return

            # Parse JSON from the String message
            data = json.loads(msg.data)

            # Check if status is complete
            if data['status'] != "complete":
                self.get_logger().info(f"FSM status update: {data['status']}")
                return    

            self.waiting_for_result = False
            self.get_logger().info(f"FSM completed: {data}")

            # TODO: publish result upstream to backend here

            self.current_idx += 1
            self.run_next_item()
        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON decode error in fsm_status_cb: {e}")
            self.waiting_for_result = False
        except Exception as e:
            self.get_logger().error(f"Error in fsm_status_cb: {e}")
            self.waiting_for_result = False
        
        
def main(args=None):
    print("Starting Controller Node...")
    rclpy.init(args=args)
    node = ControllerNode(["A1", "A2", "A3"])
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
        
        
        