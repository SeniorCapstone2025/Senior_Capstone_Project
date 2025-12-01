import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from statemachine import StateMachine, State
import json


class ScanRoutine(Node, StateMachine):
    # Define states
    IDLE = State('idle', initial=True)
    FIND_LOCATION = State('find')
    SCAN_QR_CODE = State('scanQR')
    VERIFY_OBJECT = State('verifyObject')
    LOG_RESULT = State('logResult')
    COMPLETE = State('complete')
    ERROR = State('error')

    # Define state transitions
    start_scan = IDLE.to(FIND_LOCATION)
    location_found = FIND_LOCATION.to(SCAN_QR_CODE)
    qr_scanned = SCAN_QR_CODE.to(VERIFY_OBJECT)
    object_verified = VERIFY_OBJECT.to(LOG_RESULT)
    log_complete = LOG_RESULT.to(COMPLETE)
    finish = COMPLETE.to(IDLE)
    handle_error = (
        FIND_LOCATION.to(ERROR) |
        SCAN_QR_CODE.to(ERROR) |
        VERIFY_OBJECT.to(ERROR) |
        LOG_RESULT.to(ERROR)
    )
    reset = ERROR.to(IDLE)

    def __init__(self):
        Node.__init__(self, 'scan_routine_node')
        StateMachine.__init__(self)

        # Subscribers
        self.cmd_sub = self.create_subscription(String, '/commands', self.command_callback, 10)
        self.nav_result_sub = self.create_subscription(String, '/nav/result', self.nav_result_callback, 10)
        self.qr_result_sub = self.create_subscription(String, '/qr/result', self.qr_result_callback, 10)

        # Publishers
        self.status_pub = self.create_publisher(String, '/fsm_status', 10)
        self.nav_pub = self.create_publisher(String, '/nav/goal', 10)

        # State data
        self.target_position = None
        self.qr_data = None
        self.error_msg = None

        self.get_logger().info("ScanRoutine initialized in IDLE state")

    def command_callback(self, msg):
        """Handle incoming commands from controller."""
        try:
            data = json.loads(msg.data)
            action = data.get("action")

            if action == "scan_item" and self.current_state == self.IDLE:
                self.target_position = data.get("position")
                self.get_logger().info(f"Starting scan for position: {self.target_position}")
                self.start_scan()
        except Exception as e:
            self.get_logger().error(f"Error in command_callback: {e}")

    def nav_result_callback(self, msg):
        """Handle navigation results."""
        
        # TODO : Implement handling of navigation results
        self.get_logger().info(f"Bypassing nav_result_callback until implemented.")
        self.location_found()
        # try:
        #     if self.current_state == self.FIND_LOCATION:
        #         result = msg.data
        #         if result == "success":
        #             self.location_found()
        #         else:
        #             self.error_msg = f"Navigation failed: {result}"
        #             self.handle_error()
        # except Exception as e:
        #     self.get_logger().error(f"Error in nav_result_callback: {e}")

    def qr_result_callback(self, msg):
        """Handle QR scan results."""
        try:
            if self.current_state == self.SCAN_QR_CODE:
                self.qr_data = msg.data
                if self.qr_data:
                    self.qr_scanned()
                else:
                    self.error_msg = "QR scan failed - no data"
                    self.handle_error()
        except Exception as e:
            self.get_logger().error(f"Error in qr_result_callback: {e}")

    # Transition functions for each state
    def on_enter_FIND_LOCATION(self):
        """Called when entering FIND_LOCATION state."""
        try:
            self.get_logger().info(f"STATE: FIND_LOCATION - Navigating to {self.target_position}")

            # TODO: implement navigation command
            self.get_logger().info(f"Bypassing navigation command until implemented.")
            self.location_found()
            
            # nav_cmd = String()
            # nav_cmd.data = json.dumps({
            #     "goal": self.target_position,
            #     "action": "navigate"
            # })
            # self.nav_pub.publish(nav_cmd)
            # self.send_status("finding_location")
        except Exception as e:
            self.get_logger().error(f"Error in on_enter_FIND_LOCATION: {e}")
            self.error_msg = str(e)
            self.handle_error()

    def on_enter_SCAN_QR_CODE(self):
        """Called when entering SCAN_QR_CODE state."""
        try:
            self.get_logger().info("STATE: SCAN_QR_CODE - Starting QR scan")

            qr_cmd = String()
            qr_cmd.data = json.dumps({
                "action": "scan_qr",
                "position": self.target_position
            })
            # Publish to QR scanner topic (would be implemented separately)
            self.send_status(json.dumps({
                "status": "in_progress",
                "position": self.target_position,
                "message": "scanning_qr"
            }))

            # In real implementation, wait for QR scanner callback
            # For now, simulate with timer
            self.simulate_qr_scan()
        except Exception as e:
            self.get_logger().error(f"Error in on_enter_SCAN_QR_CODE: {e}")
            self.error_msg = str(e)
            self.handle_error()

    def simulate_qr_scan(self):
        """Temporary simulation of QR scan."""
        self.qr_data = f"QR_{self.target_position}"
        self.qr_scanned()

    def on_enter_VERIFY_OBJECT(self):
        """Called when entering VERIFY_OBJECT state."""
        try:
            self.get_logger().info(f"STATE: VERIFY_OBJECT - Verifying {self.qr_data}")

            is_valid = self.verify_qr_data()

            if is_valid:
                log_data = {
                    "status": "in_progress",
                    "position": self.target_position,
                    "qr_data": self.qr_data,
                    "timestamp": str(self.get_clock().now().to_msg()),
                }
                
                self.send_status(json.dumps(log_data))
                self.object_verified()
            else:
                self.error_msg = f"Verification failed for {self.target_position}"
                self.handle_error()
        except Exception as e:
            self.get_logger().error(f"Error in on_enter_VERIFY_OBJECT: {e}")
            self.error_msg = str(e)
            self.handle_error()

    def verify_qr_data(self):
        """Verify QR data matches expected format."""
        # TODO: Implement object recognition/verification logic
        
        if not self.qr_data:
            return False
        # return self.target_position in self.qr_data
        self.get_logger().info(f"Bypassing verify_qr_data until implemented.")
        return True

    def on_enter_LOG_RESULT(self):
        """Called when entering LOG_RESULT state."""
        try:
            self.get_logger().info(f"STATE: LOG_RESULT - Logging result for {self.target_position}")

            log_data = {
                "status": "success",
                "position": self.target_position,
                "qr_data": self.qr_data,
                "timestamp": str(self.get_clock().now().to_msg()),
            }

            self.send_status(json.dumps(log_data))
            self.log_complete()
        except Exception as e:
            self.get_logger().error(f"Error in on_enter_LOG_RESULT: {e}")
            self.error_msg = str(e)
            self.handle_error()

    def on_enter_COMPLETE(self):
        """Called when entering COMPLETE state."""
        try:
            self.get_logger().info(f"STATE: COMPLETE - Scan completed for {self.target_position}")

            completion_msg = json.dumps({
                "status": "complete",
                "position": self.target_position,
                "qr_data": self.qr_data,
                "timestamp": str(self.get_clock().now().to_msg()),
            })
            self.send_status(completion_msg)

            # Reset and return to IDLE
            self.finish()
        except Exception as e:
            self.get_logger().error(f"Error in on_enter_COMPLETE: {e}")

    def on_enter_ERROR(self):
        """Called when entering ERROR state."""
        try:
            self.get_logger().error(f"STATE: ERROR - {self.error_msg}")

            error_data = json.dumps({
                "status": "error",
                "position": self.target_position,
                "error": self.error_msg
            })
            self.send_status(error_data)

            # Reset to IDLE
            self.reset()
        except Exception as e:
            self.get_logger().error(f"Critical error in on_enter_ERROR: {e}")

    def on_enter_IDLE(self):
        """Called when entering IDLE state."""
        self.target_position = None
        self.qr_data = None
        self.error_msg = None
        self.get_logger().info("STATE: IDLE - Ready for next command")

    def send_status(self, status: str):
        """Publish status message."""
        pub_msg = String()
        pub_msg.data = status
        self.status_pub.publish(pub_msg)


def main(args=None):
    print("Starting Scan Routine Node...")
    rclpy.init(args=args)
    node = ScanRoutine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()