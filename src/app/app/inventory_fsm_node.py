#!/usr/bin/env python3
# encoding: utf-8
"""
Inventory FSM Node - Main controller for autonomous inventory scanning.

Coordinates the scan process through states:
IDLE -> NAVIGATE_TO_SHELF -> ALIGN_WITH_SHELF -> FETCH_EXPECTED_INVENTORY ->
RUN_YOLO_DETECTION -> COMPARE_INVENTORY -> SEND_RESULTS -> RETURN_HOME -> IDLE

Communicates with backend via rosbridge topics (no HTTP).
"""

import json
import rclpy
import threading
from datetime import datetime
from enum import Enum, auto
from rclpy.node import Node
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from app.common import Heart


class ScanState(Enum):
    """FSM states for inventory scanning."""
    IDLE = auto()
    NAVIGATE_TO_SHELF = auto()
    ALIGN_WITH_SHELF = auto()
    FETCH_EXPECTED_INVENTORY = auto()
    RUN_YOLO_DETECTION = auto()
    COMPARE_INVENTORY = auto()
    SEND_RESULTS = auto()
    RETURN_HOME = auto()
    ERROR = auto()


class InventoryFSMNode(Node):
    def __init__(self, name='inventory_fsm'):
        rclpy.init()
        super().__init__(name)

        self.name = name
        self.lock = threading.RLock()

        # State management
        self.state = ScanState.IDLE
        self.is_active = False

        # Scan data
        self.current_scan_id = None
        self.current_shelf_id = None
        self.shelf_queue = []
        self.expected_items = []
        self.detected_items = []
        self.missing_items = []
        self.unexpected_items = []
        self.match = False

        # Timeout for waiting on expected inventory
        self.fetch_timeout_timer = None

        # Publishers - Navigation and detection control
        self.waypoint_goal_pub = self.create_publisher(
            String, '/waypoint_navigator/goal', 10)
        self.yolo_trigger_pub = self.create_publisher(
            Bool, '/yolo_detector/trigger', 10)

        # Publishers - Backend communication via rosbridge
        self.fsm_status_pub = self.create_publisher(
            String, '/fsm_status', 10)
        self.scan_results_pub = self.create_publisher(
            String, '/scan_results', 10)

        # Subscribers - Backend commands via rosbridge
        self.scan_start_sub = self.create_subscription(
            String, '/scan/start',
            self.scan_start_callback, 10)
        self.expected_inventory_sub = self.create_subscription(
            String, '/scan/expected_inventory',
            self.expected_inventory_callback, 10)

        # Subscribers - Created on enter (robot sensors)
        self.nav_status_sub = None
        self.detections_sub = None

        # Lifecycle services
        self.create_service(Trigger, '~/enter', self.enter_srv_callback)
        self.create_service(Trigger, '~/exit', self.exit_srv_callback)
        self.create_service(Trigger, '~/init_finish', self.init_finish_callback)

        # Scan control services (still available for direct ROS2 calls)
        self.create_service(Trigger, '~/start_scan', self.start_scan_srv_callback)
        self.create_service(Trigger, '~/stop_scan', self.stop_scan_callback)
        self.create_service(Trigger, '~/emergency_stop', self.emergency_stop_callback)

        # Heartbeat for dashboard connection
        Heart(self, f'{self.name}/heartbeat', 5,
              lambda _: self.exit_srv_callback(Trigger.Request(), Trigger.Response()))

        # Publish initial IDLE status
        self.publish_fsm_status()

        self.get_logger().info('\033[1;32m%s\033[0m' % 'Inventory FSM Node initialized')

    # =========================================================================
    # FSM Status Publishing
    # =========================================================================

    def publish_fsm_status(self):
        """Publish current state for backend subscription."""
        status = {
            "state": self.state.name,
            "scan_id": self.current_scan_id,
            "shelf_id": self.current_shelf_id,
            "timestamp": datetime.now().isoformat(),
            "message": self.get_state_message()
        }

        msg = String()
        msg.data = json.dumps(status)
        self.fsm_status_pub.publish(msg)
        self.get_logger().debug(f'Published FSM status: {self.state.name}')

    def get_state_message(self) -> str:
        """Human-readable message for current state."""
        messages = {
            ScanState.IDLE: "Ready for commands",
            ScanState.NAVIGATE_TO_SHELF: f"Navigating to {self.current_shelf_id}",
            ScanState.ALIGN_WITH_SHELF: "Aligning with shelf",
            ScanState.FETCH_EXPECTED_INVENTORY: "Waiting for expected inventory",
            ScanState.RUN_YOLO_DETECTION: "Running object detection",
            ScanState.COMPARE_INVENTORY: "Comparing inventory",
            ScanState.SEND_RESULTS: "Sending results",
            ScanState.RETURN_HOME: "Returning to home position",
            ScanState.ERROR: "Error occurred",
        }
        return messages.get(self.state, "Unknown state")

    # =========================================================================
    # Lifecycle Services
    # =========================================================================

    def enter_srv_callback(self, request, response):
        """Activate the inventory scanner mode."""
        self.get_logger().info('\033[1;32m%s\033[0m' % 'inventory_fsm enter')

        with self.lock:
            self.reset_state()
            self.is_active = True

            # Subscribe to robot sensor topics
            self.nav_status_sub = self.create_subscription(
                String, '/waypoint_navigator/status',
                self.nav_status_callback, 10)

            # Subscribe to YOLO detections
            self.detections_sub = self.create_subscription(
                String, '/yolo_detector/detections',
                self.detections_callback, 10)

        self.publish_fsm_status()

        response.success = True
        response.message = "enter"
        return response

    def exit_srv_callback(self, request, response):
        """Deactivate the inventory scanner mode."""
        self.get_logger().info('\033[1;32m%s\033[0m' % 'inventory_fsm exit')

        with self.lock:
            self.is_active = False
            self.reset_state()

            # Cancel any pending timers
            if self.fetch_timeout_timer:
                self.fetch_timeout_timer.cancel()
                self.destroy_timer(self.fetch_timeout_timer)
                self.fetch_timeout_timer = None

            # Destroy subscriptions
            if self.nav_status_sub:
                self.destroy_subscription(self.nav_status_sub)
                self.nav_status_sub = None
            if self.detections_sub:
                self.destroy_subscription(self.detections_sub)
                self.detections_sub = None

        self.publish_fsm_status()

        response.success = True
        response.message = "exit"
        return response

    def init_finish_callback(self, request, response):
        """Health check - node ready status."""
        response.success = True
        response.message = f"state: {self.state.name}"
        return response

    # =========================================================================
    # Scan Control Services
    # =========================================================================

    def start_scan_srv_callback(self, request, response):
        """Start scan via ROS2 service (legacy support)."""
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start_scan service called')

        with self.lock:
            if not self.is_active:
                response.success = False
                response.message = "Node not active. Call ~/enter first."
                return response

            if self.state != ScanState.IDLE:
                response.success = False
                response.message = f"Cannot start scan. Current state: {self.state.name}"
                return response

            # Default shelf for service-based start
            self.shelf_queue = ['shelf_1']
            self.current_scan_id = f"srv_{datetime.now().strftime('%H%M%S')}"
            self.transition_to(ScanState.NAVIGATE_TO_SHELF)

        response.success = True
        response.message = "Scan started"
        return response

    def stop_scan_callback(self, request, response):
        """Gracefully stop the current scan."""
        self.get_logger().info('\033[1;32m%s\033[0m' % 'stop_scan requested')

        with self.lock:
            if self.fetch_timeout_timer:
                self.fetch_timeout_timer.cancel()
                self.destroy_timer(self.fetch_timeout_timer)
                self.fetch_timeout_timer = None
            self.transition_to(ScanState.RETURN_HOME)

        response.success = True
        response.message = "Stopping scan, returning home"
        return response

    def emergency_stop_callback(self, request, response):
        """Immediate halt of all operations."""
        self.get_logger().error('\033[1;31m%s\033[0m' % 'EMERGENCY STOP')

        with self.lock:
            if self.fetch_timeout_timer:
                self.fetch_timeout_timer.cancel()
                self.destroy_timer(self.fetch_timeout_timer)
                self.fetch_timeout_timer = None

            self.state = ScanState.IDLE
            self.disable_all_triggers()
            self.publish_fsm_status()

        response.success = True
        response.message = "Emergency stop executed"
        return response

    # =========================================================================
    # Backend Command Callbacks (via rosbridge topics)
    # =========================================================================

    def scan_start_callback(self, msg):
        """Handle scan start command from backend."""
        try:
            data = json.loads(msg.data)
            scan_id = data.get("scan_id")
            shelf_ids = data.get("shelf_ids", [])

            self.get_logger().info(f'Scan start received: {scan_id} -> {shelf_ids}')

            with self.lock:
                if not self.is_active:
                    self.get_logger().warn('Scan start ignored - node not active')
                    return

                if self.state != ScanState.IDLE:
                    self.get_logger().warn(f'Scan start ignored - current state: {self.state.name}')
                    return

                self.current_scan_id = scan_id
                self.shelf_queue = list(shelf_ids)
                self.transition_to(ScanState.NAVIGATE_TO_SHELF)

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid scan start JSON: {e}')

    def expected_inventory_callback(self, msg):
        """Handle expected inventory from backend."""
        try:
            data = json.loads(msg.data)
            scan_id = data.get("scan_id")
            shelf_id = data.get("shelf_id")
            expected_items = data.get("expected_items", [])

            self.get_logger().info(f'Expected inventory received for {shelf_id}: {expected_items}')

            with self.lock:
                # Verify this is for our current scan
                if scan_id != self.current_scan_id:
                    self.get_logger().warn(f'Ignoring inventory for wrong scan_id: {scan_id}')
                    return

                if shelf_id != self.current_shelf_id:
                    self.get_logger().warn(f'Ignoring inventory for wrong shelf_id: {shelf_id}')
                    return

                if self.state != ScanState.FETCH_EXPECTED_INVENTORY:
                    self.get_logger().warn(f'Ignoring inventory - wrong state: {self.state.name}')
                    return

                # Cancel timeout timer
                if self.fetch_timeout_timer:
                    self.fetch_timeout_timer.cancel()
                    self.destroy_timer(self.fetch_timeout_timer)
                    self.fetch_timeout_timer = None

                self.expected_items = expected_items
                self.transition_to(ScanState.RUN_YOLO_DETECTION)

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid expected inventory JSON: {e}')

    # =========================================================================
    # State Machine
    # =========================================================================

    def transition_to(self, new_state: ScanState):
        """Transition to a new state and execute entry action."""
        old_state = self.state
        self.state = new_state
        self.get_logger().info(f'State: {old_state.name} -> {new_state.name}')

        # Publish state change to backend
        self.publish_fsm_status()

        # Execute state entry action
        state_handlers = {
            ScanState.IDLE: self.on_enter_idle,
            ScanState.NAVIGATE_TO_SHELF: self.on_enter_navigate_to_shelf,
            ScanState.ALIGN_WITH_SHELF: self.on_enter_align_with_shelf,
            ScanState.FETCH_EXPECTED_INVENTORY: self.on_enter_fetch_expected,
            ScanState.RUN_YOLO_DETECTION: self.on_enter_yolo_detection,
            ScanState.COMPARE_INVENTORY: self.on_enter_compare_inventory,
            ScanState.SEND_RESULTS: self.on_enter_send_results,
            ScanState.RETURN_HOME: self.on_enter_return_home,
            ScanState.ERROR: self.on_enter_error,
        }

        handler = state_handlers.get(new_state)
        if handler:
            handler()

    # =========================================================================
    # State Entry Handlers
    # =========================================================================

    def on_enter_idle(self):
        """Entry action for IDLE state."""
        self.get_logger().info('Entered IDLE state')
        self.disable_all_triggers()

    def on_enter_navigate_to_shelf(self):
        """Entry action for NAVIGATE_TO_SHELF state."""
        if not self.shelf_queue:
            self.get_logger().info('No more shelves to scan')
            self.transition_to(ScanState.RETURN_HOME)
            return

        self.current_shelf_id = self.shelf_queue.pop(0)
        self.get_logger().info(f'Navigating to {self.current_shelf_id}')

        # Send goal to navigator
        msg = String()
        msg.data = self.current_shelf_id
        self.waypoint_goal_pub.publish(msg)

    def on_enter_align_with_shelf(self):
        """Entry action for ALIGN_WITH_SHELF state."""
        self.get_logger().info('Aligning with shelf')
        # TODO: Fine positioning logic
        # For now, proceed directly to fetch expected inventory
        self.transition_to(ScanState.FETCH_EXPECTED_INVENTORY)

    def on_enter_fetch_expected(self):
        """Entry action for FETCH_EXPECTED_INVENTORY state."""
        self.get_logger().info(f'Waiting for expected inventory for {self.current_shelf_id}')

        # Backend will see our state via /fsm_status subscription and send inventory
        # Start timeout timer (10 seconds)
        self.fetch_timeout_timer = self.create_timer(
            10.0, self.fetch_timeout_callback)

    def fetch_timeout_callback(self):
        """Handle timeout waiting for expected inventory."""
        self.get_logger().error('Timeout waiting for expected inventory from backend')

        if self.fetch_timeout_timer:
            self.fetch_timeout_timer.cancel()
            self.destroy_timer(self.fetch_timeout_timer)
            self.fetch_timeout_timer = None

        with self.lock:
            if self.state == ScanState.FETCH_EXPECTED_INVENTORY:
                self.transition_to(ScanState.ERROR)

    def on_enter_yolo_detection(self):
        """Entry action for RUN_YOLO_DETECTION state."""
        self.get_logger().info('Starting YOLO detection')
        msg = Bool()
        msg.data = True
        self.yolo_trigger_pub.publish(msg)

    def on_enter_compare_inventory(self):
        """Entry action for COMPARE_INVENTORY state."""
        self.get_logger().info('Comparing inventory')

        expected_set = set(self.expected_items)
        detected_set = set(self.detected_items)

        self.missing_items = list(expected_set - detected_set)
        self.unexpected_items = list(detected_set - expected_set)
        self.match = (len(self.missing_items) == 0 and len(self.unexpected_items) == 0)

        self.get_logger().info(f'Expected: {self.expected_items}')
        self.get_logger().info(f'Detected: {self.detected_items}')
        self.get_logger().info(f'Missing: {self.missing_items}')
        self.get_logger().info(f'Unexpected: {self.unexpected_items}')
        self.get_logger().info(f'Match: {self.match}')

        self.transition_to(ScanState.SEND_RESULTS)

    def on_enter_send_results(self):
        """Entry action for SEND_RESULTS state - publish to topic."""
        self.get_logger().info('Publishing results to backend')

        results = {
            "scan_id": self.current_scan_id,
            "shelf_id": self.current_shelf_id,
            "timestamp": datetime.now().isoformat(),
            "expected_items": self.expected_items,
            "detected_items": self.detected_items,
            "missing_items": self.missing_items,
            "unexpected_items": self.unexpected_items,
            "match": self.match
        }

        msg = String()
        msg.data = json.dumps(results)
        self.scan_results_pub.publish(msg)

        self.get_logger().info('Results published')

        # Check if more shelves to scan
        if self.shelf_queue:
            self.transition_to(ScanState.NAVIGATE_TO_SHELF)
        else:
            self.transition_to(ScanState.RETURN_HOME)

    def on_enter_return_home(self):
        """Entry action for RETURN_HOME state."""
        self.get_logger().info('Returning home')
        msg = String()
        msg.data = 'home'
        self.waypoint_goal_pub.publish(msg)

    def on_enter_error(self):
        """Entry action for ERROR state."""
        self.get_logger().error('Entered ERROR state')
        self.disable_all_triggers()

    # =========================================================================
    # Robot Sensor Callbacks
    # =========================================================================

    def nav_status_callback(self, msg):
        """Handle navigation status updates."""
        status = msg.data
        self.get_logger().info(f'Nav status: {status}')

        with self.lock:
            if self.state == ScanState.NAVIGATE_TO_SHELF:
                if status == 'reached':
                    self.transition_to(ScanState.ALIGN_WITH_SHELF)
                elif status == 'failed':
                    self.transition_to(ScanState.ERROR)

            elif self.state == ScanState.RETURN_HOME:
                if status == 'reached':
                    self.transition_to(ScanState.IDLE)

    def detections_callback(self, msg):
        """Handle YOLO detection results (comma-separated class names)."""
        self.get_logger().info(f'Detections received: {msg.data}')

        with self.lock:
            if self.state == ScanState.RUN_YOLO_DETECTION:
                # Stop YOLO detection
                trigger_msg = Bool()
                trigger_msg.data = False
                self.yolo_trigger_pub.publish(trigger_msg)

                # Parse comma-separated class names from yolo_detector_node
                if msg.data:
                    self.detected_items = [name.strip() for name in msg.data.split(',') if name.strip()]
                else:
                    self.detected_items = []

                self.get_logger().info(f'Detected items: {self.detected_items}')
                self.transition_to(ScanState.COMPARE_INVENTORY)

    # =========================================================================
    # Utility Methods
    # =========================================================================

    def reset_state(self):
        """Reset all state variables."""
        self.state = ScanState.IDLE
        self.current_scan_id = None
        self.current_shelf_id = None
        self.shelf_queue = []
        self.expected_items = []
        self.detected_items = []
        self.missing_items = []
        self.unexpected_items = []
        self.match = False

    def disable_all_triggers(self):
        """Disable all scanner triggers."""
        msg = Bool()
        msg.data = False
        self.yolo_trigger_pub.publish(msg)


def main():
    node = InventoryFSMNode('inventory_fsm')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
