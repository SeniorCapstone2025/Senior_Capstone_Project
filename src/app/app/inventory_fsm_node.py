#!/usr/bin/env python3
# encoding: utf-8
"""
Inventory FSM Node - Main controller for autonomous inventory scanning.

Coordinates the scan process through states:
IDLE -> NAVIGATE_TO_SHELF -> ALIGN_WITH_SHELF -> SCAN_QR ->
FETCH_EXPECTED_INVENTORY -> RUN_YOLO_DETECTION -> COMPARE_INVENTORY ->
SEND_RESULTS_TO_BACKEND -> RETURN_HOME -> IDLE
"""

import rclpy
import threading
import requests
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
    SCAN_QR = auto()
    FETCH_EXPECTED_INVENTORY = auto()
    RUN_YOLO_DETECTION = auto()
    COMPARE_INVENTORY = auto()
    SEND_RESULTS_TO_BACKEND = auto()
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
        self.shelf_queue = []  # List of shelves to scan
        self.expected_items = []
        self.detected_items = []

        # Backend config
        self.declare_parameter('backend_url', 'http://localhost:8000')
        self.declare_parameter('backend_timeout', 10.0)
        self.backend_url = self.get_parameter('backend_url').value
        self.backend_timeout = self.get_parameter('backend_timeout').value

        # Publishers
        self.waypoint_goal_pub = self.create_publisher(
            String, '/waypoint_navigator/goal', 10)
        self.qr_trigger_pub = self.create_publisher(
            Bool, '/qr_scanner/trigger', 10)
        self.yolo_trigger_pub = self.create_publisher(
            Bool, '/yolo_detector/trigger', 10)

        # Subscribers (created on enter)
        self.nav_status_sub = None
        self.qr_code_sub = None
        self.detections_sub = None

        # Lifecycle services
        self.create_service(Trigger, '~/enter', self.enter_srv_callback)
        self.create_service(Trigger, '~/exit', self.exit_srv_callback)
        self.create_service(Trigger, '~/init_finish', self.init_finish_callback)

        # Scan control services
        self.create_service(Trigger, '~/start_scan', self.start_scan_callback)
        self.create_service(Trigger, '~/stop_scan', self.stop_scan_callback)
        self.create_service(Trigger, '~/emergency_stop', self.emergency_stop_callback)

        # Heartbeat for dashboard connection
        Heart(self, f'{self.name}/heartbeat', 5,
              lambda _: self.exit_srv_callback(Trigger.Request(), Trigger.Response()))

        self.get_logger().info('\033[1;32m%s\033[0m' % 'Inventory FSM Node initialized')

    # =========================================================================
    # Lifecycle Services
    # =========================================================================

    def enter_srv_callback(self, request, response):
        """Activate the inventory scanner mode."""
        self.get_logger().info('\033[1;32m%s\033[0m' % 'inventory_fsm enter')

        with self.lock:
            self.reset_state()
            self.is_active = True

            # Subscribe to topics
            self.nav_status_sub = self.create_subscription(
                String, '/waypoint_navigator/status',
                self.nav_status_callback, 10)
            self.qr_code_sub = self.create_subscription(
                String, '/qr_scanner/qr_code',
                self.qr_code_callback, 10)
            # TODO: Subscribe to /yolo_detector/detections when message type is created

        response.success = True
        response.message = "enter"
        return response

    def exit_srv_callback(self, request, response):
        """Deactivate the inventory scanner mode."""
        self.get_logger().info('\033[1;32m%s\033[0m' % 'inventory_fsm exit')

        with self.lock:
            self.is_active = False
            self.reset_state()

            # Destroy subscriptions
            if self.nav_status_sub:
                self.destroy_subscription(self.nav_status_sub)
                self.nav_status_sub = None
            if self.qr_code_sub:
                self.destroy_subscription(self.qr_code_sub)
                self.qr_code_sub = None
            if self.detections_sub:
                self.destroy_subscription(self.detections_sub)
                self.detections_sub = None

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

    def start_scan_callback(self, request, response):
        """Start the inventory scan sequence."""
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start_scan requested')

        with self.lock:
            if not self.is_active:
                response.success = False
                response.message = "Node not active. Call ~/enter first."
                return response

            if self.state != ScanState.IDLE:
                response.success = False
                response.message = f"Cannot start scan. Current state: {self.state.name}"
                return response

            # TODO: Get shelf list from request or config
            self.shelf_queue = ['shelf_1']  # Default for testing
            self.transition_to(ScanState.NAVIGATE_TO_SHELF)

        response.success = True
        response.message = "Scan started"
        return response

    def stop_scan_callback(self, request, response):
        """Gracefully stop the current scan."""
        self.get_logger().info('\033[1;32m%s\033[0m' % 'stop_scan requested')

        with self.lock:
            self.transition_to(ScanState.RETURN_HOME)

        response.success = True
        response.message = "Stopping scan, returning home"
        return response

    def emergency_stop_callback(self, request, response):
        """Immediate halt of all operations."""
        self.get_logger().error('\033[1;31m%s\033[0m' % 'EMERGENCY STOP')

        with self.lock:
            self.state = ScanState.IDLE
            self.disable_all_triggers()
            # TODO: Send stop command to motors

        response.success = True
        response.message = "Emergency stop executed"
        return response

    # =========================================================================
    # State Machine
    # =========================================================================

    def transition_to(self, new_state: ScanState):
        """Transition to a new state and execute entry action."""
        old_state = self.state
        self.state = new_state
        self.get_logger().info(f'State: {old_state.name} -> {new_state.name}')

        # Execute state entry action
        state_handlers = {
            ScanState.IDLE: self.on_enter_idle,
            ScanState.NAVIGATE_TO_SHELF: self.on_enter_navigate_to_shelf,
            ScanState.ALIGN_WITH_SHELF: self.on_enter_align_with_shelf,
            ScanState.SCAN_QR: self.on_enter_scan_qr,
            ScanState.FETCH_EXPECTED_INVENTORY: self.on_enter_fetch_expected,
            ScanState.RUN_YOLO_DETECTION: self.on_enter_yolo_detection,
            ScanState.COMPARE_INVENTORY: self.on_enter_compare_inventory,
            ScanState.SEND_RESULTS_TO_BACKEND: self.on_enter_send_results,
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
        # For now, skip directly to QR scan
        self.transition_to(ScanState.SCAN_QR)

    def on_enter_scan_qr(self):
        """Entry action for SCAN_QR state."""
        self.get_logger().info('Starting QR scan')
        msg = Bool()
        msg.data = True
        self.qr_trigger_pub.publish(msg)

    def on_enter_fetch_expected(self):
        """Entry action for FETCH_EXPECTED_INVENTORY state."""
        self.get_logger().info(f'Fetching expected inventory for {self.current_shelf_id}')

        try:
            url = f"{self.backend_url}/api/inventory/shelf/{self.current_shelf_id}"
            resp = requests.get(url, timeout=self.backend_timeout)
            resp.raise_for_status()
            data = resp.json()
            self.expected_items = data.get('expected_items', [])
            self.get_logger().info(f'Expected items: {self.expected_items}')
            self.transition_to(ScanState.RUN_YOLO_DETECTION)
        except Exception as e:
            self.get_logger().error(f'Failed to fetch inventory: {e}')
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

        self.get_logger().info(f'Missing: {self.missing_items}')
        self.get_logger().info(f'Unexpected: {self.unexpected_items}')
        self.get_logger().info(f'Match: {self.match}')

        self.transition_to(ScanState.SEND_RESULTS_TO_BACKEND)

    def on_enter_send_results(self):
        """Entry action for SEND_RESULTS_TO_BACKEND state."""
        self.get_logger().info('Sending results to backend')

        try:
            url = f"{self.backend_url}/api/scan/results"
            payload = {
                'scan_id': self.current_scan_id,
                'shelf_id': self.current_shelf_id,
                'expected_items': self.expected_items,
                'detected_items': self.detected_items,
                'missing_items': self.missing_items,
                'unexpected_items': self.unexpected_items,
                'match': self.match,
            }
            resp = requests.post(url, json=payload, timeout=self.backend_timeout)
            resp.raise_for_status()
            self.get_logger().info('Results sent successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to send results: {e}')

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
        # TODO: Notify dashboard of error

    # =========================================================================
    # Topic Callbacks
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

    def qr_code_callback(self, msg):
        """Handle QR code detection."""
        qr_data = msg.data
        self.get_logger().info(f'QR detected: {qr_data}')

        with self.lock:
            if self.state == ScanState.SCAN_QR:
                # Stop QR scanning
                trigger_msg = Bool()
                trigger_msg.data = False
                self.qr_trigger_pub.publish(trigger_msg)

                # Verify QR matches expected shelf
                if qr_data == self.current_shelf_id:
                    self.transition_to(ScanState.FETCH_EXPECTED_INVENTORY)
                else:
                    self.get_logger().warn(f'QR mismatch: expected {self.current_shelf_id}, got {qr_data}')
                    self.current_shelf_id = qr_data  # Use actual QR
                    self.transition_to(ScanState.FETCH_EXPECTED_INVENTORY)

    def detections_callback(self, msg):
        """Handle YOLO detection results."""
        # TODO: Parse DetectionArray message
        self.get_logger().info('Detections received')

        with self.lock:
            if self.state == ScanState.RUN_YOLO_DETECTION:
                # Stop YOLO detection
                trigger_msg = Bool()
                trigger_msg.data = False
                self.yolo_trigger_pub.publish(trigger_msg)

                # Extract class names from detections
                # self.detected_items = [d.class_name for d in msg.detections]
                self.detected_items = []  # TODO: Parse actual message

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
        self.qr_trigger_pub.publish(msg)
        self.yolo_trigger_pub.publish(msg)


def main():
    node = InventoryFSMNode('inventory_fsm')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
