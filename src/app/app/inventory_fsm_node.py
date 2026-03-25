#!/usr/bin/env python3
# encoding: utf-8
"""
Inventory FSM Node - Main controller for autonomous inventory scanning.

Coordinates the scan process through states:
IDLE -> NAVIGATE_TO_SHELF -> ALIGN_WITH_SHELF -> FETCH_EXPECTED_INVENTORY ->
RUN_YOLO_DETECTION -> COMPARE_INVENTORY -> SEND_RESULTS -> RETURN_HOME -> IDLE

Uses Nav2 NavigateToPose action for navigation (v2).
Communicates with backend via rosbridge topics (no HTTP).
"""

import json
import math
import rclpy
import threading
import yaml
from datetime import datetime
from enum import Enum, auto
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
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

        # Navigation
        self._nav_goal_handle = None
        self.waypoints = {}

        # Load waypoints from parameter
        self.declare_parameter('waypoints_file', '')
        waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
        if waypoints_file:
            self._load_waypoints(waypoints_file)

        # Timeout for waiting on expected inventory
        self.fetch_timeout_timer = None

        # Nav2 action client
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Publishers - Detection control
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
        self.detections_sub = None

        # Service clients for entering/exiting yolo detector
        self.yolo_enter_client = self.create_client(
            Trigger, '/yolo_detector/enter')
        self.yolo_exit_client = self.create_client(
            Trigger, '/yolo_detector/exit')

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

        self.get_logger().info('\033[1;32m%s\033[0m' % 'Inventory FSM Node initialized (Nav2)')

    # =========================================================================
    # Waypoint Loading
    # =========================================================================

    def _load_waypoints(self, filepath: str):
        """Load waypoints from YAML file."""
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
            self.waypoints = data.get('waypoints', {})
            self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints from {filepath}')
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')
            self.waypoints = {}

    def _waypoint_to_pose(self, waypoint_name: str) -> PoseStamped:
        """Convert a named waypoint to a PoseStamped in the map frame."""
        wp = self.waypoints.get(waypoint_name)
        if wp is None:
            self.get_logger().error(f'Unknown waypoint: {waypoint_name}')
            return None

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(wp['x'])
        pose.pose.position.y = float(wp['y'])
        pose.pose.position.z = 0.0

        # Convert theta (yaw) to quaternion
        yaw = float(wp.get('theta', 0.0))
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        return pose

    # =========================================================================
    # Nav2 Action Client
    # =========================================================================

    def _send_nav_goal(self, waypoint_name: str):
        """Send a navigation goal to Nav2."""
        pose = self._waypoint_to_pose(waypoint_name)
        if pose is None:
            self.get_logger().error(f'Cannot navigate to unknown waypoint: {waypoint_name}')
            self.transition_to(ScanState.ERROR)
            return

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available')
            self.transition_to(ScanState.ERROR)
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info(
            f'Sending Nav2 goal: {waypoint_name} '
            f'({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})')

        send_goal_future = self._nav_client.send_goal_async(
            goal, feedback_callback=self._nav_feedback_callback)
        send_goal_future.add_done_callback(self._nav_goal_response_callback)

    def _nav_goal_response_callback(self, future):
        """Called when Nav2 accepts or rejects the goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 goal rejected')
            with self.lock:
                self.transition_to(ScanState.ERROR)
            return

        self.get_logger().info('Nav2 goal accepted')
        self._nav_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_callback)

    def _nav_feedback_callback(self, feedback_msg):
        """Called periodically with navigation progress."""
        feedback = feedback_msg.feedback
        remaining = feedback.distance_remaining
        self.get_logger().debug(f'Nav2 distance remaining: {remaining:.2f}m')

    def _nav_result_callback(self, future):
        """Called when Nav2 navigation finishes."""
        result = future.result()
        status = result.status

        with self.lock:
            self._nav_goal_handle = None

            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Nav2 goal reached')
                if self.state == ScanState.NAVIGATE_TO_SHELF:
                    self.transition_to(ScanState.ALIGN_WITH_SHELF)
                elif self.state == ScanState.RETURN_HOME:
                    self.transition_to(ScanState.IDLE)

            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().warn('Nav2 goal canceled')
                # Don't transition — cancellation is intentional (e.g. stop_scan)

            else:
                self.get_logger().error(f'Nav2 goal failed with status: {status}')
                if self.state in (ScanState.NAVIGATE_TO_SHELF, ScanState.RETURN_HOME):
                    self.transition_to(ScanState.ERROR)

    def _cancel_nav_goal(self):
        """Cancel any active Nav2 goal."""
        if self._nav_goal_handle is not None:
            self.get_logger().info('Canceling Nav2 goal')
            self._nav_goal_handle.cancel_goal_async()
            self._nav_goal_handle = None

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
            self._cancel_nav_goal()
            self.reset_state()

            # Cancel any pending timers
            if self.fetch_timeout_timer:
                self.fetch_timeout_timer.cancel()
                self.destroy_timer(self.fetch_timeout_timer)
                self.fetch_timeout_timer = None

            # Destroy subscriptions
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

            # Activate YOLO detector
            self.activate_dependent_nodes()

            self.transition_to(ScanState.NAVIGATE_TO_SHELF)

        response.success = True
        response.message = "Scan started"
        return response

    def stop_scan_callback(self, request, response):
        """Gracefully stop the current scan."""
        self.get_logger().info('\033[1;32m%s\033[0m' % 'stop_scan requested')

        with self.lock:
            self._cancel_nav_goal()
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
            self._cancel_nav_goal()
            if self.fetch_timeout_timer:
                self.fetch_timeout_timer.cancel()
                self.destroy_timer(self.fetch_timeout_timer)
                self.fetch_timeout_timer = None

            self.state = ScanState.IDLE
            self.deactivate_dependent_nodes()
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

                # Activate YOLO detector
                self.activate_dependent_nodes()

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

        # Execute state entry action FIRST so fields like shelf_id are set
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

        # Publish AFTER entry handler so shelf_id, scan_id, etc. are populated.
        # Only publish if the handler didn't already trigger another transition
        # (e.g. ALIGN_WITH_SHELF immediately transitions to FETCH_EXPECTED_INVENTORY).
        if self.state == new_state:
            self.publish_fsm_status()

    # =========================================================================
    # State Entry Handlers
    # =========================================================================

    def on_enter_idle(self):
        """Entry action for IDLE state."""
        self.get_logger().info('Entered IDLE state')
        self.deactivate_dependent_nodes()

    def on_enter_navigate_to_shelf(self):
        """Entry action for NAVIGATE_TO_SHELF state."""
        if not self.shelf_queue:
            self.get_logger().info('No more shelves to scan')
            self.transition_to(ScanState.RETURN_HOME)
            return

        self.current_shelf_id = self.shelf_queue.pop(0)
        self.get_logger().info(f'Navigating to {self.current_shelf_id}')

        # Send goal via Nav2
        self._send_nav_goal(self.current_shelf_id)

    def on_enter_align_with_shelf(self):
        """Entry action for ALIGN_WITH_SHELF state."""
        self.get_logger().info('Aligning with shelf')
        # Nav2 already handles final orientation via the goal pose quaternion.
        # Proceed directly to fetch expected inventory.
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

        expected_set = set([x.lower() for x in self.expected_items])
        detected_set = set([x.lower() for x in self.detected_items])

        self.missing_items = list(expected_set - detected_set)
        self.unexpected_items = list(detected_set - expected_set)
        self.match = expected_set == detected_set

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
        self._send_nav_goal('home')

    def on_enter_error(self):
        """Entry action for ERROR state."""
        self.get_logger().error('Entered ERROR state')
        self._cancel_nav_goal()
        self.deactivate_dependent_nodes()

    # =========================================================================
    # Robot Sensor Callbacks
    # =========================================================================

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

    def activate_dependent_nodes(self):
        """Enter yolo_detector node."""
        self.get_logger().info('Activating dependent nodes...')
        self._call_trigger_service(self.yolo_enter_client, 'yolo_detector/enter')

    def deactivate_dependent_nodes(self):
        """Exit yolo_detector node."""
        self.get_logger().info('Deactivating dependent nodes...')
        self.disable_all_triggers()
        self._call_trigger_service(self.yolo_exit_client, 'yolo_detector/exit')

    def _call_trigger_service(self, client, name: str):
        """Call a Trigger service asynchronously."""
        if not client.service_is_ready():
            self.get_logger().warn(f'Service {name} not available, skipping')
            return
        request = Trigger.Request()
        future = client.call_async(request)
        future.add_done_callback(
            lambda f: self._service_done_callback(f, name)
        )

    def _service_done_callback(self, future, name: str):
        """Handle service call result."""
        try:
            result = future.result()
            if result.success:
                self.get_logger().info(f'{name}: {result.message}')
            else:
                self.get_logger().warn(f'{name} failed: {result.message}')
        except Exception as e:
            self.get_logger().error(f'{name} service error: {e}')


def main():
    node = InventoryFSMNode('inventory_fsm')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
