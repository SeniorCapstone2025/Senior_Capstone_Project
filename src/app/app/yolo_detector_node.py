#!/usr/bin/env python3
# encoding: utf-8
"""
YOLO Detector Node - Wrapper for inventory scanning integration.

Interfaces with the yolov11_ros2 node and provides:
- FSM-compatible lifecycle (enter/exit)
- Trigger-based detection for inventory scanning
- Aggregated detection results
"""

import rclpy
import threading
from rclpy.node import Node
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from interfaces.msg import ObjectsInfo
from app.common import Heart


class YoloDetectorNode(Node):
    def __init__(self, name='yolo_detector'):
        rclpy.init()
        super().__init__(name)

        self.name = name
        self.lock = threading.RLock()
        self.is_active = False
        self.is_detecting = False

        # Parameters
        self.declare_parameter('detection_duration', 3.0)
        self.declare_parameter('min_confidence', 0.5)
        self.detection_duration = self.get_parameter('detection_duration').value
        self.min_confidence = self.get_parameter('min_confidence').value

        # Detection results aggregation
        self.detected_objects = {}  # class_name -> count

        # Publishers
        self.detections_pub = self.create_publisher(
            String, '/yolo_detector/detections', 10)
        self.status_pub = self.create_publisher(
            String, '/yolo_detector/status', 10)

        # Subscribers (created on enter)
        self.trigger_sub = None
        self.yolo_results_sub = None

        # Service clients for yolov11_ros2 node
        self.yolo_start_client = self.create_client(Trigger, '/yolov11/start')
        self.yolo_stop_client = self.create_client(Trigger, '/yolov11/stop')

        # Lifecycle services
        self.create_service(Trigger, '~/enter', self.enter_srv_callback)
        self.create_service(Trigger, '~/exit', self.exit_srv_callback)
        self.create_service(Trigger, '~/init_finish', self.init_finish_callback)

        # Heartbeat
        Heart(self, f'{self.name}/heartbeat', 5,
              lambda _: self.exit_srv_callback(Trigger.Request(), Trigger.Response()))

        # Detection timer
        self.detection_timer = None

        self.get_logger().info('\033[1;32m%s\033[0m' % 'YOLO Detector Node initialized')

    # =========================================================================
    # Lifecycle Services
    # =========================================================================

    def enter_srv_callback(self, request, response):
        """Activate the YOLO detector wrapper."""
        self.get_logger().info('\033[1;32m%s\033[0m' % 'yolo_detector enter')

        # Check if yolov11 services are available
        if not self.yolo_start_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('yolov11_ros2 start service not available')
            response.success = False
            response.message = "yolov11_ros2 not available"
            return response

        with self.lock:
            self.is_active = True
            self.is_detecting = False

            # Subscribe to trigger
            self.trigger_sub = self.create_subscription(
                Bool, '/yolo_detector/trigger',
                self.trigger_callback, 10)

            # Subscribe to yolov11 results
            self.yolo_results_sub = self.create_subscription(
                ObjectsInfo, '/yolov11_ros2/object_detect',
                self.yolo_results_callback, 10)

        response.success = True
        response.message = "enter"
        return response

    def exit_srv_callback(self, request, response):
        """Deactivate the YOLO detector wrapper."""
        self.get_logger().info('\033[1;32m%s\033[0m' % 'yolo_detector exit')

        with self.lock:
            self.is_active = False
            self.stop_detection()

            # Destroy timer
            if self.detection_timer:
                self.detection_timer.cancel()
                self.destroy_timer(self.detection_timer)
                self.detection_timer = None

            # Destroy subscriptions
            if self.trigger_sub:
                self.destroy_subscription(self.trigger_sub)
                self.trigger_sub = None
            if self.yolo_results_sub:
                self.destroy_subscription(self.yolo_results_sub)
                self.yolo_results_sub = None

        response.success = True
        response.message = "exit"
        return response

    def init_finish_callback(self, request, response):
        """Health check."""
        response.success = True
        response.message = f"detecting: {self.is_detecting}"
        return response

    # =========================================================================
    # Topic Callbacks
    # =========================================================================

    def trigger_callback(self, msg):
        """Handle detection trigger from FSM."""
        with self.lock:
            if msg.data:
                self.start_detection()
            else:
                self.stop_detection()

    def yolo_results_callback(self, msg):
        """Aggregate YOLO detection results."""
        with self.lock:
            if not self.is_active or not self.is_detecting:
                return

            for obj in msg.objects:
                if obj.score >= self.min_confidence:
                    class_name = obj.class_name
                    if class_name in self.detected_objects:
                        self.detected_objects[class_name] += 1
                    else:
                        self.detected_objects[class_name] = 1

    # =========================================================================
    # Detection Control
    # =========================================================================

    def start_detection(self):
        """Start YOLO detection."""
        self.get_logger().info('Starting YOLO detection')

        with self.lock:
            self.is_detecting = True
            self.detected_objects = {}

        # Call yolov11 start service
        self.call_yolo_service(self.yolo_start_client)

        # Start timer to stop detection after duration
        if self.detection_timer:
            self.detection_timer.cancel()
            self.destroy_timer(self.detection_timer)
        self.detection_timer = self.create_timer(
            self.detection_duration, self.detection_complete_callback)

        self.publish_status('detecting')

    def stop_detection(self):
        """Stop YOLO detection."""
        self.get_logger().info('Stopping YOLO detection')

        with self.lock:
            self.is_detecting = False

        # Call yolov11 stop service
        self.call_yolo_service(self.yolo_stop_client)

        # Cancel timer
        if self.detection_timer:
            self.detection_timer.cancel()
            self.destroy_timer(self.detection_timer)
            self.detection_timer = None

    def detection_complete_callback(self):
        """Handle detection duration complete."""
        self.get_logger().info('Detection duration complete')

        # Stop the timer from repeating
        if self.detection_timer:
            self.detection_timer.cancel()
            self.destroy_timer(self.detection_timer)
            self.detection_timer = None

        # Stop yolov11
        self.call_yolo_service(self.yolo_stop_client)

        with self.lock:
            self.is_detecting = False

            # Get unique detected class names
            detected_classes = list(self.detected_objects.keys())

            self.get_logger().info(f'Detected objects: {self.detected_objects}')

        # Publish results as comma-separated class names
        msg = String()
        msg.data = ','.join(detected_classes)
        self.detections_pub.publish(msg)

        self.publish_status('complete')

    def call_yolo_service(self, client):
        """Call a yolov11 service asynchronously."""
        if not client.service_is_ready():
            self.get_logger().warn(f'Service {client.srv_name} not ready')
            return

        request = Trigger.Request()
        future = client.call_async(request)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        """Handle service response."""
        try:
            response = future.result()
            if not response.success:
                self.get_logger().warn(f'Service call failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call error: {e}')

    def publish_status(self, status):
        """Publish detector status."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main():
    node = YoloDetectorNode('yolo_detector')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
