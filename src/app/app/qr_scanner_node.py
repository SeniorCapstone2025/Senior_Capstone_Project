#!/usr/bin/env python3
# encoding: utf-8
"""
QR Scanner Node - Detects and decodes QR codes from camera feed.

Uses pyzbar library for QR code detection.
Only processes images when triggered by FSM.
"""

import rclpy
import threading
import cv2
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
from app.common import Heart

# Try to import pyzbar, handle if not available
try:
    from pyzbar import pyzbar
    PYZBAR_AVAILABLE = True
except ImportError:
    PYZBAR_AVAILABLE = False


class QRScannerNode(Node):
    def __init__(self, name='qr_scanner'):
        rclpy.init()
        super().__init__(name)

        self.name = name
        self.lock = threading.RLock()
        self.is_active = False
        self.is_scanning = False

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # Parameters
        self.declare_parameter('timeout', 10.0)
        self.declare_parameter('camera_topic', '/ascamera/camera_publisher/rgb0/image')
        self.timeout = self.get_parameter('timeout').value
        self.camera_topic = self.get_parameter('camera_topic').value

        # Publishers
        self.qr_code_pub = self.create_publisher(String, '/qr_scanner/qr_code', 10)
        self.status_pub = self.create_publisher(String, '/qr_scanner/status', 10)

        # Subscribers (created on enter)
        self.trigger_sub = None
        self.image_sub = None

        # Lifecycle services
        self.create_service(Trigger, '~/enter', self.enter_srv_callback)
        self.create_service(Trigger, '~/exit', self.exit_srv_callback)
        self.create_service(Trigger, '~/init_finish', self.init_finish_callback)

        # Heartbeat
        Heart(self, f'{self.name}/heartbeat', 5,
              lambda _: self.exit_srv_callback(Trigger.Request(), Trigger.Response()))

        # Timeout timer
        self.timeout_timer = None

        if not PYZBAR_AVAILABLE:
            self.get_logger().error('pyzbar not installed! QR scanning will not work.')

        self.get_logger().info('\033[1;32m%s\033[0m' % 'QR Scanner Node initialized')

    # =========================================================================
    # Lifecycle Services
    # =========================================================================

    def enter_srv_callback(self, request, response):
        """Activate the QR scanner."""
        self.get_logger().info('\033[1;32m%s\033[0m' % 'qr_scanner enter')

        if not PYZBAR_AVAILABLE:
            response.success = False
            response.message = "pyzbar not installed"
            return response

        with self.lock:
            self.is_active = True
            self.is_scanning = False

            # Subscribe to trigger
            self.trigger_sub = self.create_subscription(
                Bool, '/qr_scanner/trigger',
                self.trigger_callback, 10)

            # Subscribe to camera
            self.image_sub = self.create_subscription(
                Image, self.camera_topic,
                self.image_callback, 10)

        response.success = True
        response.message = "enter"
        return response

    def exit_srv_callback(self, request, response):
        """Deactivate the QR scanner."""
        self.get_logger().info('\033[1;32m%s\033[0m' % 'qr_scanner exit')

        with self.lock:
            self.is_active = False
            self.is_scanning = False

            # Cancel timeout timer
            if self.timeout_timer:
                self.timeout_timer.cancel()
                self.destroy_timer(self.timeout_timer)
                self.timeout_timer = None

            # Destroy subscriptions
            if self.trigger_sub:
                self.destroy_subscription(self.trigger_sub)
                self.trigger_sub = None
            if self.image_sub:
                self.destroy_subscription(self.image_sub)
                self.image_sub = None

        response.success = True
        response.message = "exit"
        return response

    def init_finish_callback(self, request, response):
        """Health check."""
        if not PYZBAR_AVAILABLE:
            response.success = False
            response.message = "pyzbar not installed"
        else:
            response.success = True
            response.message = f"scanning: {self.is_scanning}"
        return response

    # =========================================================================
    # Topic Callbacks
    # =========================================================================

    def trigger_callback(self, msg):
        """Handle scan trigger from FSM."""
        with self.lock:
            if msg.data:
                self.get_logger().info('QR scanning enabled')
                self.is_scanning = True

                # Start timeout timer
                if self.timeout_timer:
                    self.timeout_timer.cancel()
                    self.destroy_timer(self.timeout_timer)
                self.timeout_timer = self.create_timer(self.timeout, self.timeout_callback)
            else:
                self.get_logger().info('QR scanning disabled')
                self.is_scanning = False
                if self.timeout_timer:
                    self.timeout_timer.cancel()
                    self.destroy_timer(self.timeout_timer)
                    self.timeout_timer = None

    def image_callback(self, msg):
        """Process camera images for QR codes."""
        with self.lock:
            if not self.is_active or not self.is_scanning:
                return

        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Detect QR codes
            qr_data = self.detect_qr_code(cv_image)

            # If QR detected, publish and stop scanning
            if qr_data:
                with self.lock:
                    self.is_scanning = False
                    if self.timeout_timer:
                        self.timeout_timer.cancel()
                        self.destroy_timer(self.timeout_timer)
                        self.timeout_timer = None

                qr_msg = String()
                qr_msg.data = qr_data
                self.qr_code_pub.publish(qr_msg)
                self.publish_status('detected')
                self.get_logger().info(f'QR detected: {qr_data}')

        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')

    def timeout_callback(self):
        """Handle scan timeout."""
        self.get_logger().warn('QR scan timeout')

        with self.lock:
            self.is_scanning = False
            if self.timeout_timer:
                self.timeout_timer.cancel()
                self.destroy_timer(self.timeout_timer)
                self.timeout_timer = None

        # Publish empty string to indicate timeout/failure
        msg = String()
        msg.data = ''
        self.qr_code_pub.publish(msg)
        self.publish_status('timeout')

    # =========================================================================
    # QR Detection
    # =========================================================================

    def detect_qr_code(self, image):
        """
        Detect and decode QR codes in the image.

        Args:
            image: OpenCV BGR image

        Returns:
            str: QR code data, or None if not found
        """
        # Convert to grayscale for better detection
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Detect QR codes
        decoded_objects = pyzbar.decode(gray)

        for obj in decoded_objects:
            # Return first valid QR code data
            return obj.data.decode('utf-8')

        return None

    def publish_status(self, status):
        """Publish scanner status."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main():
    node = QRScannerNode('qr_scanner')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
