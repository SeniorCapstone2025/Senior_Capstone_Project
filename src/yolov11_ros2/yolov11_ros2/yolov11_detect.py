#!/usr/bin/env python3
# encoding: utf-8
"""
YOLOv11 ROS 2 Detection Node

Uses ultralytics YOLOv11 nano with COCO pretrained weights.
"""

import os
import cv2
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import Trigger

from ultralytics import YOLO

import yolov11_ros2.fps as fps
from interfaces.msg import ObjectInfo, ObjectsInfo

ros_distribution = os.environ.get("ROS_DISTRO")
package_share_directory = get_package_share_directory('yolov11_ros2')


class YoloV11Ros2(Node):
    def __init__(self):
        super().__init__('yolov11_ros2')
        self.get_logger().info(f"Current ROS 2 distribution: {ros_distribution}")
        self.fps = fps.FPS()
        self.start = False

        # Declare parameters
        self.declare_parameter("device", "cpu", ParameterDescriptor(
            name="device", description="Compute device: cpu, cuda, or cuda:0"))

        self.declare_parameter("model", "yolo11n.pt", ParameterDescriptor(
            name="model", description="Model file name (default: yolo11n.pt)"))

        self.declare_parameter("image_topic", "/ascamera/camera_publisher/rgb0/image", ParameterDescriptor(
            name="image_topic", description="Image topic to subscribe to"))

        self.declare_parameter("confidence", 0.35, ParameterDescriptor(
            name="confidence", description="Minimum confidence threshold (0.0-1.0)"))

        self.declare_parameter("show_result", False, ParameterDescriptor(
            name="show_result", description="Display detection results in window"))

        self.declare_parameter("pub_result_img", False, ParameterDescriptor(
            name="pub_result_img", description="Publish annotated result images"))

        # Create services
        self.create_service(Trigger, '/yolov11/start', self.start_srv_callback)
        self.create_service(Trigger, '/yolov11/stop', self.stop_srv_callback)
        self.create_service(Trigger, '~/init_finish', self.get_node_state)

        # Load model
        model_name = self.get_parameter('model').value
        model_path = os.path.join(package_share_directory, "config", model_name)

        # If model not in config folder, ultralytics will auto-download
        if not os.path.exists(model_path):
            model_path = model_name
            self.get_logger().info(f"Model not in config folder, using: {model_path}")

        device = self.get_parameter('device').value
        self.confidence = self.get_parameter('confidence').value

        self.get_logger().info(f"Loading YOLOv11 model: {model_path} on {device}")
        self.model = YOLO(model_path)
        self.model.to(device)

        # Publishers
        self.object_pub = self.create_publisher(ObjectsInfo, '~/object_detect', 10)
        self.result_img_pub = self.create_publisher(Image, "result_img", 10)

        # Subscriber
        image_topic = self.get_parameter('image_topic').value
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10)

        # CV Bridge
        self.bridge = CvBridge()

        self.show_result = self.get_parameter('show_result').value
        self.pub_result_img = self.get_parameter('pub_result_img').value

        self.get_logger().info('\033[1;32m%s\033[0m' % 'YOLOv11 node initialized')

    def get_node_state(self, request, response):
        """Health check service."""
        response.success = True
        response.message = f"running: {self.start}"
        return response

    def start_srv_callback(self, request, response):
        """Start detection service."""
        self.get_logger().info('\033[1;32m%s\033[0m' % "start yolov11 detect")
        self.start = True
        response.success = True
        response.message = "start"
        return response

    def stop_srv_callback(self, request, response):
        """Stop detection service."""
        self.get_logger().info('\033[1;32m%s\033[0m' % "stop yolov11 detect")
        self.start = False
        response.success = True
        response.message = "stop"
        return response

    def image_callback(self, msg: Image):
        """Process incoming images."""
        if not self.start:
            return

        # Convert ROS image to OpenCV
        image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        h, w = image.shape[:2]

        # Run inference
        results = self.model.predict(
            image,
            conf=self.confidence,
            verbose=False
        )

        objects_info = []

        # Process results
        for result in results:
            boxes = result.boxes
            if boxes is None:
                continue

            for i in range(len(boxes)):
                # Get box coordinates
                x1, y1, x2, y2 = boxes.xyxy[i].cpu().numpy().astype(int)
                conf = float(boxes.conf[i].cpu().numpy())
                cls_id = int(boxes.cls[i].cpu().numpy())
                class_name = result.names[cls_id]

                # Create ObjectInfo message
                object_info = ObjectInfo()
                object_info.class_name = class_name
                object_info.box = [int(x1), int(y1), int(x2), int(y2)]
                object_info.score = round(conf, 2)
                object_info.width = w
                object_info.height = h
                objects_info.append(object_info)

                # Draw on image if needed
                if self.show_result or self.pub_result_img:
                    cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    label = f"{class_name}: {conf:.2f}"
                    cv2.putText(image, label, (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Publish detections
        if objects_info:
            object_msg = ObjectsInfo()
            object_msg.objects = objects_info
            self.object_pub.publish(object_msg)

        # Show result window
        if self.show_result:
            self.fps.update()
            image = self.fps.show_fps(image)
            cv2.imshow('YOLOv11 Result', cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
            cv2.waitKey(1)

        # Publish result image
        if self.pub_result_img:
            result_img_msg = self.bridge.cv2_to_imgmsg(image, encoding="rgb8")
            result_img_msg.header = msg.header
            self.result_img_pub.publish(result_img_msg)


def main():
    rclpy.init()
    node = YoloV11Ros2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.show_result:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
