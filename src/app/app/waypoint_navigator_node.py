#!/usr/bin/env python3
# encoding: utf-8
"""
Waypoint Navigator Node - Simple navigation to predefined waypoints.

Uses lidar for obstacle detection and basic geometry for navigation.
Publishes motor commands to reach waypoint coordinates.
"""

import os
import math
import yaml
import rclpy
import threading
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from app.common import Heart


class WaypointNavigatorNode(Node):
    def __init__(self, name='waypoint_navigator'):
        rclpy.init()
        super().__init__(name)

        self.name = name
        self.lock = threading.RLock()
        self.is_active = False

        # Navigation parameters
        self.declare_parameter('speed_linear', 0.2)
        self.declare_parameter('speed_angular', 0.5)
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('obstacle_distance', 0.3)
        self.declare_parameter('waypoints_file', '')

        self.speed_linear = self.get_parameter('speed_linear').value
        self.speed_angular = self.get_parameter('speed_angular').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.obstacle_distance = self.get_parameter('obstacle_distance').value

        # Load waypoints
        self.waypoints = {}
        waypoints_file = self.get_parameter('waypoints_file').value
        if waypoints_file and os.path.exists(waypoints_file):
            self.load_waypoints(waypoints_file)
        else:
            # Default waypoints for testing
            self.waypoints = {
                'home': {'x': 0.0, 'y': 0.0, 'theta': 0.0},
                'shelf_1': {'x': 1.0, 'y': 0.0, 'theta': 1.57},
                'shelf_2': {'x': 2.0, 'y': 0.0, 'theta': 1.57},
            }

        # Current state
        self.current_goal = None
        self.current_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.is_navigating = False
        self.is_blocked = False

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)
        self.status_pub = self.create_publisher(String, '/waypoint_navigator/status', 10)

        # Subscribers (created on enter)
        self.goal_sub = None
        self.lidar_sub = None
        self.odom_sub = None

        # Lifecycle services
        self.create_service(Trigger, '~/enter', self.enter_srv_callback)
        self.create_service(Trigger, '~/exit', self.exit_srv_callback)
        self.create_service(Trigger, '~/init_finish', self.init_finish_callback)

        # Heartbeat
        Heart(self, f'{self.name}/heartbeat', 5,
              lambda _: self.exit_srv_callback(Trigger.Request(), Trigger.Response()))

        # Navigation timer (created on enter)
        self.nav_timer = None

        self.get_logger().info('\033[1;32m%s\033[0m' % 'Waypoint Navigator Node initialized')

    def load_waypoints(self, filepath):
        """Load waypoints from YAML file."""
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
                self.waypoints = data.get('waypoints', {})
            self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')

    # =========================================================================
    # Lifecycle Services
    # =========================================================================

    def enter_srv_callback(self, request, response):
        """Activate the navigator."""
        self.get_logger().info('\033[1;32m%s\033[0m' % 'waypoint_navigator enter')

        with self.lock:
            self.is_active = True
            self.is_navigating = False

            # Subscribe to goal commands
            self.goal_sub = self.create_subscription(
                String, '/waypoint_navigator/goal',
                self.goal_callback, 10)

            # Subscribe to lidar for obstacle detection
            qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
            self.lidar_sub = self.create_subscription(
                LaserScan, '/scan_raw',
                self.lidar_callback, qos)

            # Subscribe to odometry for position
            self.odom_sub = self.create_subscription(
                Odometry, '/odom',
                self.odom_callback, 10)

            # Start navigation control loop
            self.nav_timer = self.create_timer(0.1, self.navigation_loop)

        response.success = True
        response.message = "enter"
        return response

    def exit_srv_callback(self, request, response):
        """Deactivate the navigator."""
        self.get_logger().info('\033[1;32m%s\033[0m' % 'waypoint_navigator exit')

        with self.lock:
            self.is_active = False
            self.is_navigating = False
            self.stop_robot()

            # Destroy timer
            if self.nav_timer:
                self.nav_timer.cancel()
                self.destroy_timer(self.nav_timer)
                self.nav_timer = None

            # Destroy subscriptions
            if self.goal_sub:
                self.destroy_subscription(self.goal_sub)
                self.goal_sub = None
            if self.lidar_sub:
                self.destroy_subscription(self.lidar_sub)
                self.lidar_sub = None
            if self.odom_sub:
                self.destroy_subscription(self.odom_sub)
                self.odom_sub = None

        response.success = True
        response.message = "exit"
        return response

    def init_finish_callback(self, request, response):
        """Health check."""
        response.success = True
        response.message = f"navigating: {self.is_navigating}"
        return response

    # =========================================================================
    # Topic Callbacks
    # =========================================================================

    def goal_callback(self, msg):
        """Handle new navigation goal."""
        waypoint_id = msg.data
        self.get_logger().info(f'Goal received: {waypoint_id}')

        with self.lock:
            if waypoint_id not in self.waypoints:
                self.get_logger().error(f'Unknown waypoint: {waypoint_id}')
                self.publish_status('failed')
                return

            self.current_goal = self.waypoints[waypoint_id]
            self.is_navigating = True
            self.is_blocked = False
            self.publish_status('navigating')

    def lidar_callback(self, msg):
        """Process lidar data for obstacle detection."""
        with self.lock:
            if not self.is_navigating:
                return

            # Check front sector for obstacles
            ranges = np.array(msg.ranges)

            # Get front 60 degrees (30 degrees each side)
            num_points = len(ranges)
            angle_per_point = (msg.angle_max - msg.angle_min) / num_points
            front_angle = math.radians(30)
            front_points = int(front_angle / angle_per_point)

            # Front sector indices (wrap around for lidar that starts at 0)
            front_ranges = np.concatenate([
                ranges[:front_points],
                ranges[-front_points:]
            ])

            # Filter out invalid readings
            valid_ranges = front_ranges[np.isfinite(front_ranges)]
            valid_ranges = valid_ranges[valid_ranges > 0.01]

            if len(valid_ranges) > 0:
                min_distance = np.min(valid_ranges)
                self.is_blocked = min_distance < self.obstacle_distance

    def odom_callback(self, msg):
        """Update current pose from odometry."""
        with self.lock:
            self.current_pose['x'] = msg.pose.pose.position.x
            self.current_pose['y'] = msg.pose.pose.position.y

            # Extract yaw from quaternion
            q = msg.pose.pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            self.current_pose['theta'] = math.atan2(siny_cosp, cosy_cosp)

    # =========================================================================
    # Navigation Logic
    # =========================================================================

    def navigation_loop(self):
        """Main navigation control loop - runs at 10Hz."""
        with self.lock:
            if not self.is_active or not self.is_navigating or not self.current_goal:
                return

            # Check if blocked
            if self.is_blocked:
                self.stop_robot()
                self.publish_status('blocked')
                return

            # Calculate distance and angle to goal
            dx = self.current_goal['x'] - self.current_pose['x']
            dy = self.current_goal['y'] - self.current_pose['y']
            distance = math.sqrt(dx*dx + dy*dy)
            angle_to_goal = math.atan2(dy, dx)
            angle_error = self.normalize_angle(angle_to_goal - self.current_pose['theta'])

            # Check if reached
            if distance < self.goal_tolerance:
                self.stop_robot()
                self.is_navigating = False
                self.get_logger().info('Goal reached!')
                self.publish_status('reached')
                return

            # Generate velocity command
            twist = Twist()

            # Turn first if angle error is large
            if abs(angle_error) > math.radians(15):
                twist.angular.z = self.speed_angular if angle_error > 0 else -self.speed_angular
                twist.linear.x = 0.0
            else:
                # Move forward with proportional angular correction
                twist.linear.x = self.speed_linear
                twist.angular.z = 2.0 * angle_error  # P controller for heading

                # Limit angular velocity
                twist.angular.z = max(-self.speed_angular, min(self.speed_angular, twist.angular.z))

            self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        """Stop all robot motion."""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def publish_status(self, status):
        """Publish navigation status."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.get_logger().info(f'Status: {status}')

    @staticmethod
    def normalize_angle(angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main():
    node = WaypointNavigatorNode('waypoint_navigator')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
