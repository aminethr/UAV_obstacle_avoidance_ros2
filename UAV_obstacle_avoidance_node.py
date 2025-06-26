#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy
import cv2
from cv_bridge import CvBridge
import joblib

class ObstacleAvoidanceDirectNode(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance_direct_node')

        self.bridge = CvBridge()

        self.model = joblib.load("xgboost_stopping_distance_model.joblib")

        # NOTE: The following topic names depend on the topic configuration
        # defined in the drone's SDF or URDF model. Make sure they match
        # the actual topics published by your simulation setup or drone.

        self.depth_sub = self.create_subscription(
            Image, 
            '/drone0_camera1_sensor/depth/image_raw',  # Depth image topic from drone camera
            self.depth_callback, 
            10
        )

        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.vel_sub = self.create_subscription(
            TwistStamped,
            '/drone0/local_position/velocity_local',  # Velocity topic from drone localization
            self.velocity_callback,
            qos_profile
        )

        self.debug_pub = self.create_publisher(
            Image, 
            '/debug/depth_regions',  # Debug image topic for visualization
            10
        )

        self.vel_pub = self.create_publisher(
            Twist,
            '/drone0/setpoint_velocity/cmd_vel_unstamped',  # Command velocity topic for controlling the drone
            10
        )


        self.K = 1.5
        self.max_x_speed = 1.0
        self.max_z_speed = 0.5
        self.deadzone = 0.1
        self.align_threshold = 0.15

        self.current_speed = 2.0  # drone speed
        self.safety_margin = 5.0  # meters

    def velocity_callback(self, msg):
        self.current_speed = abs(msg.twist.linear.y)

    def depth_callback(self, msg):
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        depth_array = np.array(depth_image, dtype=np.float32)
        depth_array = np.nan_to_num(depth_array, nan=0.0, posinf=0.0, neginf=0.0)

        display_img = cv2.normalize(depth_array, None, 0, 255, cv2.NORM_MINMAX)
        display_img = np.uint8(display_img)
        display_img = cv2.cvtColor(display_img, cv2.COLOR_GRAY2BGR)

        height, width = depth_array.shape
        region_height = height // 4
        region_width = width // 4

        regions = []
        means = []

        for row in range(4):
            for col in range(4):
                x1 = col * region_width
                x2 = (col + 1) * region_width
                y1 = row * region_height
                y2 = (row + 1) * region_height

                region = depth_array[y1:y2, x1:x2]
                valid = region[region > 0.1]
                mean_val = np.mean(valid) if valid.size > 0 else 0.0
                regions.append((row, col, x1, y1, x2, y2))
                means.append(mean_val)

        center_x = width // 2
        center_y = height // 2
        center_distance = depth_array[center_y, center_x]

        cv2.circle(display_img, (center_x, center_y), 5, (255, 255, 0), -1)
        cv2.putText(display_img, f"Center: {center_distance:.1f}m", (10, height - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        twist = Twist()

        if self.current_speed > 0:
            predicted_stop_distance = self.model.predict(np.array([[self.current_speed]]))[0]
        else:
            predicted_stop_distance = 1.0

        safe_stop_distance = predicted_stop_distance + self.safety_margin

        self.get_logger().info(
            f"Speed: {self.current_speed:.2f} m/s | Model stopping distance: {predicted_stop_distance:.2f} m | Safe stopping distance (+5m): {safe_stop_distance:.2f} m | Center distance: {center_distance:.2f} m"
        )

        if center_distance > safe_stop_distance:
            twist.linear.y = 2.0
            self.vel_pub.publish(twist)
            self.get_logger().info("Moving forward...")
        else:
            best_idx = np.argmax(means)
            row, col, x1, y1, x2, y2 = regions[best_idx]
            region_center_x = int((col + 0.5) * region_width)
            region_center_y = int((row + 0.5) * region_height)
            box_depth = depth_array[region_center_y, region_center_x]

            norm_x = (region_center_x - center_x) / center_x
            norm_y = (center_y - region_center_y) / center_y

            if abs(norm_x) < self.deadzone:
                norm_x = 0.0
            if abs(norm_y) < self.deadzone:
                norm_y = 0.0

            self.get_logger().info(f"Aligning to box: norm_x={norm_x:.2f}, norm_y={norm_y:.2f}, box_depth={box_depth:.1f}m")

            twist.linear.x = np.clip(self.K * norm_x, -self.max_x_speed, self.max_x_speed)
            twist.linear.z = np.clip(self.K * norm_y, -self.max_z_speed, self.max_z_speed)
            self.vel_pub.publish(twist)

            # === Draw selected region on debug image ===
            cv2.rectangle(display_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(display_img, f"Target: {means[best_idx]:.1f}m", (x1+5, y1+20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Publish the updated debug image
        debug_msg = self.bridge.cv2_to_imgmsg(display_img, encoding='bgr8')
        self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceDirectNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

