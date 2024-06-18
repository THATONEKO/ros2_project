#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class DepthObjectDetector(Node):
    def __init__(self):
        super().__init__('depth_object_detector')
        self.subscription = self.create_subscription(
            Image,
            '/zed2_leftd_camera/depth/image_raw',
            self.depth_image_callback,
            10
        )
        self.bridge = CvBridge()

    def depth_image_callback(self, msg):
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Example: Thresholding to detect objects at a certain distance
        min_distance = 0.5  # Minimum distance in meters
        max_distance = 2.0  # Maximum distance in meters
        mask = np.logical_and(depth_image > min_distance, depth_image < max_distance).astype(np.uint8) * 255
        mask = np.uint8(mask)

        # Find contours of objects
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Check if objects are detected
        if len(contours) > 0:
            self.get_logger().info('Object detected!')

            # Draw bounding boxes around detected objects
            depth_image_colored = cv2.cvtColor(depth_image, cv2.COLOR_GRAY2BGR)
            for cnt in contours:
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(depth_image_colored, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Display the depth image with bounding boxes
            cv2.imshow('Depth Object Detection', depth_image_colored)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = DepthObjectDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
