#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge

class AutoCannyNode(Node):
    def __init__(self):
        super().__init__("auto_canny_node")
        self.bridge = CvBridge()

        # Subscriber to the fisheye left camera topic
        self.subscription = self.create_subscription(
            Image,
            "/rs_t265/fisheye_left",  # Change to "/rs_t265/fisheye_right" for right camera
            self.image_callback,
            10
        )

        # Publisher for the processed image
        self.publisher = self.create_publisher(Image, "/rs_t265/canny_edge_detection", 10)

        self.get_logger().info("Auto-Canny edge detection node started!")

    def auto_canny(self, image, sigma=0.4):
        """Applies Bilateral Filter before adaptive Canny edge detection."""
        # Apply Bilateral Filter (preserves edges better than Gaussian Blur)
        filtered = cv2.bilateralFilter(image, d=9, sigmaColor=75, sigmaSpace=75)

        # Compute median pixel intensity
        median = np.median(filtered)

        # Compute lower and upper thresholds
        lower = int(max(0, (1.0 - sigma) * median))
        upper = int(min(255, (1.0 + sigma) * median))

        # Apply Canny Edge Detection
        edges = cv2.Canny(filtered, lower, upper)
        return edges

    def image_callback(self, msg):
        """Callback function that receives the fisheye image, processes it, and republishes it."""
        try:
            # Convert ROS2 image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")

            # Apply auto_canny edge detection
            processed_image = self.auto_canny(cv_image)
            
            # Convert back to ROS2 Image message
            processed_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding="mono8")
            processed_msg.header = msg.header  # Preserve original timestamp and frame_id

            # Publish the processed image
            self.publisher.publish(processed_msg)
            self.get_logger().info("Published processed fisheye image with auto Canny")

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = AutoCannyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
