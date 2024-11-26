#!/usr/bin/env python3

import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # ROS2 package to convert between ROS and OpenCV Images
import cv2  # Python OpenCV library
import numpy as np  # For array manipulation


def listener_callback(image_data):
    # Convert ROS Image message to OpenCV image
    cv_image = CvBridge().imgmsg_to_cv2(image_data, "bgr8")
    
    # Convert the image to HSV (Hue, Saturation, Value) color space
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    
    # Define the range of red color in HSV
    lower_red1 = np.array([0, 120, 70])  # Lower bound for red color
    upper_red1 = np.array([10, 255, 255])  # Upper bound for the first range of red
    
    lower_red2 = np.array([170, 120, 70])  # Lower bound for the second range of red
    upper_red2 = np.array([180, 255, 255])  # Upper bound for the second range of red
    
    # Create a mask for detecting red color
    mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(mask1, mask2)
    
    # Bitwise-AND the original image with the mask to extract the red parts
    red_detected_image = cv2.bitwise_and(cv_image, cv_image, mask=red_mask)
    
    # Find contours in the mask (red areas)
    contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Loop through the contours to draw bounding boxes around the red objects
    for contour in contours:
        if cv2.contourArea(contour) > 500:  # Filter out small contours
            # Get the bounding box around the red object
            x, y, w, h = cv2.boundingRect(contour)
            # Draw a green rectangle around the detected red object
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # Display the original image with the detected red objects
    cv2.imshow("camera", cv_image)
    
    # Wait for a key press and continue (to display image)
    cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    
    # Create the node
    node = Node('camera_node')
    # Log information into the console
    node.get_logger().info('Hello node')
    
    # Create the subscriber. This subscriber will receive an Image
    # from the image_raw topic. The queue size is 10 messages.
    subscription = node.create_subscription(
        Image, 'image_raw', listener_callback, 10)
    
    # Spin the node so the callback function is called.
    rclpy.spin(node)
    
    # Shutdown when finished
    rclpy.shutdown()


if __name__ == '__main__':
    main()
