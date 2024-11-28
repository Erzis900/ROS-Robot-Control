#!/usr/bin/env python3

import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from geometry_msgs.msg import Twist  # Twist message type for controlling the robot
from cv_bridge import CvBridge  # ROS2 package to convert between ROS and OpenCV Images
import cv2  # Python OpenCV library
import numpy as np  # For array manipulation


class RedObjectFollower(Node):
    def __init__(self):
        super().__init__('red_object_follower')
        
        # Create the subscriber for the image_raw topic
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.listener_callback,
            10
        )
        
        # Create the publisher for the /cmd_vel topic to control the robot
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Initialize the CvBridge object for image conversion
        self.bridge = CvBridge()

    def listener_callback(self, image_data):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(image_data, "bgr8")
        
        # Get the image dimensions
        height, width, _ = cv_image.shape
        
        # Convert the image to HSV (Hue, Saturation, Value) color space
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Define the stricter range of red color in HSV
        # Narrowed down range for pure red detection, reducing false positives
        lower_red1 = np.array([0, 150, 100])  # Lower bound for red color (stricter)
        upper_red1 = np.array([10, 255, 255])  # Upper bound for the first range of red
        
        lower_red2 = np.array([170, 150, 100])  # Lower bound for the second range of red
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
                
                # Calculate the center of the bounding box
                center_x = x + w // 2
                center_y = y + h // 2
                
                # Determine which quadrant the center of the bounding box belongs to
                if center_x < width // 2 and center_y < height // 2:
                    print("First quadrant (Top-left) - Moving forward")
                    self.publish_cmd_vel(linear=0.5, angular=0.0)  # Move forward
                elif center_x >= width // 2 and center_y < height // 2:
                    print("Second quadrant (Top-right) - Moving backward")
                    self.publish_cmd_vel(linear=-0.5, angular=0.0)  # Move backward
                elif center_x < width // 2 and center_y >= height // 2:
                    print("Third quadrant (Bottom-left) - Rotating left")
                    self.publish_cmd_vel(linear=0.0, angular=0.5)  # Rotate left
                else:
                    print("Fourth quadrant (Bottom-right) - Rotating right")
                    self.publish_cmd_vel(linear=0.0, angular=-0.5)  # Rotate right
        
        # Display the original image with the detected red objects
        cv2.imshow("camera", cv_image)
        
        # Wait for a key press and continue (to display image)
        cv2.waitKey(1)

    def publish_cmd_vel(self, linear, angular):
        # Create a Twist message to control the robot
        twist = Twist()
        
        # Set the linear and angular velocity values
        twist.linear.x = linear
        twist.angular.z = angular
        
        # Publish the Twist message to /cmd_vel
        self.cmd_vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    
    # Create the RedObjectFollower node
    red_object_follower = RedObjectFollower()
    
    # Spin the node so the callback function is called.
    rclpy.spin(red_object_follower)
    
    # Shutdown when finished
    rclpy.shutdown()


if __name__ == '__main__':
    main()
