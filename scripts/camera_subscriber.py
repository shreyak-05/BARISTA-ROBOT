import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2

class CameraController(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.camera_subscription = self.create_subscription(Image,'/camera/camera_sensor/image_raw',self.image_callback,10)


    def image_callback(self,msg):
        if self.detect_cups(msg):
            self.get_logger().info("Cup Detected. Ready For Pickup.")
        else:
            self.get_logger().info("Cup Not Detected. Waiting for Order.")


    
    def detect_cups(self,image):
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Blur the image to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Canny edge detection
        edges = cv2.Canny(blurred, 50, 150)

        # Find contours
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Check for valid contours
        for contour in contours:
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)

            # Adjust thresholds if necessary
            if area > 200 and 0.3 < (4 * np.pi * area) / (perimeter ** 2) < 2.0:
                return True  # Cup detected

        return False  # No cup detected