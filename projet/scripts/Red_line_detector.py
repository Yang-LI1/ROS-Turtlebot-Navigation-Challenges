#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import cv_bridge
import numpy as np
from std_msgs.msg import Bool
from sensor_msgs.msg import CompressedImage


class RedLineDetector:
    """
    This class will detect the small red lines on the circuit en send it to the /red_line_detected topic 
    in order to decide which nodes to launch
    """
    def __init__(self):
        # Initialize the ROS node with name 'red_line_detector'
        rospy.init_node('red_line_detector')

        # Create a CvBridge object to convert ROS messages to OpenCV format
        self.bridge = cv_bridge.CvBridge()

        # Create a publisher to publish 'Bool' messages to '/red_line_detected' topic
        self.pub = rospy.Publisher('/red_line_detected', Bool, queue_size=1)
        
        # Create a subscriber to subscribe to '/camera/image/compressed' topic
        # and call 'image_callback' function every time a message is received
        self.sub = rospy.Subscriber('/camera/image/compressed', CompressedImage, self.image_callback)

    def image_callback(self, msg):
        try:
            # Convert the compressed ROS image message to OpenCV format
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except cv_bridge.CvBridgeError as e:
            rospy.logerr(e)

        # Convert the image to HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define the region of interest (ROI) to detect the red lines
        height, width, _ = hsv.shape
        top = int(height * 0.98)
        bottom = int(height * 1)
        left=int(width*0.55)
        right=int(width*0.6)
        hsv = hsv[top:bottom, left:right] # Crop the image to the ROI

        # Define the range of red colors in HSV color space
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        lower_red2 = np.array([170, 50, 50])
        upper_red2 = np.array([180, 255, 255])

        # Mask the image to keep only red pixels in the ROI
        mask1 = cv2.inRange(hsv, lower_red, upper_red)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2

        # Calculate the moments of the mask
        M= cv2.moments(mask)

        # If red pixels are detected, publish True on the "red_line_detected" topic
        if M["m00"] >0:
            self.pub.publish(True)
        
        else:
            self.pub.publish(False)

     

if __name__ == '__main__':
    try:
        RedLineDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
