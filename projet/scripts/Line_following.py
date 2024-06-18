#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage

bridge = CvBridge()
image = None


def image_callback(msg):
    global image
    image = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

def main():
    global image
    rospy.init_node('Line_following')
    rospy.Subscriber("/camera/image/compressed", CompressedImage, image_callback)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    while not rospy.is_shutdown():
        if image is not None:
            twist = Twist()
            # Convert image to HSV color space
            hsv= cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            # Get the height of the image
            height, width, _ = hsv.shape

            # Calculate the region of interest ROI
            top = int(height * 0.65)
            bottom = int(height * 0.9)
            left=int(width*0.2)
            right=int(width*0.85)

            # Crop the image to the ROI
            hsv = hsv[top:bottom, left:right]
           
            # Define the thresholds for the white color
            lower_white = np.array([0, 0, 200])
            upper_white = np.array([180, 30, 255])
      
            # Define the thresholds for the yellow color
            lower_yellow = np.array([22, 60, 60])
            upper_yellow = np.array([60, 255, 255]) 

            # masks for right and lest lines to keep only the white and yellow pixels
            mask_right=cv2.inRange(hsv, lower_white, upper_white)
            mask_left=cv2.inRange(hsv, lower_yellow, upper_yellow)

            # Compute moments of thresholded image
            M1 = cv2.moments(mask_right)
            M2 = cv2.moments(mask_left)
           
            
            # white and yellow lines are detected
            if M1["m00"] > 0 and M2["m00"]> 0:
                # Calculate x, y coordinate of center
                cX1 = int(M1["m10"] / M1["m00"])
                cY1 = int(M1["m01"] / M1["m00"])
                cX2 = int(M2["m10"] / M2["m00"])
                cY2 = int(M2["m01"] / M2["m00"])
                centre = (cX1 + cX2)/2
                error = centre - width/2
                
                #error below the threshold, the robot continues to move forward
                if abs(error) < 20:
                    twist.linear.x = 0.6
                    twist.angular.z = 0
                    pub.publish(twist)

                #the moments of the yellow is the highest, we see more the yellow line, turn left
                elif M2["m00"]>M1["m00"]:
                    twist.linear.x = 0.2
                    twist.angular.z = -1
                    pub.publish(twist)

                #  # the moments of the white is the highest, we see more the white line, turn right
                elif M1["m00"]>M2["m00"]:
                    twist.linear.x = 0.2
                    twist.angular.z = 1
                    pub.publish(twist)
                
                #error greater than the threshold, correction of the trajectory
                else:
                    twist.linear.x = 0.2
                    if error > width//4:
                        error = width//4
                    twist.angular.z = 0.5* error
                    pub.publish(twist)
                   
            #Only yellow line is detected, turn left       
            elif M2["m00"]> 0 and M1["m00"] == 0:
                twist.linear.x = 0.0
                twist.angular.z = -1
                pub.publish(twist)
               
            #Only white line is detected, turn right
            elif M1["m00"]> 0 and M2["m00"] == 0 :
                twist.linear.x = 0.0
                twist.angular.z = 1
                pub.publish(twist)
       
            #Nothing is detected, keep going
            else : 
                twist.linear.x = 0.7
                pub.publish(twist)
                
    rospy.spin()

if __name__ == '__main__':
    main()