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
    """
    this callback function is called each time an image is received on the camera/image/compressed topic. 
    The image is decompressed, converted to cv2 and stored in the image variable
    """
    global image
    image = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

def main():
    """
    This function allows you to avoid obstacles on the road.
    We detect the two white and yellow lines of the road and also the obstacles which are red.
    The robot will avoid obstacles while staying on the road
    """
    global image
    rospy.init_node('Obstacle_avoidance')
    # Subscribe to the camera topic
    rospy.Subscriber("/camera/image/compressed", CompressedImage, image_callback)
    # Publish to the cmd_vel topic to control the robot's movements
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    while not rospy.is_shutdown():
        if image is not None:
            twist = Twist()
            # Convert the image to the HSV color space
            hsv= cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            # Get the height of the image
            height, width, _ = hsv.shape

            # Calculate the region of interest ROI
            top = int(height * 0.7)
            bottom = int(height * 1)
            left=int(width*0.17)
            right=int(width*0.9)

            # Crop the image to ROI
            hsv1 = hsv[top:bottom, left:right]

            # Define the thresholds for the white color
            lower_white = np.array([0, 0, 200])
            upper_white = np.array([180, 30, 255])
                        
            # Define the thresholds for the yellow color
            lower_yellow = np.array([22, 60, 60])
            upper_yellow = np.array([60, 255, 255]) 

            # Define the thresholds for the red color in the HSV color space
            lower_red = np.array([0, 50, 50])
            upper_red = np.array([10, 255, 255])
            lower_red2 = np.array([170, 50, 50])
            upper_red2 = np.array([180, 255, 255])

            # mask  to keep only the red pixels
            mask1 = cv2.inRange(hsv1, lower_red, upper_red)
            mask2 = cv2.inRange(hsv1, lower_red2, upper_red2)
            mask_red=cv2.bitwise_or(mask1,mask2)

            # masks for right and lest lines to keep only the white and yellow pixels
            mask_right=cv2.inRange(hsv1, lower_white, upper_white)
            mask_left=cv2.inRange(hsv1, lower_yellow, upper_yellow)

            # Combine the masks
            mask=cv2.bitwise_or(mask_right,mask_left)
            mask=cv2.bitwise_or(mask,mask_red)
           
            # Calculate the moments of the masks to get the center of the detected blobs
            M1 = cv2.moments(mask_right)
            M2 = cv2.moments(mask_left)
            M3= cv2.moments(mask_red)

            # white and yellow lines and red obstacles are detected
            if  M1["m00"] > 0 and M2["m00"] > 0 and M3["m00"] > 0:
                # Calculate x, y coordinate of center
                cX3 = int(M3["m10"] / M3["m00"])
                cY3 = int(M3["m01"] / M3["m00"])
                # the moments of the white is the highest (we see more the white line)
                if M1["m00"] > M3["m00"]and M1["m00"] > M2["m00"]:
                    twist.linear.x = 0.06
                    twist.angular.z = 0.25
                    pub.publish(twist)
     
                # the moments of the yellow is the highest (we see more the yellow line)
                elif M2["m00"] > M3["m00"] and M2["m00"] > M1["m00"] :
                    twist.linear.x = 0.06
                    twist.angular.z = -0.4
                    pub.publish(twist)
              
                # the moments of the red is the highest (we see more the red obstacles)
                else:
                    if cX3>width//3:
                        twist.linear.x = 0.09
                        twist.angular.z = 0.8
                        pub.publish(twist)
                     
                    else:
                        twist.linear.x = 0.09
                        twist.angular.z = -0.8
                        pub.publish(twist)
                
            # Only red obstacles are detected
            elif  M1["m00"] == 0 and M2["m00"] == 0 and M3["m00"] > 0:
                # Calculate x, y coordinate of center
                cX3 = int(M3["m10"] / M3["m00"])
                cY3 = int(M3["m01"] / M3["m00"])
                #if the obstacle is on the right, turn right
                if cX3>width//3:
                    twist.linear.x = 0.09
                    twist.angular.z = 0.8
                    pub.publish(twist)
                
                #if the obstacle is on the left, turn left
                else:
                    twist.linear.x = 0.09
                    twist.angular.z = -0.8
                    pub.publish(twist)

            # white and red obstacles are detected
            elif M1["m00"] > 0 and M3["m00"]> 0:
                # Calculate x, y coordinate of center
                cX3 = int(M3["m10"] / M3["m00"])
                cY3 = int(M3["m01"] / M3["m00"])
                # the moments of the white is the highest (we see more the white line)
                if M1["m00"] > M3["m00"]:
                    twist.linear.x = 0.06
                    twist.angular.z = 0.25
                    pub.publish(twist)
                 
                # we see more the red obstacles 
                elif M3["m00"] > M1["m00"]:
               
                    #if the obstacle is on the right, turn right
                    if cX3>width//3:
                        twist.linear.x = 0.09
                        twist.angular.z = 1
                        pub.publish(twist)

                    #if the obstacle is on the left, turn left
                    else:
                        twist.linear.x = 0.09
                        twist.angular.z = -1
                        pub.publish(twist)

             # yellow lines and red obstacles are detected
            elif M2["m00"] > 0 and M3["m00"]> 0:
                # Calculate x, y coordinate of center
                cX3 = int(M3["m10"] / M3["m00"])
                cY3 = int(M3["m01"] / M3["m00"])
                 # the moments of the yellow is the highest (we see more the yellow line)
                if M2["m00"] > M3["m00"]:
                    twist.linear.x = 0.06
                    twist.angular.z = -0.4
                    pub.publish(twist)
            
                # we see more the red obstacle
                elif M3["m00"] > M2["m00"]:
               
                    #if the obstacle is on the right, turn right
                    if cX3>width//4:
                        twist.linear.x = 0.09
                        twist.angular.z = 1
                        pub.publish(twist)
                    
                     #if the obstacle is on the left, turn left
                    else:
                        twist.linear.x = 0.09
                        twist.angular.z = -1
                        pub.publish(twist)

            # white and yellow lines are detected
            elif M1["m00"] > 0 and M2["m00"]> 0:
                # Calculate x, y coordinate of center
                cX1 = int(M1["m10"] / M1["m00"])
                cY1 = int(M1["m01"] / M1["m00"])
                cX2 = int(M2["m10"] / M2["m00"])
                cY2 = int(M2["m01"] / M2["m00"])
                centre = (cX1 + cX2)/2
                error = centre - width/2
        
                #error below the threshold, the robot continues to move forward
                if abs(error) < 30:
                    twist.linear.x = 0.5
                    twist.angular.z = 0
                    pub.publish(twist)
                 
                #we see more the yellow line
                elif M2["m00"]>M1["m00"]:
                    twist.linear.x = 0.06
                    twist.angular.z = -0.3
                    pub.publish(twist)
                    
                #we see more the white line
                elif M1["m00"]>M2["m00"]:
                    twist.linear.x = 0.06
                    twist.angular.z = 0.3
                    pub.publish(twist)
                #error greater than the threshold, correction of the trajectory
                else:
                    twist.linear.x = 0.06
                    if error > width//4:
                        error = width//4
                    twist.angular.z = 0.4* error
                    pub.publish(twist)

            #Only yellow line is detected, turn left
            elif M2["m00"]> 0 and  M1["m00"] == 0 and M3["m00"] == 0 :
                twist.linear.x = 0.06
                twist.angular.z = -0.3
                pub.publish(twist)

            #Only white line is detected, turn right
            elif M1["m00"]> 0 and M2["m00"] == 0 and M3["m00"] == 0 :
                twist.angular.z = 0.25
                pub.publish(twist)

            #Nothing is detected, keep going
            else : 
                twist.linear.x = 0.6
                pub.publish(twist)
              
    rospy.spin()

if __name__ == '__main__':
    main()