#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""

"""

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def shrehold(var,limit):
    """
    Apply a threshold to a variable.

    Args:
        var (float): The variable to apply the threshold to.
        limit (float): The threshold limit.

    Returns:
        float: The thresholded value.
        
    This is used to limit the vitesse 
    """
    if var < -limit : 
        return -limit
    elif var > limit:
        return limit
    else:
        return var



def image_callback(msg):
    """
    Callback function for the image subscriber.

    Args:
        msg (sensor_msgs.msg.Image): The received image message. Store the image data in a global variable
    """
    global image
    image = bridge.imgmsg_to_cv2(msg, "bgr8")
    
def laser_callback(msg):
    """
    Callback function for the laser scan subscriber. Store the laser scan data in a global variable

    Args:
        msg (sensor_msgs.msg.LaserScan): The received laser scan message.
    """
    global scan_msg
    scan_msg = msg.ranges






bridge = CvBridge()
image = None
def main():
    # Initialize global variables
    global image
    global scan_msg
    scan_msg = []
    # Initialize the ROS node
    rospy.init_node('image_processor')
    # Subscribe to the LaserScan and camera image topics
    rospy.Subscriber('/scan', LaserScan,laser_callback)
    rospy.Subscriber("/camera/image", Image, image_callback)
    
    # Publish velocity control commands
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    twist = Twist()
    while not rospy.is_shutdown():
        # If there is an obstacle detected by the laser scanner
        if any(i < 0.2 for i in np.array(scan_msg[0:90]+scan_msg[-90:])):
            distance_threshold = 0.2
            angular_speed = 0.5
            
            
            #right_scan refers to the laser scan data in the right front direction of the robot.
	    #left_scan refers to the laser scan data in the left front direction of the robot.
	    #rr refers to the laser scan data in the right side direction of the robot.
	    #ll refers to the laser scan data in the left side direction of the robot.
            right_scan = np.array(scan_msg[20:50])
            left_scan = np.array(scan_msg[-50:-20])  
            ll = np.array(scan_msg[-90:-60]) 
            rr = np.array(scan_msg[60:90]) 
            
            #Filtering scan data to remove non-finite values
            right_scan = right_scan[np.isfinite(right_scan)]
            left_scan = left_scan[np.isfinite(left_scan)]
            face_scan = np.array(scan_msg[0:20]+scan_msg[-20:])
            face_scan = face_scan[np.isfinite(face_scan)]
        	
        	
        	
            # Calculate the mean distance of the obstacle in front of the robot 
            facemean = np.mean(face_scan)
            # If there is an obstacle in front of the robot, move backward and rotate
            if facemean < 0.2:
                twist.linear.x = -0.08
                twist.angular.z = 0.6
                pub.publish(twist)
        
            # If there is not any obstacle in left or right or front of the robot, move forward
            elif len(right_scan)==0 or len(left_scan) == 0:
                twist.linear.x = 0.08
                twist.angular.z = 0
                pub.publish(twist)
                
            # Calculate the average distances on the left and right sides
            elif len(right_scan)!=0 or len(left_scan) != 0 :
                right_distance = np.mean(right_scan)
                left_distance = np.mean(left_scan)
                rr_distance = np.mean(rr)
                ll_distance = np.mean(ll)
                
                   
                #If there is an obstacle in left or right side of the robot, stay away from obstacles
                if  rr_distance  < distance_threshold or ll_distance  < distance_threshold:
                    if rr_distance  <  ll_distance :
                        twist.linear.x = 0.01
                        twist.angular.z = -angular_speed
                        pub.publish(twist)
                    elif ll_distance  < rr_distance:
                        twist.linear.x = 0.01
                        twist.angular.z = angular_speed
                        pub.publish(twist)
                    else: 
                        twist.linear.x = 0.08
                        pub.publish(twist)
                elif right_distance  < distance_threshold or left_distance  < distance_threshold:
                    if right_distance  <  left_distance :
                        twist.linear.x = 0.01
                        twist.angular.z = -angular_speed
                        pub.publish(twist)
                    elif left_distance  < right_distance:
                        twist.linear.x = 0.01
                        twist.angular.z = angular_speed
                        pub.publish(twist)
                else: 
                    twist.linear.x = 0.08
                    pub.publish(twist)
            else: 
                    
                    twist.linear.x = 0.08
                    pub.publish(twist)
        else:
            if image is not None:
               
                # Convert image to HSV color space
                hsv_tout = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                # #Get the height of the image
                height, width, _ = hsv_tout.shape
                
                
                # the region of interest
                top = int(height * 0.85)
                bottom = int(height * 1)
                left = int(height * 0.3)
                right = int(height * 0.7)
                hsv = hsv_tout[top:bottom,left:right]
                
                # Define the color ranges for blue and green
                lower_blue = np.array([100, 50, 50])
                upper_blue = np.array([130, 255, 255])
            
                lower_green = np.array([36, 25, 25])
                upper_green = np.array([86, 255, 255])
                
                # Create masks for green and blue colors
                mask_green = cv2.inRange(hsv_tout, lower_green, upper_green)#this mask detect green, it is used to find the destination
                mask_blue = cv2.inRange(hsv, lower_blue, upper_blue) # this mask detect blue in the region of interest, it is used to find the line blue on the ground  
                mask_blue_tout = cv2.inRange(hsv_tout, lower_blue, upper_blue) #this mask detect blue in the entire image
                mask_tout = cv2.bitwise_or(mask_green, mask_blue_tout)#this mask detect blue and green in the entire image. When there isn't blue and green, the robot has reached the destination.
                
                
                # Compute moments to calculate centroid of the blue and green regions
                M1 = cv2.moments(mask_blue)
                M2 = cv2.moments(mask_green)#blue line means edge of region
                M3 = cv2.moments(mask_tout)
                green_area = M2["m00"]
                # Compute the total area of the image
                total_area = mask_green.shape[0] * mask_green.shape[1]
                # Calculate the area ratio of the blue color
                area_ratio = green_area / total_area
                # calculate the green color occupies how much pourcentage of the image
                
                
                if M3["m00"] == 0:
                # Adjust robot motion to stop when the goal is reached (no blue or green in the entire image)
                    twist.linear.x = 0
                    twist.angular.z = 0
                    pub.publish(twist)
                    
		# Check if the green color occupies at least 10% of the image, indicating that the robot is close to the goal.
		# Additionally, there is a blue line at the destination, so we add this condition:
		# When the robot is close to the goal, it will move forward (trying to center the green region in the image)
		# instead of avoiding the blue line.
                elif M1["m00"] > 0 and  M2["m00"] > 0 and area_ratio > 0.1:
                    cX2 = int(M2["m10"] / M2["m00"])
                    cY2 = int(M2["m01"] / M2["m00"])
                    error = cX2 - width/2
                    if error < 20:
                        twist.linear.x = 0.2
                        twist.angular.z = 0
                        pub.publish(twist)
                    else:
                        twist.linear.x = 0.05
                        twist.angular.z = shrehold(-0.02* error,1)
                        pub.publish(twist)

    
        	# If a blue color is detected, try to avoid the blue region
                elif M1["m00"] > 0 :
                    cX1 = int(M1["m10"] / M1["m00"])
                    cY1 = int(M1["m01"] / M1["m00"])
                    twist.linear.x = -0.02
                    error = cX1 - width/2
                    twist.angular.z = shrehold(0.02* error,1)
                    pub.publish(twist)
  
                # If a green color is detected, try to center the green region in the image
                elif M2["m00"] > 0 :
                    cX2 = int(M2["m10"] / M2["m00"])
                    cY2 = int(M2["m01"] / M2["m00"])
                    error = cX2 - width/2
                    if error < 20:
                        twist.linear.x = 0.2
                        twist.angular.z = 0
                        pub.publish(twist)
                    else:
                        twist.linear.x = 0.05
                        twist.angular.z = shrehold(-0.02* error,1)
                        pub.publish(twist)

                 #else go forward
                else: 
                    twist.linear.x = 0.2
                    twist.angular.z = 0
                    pub.publish(twist)
                    
    

if __name__ == '__main__':
    main()