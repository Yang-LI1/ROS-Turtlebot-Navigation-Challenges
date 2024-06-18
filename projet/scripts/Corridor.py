#!/usr/bin/env python3
# -- coding: utf-8 --
"""
This node implements a wall-following behavior using a PID controller.
The robot uses the LaserScan data to measure the distances on the left and right sides.
The error is calculated as the difference between the left and right distances.
The PID controller adjusts the angular speed of the robot based on the error.
The robot moves forward until the error is within a certain threshold, indicating it has reached the target distance from the wall.
"""

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

# Set PID controller parameters
kp = 1
ki = 0
kd = 0

#Set maximum linear and angular speeds of the robot
max_linear_speed = 0.1
max_angular_speed = 1.5

# Set the desired difference between left and right distances (adjust according to the actual scenario)
# here we set 0 because we want the robot at the center of the corridor
target_distance_diff = 0

# Set the distance threshold, when the difference between left and right distances is smaller than this value, consider the goal reached
distance_threshold = 0.02

class WallFollowing:
    def __init__(self):
        #Initialize the node
        rospy.init_node('wall_following')

        # Subscribe to laser scan data
        rospy.Subscriber('/scan', LaserScan, self.process_scan)

        # Publish velocity control commands
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        #Initialize error variables
        self.error = 0.0
        self.error_integral = 0.0
        self.error_derivative = 0.0
        self.last_error = 0.0

    def process_scan(self, scan_msg):

        cmd_vel = Twist()
        right_scan = np.array(scan_msg.ranges[290:310])
        left_scan = np.array(scan_msg.ranges[50:70])   
        face_scan = np.array(scan_msg.ranges[0:5]+scan_msg.ranges[-5:])
        
	# Remove infinities and NaNs
        right_scan = right_scan[np.isfinite(right_scan)]
        left_scan = left_scan[np.isfinite(left_scan)]
        face_scan = face_scan[np.isfinite(face_scan)]


        # Calculate distances on the left and right sides
        if len(right_scan)==0 or len(left_scan) == 0:
            cmd_vel.linear.x = 0.2
            cmd_vel.angular.z = 0
            self.cmd_vel_pub.publish(cmd_vel)
        else:
            right_distance = np.mean(right_scan)
            left_distance = np.mean(left_scan)
                            
    
            #Calculate the error (difference between left and right distances)
            self.error = left_distance - right_distance - target_distance_diff 

            # Calculate the integral term of the error
            self.error_integral += self.error
    
            #Calculate the derivative term of the error
            self.error_derivative = self.error - self.last_error
            self.last_error = self.error
    
            # Calculate the output of the PID controller
            angular_speed = kp * self.error + ki *self.error_integral + kd * self.error_derivative

       
            
            # Limit the angular speed of the robot within the maximum angular speed range
            if angular_speed > max_angular_speed:
                angular_speed = max_angular_speed
            elif angular_speed < -max_angular_speed:
                angular_speed = -max_angular_speed
    
            #If the difference between left and right distances is smaller than the threshold, the robot go ahead
            if abs(self.error-target_distance_diff) < distance_threshold:
                # Move forward
                cmd_vel = Twist()
                cmd_vel.linear.x = max_linear_speed
                self.cmd_vel_pub.publish(cmd_vel)
            else:
                # Publish velocity control command
                cmd_vel = Twist()
                cmd_vel.linear.x = 0.04
                cmd_vel.angular.z = angular_speed
                self.cmd_vel_pub.publish(cmd_vel)
                
if __name__ == '__main__':
    node = WallFollowing()
    rospy.spin()