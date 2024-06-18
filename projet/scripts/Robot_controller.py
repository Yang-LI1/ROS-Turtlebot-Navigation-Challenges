#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import time
import rospy
import subprocess
import numpy as np
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage


class Brain:
    """
    the brain class will control the launching and stopping of the different nodes:
        On the one hand with the detection of red lines published on the topic /red_line_detected by the node Red_line_detction.py
        On the other hand with the detection of blue panels on the circuit which we will use as stop signs
    """

    def __init__(self):

        rospy.init_node('Robot_controller')

        self.cpt = 6
        self.stop=0
        self.stop2=0
        self.etat_init=False
        self.run_lin_following=0
        self.run_Obstacle=0
        self.run_corridor=0
        self.run_arrene=0

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub_cam = rospy.Subscriber('/camera/image/compressed', CompressedImage, self.image_callback)
        rospy.Subscriber('/red_line_detected', Bool, self.red_line_callback)
     
    def red_line_callback(self, msg):
        """
        this callback function launch the nodes of the different challenges each time a red line has been detected.
        We determine which nodes to execute based on the number of detection of red lines on the ground.
        """ 
        # Initialize a Twist object for velocity commands
        cmd_vel = Twist()
        # Check if the message data is not equal to the initial state and is not false
        if msg.data != self.etat_init and msg.data:
            # Decrement the counter variable
            self.cpt -= 1
            # If the counter variable reaches 5, launch the line following node
            if self.cpt == 5:
                # Create a LaunchManager object to run the Line_following node
                self.run_lin_following = subprocess.Popen(['rosrun', 'projet', 'Line_following.py'])

            # If the counter variable is 3, stop the previous node and launch the obstacle avoidance node
            elif self.cpt == 3:
                # Stop the previous node and set the velocity commands to 0
                time.sleep(1)
                self.run_lin_following.terminate()
                cmd_vel.linear.x = 0
                cmd_vel.angular.z = 0
                self.cmd_vel_pub.publish(cmd_vel)
                # Create a LaunchManager object to run the Obstacle node
                self.run_Obstacle = subprocess.Popen(['rosrun', 'projet', 'Obstacle.py'])

            # If the counter variable is 2, stop the previous nodes and launch the robot movement node in a corridor
            elif self.cpt == 2:
                # Stop the previous nodes and set the velocity commands to move forward slowly
                time.sleep(1)
                self.run_lin_following.terminate()
                self.run_Obstacle.terminate()
                cmd_vel.linear.x = 0.05
                cmd_vel.angular.z = 0
                self.cmd_vel_pub.publish(cmd_vel)
                # Wait for 5 seconds before stopping the robot
                time.sleep(6)
                # Set the velocity commands to stop  the robot movement
                cmd_vel.linear.x = 0
                cmd_vel.angular.z = 0
                self.cmd_vel_pub.publish(cmd_vel)
                # Create a LaunchManager object to run the Corridor node
                self.run_corridor = subprocess.Popen(['rosrun', 'projet', 'Corridor.py'])

            # If the counter variable is 1, stop the previous nodes and restart the line following node
            elif self.cpt == 1:
                # Stop the previous nodes and set the velocity commands to stop
                time.sleep(1)
                cmd_vel.linear.x = 0
                cmd_vel.angular.z = 0
                self.cmd_vel_pub.publish(cmd_vel)
                # Create a LaunchManager object to run the Line_following node
                self.run_lin_following = subprocess.Popen(['rosrun', 'projet', 'Line_following.py'])

            # If the counter variable is 0, stop the previous node and launch the arrene node
            elif self.cpt == 0:
                # Stop the previous node and set the velocity commands to move slowly in the arrene
                self.run_lin_following.terminate()
                cmd_vel.linear.x = 0.06
                cmd_vel.angular.z = 0.1
                self.cmd_vel_pub.publish(cmd_vel)
                #Wait 5s before stopping the robot and launch the next node
                time.sleep(5)
                cmd_vel.linear.x = 0
                cmd_vel.angular.z = 0
                self.cmd_vel_pub.publish(cmd_vel)
                # Create a LaunchManager object to run the arren node
                self.run_arrene=subprocess.Popen(['rosrun', 'projet', 'arrene.py'])

            else:#reset
                if self.cpt<0: 
                    self.cpt=6
                    self.run_lin_following=0
                    self.run_Obstacle=0
                    self.run_corridor=0
                    self.run_arrene=0

            self.etat_init=msg.data# status update

        self.etat_init=msg.data#status update


    def SetStop(self):
        """
        the SetStop function is called by the image_callback when:
        the first panel is detected and the robot is in status 3
        the second panel is detected and the robot is in state 2
        """

        # Initialize a Twist message
        cmd_vel = Twist()

        # If stop condition 1 is met, do the following:
        if self.stop==1:
            # Stop the previous node
            self.run_Obstacle.terminate()

            # Set the linear and angular velocities to zero
            cmd_vel.linear.x = 0
            cmd_vel.angular.z = 0
            self.cmd_vel_pub.publish(cmd_vel)

            # Launch the line following node
            self.run_lin_following=subprocess.Popen(['rosrun', 'projet', 'Line_following.py'])

        # If stop condition 2 is met, do the following:
        if self.stop2==1:
            # Stop the previous node
            self.run_corridor.terminate()

            # Set the linear and angular velocities to zero
            cmd_vel.linear.x = 0
            cmd_vel.angular.z = 0
            self.cmd_vel_pub.publish(cmd_vel)

            # Launch the line following node
            self.run_lin_following=subprocess.Popen(['rosrun', 'projet', 'Line_following.py'])



    def image_callback(self,msg):
        """
        This callback function is called each time an image is received on the topic /camera/image/compressed.
        For each image we check if one of the two blue traffic signs, which we use as stop signs, have been detected.
        If one of them is detected and the robot is in a particular state, the SetStop function defined above is called.
        """
        # Convert the compressed image message to a cv2 image
        bridge = CvBridge()
        image = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        
        # Convert the image to the HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Get the height and width of the image
        height, width, _ = hsv.shape
        
        # Define the region of interest (ROI) to detect the first blue traffic signs
        top = int(height * 0.4)
        bottom = int(height * 0.45)
        left = int(width * 0.77)
        right = int(width * 0.9)
        hsv2 = hsv[top:bottom, left:right] # Crop the image to the ROI
        
        # Define the region of interest (ROI) to detect the second blue traffic signs
        top = int(height * 0.3)
        bottom = int(height * 0.4)
        left = int(width * 0.45)
        right = int(width * 0.55)
        hsv3 = hsv[top:bottom, left:right] # Crop the image to the ROI
        
        # Define the lower and upper blue color range in the HSV color space
        lower_blue = np.array([100, 50, 50])
        upper_blue = np.array([130, 255, 255])
        
        # Apply a mask to the image to keep only blue pixels in the fisrt ROI
        mask_blue = cv2.inRange(hsv2, lower_blue, upper_blue)
        # Apply a mask to the image to keep only white pixels in the second ROI
        mask_blue2 = cv2.inRange(hsv3, lower_blue, upper_blue)
    
        # Compute the moments of the blue masks 
        self.M = cv2.moments(mask_blue)
        self.M2 = cv2.moments(mask_blue2)
        
        # If the first blue sign is detected and the robot is in state 3, call SetStop
        if self.M["m00"] > 0 and self.cpt == 3:
            self.stop += 1
            self.SetStop()
        
        # If the second blue sign is detected and the robot is in state 2, call SetStop
        if self.M2["m00"] > 0 and self.cpt == 2:
            self.stop2 += 1
            self.SetStop()
    

    
                


if __name__ == '__main__':
    try:
        Brain()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass