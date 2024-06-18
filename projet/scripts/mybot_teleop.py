#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys, termios, tty
import click
from std_msgs.msg import String
from geometry_msgs.msg import Twist 

def talker(): 

    linear_scale=rospy.get_param('linear_scale')
    angular_scale=rospy.get_param('angular_scale')
    topic=rospy.get_param('/topic')
    pub = rospy.Publisher(topic, Twist, queue_size=10)
    rospy.init_node('mybot_teleop')# name of the node 
    

    
    while not rospy.is_shutdown():
        move_msg=Twist()
    
        # Get character from console
        mykey = click.getchar()
        if mykey in keys.keys():
            char=keys[mykey]

        if char == 'up':    # UP key
            # Do something
            move_msg.linear.x=linear_scale
            pub.publish(move_msg)
        if char == 'down':  # DOWN key
            # Do something
            move_msg.linear.x=-linear_scale
            pub.publish(move_msg)
        if char == 'right':  # RIGHT key
            # Do something
            move_msg.angular.z=-angular_scale
            pub.publish(move_msg)
        if char == 'left': # LEFT
            # Do something*
            move_msg.angular.z=angular_scale
            pub.publish(move_msg)
        if char == 'stop': # stop
            # Do something*
            move_msg.angular.z=0
            move_msg.linear.x=0
            pub.publish(move_msg)
        if char == "quit":  # QUIT
            # Do something
            sys.exit()
    

        

# Arrow keys codes
keys = {'\x1b[A':'up', '\x1b[B':'down', '\x1b[C':'right', '\x1b[D':'left', 's':'stop', 'q':'quit'}

if __name__ == '__main__':    
    try:
        talker()
    except rospy.ROSInterruptException:
        pass