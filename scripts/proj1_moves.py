#!/usr/bin/env python

import sys
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from stero_mobile_init.srv import Proj1Moves
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import numpy as np
import csv

DIST = 2
ALPHA = 2*math.pi
OMEGA = 0.5
VELOCITY = 0.5
twist_translation = Twist()



def move_forwards_backwards(pub):
    rospy.sleep(1)
    twist_translation.linear.x = -1*VELOCITY
    translation_time = DIST/VELOCITY
    pub.publish(twist_translation)
    rospy.sleep(translation_time)
    
    twist_translation.linear.x = VELOCITY
    pub.publish(twist_translation)
    rospy.sleep(translation_time)

    pub.publish(Twist())
    return "Success"


def move_rotate():
    pub = rospy.Publisher('mux_vel_nav/cmd_vel', Twist, queue_size=1)
    rospy.sleep(1)
    twist_rotation = Twist()
    twist_rotation.angular.z = OMEGA
    rotation_time = math.fabs(ALPHA/OMEGA)
    pub.publish(twist_rotation)
    rospy.sleep(rotation_time)

    twist_rotation.angular.z = -1*OMEGA
    rotation_time = math.fabs(ALPHA/OMEGA)
    pub.publish(twist_rotation)
    rospy.sleep(rotation_time)

    pub.publish(Twist())
    return "Success"


def service(data):
    pub = rospy.Publisher('mux_vel_nav/cmd_vel', Twist, queue_size=1)
    if data.choice == 0:
        move_forwards_backwards(pub)
    elif data.choice == 1:
        move_rotate()
    else: 
        return "Wrong choice"
    return "Perfect!"

if __name__ == "__main__":

    rospy.init_node('proj1_moves')
    srv = rospy.Service('proj1_moves', Proj1Moves, service)
    
rospy.spin()

