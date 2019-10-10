#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

OMEGA = 0.2
VELOCITY = 0.2


def move_elektron(data):

    current_pos = [0, 0, 0]

    orientation_list = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
    _, _, th = euler_from_quaternion (orientation_list)
    new_pos = [data.position.x, data.position.y, th]

    alpha = math.atan2(new_pos[1], new_pos[0])
    x = math.sqrt(new_pos[0]**2 + new_pos[1]**2)
    alpha_end = new_pos[2]-alpha
    
    twist_rotation = Twist()
    twist_rotation.angular.z = math.copysign(OMEGA, alpha)
    rotation_time = math.fabs(alpha/OMEGA)
    
    twist_translation = Twist()
    twist_translation.linear.x = VELOCITY
    translation_time = x/VELOCITY

    twist_rotation_end = Twist()
    print(alpha_end)
    twist_rotation_end.angular.z = math.copysign(OMEGA, alpha_end)
    print(twist_rotation_end)
    rotation_end_time = math.fabs(alpha_end/OMEGA)

    twist_stop = Twist()

    pub.publish(twist_rotation)
    rospy.sleep(rotation_time)
    pub.publish(twist_translation)
    rospy.sleep(translation_time)
    pub.publish(twist_rotation_end)
    rospy.sleep(rotation_end_time)
    pub.publish(twist_stop)


if __name__ == "__main__":
    rospy.init_node('lab1')
    pub = rospy.Publisher('mux_vel_nav/cmd_vel', Twist, queue_size=1)
    s = rospy.Subscriber('lab1_topic', Pose, move_elektron)
rospy.spin()
