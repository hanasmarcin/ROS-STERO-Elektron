#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

OMEGA = 0.2
VELOCITY = 0.2
global.path = None
global.prev_pos = [0, 0, 0]
global.current_pos = [0, 0, 0]
global.i = 0


def get_path(path):


def move_elektron():
    #s2 = rospy.Subscriber('elektron/mobile_base_controller/odom', Pose, check_position)
    global.path = path
    twist_rotation = Twist()
    twist_translation = Twist()
    twist_translation.linear.x = VELOCITY

    for poseStamped in path:
	pose = path.pose
	
	orientation_list = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
	_, _, th = euler_from_quaternion (orientation_list)
	new_pos = [pose.position.x, pose.position.y, th]

	alpha = math.atan2(new_pos[1], new_pos[0])

    	twist_rotation.angular.z = math.copysign(OMEGA, alpha)
	
	pub.publish(twist_rotation)
	while not (global.i > 1 and global.prev_pos[2] - new_pos[2] < global.current_pos[2] - new_pos[2]):
	    pass
	
    	pub.publish(twist_translation)
	while not (global.prev_pos[2] - new_pos[2] < global.current_pos[2] - new_pos[2]):
	    pass


    twist_rotation_end = Twist()
    print(alpha_end)
    twist_rotation_end.angular.z = math.copysign(OMEGA, alpha_end)
    print(twist_rotation_end)
    rotation_end_time = alpha_end/OMEGA

    twist_stop = Twist()

    pub.publish(twist_rotation)
    rospy.sleep(rotation_time)
    pub.publish(twist_translation)
    rospy.sleep(translation_time)
    pub.publish(twist_rotation_end)
    rospy.sleep(rotation_end_time)
    pub.publish(twist_stop)


def check_position(data):
    global.prev_pos = global.current_pos
    global.current_pos = [data.position.x, data.position.y, th]
    global.i += 1
    print(data)


if __name__ == "__main__":
    rospy.init_node('lab1_2')
    pub = rospy.Publisher('mux_vel_nav/cmd_vel', Twist, queue_size=1)
    s = rospy.Subscriber('lab1_2_topic', Path, move_elektron)
    while global.path is None:
	rospy.spinOnce()
    s2 = rospy.Subscriber('elektron/mobile_base_controller/odom', Pose, check_position)

    
rospy.spin()
