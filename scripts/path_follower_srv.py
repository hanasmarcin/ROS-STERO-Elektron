#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose, PoseStamped
from nav_msgs.msg import Path, Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from stero_mobile_init.srv import PathFollower
import math
import numpy as np


OMEGA = 0.3
VELOCITY = 0.3
ATOL = 1e-07
RTOL = 1e-07

def normalize_angle(alpha):
    """
    Function normalize angle to range [-pi, pi]
    """
    if alpha > math.pi:
        alpha -= 2*math.pi
    elif alpha < -1*math.pi:
        alpha += 2*math.pi
    return alpha


def get_next_position(pose):
    """
    Function calculates position for given pose and returns it in format [x, y, th]
    """
    th = math.atan2(pose.position.y - current_pos[1],pose.position.x -current_pos[0])
    new_pos = np.array((pose.position.x, pose.position.y, normalize_angle(th)))
    return new_pos


def get_position(data):
    """
    Function translates pose given by listener to [x, y, th] format 
    and updates global variables current_pos and prev_pos
    """
    global i 
    global current_pos
    global prev_pos 
    prev_pos[:] = np.copy(current_pos)
    pose = data.pose.pose
    orientation_list = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    _, _, th = euler_from_quaternion (orientation_list)
    current_pos[:] = [pose.position.x, pose.position.y, th]
    i += 1


def set_robots_angular_velocity(new_pos):
    """
    Function calculates, whether robot should rotate clockwise or counterclockwise, sets it's angular velocity 
    and starts the rotation
    """
    global twist_rotation
    alpha = new_pos[2] - current_pos[2]
    alpha = normalize_angle(alpha)

    twist_rotation.angular.z = math.copysign(OMEGA, alpha)
    pub.publish(twist_rotation)


def get_pos_rotation_diff(pos_a, pos_b):
    """
    Function calculates difference between two angles
    """
    diff = pos_b[2] - pos_a[2]
    diff = normalize_angle(diff)
    return math.fabs(diff)


def is_rotation_reached(new_pos):
    """
    Function checks, whether distance between robot's current angle and desirable angle has started increasing 
    and returns True, if that has happened
    """
    return i > 0 and get_pos_rotation_diff(new_pos, prev_pos) < (1-RTOL)*get_pos_rotation_diff(new_pos, current_pos) - ATOL


def get_distance(pos_a, pos_b):
    """
    Function calculates Euclidean distance between two points
    """
    return np.linalg.norm(pos_a[0:2] - pos_b[0:2])


def is_translation_reached(new_pos):
    """
    Function checks, whether distance between robot's current position and desirable position has started increasing 
    and returns True, if that has happened
    """
    current_distance = get_distance(new_pos, current_pos)
    return i > 0 and get_distance(new_pos, prev_pos) < (1-RTOL)*current_distance - ATOL and current_distance < 0.5


def stop_robot():
    """
    Function stops robot and waits until it is no longer moving
    """
    twist_stop = Twist()
    pub.publish(twist_stop)
    global current_pos
    global prev_pos
    while not np.allclose(current_pos, prev_pos, rtol=RTOL, atol=ATOL):
        rospy.wait_for_message('elektron/mobile_base_controller/odom', Odometry)


def move_elektron(data):
    """
    Service's main function
    Moves robot on given path and returns message
    """
    # initialize variables
    global i
    global prev_pos
    global current_pos
    prev_pos = np.empty(3)
    current_pos = np.empty(3)
    i = 0
    path = data.path.poses

    # create variables for setting robot's velocity
    global twist_rotation
    twist_rotation = Twist()
    twist_translation = Twist()
    twist_translation.linear.x = VELOCITY
    twist_stop = Twist()
    
    # create subscriber to get current robot's position
    sub_pos = rospy.Subscriber('elektron/mobile_base_controller/odom', Odometry, get_position)
    rospy.wait_for_message('elektron/mobile_base_controller/odom', Odometry)

    # loop for every point in path
    for poseStamped in path:
        # find position for robot to reach for the given point
        pose = poseStamped.pose
        new_pos = get_next_position(pose)

        # rotate robot
        i = 0
        set_robots_angular_velocity(new_pos)
        while not is_rotation_reached(new_pos):
            rospy.wait_for_message('elektron/mobile_base_controller/odom', Odometry)
            # pub.publish(twist_rotation)
        stop_robot()

        # translate robot
        i = 0
        pub.publish(twist_translation)
        while not is_translation_reached(new_pos):
            rospy.wait_for_message('elektron/mobile_base_controller/odom', Odometry)
            # pub.publish(twist_translation)
        stop_robot()

    # clean after moving robot
    success_msg = "Robot moved!\nCurrent robot's position: " + np.array2string(current_pos)
    sub_pos.unregister()
    del prev_pos
    del current_pos
    del i
    return success_msg


if __name__ == "__main__":
    rospy.init_node('lab1_2')
    pub = rospy.Publisher('mux_vel_nav/cmd_vel', Twist, queue_size=1)

    srv = rospy.Service('path_follower', PathFollower, move_elektron)
    
rospy.spin()
