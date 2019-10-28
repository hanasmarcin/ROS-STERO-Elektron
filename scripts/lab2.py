#!/usr/bin/env python

import sys
import rospy
from geometry_msgs.msg import PoseStamped, Pose2D
from nav_msgs.msg import Odometry, Path
from nav_msgs.srv import GetPlan
from stero_mobile_init.srv import PathFollower
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np


def get_position(data):
    """
    Function translates pose given by listener to [x, y] format 
    and updates global variables current_pos and prev_pos
    """
    global current_pos
    pose = data.pose.pose
    current_pos = np.array((pose.position.x, pose.position.y))

def get_plan(data):
    start = PoseStamped()
    start.header.frame_id = 'map'
    rospy.wait_for_message('elektron/mobile_base_controller/odom', Odometry)
    start.pose.position.x = current_pos[0]
    start.pose.position.y = current_pos[1]

    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.pose.position.x = data[0]
    goal.pose.position.y = data[1]

    make_path_srv = rospy.ServiceProxy('/global_planner/planner/make_plan', GetPlan)
    path = make_path_srv(start=start, goal=goal, tolerance=0.0).plan

    new_path = Path()
    new_path.header = path.header
    new_path.poses = [path.poses[i] for i in range(len(path.poses)/10, len(path.poses), len(path.poses)/10)]
    new_path.poses.append(path.poses[-1])
    path_follower_srv = rospy.ServiceProxy('path_follower', PathFollower)
    resp1 = path_follower_srv(new_path)


def is_digit(x):
    try:
        float(x)
        return True
    except ValueError:
        return False


if __name__ == "__main__":
    rospy.init_node('lab2')
    sub_pos = rospy.Subscriber('elektron/mobile_base_controller/odom', Odometry, get_position)

    args = [arg for arg in sys.argv if is_digit(arg)]
    position = np.empty(2)
    if len(args) == 2:
        position[:] = np.array(args)
    else:
        print "Invalid arguments"
        sys.exit(1)
    
    print get_plan(position)