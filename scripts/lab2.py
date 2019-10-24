#!/usr/bin/env python

import sys
import rospy
from geometry_msgs.msg import PoseStamped, Pose2D
from nav_msgs.msg import Odometry, Path
from stero_mobile_init.srv import PathFollower
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np


def get_plan(path):
    new_path = Path()
    new_path.header = path.header
    new_path.poses = [path.poses[i] for i in range(0, len(path.poses), len(path.poses)/10)]
    new_path.poses.append(path.poses[-1])
    srv = rospy.ServiceProxy('path_follower', PathFollower)
    print len(new_path.poses)
    resp1 = srv(new_path)


if __name__ == "__main__":
    rospy.init_node('lab2')
    plan_sub = rospy.Subscriber('/global_planner/planner/plan', Path, get_plan)
    rospy.spin()
