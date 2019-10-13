#!/usr/bin/env python

import sys
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from stero_mobile_init.srv import PathFollower
import numpy as np


def path_follower_client(positions):
    path = Path()
    for position in positions:
        pose = PoseStamped()
        pose.pose.position.x, pose.pose.position.y = position
        path.poses.append(pose)
    
    try:
        srv = rospy.ServiceProxy('path_follower', PathFollower)
        resp1 = srv(path)
        return resp1.message
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def is_digit(x):
    try:
        float(x)
        return True
    except ValueError:
        return False


if __name__ == "__main__":
    args = [arg for arg in sys.argv if is_digit(arg)]
    positions = np.empty((len(args)/2, 2))
    if len(args) > 0 and len(args) % 2 == 0:
        positions[:] = np.array(args).reshape((len(args)/2, 2))
    else:
        print "Invalid arguments"
        sys.exit(1)
    
    print path_follower_client(positions)
