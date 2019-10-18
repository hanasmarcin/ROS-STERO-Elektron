#!/usr/bin/env python

import sys
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from stero_mobile_init.srv import PathFollower
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import csv

row = np.empty(9)
data_file = open('proj1test1.csv', mode='w')
file_writer = csv.writer(data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

i = 0


def write_odom_row(odom, row_order):
    
    pose = odom.pose.pose
    orientation_list = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    _, _, th = euler_from_quaternion (orientation_list)
    row[row_order:row_order+4] = [odom.header.stamp.secs*1000 + odom.header.stamp.nsecs/1000000, pose.position.x, pose.position.y, th]


def get_position_gazebo(odom):
    """
    Function translates pose given by listener to [x, y, th] format 
    and updates global variables current_pos and prev_pos
    """
    write_odom_row(odom, 1)


def get_position_base(odom):
    """
    Function translates pose given by listener to [x, y, th] format 
    and updates global variables current_pos and prev_pos
    """
    write_odom_row(odom, 5)

def collect_data():
    data_file.flush()
    gazebo_odom = rospy.Subscriber('gazebo_odom', Odometry, get_position_gazebo)
    gazebo_base = rospy.Subscriber('elektron/mobile_base_controller/odom', Odometry, get_position_base)
    # rospy.spin()
    rate = rospy.Rate(25)
    k = 0
    while k < 350:
        row[0] = k
        rospy.wait_for_message('gazebo_odom', Odometry)
        rospy.wait_for_message('elektron/mobile_base_controller/odom', Odometry)
        file_writer.writerow(row)
        k += 1
        rate.sleep()
    # mobile_base_controller = rospy.Subscriber('elektron/mobile_base_controller/odom', Odometry)




# def path_follower_client(positions):
#     path = Path()
#     for position in positions:
#         pose = PoseStamped()
#         pose.pose.position.x, pose.pose.position.y = position
#         path.poses.append(pose)
    
#     try:
#         srv = rospy.ServiceProxy('path_follower', PathFollower)
#         resp1 = srv(path)
#         return resp1.message
#     except rospy.ServiceException, e:
#         print "Service call failed: %s"%e


# def is_digit(x):
#     try:
#         float(x)
#         return True
#     except ValueError:
#         return False


if __name__ == "__main__":
    # args = [arg for arg in sys.argv if is_digit(arg)]
    # positions = np.empty((len(args)/2, 2))
    # if len(args) > 0 and len(args) % 2 == 0:
    #     positions[:] = np.array(args).reshape((len(args)/2, 2))
    # else:
    #     print "Invalid arguments"
    #     sys.exit(1)
    
    # print path_follower_client(positions)
    rospy.init_node('proj1')
    collect_data()
