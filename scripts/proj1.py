#!/usr/bin/env python

import sys
import rospy
from geometry_msgs.msg import PoseStamped, Pose2D
from nav_msgs.msg import Odometry
from stero_mobile_init.srv import PathFollower
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import csv

row = np.empty(13)
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

def get_position_laser(pose):
    """
    Function translates pose given by listener to [x, y, th] format 
    and updates global variables current_pos and prev_pos
    """
    now = rospy.get_rostime()
    row[9:13] = [now.secs*1000 + now.nsecs/1000000, pose.x, pose.y, pose.theta]


def collect_data():
    data_file.flush()
    file_writer.writerow(('id','t_go', 'x_go', 'y_go', 'th_go', 't_mbc', 'x_mbc', 'y_mbc', 'th_mbc', 't_l', 'x_l', 'y_l', 'th_l'))
    gazebo_odom = rospy.Subscriber('gazebo_odom', Odometry, get_position_gazebo)
    gazebo_base = rospy.Subscriber('elektron/mobile_base_controller/odom', Odometry, get_position_base)
    gazebo_laser = rospy.Subscriber('pose2D', Pose2D, get_position_laser)
    # rospy.spin()
    rate = rospy.Rate(25)
    k = 0
    while k < 850:
        row[0] = k
        rospy.wait_for_message('gazebo_odom', Odometry)
        rospy.wait_for_message('elektron/mobile_base_controller/odom', Odometry)
        rospy.wait_for_message('pose2D', Pose2D)

        file_writer.writerow(row)
        k += 1
        rate.sleep()
    
    # data_file.close()
    # mobile_base_controller = rospy.Subscriber('elektron/mobile_base_controller/odom', Odometry)





if __name__ == "__main__":

    rospy.init_node('proj1')
    collect_data()
