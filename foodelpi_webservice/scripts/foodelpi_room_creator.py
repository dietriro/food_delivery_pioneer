#!/usr/bin/env python

from __future__ import print_function

# Math/Data structures/Functions
import numpy as np
from math import sin, cos, tanh
import yaml

# ROS
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from foodelpi_webservice.srv import *
from foodelpi_webservice.msg import MenuItem, Room, Order, RobotState
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import *
from move_base_msgs.msg import MoveBaseActionGoal

if __name__ == '__main__':
    ##### Read data from yaml #####
    read_menu_from_yaml()
    read_rooms_from_yaml()
    
    ##### ROS #####
    # Initialize this node
    rospy.init_node('foodelpi_server', anonymous=True)
    
    # Subscriber
    # sub_sensor_data = rospy.Subscriber('/map', OccupancyGrid, cb_map)
    # sub_pose_data = rospy.Subscriber('/move_base_simple/goal', PoseStamped, cb_goal)
    sub_goal_status = rospy.Subscriber('/move_base/status', GoalStatusArray, cb_goal_status)
    
    # Publisher
    pub_status = rospy.Publisher('/foodelpi/robot_state', RobotState, queue_size=1)
    
    print('Initialization completed. Waiting for requests...')
    
    # ROS spin
    rospy.spin()