#!/usr/bin/env python

from __future__ import print_function

# Math/Data structures/Functions
import numpy as np
from math import sin, cos, tanh

# ROS
import rospy
import message_filters
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from foodelpi_webservice.srv import *

### Global Variables


def handle_get_room_list(req):
    print('Got a room-list request!')
    
    res = GetRoomListResponse()
    res.success = True
    res.rooms.append(10023)
    res.rooms.append(10040)
    res.rooms.append(20058)
    res.rooms.append(30010)

    return res


def handle_get_menu(req):
    print('Got a menu request!')
    
    res = GetMenuResponse()
    res.success = True
    res.menu_options.append('Grilled Cheese')
    res.menu_options.append('Mac\'N\'Cheese')
    res.prices.append(2.95)
    res.prices.append(4.50)
    
    return res


if __name__ == '__main__':
    ##### ROS #####
    # Initialize this node
    rospy.init_node('foodelpi_server', anonymous=True)
    
    # Subscriber
    # sub_sensor_data = rospy.Subscriber('/map', OccupancyGrid, cb_map)
    # sub_pose_data = rospy.Subscriber('/move_base_simple/goal', PoseStamped, cb_goal)
    
    # Publisher
    # pub_map = rospy.Publisher('/ia/map', OccupancyGrid, queue_size=1)
    
    # Services
    srv_menu = rospy.Service('/foodelpi/get_menu', GetMenu, handle_get_menu)
    srv_rooms = rospy.Service('/foodelpi/get_room_list', GetRoomList, handle_get_room_list)

    # ROS spin
    rospy.spin()
