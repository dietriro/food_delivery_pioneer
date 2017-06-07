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
from geometry_msgs.msg import Pose, Point32, PoseWithCovarianceStamped
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from foodelpi_webservice.srv import *
from foodelpi_webservice.msg import MenuItem, Room, Order, RobotState
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import *
from move_base_msgs.msg import MoveBaseActionGoal

### Global Variables
room_list_ = None
menu_ = None
orders_ = []
# ToDo: Set or load correct home position for robot
home_ = Pose()

current_goal_ = None
current_pos_ = None
close_to_goal_ = False

current_order_ = None
robot_state_ = 3
robot_floor_ = 1
robot_states_ = ['Home', 'Delivery', 'Returning', 'Unknown', 'Delivered']


def check_pos():
    global current_goal_, close_to_goal_, current_pos_
    
    if current_goal_ is None:
        return
    
    if np.mean(np.abs(current_pos_ - current_goal_)) < 0.1:
        close_to_goal_ = True
    else:
        close_to_goal_ = False
    

def cb_pose(msg):
    global current_pos_

    current_pos_ = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
    

def cb_goal_status(msg):
    global robot_state_, home_, close_to_goal_
    
    if len(msg.status_list) <= 0:
        return
    
    # Check the status of move_base
    if msg.status_list[0].status == 3 and close_to_goal_:
        # Reached home
        if robot_state_ == 2:
            robot_state_ = 0
        # Successfully delivered
        elif robot_state_ == 1:
            robot_state_ = 4
        # elif robot_state_ == 4:
        #     robot_state_ = 2
        #     publish_goal(home_)


def read_menu_from_yaml():
    global menu_
    
    if menu_ is not None:
        print('Menu has already been filled.')
        return

    print('Reading menu from yaml file.')

    menu_ = []

    stream = open('/home/pio-rosa/catkin_ws/src/food_delivery_pioneer/foodelpi_webservice/cfg/menu.yaml', 'r')
    menus = yaml.load(stream)
    
    for menu in menus:
        for menu_part, items in menu.iteritems():
            for item in items:
                tmp_item = MenuItem()
                for key, value in item.iteritems():
                    if 'id' in key:
                        tmp_item.id = value
                    if 'name' in key:
                        tmp_item.name = value
                    if 'price' in key:
                        tmp_item.price = value
                menu_.append(tmp_item)
                        

def read_rooms_from_yaml():
    global room_list_, home_
    
    if room_list_ is not None:
        print('Menu has already been filled.')
        return
    
    print('Reading room list from yaml file.')

    room_list_ = []
    
    stream = open('/home/pio-rosa/catkin_ws/src/food_delivery_pioneer/foodelpi_webservice/cfg/room_list.yaml', 'r')
    rooms = yaml.load(stream)
    
    for room in rooms:
        tmp_item = Room()
        for key, value in room.iteritems():
            if 'id' in key:
                tmp_item.id = value
            if 'number' in key:
                tmp_item.number = value
            if 'floor' in key:
                tmp_item.floor = value
            if 'position' in key:
                for pos_key, pos_value in value.iteritems():
                    if 'x' in pos_key:
                        tmp_item.pose.position.x = pos_value
                    if 'y' in pos_key:
                        tmp_item.pose.position.y = pos_value
                    if 'z' in pos_key:
                        tmp_item.pose.position.z = pos_value
            if 'orientation' in key:
                for pos_key, pos_value in value.iteritems():
                    if 'x' in pos_key:
                        tmp_item.pose.orientation.x = pos_value
                    if 'y' in pos_key:
                        tmp_item.pose.orientation.y = pos_value
                    if 'z' in pos_key:
                        tmp_item.pose.orientation.z = pos_value
                    if 'w' in pos_key:
                        tmp_item.pose.orientation.w = pos_value
        
        room_list_.append(tmp_item)

    home_ = room_list_[0].pose
    

def handle_get_room_list(req):
    global room_list_
    
    print('Got a room-list request!')
    
    res = GetRoomListResponse()
    res.success = True
    res.rooms = room_list_[1:]

    return res


def handle_get_menu(req):
    global menu_
    
    print('Got a menu request!')
    
    res = GetMenuResponse()
    res.success = True
    res.menu_items = menu_
    
    return res


def handle_get_orders(req):
    global orders_
    
    print('Got an order request!')
    
    res = GetOrdersResponse()
    res.success = True
    res.orders = orders_
    
    return res


def handle_send_order(req):
    global orders_
    
    print('Got a new order!')
    
    res = SendOrderResponse()
    res.success = True
    
    # Add order to list
    new_order = Order()
    new_order.room = req.room
    for i in range(len(req.menu_items)):
        new_order.menu_items.append(get_menu_item_by_id(req.menu_items[i]))
    orders_.append(new_order)
    
    return res


def handle_send_goal(req):
    global orders_, home_, robot_state_, current_order_
    
    # print(req.room)
    # current_order_ = get_order_by_id(req.room)
    
    print('Got a new goal for room: ', req.room)
    
    # Define new goal
    # Check for home position (caffe)
    if req.room == 0:
        goal_pose = home_
        robot_state_ = 2
        current_order_ = None
    else:
        goal_pose = get_room_pose_by_nb(req.room)
        robot_state_ = 1
    
    res = SendGoalResponse()
    res.success = True
    
    # Publish robot goal
    publish_goal(goal_pose)
    
    return res


def publish_goal(goal_pose):
    global current_goal_, close_to_goal_
    
    # Simple Action Client
    sac = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # create goal
    goal = MoveBaseGoal()

    # use self?
    # set goal
    goal.target_pose.pose = goal_pose
    
    # Set current goal
    current_goal_ = np.array([goal_pose.position.x, goal_pose.position.y, goal_pose.position.z])
    
    # Set close to goal false
    close_to_goal_ = False
    
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()

    # start listner
    sac.wait_for_server()

    # send goal
    sac.send_goal(goal)

    # finish
    sac.wait_for_result()
    
    print('Published Goal Pose:  ', goal_pose.position.x, goal_pose.position.y)


def publish_robot_state():
    global pub_status, robot_state_
    
    robot_state = RobotState()
    robot_state.floor = robot_floor_
    robot_state.status_name = robot_states_[robot_state_]
    robot_state.status_id = robot_state_
    
    pub_status.publish(robot_state)


def publish_rooms_as_marker():
    global room_list_
    
    rooms = MarkerArray()

    for i in range(len(room_list_)):
        marker = Marker()
    
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'foodelpi_rooms'
        marker.id = i
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
    
        marker.scale.x = 0.2
        marker.scale.y = 0.5
        marker.scale.z = 0.1
        marker.color.a = 1.0

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        if i == 0:
            marker.color.r = 0.0
            marker.color.b = 1.0
        
        marker.pose.position.x = room_list_[i].pose.position.x
        marker.pose.position.y = room_list_[i].pose.position.y
        marker.pose.position.z = room_list_[i].pose.position.z
        marker.pose.orientation.x = room_list_[i].pose.orientation.x
        marker.pose.orientation.y = room_list_[i].pose.orientation.y
        marker.pose.orientation.z = room_list_[i].pose.orientation.z
        marker.pose.orientation.w = room_list_[i].pose.orientation.w
        
        rooms.markers.append(marker)

    pub_room_positions.publish(rooms)


def get_menu_item_by_id(id):
    global menu_
    for i in range(len(menu_)):
        if menu_[i].id == id:
            return menu_[i]


def get_room_pose_by_nb(nb):
    global room_list_
    for i in range(len(room_list_)):
        if room_list_[i].number == nb:
            return room_list_[i].pose


def get_order_by_id(id):
    global orders_
    for i in range(len(orders_)):
        if orders_[i].id == id:
            return orders_[i]


if __name__ == '__main__':
    ##### Read data from yaml #####
    read_menu_from_yaml()
    read_rooms_from_yaml()
    
    
    ##### ROS #####
    # Initialize this node
    rospy.init_node('foodelpi_server', anonymous=True)
    
    # Subscriber
    # sub_sensor_data = rospy.Subscriber('/map', OccupancyGrid, cb_map)
    sub_pose_data = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, cb_pose)
    sub_goal_status = rospy.Subscriber('/move_base/status', GoalStatusArray, cb_goal_status)

    # Publisher
    pub_status = rospy.Publisher('/foodelpi/robot_state', RobotState, queue_size=1)
    pub_room_positions = rospy.Publisher('/foodelpi/room_positions', MarkerArray, queue_size=1)
    
    # Services
    srv_menu = rospy.Service('/foodelpi/get_menu', GetMenu, handle_get_menu)
    srv_rooms = rospy.Service('/foodelpi/get_room_list', GetRoomList, handle_get_room_list)
    srv_orders = rospy.Service('/foodelpi/send_order', SendOrder, handle_send_order)
    srv_get_orders = rospy.Service('/foodelpi/get_orders', GetOrders, handle_get_orders)
    srv_send_goal = rospy.Service('/foodelpi/send_goal', SendGoal, handle_send_goal)

    print('Initialization completed. Waiting for requests...')

    # ROS spin
    # rospy.spin()
    
    rate = rospy.Rate(2) # 10hz
    while not rospy.is_shutdown():
        check_pos()
        publish_robot_state()
        publish_rooms_as_marker()
        rate.sleep()
