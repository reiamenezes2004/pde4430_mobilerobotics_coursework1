#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill
import math

#turtlesim_vacuum_single_code


current_turtle_position = None

# turtlesim window dimensions (11x11)
grid_dimensions = 1.0  # turtle will cover a cell space of 1m
minimum_window_x = 0.5
maximum_window_x = 10.5
minimum_window_y = 0.5
maximum_window_y = 10.5

# callback function to update the turtle's current position
def update_position_callback(data):
    global current_turtle_position
    current_turtle_position = data

def kill_default_turtle():
    rospy.wait_for_service('/kill')
    try:
        kill_turtle = rospy.ServiceProxy('/kill', Kill)
        kill_turtle("turtle1")
    except rospy.ServiceException as e:
        rospy.logwarn(f"Could not kill default turtle: {e}")

#  spawns the turtle at the bottom-left corner
def spawn_turtle_at_bottom_left_corner():
    rospy.wait_for_service('/spawn')
    try:
        spawn_turtle = rospy.ServiceProxy('/spawn', Spawn)
        spawn_turtle(minimum_window_x, minimum_window_y, 0, "turtle1")  # spawn directly at bottom-left corner (0.5, 0.5)
        rospy.loginfo("Turtle spawned at bottom-left corner.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to spawn turtle: {e}")
