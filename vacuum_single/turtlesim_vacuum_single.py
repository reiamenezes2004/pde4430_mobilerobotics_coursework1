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
