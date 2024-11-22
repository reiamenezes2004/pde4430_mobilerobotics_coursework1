#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill, SetPen
import math

# turtlesim_vaccum_multiple_code

# global Variables
robot_spawning_positions = {}
grid_dimensions = 0.5  # decided grid size for better coverage - avoids wall collision
minimum_window_x = 0.3
maximum_window_x = 10.6
minimum_window_y = 0.3
maximum_window_y = 10.8
turtle_line_colours = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (0, 255, 255)]  # line colours for 5 robots

publishers = {}
grid_points_assignments = {}
progress_counters = {}
turtle_completed = {}

# callback to update the robot's position
def update_position_callback(data, robot_name):
    global robot_spawning_positions
    robot_spawning_positions[robot_name] = data
