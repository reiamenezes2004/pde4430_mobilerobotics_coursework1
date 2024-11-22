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

# move the robot to a target position
def move_to_target(robot_name, publisher_cmd_vel, target_x, target_y):
    global robot_spawning_positions

    rate = rospy.Rate(10)
    tolerance = 0.05  # distance to consider target reached

    while not rospy.is_shutdown():
        if robot_name not in robot_spawning_positions:
            rospy.sleep(0.1)
            continue

        current_position = robot_spawning_positions[robot_name]
        distance = math.sqrt((target_x - current_position.x)**2 + (target_y - current_position.y)**2)
        angle_to_target = math.atan2(target_y - current_position.y, target_x - current_position.x)

        if distance < tolerance:
            twist = Twist()
            publisher_cmd_vel.publish(twist)  # stop the robot
            return

        angle_difference = angle_to_target - current_position.theta
        angle_difference = math.atan2(math.sin(angle_difference), math.cos(angle_difference))

        twist = Twist()
        twist.linear.x = 7.0 * distance if abs(angle_difference) < 0.2 else 0.0  # increase the speed of the turtle
        twist.angular.z = 8.0 * angle_difference
        publisher_cmd_vel.publish(twist)
        rate.sleep()

# make grid points for each robot
def generate_robot_area(robot_index, num_robots):
    grid_points = []
    x_start = minimum_window_x + (robot_index * (maximum_window_x - minimum_window_x) / num_robots)
    x_end = x_start + (maximum_window_x - minimum_window_x) / num_robots

    x = x_start
    while x < x_end:
        y = minimum_window_x
        while y <= maximum_window_y:
            grid_points.append((x, y))
            y += grid_dimensions
        x += grid_dimensions
    return grid_points
