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

# calculates the distance to a target position
def calculate_target_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

#  moves the turtle to a target position
def move_to_target(publisher_cmd_vel, target_x, target_y):
    global current_turtle_position

    rate = rospy.Rate(10)  
    threshold_distance = 0.1  

    while not rospy.is_shutdown():
        if current_turtle_position is None:
            rospy.sleep(0.1)
            continue

        # distance and angle to the target
        distance = calculate_target_distance(
            current_turtle_position.x, current_turtle_position.y, target_x, target_y
        )
        angle_to_target = math.atan2(target_y - current_turtle_position.y, target_x - current_turtle_position.x)

        # turtle stops if it is within the tolerance distance
        if distance < threshold_distance:
            rospy.loginfo(f"Reached target at x={target_x}, y={target_y}")
            twist = Twist()
            publisher_cmd_vel.publish(twist)  # turtle stop
            break

        # calculate the angular difference
        angle_difference = angle_to_target - current_turtle_position.theta
        angle_difference = math.atan2(math.sin(angle_difference), math.cos(angle_difference))

        twist = Twist()
        twist.linear.x = 8.0 * distance if abs(angle_difference) < 0.1 else 0.0  # increase linear speed of the turtle
        twist.angular.z = 8.0 * angle_difference  # increase turning speed of the turtle

        # publish Twist message
        publisher_cmd_vel.publish(twist)
        rate.sleep()

