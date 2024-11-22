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

# generate grid points for vacuuming the window using the wavefront coverage method
def create_grid_points():
    grid_points = []
    x = minimum_window_x
    while x <= maximum_window_x:
        y = minimum_window_y
        while y <= maximum_window_y:
            grid_points.append((x, y))
            y += grid_dimensions
        x += grid_dimensions
    return grid_points

# method for wavefront coverage
def window_wavefront_vacuum(publisher_cmd_vel):
    global current_turtle_position

    grid_points = create_grid_points()

    for target_x, target_y in grid_points:
        rospy.loginfo(f"Moving to grid point x={target_x}, y={target_y}")
        move_to_target(publisher_cmd_vel, target_x, target_y)

    rospy.loginfo("Window Vacuuming complete!")


if __name__ == '__main__':
    try:
        rospy.init_node('window_wavefront_vacuum', anonymous=True)

        # kills the default turtle - done to avoid renames 
        kill_default_turtle()

        spawn_turtle_at_bottom_left_corner()

        # publisher to send velocity commands
        publisher_cmd_vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # subscriber to /turtle1/pose topic
        rospy.Subscriber('/turtle1/pose', Pose, update_position_callback)

        while current_turtle_position is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

        rospy.loginfo(f"Current Position: x={current_turtle_position.x:.2f}, y={current_turtle_position.y:.2f}")

        # start wavefront-based window coverage
        window_wavefront_vacuum(publisher_cmd_vel)

    except rospy.ROSInterruptException:
        pass
