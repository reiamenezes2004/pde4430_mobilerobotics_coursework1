#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

# variable to store the turtle's current position
current_turtle_position = None

# callback function to update the turtle's current position
def update_position_callback(data):
    global current_turtle_position
    current_turtle_position = data

# function to calculate the distance between the two points
def calculate_distance(x_start, y_start, x_target, y_target):
    """Compute the distance between the current and target positions."""
    return math.sqrt((x_target - x_start)**2 + (y_target - y_start)**2)

# function to compute the angle to the target
def calculate_angle_to_target(curr_x, curr_y, coord_x, coord_y):
    """Compute the angle from the current position to the target."""
    return math.atan2(coord_y - curr_y, coord_x - curr_x)

# main navigation function
def go_to_new_target(publisher_cmd_vel, coord_x, coord_y):
    global current_turtle_position

    tolerance_distance = 0.2  # meters
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        if current_turtle_position is None:
            rospy.loginfo("Waiting for position data...")
            rospy.sleep(0.1)
            continue

        # compute distance and angle to the target
        remaining_distance = calculate_distance(
            current_turtle_position.x, current_turtle_position.y, coord_x, coord_y
        )
        angle_to_goal = calculate_angle_to_target(
            current_turtle_position.x, current_turtle_position.y, coord_x, coord_y
        )

        # stop if within tolerance
        if remaining_distance < tolerance_distance:
            rospy.loginfo("Target reached!")
            twist = Twist()
            publisher_cmd_vel.publish(twist)  # Stop the turtle
            break

        # normalize the angle difference
        angle_difference = math.atan2(
            math.sin(angle_to_goal - current_turtle_position.theta),
            math.cos(angle_to_goal - current_turtle_position.theta),
        )

        # compute linear and angular velocities
        velocity_linear = 2.0 * remaining_distance if abs(angle_difference) < 0.5 else 0.0
        velocity_angular = 4.0 * angle_difference

        # debugging
        rospy.loginfo(f"Distance: {remaining_distance:.2f}, Angle Difference: {angle_difference:.2f}")
        rospy.loginfo(f"Linear Velocity: {velocity_linear:.2f}, Angular Velocity: {velocity_angular:.2f}")

        # creating and publishing the twist message
        twist = Twist()
        twist.linear.x = velocity_linear
        twist.angular.z = velocity_angular
        publisher_cmd_vel.publish(twist)

        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('autonomous_turtle_navigation', anonymous=True)

        # publisher to send velocity commands
        publisher_cmd_vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # subscriber subscribes to /turtle1/pose topic
        rospy.Subscriber('/turtle1/pose', Pose, update_position_callback)

        rospy.loginfo("Waiting for the current turtle position...")
        while current_turtle_position is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

        rospy.loginfo(f"Current Position of the turtle: x={current_turtle_position.x:.2f}, y={current_turtle_position.y:.2f}, theta={current_turtle_position.theta:.2f}")

        # asking the user for coordinates of x and y
        coordinate_x = float(input("Enter target x-coordinate (0-11): "))
        coordinate_y = float(input("Enter target y-coordinate (0-11): "))

        # check the input coordinates to ensure it is within the range
        if not (0 <= coordinate_x <= 11) or not (0 <= coordinate_y <= 11):
            rospy.logerr("Target coordinates must be within the range [0, 11].")
        else:
            # go to the target
            go_to_new_target(publisher_cmd_vel, coordinate_x, coordinate_y)

    except rospy.ROSInterruptException:
        pass
