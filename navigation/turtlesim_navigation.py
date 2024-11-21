#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

# Global variable to store the turtle's current pose
turtle_current_pose = None

# Callback function to update the turtle's current pose
def pose_callback(data):
    global turtle_current_pose
    turtle_current_pose = data

# Function to compute Euclidean distance between two points
def compute_distance(x1, y1, x2, y2):
    """Compute the distance between the current and target positions."""
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

# Function to compute the angle to the target
def compute_angle_to_target(current_x, current_y, target_x, target_y):
    """Compute the angle from the current position to the target."""
    return math.atan2(target_y - current_y, target_x - current_x)

# Main navigation function
def navigate_to_target(cmd_vel_pub, target_x, target_y):
    global turtle_current_pose

    # Define a tolerance for reaching the target
    tolerance = 0.2  # meters
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        if turtle_current_pose is None:
            rospy.loginfo("Waiting for pose data...")
            rospy.sleep(0.1)
            continue

        # Compute distance and angle to the target
        distance = compute_distance(
            turtle_current_pose.x, turtle_current_pose.y, target_x, target_y
        )
        angle_to_target = compute_angle_to_target(
            turtle_current_pose.x, turtle_current_pose.y, target_x, target_y
        )

        # Stop if within tolerance
        if distance < tolerance:
            rospy.loginfo("Target reached!")
            twist = Twist()
            cmd_vel_pub.publish(twist)  # Stop the turtle
            break

        # Normalize the angle difference
        angle_difference = math.atan2(
            math.sin(angle_to_target - turtle_current_pose.theta),
            math.cos(angle_to_target - turtle_current_pose.theta),
        )

        # Compute linear and angular velocities
        linear_velocity = 2.0 * distance if abs(angle_difference) < 0.5 else 0.0
        angular_velocity = 4.0 * angle_difference

        # Debugging information
        rospy.loginfo(f"Distance: {distance:.2f}, Angle Difference: {angle_difference:.2f}")
        rospy.loginfo(f"Linear Velocity: {linear_velocity:.2f}, Angular Velocity: {angular_velocity:.2f}")

        # Create and publish Twist message
        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity
        cmd_vel_pub.publish(twist)

        rate.sleep()

# Main entry point
if __name__ == '__main__':
    try:
        # Initialize ROS node
        rospy.init_node('autonomous_turtle_navigation', anonymous=True)

        # Publisher to send velocity commands
        cmd_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # Subscriber to /turtle1/pose topic
        rospy.Subscriber('/turtle1/pose', Pose, pose_callback)

        # Wait for pose data to be received
        rospy.loginfo("Waiting for turtle pose...")
        while turtle_current_pose is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

        # Print the current position of the turtle once
        rospy.loginfo(f"Current Position: x={turtle_current_pose.x:.2f}, y={turtle_current_pose.y:.2f}, theta={turtle_current_pose.theta:.2f}")

        # Input target coordinates
        target_x = float(input("Enter target x-coordinate (0-11): "))
        target_y = float(input("Enter target y-coordinate (0-11): "))

        # Validate input coordinates
        if not (0 <= target_x <= 11) or not (0 <= target_y <= 11):
            rospy.logerr("Target coordinates must be within the range [0, 11].")
        else:
            # Navigate to the target
            navigate_to_target(cmd_vel_pub, target_x, target_y)

    except rospy.ROSInterruptException:
        pass

