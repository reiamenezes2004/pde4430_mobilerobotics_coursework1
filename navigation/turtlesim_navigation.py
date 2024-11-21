#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

# variable to store the turtle's current position
turtle_current_pose = None

# callback function to update the turtle's current position
def pose_callback(data):
    global turtle_current_pose
    turtle_current_pose = data


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
        # else:
            # Navigate to the target
            # navigate_to_target(cmd_vel_pub, target_x, target_y)

    except rospy.ROSInterruptException:
        pass