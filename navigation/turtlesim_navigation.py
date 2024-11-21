#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

# Global variable to store the turtle's current pose
turtle_current_pose = None

# Callback function to update the turtle's current pose
def pose_callback(data):
    global turtle_current_pose
    turtle_current_pose = data

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

    except rospy.ROSInterruptException:
        pass
