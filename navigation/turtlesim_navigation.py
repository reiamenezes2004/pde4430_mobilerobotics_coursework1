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

if __name__ == '__main__':
    try:
        # initialising ROS node
        rospy.init_node('autonomous_turtle_navigation', anonymous=True)

        # publisher publishes velocity commands
        cmd_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # subscriber subscribes to /turtle1/pose topic
        rospy.Subscriber('/turtle1/pose', Pose, pose_callback)

        rospy.loginfo("Waiting for the current turtle position...")
        while turtle_current_pose is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

    # Print the current position of the turtle once
        rospy.loginfo(f"Current Position of the turtle: x={turtle_current_pose.x:.2f}, y={turtle_current_pose.y:.2f}, theta={turtle_current_pose.theta:.2f}")


    except rospy.ROSInterruptException:
        pass
