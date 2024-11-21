#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

# initializing the node and publisher
rospy.init_node('turtlesim_teleoperation', anonymous=True)
pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
rate = rospy.Rate(10)  # 10 Hz

# creating a Twist message
twist = Twist()
