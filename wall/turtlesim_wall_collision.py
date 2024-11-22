#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

if __name__ == '__main__':
    try:
        rospy.init_node('wall_collision_detection', anonymous=True)

        # publisher to publish velocity commands
        publisher_cmd_vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # subscriber subscribes to /turtle1/pose topic
        rospy.Subscriber('/turtle1/pose', Pose, lambda x: None)

    except rospy.ROSInterruptException:
        pass
