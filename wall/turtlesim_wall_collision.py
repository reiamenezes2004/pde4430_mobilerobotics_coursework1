#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

#turtlesim_wall_collision_code


# variable to store the turtle's current position
current_turtle_position = None

# turtlesim window boundaries for wall collision detection (11x11)
minimum_window_x = 0.5
maximum_window_x = 10.5
minimum_window_y = 0.5
maximum_window_y = 10.5

# threshold prior to the turtle hitting the wall (distance in meters)
PROXIMITY_THRESHOLD = 0.5  

starting_position_logged = False
wall_detected_logged = False

# callback function to update the turtle's current position
def update_position_callback(data):
    global current_turtle_position
    current_turtle_position = data

# checking for wall proximity
def turtle_near_wall(x, y):
    return x <= minimum_window_x + PROXIMITY_THRESHOLD or x >= maximum_window_x - PROXIMITY_THRESHOLD or \
           y <= minimum_window_y + PROXIMITY_THRESHOLD or y >= maximum_window_y - PROXIMITY_THRESHOLD


if __name__ == '__main__':
    try:
        rospy.init_node('wall_collision_detection', anonymous=True)

        # publisher to publish velocity commands
        publisher_cmd_vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # subscriber subscribes to /turtle1/pose topic
        rospy.Subscriber('/turtle1/pose', Pose, lambda x: None)

    except rospy.ROSInterruptException:
        pass
