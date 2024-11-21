#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard
import sys

# turtlesim_teleoperation

# initializing the variables
speed = 1.0         # declaring linear speed (m/s)
turn_speed = 1.0    # declaring angular speed (rad/s)
move_bindings = {
    'f': (1, 0),  # moves the turtle forward
    'b': (-1, 0), # moves the turtle backward
    'l': (0, 1),  # rotates the turtle to the left
    'r': (0, -1), # rotates the turtle to the right
}

# initializing the node and publisher
rospy.init_node('turtlesim_teleoperation', anonymous=True)
pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
rate = rospy.Rate(10)  # 10 Hz

# creating a Twist message
twist = Twist()

# defining the key press functions
def on_press(key):
    global speed, turn_speed
    try:
        # check if the key is in movement bindings
        if key.char in move_bindings:
            linear, angular = move_bindings[key.char]
            twist.linear.x = linear * speed
            twist.angular.z = angular * turn_speed
            pub.publish(twist)
    except AttributeError:
        pass

# define key release function
def on_release(key):
    # stop the turtle once the key is released
    twist.linear.x = 0
    twist.angular.z = 0
    pub.publish(twist)
    # exit the program on ESC key
    if key == keyboard.Key.esc:
        return False

