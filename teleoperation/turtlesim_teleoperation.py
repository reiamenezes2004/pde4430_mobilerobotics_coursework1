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

                # asks the user for speed input when 's' is pressed
        elif key.char == 's':
            sys.stdout.write("\r")  # clear the line where 's' was printed. resolves the alphanumeric issue
            sys.stdout.flush()
            prompt_for_speed()
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

# function to ask the user for speed input
def prompt_for_speed():
    global speed, turn_speed
    try:
        # ask user to enter linear speed
        print("Enter the new turtle speeds:")
        new_speed = float(input("Linear speed (m/s): ").strip())
        # ask user to enter angular speed
        new_turn_speed = float(input("Angular speed (rad/s): ").strip())

        # update global speed variables
        speed = new_speed
        turn_speed = new_turn_speed
        print(f"Updated new turtle speed: Linear = {speed} m/s, Angular = {turn_speed} rad/s")
    except ValueError:
        print("Invalid input. Please enter numeric values only.")

# main function to handle turtlesim teleoperation
def teleop_turtle():
    print("Control Your Turtle! Press 'f', 'b', 'l', 'r' to move the turtle.")
    print("Press 's' to set new speed values for linear and angular speed.")
    print("Press 'esc' to exit the program.")

    # keyboard listener
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()
