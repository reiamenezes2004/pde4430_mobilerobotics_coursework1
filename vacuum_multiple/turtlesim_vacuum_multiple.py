#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill, SetPen
import math

# turtlesim_vaccum_multiple_code


# global Variables
robot_spawning_positions = {}
grid_dimensions = 0.5  # decided grid size for better coverage - avoids wall collision
minimum_window_x = 0.3
maximum_window_x = 10.6
minimum_window_y = 0.3
maximum_window_y = 10.8
turtle_line_colours = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (0, 255, 255)]  # line colours for 5 robots

publishers = {}
grid_points_assignments = {}
progress_counters = {}
turtle_completed = {}

# callback to update the robot's position
def update_position_callback(data, robot_name):
    global robot_spawning_positions
    robot_spawning_positions[robot_name] = data

def move_robot_to_next_target(event, robot_name):
    global grid_points_assignments, progress_counters, robot_spawning_positions, turtle_completed

    if robot_name not in robot_spawning_positions:
        return  

    # check if the robot has completed its tasks
    if turtle_completed[robot_name]:
        return

    counter = progress_counters[robot_name]
    if counter >= len(grid_points_assignments[robot_name]):
        rospy.loginfo(f"{robot_name} has finished its vacuuming. Moving the turtle to the center.")
        move_to_target(robot_name, publishers[robot_name], 5.5, 5.5)  # once completed move to center
        rospy.loginfo(f"{robot_name} is now at the center.")
        turtle_completed[robot_name] = True  # marks robot as completed vacuuming
        return

    target_x, target_y = grid_points_assignments[robot_name][counter]
    progress_counters[robot_name] += 1
    rospy.loginfo(f"{robot_name} moving to grid point x={target_x}, y={target_y}")
    move_to_target(robot_name, publishers[robot_name], target_x, target_y)

# move the robot to a target position
def move_to_target(robot_name, publisher_cmd_vel, target_x, target_y):
    global robot_spawning_positions

    rate = rospy.Rate(10)
    tolerance = 0.05  # distance to consider target reached

    while not rospy.is_shutdown():
        if robot_name not in robot_spawning_positions:
            rospy.sleep(0.1)
            continue

        current_position = robot_spawning_positions[robot_name]
        distance = math.sqrt((target_x - current_position.x)**2 + (target_y - current_position.y)**2)
        angle_to_target = math.atan2(target_y - current_position.y, target_x - current_position.x)

        if distance < tolerance:
            twist = Twist()
            publisher_cmd_vel.publish(twist)  # stop the robot
            return

        angle_difference = angle_to_target - current_position.theta
        angle_difference = math.atan2(math.sin(angle_difference), math.cos(angle_difference))

        twist = Twist()
        twist.linear.x = 7.0 * distance if abs(angle_difference) < 0.2 else 0.0  # increase the speed of the turtle
        twist.angular.z = 8.0 * angle_difference
        publisher_cmd_vel.publish(twist)
        rate.sleep()

# make grid points for each robot
def generate_robot_area(robot_index, num_robots):
    grid_points = []
    x_start = minimum_window_x + (robot_index * (maximum_window_x - minimum_window_x) / num_robots)
    x_end = x_start + (maximum_window_x - minimum_window_x) / num_robots

    x = x_start
    while x < x_end:
        y = minimum_window_x
        while y <= maximum_window_y:
            grid_points.append((x, y))
            y += grid_dimensions
        x += grid_dimensions
    return grid_points

# spawn 5 robots along the bottom of the window 
def spawn_robots_at_bottom_of_the_window():
    spawn_points = [
        (minimum_window_x + 0.5, minimum_window_y + 0.5),
        (minimum_window_x + 3.0, minimum_window_y + 0.5),
        (minimum_window_x + 5.0, minimum_window_y + 0.5),
        (minimum_window_x + 7.0, minimum_window_y + 0.5),
        (minimum_window_x + 9.0, minimum_window_y + 0.5),
    ]
    robots = []
    rospy.wait_for_service('/spawn')

    for i in range(5):  # spawing the five robots
        robot_name = f"turtle{i+1}"
        spawn_x, spawn_y = spawn_points[i]
        try:
            spawn_turtle = rospy.ServiceProxy('/spawn', Spawn)
            spawn_turtle(spawn_x, spawn_y, 0, robot_name)
            robots.append(robot_name)
            rospy.loginfo(f"Spawned {robot_name} at x={spawn_x}, y={spawn_y}")

            # adding a unique line color for each robot for better clarity
            set_robot_line_colour(robot_name, turtle_line_colours[i][0], turtle_line_colours[i][1], turtle_line_colours[i][2], 3, 0)
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to spawn {robot_name}: {e}")

    return robots

def set_robot_line_colour(robot_name, r, g, b, width, off):
    rospy.wait_for_service(f'/{robot_name}/set_pen')
    try:
        set_pen = rospy.ServiceProxy(f'/{robot_name}/set_pen', SetPen)
        set_pen(r, g, b, width, off)
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to set pen for {robot_name}: {e}")

# check if all turtles have completed their designated vacuuming areas
def all_robots_finished_vacuuming():
    for robot_name in turtle_completed:
        if not turtle_completed[robot_name]:
            return False
    return True

if __name__ == '__main__':
    try:
        rospy.init_node('multi_robot_wavefront_vacuum', anonymous=True)

        # kill the default turtle - center
        rospy.wait_for_service('/kill')
        try:
            kill_turtle = rospy.ServiceProxy('/kill', Kill)
            kill_turtle("turtle1")
        except rospy.ServiceException:
            rospy.logwarn("Default turtle already killed.")

        # spawn robots along the bottom of the window
        robots = spawn_robots_at_bottom_of_the_window()

        # initialize publishers, subscribers, and grid assignments
        for i in range(5):  # five robots
            robot_name = f"turtle{i+1}"
            publishers[robot_name] = rospy.Publisher(f'/{robot_name}/cmd_vel', Twist, queue_size=10)
            rospy.Subscriber(f'/{robot_name}/pose', Pose, update_position_callback, robot_name)
            grid_points_assignments[robot_name] = generate_robot_area(i, 5)
            progress_counters[robot_name] = 0  # initialize counter for tracking
            turtle_completed[robot_name] = False  # initialize completion flags for the robots

            # timers for simultaneous vacuuming of the turtles
            rospy.Timer(rospy.Duration(0.1), lambda event, name=robot_name: move_robot_to_next_target(event, name))

        # checking again if all robots have finished
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if all_robots_finished_vacuuming():
                rospy.loginfo("All robots have finished vacuuming and reached the center. Multiple robot vacuuming is now complete.")
                break
            rate.sleep()

    except rospy.ROSInterruptException:
        pass