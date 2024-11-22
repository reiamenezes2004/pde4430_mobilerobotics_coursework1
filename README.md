# pde4430_mobilerobotics_coursework1

Name = Reia Winola Merceline Menezes
MISIS = M00791121
Email = RM1661@live.mdx.ac.uk
Course = MSc Robotics
Professor = Dr. Sameer Kishore

Github Repository link:
https://github.com/reiamenezes2004/pde4430_mobilerobotics_coursework1.git

Demonstration Link:
https://youtu.be/bj-NYl1RYgs

How to run the tasks:
- Terminal1: run roscore
- Terminal2: run turtlesim turtlesim_node     (opens the turtlesim window)
- Terminal3: roslaunch pde4430_mobilerobotics_coursework1 <filename> eg: turtlesim_teleoperation.launch

Coursework Brief:
: perform 5 tasks with turtlesim. 

Package Organization:
pde4430_mobilerobotics_coursework1
├── CMakeLists.txt
├── launch
│   ├── turtlesim_navigation.launch
│   ├── turtlesim_teleoperation.launch
│   ├── turtlesim_vacuum_multiple.launch
│   ├── turtlesim_vacuum_single.launch
│   └── turtlesim_wall_collision.launch
├── navigation
│   └── turtlesim_navigation.py
├── package.xml
├── README.md
├── rqt_graph_images
│   ├── navigation.png
│   ├── teleoperation.png
│   ├── vacuummultiple.png
│   ├── vacuumsingle.png
│   └── wallcollision.png
├── teleoperation
│   └── turtlesim_teleoperation.py
├── vacuum_multiple
│   └── turtlesim_vacuum_multiple.py
├── vacuum_single
│   └── turtlesim_vacuum_single.py
└── wall
    └── turtlesim_wall_collision.py

7 directories, 18 files

Individual Task Explanation:

Task1 - teleoperation
: this task lets the user control the turtlebot using the 'f,b,l,r,s' keys, these stand for forward, backward, right, left and speed, respectively. Based on the key pressed the turtle acts accordingly, once key is released turtle will stop moving/rotating. Entering 's' will let the user input the speed of the turtle where giving the linear and angular speed the turtle will adapt and move at the new pace. 

Task2 - navigation
: this task asks the user to input the x and y co-ordinates, the turtle navigates to these co-ordinates entered from its current/starting position.

Task3- wall collision
: with the help of a proximity threshold, the turtle considers a certain boundary line as the 'wall' where if the turtle hits the wall the movement is overridden and the turtle stops moving.

Task4 - vacuum single
: in this task a turtle is spawned and using the wavefront coverage method the turtle 'vacuums' the window from top to bottom.

Task5 - vacuum multiple
: this task is a replica of the fourth task except instead of spawning 1 robot, 5 are spawned. The turtles have been given different pen 'line' colours for clarity purposes. rospy.Timer was used to ensure the robots vacuum simultaneously.
