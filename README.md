# ResearchTrack1-assignment2
This is the second assignment for the Research Track 1 exam for the Robotics Engineering Master's degree at the University of Genoa.

## Simulator
The assignemnt takes place in Gazebo, a 3D ROS simulator. The Rviz tool is also used to provide a model of the simulator robot, including sensors, to help the user debug the robot application.

To run the environment, namely, the windows mentioned above Gazebo and Rviz, complete the following procedure (If you do not have already installed ROS on your computer, you can download and install the Noetic version used in this assignment at https://wiki.ros.org/noetic/Installation).
Steps:
* open at least two terminals.
* in the first one, move into your ROS workspace and run the master with the command `roscore`.
* move to the second terminal and choose where to clone the repository
* clone this repository with the command `git clone _url_`.
* move into the directory
* run the command `roslaunch assignment_2_2023 assignment1.launch` to start the simulation.
At this point the simulator has started, the program is running and a mobile robot is spawned in the center of the playground.

It is important to notice (this topic is also better explained later) that the launch file 'assignment1.launch`, not only runs the environment but also all the nodes of the assignment`. By knowing that, to run just a single node the command `rosrun assignment_2_2023 _NodeName.py_` must be used.

The playground is represented by a squared arena in which the robot can move. Besides, some walls around and inside the playground obstruct the robot's motion.

## Already implemented nodes
Some nodes were already implemented by the Professor of the course. These nodes aim to plan the motion of the robot.

The first node `reading_laser.py` processes the output of the sensors, a necessary step to correctly move the robot around the arena. It converts the 720 readings contained inside the LaserScan msg into five distinct readings. Each reading is the minimum distance measured on a sector of 60 degrees.

Node `obstacle_avoidance.py` processes data to detect obstacles and avoid them: if an obstacle is detected on the front, the node makes the robot rotate until there are no obstacles perceived. The same behaviour is implemented for obstacles on the right and the left. **CHECK PERCHé POTREBBE ESSERE STATA INGLOABATA DA BUSAS**

Node `wall_follow_service.py` is used to let the robot follow a wall, for instance, to circumnavigate it.

Node `go_to_point_service_service.py` implements a finite state machine that controls whether the robot behaves correctly and lets it move towards a specified point. It also checks if and when the robot successfully reaches the goal.

Node `bug_as.py` ...

**FINIRE** notare anche che il package assignemnt fa la stessa cosa di questo ma come action server, quindi modificare opportunamente.

## The assignment
The assignment requires handling long-running tasks: until now, when the robot is moving the user cannot do anything. 
The nodes that must be implemented are:
* an action client, allowing the user to set a target point or to cancel it; this node also publishes robot position and velocity as a custom message.
* a service node that, when called, returns the coordinates of the last target sent by the user.
* a service node that subscribes to the robot's position and velocity and implements a server to retrieve the distance of the robot from the target and the robot's average speed.
To conclude, a launch file to start the whole simulation must be implemented. It also must include a parameter to select the size of the averaging window of the last node.

### set_target_client

### last_target_service

### info_service

### assignment1.launch


