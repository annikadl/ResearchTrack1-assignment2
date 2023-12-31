# ResearchTrack1-assignment2
This is the second assignment for the Research Track 1 exam for the Robotics Engineering Master's degree at the University of Genoa.

## Simulator
The assignment takes place in Gazebo, a 3D ROS simulator. The Rviz tool is also used to provide a model of the simulator robot, including sensors, to help the user debug the robot application.

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

Node `wall_follow_service.py` is used to let the robot follow a wall, for instance, to circumnavigate it. This node also processes data to detect obstacles and avoid them: if an obstacle is detected on the front, the node makes the robot rotate until there are no obstacles perceived. The same behaviour is implemented for obstacles on the right and the left.

Node `go_to_point_service_service.py` implements a finite state machine that controls whether the robot behaves correctly and lets it move towards a specified point. It also checks if and when the robot successfully reaches the goal.

Node `bug_as.py` is an action server used to decide the robot's behaviour according to a function `change_state`, responsible for switching the robot states. This function lets activate or deactivate specific behaviour such as `go_to_point` or `wall_following`. Besides, it imports messages and services, also from the other nodes, to facilitate communication between different components of the simulation system. These include messages for laser scan data, odometry information, twist commands for velocity control, and services for switching between navigation modes. Callback functions are implemented to process data, updating the robot's position and orientation. The `planning` function is used as a callback for the action server, implementing the robot's goal-planning behaviour. It considers obstacles in the environment.

Besides, a launch file `assignment1.launch` to run the whole simulation, both scripts and environment, is provided.

## The assignment
The assignment requires handling long-running tasks: until now, when the robot is moving the user cannot do anything. 
The nodes that must be implemented are:
* an action client, allowing the user to set a target point or to cancel it; this node also publishes robot position and velocity as a custom message.
* a service node that, when called, returns the coordinates of the last target sent by the user.
* a service node that subscribes to the robot's position and velocity and implements a server to retrieve the distance of the robot from the target and the robot's average speed.
To conclude, a launch file to start the whole simulation must be implemented. It also must include a parameter to select the size of the averaging window of the last node.

### set_target_client
`set_target_client.py` is the action server that satisfies the first request. It implements a "graphic user interface" to let the user choose from the terminal either to:
* set a new target point that the robot must reach.
* maintain the same target as before **CHECK SE HA VERAMENTE SENSO OPPURE N0, ad esempio se il punto in cui viene spawnato il robot non corrisponde al target pre impostato (potrei cambiarlo a manina eventualmente) potrebbe avere senso**
* cancel the goal previously chosen.

The goal position is extracted from `assignment_2_2023.msg.PlanningGoal()`. The type of the goal is specified by the `Planning.action` action file, which, indeed, defines the types of the goal that the action client sends to the action servers and the result and feedback that the action server sends back to the client.

When the target values, both x and y positions, are updated from the user and stored in such `input_x` and `input_y` variables, the ros parameters `des_pos_x` and `des_pos_y` are updated too, by using `rospy.set_param('/des_pos_x', input_x)` and  `rospy.set_param('/des_pos_y', input_y)`.

**SPIEGARE MAGARI UN PO' MEGLIO**
Besides, the custom message, which contains velocity and position parameters, is created and sent by a publisher `pub` and subscribed from subscriber `sub_from_Odom`, which gets from "Odom" the two parameters.

This client node is run by the launch file in a separate terminal, allowing the user to access directly to the set-target interface. 

### last_target_service
`last_target_service.py` is a node implementing a service that, when called, returns the values of the last target sent by the user. To make it feasible, a srv file `Last_target.srv` is created in the so-called directory; it contains the expected service response type.
The last target values are extracted from the ros parameters updated from the `set_target_client,py` and returned as response from the service. If the service is called before the user sets a target, the response gives the default values (`/des_pos_x = 0.0` and `des_pos_y = 1.0`) chosen in the launch file `assignment1.launch`. 

Furthermore, this service is run by the launch file; to call it and get the last target sent by the user run the command `rosservice call /last_target` on the terminal.

### info_service
`info_service,py` is a node implementing a service that, when called, returns the distance from the goal and the average velocity of the robot. To make it feasible, a srv file `info_service.srv` is created in the so-called directory; it contains the expected service response type. A subscriber is implemented and its callback takes the target position from the ros parameters and the actual one from the custom message sent by the `set_target_client.py`. The distance is computed as the Euclidean distance by using the Python built-in function `math.dist(des_coordinates, actual coordinates)` by importing `math` library. The velocity values are extracted by the custom message too, they are collected in a list of dimension `window_size` with a default value of 10 (that can be modified in the launch file). Then the average velocity is computed as `average_vel_x = sum(vel_data) / min(len(vel_data), velocity_window_size)`. These values compose the response of the service.

As for `last_target_service.py`, `info_service.py` is run by the launch file and the service can be called by using the command `rosservice call /info_service` on the terminal.

### assignment1.launch
`assignment1.launch` is the launch file that provides the possibility of running the whole simulation, including both the scripts and the environment, by using just one command: `roslaunch assignment_2_2023 assignment1.launch`.

In the launch file:
* the ros parameters `/des_pos_x`, `/des_pos_y` and `/window_size` are defined and can be easily modified.
* Gazebo and Rviz tools are launched.
* All the scripts previously mentioned are launched too.
* The `set_target_client.py` is launched in a different terminal, to allow the user to easily set the target, by adding `launch-prefix="lxterminal -e"` to the respective node tag.

## Flowchart


## Possible improvements
Several improvements are possible:
* increase the robot's velocity, to make the simulation smoother.
* improving the user interface by:
    * making it more attractive;
    * making it possible to also call the service with a specific command


