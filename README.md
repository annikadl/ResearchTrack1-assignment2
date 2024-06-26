# ResearchTrack1-assignment2
This is the second assignment for the Research Track 1 exam for the Robotics Engineering Master's degree at the University of Genoa.

The Sphinx documentation of this project is available at https://annikadl.github.io/ResearchTrack1-assignment2/.

If you prefer Doxygen documentation, move to the proper branch: it contains all the necessary files to generate it.  

## Simulator
The assignment takes place in Gazebo, a 3D ROS simulator. The Rviz tool is also used to provide a model of the simulator robot, including sensors, to help the user debug the robot application.

To run the environment, namely, the windows mentioned above Gazebo and Rviz, complete the following procedure (If you do not have already installed ROS on your computer, you can download and install the Noetic version used in this assignment at https://wiki.ros.org/noetic/Installation).

Steps:
* open at least two terminals.
* in the first one, move into your ROS workspace and run the ROS master with the command `roscore`.
* move to the second terminal and choose where to clone the repository
* clone this repository with the command `git clone _url_`.
* move into the directory
* run the command `roslaunch assignment_2_2023 assignment1.launch` to start the simulation.
At this point the simulator has started, the program is running and a mobile robot is spawned in the center of the playground.

N.B. If the scripts are not found, go to the `scripts` folder and run the command `chmod +x *.py`. Then try again to run `roslaunch assignment_2_2023 assignment1.launch`.

It is important to notice (this topic is also better explained later) that the launch file `assignment1.launch`, not only runs the environment but also all the nodes of the assignment. By knowing that, to run just a single node the command `rosrun assignment_2_2023 _NodeName.py_` must be used.

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
* an action client, allowing the user to set a target point or to cancel it (this node also uses the feedback/status of the action server to know when the target has been reached); this node also publishes robot position and velocity as a custom message.
* a service node that, when called, returns the coordinates of the last target sent by the user.
* a service node that subscribes to the robot's position and velocity and implements a server to retrieve the distance of the robot from the target and the robot's average speed.
To conclude, a launch file to start the whole simulation must be implemented. It also must include a parameter to select the size of the averaging window of the last node.

### set_target_client.py
`set_target_client.py` is the action client that satisfies the first request. It consists of different functions.
Besides, this node is run, by the launch file, in a separate terminal, allowing the user to directly access the set-target interface. 

#### set_client_parameters
This function implements an action client, which also provides a user interface, running on a separate terminal, to let the user choose from the terminal either to:
* set a new target point that the robot must reach.
* cancel the goal previously chosen.
  
Firstly, the action client is activated and waits for the server. Once the server is running too, the goal position is extracted from `assignment_2_2023.msg.PlanningGoal()`; The type of the goal is specified by the `Planning.action` action file, which, indeed, defines the types of the goal that the action client sends to the action servers and the result and feedback that the action server sends back to the client.

If the user chooses to set a new goal, the target values, both x and y positions, are taken from the keyboard and the script checks whether they are float or not; if not the user can insert them again. Then the values are stored in such `input_x` and `input_y` variables and the ros parameters `des_pos_x` and `des_pos_y` are updated too, by using `rospy.set_param('/des_pos_x', input_x)` and  `rospy.set_param('/des_pos_y', input_y)`.  In conclusion, the goal is sent to the server.

On the other hand, if the user chooses to cancel the goal, some singular situations must be handled. If the goal has never been entered or it has already been reached, it does not make sense to cancel it. To avoid this kind of scenario, two methods were implemented:
* the global variable `first_start` is used to check if the target has ever been set; if not it cannot be cancelled.
* the status of the goal is checked: if it is not active, namely that the status is either "succeded" or "preempted", the goal cannot be cancelled.
Both checks provide the user with a brief explanation by printing why the goal cannot be cancelled.
In all the other scenarios, the goal is correctly removed by using the command `client.cancel_goal()` and suddenly the robot stops. 

#### publisher_node
The `publisher_node` function is used to create and publish a custom message containing the actual position (x,y) and velocity (linear, angular) of the robot. This function represents the callback of a subscriber, which takes the required information subscribing to the topic `odom`.


### last_target_service.py
`last_target_service.py` is a node implementing a service that, when called, returns the values of the last target sent by the user. To make it feasible, a srv file `Last_target.srv` is created in the so-called directory; it contains the expected service response type.
The last target values are extracted from the ros parameters updated from the `set_target_client,py` and returned as response from the service. If the service is called before the user sets a target, the response gives the default values (`/des_pos_x = 0.0` and `des_pos_y = 1.0`) chosen in the launch file `assignment1.launch`. 

Furthermore, this service is run by the launch file; to call it and get the last target sent by the user run the command `rosservice call /last_target` on the terminal.

### info_service.py
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
[![](https://mermaid.ink/img/pako:eNptVcGO2jAQ_RXLl16AksACpVKrZWF76mmrVmqyQiYxxGpiR7bDLgX-vWM7EBvKAcJ45s2bN-PJEWcip3iO-_1-yjXTJZ0jRfVaE7mDn6xklOtBfUi59Uj5thRvWUGkRj8Wn1OO4KOazU6SukAVYdyZzOcxuQNC3SmHvIhxphkpmaL5qzuiPHcP7vsR9ftfUB35_xaR4XGT-i5V6jFZJCTTTHDUsmAKZaQsaY4Iz9EbYVqhrZAeO11Q1MYoKvdUogbUKRGzweZkT7Ql7WWJEiMA2ja8zQY5lAdaE0kqqqlULce18Q8xYkO1ISXaCfiCXJr8oYApRYWkUB6GB_wxp2pdC7V-7x4P6I3pAuV0S5pSoz0pG6qCXE_HpaDKltoZGygXFAlapYWRFxHHqbODYnCUEZ5Ro8zXs4--TAxziw6UmASxgRazwhAFna8bHdBZRklT56CqS2M6ExYceK-OvwqWFVahoAAbrDTRjQoJraLkR0Gvutoe0lfvPA7OudD_8VkfqLJu7SS54h2HG3VMAcYsxQaglBa1CpC2pFQ0Sr6ZKID5oNEGnGkl9jdz9Zy8wK1w8KB3MJavrU93I9wkuovisVlYQ-xbYmN68ixPJuoErT6hpdcXi7WM_E5Z07NH0eULpsuCOX1OaNUdBD2xcSuHZ8W9O4stzlXfAmaHlJKS_AB6Ud52AO7y6aqp31MLEoQbpQMEW_Al9sKxYxv0_lLnbRvv6-8KtSvNX2rXpVU3G9h9BZVrsw29RREl4KMyyWrba-g6LLbLUmqCMesuFczInpYiY_rgDVodJxkUq4E7QT_holZUKbLzp7oeJRcmbpJbF1Qd7CoB5DUg-xER1FubAapH1yJxD1dUwkrL4X1yNOYUA1pFUzyHx3YTpTjlZ3AljRYvB57huVWwh93lXzIC2lRXK82ZFvK7e0nZd1UP14T_FgJ8tGyo_YvnR_yO59FwNpgOZ6N4Molmo2j0EPfwAc9n40E8ng6n0WwcDacPk_jcw38twHAweZiO4-FsOpqMo0-T6ez8D_exG6c?type=png)](https://mermaid.live/edit#pako:eNptVcGO2jAQ_RXLl16AksACpVKrZWF76mmrVmqyQiYxxGpiR7bDLgX-vWM7EBvKAcJ45s2bN-PJEWcip3iO-_1-yjXTJZ0jRfVaE7mDn6xklOtBfUi59Uj5thRvWUGkRj8Wn1OO4KOazU6SukAVYdyZzOcxuQNC3SmHvIhxphkpmaL5qzuiPHcP7vsR9ftfUB35_xaR4XGT-i5V6jFZJCTTTHDUsmAKZaQsaY4Iz9EbYVqhrZAeO11Q1MYoKvdUogbUKRGzweZkT7Ql7WWJEiMA2ja8zQY5lAdaE0kqqqlULce18Q8xYkO1ISXaCfiCXJr8oYApRYWkUB6GB_wxp2pdC7V-7x4P6I3pAuV0S5pSoz0pG6qCXE_HpaDKltoZGygXFAlapYWRFxHHqbODYnCUEZ5Ro8zXs4--TAxziw6UmASxgRazwhAFna8bHdBZRklT56CqS2M6ExYceK-OvwqWFVahoAAbrDTRjQoJraLkR0Gvutoe0lfvPA7OudD_8VkfqLJu7SS54h2HG3VMAcYsxQaglBa1CpC2pFQ0Sr6ZKID5oNEGnGkl9jdz9Zy8wK1w8KB3MJavrU93I9wkuovisVlYQ-xbYmN68ixPJuoErT6hpdcXi7WM_E5Z07NH0eULpsuCOX1OaNUdBD2xcSuHZ8W9O4stzlXfAmaHlJKS_AB6Ud52AO7y6aqp31MLEoQbpQMEW_Al9sKxYxv0_lLnbRvv6-8KtSvNX2rXpVU3G9h9BZVrsw29RREl4KMyyWrba-g6LLbLUmqCMesuFczInpYiY_rgDVodJxkUq4E7QT_holZUKbLzp7oeJRcmbpJbF1Qd7CoB5DUg-xER1FubAapH1yJxD1dUwkrL4X1yNOYUA1pFUzyHx3YTpTjlZ3AljRYvB57huVWwh93lXzIC2lRXK82ZFvK7e0nZd1UP14T_FgJ8tGyo_YvnR_yO59FwNpgOZ6N4Molmo2j0EPfwAc9n40E8ng6n0WwcDacPk_jcw38twHAweZiO4-FsOpqMo0-T6ez8D_exG6c)

## Possible improvements
This project has several possible improvements.

First of all, as it is possible to notice, the robot moves quite slowly; its speed can be increased to achieve a faster and smoother simulation. However, the speed increase must be feasible also in the real world, namely, it is required to not exceed physical boundaries.

The provided user interface can be modified to become more user-friendly. It can be redesigned to be more attractive, for instance by presenting a more detailed and easy-to-use menu for the user. Besides, it is possible to build commands to use services directly from the same terminal where the desired position is entered. This would make it easier for the user to interact with the robot and the simulation. Until now just one terminal for the desired position is opened, while the services must be called separately on another terminal.

Furthermore, the desired goal, inserted by the user, should be graphically added to the playground. A goal marker would clarify where the goal is located and make it possible to visually check how the simulation is proceeding. 

In order to make the simulation even more user-friendly, it should be possible to choose the goal directly from the playground, by clicking on it. In addition, non-performable goals should not be eligible, such as points that do not belong to the playground or points belonging to the obstacles.
Besides, the possibility of choosing multiple goals to create a path that the robot must follow can be implemented too. 

When encountering obstacles, the robot "autonomously" chooses the direction along which follows the wall. To improve performance, the chosen direction should be the one that belongs to the shortest path to the goal. 

To conclude, although several improvements are possible, the project fully accomplishes the required tasks. 
