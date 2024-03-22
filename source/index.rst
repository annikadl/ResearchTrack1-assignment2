.. assignment_2_2023 documentation master file, created by
   sphinx-quickstart on Sun Mar 17 11:48:39 2024.
   You can adapt this file completely to your liking, but it should at least
   contain the root toctree directive.

Welcome to assignment_2_2023's documentation!
=============================================

.. toctree::
   :maxdepth: 2
   :caption: Contents:

Indices and tables
==================

   * :ref:`genindex`
   * :ref:`modindex`
   * :ref:`search`

assignment_2_2023 documentation!
================================
This is the documentation of the assignment_2_2023 package!
This package was developed to fulfil the requirements for the the second 
assignment for the Research Track 1 exam for the Robotics Engineering 
Master's degree at the University of Genoa.

The assignment takes place in Gazebo, a 3D ROS simulator. 
The Rviz tool is also used to provide a model of the simulator robot, 
including sensors, to help the user debug the robot application.
To run the environment check the ReadMe file of the package repository
on github at the following link: https://github.com/annikadl/ResearchTrack1-assignment2 

Already implemented nodes
*************************
Some nodes were already implemented by the Professor of the course. 
These nodes aim to plan the motion of the robot.

Node wall_follow_service.py 
 is used to let the robot follow a wall. This node also processes data to 
 detect obstacles and avoid them: if an obstacle is detected on the 
 front, the node makes the robot rotate until there are no obstacles 
 perceived. The same behaviour is implemented for obstacles on the 
 right and the left.

Node go_to_point_service_service.py 
   implements a finite state machine 
   that controls whether the robot behaves correctly and lets it move 
   towards a specified point. It also checks if and when the robot 
   successfully reaches the goal.

Node bug_as.py 
   implements an action server used to decide the robot's 
   behaviour according to a function change_state, responsible for 
   switching the robot states. This function lets activate or deactivate 
   specific behaviour such as go_to_point or wall_following. Besides, it 
   imports messages and services communication between different components 
   of the simulation system. 
   These include messages for laser scan data, odometry information, twist 
   commands for velocity control, and services for switching between 
   navigation modes. Callback functions are implemented to process data, 
   updating the robot's position and orientation. The planning function is 
   used as a callback for the action server, implementing the robot's 
   goal-planning behaviour. It considers obstacles in the environment.


My contribution
================
The assignment requires handling long-running tasks: until now, when the 
robot is moving the user cannot do anything.

Set Target Client Module
*************************
.. automodule:: scripts.set_target_client
   :members:
   :show-inheritance:
   :undoc-members:
   

Last Target Service Module
***************************
.. automodule:: scripts.last_target_service
   :members:
   :show-inheritance:
   :undoc-members:


Info Service Module
*************************
.. automodule:: scripts.info_service
   :members:  
   :show-inheritance:
   :undoc-members:

