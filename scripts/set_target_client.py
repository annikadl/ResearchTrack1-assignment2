#!/usr/bin/env python
"""
.. module:: set_target_client
   :platform: Unix
   :synopsis: Python module for the assignment_2_2023
.. moduleauthor:: Annika Delucchi

Description:
    This module implements an action client for setting target points and cancelling goals in the assignment_2_2023 package. It also publishes the robot's position and velocity as a custom message.
    It provides the following functions:
    - :func:`publisher_node`: Callback function for the subscriber to the topic /odom.
    - :func:`parameters_client_main`: Implements an action client with a user interface for setting new target points or cancelling previous goals.
    - :func:`main`: Entry point of the module.

Action Client:
    /reaching_goal

Publisher:
    /pos_vel

Subscriber:
    /odom 
"""

import rospy
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry
import actionlib
import actionlib.msg
import assignment_2_2023.msg
from assignment_2_2023.msg import Vel
from assignment_2_2023.msg import PlanningAction, PlanningGoal, PlanningResult, PlanningActionResult
from std_srvs.srv import SetBool
from actionlib_msgs.msg import GoalStatus

pub = None
first_start = 0

def publisher_node(msg):
    """
    Callback function for the subscriber to the topic /odom. 
    It takes the actual position and velocity of the robot and publishes them as a custom message.
    
    :param msg: message received from the topic /odom
    :type msg: nav_msgs.msg.Odometry
    """
    global pub

    actual_pos = msg.pose.pose.position
    actual_vel_linear = msg.twist.twist.linear
    actual_vel_angular = msg.twist.twist.angular

    my_pos_and_vel = Vel()
    my_pos_and_vel.pos_x = actual_pos.x
    my_pos_and_vel.pos_y = actual_pos.y
    my_pos_and_vel.vel_x = actual_vel_linear.x
    my_pos_and_vel.vel_z = actual_vel_angular.z

    pub.publish(my_pos_and_vel)

def parameters_client_main():
    """
    Implements an action client, which also provides a user interface, running on a separate terminal, 
    to let the user choose from the terminal either to set a new target point that the robot must reach 
    or to cancel the goal previously chosen.

    :param des_pos_x: desired position x
    :param des_pos_y: desired position y
    :type des_pos_x: float
    :type des_pos_y: float
    
    """
    global first_start

    client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2023.msg.PlanningAction)
    client.wait_for_server()

    while not rospy.is_shutdown():
        rospy.loginfo("\n Hello! This is the simulation menu! \n PRESS: \n y - to set a new goal \n c - to cancel the last goal \n")

        command = input("What do you want to do?: ")

        goal = assignment_2_2023.msg.PlanningGoal()
        goal.target_pose.pose.position.x = rospy.get_param('/des_pos_x')
        goal.target_pose.pose.position.y = rospy.get_param('/des_pos_y')
        
        if command == 'y':
            while True:
                input_x = input("Enter desired position x: ")
                try:
                    input_x = float(input_x)
                    break
                except:
                    print("Invalid input, enter a number")

            while True:
                input_y = input("Enter desired position y: ")
                try:
                    input_y = float(input_y)
                    break
                except:
                    print("Invalid input, enter a number")

            rospy.set_param('/des_pos_x', input_x)
            rospy.set_param('/des_pos_y', input_y)

            goal.target_pose.pose.position.x = input_x
            goal.target_pose.pose.position.y = input_y

            client.send_goal(goal)
            rospy.loginfo("Inserted goal: des_x = %f, des_y = %f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)
            
            first_start = 1 

        elif command == 'c':
            if first_start == 0:
                rospy.loginfo("Goal has not been already set, cannot cancel it")
            elif client.get_state() == actionlib.GoalStatus.ACTIVE:
                client.cancel_goal()
                rospy.loginfo("Goal cancelled")    
            else:
                rospy.loginfo("Goal already reached, cannot cancel it")

        else:
            rospy.loginfo("Invalid input")

def main():
    """
    Initializes the set_target_client node, sets up publishers and subscribers, and runs the action client.
    """
    rospy.init_node('set_target_client')

    global pub

    pub = rospy.Publisher("/pos_vel", Vel, queue_size=1)

    sub_from_Odom = rospy.Subscriber("/odom", Odometry, publisher_node)

    parameters_client_main()

if __name__ == '__main__':
    main()
