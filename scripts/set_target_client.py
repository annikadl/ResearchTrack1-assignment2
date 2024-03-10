## @package assignment_2_2023
# \file set_target_client.py
# \brief Action server that allows the user to set a target point or to cancel it
# \author Annika Delucchi
# \version 1.0
# \date    9/03/2024
#
# \details
#
# Subscribes to: <BR>
#    ° /odom: topic to get the actual position and velocity of the robot
#
# Publishes to: <BR>
#    ° /pos_vel: custom message containing the actual position and velocity of the robot
#
# Services: <BR>
#    ° None


#!/usr/bin/env python

"""
This is the action server that satisfies the request: implement an action client, 
allowing the user to set a target point or to cancel it (this node also uses the 
status of the goal to know when the target has been reached); 
this node also publishes robot position and velocity as a custom message.

It implements different functions.

set_client_parameters() implements an action client, which also provides a user 
interface, running on a separate terminal, to let the user choose from the terminal either to:
* set a new target point that the robot must reach.
* cancel the goal previously chosen.

The publisher_node() function is used to create and publish a custom message containing 
the actual position (x,y) and velocity (linear, angular) of the robot. This function 
represents the callback of a subscriber, which takes the required information subscribing to the topic `odom`.

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

# \brief This function is the callback of the subscriber to the topic /odom. It takes the actual position and velocity of the robot and publishes them as a custom message.
# \param msg is the message received from the topic /odom
# \return None
def publisher_node(msg):
    # Create a publisher
    global pub

    # Get the actual position and velocity
    actual_pos = msg.pose.pose.position
    actual_vel_linear = msg.twist.twist.linear
    actual_vel_angular = msg.twist.twist.angular

    # Create a Vel message
    my_pos_and_vel = Vel()
    my_pos_and_vel.pos_x = actual_pos.x
    my_pos_and_vel.pos_y = actual_pos.y
    my_pos_and_vel.vel_x = actual_vel_linear.x
    my_pos_and_vel.vel_z = actual_vel_angular.z

    # rospy.loginfo(my_pos_and_vel)
    pub.publish(my_pos_and_vel)

    # Set the publishing rate (e.g., 1 Hz)
    # rate = rospy.Rate(0.1)  # 1 Hz


# \brief This function implements an action client, which also provides a user interface, running on a separate terminal, to let the user choose from the terminal either to: set a new target point that the robot must reach andcancel the goal previously chosen.
# \param None
# \return None
def parameters_client_main():
    #global reached
    global first_start

    # Create an action client
    client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2023.msg.PlanningAction)
    client.wait_for_server()

    while not rospy.is_shutdown():
        rospy.loginfo("\n Hello! This is the simulation menu! \n PRESS: \n y - to set a new goal \n c - to cancel the last goal \n")

        command = input("What do you want to do?: ")

        # Get the actual goal
        goal = assignment_2_2023.msg.PlanningGoal()
        goal.target_pose.pose.position.x = rospy.get_param('/des_pos_x')
        goal.target_pose.pose.position.y = rospy.get_param('/des_pos_y')
        
        # Handling inputs
        
        # the user wants to set a new goal
        if command == 'y':
            #rospy.loginfo("Last goal: des_x = %f, des_y = %f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)
            
            # get the input and check their consistency
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

            # Update ros parameters
            rospy.set_param('/des_pos_x', input_x)
            rospy.set_param('/des_pos_y', input_y)

            # Set goal parameters
            goal.target_pose.pose.position.x = input_x
            goal.target_pose.pose.position.y = input_y

            # send goal to the service
            client.send_goal(goal)
            rospy.loginfo("Inserted goal: des_x = %f, des_y = %f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)
            
            # update variable to handle cancel operation as soon as simulation starts
            # after inserting the first goal, first_start condition is always set to false
            first_start = 1 


        # The user wants to cancel the goal
        elif command == 'c':
            
            # check variable to handle cancel operation as soon as simulation starts
            # after inserting the first goal, first_start condition is always set to false
            if first_start == 0:
                rospy.loginfo("Goal has not been already set, cannot cancel it")
                
            elif client.get_state() == actionlib.GoalStatus.ACTIVE:

                    # if goal not reached and goal in active state cancel the goal
                    client.cancel_goal()
                    rospy.loginfo("Goal cancelled")    
            
            else:
                rospy.loginfo("Goal already reached, cannot cancel it")

        # The user digits neither y nor c
        else:
            rospy.loginfo("Invalid input")


def main():
    rospy.init_node('set_target_client')

    global pub

    # PUBLISHER: send a message which contains two parameters (velocity and position)
    pub = rospy.Publisher("/pos_vel", Vel, queue_size=1)

    # SUBSCRIBER: get from "Odom" two parameters (velocity and position)
    sub_from_Odom = rospy.Subscriber("/odom", Odometry, publisher_node)

    # Calling the action client
    parameters_client_main()

if __name__ == '__main__':
    main()

