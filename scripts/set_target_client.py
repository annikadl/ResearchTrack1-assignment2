#!/usr/bin/env python

"""
This is the action server that satisfies the request: implement an action client, 
allowing the user to set a target point or to cancel it (this node also uses the 
feedback/status of the action server to know when the target has been reached); 
this node also publishes robot position and velocity as a custom message.

It implements different functions.

set_client_parameters() implements an action client, which also provides a user 
interface, running on a separate terminal, to let the user choose from the terminal either to:
* set a new target point that the robot must reach.
* cancel the goal previously chosen.

on_sub_result() is the callback of the sub_from_result subscriber, which subscribes
to the /reaching_goal/result topic, to get the result of the task associated with 
the goal. The callback stores in a variable called `reached` whether the goal has 
been succesfully reached or not.

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
reached = False
first_start = 0

###### PUBLISHER
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


####### CLIENT
def parameters_client_main():
    global reached, first_start

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
            rospy.loginfo("Last goal: des_x = %f, des_y = %f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)
            
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
            rospy.loginfo("Actual goal: des_x = %f, des_y = %f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)
            
            # update variable to handle cancel operation as soon as simulation starts
            # after inserting the first goal, first_start condition is always set to false
            first_start = 1 


        # The user wants to cancel the goal
        elif command == 'c':
            
            # check variable to handle cancel operation as soon as simulation starts
            # after inserting the first goal, first_start condition is always set to false
            if first_start == 0:
                rospy.loginfo("Goal has not been already set, can't cancel")
                
            elif not reached:
                # Cancel the goal only if it has not been already reached
                client.cancel_goal()
                rospy.loginfo("Goal cancelled")
            
            else:
                rospy.loginfo("Goal already reached, can't be canceled")

        # The user digits neither y nor c
        else:
            rospy.loginfo("Invalid input")


# callback to get the state --> used to check if goal cancellable
def on_sub_result(action_result):
    global reached
        
    # Check if the goal is either succeeded or preempted
    reached = action_result.status.status in [GoalStatus.SUCCEEDED, GoalStatus.PREEMPTED]



def main():
    rospy.init_node('set_target_client')

    global pub

    # PUBLISHER: send a message which contains two parameters (velocity and position)
    pub = rospy.Publisher("/pos_vel", Vel, queue_size=1)

    # SUBSCRIBER: get from "Odom" two parameters (velocity and position)
    sub_from_Odom = rospy.Subscriber("/odom", Odometry, publisher_node)

    # SUBSCRIBER: get from PlanningActionResult a parameter to know whether the goal has been reached or not
    sub_from_result = rospy.Subscriber("/reaching_goal/result", PlanningActionResult, on_sub_result)

    # Calling the action client
    parameters_client_main()

if __name__ == '__main__':
    main()

