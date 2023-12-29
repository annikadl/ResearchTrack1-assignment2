#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Point, Pose, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import actionlib
import actionlib.msg
import assignment_2_2023.msg
from tf import transformations
from std_srvs.srv import *
import time
from assignment_2_2023.msg import Vel
from assignment_2_2023.msg import PlanningAction
from assignment_2_2023.msg import PlanningAction, PlanningGoal, PlanningResult
from std_msgs.msg import String

pub = None

###### PUBLISHER
def publisher_node(msg):
    # Create a publisher
    global pub
    # pub = rospy.Publisher('pub_position_velocity', Vel, queue_size=10)

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

    #rospy.loginfo(my_pos_and_vel)
    pub.publish(my_pos_and_vel)
    
    # Set the publishing rate (e.g., 1 Hz)
    #rate = rospy.Rate(0.0001)  # 1 Hz

####### CLIENT
def parameters_client_main():
    
    # Create an action client
    client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2023.msg.PlanningAction)
    client.wait_for_server()

    while not rospy.is_shutdown():
        # subscriber_node()
        print("Do you want to set or cancel the goal?")
        # TODO: handle non-char input
        # try:
        command = input("Press y to set, n to continue, or c to cancel: ")
        # except ValueError:
        # rospy.logerr("Invalid input. Please enter a character.")

        # Get the actual goal
        goal = assignment_2_2023.msg.PlanningGoal()
        goal.target_pose.pose.position.x = rospy.get_param('/des_pos_x')
        goal.target_pose.pose.position.y = rospy.get_param('/des_pos_y')
        rospy.loginfo("Actual goal: des_x = %f, des_y = %f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)

        # Modify goal from keyboard
        if command == 'y':
            input_x = input("Enter desired position x: ")
            input_y = input("Enter desired position y: ")
            # TODO: handle non-numerical values
            # except ValueError:
            # rospy.logerr("Invalid input. Please enter numerical values.")

            input_x = float(input_x)
            input_y = float(input_y)
            # Set parameters on the parameter server
            rospy.set_param('/des_pos_x', input_x)
            rospy.set_param('/des_pos_y', input_y)

            goal.target_pose.pose.position.x = input_x
            goal.target_pose.pose.position.y = input_y

        # Goal isn't modified
        if command == 'n':
            pass

        # Cancel the goal -> desired values go to zero? or to random ones?
        if command == 'c':
            client.cancel_goal()
            print("Goal cancelled")

        rospy.loginfo("Received goal: des_x = %f, des_y = %f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)

        client.send_goal(goal)

def main():
    rospy.init_node('set_target_client')

    # Global pub
    global pub

    # PUBLISHER: send a message which contains two parameters (velocity and position)
    pub = rospy.Publisher("/pos_vel", Vel, queue_size=1)

    # SUBSCRIBER: get from "Odom" two parameters (velocity and position)
    sub_from_Odom = rospy.Subscriber("/odom", Odometry, publisher_node)

    # Calling the function client
    parameters_client_main()

if __name__ == '__main__':
    main()

