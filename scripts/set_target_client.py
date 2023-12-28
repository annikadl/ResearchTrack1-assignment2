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
from assignment_2_2023.msg import ParametersAction, ParametersGoal, ParametersResult

####### CLIENT
	
def parameters_client_main():
    rospy.init_node('set_target_client')

    # Create an action client
    client = actionlib.SimpleActionClient('parameters', ParametersAction)
    client.wait_for_server()
    
    while not rospy.is_shutdown():
        print("Do you want to set or cancel the goal?")
        # TODO: handle non-char input
        # try:
        command = input("Press y to set, n to continue, or c to cancel: ")
        # except ValueError:
        #     rospy.logerr("Invalid input. Please enter a character.")

        # Get the actual goal
        goal = ParametersGoal()
        goal.des_x = rospy.get_param('/des_pos_x')
        goal.des_y = rospy.get_param('/des_pos_y')
        rospy.loginfo("Actual goal: des_x = %f, des_y = %f", goal.des_x, goal.des_y)

        # Modify goal from keyboard
        if command == 'y':
            input_x = input("Enter desired position x: ")
            input_y = input("Enter desired position y: ")
            # TODO: handle non-numerical values
            # except ValueError:
            #     rospy.logerr("Invalid input. Please enter numerical values.")

            input_x = float(input_x)
            input_y = float(input_y)
            # Set parameters on the parameter server
            rospy.set_param('/des_pos_x', input_x)
            rospy.set_param('/des_pos_y', input_y)

            goal.des_x = input_x
            goal.des_y = input_y

        # Goal isn't modified
        if command == 'n':
            pass

        # Cancel the goal -> desired values go to zero? or to random ones?
        if command == 'c':
            rospy.set_param('/des_pos_x', 0.0)
            rospy.set_param('/des_pos_y', 0.0)

        rospy.loginfo("Received goal: des_x = %f, des_y = %f", goal.des_x, goal.des_y)
        
        client.send_goal(goal)
        #rospy.spin()

if __name__ == '__main__':
    parameters_client_main()

