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


def handle_parameters_server(goal):
    # Extract the desired position from the action goal
    des_pos_x = goal.des_x
    des_pos_y = goal.des_y
     
    # Perform any actions or computations based on the received goal
    # In this example, let's just print the received values
    rospy.loginfo("Received goal: des_x = %f, des_y = %f", des_pos_x, des_pos_y)

    # Simulate some processing time (replace this with your actual logic)
    rospy.sleep(2.0)

    # Create an action result
    result = ParametersResult()
    # result.final_x = 42.0  # Replace with the actual final position x
    # result.final_y = 24.0  # Replace with the actual final position y
    rospy.loginfo("Goal reached? Success = %s", str(result))


    # Send the result back to the client
    return result

def parameters_server_main():
    # Initialize the ROS node
    rospy.init_node('parameters_server')

    # Create an action server
    server = actionlib.SimpleActionServer('parameters', ParametersAction, handle_parameters_server, auto_start=False)
    server.start()

    # Print a message indicating the server is ready
    rospy.loginfo("Parameters Action Server is ready.")
    
    
    
   # handle_parameters_server(goal)

    # Keep the script running
    rospy.spin()


    
    
if __name__ == '__main__':
    parameters_server_main()






