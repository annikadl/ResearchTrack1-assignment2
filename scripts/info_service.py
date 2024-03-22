#!/usr/bin/env python
"""
.. module:: info_service
   :platform: Unix
   :synopsis: Python module for the assignment_2_2023
.. moduleauthor:: Annika Delucchi
  
Description:
    This module implements a ROS node that provides a service to retrieve the distance from the goal and the average velocity of the robot.
    To make it feasible, a service file `info_service.srv` is created in the package directory, defining the expected service response type.
    The subscriber implemented in this node's callback function extracts the target position from ROS parameters and the actual position from the custom message sent by `set_target_client.py`. The distance is computed as the Euclidean distance using the Python built-in function `math.dist(des_coordinates, actual_coordinates)`.
    The velocity values are also extracted from the custom message, collected in a list of dimension `window_size` with a default value of 10 (modifiable in the launch file). The average velocity is then computed as `average_vel_x = sum(vel_data) / min(len(vel_data), velocity_window_size)`.
    These computed values compose the response of the service.
    The `info_service.py` module is run by the launch file, and the service can be called using the command `rosservice call /info_service` in the terminal.

    
Service:
    /info_service

Subscriber:
    /pos_vel

**Functions**: 

"""

import rospy
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry
import math
import assignment_2_2023.msg
from assignment_2_2023.msg import Vel
from actionlib_msgs.msg import GoalStatusArray
from assignment_2_2023.srv import info_service, info_serviceResponse

def get_distance_and_averagevelocity(msg):
    """
    Callback function for the subscriber to the topic /pos_vel.
    
    :param msg: custom message containing the actual position and velocity of the robot
    :param des_pos_x: desired position x
    :param des_pos_y: desired position y
    :type des_pos_x: float
    :type des_pos_y: float
    :param window_size: size of the window for the average velocity
    :type msg: assignment_2_2023.msg.Vel
    """
    global success, average_vel_x, distance

    # desired position 
    des_x = rospy.get_param('/des_pos_x')
    des_y = rospy.get_param('/des_pos_y')
    
    # actual position
    actual_x = msg.pos_x
    actual_y = msg.pos_y
    
    # compute distance
    des_coordinates = [des_x, des_y]
    actual_coordinates = [actual_x, actual_y]
    distance = math.dist(des_coordinates, actual_coordinates)

    # compute linear average velocity
    if isinstance(msg.vel_x, list):
        vel_data = msg.vel_x[-velocity_window_size:]
    else:
        vel_data = [msg.vel_x]

    average_vel_x = sum(vel_data) / min(len(vel_data), velocity_window_size)

def get_values(s):      
    """
    Callback function for the service /info_service. Returns the distance from the goal and the average velocity of the robot.
    
    :param s: service request
    :type s: assignment_2_2023.srv.info_serviceRequest
    :return: service response containing the distance from the goal and the average velocity of the robot
    :rtype: assignment_2_2023.srv.info_serviceResponse
    """
    global average_vel_x, distance
    
    rospy.loginfo("Distance= %f Average velocity = %f", distance, average_vel_x)
    return info_serviceResponse(distance, average_vel_x)	

def info_service_main():
    """
    Initializes the node info_service.
    """
    global velocity_window_size

    rospy.init_node('info_service')
    rospy.loginfo("Info service node initialized")
    
    # Read window size from the parameter server (set in the launch file)
    velocity_window_size = rospy.get_param('/window_size')

    # SUBSCRIBER: Subscribe to the custom message
    subscriber = rospy.Subscriber("/pos_vel", Vel, get_distance_and_averagevelocity)
    
    # SERVICE: Service to get the interested values
    service = rospy.Service("info_service", info_service, get_values)

    rospy.spin()

if __name__ == "__main__":
    info_service_main()
