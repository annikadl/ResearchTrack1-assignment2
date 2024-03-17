#!/usr/bin/env python
"""
.. module:: last_target_service
   :platform: Unix
   :synopsis: Python module for the assignment_2_2023
.. moduleauthor:: Annika Delucchi

Service: 
    last_target
Subscriber:
    /pos_vel    
"""


"""
last_target_service.py is a node implementing a service that, when called, returns 
the values of the last target sent by the user. 

To make it feasible, a srv file Last_target.srv is created in the so-called directory; it
contains the expected service response type. 

The last target values are extracted from the ros parameters updated from the
set_target_client,py and returned as response from the service. 

If the service is called before the user sets a target, the response gives the default
values(/des_pos_x = 0.0 and des_pos_y = 1.0) chosen in the launch file assignment1.launch.

Furthermore, this service is run by the launch file; to call it and get the last target sent by
the user run the command rosservice call /last_target on the terminal.
"""

import rospy
import assignment_2_2023.msg
from assignment_2_2023.msg import Vel
from assignment_2_2023.srv import Last_target, Last_targetResponse

def get_last_target(msg):
    """
    Callback function for the subscriber to the topic /pos_vel.
    
    :param msg: custom message containing the actual position and velocity of the robot
    :type msg: assignment_2_2023.msg.Vel
    """
    global last_des_x, last_des_y

    last_des_x = rospy.get_param('/des_pos_x')
    last_des_y = rospy.get_param('/des_pos_y')

def result_callback(s):
    """
    Callback function for the service /last_target. It returns the last target entered by the user.
    
    :param s: service request
    :type s: assignment_2_2023.srv.Last_targetRequest
    :return: service response containing the last target entered by the user
    :rtype: assignment_2_2023.srv.Last_targetResponse
    """
    global last_des_x, last_des_y 
    
    response = Last_targetResponse()
    response.last_target_x = last_des_x
    response.last_target_y = last_des_y
    
    return response

def last_target_service():
    """
    Initializes the node last_target_service.
    """
    rospy.init_node('last_target_service')
    rospy.loginfo("Last target node initialized")

    rospy.Subscriber("/pos_vel", Vel, get_last_target)
    service = rospy.Service('last_target', Last_target, result_callback)
    
    rospy.spin()

if __name__ == "__main__":
    last_target_service()
