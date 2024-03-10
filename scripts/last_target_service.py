## @package assignment_2_2023 
# \file last_target_service.py
# \brief Service that returns the values of the last target sent by the user.
# \author Annika Delucchi
# \version 1.0
# \date    9/03/2024
#
# \details
#
# Subscribes to: <BR>
#    ° /pos_vel: custom message containing the actual position and velocity of the robot
#
# Publishes to: <BR>
#    ° None
#
# Services: <BR>
#    ° /last_target: service to get the last target
#
# Ros parameters: <BR>
#    ° /des_pos_x: x coordinate of the last target
#    ° /des_pos_y: y coordinate of the last target
#
# Description: <BR>
# last_target_service.py is a node implementing a service that, when called, returns 
# the values of the last target sent by the user. 
# <BR>
# To make it feasible, a srv file Last_target.srv is created in the so-called directory; it
# contains the expected service response type. 
# <BR>
# The last target values are extracted from the ros parameters updated from the
# set_target_client,py and returned as response from the service. 
# <BR>
# If the service is called before the user sets a target, the response gives the default
# values(/des_pos_x = 0.0 and des_pos_y = 1.0) chosen in the launch file assignment1.launch.
# <BR>
# Furthermore, this service is run by the launch file; to call it and get the last target sent by
# the user run the command rosservice call /last_target on the terminal.
#
##


#!/usr/bin/env python

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

##
# \brief This function is the callback of the subscriber to the topic /pos_vel
# \param msg: custom message containing the actual position and velocity of the robot
# \return None
#
# It takes the last target from the ros parameters.
def get_last_target(msg):
    global last_des_x, last_des_y

    # get last target from ros parameters. 
    # they have been updated when the last target was entered by the user
    last_des_x = rospy.get_param('/des_pos_x')
    last_des_y = rospy.get_param('/des_pos_y')
    
    #print("Last input target is des_x = %f, des_y = %f" % (last_des_x, last_des_y))
    
##    
# \brief This function is the callback of the service /last_target
# \param s: service request
# \return response: service response containing the last target entered by the user    
#
# It returns the last target entered by the user. If the service is called before the user sets a target, 
# the response gives the default values(/des_pos_x = 0.0 and des_pos_y = 1.0) chosen in the launch file assignment1.launch
def result_callback(s):
    global last_des_x, last_des_y 
    
    # store last target
    response = Last_targetResponse()
    response.last_target_x = last_des_x
    response.last_target_y = last_des_y
    
    return response

##
# \brief This function initializes the node last_target_service            
# \param None
# \return None
#
# It initializes the node last_target_service and implements the service to get the last target
def last_target_service():
    rospy.init_node('last_target_service')
    rospy.loginfo("Last target node initialized")

    # SUBSCRIBER: Subscribe to the correct action goal topic
    rospy.Subscriber("/pos_vel", Vel, get_last_target)
    
    # SERVICE: Service to get the last target. It uses Last_target service type
    service = rospy.Service('last_target', Last_target, result_callback)
    
    rospy.spin()

if __name__ == "__main__":
    last_target_service()
    
    









