#!/usr/bin/env python

import rospy
import assignment_2_2023.msg
from assignment_2_2023.msg import Vel
from assignment_2_2023.srv import Last_target, Last_targetResponse


def get_last_target(msg):
    # Use EmptyResponse as the response type
    #response = EmptyResponse()
    
    global last_des_x, last_des_y

    
    last_des_x = rospy.get_param('/des_pos_x')
    last_des_y = rospy.get_param('/des_pos_y')
    
    #print("Last input target is des_x = %f, des_y = %f" % (last_des_x, last_des_y))
    
    
def result_callback(s):
    global last_des_x, last_des_y 
    
    response = Last_targetResponse()
    response.last_target_x = last_des_x
    response.last_target_y = last_des_y
    
    return response
    	    

def last_target_service():
    rospy.init_node('last_target_service')
    rospy.loginfo("Last target node initialized")

    # Subscribe to the correct action goal topic
    rospy.Subscriber("/pos_vel", Vel, get_last_target)
    
    # Use Last_target service type
    service = rospy.Service('last_target', Last_target, result_callback)
    #print("Last input target is des_x = %f, des_y = %f" % (last_des_x, last_des_y))
    rospy.spin()

if __name__ == "__main__":
    last_target_service()




