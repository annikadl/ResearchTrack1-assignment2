#!/usr/bin/env python

"""
info_service,py is a node implementing a service that, when called, returns the distance from
the goal and the average velocity of the robot. 

To make it feasible, a srv file info_service.srv is created in the so-called directory; it
contains the expected service response type. 

A subscriber is implemented and its callback takes the target position from the ros parameters
and the actual one from the custom message sent by the set_target_client.py. The distance is
computed as the Euclidean distance by using the Python built-in function
math.dist(des_coordinates, actual coordinates) by importing math library. 
The velocity values are extracted by the custom message too, they are collected in a list of
dimension window_size with a default value of 10 (that can be modified in the launch file).
Then the average velocity is computed as average_vel_x = sum(vel_data) / min(len(vel_data),
velocity_window_size). These values compose the response of the service.

info_service.py is run by the launch file and the service can be called by using the command
rosservice call /info_service on the terminal.


"""

import rospy
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry
import math
import assignment_2_2023.msg
from assignment_2_2023.msg import Vel
from actionlib_msgs.msg import GoalStatusArray
from assignment_2_2023.srv import info_service, info_serviceResponse

# Service node thatsubscribes to the robot's position and velocity (using the custom message)
# and implements a server to retrieve the distance of the robot from the target and the robot's
# average speed.


def get_distance_and_averagevelocity(msg):
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

    #rospy.loginfo("Average velocity: %f", average_vel_x)
    
        
def get_values(s):      
    global average_vel_x, distance
    
    rospy.loginfo("Distance= %f Average velocity = %f", distance, average_vel_x)
    return info_serviceResponse(distance, average_vel_x)	
    		      

def info_service_main():
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
	
