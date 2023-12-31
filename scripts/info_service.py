#!/usr/bin/env python

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
from assignment_2_2023.msg import PlanningActionGoal
from assignment_2_2023.msg import PlanningAction, PlanningGoal, PlanningResult
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse, SetBool, SetBoolResponse
from actionlib_msgs.msg import GoalStatusArray
from assignment_2_2023.srv import info_service, info_serviceResponse

# Another service node thatsubscribes to the robot's position and velocity (using the custom message) and implements a server to retrieve the distance of the robot from the target and the robot's average speed.

## Global variables
success = False
#velocity_window_size = 10  # Default window size

def get_distance_and_averagevelocity(msg):
    global success, average_vel_x, distance

    # desired position - actual position
    des_x = rospy.get_param('/des_pos_x')
    des_y = rospy.get_param('/des_pos_y')
    
    actual_x = msg.pos_x
    actual_y = msg.pos_y
    
    #rospy.loginfo("Desired position: des_x = %f, des_y = %f", des_x, des_y)
    #rospy.loginfo("Actual position: actual_x = %f, actual_y = %f", actual_x, actual_y)
    
    des_coordinates = [des_x, des_y]
    actual_coordinates = [actual_x, actual_y]
    distance = math.dist(des_coordinates, actual_coordinates)
    
    #rospy.loginfo("Actual distance from target: distance = %f", distance)

    # Compute average velocity along x-axis
    if isinstance(msg.vel_x, list):
        vel_data = msg.vel_x[-velocity_window_size:]
    else:
        vel_data = [msg.vel_x]

    average_vel_x = sum(vel_data) / min(len(vel_data), velocity_window_size)

    #rospy.loginfo("Average velocity: %f", average_vel_x)

    if success:
        rospy.loginfo("Goal successfully reached")
 
# attualmente non chiamata da nessuno        
def result_callback(msg):
    global success
    if msg.status_list and msg.status_list[-1].status == 3:  # Check if the last status is SUCCEEDED
        success = True    
        
def get_values(s):      
    global average_vel_x, distance
    
    rospy.loginfo("Distance= %f Average velocity = %f", distance, average_vel_x)
    return info_serviceResponse(distance, average_vel_x)			      

def info_service_main():
    global success, velocity_window_size

    rospy.init_node('info_service')
    rospy.loginfo("Info service node initialized")

    # Subscribe to the custom message
    subscriber = rospy.Subscriber("/pos_vel", Vel, get_distance_and_averagevelocity)
    
    # Service
    service = rospy.Service("info_service", info_service, get_values)
    
    # Subscribe to action result
    #result_subscriber = rospy.Subscriber("/reaching_goal/result", GoalStatusArray, result_callback)

    # Read window size from the parameter server (set in the launch file)
    velocity_window_size = rospy.get_param('/window_size')

    rospy.spin()

if __name__ == "__main__":
    info_service_main()
	
