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
from std_srvs.srv import Empty, EmptyResponse

global last_des_x, last_des_y
last_des_x = 0.0
last_des_y = 1.0

#def goal_callback(msg):
#    last_des_x = msg.target_pose.pose.position.x
#    last_des_y = msg.target_pose.pose.position.y

def get_last_target(request):
    # Use EmptyResponse as the response type
    response = EmptyResponse()
    
    last_des_x = rospy.get_param('/des_pos_x')
    last_des_y = rospy.get_param('/des_pos_y')
    
    print("Last input target is des_x = %f, des_y = %f" % (last_des_x, last_des_y))
    
    return response

def last_target_service():
    rospy.init_node('last_target_server')

    # Subscribe to the correct action goal topic
    #rospy.Subscriber('/reaching_goal/goal', PlanningActionGoal, goal_callback)
    
    # Use Empty service type
    service = rospy.Service('last_target', Empty, get_last_target)
    rospy.spin()

if __name__ == "__main__":
    last_target_service()




