#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry
import actionlib
import actionlib.msg
import assignment_2_2023.msg
from assignment_2_2023.msg import Vel
from assignment_2_2023.msg import PlanningAction, PlanningGoal, PlanningResult, PlanningActionResult
from std_srvs.srv import SetBool
from actionlib_msgs.msg import GoalStatus

pub = None
reached = False

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

    # rospy.loginfo(my_pos_and_vel)
    pub.publish(my_pos_and_vel)

    # Set the publishing rate (e.g., 1 Hz)
    # rate = rospy.Rate(0.0001)  # 1 Hz


####### CLIENT
def parameters_client_main():
    global reached

    # Create an action client
    client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2023.msg.PlanningAction)
    client.wait_for_server()

    while not rospy.is_shutdown():
        # subscriber_node()
        print("Do you want to set or cancel the goal?")
        # TODO: handle non-char input
        # try:
        command = input("Press y to set or c to cancel: ")
        # except ValueError:
        # rospy.logerr("Invalid input. Please enter a character.")

        # Get the actual goal
        goal = assignment_2_2023.msg.PlanningGoal()
        goal.target_pose.pose.position.x = rospy.get_param('/des_pos_x')
        goal.target_pose.pose.position.y = rospy.get_param('/des_pos_y')
        #rospy.loginfo("Actual goal: des_x = %f, des_y = %f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)

        # Modify goal from keyboard
        if command == 'y':
            rospy.loginfo("Last goal: des_x = %f, des_y = %f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)

            while(True):
                input_x = input("Enter desired position x: ")
                try:
                    input_x = float(input_x)
                    break
                    
                except:
                    print("Invalid input, enter a number")
            
            while(True):
                input_y = input("Enter desired position y: ")
                try:
                    input_y = float(input_y)
                    break
                    
                except:
                    print("Invalid input, enter a number")            
            
            # Set ros parameters
            rospy.set_param('/des_pos_x', input_x)
            rospy.set_param('/des_pos_y', input_y)

            goal.target_pose.pose.position.x = input_x
            goal.target_pose.pose.position.y = input_y

            client.send_goal(goal)
            rospy.loginfo("Actual goal: des_x = %f, des_y = %f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)

        # Cancel the goal
        elif command == 'c':

            if reached == False:
                # Check if the client is in the ACTIVE state before canceling
                if client.get_state() == actionlib.GoalStatus.ACTIVE:
                    client.cancel_goal()
                    rospy.loginfo("Goal cancelled")
                else:
                    rospy.loginfo("Goal is not active, cannot be cancelled")
            elif reached == True:
                rospy.loginfo("Goal already reached, can't be canceled")

        else:
            rospy.loginfo("Invalid input")

        # rospy.loginfo("Last received goal: des_x = %f, des_y = %f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)


def on_sub_result(action_result):
    global reached
    reached = not (action_result.status.status == action_result.status.SUCCEEDED or action_result.status.status == action_result.status.PREEMPTED)

      

def main():
    rospy.init_node('set_target_client')

    # Global pub
    global pub

    # PUBLISHER: send a message which contains two parameters (velocity and position)
    pub = rospy.Publisher("/pos_vel", Vel, queue_size=1)

    # SUBSCRIBER: get from "Odom" two parameters (velocity and position)
    sub_from_Odom = rospy.Subscriber("/odom", Odometry, publisher_node)

    # SUBSCRIBER: get from PlanningActionResult a parameter to know whether the goal has been reached or not
    sub_from_result = rospy.Subscriber("/reaching_goal/result", PlanningActionResult, on_sub_result)

    # Calling the action client
    parameters_client_main()

if __name__ == '__main__':
    main()

