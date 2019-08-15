#!/usr/bin/env python
###################################
# This file provides a service /get_goal, which upon calling returns x, y, yaw of robot
#
#
###################################
import math
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from ca_main.srv import GoalCustomSrv, GoalCustomSrvResponse

goal_x = 0.0


def goal_service_callback(request):
	rospy.loginfo("get_robot_0_pos Callback has been called")
	response = GoalCustomSrvResponse()
	response.t = goal_x
	rospy.loginfo("get_robot_0_pos X: " + str(response.t))
	return response
	
def get_yaw_angle(msg):
	global goal_x, goal_y, goal_circle_rad, inner_circle_rad, tol, roll, pitch, yaw, measured_circle_rad, theta_0
	pose_q = msg.pose.pose.position
	goal_x = pose_q.x


rospy.init_node('robot_0_pos_srv_node')
 
sub = rospy.Subscriber ('/robot_0/odom', Odometry, get_yaw_angle)

my_service = rospy.Service('/get_robot_0_pos', GoalCustomSrv , goal_service_callback) # create the Service called my_service with the defined callback
 
rospy.loginfo("Service /get_robot_0_pos Ready")

r = rospy.Rate(100)
rospy.spin()