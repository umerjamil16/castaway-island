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

# GLOBAL VARS
inner_circle_rad =  ((2.73272664774 + 2.68263825162)/2) #2.70765
theta_0 = 0
tol =0.8

goal_x = 0.0
goal_y = 0.0
goal_circle_rad = 0.0
measured_circle_rad = 0

def goal_service_callback(request):
		rospy.loginfo("GoalService Callback has been called")
		response = GoalCustomSrvResponse()
		response.x = goal_x
		response.y = goal_y
		response.t = theta_0
		rospy.loginfo("SRV Goal Generated: X: " + str(response.x) + "  Y: " + str(response.y)+ " Yaw: " + str(response.t))
		return response
	
# GOAL GENERATION FUNCTION
def gen_goal(msg):
	global goal_x, goal_y, goal_circle_rad, inner_circle_rad, tol, roll, pitch, yaw, measured_circle_rad, theta_0
	pose_q = msg.pose.pose.position
	if pose_q.x != 0 and pose_q.y != 0:
		theta_0 = math.atan2(pose_q.y, pose_q.x)
	else:
		theta_0 = 0
	measured_circle_rad = math.sqrt((pose_q.x * pose_q.x) + (pose_q.y * pose_q.y)) #for manual measurement of radius of inner circle
	orientation_q = msg.pose.pose.orientation
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	(roll, pitch, yaw) = euler_from_quaternion(orientation_list)
	yaw_deg = (yaw*180)/3.1

	goal_x = (inner_circle_rad + tol) * math.cos(theta_0)
	goal_y = (inner_circle_rad + tol) * math.sin(theta_0)
	goal_circle_rad = math.sqrt(goal_x * goal_x + goal_y*goal_y)
	print 'Goal_x: ' + str(goal_x) + 'Goal_y' + str(goal_y) + 'Goal_YAW' + str(theta_0)


rospy.init_node('my_quaternion_to_euler1')
 
sub = rospy.Subscriber ('/robot_0/odom', Odometry, gen_goal)

my_service = rospy.Service('/get_goal', GoalCustomSrv , goal_service_callback) # create the Service called my_service with the defined callback
 
rospy.loginfo("Service /get_goal Ready")


## MAIN PROGRAM FILE
 
r = rospy.Rate(100)
#print 'Goal_x: ' + str(goal_x) + 'Goal_y' + str(goal_y) + 'Goal_YAW' + str(theta_0)

rospy.spin()
#while not rospy.is_shutdown():
#	print "node live"
#	print 'Goal_x: ' + str(goal_x) + 'Goal_y' + str(goal_y) + 'Goal_YAW' + str(theta_0)
	#	print yaw_rad
#	r.sleep()
