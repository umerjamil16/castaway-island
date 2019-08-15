#!/usr/bin/env python
###################################
# TESTING PURPOSES  /get_goal service
#
###################################

import math
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
# GLOBAL VARS
inner_circle_rad =  ((2.73272664774 + 2.68263825162)/2) #2.70765
theta_0 = 0
tol =0.8

goal_x = 0.0
goal_y = 0.0
goal_circle_rad = 0.0
measured_circle_rad = 0

# GOAL GENERATION FUNCTION
def gen_goal(msg):
	global goal_x, goal_y, goal_circle_rad, inner_circle_rad, tol, roll, pitch, yaw, measured_circle_rad, theta_0
	pose_q = msg.pose.pose.position
	if pose_q.x != 0 and pose_q.y != 0:
		theta_0 = math.atan(pose_q.y/ pose_q.x) *180/3.14
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



## MAIN PROGRAM FILE
rospy.init_node('my_quaternion_to_euler')
 
sub = rospy.Subscriber ('/robot_0/odom', Odometry, gen_goal)
 
r = rospy.Rate(100)
while not rospy.is_shutdown():
	print "node live"
	print 'Goal_x: ' + str(goal_x) + 'Goal_y' + str(goal_y) + 'Goal_YAW' + str(theta_0)
	#	print yaw_rad
	r.sleep()
