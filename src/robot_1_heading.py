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

# GLOBAL VARS
roll = 0
pitch = 0
yaw = 0
a = 0
# GOAL GENERATION FUNCTION
def get_yaw_angle(msg):
	global roll, pitch, yaw, a
	orientation_q = msg.pose.pose.orientation
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	(roll, pitch, yaw) = euler_from_quaternion(orientation_list)
	a = yaw
	rospy.loginfo(" ROLL: " + str(roll) + " PITCH: " + str(pitch)  + " YAW: "+ str(yaw))

rospy.init_node('robot_1_heading')
 
sub = rospy.Subscriber ('/robot_1/odom', Odometry, get_yaw_angle)
rospy.loginfo(" TOPICROLL: " + str(roll) + " PITCH: " + str(pitch)  + " YAW: "+ str(a))

rate = rospy.Rate(10.0)

rospy.spin()