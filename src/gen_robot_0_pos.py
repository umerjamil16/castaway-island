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

from ca_main.srv import GoalSrv0, GoalSrv0Response



def goal_service_callback(request):
    global goal_x, goal_y
    rospy.loginfo("robot_0_pose Callback has been called")
    response = GoalSrv0Response()
    response.x = goal_x
    response.y = goal_y
    response.t = 0
    rospy.loginfo("X: " + str(response.x) + "Y: " + str(response.y))
    return response
	
def get_pos(msg):
    global goal_x, goal_y
    pose_q = msg.pose.pose.position
    r0_x = pose_q.x
    r0_y = pose_q.y

if __name__ == '__main__':
    rospy.init_node('get_robot_0_pose_service')
    goal_x = 0.0
    goal_y = 0.0
    rate = rospy.Rate(10.0)
    my_service = rospy.Service('/robot_0_pose', GoalSrv0 , goal_service_callback) 
    rospy.loginfo("Service /robot_0_pose Ready")
    while not rospy.is_shutdown():
        sub = rospy.Subscriber ('/robot_0/odom', Odometry, get_pos)
        r = rospy.Rate(100)
        rate.sleep()
#rospy.spin()