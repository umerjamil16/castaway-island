#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from ca_main.srv import GoalSrv0, GoalSrv0Request # you import the service message python classes generated from Empty.srv.
from nav_msgs.msg import Odometry

goal_x = 0
goal_y = 0
goal_yaw = 0


robot_1_x = 0
robot_1_y = 0



def update_robot_1_pos(msg):
    print "hello"
    global robot_1_x, robot_1_y
    robot_1_x = msg.pose.pose.position.x
    robot_1_y = msg.pose.pose.position.y #+ 3
    rospy.loginfo("Current pos of robot_1: X: " + str(robot_1_x) + " Y: " + str(robot_1_y))
	        
if __name__ == '__main__':
    rospy.init_node('robot_1_pos_calc')
    ##Calculating /robot_1 path distance to the generated goal of robot_0
    sub = rospy.Subscriber ('/robot_1/odom', Odometry, update_robot_1_pos)
    rospy.loginfo("POS of robot_1: X: " + str(robot_1_x) + " Y: " + str(robot_1_y))
    rospy.spin()