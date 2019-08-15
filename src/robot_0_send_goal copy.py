#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from ca_main.srv import GoalSrv0, GoalSrv0Request # you import the service message python classes generated from Empty.srv.

goal_x = 0
goal_y = 0
goal_yaw = 0

def get_goal_robot_0():
    global goal_x, goal_y, goal_yaw
    rospy.wait_for_service('/get_robot_0_goal')
    goal_srv_client = rospy.ServiceProxy('/get_robot_0_goal', GoalSrv0)
    goal_srv_client_obj = GoalSrv0Request()
    
    rospy.loginfo("Doing /get_robot_0_goal Service Call...")
    result = goal_srv_client(goal_srv_client_obj)
    goal_x = result.x
    goal_y = result.y
    goal_yaw = result.t
    rospy.loginfo("END of /get_robot_0_goal Service call...")

def movebase_client():

    #client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client = actionlib.SimpleActionClient('robot_0/move_base', MoveBaseAction)
    #client = actionlib.SimpleActionClient('turtlebot2i/move_base', MoveBaseAction)
    client.wait_for_server()
    global goal_x, goal_y

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = goal_x
    #goal.target_pose.pose.position.y = -4
    goal.target_pose.pose.position.y = goal_y
    goal.target_pose.pose.position.z = 0
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    rospy.loginfo("ASG: X"+ str(goal_x) + " Y: " + str(goal_y))
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        state = client.get_state()
        rospy.loginfo("Goal state: ")
        rospy.loginfo(state)
        #return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('robot_0_movebase_client_py')
        get_goal_robot_0()
        rospy.loginfo("Goal Recieved from SRV: X:" + str(goal_x) + " Y:" + str(goal_y))

        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
