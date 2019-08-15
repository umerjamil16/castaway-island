#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from ca_main.srv import GoalSrv0, GoalSrv0Request # you import the service message python classes generated from Empty.srv.
from nav_msgs.msg import Odometry

from actionlib_msgs.msg import * 
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from nav_msgs.msg import Path
from nav_msgs.srv import GetMap, GetPlan
import math
goal_x = 0
goal_y = 0
goal_yaw = 0

#for make_plan
pose_c = 0
pose_q = 0

def distance(p0, p1):
        return math.sqrt((p0[0] - p1[0])**2 + (p0[1] - p1[1])**2)
    

def get_goal_robot_1():
    global goal_x, goal_y, goal_yaw
    rospy.wait_for_service('/get_robot_1_goal')
    goal_srv_client = rospy.ServiceProxy('/get_robot_1_goal', GoalSrv0)
    goal_srv_client_obj = GoalSrv0Request()
    
    rospy.loginfo("Doing /get_robot_1_goal Service Call...")
    result = goal_srv_client(goal_srv_client_obj)
    goal_x = result.x
    goal_y = result.y
    goal_yaw = result.t
    rospy.loginfo("END of /get_robot_1_goal Service call...")

def get_present_pose(msg):
    global pose_c, pose_q
    pose_c = msg.pose.pose.position
    pose_q = msg.pose.pose.orientation
	
def movebase_client():

    #client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client = actionlib.SimpleActionClient('robot_1/move_base', MoveBaseAction)
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

    sub = rospy.Subscriber ('/robot_0/odom', Odometry, get_present_pose)

    rospy.wait_for_service('/robot_1/move_base1/make_plan')

    rospy.loginfo("ASG: X"+ str(goal_x) + " Y: " + str(goal_y))
  
    start = PoseStamped()
    start.header.frame_id = 'map'
    # start.header.stamp = rospy.Time.now()        
    start.pose = Pose( Point(3.45577396642, 3.85964870759, 0), Quaternion(0,0,0,1))


    end = PoseStamped()
    end.header.frame_id = 'map'             
    end.pose = Pose( Point(-1.34916920128, 2.22159817531, 0), Quaternion(0,0,0,1))

    tolerance = 0.5
    getplan = rospy.ServiceProxy('/robot_1/move_base1/make_plan', GetPlan)

    plan = getplan(start, end, tolerance)
    rospy.loginfo("Pland recieved")
#    i, j, k = 
    path_arr = []
    for i in range(0, len(plan.plan.poses)):
        path_arr.append([plan.plan.poses[i].pose.position.x, plan.plan.poses[i].pose.position.x])
    #rospy.loginfo("Length: " + str((plan.plan.poses[0].pose.position.x)))

    total_dist = 0

    for i in range(1, len(path_arr)):
        dist_compute = distance(path_arr[i-1], path_arr[i])
        total_dist = total_dist + dist_compute

    rospy.loginfo("total_distance: " + str(total_dist))        
    '''
    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        state = client.get_state()
        rospy.loginfo("Goal state: ")
        rospy.loginfo(state)
        #return client.get_result()
    '''
if __name__ == '__main__':
    try:
        rospy.init_node('robot_1_movebase_client_py')
        get_goal_robot_1()
        rospy.loginfo("Goal Recieved from SRV: X:" + str(goal_x) + " Y:" + str(goal_y))

        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
