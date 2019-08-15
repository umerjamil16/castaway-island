#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from ca_main.srv import GoalSrv0, GoalSrv0Request # you import the service message python classes generated from Empty.srv.
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from nav_msgs.msg import Path
from nav_msgs.srv import GetMap, GetPlan

goal_x_0 = 0
goal_y_0 = 0
goal_yaw_0 = 0

goal_x_1 = 0
goal_y_1 = 0
goal_yaw_1 = 0

robot_1_x = 3.45577396642
robot_1_y = 3.85964870759

def get_goal_robot_0():
    global goal_x_0, goal_y_0, goal_yaw_0
    rospy.wait_for_service('/get_robot_0_goal')
    goal_srv_client = rospy.ServiceProxy('/get_robot_0_goal', GoalSrv0)
    goal_srv_client_obj = GoalSrv0Request()
    
    rospy.loginfo("Doing /get_robot_0_goal Service Call...")
    result = goal_srv_client(goal_srv_client_obj)
    goal_x_0 = result.x
    goal_y_0 = result.y
    goal_yaw_0 = result.t
    rospy.loginfo("END of /get_robot_0_goal Service call...")

def get_goal_robot_1():
    global goal_x_1, goal_y_1, goal_yaw_1
    rospy.wait_for_service('/get_robot_1_goal')
    goal_srv_client = rospy.ServiceProxy('/get_robot_1_goal', GoalSrv0)
    goal_srv_client_obj = GoalSrv0Request()
    
    rospy.loginfo("Doing /get_robot_1_goal Service Call...")
    result = goal_srv_client(goal_srv_client_obj)
    goal_x_1 = result.x
    goal_y_1 = result.y
    goal_yaw_1 = result.t
    rospy.loginfo("END of /get_robot_1_goal Service call...")

def movebase_client():

    #client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client = actionlib.SimpleActionClient('robot_0/move_base', MoveBaseAction)
    #client = actionlib.SimpleActionClient('turtlebot2i/move_base', MoveBaseAction)
    client.wait_for_server()
    global goal_x_0, goal_y_0

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = goal_x_0
    #goal.target_pose.pose.position.y = -4
    goal.target_pose.pose.position.y = goal_y_0
    goal.target_pose.pose.position.z = 0
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    rospy.loginfo("ASG: X"+ str(goal_x_0) + " Y: " + str(goal_y_0))
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        state = client.get_state()
        rospy.loginfo("Goal state: ")
        rospy.loginfo(state)
        #return client.get_result()

def update_robot_1_pos(msg):
    print "hello"
    global robot_1_x, robot_1_y
    robot_1_x = msg.pose.pose.position.x
    robot_1_y = msg.pose.pose.position.y
    rospy.loginfo("Current pos of robot_1: X: " + str(robot_1_x) + " Y: " + str(robot_1_y))
	        
if __name__ == '__main__':
    try:
        rospy.init_node('robot_0_movebase_client_py')
        get_goal_robot_0()
        get_goal_robot_1()
        rospy.loginfo("Goal Recieved from SRV: X:" + str(goal_x_0) + " Y:" + str(goal_y_0))
        ####################################################################################################
        ##Calculating /robot_1 path distance to the generated goal of robot_0
        #sub = rospy.Subscriber ('/robot_1/odom', Odometry, update_robot_1_pos)
        client = actionlib.SimpleActionClient('robot_1/move_base', MoveBaseAction)
        #client = actionlib.SimpleActionClient('turtlebot2i/move_base', MoveBaseAction)
        client.wait_for_server()
        rospy.loginfo("Current of robot_1: X: " + str(robot_1_x) + " Y: " + str(robot_1_y))
        rospy.loginfo("Goal of robot_1: X: " + str(goal_x_1) + " Y: " + str(goal_y_1))
        rospy.loginfo("Goal of robot_0: X: " + str(goal_x_0) + " Y: " + str(goal_y_0))

        rospy.wait_for_service('/robot_1/move_base1/make_plan')
      
        start = PoseStamped()
        start.header.frame_id = 'map'
        # start.header.stamp = rospy.Time.now()        
        start.pose = Pose( Point(robot_1_x, robot_1_y, 0), Quaternion(0,0,0,1))
    
    
        end = PoseStamped()
        end.header.frame_id = 'map'             
        end.pose = Pose( Point(goal_x_1, goal_y_1, 0), Quaternion(0,0,0,1))
    
        tolerance = 0.5
        getplan = rospy.ServiceProxy('/robot_1/move_base1/make_plan', GetPlan)
    
        plan = getplan(start, end, tolerance)
        rospy.loginfo("Pland recieved")
    #    i, j, k = 
        path_arr = []
        for i in range(0, len(plan.plan.poses)):
            print "here: " + str(path_arr[0])
            path_arr.append([plan.plan.poses[i].pose.position.x, plan.plan.poses[i].pose.position.x])
        #rospy.loginfo("Length: " + str((plan.plan.poses[0].pose.position.x)))
    
        total_dist = 0
    
        for i in range(1, len(path_arr)):
            dist_compute = distance(path_arr[i-1], path_arr[i])
            total_dist = total_dist + dist_compute
    
        rospy.loginfo("total_distance: " + str(total_dist)) 
        rospy.loginfo("lenght_of_array: " + str(len(path_arr))) 
        ###################################################################################################

        result = 0#movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")

        rospy.spin() #added for robot_1_pos_cal
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
