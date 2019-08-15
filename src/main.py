#! /usr/bin/env python
###################################
# This file find the goal and publish a relative tf GF_02
#
#
###################################
import rospy
import roslib
import math
import tf
from ca_main.srv import GoalCustomSrv, GoalCustomSrvRequest, GoalCustomSrvResponse # you import the service message python classes generated from Empty.srv.
from ca_main.srv import GoalSrv0, GoalSrv0Response # for acting as server ###
from ca_main.srv import GoalSrv1, GoalSrv1Response # for acting as server ###
#from std_srvs.srv import Empty, EmptyResponse # you import the service message python classes generated from Empty.srv.
#from geometry_msgs.msg import Twist

inner_circle_rad = 2.7
tol_inner = 0.25
tol_upper = 0.5

## vars
goal_x = 0
goal_y = 0
goal_yaw = 0

br0_x = 0
br0_y = 0
br1_x = 0
br1_y = 0

def goal_service_callback_0(request):
    rospy.loginfo("GoalService0 Callback has been called")
    response = GoalSrv0Response()
    response.x = br0_x
    response.y = br0_y
    response.t = goal_yaw
    rospy.loginfo("SRV robot_0 goal: X: " + str(response.x) + "  Y: " + str(response.y) + " Yaw: " + str(goal_yaw))
    return response

def goal_service_callback_1(request):
    rospy.loginfo("GoalService1 Callback has been called")
    response = GoalSrv0Response()
    response.x = br1_x
    response.y = br1_y
    response.t = goal_yaw
    rospy.loginfo("SRV robot_1 goal: X: " + str(response.x) + "  Y: " + str(response.y) + " Yaw: " + str(goal_yaw))
    return response

    
def get_yaw_robot_0():
    global goal_x, goal_y, goal_yaw
    rospy.wait_for_service('/get_robot_0_yaw_angle')
    goal_srv_client = rospy.ServiceProxy('/get_robot_0_yaw_angle', GoalCustomSrv)
    goal_srv_client_obj = GoalCustomSrvRequest()
    
    rospy.loginfo("Doing /get_robot_0_yaw_angle Service Call")
    result = goal_srv_client(goal_srv_client_obj)
    goal_yaw = result.t
    rospy.loginfo("Got following yaw angle: " + str(goal_yaw))
    rospy.loginfo("END of /get_robot_0_yaw_angle Service call")
    
## goal_x and goal_y are redundent

if __name__ == '__main__':
    rospy.init_node('main_program_node')
    ## Update Goal/Get Current Goal
    get_yaw_robot_0()


    ## Based on generated goal, add a tf
#    rospy.loginfo("Addign respective goal frame w.r.t /robot_0_odom")
    br0 = tf.TransformBroadcaster()
    br1 = tf.TransformBroadcaster()

    rate = rospy.Rate(10.0)
    goal_yaw = goal_yaw #+ 3.14

    br0_x = (inner_circle_rad - tol_inner) *math.cos(goal_yaw )
    br0_y = (inner_circle_rad - tol_inner) *math.sin(goal_yaw )

    br1_x = (inner_circle_rad + tol_upper) *math.cos(goal_yaw )
    br1_y = (inner_circle_rad + tol_upper) *math.sin(goal_yaw )

#    rospy.loginfo("Goal Recieved: Goal_x: " + str(goal_x) + " Goal_y: " + str(goal_y) + " Yaw: " + str(goal_yaw))
    rospy.loginfo("br0_x: " + str(br0_x) + "br0_y: " + str(br0_y))
    rospy.loginfo("br1_x: " + str(br1_x) + "br1_y: " + str(br1_y))

    robot_0_goal_service = rospy.Service('/get_robot_0_goal', GoalSrv0 , goal_service_callback_0) # create the Service called my_service with the defined callback
    robot_1_goal_service = rospy.Service('/get_robot_1_goal', GoalSrv0 , goal_service_callback_1) # create the Service called my_service with the defined callback

    while not rospy.is_shutdown():
#        rospy.loginfo("br0_x: " + str(br0_x) + "br0_y: " + str(br0_y))
#        rospy.loginfo("br1_x: " + str(br1_x) + "br1_y: " + str(br1_y))
        """
        br1.sendTransform((br1_x, br1_y, 0.0),
            tf.transformations.quaternion_from_euler(0, 0, 0),
            rospy.Time.now(),
            "map",
            "GF_01")

        br0.sendTransform((br0_x, br0_y, 0.0),
            tf.transformations.quaternion_from_euler(0, 0, 0),
            rospy.Time.now(),
            "map",
            "GF_00")
        """  
    rospy.spin()
    
#    rate.sleep()
