#! /usr/bin/env python
import rospy
from ca_main.srv import GoalCustomSrv, GoalCustomSrvRequest # you import the service message python classes generated from Empty.srv.
#from std_srvs.srv import Empty, EmptyResponse # you import the service message python classes generated from Empty.srv.
#from geometry_msgs.msg import Twist

rospy.init_node('goal_client_srv')
rospy.wait_for_service('/get_robot_0_pos')
bb8_move_srv_client = rospy.ServiceProxy('/get_robot_0_pos', GoalCustomSrv)
bb8_move_srv_client_obj = GoalCustomSrvRequest()

rospy.loginfo("Doing Service Call...")
result = bb8_move_srv_client(bb8_move_srv_client_obj)
rospy.loginfo("Goal Recieved: Goal_x: " + str(result.t))
rospy.loginfo("END of Service call...")

