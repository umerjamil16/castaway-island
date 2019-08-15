#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from ca_main.srv import GoalCustomSrv, GoalCustomSrvRequest
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
pub = None

def sub0_callback(msg):
    global r0_x, r0_y
    pose_q = msg.pose.pose.position
    r0_x = pose_q.x
    r0_y = pose_q.y
    #rospy.loginfo(" X0: " + str(r0_x) +  " Y0: " + str(r0_y))
def sub1_callback(msg):
    global r1_x, r1_y
    pose_q = msg.pose.pose.position
    r1_x = pose_q.x 
    r1_y = pose_q.y 
    #rospy.loginfo(" X1: " + str(r1_x) +  " Y1: " + str(r1_y))

def clbk_laser(msg):
    global shark_speed
    laser_reading =  (msg.ranges[540])
    global dist
    #rospy.loginfo(" LASER: " + str(r0_x) +  "DIST: " + str(r0_x))
    if (laser_reading < 0.13 or laser_reading > 5.45):
        if dist > 0.30:#< 3:
            rospy.loginfo("REACHED WALL FIRST")
            shark_speed = 0
            #rospy.signal_shutdown('Quit')
#    if (laser_reading > 0.1 or laser_reading < 5.45):
 #       if dist > 5:
  #          rospy.loginfo("SHARK REACHED FIRST")
    
 
if __name__ == '__main__':
    rospy.init_node('tf_turtle1123')
    #global pub, v_x, w_z
    dist = 0
    r0_x = 0
    r0_y = 0
    r1_x = 0
    r1_y = 0

    island_radius = 3.0
    shark_speed = 0.5

    listener = tf.TransformListener()
    pub = rospy.Publisher('robot_1/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
    per_vel = rospy.Publisher('robot_0/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)

    """"
    rospy.wait_for_service('/get_robot_0_pos')
    bb8_move_srv_client = rospy.ServiceProxy('/get_robot_0_pos', GoalCustomSrv)
    bb8_move_srv_client_obj = GoalCustomSrvRequest() 
    rospy.loginfo("Doing Service Call...")
    result = bb8_move_srv_client(bb8_move_srv_client_obj)
    r0_x = result.t
    """
    #rospy.loginfo("P_x: " + str(r0_x))

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            now = rospy.Time.now() + rospy.Duration(0.0)
            listener.waitForTransform("/map", "/robot_0/base_link", now, rospy.Duration(10000))
            listener.waitForTransform('/robot_1/base_link', '/robot_0/base_link', now, rospy.Duration(10000))
            (trans,rot) = listener.lookupTransform('/map', '/robot_0/base_link', rospy.Time(0))
            (trans1,rot1) = listener.lookupTransform('/robot_1/base_link', '/robot_0/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("TF not found")            
            continue
       
        vel_msg1 = geometry_msgs.msg.Twist()
        vel_msg1.angular.z = 0#4 * math.atan2(trans1[1], trans1[0])
        vel_msg1.linear.x = shark_speed/4#-4.5 * math.sqrt(trans1[0] ** 2 + trans1[1] ** 2)
        #per_vel.publish(vel_msg1)


        pub = rospy.Publisher('robot_1/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
        d2 = math.sqrt(trans1[0] ** 2 + trans1[1] ** 2)
        d1 = math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        dist_bw_shark_ca = 3 - d1 + 0.03

        #print "r0_0: " + str(d1) + "Thres: " + str(dist_bw_shark_ca) + "SHK DIST: " + str(d2)

        #rospy.loginfo("ROB_0 x: " + str(r0_x) + " y: " + str(r0_y))
        vel_msg = geometry_msgs.msg.Twist()
        if d2 > dist_bw_shark_ca:
            rospy.loginfo("Shark moving towards castaway")
            rate = rospy.Rate(10.0)            
            if r0_x > 0:
                vel_msg.linear.x = shark_speed
                vel_msg.angular.z = -1 * shark_speed/island_radius
            elif r0_x < 0:
                vel_msg.linear.x = -1 * shark_speed
                vel_msg.angular.z = shark_speed/island_radius
            else:
                vel_msg.linear.x = shark_speed
                vel_msg.angular.z = -1 * shark_speed/island_radius
        else:
            rospy.loginfo("SHARK IN FRONT OF CASTAWAY")
            rospy.signal_shutdown('Quit')

        rate = rospy.Rate(10.0)
    #while not rospy.is_shutdown():
        pub.publish(vel_msg)
        #sub = rospy.Subscriber('/robot_0/base_scan', LaserScan, clbk_laser)
        sub0 = rospy.Subscriber ('/robot_0/odom', Odometry, sub0_callback)
        sub1 = rospy.Subscriber ('/robot_1/odom', Odometry, sub1_callback)

        rate.sleep()
#    rospy.spin()
