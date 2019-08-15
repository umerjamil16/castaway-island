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

if __name__ == '__main__':
    rospy.init_node('castaway_distance')

    listener = tf.TransformListener()
    pub = rospy.Publisher('robot_1/cmd_vel', geometry_msgs.msg.Twist)
    per_vel = rospy.Publisher('robot_0/cmd_vel', geometry_msgs.msg.Twist)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            now = rospy.Time.now() + rospy.Duration(0.0)
            listener.waitForTransform("/map", "/robot_0/base_link", now, rospy.Duration(10000))
            (trans,rot) = listener.lookupTransform('/map', '/robot_0/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("TF not found")            
            continue
        d1 = math.sqrt(trans[0] ** 2 + (trans[1] )** 2)

        print "CASTAWAY DIST FROM ISLAND CENTER: " + str(d1)

        rate.sleep()
#    rospy.spin()
